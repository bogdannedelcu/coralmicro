# _t_models_bench.py — on-board EdgeTPU comparative bench across the
# 6 candidate models in /models/.
#
# Per agent.md §5.1.2: SELF-CONTAINED — only imports `sentai`, inlines
# its own _session_dir, no diag/* dependencies.
# Per agent.md §5.1.1: bounded loops + repl_kick() defense-in-depth.
# Per agent.md §2.9: cross-test contamination risk; mitigations:
#   - smallest-first ordering (a wedge from a big model leaves smaller
#     numbers already collected);
#   - try/except around every TPU call so one bad model does not
#     poison the run;
#   - CSV written incrementally INSIDE the loop, so partial results
#     survive even if the next model wedges the TPU;
#   - on a load/invoke failure we record an "error" row and proceed.
import sentai

def _session_dir(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try:
        sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception:
        sid = 1
    try: sentai.fs.write(counter, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d

sentai.verbose(1)
build_id = sentai.version().split("build")[1].split()[0]
sess = _session_dir("models_bench")
print("session dir:", sess, "build:", build_id)

# Smallest-first ordering — minimises blast-radius if a later/larger
# model wedges the TPU.  yolo1_1up_alt is largest + least proven, last.
MODELS = [
    ("p3p4_MSBlock",   "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite", 640, 480),
    ("p3p4_C2f",       "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite",     640, 480),
    ("p3p4_GELAN",     "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite",   640, 480),
    ("p2p4_5ep_BASE",  "/models/iarna_p2p4_5ep_export_640x480_uint8.tflite",                640, 480),
    ("yolo1_inloc_P5", "/models/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite", 512, 512),
    ("yolo1_1up_alt",  "/models/yolo_1_class_512_1_upsample.tflite",                       512, 512),
]

WARMUP = 3
N      = 30
csv_path = "%s/results.csv" % sess

# Init CSV header up-front so a partial run still produces a valid file.
sentai.fs.write(csv_path,
    "label,path,w,h,median_ms,p99_ms,mean_ms,fps,p_bytes_per_invoke,i_bytes_per_invoke,in_bytes_per_invoke,fails,note\r\n")

last_w = last_h = -1

def append_row(s):
    try: sentai.fs.append(csv_path, s)
    except Exception as e: print("  csv append fail:", e)

for label, path, w, h in MODELS:
    sentai.diag.repl_kick()
    print("\n--- %s ---" % label)
    print("  path:", path, "input:", w, "x", h)
    note = ""
    # Stop pipeline if running.
    try:
        if sentai.pipeline.running(): sentai.pipeline.stop()
    except Exception: pass
    # Camera resolution (best-effort; 512×512 may fall back to default).
    if (w, h) != (last_w, last_h):
        try: sentai.camera.set_resolution(w, h)
        except Exception as e: note = "set_res:%s " % e
        try:
            if sentai.camera.frame_count() == 0: sentai.camera.init(1)
        except Exception as e: note += "cam_init:%s " % e
        last_w, last_h = w, h
    # Load model.
    try:
        rc = sentai.tpu.load(path)
        print("  tpu.load rc=%s" % rc)
        if rc != 0 and rc is not None:
            note += "load_rc=%s " % rc
    except Exception as e:
        print("  tpu.load FAILED:", e)
        append_row('%s,%s,%d,%d,,,,,,,,,LOAD_FAIL:%s\r\n' % (label, path, w, h, str(e).replace(",", " ")))
        continue
    # Camera → input tensor (best-effort).
    try: sentai.camera.to_tensor()
    except Exception as e: note += "to_tensor:%s " % e
    sentai.rtos.sleep_ms(50)
    # Warmup.
    warm_fail = 0
    for i in range(WARMUP):
        t0 = sentai.rtos.ticks_ms()
        try: rc = sentai.tpu.invoke()
        except Exception as e:
            rc = -1; warm_fail += 1
            print("  warmup #%d invoke EXC: %s" % (i, e))
        t1 = sentai.rtos.ticks_ms()
        print("  warmup #%d: %d ms rc=%s" % (i, t1 - t0, rc))
    if warm_fail >= WARMUP:
        # All warmups failed → don't try the timed loop.
        append_row('%s,%s,%d,%d,,,,,,,,,WARMUP_ALL_FAIL %s\r\n' % (label, path, w, h, note))
        continue
    # Reset USB call counters AFTER warmup so steady-state is measured.
    try:
        sentai.diag.tpu_call_stats(1)
        s_pre = sentai.diag.tpu_call_stats()
    except Exception:
        s_pre = {'p_calls': 0, 'i_calls': 0, 'in_calls': 0,
                 'p_bytes': 0, 'i_bytes': 0, 'in_bytes': 0}
    # Bench loop.
    samples_ms = []
    invoke_fail = 0
    for i in range(N):
        t0 = sentai.rtos.ticks_ms()
        try: rc = sentai.tpu.invoke()
        except Exception:
            rc = -1
        t1 = sentai.rtos.ticks_ms()
        # tpu.invoke() returns the per-invoke elapsed ms on success
        # (positive small int).  rc < 0 is an actual failure; rc == 0
        # is ambiguous on this firmware so accept it.
        if isinstance(rc, int) and rc < 0:
            invoke_fail += 1
        samples_ms.append(t1 - t0)
        if (i & 0x07) == 0: sentai.diag.repl_kick()
        if invoke_fail > 5:
            note += "ABORT_after_5_invoke_fails "
            break
    try: s_post = sentai.diag.tpu_call_stats()
    except Exception: s_post = s_pre
    samples_ms.sort()
    n_real = len(samples_ms)
    if n_real == 0:
        append_row('%s,%s,%d,%d,,,,,,,,%d,NO_SAMPLES %s\r\n' % (label, path, w, h, invoke_fail, note))
        continue
    median = samples_ms[n_real // 2]
    p99    = samples_ms[min(n_real - 1, int(n_real * 0.99))]
    mn     = sum(samples_ms) // n_real
    fps    = (1000 // median) if median > 0 else 0
    p_per   = (s_post['p_bytes']  - s_pre['p_bytes'])  // n_real
    i_per   = (s_post['i_bytes']  - s_pre['i_bytes'])  // n_real
    in_per  = (s_post['in_bytes'] - s_pre['in_bytes']) // n_real
    print("  N=%d med=%dms p99=%dms mean=%dms fail=%d fps≈%d" %
          (n_real, median, p99, mn, invoke_fail, fps))
    print("  per-invoke USB bytes: p=%d i=%d in=%d total=%d" %
          (p_per, i_per, in_per, p_per + i_per + in_per))
    append_row('%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\r\n' %
               (label, path, w, h, median, p99, mn, fps,
                p_per, i_per, in_per, invoke_fail, note.strip()))
    # Brief settle between models — gives USB stack a moment to drain.
    sentai.rtos.sleep_ms(200)
    sentai.diag.repl_kick()

print("\nCSV saved: %s" % csv_path)
print("=== done ===")

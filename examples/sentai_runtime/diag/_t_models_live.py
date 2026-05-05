# _t_models_live.py — pure-TPU bench, no camera, no pipeline.
#
# Tests live model switching: loads each model in sequence on the same
# boot, runs warmup + 30 timed invokes, captures call-stats per model.
# The input tensor is left at whatever state tpu.load() puts it in
# (typically zero-initialised) — this is a TIMING test, not an
# accuracy test.
#
# Per agent.md §2.9: cross-test contamination is a known risk.  This
# driver INTENTIONALLY exercises that path to find which model
# wedges and whether tpu.load() can recover.  CSV is written
# incrementally so partial results survive a wedge.
#
# Per agent.md §5.1.2: self-contained, no diag/* imports.
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
sess = _session_dir("models_live")
print("session dir:", sess, "build:", build_id)

# Smallest first → biggest blast-radius last.
# NOTE: iarna_p2p4_5ep_export_640x480_uint8 is INTENTIONALLY EXCLUDED.
# Per-boot orchestrator bench (orch_models_bench.csv) showed it: median
# 200 ms, 6 invoke fails, in_bytes_per_invoke=0 → TPU rejects it before
# the input DMA (likely a libedgetpu PrepareInputs failure or unsupported
# op for Coral USB).  Once this model is touched, the TPU is wedged for
# the rest of the boot — no public reset path recovers it.  Excluding it
# here keeps the live-switch test producing useful data for the 5
# working models.
MODELS = [
    "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite",
    "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite",
    "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite",
    "/models/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite",
    "/models/yolo_1_class_512_1_upsample.tflite",
]

WARMUP = 3
N      = 30
csv_path = "%s/results.csv" % sess
sentai.fs.write(csv_path,
    "label,path,load_ms,warm0_ms,median_ms,p99_ms,mean_ms,fps,p_bytes_per_invoke,i_bytes_per_invoke,in_bytes_per_invoke,fails,note\r\n")

# Make sure pipeline is OFF (per user: bench TPU only).
try:
    if sentai.pipeline.running(): sentai.pipeline.stop()
    print("pipeline stopped/idle")
except Exception as e:
    print("pipeline check exc:", e)

for path in MODELS:
    sentai.diag.repl_kick()
    label = path.split("/")[-1].replace(".tflite", "")
    print("\n=== %s ===" % label)
    note = ""
    # Load.
    t0 = sentai.rtos.ticks_ms()
    try:
        rc_load = sentai.tpu.load(path)
        load_ms = sentai.rtos.ticks_ms() - t0
        print("  load rc=%s in %dms" % (rc_load, load_ms))
    except Exception as e:
        print("  load EXC:", e)
        sentai.fs.append(csv_path,
            '%s,%s,,,,,,,,,,,LOAD_EXC %s\r\n' % (label, path, str(e).replace(",", " ")))
        continue
    # Warmup (no camera, input is whatever tpu.load left it).
    # warm0 abort: bail only if first invoke returns rc<0 (libedgetpu
    # error) or elapsed > 3000 ms (true hang).  A slow-but-positive
    # warm0 is normal — that's the parameter-caching upload (~MB).
    warm0 = -1
    warm_fail = 0
    bail = False
    for i in range(WARMUP):
        t0 = sentai.rtos.ticks_ms()
        try: rc = sentai.tpu.invoke()
        except Exception as e:
            rc = -1; warm_fail += 1
            print("  warmup #%d EXC: %s" % (i, e))
        t1 = sentai.rtos.ticks_ms()
        if i == 0:
            warm0 = t1 - t0
            if (isinstance(rc, int) and rc < 0) or warm0 > 3000:
                note = "rc=%s ms=%d" % (rc, warm0)
                print("  WARM0 BAIL — rc=%s ms=%d, skipping bench" % (rc, warm0))
                bail = True
                break
        print("  warmup #%d: %dms rc=%s" % (i, t1 - t0, rc))
        sentai.diag.repl_kick()
    if bail:
        sentai.fs.append(csv_path,
            '%s,%s,%d,%d,,,,,,,,,WARM0_BAIL %s\r\n' %
            (label, path, load_ms, warm0, note))
        continue
    if warm_fail >= WARMUP:
        sentai.fs.append(csv_path,
            '%s,%s,%d,%d,,,,,,,,,WARMUP_ALL_FAIL %s\r\n' %
            (label, path, load_ms, warm0, note))
        print("  ALL WARMUPS FAILED — skipping bench")
        continue
    # Counter-reset before timed loop.
    try:
        sentai.diag.tpu_call_stats(1)
        s_pre = sentai.diag.tpu_call_stats()
    except Exception:
        s_pre = {'p_calls':0,'i_calls':0,'in_calls':0,
                 'p_bytes':0,'i_bytes':0,'in_bytes':0}
    samples_ms = []
    invoke_fail = 0
    for i in range(N):
        t0 = sentai.rtos.ticks_ms()
        try: rc = sentai.tpu.invoke()
        except Exception:
            rc = -1
        t1 = sentai.rtos.ticks_ms()
        if isinstance(rc, int) and rc < 0:
            invoke_fail += 1
        samples_ms.append(t1 - t0)
        if (i & 0x07) == 0: sentai.diag.repl_kick()
        if invoke_fail > 5:
            note += "ABORT_after_5_fails "
            break
    try: s_post = sentai.diag.tpu_call_stats()
    except Exception: s_post = s_pre
    samples_ms.sort()
    n_real = len(samples_ms)
    if n_real == 0:
        sentai.fs.append(csv_path,
            '%s,%s,%d,%d,,,,,,,,%d,NO_SAMPLES %s\r\n' %
            (label, path, load_ms, warm0, invoke_fail, note))
        continue
    median = samples_ms[n_real // 2]
    p99    = samples_ms[min(n_real - 1, int(n_real * 0.99))]
    mn     = sum(samples_ms) // n_real
    fps    = (1000 // median) if median > 0 else 0
    p_per   = (s_post['p_bytes']  - s_pre['p_bytes'])  // n_real
    i_per   = (s_post['i_bytes']  - s_pre['i_bytes'])  // n_real
    in_per  = (s_post['in_bytes'] - s_pre['in_bytes']) // n_real
    print("  N=%d med=%dms p99=%dms fail=%d fps≈%d" %
          (n_real, median, p99, invoke_fail, fps))
    print("  USB/inv: p=%d i=%d in=%d total=%d" %
          (p_per, i_per, in_per, p_per+i_per+in_per))
    sentai.fs.append(csv_path,
        '%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\r\n' %
        (label, path, load_ms, warm0, median, p99, mn, fps,
         p_per, i_per, in_per, invoke_fail, note.strip()))
    sentai.rtos.sleep_ms(200)
    sentai.diag.repl_kick()

print("\nCSV: %s" % csv_path)
print("=== done ===")

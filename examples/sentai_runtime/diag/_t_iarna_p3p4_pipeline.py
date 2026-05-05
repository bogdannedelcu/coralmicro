# _t_iarna_p3p4_pipeline.py — pipeline FPS sweep for iarna p3p4
# candidates at VGA45.
#
# Self-contained per agent.md §5.1.2 — and per user feedback (memory:
# feedback_no_complex_serial_orchestration.md): the entire experiment
# (3 models × probed ratios) lives INSIDE this on-board driver.
# The host only:
#   1. pushes this file via diag/_host_upload_repl.py,
#   2. exec()s it ONCE over REPL (`exec(sentai.fs.read_str("/lib/diag/_t_iarna_p3p4_pipeline.py"))`),
#   3. waits for `=== done ===`,
#   4. pulls /diags/sNNN_iarna_p3p4_pipeline/results.csv afterwards.
#
# Cross-model TPU contamination is mitigated by `sentai.sys.reset()`
# between models.  The driver remembers progress in
# /diags/.iarna_p3p4_state and resumes after the warm reset, appending
# to the same CSV across reboots until all 3 models are benched.
import sentai
sentai.verbose(1)

MODELS = [
    ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite"),
    ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite"),
    ("GELAN",   "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite"),
]
FPS          = 45
NB           = 100
PROBE_FRAMES = 50
TOL_PCT      = 10
STATE_PATH   = "/diags/.iarna_p3p4_state"


def _sd(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    c = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(c).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(c, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


def _calibrate_at(model_path, ra, rb):
    sentai.camera.ratio(ra, rb)
    sentai.camera.switch_drain(1)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    res = sentai.pipeline.calibrate(model_path, NB, 5000)
    sentai.rtos.sleep_ms(50)
    return res


def _bench_one(label, model_path, csv_path):
    print("\n=== %s @ VGA%d ===" % (label, FPS))
    rc = sentai.camera.init(1, FPS)
    print("init(1, %d) =" % FPS, rc)
    if rc == -11:
        print("mismatch -> reset")
        sentai.rtos.sleep_ms(200)
        sentai.sys.reset()
    if rc != 0:
        sentai.fs.append(csv_path,
            "%d,%s,INIT_FAIL,0,0,0,0,0,0,0,0,0,0\r\n" % (FPS, label))
        return
    sentai.rtos.sleep_ms(800)
    sentai.camera.select(0); sentai.rtos.sleep_ms(300)

    print("--- probe ratios @ fps=%d ---" % FPS)
    probe = sentai.pipeline.probe_ratios(model_path, PROBE_FRAMES, TOL_PCT)
    for p in probe:
        a, b = p["ratio"]
        flag = "OK" if p["ok"] else "SKIP"
        print("  %d:%d %4s cam0:cam1=%d:%d (%d frames) reason=%s" %
              (a, b, flag, p["cam0"], p["cam1"], p["frames"], p["reason"]))

    rows = []
    print("--- pipeline 1cam ---")
    rows.append(("1cam", 0, 0, _calibrate_at(model_path, 0, 0)))
    for p in probe:
        if not p["ok"]: continue
        a, b = p["ratio"]
        lab = "%d:%d" % (a, b)
        print("--- pipeline %s ---" % lab)
        rows.append((lab, a, b, _calibrate_at(model_path, a, b)))

    sentai.pipeline.stop()
    sentai.camera.ratio(0, 0)

    print("--- summary fps=%d label=%s ---" % (FPS, label))
    print("  mode | frames | cam0:cam1 | invoke ms (avg/min/max) | total ms (avg) | pipeline fps")
    for lab, _a, _b, r in rows:
        n   = r.get("frames", 0) or 1
        ia  = r.get("invoke_ms_sum", 0) // n
        ta  = r.get("total_ms_sum", 0) // n
        f100 = r.get("fps_x100", 0)
        print("  %-4s | %6d | %3d:%-3d   | %3d / %3d / %3d         | %3d            | %d.%02d" %
              (lab, n, r.get("cam0", 0), r.get("cam1", 0),
               ia, r.get("invoke_ms_min", 0), r.get("invoke_ms_max", 0),
               ta, f100 // 100, f100 % 100))

    for lab, a, b, r in rows:
        n = r.get("frames", 0) or 1
        sentai.fs.append(csv_path,
            "%d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n" %
            (FPS, label, lab, a, b, n,
             r.get("cam0", 0), r.get("cam1", 0),
             r.get("invoke_ms_sum", 0) // n,
             r.get("invoke_ms_min", 0),
             r.get("invoke_ms_max", 0),
             r.get("total_ms_sum", 0) // n,
             r.get("fps_x100", 0)))


# Resume across reboots: state file holds "session_dir|next_idx".
sess = None
idx  = 0
try:
    raw = sentai.fs.read_str(STATE_PATH).strip()
    parts = raw.split("|")
    if len(parts) == 2:
        sess = parts[0]
        idx  = int(parts[1])
        # Verify session dir still exists.
        if not sentai.fs.exists(sess):
            sess = None; idx = 0
except Exception:
    pass

if sess is None:
    sess = _sd("iarna_p3p4_pipeline")
    csv_path = sess + "/results.csv"
    sentai.fs.write(csv_path,
        "fps,label,mode,a,b,frames,cam0,cam1,invoke_avg,invoke_min,invoke_max,total_avg,fps_x100\r\n")
    print("=== fresh session:", sess, "===")
else:
    csv_path = sess + "/results.csv"
    print("=== resuming session:", sess, "@ idx", idx, "===")

while idx < len(MODELS):
    label, path = MODELS[idx]
    _bench_one(label, path, csv_path)
    idx += 1
    if idx < len(MODELS):
        # Persist progress, then warm-reset to clear TPU state.
        sentai.fs.write(STATE_PATH, "%s|%d" % (sess, idx))
        print("--- %d/%d done; sys.reset() to clear TPU ---" % (idx, len(MODELS)))
        sentai.rtos.sleep_ms(500)
        sentai.sys.reset()

# All done — clean up state file.
try: sentai.fs.remove(STATE_PATH)
except Exception: pass
print("\nCSV: %s" % csv_path)
print("=== done ===")

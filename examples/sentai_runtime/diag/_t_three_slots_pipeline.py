# _t_three_slots_pipeline.py — pipeline FPS for 3 p3p4 candidates
# on the multi-slot firmware (build #1100+).
#
# Goal: confirm the multi-slot path matches the single-slot baseline
# (legacy slot 0 only) at VGA45.  Same probed-ratios + 1cam sweep as
# `_t_iarna_p3p4_pipeline.py`, but exercising the new
# `sentai.pipeline.set_slot_for_cam` plumbing.
#
# Test matrix (per model):
#   - load model into slot 0 only; cam0/cam1 both → slot 0 (legacy)
#   - run pipeline.calibrate at 1cam + every probed alt ratio
# This is 3 models in sequence, persisting progress across sys.reset()
# in /diags/.three_slot_pipe_state so a wedged cycle doesn't lose
# earlier rows.
#
# Self-contained per agent.md §5.1.2 + §5.1.5.
import sentai
sentai.verbose(1)

MODELS = [
    ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite"),
    ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite"),
    ("GELAN",   "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite"),
]
FPS = 45
NB  = 100
PROBE_FRAMES = 50
TOL_PCT      = 10
STATE_PATH   = "/diags/.three_slot_pipe_state"


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


def _calibrate_at(path, ra, rb):
    sentai.camera.ratio(ra, rb)
    sentai.camera.switch_drain(1)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    res = sentai.pipeline.calibrate(path, NB, 5000)
    sentai.rtos.sleep_ms(50)
    return res


# Resume across reboots.
sess = None
idx  = 0
try:
    raw = sentai.fs.read_str(STATE_PATH).strip()
    parts = raw.split("|")
    if len(parts) == 2:
        sess = parts[0]
        idx  = int(parts[1])
        if not sentai.fs.exists(sess):
            sess = None; idx = 0
except Exception:
    pass

if sess is None:
    sess = _sd("three_slot_pipe")
    csv_path = sess + "/results.csv"
    sentai.fs.write(csv_path,
        "fps,model,mode,a,b,frames,cam0,cam1,invoke_avg,invoke_min,invoke_max,total_avg,fps_x100\r\n")
    print("=== fresh session:", sess, "===")
else:
    csv_path = sess + "/results.csv"
    print("=== resuming session:", sess, "@ idx", idx, "===")


def _bench(label, path, csv_path):
    print("\n=== %s @ VGA%d (slot 0 only) ===" % (label, FPS))
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

    # Default routing: cam0 + cam1 both -> slot 0 (legacy).
    sentai.pipeline.set_slot_for_cam(0, 0)
    sentai.pipeline.set_slot_for_cam(1, 0)

    print("--- probe ratios @ fps=%d ---" % FPS)
    probe = sentai.pipeline.probe_ratios(path, PROBE_FRAMES, TOL_PCT)
    for p in probe:
        a, b = p["ratio"]
        flag = "OK" if p["ok"] else "SKIP"
        print("  %d:%d %4s cam0:cam1=%d:%d (%d frames) reason=%s" %
              (a, b, flag, p["cam0"], p["cam1"], p["frames"], p["reason"]))

    rows = []
    print("--- pipeline 1cam ---")
    rows.append(("1cam", 0, 0, _calibrate_at(path, 0, 0)))
    for p in probe:
        if not p["ok"]: continue
        a, b = p["ratio"]
        lab = "%d:%d" % (a, b)
        print("--- pipeline %s ---" % lab)
        rows.append((lab, a, b, _calibrate_at(path, a, b)))

    sentai.pipeline.stop()
    sentai.camera.ratio(0, 0)

    print("--- summary fps=%d label=%s ---" % (FPS, label))
    print("  mode | frames | cam0:cam1 | invoke ms (avg/min/max) | total ms | pipeline fps")
    for lab, _a, _b, r in rows:
        n = r.get("frames", 0) or 1
        ia = r.get("invoke_ms_sum", 0) // n
        ta = r.get("total_ms_sum", 0) // n
        f100 = r.get("fps_x100", 0)
        print("  %-4s | %6d | %3d:%-3d   | %3d / %3d / %3d         | %3d      | %d.%02d" %
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


while idx < len(MODELS):
    label, path = MODELS[idx]
    _bench(label, path, csv_path)
    idx += 1
    if idx < len(MODELS):
        sentai.fs.write(STATE_PATH, "%s|%d" % (sess, idx))
        print("--- %d/%d done; sys.reset() to clear TPU between models ---" %
              (idx, len(MODELS)))
        sentai.rtos.sleep_ms(500)
        sentai.sys.reset()

try: sentai.fs.remove(STATE_PATH)
except Exception: pass
print("\nCSV: %s" % csv_path)
print("=== done ===")

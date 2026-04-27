# _t_vga_bench.py — timing + cam_id correctness benchmark at VGA{30,45,60}.
# FPS picked by setting _target_fps in REPL before exec().
#
# For each of {single cam0, single cam1, alt 1:1, alt 2:1, alt 3:1}:
#   - capture N=100 frames via single-grab 5-row peek5_b40
#   - measure wall-clock per frame
#   - classify each frame and confirm tag == content
#   - report fps, mean/min/max us, correct/scrambled/wrong-tag counts
#
# Patterns: cam0=BARS, cam1=HBAND.  Set ONCE at init.
# In-RAM bookkeeping; single LFS write at end.
import sentai
sentai.verbose(1)


def _session_dir(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    counter = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(counter).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(counter, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


def _classify(b40):
    if b40 >= 0xA0: return "BARS"
    if b40 <= 0x80: return "HBAND"
    return "AMBIG"


def _classify_5b(s5):
    classes = tuple(_classify(b) for b in s5)
    if all(c == "BARS" for c in classes):  return "BARS"
    if all(c == "HBAND" for c in classes): return "HBAND"
    if all(c == "AMBIG" for c in classes): return "AMBIG"
    return "SCRAMBLED"


def _bench(label, N):
    """Run N captures in current camera config; return stats dict."""
    correct = 0
    scrambled = 0
    wrong_tag = 0
    cam0 = 0
    cam1 = 0
    # ms per frame: sample wall clock per capture (ticks_ms = 1 ms
    # resolution — coarse but fine for VGA45's ~22 ms/frame).
    # Warm-up grab thrown away so the first interval reflects the
    # steady state, not the mode-change settle.
    sentai.camera.peek5_b40()
    durs = []
    last = sentai.rtos.ticks_ms()
    for i in range(N):
        r = sentai.camera.peek5_b40()
        now = sentai.rtos.ticks_ms()
        durs.append(now - last)
        last = now
        cam = r[0]
        s5 = r[1:6]
        cls = _classify_5b(s5)
        if cls == "SCRAMBLED":
            scrambled += 1
        elif cls == "BARS" and cam == 0:
            correct += 1
        elif cls == "HBAND" and cam == 1:
            correct += 1
        else:
            wrong_tag += 1
        if cam == 0: cam0 += 1
        elif cam == 1: cam1 += 1
        if (i % 20) == 0:
            sentai.diag.repl_kick()
    durs.sort()
    n = len(durs)
    p50 = durs[n // 2]
    p99 = durs[(n * 99) // 100]
    avg = sum(durs) // n
    fps = (1000 + avg // 2) // avg if avg > 0 else 0
    return {
        "label": label, "N": N,
        "correct": correct, "scrambled": scrambled, "wrong_tag": wrong_tag,
        "cam0": cam0, "cam1": cam1,
        "ms_min": durs[0], "ms_p50": p50, "ms_p99": p99, "ms_max": durs[-1],
        "ms_avg": avg, "fps": fps,
    }


print("=== boot ===")
print("version=", sentai.version())

# `DEMO_CAMERA_FRAME_RATE` is a compile-time constant in this build
# (libs/camera/camera_support.h:107).  The driver runs whichever
# fps the firmware was built with; the host orchestrator records
# that fps separately.
sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)

# Patterns at init.
sentai.camera.test_pattern(0, 1)
sentai.camera.test_pattern(1, 2)
sentai.camera.select(0); sentai.rtos.sleep_ms(500)
sentai.camera.select(1); sentai.rtos.sleep_ms(500)
sentai.camera.select(0); sentai.rtos.sleep_ms(300)

NB = 100
results = []

# 1. Single cam0
print("--- single cam0 ---")
sentai.camera.ratio(0, 0)
sentai.camera.select(0); sentai.rtos.sleep_ms(300)
results.append(_bench("single_cam0", NB))

# 2. Single cam1
print("--- single cam1 ---")
sentai.camera.ratio(0, 0)
sentai.camera.select(1); sentai.rtos.sleep_ms(300)
results.append(_bench("single_cam1", NB))

# 3. Alt 1:1
print("--- alt 1:1 ---")
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0); sentai.rtos.sleep_ms(300)
results.append(_bench("alt_1_1", NB))

# 4. Alt 2:1
print("--- alt 2:1 ---")
sentai.camera.ratio(2, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0); sentai.rtos.sleep_ms(300)
results.append(_bench("alt_2_1", NB))

# 5. Alt 3:1
print("--- alt 3:1 ---")
sentai.camera.ratio(3, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0); sentai.rtos.sleep_ms(300)
results.append(_bench("alt_3_1", NB))

# Restore
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.test_pattern(0, 0)
sentai.camera.test_pattern(1, 0)

print("--- summary (VGA45, %d frames per mode) ---" % NB)
print("  %-13s | correct/N | scram | wrong | cam0:cam1 | ms avg/p50/p99 | fps" % "mode")
for r in results:
    print("  %-13s | %3d/%-3d   | %3d   | %3d   |  %2d:%-2d    | %3d/%3d/%3d    | %d" %
          (r["label"], r["correct"], r["N"], r["scrambled"], r["wrong_tag"],
           r["cam0"], r["cam1"], r["ms_avg"], r["ms_p50"], r["ms_p99"], r["fps"]))

# Single LFS write — CSV log
sd = _session_dir("vga_bench")
lines = ["mode,N,correct,scrambled,wrong_tag,cam0,cam1,ms_min,ms_avg,ms_p50,ms_p99,ms_max,fps\n"]
for r in results:
    lines.append("%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" %
                 (r["label"], r["N"], r["correct"], r["scrambled"], r["wrong_tag"],
                  r["cam0"], r["cam1"], r["ms_min"], r["ms_avg"], r["ms_p50"],
                  r["ms_p99"], r["ms_max"], r["fps"]))
sentai.fs.write(sd + "/bench.csv", "".join(lines))
print("log saved:", sd + "/bench.csv")
print("=== done ===")

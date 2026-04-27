# _t_vga_quick.py — minimal cam_id timing test, only single_cam0 +
# alt 3:1.  Designed to fit comfortably under the alt-mode wedge
# threshold so the host can collect a clean number per fps.
# Whichever fps the firmware was built with, this driver doesn't try
# to change it (no set_hw on this firmware) — host orchestrator
# records the fps separately.
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
    correct = scrambled = wrong = cam0 = cam1 = 0
    sentai.camera.peek5_b40()  # warm
    durs = []
    last = sentai.rtos.ticks_ms()
    for i in range(N):
        r = sentai.camera.peek5_b40()
        now = sentai.rtos.ticks_ms()
        durs.append(now - last); last = now
        cam = r[0]; s5 = r[1:6]; cls = _classify_5b(s5)
        if cls == "SCRAMBLED": scrambled += 1
        elif cls == "BARS" and cam == 0: correct += 1
        elif cls == "HBAND" and cam == 1: correct += 1
        else: wrong += 1
        if cam == 0: cam0 += 1
        elif cam == 1: cam1 += 1
        if (i % 20) == 0: sentai.diag.repl_kick()
    durs.sort()
    avg = sum(durs) // len(durs)
    return {"label": label, "N": N, "correct": correct, "scrambled": scrambled,
            "wrong": wrong, "cam0": cam0, "cam1": cam1,
            "ms_min": durs[0], "ms_avg": avg, "ms_p50": durs[len(durs) // 2],
            "ms_p99": durs[(len(durs) * 99) // 100], "ms_max": durs[-1],
            "fps": (1000 + avg // 2) // avg if avg else 0}


print("=== boot ===")
print("version=", sentai.version())
sentai.camera.init(1)
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)
sentai.camera.test_pattern(0, 1)
sentai.camera.test_pattern(1, 2)
sentai.camera.select(0); sentai.rtos.sleep_ms(500)
sentai.camera.select(1); sentai.rtos.sleep_ms(500)
sentai.camera.select(0); sentai.rtos.sleep_ms(300)

NB = 100
results = []

print("--- single cam0 ---")
sentai.camera.ratio(0, 0); sentai.camera.select(0); sentai.rtos.sleep_ms(300)
results.append(_bench("single_cam0", NB))

print("--- alt 3:1 ---")
sentai.camera.ratio(3, 1); sentai.camera.switch_drain(1)
sentai.camera.select(0); sentai.rtos.sleep_ms(300)
results.append(_bench("alt_3_1", NB))

sentai.camera.ratio(0, 0); sentai.camera.select(0)
sentai.camera.test_pattern(0, 0); sentai.camera.test_pattern(1, 0)

print("--- summary ---")
for r in results:
    print("  %-13s  ok=%d/%d  scr=%d  wrng=%d  cam0:cam1=%d:%d  ms %d/%d/%d  fps %d" %
          (r["label"], r["correct"], r["N"], r["scrambled"], r["wrong"],
           r["cam0"], r["cam1"], r["ms_avg"], r["ms_p50"], r["ms_p99"], r["fps"]))

sd = _session_dir("vga_quick")
lines = ["mode,N,correct,scrambled,wrong,cam0,cam1,ms_min,ms_avg,ms_p50,ms_p99,ms_max,fps\n"]
for r in results:
    lines.append("%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" %
                 (r["label"], r["N"], r["correct"], r["scrambled"], r["wrong"],
                  r["cam0"], r["cam1"], r["ms_min"], r["ms_avg"], r["ms_p50"],
                  r["ms_p99"], r["ms_max"], r["fps"]))
sentai.fs.write(sd + "/vga_quick.csv", "".join(lines))
print("log saved:", sd + "/vga_quick.csv")
print("=== done ===")

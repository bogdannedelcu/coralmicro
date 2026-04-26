# _t_dirty_sweep.py — sweep N=1..6 for sentai.camera.dirty_skip_n
# at ratio 3:1 with synthetic patterns.  Reports per-N accuracy.
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

# Engage alt 3:1.
sentai.camera.ratio(3, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(300)

NB = 100
results = []  # [(N, correct, scrambled, wrong_tag, cam0_count, cam1_count)]

for N in (1, 2, 3, 4, 5, 6):
    prev = sentai.camera.dirty_skip_n(N)
    sentai.rtos.sleep_ms(200)  # let pipeline settle
    sentai.diag.repl_kick()
    correct = 0
    scrambled = 0
    wrong_tag = 0
    cam0 = 0
    cam1 = 0
    for i in range(NB):
        r = sentai.camera.peek5_b40()
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
    results.append((N, correct, scrambled, wrong_tag, cam0, cam1))
    print("  N=%d  correct=%d  scrambled=%d  wrong_tag=%d  cam0=%d cam1=%d" %
          (N, correct, scrambled, wrong_tag, cam0, cam1))

# restore
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.test_pattern(0, 0)
sentai.camera.test_pattern(1, 0)
sentai.camera.dirty_skip_n(1)

print("--- sweep summary (alt 3:1, %d frames each) ---" % NB)
for (N, c, s, w, c0, c1) in results:
    print("  N=%d  %3d/%d  scr=%d  wrong=%d  ratio=%d:%d" %
          (N, c, NB, s, w, c0, c1))

# Save log to LFS
sd = _session_dir("dirty_sweep")
parts = ["N,correct,scrambled,wrong_tag,cam0,cam1\n"]
for (N, c, s, w, c0, c1) in results:
    parts.append("%d,%d,%d,%d,%d,%d\n" % (N, c, s, w, c0, c1))
sentai.fs.write(sd + "/sweep.csv", "".join(parts))
print("log saved:", sd + "/sweep.csv")
print("=== done ===")

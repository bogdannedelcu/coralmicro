# _t_two_model_alt.py — pure-TPU 2-model alternation, no pipeline.
#
# Hypothesis from the host pycoral 2-model experiment:
# libedgetpu's on-chip parameter cache is keyed by the model's
# parameter_caching token, not by the interpreter lifetime.  If the
# SentAI firmware destroys+recreates the tflite::Interpreter on each
# tpu.load() but the EdgeTPU device stays open, the cache should be
# preserved across loads — the SECOND load of the same model should
# NOT pay a parameter-upload spike on warm0.
#
# Method: 5 cycles of [load(A) + warm0 + 30 invokes + load(B) + warm0
# + 30 invokes].  Records load_ms, warm0_ms, median per phase.
# If warm0 stays small (~6-9 ms) across all 10 reloads of A and B,
# the cache is persistent and a future `load_slot` firmware API
# could keep both models hot at zero swap cost.
#
# Self-contained per agent.md §5.1.2 + §5.1.5.
import sentai
sentai.verbose(1)

A = ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite")
B = ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite")
CYCLES = 5
N      = 30


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


def _phase(label, path, csv_path, cycle, slot):
    print("--- cycle=%d slot=%d %s ---" % (cycle, slot, label))
    t0 = sentai.rtos.ticks_ms()
    rc_load = sentai.tpu.load(path)
    load_ms = sentai.rtos.ticks_ms() - t0
    if rc_load != 0 and rc_load is not None:
        print("  load FAIL rc=%s in %d ms" % (rc_load, load_ms))
        sentai.fs.append(csv_path,
            "%d,%d,%s,%d,,,,,LOAD_FAIL\r\n" % (cycle, slot, label, load_ms))
        return
    t0 = sentai.rtos.ticks_ms()
    try: rc = sentai.tpu.invoke()
    except Exception as e:
        rc = -1; print("  warm0 EXC:", e)
    warm0 = sentai.rtos.ticks_ms() - t0
    print("  load=%d ms warm0=%d ms rc=%s" % (load_ms, warm0, rc))
    samples = []
    fails = 0
    for i in range(N):
        t0 = sentai.rtos.ticks_ms()
        try: rc = sentai.tpu.invoke()
        except Exception:
            rc = -1
        ms = sentai.rtos.ticks_ms() - t0
        if isinstance(rc, int) and rc < 0:
            fails += 1
        samples.append(ms)
        if (i & 0x07) == 0: sentai.diag.repl_kick()
    samples.sort()
    n = len(samples)
    median = samples[n // 2]
    p99    = samples[min(n - 1, int(n * 0.99))]
    mn     = sum(samples) // n
    print("  steady N=%d med=%d p99=%d mean=%d fails=%d" %
          (n, median, p99, mn, fails))
    sentai.fs.append(csv_path,
        "%d,%d,%s,%d,%d,%d,%d,%d,%d\r\n" %
        (cycle, slot, label, load_ms, warm0, median, p99, mn, fails))


sess = _sd("two_model_alt")
csv_path = sess + "/results.csv"
sentai.fs.write(csv_path,
    "cycle,slot,label,load_ms,warm0_ms,median_ms,p99_ms,mean_ms,fails\r\n")
print("=== session:", sess, "===")

# Pipeline must NOT be running.
try:
    if sentai.pipeline.running(): sentai.pipeline.stop()
except Exception: pass

for c in range(CYCLES):
    print("\n========== cycle %d/%d ==========" % (c + 1, CYCLES))
    _phase(A[0], A[1], csv_path, c, 0)
    _phase(B[0], B[1], csv_path, c, 1)

print("\nCSV:", csv_path)
print("=== done ===")

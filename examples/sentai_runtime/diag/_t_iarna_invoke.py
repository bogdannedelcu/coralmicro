# _t_iarna_invoke.py — direct tpu.invoke() with bumped USB-IN timeout.
#
# Goal: isolate GetOutputs failure cause.  No camera, no pipeline,
# no PrepTask — just load model + invoke + read tpu_call_stats.
# Bumps urb_timeout to 5000 ms to rule out a slow-but-successful
# compute being clipped by the default 200 ms ceiling.
#
# Self-contained per agent.md §5.1.2.
import sentai
sentai.verbose(1)

MODEL = "/iarna_p2p4_5ep_export_640x480_uint8.tflite"


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


print("=== boot ===")
print("--- model:", MODEL, "---")

# Bump per-URB timeout BEFORE load (covers any bulk-in during init too).
try:
    prev_to = sentai.diag.tpu_urb_timeout(5000)
    print("urb_timeout set 5000 (prev_value=%s)" % str(prev_to))
except Exception as e:
    print("urb_timeout FAIL:", e)

# Per-stage USB tracing — prints dma_hints order + each Bulk-IN chunk.
try:
    sentai.diag.tpu_trace(1)
    print("tpu_trace ON")
except Exception as e:
    print("tpu_trace FAIL:", e)

rc = sentai.tpu.load(MODEL)
print("tpu.load :", rc)
if rc != 0:
    print("=== done ===")
    raise SystemExit

# Reset call stats so we measure ONE invoke cleanly.
try: sentai.diag.tpu_call_stats(1)
except Exception as e: print("call_stats reset FAIL:", e)

print()
print("--- invokes (5x, 5 s ceiling each, per-stage cycles) ---")
# First invoke loads parameters (525 KB) and primes the on-TPU cache;
# subsequent invokes hit parameter_caching=present and skip params.
# Disable trace after the first to keep the steady-state lines clean.
# DWT cycle counter @ 800 MHz: 1 ms = 800,000 cyc.
CYC_PER_MS = 800
elapsed = []
for k in range(5):
    if k == 1:
        try: sentai.diag.tpu_trace(0)
        except Exception: pass
    # Reset per-stage cycle counters BEFORE each invoke for clean delta.
    try: sentai.diag.tpu_perf(1)
    except Exception: pass
    t0 = sentai.rtos.ticks_ms()
    try:
        irc = sentai.tpu.invoke()
    except Exception as e:
        irc = "exc %s" % e
    t1 = sentai.rtos.ticks_ms()
    dt = t1 - t0
    elapsed.append(dt)
    try:
        p = sentai.diag.tpu_perf(0)
        cp = p.get("cyc_params", 0) // CYC_PER_MS
        ci = p.get("cyc_ins", 0) // CYC_PER_MS
        cn = p.get("cyc_input", 0) // CYC_PER_MS
        co = p.get("cyc_output", 0) // CYC_PER_MS
        ce = p.get("cyc_event", 0) // CYC_PER_MS
        np = p.get("n_params", 0); ni = p.get("n_ins", 0)
        ninp = p.get("n_input", 0); no = p.get("n_output", 0)
        ne = p.get("n_event", 0)
        print("  inv[%d] %d ms  P=%d/%dms I=%d/%dms In=%d/%dms O=%d/%dms E=%d/%dms" % (
            k, dt, np, cp, ni, ci, ninp, cn, no, co, ne, ce))
    except Exception as e:
        print("  inv[%d] %d ms  perf-read FAIL: %s" % (k, dt, e))

if elapsed:
    n = len(elapsed)
    avg = sum(elapsed) // n
    mn  = min(elapsed)
    mx  = max(elapsed)
    print("--- summary: n=%d  cold=%d ms  warm[1..]=avg %d / min %d / max %d ms ---" % (
          n, elapsed[0], sum(elapsed[1:]) // max(1, n-1), min(elapsed[1:]) if n>1 else 0,
          max(elapsed[1:]) if n>1 else 0))
    irc = elapsed

# Read after-invoke stats.  in_done > 0 means USB Bulk-IN actually
# fired (output bytes arrived).  in_done == 0 = nothing came back.
try:
    cs = sentai.diag.tpu_call_stats(0)
    print("call_stats:", cs)
except Exception as e:
    print("call_stats FAIL:", e)

# Restore defaults so the next driver isn't surprised.
try: sentai.diag.tpu_urb_timeout(200)
except Exception: pass
try: sentai.diag.tpu_trace(0)
except Exception: pass

sd = _sd("iarna_invoke_to5s")
sentai.fs.write(sd + "/info.txt",
                "model=%s\nrc=%s\nelapsed_ms=%d\nstats=%s\n" %
                (MODEL, str(irc), (t1 - t0), str(cs)))
print("log:", sd + "/info.txt")
print("=== done ===")

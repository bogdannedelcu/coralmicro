import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
N_RUNS = 5
N_INVOKES = 30

TAGS = ("instructions", "input", "parameters", "output", "event", "unknown")
FIELDS = (
    "calls", "callbacks", "bytes_req", "bytes_done", "submit_cyc",
    "callback_cyc", "wait_cyc", "errors", "timeouts", "submit_fail",
)

def _row_to_dict(row):
    d = {}
    for i, name in enumerate(FIELDS):
        d[name] = row[i]
    return d

def _urb_snapshot(reset=False):
    hz, rows = sentai.tpu.urb_stats(1 if reset else 0)
    out = {}
    for i, tag in enumerate(TAGS):
        out[tag] = _row_to_dict(rows[i])
    return hz, out

def _ms(cyc, hz):
    if hz <= 0:
        return 0.0
    return (float(cyc) * 1000.0) / float(hz)

def _print_rows(hz, stats):
    print("cycle_hz=%d" % hz)
    print("tag,calls,callbacks,bytes_req,bytes_done,submit_ms,callback_ms,wait_ms,errors,timeouts,submit_fail")
    for tag in TAGS:
        s = stats[tag]
        print("%s,%d,%d,%d,%d,%.6f,%.6f,%.6f,%d,%d,%d" % (
            tag,
            s["calls"], s["callbacks"], s["bytes_req"], s["bytes_done"],
            _ms(s["submit_cyc"], hz), _ms(s["callback_cyc"], hz),
            _ms(s["wait_cyc"], hz), s["errors"], s["timeouts"],
            s["submit_fail"]))

def _maybe_perf_reset():
    try:
        sentai.diag.tpu_perf(1)
        return True
    except:
        return False

def _maybe_perf_read():
    try:
        return sentai.diag.tpu_perf()
    except:
        return None

def run_burst():
    have_perf = _maybe_perf_reset()
    _urb_snapshot(True)
    t0 = sentai.rtos.ticks_ms()
    for i in range(N_INVOKES):
        sentai.tpu.invoke()
    t1 = sentai.rtos.ticks_ms()
    hz, urb = _urb_snapshot(False)
    perf = _maybe_perf_read() if have_perf else None
    print("total_ms_per_invoke=%.3f" % ((t1 - t0) / float(N_INVOKES)))
    if perf:
        print("legacy_perf=", perf)
    _print_rows(hz, urb)

print("S234 iter4 URB timing")
print("model=%s" % MODEL)
print("runs=%d invokes_per_run=%d" % (N_RUNS, N_INVOKES))
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(5):
    sentai.tpu.invoke()
print("warmed")

for r in range(N_RUNS):
    print("=== run %d ===" % r)
    run_burst()

print("=== done ===")

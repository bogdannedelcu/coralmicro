import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
N_INVOKES = 100
TAGS = ("instructions", "input", "parameters", "output", "event", "unknown")
FIELDS = (
    "calls", "callbacks", "bytes_req", "bytes_done",
    "submit_cyc", "callback_cyc", "wait_cyc",
    "errors", "timeouts", "submit_fail",
    "submit_us", "callback_us", "wait_us",
)

def du32(a, b):
    return (int(b) - int(a)) & 0xffffffff

def stats(reset=False):
    hz, rows = sentai.tpu.urb_stats(1 if reset else 0)
    out = {}
    for i, tag in enumerate(TAGS):
        d = {}
        row = rows[i]
        for j, field in enumerate(FIELDS):
            d[field] = row[j]
        out[tag] = d
    return hz, out

def ms(cyc, hz):
    return (float(cyc) * 1000.0) / float(hz)

print("S234 iter4 URB wall-clock timing")
print("version=%s" % sentai.version())
print("model=%s" % MODEL)
print("invokes=%d" % N_INVOKES)
print("urb_row_len=%d" % len(sentai.tpu.urb_stats()[1][0]))

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(5):
    sentai.tpu.invoke()
print("warmed")

stats(True)
t0_ms = sentai.rtos.ticks_ms()
t0_us = sentai.rtos.micros()
c0 = sentai.rtos.cycles()
for i in range(N_INVOKES):
    sentai.tpu.invoke()
c1 = sentai.rtos.cycles()
t1_us = sentai.rtos.micros()
t1_ms = sentai.rtos.ticks_ms()
hz, rows = stats(False)

dt_ms = du32(t0_ms, t1_ms)
dt_us = du32(t0_us, t1_us)
dc = du32(c0, c1)
print("reported_hz=%d" % hz)
print("total_ticks_ms=%d per=%.6f" % (dt_ms, float(dt_ms) / N_INVOKES))
print("total_wall_us=%d per_ms=%.6f" %
      (dt_us, float(dt_us) / (1000.0 * N_INVOKES)))
print("total_raw_cycles=%d cycles_per_invoke=%.3f hz_vs_wall=%.3f" %
      (dc, float(dc) / N_INVOKES,
       (float(dc) * 1000000.0 / float(dt_us)) if dt_us else 0.0))
print("tag,calls,bytes_req,bytes_done,submit_us_per_inv,callback_us_per_inv,wait_us_per_inv,wait_ms_per_inv,wait_dwt_ms_per_inv,errors,timeouts,submit_fail")
for tag in TAGS:
    s = rows[tag]
    print("%s,%d,%d,%d,%.3f,%.3f,%.3f,%.6f,%.6f,%d,%d,%d" % (
        tag,
        s["calls"], s["bytes_req"], s["bytes_done"],
        float(s["submit_us"]) / N_INVOKES,
        float(s["callback_us"]) / N_INVOKES,
        float(s["wait_us"]) / N_INVOKES,
        float(s["wait_us"]) / (1000.0 * N_INVOKES),
        ms(s["wait_cyc"], hz) / N_INVOKES,
        s["errors"], s["timeouts"], s["submit_fail"]))

print("=== urb_wall_done ===")

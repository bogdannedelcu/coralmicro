import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
N_INVOKES = 100
TAGS = ("instructions", "input", "parameters", "output", "event", "unknown")
FIELDS = (
    "calls", "callbacks", "bytes_req", "bytes_done", "submit_cyc",
    "callback_cyc", "wait_cyc", "errors", "timeouts", "submit_fail",
)

def du32(a, b):
    return (int(b) - int(a)) & 0xffffffff

def snap_urb(reset=False):
    hz, rows = sentai.tpu.urb_stats(1 if reset else 0)
    out = {}
    for i, tag in enumerate(TAGS):
        row = rows[i]
        out[tag] = {}
        for j, field in enumerate(FIELDS):
            out[tag][field] = row[j]
    return hz, out

def ms_from_cyc(cyc, hz):
    return (float(cyc) * 1000.0) / float(hz)

print("S234 iter4 clock calibration + 100 invokes")
print("version=%s" % sentai.version())
print("model=%s" % MODEL)

cal = []
for sleep_ms in (100, 500, 1000, 2000):
    c0 = sentai.rtos.cycles()
    u0 = sentai.rtos.micros()
    t0 = sentai.rtos.ticks_ms()
    sentai.rtos.sleep_ms(sleep_ms)
    c1 = sentai.rtos.cycles()
    u1 = sentai.rtos.micros()
    t1 = sentai.rtos.ticks_ms()
    dc = du32(c0, c1)
    du = du32(u0, u1)
    dt = du32(t0, t1)
    hz = (float(dc) * 1000000.0) / float(du) if du > 0 else 0.0
    cal.append(hz)
    print("cal_sleep_ms=%d ticks_ms=%d micros=%d cycles=%d hz=%.3f" %
          (sleep_ms, dt, du, dc, hz))

cal_hz = sum(cal) / len(cal)
reported_hz = snap_urb(False)[0]
print("reported_hz=%d" % reported_hz)
print("calibrated_hz_avg=%.3f" % cal_hz)
print("reported_vs_calibrated_pct=%.6f" %
      ((float(reported_hz) - cal_hz) * 100.0 / cal_hz))

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(5):
    sentai.tpu.invoke()
print("warmed")

snap_urb(True)
t0 = sentai.rtos.ticks_ms()
u0 = sentai.rtos.micros()
c0 = sentai.rtos.cycles()
for i in range(N_INVOKES):
    sentai.tpu.invoke()
c1 = sentai.rtos.cycles()
u1 = sentai.rtos.micros()
t1 = sentai.rtos.ticks_ms()
reported_hz, stats = snap_urb(False)
dt = du32(t0, t1)
du = du32(u0, u1)
dc_total = du32(c0, c1)
print("invoke_count=%d" % N_INVOKES)
print("invoke_ticks_ms_total=%d per=%.6f" % (dt, float(dt) / N_INVOKES))
print("invoke_micros_total=%d per=%.6f" % (du, float(du) / N_INVOKES))
print("invoke_raw_cycles_total=%d per=%.3f hz_from_micros=%.3f" %
      (dc_total, float(dc_total) / N_INVOKES,
       (float(dc_total) * 1000000.0 / float(du)) if du > 0 else 0.0))
print("tag,calls,bytes_req,wait_ms_reported_per_invoke,wait_ms_calibrated_per_invoke,submit_ms_reported_per_invoke,callback_ms_reported_per_invoke")
for tag in TAGS:
    s = stats[tag]
    wait_rep = ms_from_cyc(s["wait_cyc"], reported_hz) / N_INVOKES
    wait_cal = ms_from_cyc(s["wait_cyc"], cal_hz) / N_INVOKES
    sub_rep = ms_from_cyc(s["submit_cyc"], reported_hz) / N_INVOKES
    cb_rep = ms_from_cyc(s["callback_cyc"], reported_hz) / N_INVOKES
    print("%s,%d,%d,%.6f,%.6f,%.6f,%.6f" %
          (tag, s["calls"], s["bytes_req"], wait_rep, wait_cal, sub_rep, cb_rep))

print("=== clock_calib_done ===")

import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
CHUNK_KB = 36
N_INVOKES = 50
WARMUP = 5
OUT_DIR = "/diags/s235_b10_pure_tpu"
OUT_CSV = OUT_DIR + "/pure_tpu_wall.csv"

TAGS = ("instructions", "input", "parameters", "output", "event", "unknown")
FIELDS = (
    "calls", "callbacks", "bytes_req", "bytes_done",
    "submit_cyc", "callback_cyc", "wait_cyc",
    "errors", "timeouts", "submit_fail",
    "submit_us", "callback_us", "wait_us",
)


def du32(a, b):
    return (int(b) - int(a)) & 0xffffffff


def ensure_out():
    try:
        sentai.fs.mkdir("/diags")
    except Exception:
        pass
    try:
        sentai.fs.mkdir(OUT_DIR)
    except Exception:
        pass
    try:
        if not sentai.fs.exists(OUT_CSV):
            sentai.fs.write(
                OUT_CSV,
                "version,chunk_kb,invokes,wall_us,wall_ms_per_invoke,"
                "fps,tick_ms_per_invoke,failures,"
                "phase,calls,bytes_req,bytes_done,wait_ms_per_invoke,"
                "errors,timeouts,submit_fail\r\n",
            )
    except Exception as e:
        print("fs init failed", repr(e))


def urb(reset=False):
    hz, rows = sentai.tpu.urb_stats(1 if reset else 0)
    out = {}
    for i, tag in enumerate(TAGS):
        row = rows[i]
        d = {}
        for j, field in enumerate(FIELDS):
            d[field] = row[j]
        out[tag] = d
    return hz, out


def append_rows(version, chunk_kb, invokes, wall_us, tick_ms, failures, rows):
    wall_ms_per = float(wall_us) / (1000.0 * invokes)
    fps = (1000000.0 * invokes) / float(wall_us) if wall_us else 0.0
    tick_ms_per = float(tick_ms) / invokes
    for tag in TAGS:
        r = rows[tag]
        line = "%s,%d,%d,%d,%.6f,%.6f,%.6f,%d,%s,%d,%d,%d,%.6f,%d,%d,%d\r\n" % (
            version,
            chunk_kb,
            invokes,
            wall_us,
            wall_ms_per,
            fps,
            tick_ms_per,
            failures,
            tag,
            int(r["calls"]),
            int(r["bytes_req"]),
            int(r["bytes_done"]),
            float(r["wait_us"]) / (1000.0 * invokes),
            int(r["errors"]),
            int(r["timeouts"]),
            int(r["submit_fail"]),
        )
        sentai.fs.append(OUT_CSV, line)
    try:
        sentai.fs.sync()
    except Exception:
        pass


sentai.verbose(1)
ensure_out()

print("S235 B10.2 pure TPU wall one chunk")
print("version=%s" % sentai.version())
print("model=%s" % MODEL)
print("chunk_kb=%d invokes=%d warmup=%d" % (CHUNK_KB, N_INVOKES, WARMUP))

try:
    prev_chunk = sentai.tpu.chunk_size()
    now_chunk = sentai.tpu.chunk_size(CHUNK_KB * 1024)
    print("chunk_set prev=%s now=%d" % (str(prev_chunk), now_chunk))
except Exception as e:
    print("chunk_set failed", repr(e))
    raise

try:
    print("urb_timeout_ms=%s desc_cache=default_off_not_toggled" %
          str(sentai.tpu.urb_timeout_ms()))
except Exception as e:
    print("tpu state failed", repr(e))

rc = sentai.tpu.load(MODEL)
print("load_rc=%s" % str(rc))
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(WARMUP):
    r = sentai.tpu.invoke()
    print("warmup%d=%s" % (i, str(r)))

urb(True)
t0_ms = sentai.rtos.ticks_ms()
t0_us = sentai.rtos.micros()
failures = 0
for i in range(N_INVOKES):
    r = sentai.tpu.invoke()
    if r < 0:
        failures += 1
t1_us = sentai.rtos.micros()
t1_ms = sentai.rtos.ticks_ms()
hz, rows = urb(False)

wall_us = du32(t0_us, t1_us)
tick_ms = du32(t0_ms, t1_ms)
wall_ms_per = float(wall_us) / (1000.0 * N_INVOKES)
fps = (1000000.0 * N_INVOKES) / float(wall_us) if wall_us else 0.0

print("reported_hz=%d" % hz)
print("result chunk_kb=%d wall_us=%d wall_ms_per=%.6f fps=%.6f tick_ms_per=%.6f failures=%d" %
      (CHUNK_KB, wall_us, wall_ms_per, fps, float(tick_ms) / N_INVOKES, failures))
print("phase,calls,bytes_req,bytes_done,wait_ms_per_invoke,errors,timeouts,submit_fail")
for tag in TAGS:
    r = rows[tag]
    print("%s,%d,%d,%d,%.6f,%d,%d,%d" % (
        tag,
        int(r["calls"]),
        int(r["bytes_req"]),
        int(r["bytes_done"]),
        float(r["wait_us"]) / (1000.0 * N_INVOKES),
        int(r["errors"]),
        int(r["timeouts"]),
        int(r["submit_fail"]),
    ))

append_rows(sentai.version(), CHUNK_KB, N_INVOKES, wall_us, tick_ms, failures, rows)
print("csv=%s" % OUT_CSV)
print("=== s235_b10_pure_tpu_one_chunk_done ===")

# _t_s236_speed.py - per-model standalone TPU speed sweep for the 8 cover_v1
# headless p3p4 detectors (640x480). Modeled on s235 pure_tpu_wall + the iarna
# resume-across-reboot pattern (self-contained per agent.md 5.1.2).
#
# Per model: load, camera 640x480 -> to_tensor (input), warmup, then time N
# invokes with sentai.rtos.micros() (wall clock, NOT ticks/DWT per A10), read
# URB phase breakdown (instructions/input/output bytes + wait_us). Writes one
# CSV row per model. sys.reset() between models to clear TPU state (a failing
# model wedges the TPU for the boot). Resumes via /diags/.s236_state.
#
# Host: upload once, exec; re-exec after each reset to resume; pull results.csv.
import sentai
sentai.verbose(1)

MODELS = [
    ("msblock",    "/msblock.tflite"),
    ("c3",         "/c3.tflite"),
    ("c2f",        "/c2f.tflite"),
    ("c2f_pan2",   "/c2f_pan2.tflite"),
    ("gelan",      "/gelan.tflite"),
    ("gelan_pan2", "/gelan_pan2.tflite"),
    ("c2f_deep",   "/c2f_deep.tflite"),
    ("c2f_thick",  "/c2f_thick.tflite"),
]
CHUNK_KB = 64
N = 50
WARMUP = 5
W, H = 640, 480
STATE = "/diags/.s236_state"
TAGS = ("instructions", "input", "parameters", "output", "event", "unknown")
HDR = ("model,size,load_rc,invokes,wall_us,ms_per_invoke,fps,fails,"
       "instr_bytes,instr_wait_ms,input_bytes,input_wait_ms,"
       "output_bytes,output_wait_ms,params_wait_ms\r\n")


def du32(a, b):
    return (int(b) - int(a)) & 0xffffffff


def urb_row(rows, tag):
    # FIELDS index: 1=calls,2=bytes_req,3=bytes_done,...,12=wait_us
    i = TAGS.index(tag)
    r = rows[i]
    return int(r[3]), float(r[12])  # bytes_done, wait_us


def bench_one(label, path, csv):
    print("\n=== %s ===" % label)
    try:
        sz = sentai.fs.size(path)
    except Exception:
        sz = -1
    sentai.tpu.chunk_size(CHUNK_KB * 1024)
    try:
        sentai.tpu.desc_cache(0)
    except Exception:
        pass
    ld = sentai.tpu.load(path)
    print("load=%s size=%d ready=%s" % (str(ld), sz, str(sentai.tpu.ready())))
    if ld != 0:
        sentai.fs.append(csv, "%s,%d,%s,0,0,0,0,%d,,,,,,,\r\n" % (label, sz, str(ld), N))
        return
    sentai.camera.to_tensor()
    sentai.rtos.sleep_ms(200)
    for _ in range(WARMUP):
        sentai.tpu.invoke()
    sentai.tpu.urb_stats(1)
    fails = 0
    t0 = sentai.rtos.micros()
    for _ in range(N):
        if sentai.tpu.invoke() < 0:
            fails += 1
    t1 = sentai.rtos.micros()
    hz, rows = sentai.tpu.urb_stats(0)
    wall = du32(t0, t1)
    mspi = float(wall) / (1000.0 * N)
    fps = (1000000.0 * N) / float(wall) if wall else 0.0
    ib, iw = urb_row(rows, "instructions")
    nb, nw = urb_row(rows, "input")
    ob, ow = urb_row(rows, "output")
    _, pw = urb_row(rows, "parameters")
    print("wall_us=%d ms/inv=%.3f fps=%.2f fails=%d" % (wall, mspi, fps, fails))
    print("  instr=%dB/%.2fms input=%dB/%.2fms output=%dB/%.2fms" %
          (ib, iw / N / 1000.0, nb, nw / N / 1000.0, ob, ow / N / 1000.0))
    sentai.fs.append(csv, "%s,%d,%s,%d,%d,%.3f,%.3f,%d,%d,%.3f,%d,%.3f,%d,%.3f,%.3f\r\n" % (
        label, sz, str(ld), N, wall, mspi, fps, fails,
        ib, iw / N / 1000.0, nb, nw / N / 1000.0, ob, ow / N / 1000.0, pw / N / 1000.0))
    try:
        sentai.fs.sync()
    except Exception:
        pass


# resume state: "sess|idx"
sess = None
idx = 0
try:
    raw = sentai.fs.read_str(STATE).strip()
    a, b = raw.split("|")
    if sentai.fs.exists(a):
        sess = a
        idx = int(b)
except Exception:
    pass

if sess is None:
    try:
        sentai.fs.mkdir("/diags")
    except Exception:
        pass
    c = "/diags/.counter"
    sid = 1
    try:
        sid = int(sentai.fs.read_str(c).strip()) + 1
    except Exception:
        pass
    try:
        sentai.fs.write(c, str(sid))
    except Exception:
        pass
    sess = "/diags/s%03d_s236_speed" % sid
    try:
        sentai.fs.mkdir(sess)
    except Exception:
        pass
    sentai.fs.write(sess + "/results.csv", HDR)
    print("=== fresh session:", sess, "===")
else:
    print("=== resume session:", sess, "@ idx", idx, "===")

csv = sess + "/results.csv"
# camera init once per boot
sentai.camera.set_resolution(W, H)
sentai.camera.init(1)
sentai.rtos.sleep_ms(800)

while idx < len(MODELS):
    label, path = MODELS[idx]
    bench_one(label, path, csv)
    idx += 1
    if idx < len(MODELS):
        sentai.fs.write(STATE, "%s|%d" % (sess, idx))
        print("--- %d/%d done; sys.reset() to clear TPU ---" % (idx, len(MODELS)))
        sentai.rtos.sleep_ms(400)
        sentai.sys.reset()

try:
    sentai.fs.remove(STATE)
except Exception:
    pass
print("\nCSV:", csv)
print("=== done ===")

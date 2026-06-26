import sentai

RUN_ID = int(globals().get("_run_id", 0))
DRAIN = int(globals().get("_drain", 1))
RATIO_A = int(globals().get("_ratio_a", 1))
RATIO_B = int(globals().get("_ratio_b", 1))
FRAMES = int(globals().get("_frames", 100))
OUT_DIR = "/diags/s235_b10_camera_pattern"

HEADER = (
    "version,run,drain,ratio_a,ratio_b,frames,correct,scrambled,"
    "wrong_tag,ambig,cam0,cam1,other,bars,hband,wall_us,fps,"
    "cam_stats_before,cam_stats_after\r\n"
)


def csv_escape(s):
    s = str(s)
    if "," in s or '"' in s or "\n" in s or "\r":
        return '"' + s.replace('"', '""') + '"'
    return s


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


def classify_b(b):
    if b >= 0xA0:
        return "BARS"
    if b <= 0x80:
        return "HBAND"
    return "AMBIG"


def classify_5(samples):
    classes = tuple(classify_b(b) for b in samples)
    if all(c == "BARS" for c in classes):
        return "BARS"
    if all(c == "HBAND" for c in classes):
        return "HBAND"
    if all(c == "AMBIG" for c in classes):
        return "AMBIG"
    return "SCRAMBLED"


def write_line(path, line):
    try:
        if sentai.fs.size(path) <= 0:
            sentai.fs.write(path, "")
    except Exception:
        sentai.fs.write(path, "")
    sentai.fs.append(path, line)


def configure():
    sentai.verbose(1)
    ensure_out()
    print("S235 B10.5 camera pattern integrity once")
    print("version=%s" % sentai.version())
    print("run=%d drain=%d ratio=%d:%d frames=%d" %
          (RUN_ID, DRAIN, RATIO_A, RATIO_B, FRAMES))
    try:
        sentai.pipeline.stop()
    except Exception:
        pass
    print("camera_init=%s" % str(sentai.camera.init(1)))
    print("ratio_prev=%s" % str(sentai.camera.ratio(0, 0)))
    print("switch_drain_prev=%s" % str(sentai.camera.switch_drain(DRAIN)))
    sentai.rtos.sleep_ms(500)
    print("pattern0=%s pattern1=%s" %
          (str(sentai.camera.test_pattern(0, 1)),
           str(sentai.camera.test_pattern(1, 2))))
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(500)
    sentai.camera.select(1)
    sentai.rtos.sleep_ms(500)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)


def run_once():
    path = "%s/run_%02d_ratio%d_%d_drain%d.csv" % (
        OUT_DIR, RUN_ID, RATIO_A, RATIO_B, DRAIN)
    detail_path = "%s/run_%02d_ratio%d_%d_drain%d_detail.csv" % (
        OUT_DIR, RUN_ID, RATIO_A, RATIO_B, DRAIN)
    sentai.fs.write(path, HEADER)
    detail_lines = [
        "i,cam_tag,frame_class,b0,b1,b2,b3,b4,verdict\r\n"
    ]

    correct = scrambled = wrong_tag = ambig = 0
    cam0 = cam1 = other = 0
    bars = hband = 0
    try:
        stats_before = sentai.camera.stats()
    except Exception:
        stats_before = {}

    sentai.camera.ratio(RATIO_A, RATIO_B)
    sentai.camera.switch_drain(DRAIN)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    try:
        sentai.camera.peek5_b40()
    except Exception:
        pass

    t0 = sentai.rtos.micros()
    for i in range(FRAMES):
        r = sentai.camera.peek5_b40()
        tag = int(r[0])
        samples = tuple(int(x) for x in r[1:6])
        frame_cls = classify_5(samples)
        verdict = "wrong"
        if tag == 0:
            cam0 += 1
        elif tag == 1:
            cam1 += 1
        else:
            other += 1
        if frame_cls == "BARS":
            bars += 1
        elif frame_cls == "HBAND":
            hband += 1
        if frame_cls == "SCRAMBLED":
            scrambled += 1
            verdict = "scrambled"
        elif frame_cls == "AMBIG":
            ambig += 1
            verdict = "ambig"
        elif (frame_cls == "BARS" and tag == 0) or (frame_cls == "HBAND" and tag == 1):
            correct += 1
            verdict = "correct"
        else:
            wrong_tag += 1
        detail_lines.append(
            "%d,%d,%s,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,%s\r\n" %
            (i, tag, frame_cls, samples[0], samples[1], samples[2],
             samples[3], samples[4], verdict))
        if (i % 20) == 0:
            try:
                sentai.diag.repl_kick()
            except Exception:
                pass
    t1 = sentai.rtos.micros()

    sentai.camera.ratio(0, 0)
    sentai.camera.select(0)
    sentai.camera.test_pattern(0, 0)
    sentai.camera.test_pattern(1, 0)
    try:
        stats_after = sentai.camera.stats()
    except Exception:
        stats_after = {}

    wall_us = du32(t0, t1)
    fps = (1000000.0 * float(FRAMES)) / float(wall_us) if wall_us else 0.0
    vals = (
        sentai.version(), RUN_ID, DRAIN, RATIO_A, RATIO_B, FRAMES, correct,
        scrambled, wrong_tag, ambig, cam0, cam1, other, bars, hband,
        wall_us, "%.6f" % fps, repr(stats_before), repr(stats_after),
    )
    sentai.fs.append(path, ",".join(csv_escape(v) for v in vals) + "\r\n")
    sentai.fs.write(detail_path, "".join(detail_lines))
    try:
        print("fs_sync=%s" % str(sentai.fs.sync()))
    except Exception as e:
        print("fs_sync_failed", repr(e))
    print(
        "result run=%d drain=%d ratio=%d:%d frames=%d correct=%d "
        "scrambled=%d wrong_tag=%d ambig=%d cam0=%d cam1=%d fps=%.6f" %
        (RUN_ID, DRAIN, RATIO_A, RATIO_B, FRAMES, correct, scrambled,
         wrong_tag, ambig, cam0, cam1, fps)
    )
    print("csv=%s" % path)
    print("detail_csv=%s" % detail_path)


configure()
run_once()
print("=== done ===")

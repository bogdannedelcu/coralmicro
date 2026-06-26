import sentai

RUN_ID = int(globals().get("_run_id", 0))
FRAMES = int(globals().get("_frames", 50))
RATIO_A = int(globals().get("_ratio_a", 1))
RATIO_B = int(globals().get("_ratio_b", 1))
OUT_DIR = "/diags/s235_b10_drain_probe"


def ensure_out():
    try:
        sentai.fs.mkdir("/diags")
    except Exception:
        pass
    try:
        sentai.fs.mkdir(OUT_DIR)
    except Exception:
        pass


def cls_b(v):
    if v >= 0xA0:
        return "BARS"
    if v <= 0x80:
        return "HBAND"
    return "AMBIG"


def cls5(samples):
    c = tuple(cls_b(x) for x in samples)
    if all(x == "BARS" for x in c):
        return "BARS"
    if all(x == "HBAND" for x in c):
        return "HBAND"
    if all(x == "AMBIG" for x in c):
        return "AMBIG"
    return "SCRAMBLED"


def empty():
    return {
        "frames": 0, "correct": 0, "wrong": 0, "scrambled": 0,
        "ambig": 0, "cam0": 0, "cam1": 0, "other": 0,
        "bars": 0, "hband": 0,
    }


def sample_frame():
    r = sentai.camera.peek5_b40()
    tag = int(r[0])
    samples = tuple(int(x) for x in r[1:6])
    kind = cls5(samples)
    return tag, kind, samples


def add_sample(acc, tag, kind):
    acc["frames"] += 1
    if tag == 0:
        acc["cam0"] += 1
    elif tag == 1:
        acc["cam1"] += 1
    else:
        acc["other"] += 1
    if kind == "BARS":
        acc["bars"] += 1
    elif kind == "HBAND":
        acc["hband"] += 1
    if kind == "SCRAMBLED":
        acc["scrambled"] += 1
    elif kind == "AMBIG":
        acc["ambig"] += 1
    elif (tag == 0 and kind == "BARS") or (tag == 1 and kind == "HBAND"):
        acc["correct"] += 1
    else:
        acc["wrong"] += 1


def run_case(label, drain, ratio_a, ratio_b, frames):
    print("case_start label=%s drain=%d ratio=%d:%d frames=%d" %
          (label, drain, ratio_a, ratio_b, frames))
    sentai.camera.ratio(0, 0)
    sentai.camera.switch_drain(drain)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    sentai.camera.ratio(ratio_a, ratio_b)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    try:
        sentai.camera.peek5_b40()
    except Exception:
        pass
    acc = empty()
    t0 = sentai.rtos.micros()
    for i in range(frames):
        tag, kind, _samples = sample_frame()
        add_sample(acc, tag, kind)
        if (i % 10) == 0:
            print("progress label=%s i=%d tag=%d kind=%s" % (label, i, tag, kind))
            try:
                sentai.diag.repl_kick()
            except Exception:
                pass
        sentai.rtos.sleep_ms(1)
    t1 = sentai.rtos.micros()
    wall = (int(t1) - int(t0)) & 4294967295
    fps = (1000000.0 * float(frames)) / float(wall) if wall else 0.0
    acc["wall_us"] = wall
    acc["fps"] = fps
    print(
        "case_result label=%s drain=%d ratio=%d:%d frames=%d correct=%d "
        "wrong=%d scrambled=%d ambig=%d cam0=%d cam1=%d bars=%d hband=%d "
        "wall_us=%d fps=%.6f" %
        (label, drain, ratio_a, ratio_b, acc["frames"], acc["correct"],
         acc["wrong"], acc["scrambled"], acc["ambig"], acc["cam0"],
         acc["cam1"], acc["bars"], acc["hband"], wall, fps)
    )
    return acc


def main():
    sentai.verbose(1)
    ensure_out()
    print("S235 B10 drain probe")
    print("version=%s" % sentai.version())
    print("run=%d frames=%d ratio=%d:%d" % (RUN_ID, FRAMES, RATIO_A, RATIO_B))
    try:
        sentai.pipeline.stop()
    except Exception:
        pass
    print("camera_init=%s" % str(sentai.camera.init(1)))
    sentai.camera.ratio(0, 0)
    sentai.rtos.sleep_ms(500)
    print("test_pattern0=%s test_pattern1=%s" %
          (str(sentai.camera.test_pattern(0, 1)),
           str(sentai.camera.test_pattern(1, 2))))
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(500)
    sentai.camera.select(1)
    sentai.rtos.sleep_ms(500)

    rows = []
    for drain in (1, 2):
        acc = run_case("alt", drain, RATIO_A, RATIO_B, FRAMES)
        rows.append((drain, acc))

    sentai.camera.ratio(0, 0)
    sentai.camera.select(0)
    sentai.camera.test_pattern(0, 0)
    sentai.camera.test_pattern(1, 0)
    csv = "drain,ratio_a,ratio_b,frames,correct,wrong,scrambled,ambig,cam0,cam1,bars,hband,wall_us,fps\r\n"
    for drain, acc in rows:
        csv += "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.6f\r\n" % (
            drain, RATIO_A, RATIO_B, acc["frames"], acc["correct"],
            acc["wrong"], acc["scrambled"], acc["ambig"], acc["cam0"],
            acc["cam1"], acc["bars"], acc["hband"], acc["wall_us"],
            acc["fps"])
    path = "%s/run_%02d_ratio%d_%d.csv" % (OUT_DIR, RUN_ID, RATIO_A, RATIO_B)
    sentai.fs.write(path, csv)
    try:
        print("fs_sync=%s" % str(sentai.fs.sync()))
    except Exception:
        pass
    print("csv=%s" % path)
    print("=== done ===")


main()

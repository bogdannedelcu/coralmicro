import sentai

RUN_ID = int(globals().get("_run_id", 0))
FRAMES = int(globals().get("_frames", 4))
QUALITY = int(globals().get("_quality", 55))
PATTERN0 = int(globals().get("_pattern0", 1))
PATTERN1 = int(globals().get("_pattern1", 2))
OUT_DIR = "/diags/s235_b10_visual_seam"


def ensure_out():
    try:
        sentai.fs.mkdir("/diags")
    except Exception:
        pass
    try:
        sentai.fs.mkdir(OUT_DIR)
    except Exception:
        pass


def du32(a, b):
    return (int(b) - int(a)) & 4294967295


def capture_case(drain):
    sentai.camera.ratio(0, 0)
    sentai.camera.switch_drain(drain)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    sentai.camera.ratio(1, 1)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    rows = []
    for i in range(FRAMES):
        tag = -1
        cur = -1
        path = "%s/run%02d_d%d_%02d.jpg" % (OUT_DIR, RUN_ID, drain, i)
        t0 = sentai.rtos.micros()
        size = sentai.camera.save_jpeg(path, QUALITY)
        t1 = sentai.rtos.micros()
        try:
            tag = int(sentai.camera.grabbed_id())
        except Exception:
            pass
        try:
            cur = int(sentai.camera.current_id())
        except Exception:
            pass
        tagged_path = "%s/run%02d_d%d_%02d_tag%d_cur%d.jpg" % (
            OUT_DIR, RUN_ID, drain, i, tag, cur)
        try:
            sentai.fs.rename(path, tagged_path)
            path = tagged_path
        except Exception:
            pass
        row = (drain, i, tag, cur, int(size), du32(t0, t1), path)
        rows.append(row)
        print("frame drain=%d i=%d tag=%d cur=%d bytes=%d us=%d path=%s" %
              (drain, i, tag, cur, int(size), du32(t0, t1), path))
        try:
            sentai.diag.repl_kick()
        except Exception:
            pass
    return rows


def main():
    sentai.verbose(1)
    ensure_out()
    print("S235 B10 visual seam")
    print("version=%s" % sentai.version())
    print("run=%d frames=%d quality=%d pattern0=%d pattern1=%d" %
          (RUN_ID, FRAMES, QUALITY, PATTERN0, PATTERN1))
    try:
        sentai.pipeline.stop()
    except Exception:
        pass
    print("camera_init=%s" % str(sentai.camera.init(1)))
    sentai.rtos.sleep_ms(500)
    print("pattern0=%s pattern1=%s" %
          (str(sentai.camera.test_pattern(0, PATTERN0)),
           str(sentai.camera.test_pattern(1, PATTERN1))))
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(500)
    sentai.camera.select(1)
    sentai.rtos.sleep_ms(500)

    rows = []
    rows.extend(capture_case(1))
    rows.extend(capture_case(2))

    sentai.camera.ratio(0, 0)
    sentai.camera.select(0)
    sentai.camera.test_pattern(0, 0)
    sentai.camera.test_pattern(1, 0)
    csv = (
        "drain,index,grabbed_id,current_id,jpeg_bytes,"
        "jpeg_artifact_time_us,path\r\n"
    )
    for r in rows:
        csv += "%d,%d,%d,%d,%d,%d,%s\r\n" % r
    csv_path = "%s/run%02d_manifest.csv" % (OUT_DIR, RUN_ID)
    sentai.fs.write(csv_path, csv)
    try:
        print("fs_sync=%s" % str(sentai.fs.sync()))
    except Exception:
        pass
    print("manifest=%s" % csv_path)
    print("=== done ===")


main()

import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
CHUNK_KB = 128
RUN_ID = globals().get("_run_id", 0)
FRAMES = globals().get("_frames", 100)
WARMUP = globals().get("_warmup", 5)
CAM_ID = 0
WIDTH = 512
HEIGHT = 512
OUT_DIR = "/diags/s235_b10_pipeline_once"


HEADER = (
    "version,run,model,chunk_kb,direct_tensor,target_fps,prep_fps,"
    "invokes_per_frame,cam_id,width,height,warmup_frames,requested_frames,"
    "observed_frames,wall_us,fps,timeouts,start_count,end_count,latest_seq,"
    "latest_inv_ms,latest_total_ms,latest_memcpy_ms,latest_nms_ms,"
    "latest_cam_id,pipeline_processed,pipeline_dropped,pipeline_diag_fps,"
    "infer_ok,infer_fail,infer_ms_sum,infer_last_rc,prep_frames,"
    "prep_sem_wait_ms_sum,prep_cam_grab_ms_sum,prep_pxp_ms_sum,"
    "prep_quant_ms_sum,prep_total_ms_sum,direct_frames,"
    "direct_prep_buf_timeout,direct_infer_wait_timeout,direct_swap_fail\r\n"
)


def du32(a, b):
    return (int(b) - int(a)) & 0xffffffff


def csv_escape(s):
    s = str(s)
    if "," in s or '"' in s or "\n" in s or "\r":
        return '"' + s.replace('"', '""') + '"'
    return s


def dget(d, k, default=0):
    try:
        return d[k]
    except Exception:
        return default


def ensure_out():
    try:
        sentai.fs.mkdir("/diags")
    except Exception:
        pass
    try:
        sentai.fs.mkdir(OUT_DIR)
    except Exception:
        pass


def reset_stats():
    try:
        sentai.pipeline.prep_reset()
    except Exception as e:
        print("prep_reset_failed", repr(e))
    try:
        sentai.pipeline.infer_reset()
    except Exception as e:
        print("infer_reset_failed", repr(e))


def wait_next(last, timeout_ms):
    return int(sentai.pipeline.frame_count(int(last), int(timeout_ms)))


def drain_warmup(n):
    last = int(sentai.pipeline.frame_count())
    got = 0
    for _ in range(int(n)):
        nxt = wait_next(last, 5000)
        if nxt < 0:
            print("warmup_timeout last=%d rc=%d" % (last, nxt))
            break
        last = nxt
        got += 1
    return last, got


def sample_latest():
    try:
        latest = sentai.pipeline.get_ex(0, 0)
    except Exception as e:
        print("latest_failed", repr(e))
        latest = None
    if latest:
        return (
            int(latest[3]),
            int(latest[1]),
            int(latest[2]),
            int(latest[4]),
            int(latest[5]),
            int(latest[6]),
        )
    return (-1, -1, -1, -1, -1, -1)


def stats_row(warm_got, requested, observed, wall_us, fps, timeouts,
              start_count, end_count):
    latest_seq, latest_inv, latest_total, latest_memcpy, latest_nms, latest_cam = sample_latest()
    try:
        pstats = sentai.pipeline.stats()
    except Exception:
        pstats = (0, 0, 0.0)
    try:
        istats = sentai.pipeline.infer_stats()
    except Exception:
        istats = {}
    try:
        prep = sentai.pipeline.prep_stats()
    except Exception:
        prep = {}
    try:
        dstats = sentai.pipeline.direct_stats()
    except Exception:
        dstats = {}

    vals = (
        sentai.version(),
        int(RUN_ID),
        MODEL,
        CHUNK_KB,
        1,
        0,
        0,
        1,
        CAM_ID,
        WIDTH,
        HEIGHT,
        int(warm_got),
        int(requested),
        int(observed),
        int(wall_us),
        "%.6f" % float(fps),
        int(timeouts),
        int(start_count),
        int(end_count),
        latest_seq,
        latest_inv,
        latest_total,
        latest_memcpy,
        latest_nms,
        latest_cam,
        int(pstats[0]),
        int(pstats[1]),
        "%.6f" % float(pstats[2]),
        int(dget(istats, "ok")),
        int(dget(istats, "fail")),
        int(dget(istats, "ms_sum")),
        int(dget(istats, "last_rc")),
        int(dget(prep, "frames")),
        int(dget(prep, "sem_wait_ms_sum")),
        int(dget(prep, "cam_grab_ms_sum")),
        int(dget(prep, "pxp_ms_sum")),
        int(dget(prep, "quant_ms_sum")),
        int(dget(prep, "total_ms_sum")),
        int(dget(dstats, "frames")),
        int(dget(dstats, "prep_buf_timeout")),
        int(dget(dstats, "infer_wait_timeout")),
        int(dget(dstats, "swap_fail")),
    )
    return ",".join(csv_escape(v) for v in vals) + "\r\n"


def configure():
    sentai.verbose(1)
    ensure_out()
    print("S235 B10.3 pipeline wall-clock once")
    print("version=%s" % sentai.version())
    print("run_id=%d model=%s" % (int(RUN_ID), MODEL))
    print(
        "chunk_kb=%d frames=%d warmup=%d cam=%d %dx%d" %
        (CHUNK_KB, int(FRAMES), int(WARMUP), CAM_ID, WIDTH, HEIGHT)
    )

    try:
        sentai.tpu.desc_cache(0)
    except Exception as e:
        print("desc_cache_disable_failed", repr(e))
    print("chunk_prev=%s chunk_now=%s" %
          (str(sentai.tpu.chunk_size()), str(sentai.tpu.chunk_size(CHUNK_KB * 1024))))
    try:
        print("urb_timeout_ms=%s" % str(sentai.tpu.urb_timeout_ms()))
    except Exception as e:
        print("urb_timeout_read_failed", repr(e))

    print("model_size=%s" % str(sentai.fs.size(MODEL)))
    print("tpu_load=%s" % str(sentai.tpu.load(MODEL)))
    print("tpu_ready=%s" % str(sentai.tpu.ready()))

    try:
        print("pipeline_stop_pre=%s" % str(sentai.pipeline.stop()))
    except Exception as e:
        print("pipeline_stop_pre_failed", repr(e))
    print("direct_tensor_prev=%s" % str(sentai.pipeline.direct_tensor(1)))
    print("target_fps_now=%s" % str(sentai.pipeline.target_fps(0)))
    print("prep_fps_now=%s" % str(sentai.pipeline.prep_fps(0)))
    print("invokes_per_frame_now=%s" % str(sentai.pipeline.invokes_per_frame(1)))
    try:
        print("multi_invoke_mode_now=%s" % str(sentai.pipeline.multi_invoke_mode(0)))
    except Exception as e:
        print("multi_invoke_mode_set_failed", repr(e))
    try:
        print("debug_no_invoke_now=%s" % str(sentai.pipeline.debug_no_invoke(0)))
    except Exception as e:
        print("debug_no_invoke_set_failed", repr(e))

    print("camera_select=%s" % str(sentai.camera.select(CAM_ID)))
    try:
        print("camera_resolution=%s" % str(sentai.camera.set_resolution(WIDTH, HEIGHT)))
    except Exception as e:
        print("camera_resolution_failed", repr(e))
    print("camera_init=%s" % str(sentai.camera.init(1)))
    sentai.rtos.sleep_ms(1000)
    print("camera_frame_count=%s" % str(sentai.camera.frame_count()))


def run_once():
    reset_stats()
    print("pipeline_start=%s" % str(sentai.pipeline.start(0.50, 0.45, 50, False)))
    _, warm_got = drain_warmup(WARMUP)
    t0 = sentai.rtos.micros()
    start_count = int(sentai.pipeline.frame_count())
    last = start_count
    observed = 0
    timeouts = 0
    for _ in range(int(FRAMES)):
        nxt = wait_next(last, 5000)
        if nxt < 0:
            timeouts += 1
            print("frame_timeout last=%d rc=%d" % (int(last), int(nxt)))
            break
        observed += nxt - last
        last = nxt
    t1 = sentai.rtos.micros()
    end_count = int(sentai.pipeline.frame_count())
    wall_us = du32(t0, t1)
    fps = (1000000.0 * float(observed)) / float(wall_us) if wall_us else 0.0
    row = stats_row(warm_got, int(FRAMES), observed, wall_us, fps, timeouts,
                    start_count, end_count)
    out_csv = "%s/run_%02d.csv" % (OUT_DIR, int(RUN_ID))
    print(
        "result run=%d observed=%d wall_us=%d fps=%.6f timeouts=%d "
        "start_count=%d end_count=%d" %
        (int(RUN_ID), int(observed), int(wall_us), float(fps),
         int(timeouts), int(start_count), int(end_count))
    )
    print("pipeline_stop=%s" % str(sentai.pipeline.stop()))
    sentai.fs.write(out_csv, HEADER + row)
    try:
        sentai.fs.sync()
    except Exception:
        pass
    print("csv=%s" % out_csv)


configure()
run_once()
print("=== done ===")

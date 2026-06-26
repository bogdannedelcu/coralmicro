import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
CHUNK_KB = 128
RUN_ID = int(globals().get("_run_id", 0))
PREP_FPS = int(globals().get("_prep_fps", 0))
IPF = int(globals().get("_ipf", 1))
MODE = int(globals().get("_mode", 0))
FRAMES = int(globals().get("_frames", 100))
WARMUP = int(globals().get("_warmup", 5))
CAM_ID = 0
WIDTH = 512
HEIGHT = 512
OUT_DIR = "/diags/s235_b10_multi_patch"

HEADER = (
    "version,run,model,chunk_kb,prep_fps,ipf,multi_invoke_mode,"
    "direct_tensor,target_fps,cam_id,width,height,warmup_frames,"
    "requested_frames,observed_frames,wall_us,frame_fps,invoke_ok,"
    "invoke_fail,invoke_fps,invokes_per_observed_frame,timeouts,"
    "start_count,end_count,latest_seq,latest_inv_ms,latest_total_ms,"
    "latest_memcpy_ms,latest_nms_ms,latest_cam_id,pipeline_processed,"
    "pipeline_dropped,pipeline_diag_fps,infer_ms_delta,infer_last_rc,"
    "prep_frames_delta,prep_sem_wait_ms_delta,prep_cam_grab_ms_delta,"
    "prep_pxp_ms_delta,prep_quant_ms_delta,prep_total_ms_delta,"
    "direct_frames_delta,direct_prep_buf_timeout_delta,"
    "direct_infer_wait_timeout_delta,direct_swap_fail_delta\r\n"
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
        return int(d[k])
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


def wait_next(last, timeout_ms):
    return int(sentai.pipeline.frame_count(int(last), int(timeout_ms)))


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


def subdict(a, b, keys):
    out = {}
    for k in keys:
        out[k] = dget(b, k) - dget(a, k)
    return out


def configure():
    sentai.verbose(1)
    ensure_out()
    print("S235 B10.4 multi-patch wall-clock once")
    print("version=%s" % sentai.version())
    print("run=%d prep_fps=%d ipf=%d mode=%d frames=%d warmup=%d" %
          (RUN_ID, PREP_FPS, IPF, MODE, FRAMES, WARMUP))
    print("model=%s" % MODEL)

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
    print("prep_fps_now=%s" % str(sentai.pipeline.prep_fps(PREP_FPS)))
    print("invokes_per_frame_now=%s" % str(sentai.pipeline.invokes_per_frame(IPF)))
    try:
        print("multi_invoke_mode_now=%s" % str(sentai.pipeline.multi_invoke_mode(MODE)))
    except Exception as e:
        print("multi_invoke_mode_set_failed", repr(e))
    try:
        print("debug_no_invoke_now=%s" % str(sentai.pipeline.debug_no_invoke(0)))
    except Exception as e:
        print("debug_no_invoke_set_failed", repr(e))

    try:
        print("camera_ratio=%s" % str(sentai.camera.ratio(0, 0)))
    except Exception as e:
        print("camera_ratio_failed", repr(e))
    print("camera_select=%s" % str(sentai.camera.select(CAM_ID)))
    try:
        print("camera_resolution=%s" % str(sentai.camera.set_resolution(WIDTH, HEIGHT)))
    except Exception as e:
        print("camera_resolution_failed", repr(e))
    print("camera_init=%s" % str(sentai.camera.init(1)))
    sentai.rtos.sleep_ms(1000)
    print("camera_frame_count=%s" % str(sentai.camera.frame_count()))


def snap():
    try:
        istats = sentai.pipeline.infer_stats()
    except Exception:
        istats = {}
    try:
        prep = sentai.pipeline.prep_stats()
    except Exception:
        prep = {}
    try:
        direct = sentai.pipeline.direct_stats()
    except Exception:
        direct = {}
    return istats, prep, direct


def run_once():
    try:
        sentai.pipeline.prep_reset()
    except Exception:
        pass
    try:
        sentai.pipeline.infer_reset()
    except Exception:
        pass
    print("pipeline_start=%s" % str(sentai.pipeline.start(0.50, 0.45, 50, False)))

    last = int(sentai.pipeline.frame_count())
    warm_got = 0
    for _ in range(WARMUP):
        nxt = wait_next(last, 5000)
        if nxt < 0:
            print("warmup_timeout last=%d rc=%d" % (last, nxt))
            break
        last = nxt
        warm_got += 1

    ist0, prep0, direct0 = snap()
    t0 = sentai.rtos.micros()
    start_count = int(sentai.pipeline.frame_count())
    last = start_count
    observed = 0
    timeouts = 0
    for _ in range(FRAMES):
        nxt = wait_next(last, 7000)
        if nxt < 0:
            timeouts += 1
            print("frame_timeout last=%d rc=%d" % (int(last), int(nxt)))
            break
        observed += nxt - last
        last = nxt
    t1 = sentai.rtos.micros()
    end_count = int(sentai.pipeline.frame_count())
    ist1, prep1, direct1 = snap()

    wall_us = du32(t0, t1)
    frame_fps = (1000000.0 * float(observed)) / float(wall_us) if wall_us else 0.0
    infer_ok = dget(ist1, "ok") - dget(ist0, "ok")
    infer_fail = dget(ist1, "fail") - dget(ist0, "fail")
    infer_ms = dget(ist1, "ms_sum") - dget(ist0, "ms_sum")
    invoke_fps = (1000000.0 * float(infer_ok)) / float(wall_us) if wall_us else 0.0
    inv_per_frame = float(infer_ok) / float(observed) if observed else 0.0
    latest_seq, latest_inv, latest_total, latest_memcpy, latest_nms, latest_cam = sample_latest()
    try:
        pstats = sentai.pipeline.stats()
    except Exception:
        pstats = (0, 0, 0.0)

    prep_delta = subdict(prep0, prep1, (
        "frames", "sem_wait_ms_sum", "cam_grab_ms_sum", "pxp_ms_sum",
        "quant_ms_sum", "total_ms_sum"))
    direct_delta = subdict(direct0, direct1, (
        "frames", "prep_buf_timeout", "infer_wait_timeout", "swap_fail"))

    vals = (
        sentai.version(), RUN_ID, MODEL, CHUNK_KB, PREP_FPS, IPF, MODE, 1,
        0, CAM_ID, WIDTH, HEIGHT, warm_got, FRAMES, observed, wall_us,
        "%.6f" % frame_fps, infer_ok, infer_fail, "%.6f" % invoke_fps,
        "%.6f" % inv_per_frame, timeouts, start_count, end_count,
        latest_seq, latest_inv, latest_total, latest_memcpy, latest_nms,
        latest_cam, int(pstats[0]), int(pstats[1]), "%.6f" % float(pstats[2]),
        infer_ms, dget(ist1, "last_rc"), prep_delta["frames"],
        prep_delta["sem_wait_ms_sum"], prep_delta["cam_grab_ms_sum"],
        prep_delta["pxp_ms_sum"], prep_delta["quant_ms_sum"],
        prep_delta["total_ms_sum"], direct_delta["frames"],
        direct_delta["prep_buf_timeout"], direct_delta["infer_wait_timeout"],
        direct_delta["swap_fail"],
    )
    row = ",".join(csv_escape(v) for v in vals) + "\r\n"
    out_csv = "%s/run_%02d_pf%d_ipf%d_mode%d.csv" % (
        OUT_DIR, RUN_ID, PREP_FPS, IPF, MODE)

    print(
        "result run=%d prep_fps=%d ipf=%d mode=%d observed=%d wall_us=%d "
        "frame_fps=%.6f invoke_ok=%d invoke_fail=%d invoke_fps=%.6f "
        "inv_per_frame=%.3f timeouts=%d" % (
            RUN_ID, PREP_FPS, IPF, MODE, observed, wall_us, frame_fps,
            infer_ok, infer_fail, invoke_fps, inv_per_frame, timeouts))
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

import sentai

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
RUN_ID = int(globals().get("_run_id", 0))
RATIO_A = int(globals().get("_ratio_a", 1))
RATIO_B = int(globals().get("_ratio_b", 1))
DRAIN = int(globals().get("_drain", 1))
FRAMES = int(globals().get("_frames", 100))
WARMUP = int(globals().get("_warmup", 5))
CHUNK_KB = int(globals().get("_chunk_kb", 128))
WIDTH = 512
HEIGHT = 512
OUT_DIR = "/diags/s235_b10_alternation"

HEADER = (
    "version,run,model,chunk_kb,ratio_a,ratio_b,drain,width,height,"
    "warmup_frames,requested_frames,observed_frames,cam0_frames,"
    "cam1_frames,other_cam_frames,wall_us,frame_fps,cam0_fps,cam1_fps,"
    "invoke_ok,invoke_fail,invoke_fps,timeouts,start_count,end_count,"
    "start_camera_frames,end_camera_frames,camera_frame_delta,"
    "pipeline_processed,pipeline_dropped,pipeline_diag_fps,infer_ms_delta,"
    "infer_last_rc,prep_frames_delta,prep_cam_grab_ms_delta,prep_pxp_ms_delta,"
    "prep_quant_ms_delta,prep_total_ms_delta,direct_frames_delta,"
    "direct_prep_buf_timeout_delta,direct_infer_wait_timeout_delta,"
    "direct_swap_fail_delta\r\n"
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


def subdict(a, b, keys):
    out = {}
    for k in keys:
        out[k] = dget(b, k) - dget(a, k)
    return out


def ensure_out():
    try:
        sentai.fs.mkdir("/diags")
    except Exception:
        pass
    try:
        sentai.fs.mkdir(OUT_DIR)
    except Exception:
        pass


def wait_result(after_seq, timeout_ms):
    try:
        return sentai.pipeline.get_ex(int(timeout_ms), int(after_seq))
    except Exception as e:
        print("get_ex_failed", repr(e))
        return None


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


def configure():
    sentai.verbose(1)
    ensure_out()
    print("S235 B10.5 pipeline alternation once")
    print("version=%s" % sentai.version())
    print("run=%d ratio=%d:%d drain=%d frames=%d warmup=%d" %
          (RUN_ID, RATIO_A, RATIO_B, DRAIN, FRAMES, WARMUP))
    try:
        sentai.tpu.desc_cache(0)
    except Exception as e:
        print("desc_cache_disable_failed", repr(e))
    print("chunk_prev=%s chunk_now=%s" %
          (str(sentai.tpu.chunk_size()), str(sentai.tpu.chunk_size(CHUNK_KB * 1024))))
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
    print("switch_drain_prev=%s" % str(sentai.camera.switch_drain(DRAIN)))
    print("camera_ratio_prev=%s" % str(sentai.camera.ratio(RATIO_A, RATIO_B)))
    print("camera_select0=%s" % str(sentai.camera.select(0)))
    try:
        print("camera_resolution=%s" % str(sentai.camera.set_resolution(WIDTH, HEIGHT)))
    except Exception as e:
        print("camera_resolution_failed", repr(e))
    print("camera_init=%s" % str(sentai.camera.init(1)))
    sentai.rtos.sleep_ms(1000)
    print("camera_frame_count=%s" % str(sentai.camera.frame_count()))


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

    last_seq = -1
    warm_got = 0
    for _ in range(WARMUP):
        fr = wait_result(last_seq, 7000)
        if not fr:
            print("warmup_timeout last_seq=%d" % int(last_seq))
            break
        last_seq = int(fr[3])
        warm_got += 1

    ist0, prep0, direct0 = snap()
    t0 = sentai.rtos.micros()
    start_count = int(sentai.pipeline.frame_count())
    start_cam_frames = int(sentai.camera.frame_count())
    observed = 0
    cam0 = 0
    cam1 = 0
    other = 0
    timeouts = 0
    for _ in range(FRAMES):
        fr = wait_result(last_seq, 7000)
        if not fr:
            timeouts += 1
            print("frame_timeout last_seq=%d" % int(last_seq))
            break
        last_seq = int(fr[3])
        observed += 1
        cam_id = int(fr[6])
        if cam_id == 0:
            cam0 += 1
        elif cam_id == 1:
            cam1 += 1
        else:
            other += 1
    t1 = sentai.rtos.micros()
    end_count = int(sentai.pipeline.frame_count())
    end_cam_frames = int(sentai.camera.frame_count())
    ist1, prep1, direct1 = snap()

    wall_us = du32(t0, t1)
    frame_fps = (1000000.0 * float(observed)) / float(wall_us) if wall_us else 0.0
    cam0_fps = (1000000.0 * float(cam0)) / float(wall_us) if wall_us else 0.0
    cam1_fps = (1000000.0 * float(cam1)) / float(wall_us) if wall_us else 0.0
    infer_ok = dget(ist1, "ok") - dget(ist0, "ok")
    infer_fail = dget(ist1, "fail") - dget(ist0, "fail")
    infer_ms = dget(ist1, "ms_sum") - dget(ist0, "ms_sum")
    invoke_fps = (1000000.0 * float(infer_ok)) / float(wall_us) if wall_us else 0.0
    try:
        pstats = sentai.pipeline.stats()
    except Exception:
        pstats = (0, 0, 0.0)
    prep_delta = subdict(prep0, prep1, (
        "frames", "cam_grab_ms_sum", "pxp_ms_sum", "quant_ms_sum",
        "total_ms_sum"))
    direct_delta = subdict(direct0, direct1, (
        "frames", "prep_buf_timeout", "infer_wait_timeout", "swap_fail"))
    vals = (
        sentai.version(), RUN_ID, MODEL, CHUNK_KB, RATIO_A, RATIO_B, DRAIN,
        WIDTH, HEIGHT, warm_got, FRAMES, observed, cam0, cam1, other,
        wall_us, "%.6f" % frame_fps, "%.6f" % cam0_fps, "%.6f" % cam1_fps,
        infer_ok, infer_fail, "%.6f" % invoke_fps, timeouts, start_count,
        end_count, start_cam_frames, end_cam_frames, end_cam_frames - start_cam_frames,
        int(pstats[0]), int(pstats[1]), "%.6f" % float(pstats[2]),
        infer_ms, dget(ist1, "last_rc"), prep_delta["frames"],
        prep_delta["cam_grab_ms_sum"], prep_delta["pxp_ms_sum"],
        prep_delta["quant_ms_sum"], prep_delta["total_ms_sum"],
        direct_delta["frames"], direct_delta["prep_buf_timeout"],
        direct_delta["infer_wait_timeout"], direct_delta["swap_fail"],
    )
    out_csv = "%s/run_%02d_ratio%d_%d_drain%d.csv" % (
        OUT_DIR, RUN_ID, RATIO_A, RATIO_B, DRAIN)
    sentai.fs.write(out_csv, HEADER)
    sentai.fs.append(out_csv, ",".join(csv_escape(v) for v in vals) + "\r\n")
    try:
        print("fs_sync=%s" % str(sentai.fs.sync()))
    except Exception as e:
        print("fs_sync_failed", repr(e))
    print(
        "result run=%d ratio=%d:%d drain=%d observed=%d cam0=%d cam1=%d "
        "wall_us=%d frame_fps=%.6f cam0_fps=%.6f cam1_fps=%.6f "
        "invoke_ok=%d invoke_fail=%d invoke_fps=%.6f timeouts=%d" %
        (RUN_ID, RATIO_A, RATIO_B, DRAIN, observed, cam0, cam1, wall_us,
         frame_fps, cam0_fps, cam1_fps, infer_ok, infer_fail, invoke_fps,
         timeouts)
    )
    print("csv=%s" % out_csv)


configure()
run_once()
print("=== done ===")

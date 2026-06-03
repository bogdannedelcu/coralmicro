import sentai


MODEL = "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
FRAME = "/images/shift/frame_000.bmp"
FRAME_DIR = "/images/shift"
PLAY_FPS = 10


def _mark(msg):
    sentai.fs.append("/pipeline_5s_trace.txt", msg + "\n")


def _compact_dets(dets):
    out = []
    for d in dets:
        conf = d[4]
        conf_i = int(conf * 1000) if isinstance(conf, float) and conf <= 1.0 else int(conf)
        out.append((
            int(d[0]), int(d[1]), int(d[2]), int(d[3]),
            conf_i, int(d[5])))
    return tuple(out)


def _flow_tuple():
    r = sentai.flow.read_tuple()
    return (
        bool(r[0]), int(r[1]), int(r[2]), int(r[3]),
        int(r[4]), int(r[5]), int(r[6]),
    )


def run(duration_ms=5000, play_count=80, use_replay=False, use_flow=True,
        direct_tensor=1):
    _mark("run_start")
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    _mark("before_tpu_load")
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    _mark("after_tpu_load")
    print("TPU_READY", sentai.tpu.ready())
    if use_flow:
        _mark("before_flow_start")
        print("FLOW_START", sentai.flow.start(-1))
        _mark("after_flow_start")
    else:
        print("FLOW_START", "SKIP")
        _mark("flow_skip")
    print("CAM_SELECT", sentai.camera.select(-1, FRAME))
    _mark("after_cam_select")
    print("PIPE_DIRECT_TENSOR_PREV", sentai.pipeline.direct_tensor(int(direct_tensor)))
    print("PIPE_TARGET_FPS", sentai.pipeline.target_fps(0))
    print("PIPE_PREP_FPS", sentai.pipeline.prep_fps(0))

    initial_count = int(sentai.pipeline.frame_count())
    initial_flow = _flow_tuple()
    _mark("before_pipe_start")
    print("PIPE_START", sentai.pipeline.start(0.10, 0.45, 20, False))
    _mark("after_pipe_start")
    if use_replay:
        play_rc = sentai.camera.replay(PLAY_FPS, int(play_count))
        print("CAM_REPLAY", play_rc, "FRAME", FRAME, "FPS", PLAY_FPS,
              "COUNT", int(play_count))
        _mark("after_cam_replay")
    else:
        play_rc = sentai.camera.play(FRAME_DIR, PLAY_FPS, int(play_count))
        print("CAM_PLAY", play_rc, "DIR", FRAME_DIR, "FPS", PLAY_FPS,
              "COUNT", int(play_count))
        _mark("after_cam_play")

    _mark("before_sleep")
    sentai.rtos.sleep_ms(int(duration_ms))
    _mark("after_sleep")

    mid_count = int(sentai.pipeline.frame_count())
    mid_flow = _flow_tuple()
    print("COUNTS_BEFORE_STOP", initial_count, mid_count)
    print("FLOW_BEFORE_STOP", mid_flow)

    print("CAM_PLAY_STOP", sentai.camera.play_stop())
    _mark("after_cam_stop")
    print("PIPE_STOP", sentai.pipeline.stop())
    _mark("after_pipe_stop")
    if use_flow:
        print("FLOW_STOP", sentai.flow.stop())
        _mark("after_flow_stop")
    else:
        print("FLOW_STOP", "SKIP")
        _mark("flow_stop_skip")
    print("OVERLAY_WRITTEN", sentai.pipeline.save("/images/detected_overlay_last.bmp"))
    _mark("after_overlay")

    latest = sentai.pipeline.get_ex(0, 0)
    inv = int(latest[1]) if latest else -1
    dets = _compact_dets(latest[0]) if latest else tuple()
    pipe_seq = int(latest[3]) if latest else -1
    result = {
        "initial_count": initial_count,
        "mid_count": mid_count,
        "delta_count": mid_count - initial_count,
        "initial_flow": initial_flow,
        "mid_flow": mid_flow,
        "flow_pub_stats": sentai.flow.pub_stats(),
        "mode": "replay" if use_replay else "play",
        "use_flow": bool(use_flow),
        "direct_tensor": int(direct_tensor),
        "latest_inv_ms": inv,
        "latest_seq": pipe_seq,
        "latest_dets": dets,
    }
    print("PIPELINE_5S_RESULT", repr(result))
    print("PIPELINE_5S_RESULT_WRITTEN",
          sentai.fs.write("/pipeline_5s_results.txt", repr(result)))
    return 0

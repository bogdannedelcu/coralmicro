import sentai


MODEL = "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
FRAME_DIR = "/images/shift"
PLAY_FPS = 10


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


def run(duration_ms=5000, play_count=80):
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("FLOW_START", sentai.flow.start(-1))
    print("PIPE_DIRECT_TENSOR_PREV", sentai.pipeline.direct_tensor(1))
    print("PIPE_TARGET_FPS", sentai.pipeline.target_fps(0))
    print("PIPE_PREP_FPS", sentai.pipeline.prep_fps(0))

    initial_count = int(sentai.pipeline.frame_count())
    initial_flow = _flow_tuple()
    print("PIPE_START", sentai.pipeline.start(0.10, 0.45, 20, False))
    play_rc = sentai.camera.play(FRAME_DIR, PLAY_FPS, int(play_count))
    print("CAM_PLAY", play_rc, "DIR", FRAME_DIR, "FPS", PLAY_FPS,
          "COUNT", int(play_count))

    sentai.rtos.sleep_ms(int(duration_ms))

    mid_count = int(sentai.pipeline.frame_count())
    mid_flow = _flow_tuple()
    print("COUNTS_BEFORE_STOP", initial_count, mid_count)
    print("FLOW_BEFORE_STOP", mid_flow)

    print("CAM_PLAY_STOP", sentai.camera.play_stop())
    print("PIPE_STOP", sentai.pipeline.stop())
    print("FLOW_STOP", sentai.flow.stop())
    print("OVERLAY_WRITTEN", sentai.pipeline.save("/images/detected_overlay_last.bmp"))

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
        "latest_inv_ms": inv,
        "latest_seq": pipe_seq,
        "latest_dets": dets,
    }
    print("PIPELINE_5S_RESULT", repr(result))
    print("PIPELINE_5S_RESULT_WRITTEN",
          sentai.fs.write("/pipeline_5s_results.txt", repr(result)))
    return 0

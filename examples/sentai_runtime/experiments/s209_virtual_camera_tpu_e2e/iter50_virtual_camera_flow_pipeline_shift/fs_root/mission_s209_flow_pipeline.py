import sentai


MODEL = "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
N_FRAMES = 16
STEP_MS = 125


def _path(i):
    return "/images/cat_shift_%02d.bmp" % i


def _flow_tuple():
    r = sentai.flow.read()
    return (
        bool(r["alive"]),
        int(r["state"]),
        int(r["dx"]),
        int(r["dy"]),
        int(r["confidence"]),
        int(r["frame_seq"]),
        int(r["cam_id"]),
    )


def _compact_frame(frame):
    if frame is None:
        return None
    dets, invoke_ms, total_ms, seq, memcpy_ms, nms_ms, cam_id = frame
    compact = []
    for d in dets:
        compact.append((
            int(d[0]), int(d[1]), int(d[2]), int(d[3]),
            int(d[4] * 1000.0 + 0.5), int(d[5])))
    return (
        tuple(compact),
        int(invoke_ms),
        int(total_ms),
        int(seq),
        int(memcpy_ms),
        int(nms_ms),
        int(cam_id),
    )


def run():
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMG0_SIZE", sentai.fs.size(_path(0)))
    print("IMGN_SIZE", sentai.fs.size(_path(N_FRAMES - 1)))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("CAM_SELECT0", sentai.camera.select(-1, _path(0)))
    print("FLOW_START", sentai.flow.start(-1))
    print("PIPE_TARGET_FPS", sentai.pipeline.target_fps(12))
    print("PIPE_PREP_FPS", sentai.pipeline.prep_fps(12))
    print("PIPE_START", sentai.pipeline.start(0.1, 0.45, 50, False))

    records = []
    for i in range(N_FRAMES):
        rc = sentai.camera.select(-1, _path(i))
        sentai.rtos.sleep_ms(STEP_MS)
        frame = sentai.pipeline.get_ex(20)
        records.append((i, rc, _flow_tuple(), _compact_frame(frame)))

    sentai.rtos.sleep_ms(200)
    tail = sentai.pipeline.get_ex(100)
    print("PIPE_STOP", sentai.pipeline.stop())
    print("FLOW_STOP", sentai.flow.stop())

    result = {
        "records": records,
        "tail": _compact_frame(tail),
        "pipeline_stats": sentai.pipeline.stats(),
        "pipeline_infer_stats": sentai.pipeline.infer_stats(),
        "flow_pub_stats": sentai.flow.pub_stats(),
    }
    print("FLOW_PIPE_RESULTS", repr(result))
    print("FLOW_PIPE_RESULTS_WRITTEN",
          sentai.fs.write("/flow_pipeline_results.txt", repr(result)))
    return 0

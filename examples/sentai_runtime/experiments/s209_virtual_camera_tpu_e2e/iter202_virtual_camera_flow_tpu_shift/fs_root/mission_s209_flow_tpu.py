import sentai


MODEL = "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
N_FRAMES = 16


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


def _wait_flow_after(prev_seq, max_polls=20000):
    last = _flow_tuple()
    for _ in range(max_polls):
        last = _flow_tuple()
        if last[5] != prev_seq:
            return last
    return last


def _compact_dets(dets):
    out = []
    for d in dets:
        out.append((
            int(d[0]), int(d[1]), int(d[2]), int(d[3]),
            int(d[4]), int(d[5])))
    return tuple(out)


def run(n_frames=N_FRAMES):
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMG0_SIZE", sentai.fs.size(_path(0)))
    print("IMGN_SIZE", sentai.fs.size(_path(n_frames - 1)))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("FLOW_START", sentai.flow.start(-1))

    records = []
    prev_flow_seq = _flow_tuple()[5]
    for i in range(n_frames):
        path = _path(i)
        li = sentai.tpu.load_image(path)
        flow = _wait_flow_after(prev_flow_seq)
        prev_flow_seq = flow[5]
        inv = sentai.tpu.invoke()
        dets = _compact_dets(sentai.pipeline.detections(100))
        records.append((i, 0, flow, li, inv, dets))

    print("FLOW_STOP", sentai.flow.stop())
    print("OVERLAY_WRITTEN", sentai.pipeline.save("/images/detected_overlay_last.bmp"))
    result = {
        "records": records,
        "flow_pub_stats": sentai.flow.pub_stats(),
    }
    print("FLOW_TPU_RESULTS", repr(result))
    print("FLOW_TPU_RESULTS_WRITTEN",
          sentai.fs.write("/flow_tpu_results.txt", repr(result)))
    return 0

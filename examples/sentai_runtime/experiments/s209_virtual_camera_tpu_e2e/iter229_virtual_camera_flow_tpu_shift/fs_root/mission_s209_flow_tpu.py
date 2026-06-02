import sentai


MODEL = "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
N_FRAMES = 16
_records = []
_started = False


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


def _wait_flow_after(prev_seq, max_polls=2000):
    last = _flow_tuple()
    for _ in range(max_polls):
        last = _flow_tuple()
        if last[5] != prev_seq:
            return last
    return last


def _wait_pipeline(max_polls=20000):
    for _ in range(max_polls):
        pipe = sentai.pipeline.get_ex(0)
        if pipe:
            return pipe
        sentai.rtos.repl_kick()
    print("PIPE_TIMEOUT")
    return None


def _compact_dets(dets):
    out = []
    for d in dets:
        conf = d[4]
        conf_i = int(conf * 1000) if isinstance(conf, float) and conf <= 1.0 else int(conf)
        out.append((
            int(d[0]), int(d[1]), int(d[2]), int(d[3]),
            conf_i, int(d[5])))
    return tuple(out)


def setup(n_frames=N_FRAMES):
    global _records, _started
    _records = []
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMG0_SIZE", sentai.fs.size(_path(0)))
    print("IMGN_SIZE", sentai.fs.size(_path(n_frames - 1)))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("FLOW_START", sentai.flow.start(-1))
    print("FRAME", 0, "SELECT_BEGIN")
    first_cam_rc = sentai.camera.select(-1, _path(0))
    print("FRAME", 0, "SELECT_RC", first_cam_rc)
    print("PIPE_START", sentai.pipeline.start(0.10, 0.45, 20))
    _started = True
    return first_cam_rc


def select_frame(i):
    print("FRAME", i, "SELECT_BEGIN")
    cam_rc = sentai.camera.select(-1, _path(i))
    print("FRAME", i, "SELECT_RC", cam_rc)
    return cam_rc


def collect_frame(i, cam_rc=0):
    global _records
    pipe = sentai.pipeline.get_ex(0)
    print("FRAME", i, "PIPE_DONE", pipe is not None)
    flow = _flow_tuple()
    print("FRAME", i, "FLOW", flow)
    inv = int(pipe[1]) if pipe else -1
    dets = _compact_dets(pipe[0]) if pipe else tuple()
    rec = (i, cam_rc, flow, 0 if pipe else -1, inv, dets)
    _records.append(rec)
    print("FRAME_RECORD", repr(rec))
    return rec


def finish():
    print("PIPE_STOP", sentai.pipeline.stop())
    print("FLOW_STOP", sentai.flow.stop())
    print("OVERLAY_WRITTEN", sentai.pipeline.save("/images/detected_overlay_last.bmp"))
    result = {
        "records": _records,
        "flow_pub_stats": sentai.flow.pub_stats(),
    }
    print("FLOW_TPU_RESULTS", repr(result))
    print("FLOW_TPU_RESULTS_WRITTEN",
          sentai.fs.write("/flow_tpu_results.txt", repr(result)))
    return result


def run(n_frames=N_FRAMES):
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMG0_SIZE", sentai.fs.size(_path(0)))
    print("IMGN_SIZE", sentai.fs.size(_path(n_frames - 1)))
    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())
    print("FLOW_START", sentai.flow.start(-1))
    print("FRAME", 0, "SELECT_BEGIN")
    first_cam_rc = sentai.camera.select(-1, _path(0))
    print("FRAME", 0, "SELECT_RC", first_cam_rc)
    print("PIPE_START", sentai.pipeline.start(0.10, 0.45, 20))

    records = []
    for i in range(n_frames):
        path = _path(i)
        if i == 0:
            cam_rc = first_cam_rc
        else:
            print("FRAME", i, "SELECT_BEGIN")
            cam_rc = sentai.camera.select(-1, path)
            print("FRAME", i, "SELECT_RC", cam_rc)
        print("FRAME", i, "PIPE_GET_BEGIN")
        pipe = _wait_pipeline()
        print("FRAME", i, "PIPE_DONE", pipe is not None)
        sentai.rtos.repl_kick()
        flow = _flow_tuple()
        print("FRAME", i, "FLOW", flow)
        inv = int(pipe[1]) if pipe else -1
        dets = _compact_dets(pipe[0]) if pipe else tuple()
        records.append((i, cam_rc, flow, 0 if pipe else -1, inv, dets))

    print("PIPE_STOP", sentai.pipeline.stop())
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

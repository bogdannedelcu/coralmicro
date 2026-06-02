import sentai


MODEL = "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
FRAME_DIR = "/images/shift"
N_FRAMES = 16
PIPE_TIMEOUT_MS = 3000
FLOW_TIMEOUT_MS = 1000
PLAY_FPS = 10


def _path(i):
    return "%s/frame_%03d.bmp" % (FRAME_DIR, i)


def _ticks_ms():
    try:
        return int(sentai.rtos.ticks_ms())
    except Exception:
        return 0


def _elapsed_ms(start):
    return (_ticks_ms() - start) & 0x3fffffff


def _yield_rtos():
    pass


def _flow_tuple():
    r = sentai.flow.read_tuple()
    return (
        bool(r[0]), int(r[1]), int(r[2]), int(r[3]),
        int(r[4]), int(r[5]), int(r[6]),
    )


def _wait_flow_after(prev_seq, timeout_ms=FLOW_TIMEOUT_MS):
    del timeout_ms
    last = _flow_tuple()
    if int(last[5]) != int(prev_seq):
        return last
    print("FLOW_TIMEOUT_AFTER", prev_seq, "LAST", last)
    return last


def _wait_pipeline_after(prev_seq, timeout_ms=PIPE_TIMEOUT_MS):
    start = _ticks_ms()
    spins = 0
    while _elapsed_ms(start) < timeout_ms:
        pipe = sentai.pipeline.get_ex(0, prev_seq)
        if pipe:
            return pipe
        spins += 1
        sentai.rtos.repl_kick()
    print("PIPE_TIMEOUT_AFTER", prev_seq, "LAST", None)
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


def _record_frame(i, cam_rc, prev_pipe_seq, prev_flow_seq):
    print("FRAME", i, "PIPE_GET_BEGIN")
    pipe = _wait_pipeline_after(prev_pipe_seq)
    print("FRAME", i, "PIPE_DONE", pipe is not None)
    if pipe:
        prev_pipe_seq = int(pipe[3])

    flow = _wait_flow_after(prev_flow_seq)
    prev_flow_seq = int(flow[5])
    print("FRAME", i, "FLOW", flow)

    inv = int(pipe[1]) if pipe else -1
    dets = _compact_dets(pipe[0]) if pipe else tuple()
    rec = (i, int(cam_rc), flow, 0 if pipe else -1, inv, dets)
    print("FRAME_RECORD", repr(rec))
    return rec, prev_pipe_seq, prev_flow_seq


def run(n_frames=N_FRAMES):
    print("MODEL_SIZE", sentai.fs.size(MODEL))
    print("IMG0_SIZE", sentai.fs.size(_path(0)))
    print("IMGN_SIZE", sentai.fs.size(_path(n_frames - 1)))

    print("TPU_LOAD", sentai.tpu.load(MODEL))
    print("TPU_READY", sentai.tpu.ready())

    print("FLOW_START", sentai.flow.start(-1))
    print("PIPE_DIRECT_TENSOR_PREV", sentai.pipeline.direct_tensor(1))
    print("PIPE_TARGET_FPS", sentai.pipeline.target_fps(0))
    print("PIPE_PREP_FPS", sentai.pipeline.prep_fps(0))

    records = []
    prev_pipe_seq = 0
    prev_flow_seq = int(_flow_tuple()[5])

    print("PIPE_START", sentai.pipeline.start(0.10, 0.45, 20, False))
    play_count = n_frames + 1
    play_rc = sentai.camera.play(FRAME_DIR, PLAY_FPS, play_count)
    print("CAM_PLAY", play_rc, "DIR", FRAME_DIR, "FPS", PLAY_FPS,
          "COUNT", play_count)

    try:
        for i in range(n_frames):
            rec, prev_pipe_seq, prev_flow_seq = _record_frame(
                i, 0 if play_rc > 0 else play_rc, prev_pipe_seq, prev_flow_seq)
            records.append(rec)
    finally:
        print("OVERLAY_WRITTEN", sentai.pipeline.save("/images/detected_overlay_last.bmp"))
        print("CAM_PLAY_STOP", sentai.camera.play_stop())
        print("PIPE_STOP", sentai.pipeline.stop())
        print("FLOW_STOP", sentai.flow.stop())

    result = {
        "records": records,
        "flow_pub_stats": sentai.flow.pub_stats(),
    }
    print("FLOW_TPU_RESULTS", repr(result))
    print("FLOW_TPU_RESULTS_WRITTEN",
          sentai.fs.write("/flow_tpu_results.txt", repr(result)))
    return 0

import sentai


FRAME = "/images/shift/frame_000.bmp"
FRAME_DIR = "/images/shift"
PLAY_FPS = 10
SLOT_FLOW_GRAY_80X60 = 4


def _stats_compact(st):
    return {
        "frames_total": int(st["frames_total"]),
        "frames_with_aux": int(st["frames_with_aux"]),
        "producer_overruns": int(st["producer_overruns"]),
        "slot_refcount": tuple(st["slot_refcount"]),
        "slot_seq": tuple(st["slot_seq"]),
    }


def _pipe_stats_compact(st):
    return {
        "frames": int(st["frames"]),
        "cam_grab_ms_sum": int(st["cam_grab_ms_sum"]),
        "pxp_ms_sum": int(st["pxp_ms_sum"]),
        "total_ms_sum": int(st["total_ms_sum"]),
    }


def run(duration_ms=3000, play_count=40, use_replay=True, use_flow=True):
    print("CAM_PREP_RESET", sentai.camera.prep_reset())
    print("PIPE_PREP_RESET", sentai.pipeline.prep_reset())
    print("PREP_ENABLE_FLOW", sentai.camera.prep_enable(SLOT_FLOW_GRAY_80X60))
    flow_start_rc = None
    if use_flow:
        flow_start_rc = sentai.flow.start(-1)
        print("FLOW_START", flow_start_rc)
    else:
        print("FLOW_START", "SKIP")
    print("CAM_SELECT", sentai.camera.select(-1, FRAME))
    flow_before = sentai.flow.read_tuple()
    before = _stats_compact(sentai.camera.prep_stats())
    pipe_before = _pipe_stats_compact(sentai.pipeline.prep_stats())
    print("PREP_BEFORE", repr(before), "PIPE", repr(pipe_before))
    print("PIPE_PREP_START", sentai.pipeline.prep_start())

    if use_replay:
        play_rc = sentai.camera.replay(PLAY_FPS, int(play_count))
        print("CAM_REPLAY", play_rc, "FRAME", FRAME, "FPS", PLAY_FPS,
              "COUNT", int(play_count))
    else:
        play_rc = sentai.camera.play(FRAME_DIR, PLAY_FPS, int(play_count))
        print("CAM_PLAY", play_rc, "DIR", FRAME_DIR, "FPS", PLAY_FPS,
              "COUNT", int(play_count))

    sentai.rtos.sleep_ms(int(duration_ms))

    flow_mid = sentai.flow.read_tuple()
    prep_stop_rc = sentai.pipeline.prep_stop()
    print("PIPE_PREP_STOP", prep_stop_rc)
    flow_stop_rc = None
    if use_flow:
        flow_stop_rc = sentai.flow.stop()
        print("FLOW_STOP", flow_stop_rc)
    after = _stats_compact(sentai.camera.prep_stats())
    pipe_after = _pipe_stats_compact(sentai.pipeline.prep_stats())
    print("PREP_AFTER", repr(after), "PIPE", repr(pipe_after))
    print("CAM_PLAY_STOP", sentai.camera.play_stop())
    print("PREP_DISABLE_FLOW", sentai.camera.prep_disable(SLOT_FLOW_GRAY_80X60))

    result = {
        "before": before,
        "after": after,
        "pipe_before": pipe_before,
        "pipe_after": pipe_after,
        "delta_frames": pipe_after["frames"] - pipe_before["frames"],
        "delta_aux": after["frames_with_aux"] - before["frames_with_aux"],
        "prep_stop_rc": prep_stop_rc,
        "flow_start_rc": flow_start_rc,
        "flow_stop_rc": flow_stop_rc,
        "flow_before": flow_before,
        "flow_mid": flow_mid,
        "flow_pub_stats": sentai.flow.pub_stats(),
        "use_flow": use_flow,
        "mode": "replay" if use_replay else "play",
        "flow_slot_seq_delta": (
            after["slot_seq"][SLOT_FLOW_GRAY_80X60] -
            before["slot_seq"][SLOT_FLOW_GRAY_80X60]
        ),
        "flow_refcount_after": after["slot_refcount"][SLOT_FLOW_GRAY_80X60],
    }
    print("PREP_ONLY_RESULT", repr(result))
    print("PREP_ONLY_RESULT_WRITTEN",
          sentai.fs.write("/prep_only_results.txt", repr(result)))
    return 0

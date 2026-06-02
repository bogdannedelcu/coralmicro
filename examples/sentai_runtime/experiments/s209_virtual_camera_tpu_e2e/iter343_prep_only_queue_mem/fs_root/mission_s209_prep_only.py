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


def run(duration_ms=3000, play_count=40, use_replay=True):
    print("PREP_RESET", sentai.camera.prep_reset())
    print("PREP_ENABLE_FLOW", sentai.camera.prep_enable(SLOT_FLOW_GRAY_80X60))
    print("CAM_SELECT", sentai.camera.select(-1, FRAME))
    before = _stats_compact(sentai.camera.prep_stats())
    print("PREP_BEFORE", repr(before))

    if use_replay:
        play_rc = sentai.camera.replay(PLAY_FPS, int(play_count))
        print("CAM_REPLAY", play_rc, "FRAME", FRAME, "FPS", PLAY_FPS,
              "COUNT", int(play_count))
    else:
        play_rc = sentai.camera.play(FRAME_DIR, PLAY_FPS, int(play_count))
        print("CAM_PLAY", play_rc, "DIR", FRAME_DIR, "FPS", PLAY_FPS,
              "COUNT", int(play_count))

    sentai.rtos.sleep_ms(int(duration_ms))

    after = _stats_compact(sentai.camera.prep_stats())
    print("PREP_AFTER", repr(after))
    print("CAM_PLAY_STOP", sentai.camera.play_stop())
    print("PREP_DISABLE_FLOW", sentai.camera.prep_disable(SLOT_FLOW_GRAY_80X60))

    result = {
        "before": before,
        "after": after,
        "delta_frames": after["frames_total"] - before["frames_total"],
        "delta_aux": after["frames_with_aux"] - before["frames_with_aux"],
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

import sentai


FRAME_DIR = "/images/shift"
PLAY_FPS = 10


def run(duration_ms=3000, play_count=30):
    before = int(sentai.camera.frame_count())
    print("CAM_BEFORE", before)
    print("CAM_PLAY", sentai.camera.play(FRAME_DIR, PLAY_FPS, int(play_count)))
    sentai.rtos.sleep_ms(int(duration_ms))
    mid = int(sentai.camera.frame_count())
    playing_mid = bool(sentai.camera.playing())
    print("CAM_MID", mid, "PLAYING", playing_mid)
    print("CAM_PLAY_STOP", sentai.camera.play_stop())
    after_stop = int(sentai.camera.frame_count())
    result = {
        "before": before,
        "mid": mid,
        "after_stop": after_stop,
        "delta": mid - before,
        "playing_mid": playing_mid,
        "play_count": int(play_count),
    }
    print("CAMERA_PLAY_ONLY_RESULT", repr(result))
    print("CAMERA_PLAY_ONLY_RESULT_WRITTEN",
          sentai.fs.write("/camera_play_only_results.txt", repr(result)))
    return 0

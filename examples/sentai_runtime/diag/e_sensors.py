# diag/e_sensors.py — E8: IMU, E9: microphone

import sentai
from diag._util import stats, time_call, save_csv, snapshot_meta, _print_stats
from diag._session import _save_path, _record


def e8_imu(repetitions=100, save=True):
    """Measure sentai.imu.read(), degrees(), radians() latency."""
    print("[E8] IMU read — %d reps" % repetitions)

    sentai.imu.init()

    read_times = []
    deg_times = []
    rad_times = []

    for i in range(repetitions):
        elapsed, _ = time_call(sentai.imu.read);    read_times.append(elapsed)
        elapsed, _ = time_call(sentai.imu.degrees); deg_times.append(elapsed)
        elapsed, _ = time_call(sentai.imu.radians); rad_times.append(elapsed)

    st_read = stats(read_times)
    st_deg  = stats(deg_times)
    st_rad  = stats(rad_times)
    meta = snapshot_meta("E8_imu")

    print("[E8] Done.")
    _print_stats("read()",    st_read)
    _print_stats("degrees()", st_deg)
    _print_stats("radians()", st_rad)

    result = {"experiment": "E8_imu", "meta": meta,
              "params": {"repetitions": repetitions},
              "samples": {"read_ms": read_times, "degrees_ms": deg_times,
                          "radians_ms": rad_times},
              "summary": {"read": st_read, "degrees": st_deg, "radians": st_rad}}

    if save:
        path = _save_path("e8_imu")
        save_csv(path, ["run", "read_ms", "degrees_ms", "radians_ms"],
                 [(i, read_times[i], deg_times[i], rad_times[i])
                  for i in range(repetitions)])
        _record("e8_imu", path,
                "read=%.1f deg=%.1f rad=%.1f ms" % (
                    st_read["mean"], st_deg["mean"], st_rad["mean"]))
        print("  Saved: %s" % path)

    return result


def e9_mic(seconds=2, repetitions=5, save=True):
    """Measure mic.start(), level(), save_mp3() latency."""
    print("[E9] Microphone — %ds recording, %d reps" % (seconds, repetitions))

    start_times = []
    level_times = []
    save_times  = []
    mp3_sizes   = []

    for i in range(repetitions):
        elapsed, _ = time_call(sentai.mic.start, seconds)
        start_times.append(elapsed)

        sentai.rtos.sleep_ms(seconds * 1000 + 200)
        sentai.mic.stop()

        elapsed, _ = time_call(sentai.mic.level)
        level_times.append(elapsed)

        elapsed, fname = time_call(sentai.mic.save_mp3)
        save_times.append(elapsed)
        if fname:
            sz = sentai.fs.size(fname)
            mp3_sizes.append(sz if sz > 0 else 0)
        else:
            mp3_sizes.append(0)

        print("  run %d: start=%dms save=%dms file=%s" % (
            i, start_times[-1], save_times[-1], fname or "None"))

    st_start = stats(start_times)
    st_level = stats(level_times)
    st_save  = stats(save_times)
    meta = snapshot_meta("E9_mic", seconds=seconds)

    print("[E9] Done.")
    _print_stats("start()",    st_start)
    _print_stats("level()",    st_level)
    _print_stats("save_mp3()", st_save)

    result = {"experiment": "E9_mic", "meta": meta,
              "params": {"seconds": seconds, "repetitions": repetitions},
              "samples": {"start_ms": start_times, "level_ms": level_times,
                          "save_ms": save_times, "mp3_bytes": mp3_sizes},
              "summary": {"start": st_start, "level": st_level, "save": st_save}}

    if save:
        path = _save_path("e9_mic_%ds" % seconds)
        save_csv(path, ["run", "start_ms", "level_ms", "save_ms", "mp3_bytes"],
                 [(i, start_times[i], level_times[i], save_times[i], mp3_sizes[i])
                  for i in range(repetitions)])
        _record("e9_mic", path,
                "%ds start=%.1f save=%.1f ms" % (
                    seconds, st_start["mean"], st_save["mean"]))
        print("  Saved: %s" % path)

    return result

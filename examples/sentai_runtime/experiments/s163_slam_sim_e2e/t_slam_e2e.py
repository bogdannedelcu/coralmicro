# s163 — SlamTask end-to-end SIM smoke (OP-S10-W11-T4).
#
# Validates the SIM SLOT_RGB_64 producer in camera_bridge_recv.
# Procedure:
#   1. start_slam() — enable SLOT_RGB_64 + spawn SlamTask.
#   2. Sleep ~3 s while inject_frames.py (run in parallel from the
#      shell runner) pushes synthetic 640x480 RGB frames through
#      /tmp/sentai_cam.sock.
#   3. Read slam_stats — expect frames_processed > 0.
#   4. Read slam_current — expect result_seq > 0.
#   5. stop_slam() — clean teardown.
#
# PASS criteria:
#   - slam_stats.frames_processed >= 5
#   - slam_current.result_seq >= 5
#   - slam_current.t_compute_us > 0  (real compute happened)
#   - is_running flips 0 after stop.

import sentai


def _g(name, ok):
    tag = "PASS" if ok else "FAIL"
    print("[{}] {}".format(tag, name))
    return 1 if ok else 0


def run():
    fails = 0
    sentai.verbose(0)

    # 1. start
    rc = sentai.places.start_slam()
    fails += 1 - _g("start_slam returns 0 (rc={})".format(rc), rc == 0)

    # 2. wait while bridge pushes frames (driver runs in parallel)
    # 3 s @ 20 Hz inject = ~60 attempted frames.  Bridge's duplicate
    # detector drops most identical ones, but synthetic frames vary
    # per seq so we should get ~all of them through.
    sentai.rtos.sleep_ms(3000)

    # 3. frames_processed grew
    s = sentai.places.slam_stats()
    fails += 1 - _g(
        "frames_processed >= 5 (got {}; dropped={}, last_us={})".format(
            s.get("frames_processed"), s.get("frames_dropped"),
            s.get("last_compute_us")),
        s.get("frames_processed", 0) >= 5,
    )

    # 4. result_seq advanced + compute time non-zero
    c = sentai.places.slam_current()
    fails += 1 - _g(
        "result_seq >= 5 (got {})".format(c.get("result_seq")),
        c.get("result_seq", 0) >= 5,
    )
    fails += 1 - _g(
        "t_compute_us > 0 (got {})".format(c.get("t_compute_us")),
        c.get("t_compute_us", 0) > 0,
    )

    # 5. stop
    rc2 = sentai.places.stop_slam()
    s2 = sentai.places.slam_stats()
    fails += 1 - _g(
        "stop clean (rc={}, running={})".format(rc2, s2.get("is_running")),
        rc2 == 0 and s2.get("is_running") == 0,
    )

    verdict = "PASS" if fails == 0 else "FAIL ({} gates)".format(fails)
    print("[VERDICT] s163 slam_task SIM e2e: {}".format(verdict))
    return fails


_FAILS = run()

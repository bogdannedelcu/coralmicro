# s162 — SlamTask lifecycle smoke test (OP-S10-W11-T3).
#
# Validates lifecycle without a real frame producer:
#   - start_slam() → 0
#   - is_running == 1 in slam_stats()
#   - frames_processed == 0 (no SLOT_RGB_64 producer wired in SIM yet —
#     Phase 1d wires camera_bridge_recv; Phase 1c only checks
#     start/stop/sem/task plumbing).
#   - Wait ~600 ms (one sem timeout cycle) → still alive.
#   - stop_slam() → 0; is_running == 0.
#
# PASS criteria: 5/5 gates green.

import sentai


def _g(name, ok):
    tag = "PASS" if ok else "FAIL"
    print("[{}] {}".format(tag, name))
    return 1 if ok else 0


def run():
    fails = 0
    sentai.verbose(0)

    # Gate 1 — clean start.
    rc = sentai.places.start_slam()
    fails += 1 - _g("start_slam returns 0 (got {})".format(rc), rc == 0)

    # Gate 2 — task reports running.
    s = sentai.places.slam_stats()
    fails += 1 - _g("slam_stats.is_running == 1 (got {})".format(s.get("is_running")),
                    s.get("is_running") == 1)

    # Gate 3 — current() is shaped correctly even before any frame.
    c = sentai.places.slam_current()
    needed = ("match_id", "score_pct", "l1_dist", "frame_seq",
              "result_seq", "t_compute_us")
    ok = all(k in c for k in needed)
    fails += 1 - _g("slam_current has all 6 keys (got {})".format(sorted(c.keys())), ok)

    # Gate 4 — task survives the 500 ms sem timeout cycle (SIM has no
    # SLOT_RGB_64 producer yet, so the task should idle without dying).
    sentai.rtos.sleep_ms(700)
    s2 = sentai.places.slam_stats()
    fails += 1 - _g(
        "task alive after 700 ms idle (running={}, processed={}, dropped={})".format(
            s2.get("is_running"), s2.get("frames_processed"), s2.get("frames_dropped")),
        s2.get("is_running") == 1 and s2.get("frames_processed") == 0,
    )

    # Gate 5 — clean stop.
    rc2 = sentai.places.stop_slam()
    s3 = sentai.places.slam_stats()
    fails += 1 - _g(
        "stop_slam returns 0 and is_running flips to 0 (rc={}, running={})".format(
            rc2, s3.get("is_running")),
        rc2 == 0 and s3.get("is_running") == 0,
    )

    verdict = "PASS" if fails == 0 else "FAIL ({} gates)".format(fails)
    print("[VERDICT] s162 slam_task lifecycle smoke: {}".format(verdict))
    return fails


_FAILS = run()

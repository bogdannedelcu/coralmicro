# mission_s152.py — HexPatrol on the sentai.servo paradigm.
#
# Equivalent to mission_s149, but driving cf2 SITL via sentai.servo.*
# instead of sentai.crazy.* + crtp_log.py.  Demonstrates Tasks #44/#45/#47:
#   - pose feedback in C (sentai.crazy.pose_*) wrapped under servo.pose()
#   - backend-agnostic action layer (sentai.servo.{init,arm,takeoff,go_to,land})
#
# Stage required upstream:
#   build-sim/sentai_fs_root/hex_helpers.py  (from s142_hex_descriptor_patrol)
# No crtp_log.py — sentai.servo subscribes pose automatically at init().

import gc
import sentai
import hex_helpers      # synth image + capture_and_store + self_query

JOURNAL_NAME    = "mission_s152_journal.txt"
SUMMARY_NAME    = "mission_s152_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
WAYPOINT_DUR    = 6.0
SAMPLE_DWELL_MS = 600

# 3 patrol waypoints, small scale per s149's empirical tuning.
WAYPOINTS = [
    (0.20, 0.00, TAKEOFF_HEIGHT),
    (0.20, 0.20, TAKEOFF_HEIGHT),
    (0.00, 0.20, TAKEOFF_HEIGHT),
]

CLOSURE_TOL_M = 0.10


def _j(ev, payload=None):
    sentai.sim.journal_write(ev, payload if payload is not None else {})


def _ser(v):
    if v is None: return "null"
    if isinstance(v, bool): return "true" if v else "false"
    if isinstance(v, (int, float)): return str(v)
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
    if isinstance(v, list): return "[" + ", ".join(_ser(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{" + ", ".join('"%s": %s' % (k, _ser(val))
                                for k, val in v.items()) + "}"
    return '"<%s>"' % type(v).__name__


def _dist_xy(a, b):
    return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5


def _wait_pose(timeout_ms=5000):
    """Poll servo.pose() until it returns a tuple (subscribe is async)."""
    polled = 0
    while polled < timeout_ms:
        p = sentai.servo.pose()
        if p is not None:
            return p
        sentai.rtos.sleep_ms(50)
        polled += 50
    return None


def _converge_to(tx, ty, label, tol=0.08, poll_ms=6000, settle_ms=4000):
    """Wait `settle_ms` first so HL Commander polynomial has a chance to
    execute (avoids the false-converge bug where the initial stale pose
    is already within tol of the target).  Then poll up to `poll_ms`
    until pose is within tol of target.  Per [[test-must-be-relevant-to-claim]]
    this guards against "drone never moved but pose was already close"
    false PASS.  Total wall-time: settle_ms + poll_ms."""
    sentai.rtos.sleep_ms(settle_ms)
    polled = 0
    last = None
    while polled < poll_ms:
        p = sentai.servo.pose()
        if p is not None:
            last = p
            d = _dist_xy(p, (tx, ty))
            if d < tol:
                _j("converge_" + label, {"x": p[0], "y": p[1],
                                          "dist": d, "ms": settle_ms + polled})
                return last, d
        sentai.rtos.sleep_ms(50)
        polled += 50
    if last is not None:
        _j("converge_timeout_" + label, {
            "x": last[0], "y": last[1],
            "dist": _dist_xy(last, (tx, ty))})
    return last, -1.0


def _dwell(ms):
    polled = 0
    while polled < ms:
        # servo.pose() drains the CRTP RX FIFO as a side effect.
        sentai.servo.pose()
        sentai.rtos.sleep_ms(50)
        polled += 50


def _tight_return(tx, ty, z, tol=0.05, max_iters=4):
    """Iterate HL Commander go_to to absorb undershoot on the home leg.
    Mirrors s149's _tight_return but uses servo.go_to instead of
    sentai.crazy.go_to directly."""
    pose = None
    d = -1.0
    for it in range(max_iters):
        # Shorter dur on retries — drone is already close.
        dur = 2.5 if it == 0 else 1.5
        sentai.servo.set_durations(TAKEOFF_DUR, dur, LAND_DUR)
        sentai.servo.go_to(tx, ty, z, 0.0)
        _j("tight_goto", {"iter": it, "dur": dur})
        # settle_ms < dur*1000 so we observe the move; poll_ms gives a
        # second-and-a-half of post-trajectory dwell for HL Commander
        # ringdown.  Total per-iter wall-time ~ 4 s.
        pose, d = _converge_to(tx, ty, "tight%d" % it,
                                tol=tol,
                                settle_ms=int(dur*1000),
                                poll_ms=1500)
        if pose is not None and 0 <= d <= tol:
            return pose, d
    return pose, d


def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"version": sentai.version()})

    summary = {
        "name": "mission_s152", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "places_stored": [],
        "self_queries":  [],
        "closure_xy": None,
        "servo_status_final": None,
        "errors": [],
    }

    try:
        # ---- Phase 1: bring up transport + servo ----
        # crazy.init() not strictly needed before servo.init(CF2) because
        # the dispatcher uses sentai_crazy_pose_subscribe which probes
        # is_running().  But explicit init is more predictable for
        # mission code — servo.init then subscribes pose in the same
        # pass.
        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["status"] = "FAIL_INIT"; return summary

        rc = sentai.servo.init(sentai.servo.CF2)
        _j("servo_init", {"rc": rc})
        if rc != 0:
            summary["status"] = "FAIL_SERVO_INIT"; return summary

        # Defaults are (2, 6, 2) — set explicitly to match s149 tuning.
        sentai.servo.set_durations(TAKEOFF_DUR, WAYPOINT_DUR, LAND_DUR)
        summary["phases_done"].append("init"); summary["phase_count"] += 1

        # ---- Phase 2: wait for pose stream ----
        if not sentai.servo.pose_ready():
            # First subscribe attempt may have raced; give the pose
            # stream up to 5 s to come online.
            origin = _wait_pose(5000)
        else:
            origin = sentai.servo.pose()
        if origin is None:
            summary["status"] = "FAIL_NO_POSE"; return summary
        summary["origin_xyz"] = [origin[0], origin[1], origin[2]]
        _j("origin", {"x": origin[0], "y": origin[1]})
        summary["phases_done"].append("origin"); summary["phase_count"] += 1

        # ---- Phase 3: places clear ----
        sentai.places.clear()
        _j("places_cleared", {})

        # ---- Phase 4: takeoff ----
        sentai.servo.arm()
        sentai.rtos.sleep_ms(200)
        sentai.servo.takeoff(TAKEOFF_HEIGHT)
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        summary["phases_done"].append("takeoff"); summary["phase_count"] += 1

        # ---- Phase 5: patrol ----
        for i, wp in enumerate(WAYPOINTS):
            sentai.servo.go_to(wp[0], wp[1], wp[2], 0.0)
            _j("goto_wp", {"i": i, "x": wp[0], "y": wp[1]})
            pose, _ = _converge_to(wp[0], wp[1], "wp%d" % i)
            if pose is None:
                summary["errors"].append("wp%d converge failed" % i)
                continue
            _dwell(SAMPLE_DWELL_MS)
            pid = hex_helpers.capture_and_store(
                seed=i, x=pose[0], y=pose[1], z=pose[2])
            _j("place_add", {"wp": i, "pid": pid,
                              "x": pose[0], "y": pose[1]})
            summary["places_stored"].append({
                "wp": i, "pid": pid, "x": pose[0], "y": pose[1],
            })
            gc.collect()
        summary["phases_done"].append("patrol"); summary["phase_count"] += 1

        # ---- Phase 6: self-query ----
        for entry in summary["places_stored"]:
            pid = entry["pid"]
            if pid < 0:
                continue
            match_id = hex_helpers.self_query(pid)
            ok = (match_id == pid)
            _j("self_query", {"pid": pid, "match": match_id, "ok": ok})
            summary["self_queries"].append({
                "pid": pid, "match_id": match_id, "ok": ok,
            })
        summary["phases_done"].append("self_queries")
        summary["phase_count"] += 1

        # ---- Phase 7: return + land ----
        _j("goto_home", {})
        _tight_return(origin[0], origin[1], TAKEOFF_HEIGHT,
                      tol=0.05, max_iters=3)
        summary["phases_done"].append("return"); summary["phase_count"] += 1

        sentai.servo.land()
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 500)
        land_pose = sentai.servo.pose()
        if land_pose is not None:
            summary["closure_xy"] = _dist_xy(land_pose, origin)
            _j("closure", {"d": summary["closure_xy"]})
        summary["phases_done"].append("land"); summary["phase_count"] += 1

        sentai.servo.disarm()
        summary["phases_done"].append("disarm"); summary["phase_count"] += 1
        summary["servo_status_final"] = sentai.servo.status()
        summary["status"] = "DONE"

    except Exception as e:
        summary["status"] = "EXCEPTION"
        summary["errors"].append("exc: " + str(e))
        _j("exception", {"msg": str(e)})
    finally:
        try: sentai.crazy.pose_stop()
        except Exception: pass
        try: sentai.fs.write(SUMMARY_NAME, _ser(summary))
        except Exception: pass
        try: sentai.sim.journal_close()
        except Exception: pass
        try: sentai.crazy.stop()
        except Exception: pass

    return summary

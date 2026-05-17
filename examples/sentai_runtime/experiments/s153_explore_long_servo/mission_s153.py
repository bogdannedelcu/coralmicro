# mission_s153.py — Long-distance exploration on the sentai.servo paradigm.
#
# Equivalent to mission_s147 (port of s137) but driving cf2 SITL through
# sentai.servo.* instead of sentai.crazy.* + crtp_log.py.  Demonstrates that
# Stage 4.A (servo backend dispatch + pose feedback in C) covers the
# "exploration" mission shape end-to-end.
#
# No crtp_log.py — sentai.servo.init(CF2) subscribes pose itself.

import sentai

JOURNAL_NAME    = "mission_s153_journal.txt"
SUMMARY_NAME    = "mission_s153_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
# Mirror s147 tuning: 8 s per 0.4 m hop -> peak ~0.10 m/s, flow-friendly.
WAYPOINT_DUR    = 8.0
INSPECT_DWELL_MS = 1500

WP1 = (0.40, 0.00, TAKEOFF_HEIGHT)
WP2 = (0.40, 0.40, TAKEOFF_HEIGHT)

CLOSURE_TOL_M   = 0.10
APPROACH_TOL_M  = 0.15
# Hard reproducibility gate per [[experiments-start-from-origin]]:
# cf2 must be respawned at world (0, 0, 0.5) before each run.  If the
# captured origin pose is > this distance from (0, 0) we abort — the
# mission would otherwise run from a biased starting position.
ORIGIN_TOL_M    = 0.05


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
    polled = 0
    while polled < timeout_ms:
        p = sentai.servo.pose()
        if p is not None:
            return p
        sentai.rtos.sleep_ms(50)
        polled += 50
    return None


def _converge_to(tx, ty, label, tol=APPROACH_TOL_M,
                  poll_ms=4000, settle_ms=6000):
    """Wait `settle_ms` for HL Commander polynomial to fly the move, then
    poll up to `poll_ms` until pose < tol.  Per [[test-must-be-relevant-to-claim]]
    settle_ms gates against the false-converge bug where the initial stale
    pose was already within tol of the target."""
    sentai.rtos.sleep_ms(settle_ms)
    polled = 0
    last = None
    while polled < poll_ms:
        p = sentai.servo.pose()
        if p is not None:
            last = p
            d = _dist_xy(p, (tx, ty))
            if d < tol:
                _j("converge_" + label, {"x": p[0], "y": p[1], "z": p[2],
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
        sentai.servo.pose()        # drains CRTP RX FIFO as a side effect
        sentai.rtos.sleep_ms(50)
        polled += 50


def _tight_return(tx, ty, z, tol=0.05, max_iters=3):
    """Iterate HL Commander go_to to absorb undershoot on the home leg.
    Mirrors s152's _tight_return — uses servo.go_to + set_durations."""
    pose, d = None, -1.0
    for it in range(max_iters):
        dur = 2.5 if it == 0 else 1.5
        sentai.servo.set_durations(TAKEOFF_DUR, dur, LAND_DUR)
        sentai.servo.go_to(tx, ty, z, 0.0)
        _j("tight_goto", {"iter": it, "dur": dur})
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
        "name": "mission_s153", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "waypoints": [],
        "total_path_m": 0.0,
        "closure_xy": None,
        "servo_status_final": None,
        "errors": [],
    }

    try:
        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["status"] = "FAIL_INIT"; return summary

        rc = sentai.servo.init(sentai.servo.CF2)
        _j("servo_init", {"rc": rc})
        if rc != 0:
            summary["status"] = "FAIL_SERVO_INIT"; return summary
        sentai.servo.set_durations(TAKEOFF_DUR, WAYPOINT_DUR, LAND_DUR)
        summary["phases_done"].append("init"); summary["phase_count"] += 1

        if not sentai.servo.pose_ready():
            origin = _wait_pose(5000)
        else:
            origin = sentai.servo.pose()
        if origin is None:
            summary["status"] = "FAIL_NO_POSE"
            summary["errors"].append("no pose"); return summary
        summary["origin_xyz"] = [origin[0], origin[1], origin[2]]
        # Hard reproducibility gate
        origin_d = (origin[0]**2 + origin[1]**2) ** 0.5
        if origin_d > ORIGIN_TOL_M:
            summary["status"] = "FAIL_ORIGIN_BIAS"
            summary["errors"].append("origin_xy=%.3f m from world (0,0) — restart SITL" % origin_d)
            _j("origin_bias", {"d": origin_d, "x": origin[0], "y": origin[1]})
            return summary
        _j("origin", {"x": origin[0], "y": origin[1], "z": origin[2]})
        summary["phases_done"].append("origin"); summary["phase_count"] += 1

        sentai.servo.arm()
        sentai.rtos.sleep_ms(200)
        sentai.servo.takeoff(TAKEOFF_HEIGHT)
        _j("takeoff", {"h": TAKEOFF_HEIGHT})
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        summary["phases_done"].append("takeoff"); summary["phase_count"] += 1

        prev_xy = (origin[0], origin[1])
        for i, wp in enumerate([WP1, WP2]):
            sentai.servo.go_to(wp[0], wp[1], wp[2], 0.0)
            _j("goto_wp%d" % i, {"x": wp[0], "y": wp[1], "z": wp[2]})
            pose, d = _converge_to(wp[0], wp[1], "wp%d" % i)
            if pose is None or d < 0:
                summary["errors"].append("wp%d timeout dist=%s" % (i, d))
            else:
                leg = _dist_xy(prev_xy, (pose[0], pose[1]))
                summary["total_path_m"] += leg
                prev_xy = (pose[0], pose[1])
                summary["waypoints"].append({
                    "target": [wp[0], wp[1], wp[2]],
                    "pose":   [pose[0], pose[1], pose[2]],
                    "dist":   d,
                    "leg_m":  leg,
                })
                _j("inspect_dwell_start", {"wp": i})
                _dwell(INSPECT_DWELL_MS)
                _j("inspect_dwell_end", {"wp": i})

        summary["phases_done"].append("waypoints"); summary["phase_count"] += 1

        _j("goto_home", {"x": origin[0], "y": origin[1]})
        pose_h, d_h = _tight_return(origin[0], origin[1], TAKEOFF_HEIGHT,
                                      tol=0.05, max_iters=3)
        if pose_h is not None:
            leg = _dist_xy(prev_xy, (pose_h[0], pose_h[1]))
            summary["total_path_m"] += leg
        summary["phases_done"].append("return_home"); summary["phase_count"] += 1

        sentai.servo.land()
        _j("land", {})
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 500)
        land_pose = sentai.servo.pose()
        if land_pose is not None:
            # closure vs WORLD origin (0, 0) per [[sim-test-must-return-home]]
            # — NOT vs captured origin pose (would mask EKF bias).
            summary["closure_xy"] = _dist_xy(land_pose, (0.0, 0.0))
            summary["land_pose"] = [land_pose[0], land_pose[1], land_pose[2]]
            _j("closure", {"d": summary["closure_xy"],
                            "x": land_pose[0], "y": land_pose[1]})
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

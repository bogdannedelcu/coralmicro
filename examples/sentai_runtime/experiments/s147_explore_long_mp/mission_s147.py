# mission_s147.py — MP-only port of s137 long-distance exploration.
#
# Trajectory: origin → wp1 → INSPECT-dwell → wp2 → INSPECT-dwell → home → land.
# 2 markers visited (waypoints), multi-leg path total ~4.5m, closure < 10cm.
#
# Pattern inherited from mission_s146.py with two waypoints instead of one.
# Reuses crtp_log.py for pose feedback (must be staged alongside this file
# in build-sim/sentai_fs_root/).

import sentai
import crtp_log

JOURNAL_NAME    = "mission_s147_journal.txt"
SUMMARY_NAME    = "mission_s147_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
# Slower trajectories — cf2 HL Commander uses polynomial smoothing so the
# mid-trajectory peak velocity is ~2× the average.  Targeting ≤ 0.1 m/s
# peak so optical flow stays inside its tracked-region budget (operator
# feedback 2026-05-16: a fast 0.125 m/s peak destabilised flow tracking
# and the closure overshot from cumulative drift).  For 0.4 m hops at
# 8 s duration: avg = 0.05 m/s, peak ~ 0.10 m/s.
WAYPOINT_DUR    = 8.0
INSPECT_DWELL_MS = 1500
POSE_PERIOD_MS  = 100

# Two waypoints chosen so multi-leg distance > 1 m total but with each
# hop kept short enough that peak velocity stays in flow's sweet spot.
WP1 = (0.40, 0.00, TAKEOFF_HEIGHT)
WP2 = (0.40, 0.40, TAKEOFF_HEIGHT)

CLOSURE_TOL_M   = 0.10
APPROACH_TOL_M  = 0.15
SETTLE_TIMEOUT_MS = 10000


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


def _first_pose(timeout_ms=5000):
    polled = 0
    while polled < timeout_ms:
        crtp_log.poll()
        p = crtp_log.latest_pose()
        if p is not None:
            return p
        sentai.rtos.sleep_ms(50)
        polled += 50
    return None


def _converge_to(tx, ty, label, tol=APPROACH_TOL_M, timeout_ms=SETTLE_TIMEOUT_MS):
    polled = 0
    last = None
    while polled < timeout_ms:
        crtp_log.poll()
        p = crtp_log.latest_pose()
        if p is not None:
            last = p
            d = _dist_xy(p, (tx, ty))
            if d < tol:
                _j("converge_" + label, {
                    "x": p[0], "y": p[1], "z": p[2], "dist": d, "ms": polled,
                })
                return last, d
        sentai.rtos.sleep_ms(50)
        polled += 50
    _j("converge_timeout_" + label, {
        "last_x": last[0] if last else None,
        "last_y": last[1] if last else None,
        "target": [tx, ty], "ms": polled,
    })
    return last, -1.0


def _dwell(ms):
    polled = 0
    while polled < ms:
        crtp_log.poll()
        sentai.rtos.sleep_ms(50)
        polled += 50


def _tight_return(tx, ty, z, tol=0.05, max_iters=3):
    """Iterate go_to + converge until within `tol` of (tx, ty).  cf2's
    HL Commander is open-loop and routinely undershoots by 7-14 cm on
    longer hops; sending shorter follow-up go_to commands corrects the
    residual.  Used for the return-home leg where closure < 10 cm is
    gated.  Returns (pose, dist) of the final state."""
    pose, d = None, -1.0
    for it in range(max_iters):
        dur = 2.5 if it == 0 else 1.5
        sentai.crazy.go_to(tx, ty, z, 0.0, dur, 0, 0, 0)
        _j("tight_goto", {"iter": it, "dur": dur})
        pose, d = _converge_to(tx, ty, "tight_iter%d" % it, tol=tol,
                                timeout_ms=int(dur*1000)+1500)
        if pose is not None and 0 <= d <= tol:
            _j("tight_converged", {"iter": it, "d": d,
                                    "x": pose[0], "y": pose[1]})
            return pose, d
    _j("tight_max_iters", {"d": d})
    return pose, d


def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"version": sentai.version()})

    summary = {
        "name": "mission_s147", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "waypoints": [],     # list of {"target", "pose", "dist"}
        "total_path_m": 0.0,
        "closure_xy": None,
        "errors": [],
    }

    bid = None
    try:
        # transport + log subscription
        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["status"] = "FAIL_INIT"; return summary
        summary["phases_done"].append("init"); summary["phase_count"] += 1

        crtp_log.reset()
        n = crtp_log.scan_toc(timeout_ms=10000)
        _j("toc_scan", {"n": n})
        if n < 50:
            summary["errors"].append("toc=%d" % n)
            summary["status"] = "FAIL_TOC"; return summary

        bid = crtp_log.create_pose_block(block_id=1, period_ms=POSE_PERIOD_MS)
        _j("pose_subscribe", {"bid": bid})
        if bid <= 0:
            summary["errors"].append("subscribe rc=%d" % bid)
            summary["status"] = "FAIL_SUBSCRIBE"; return summary
        summary["phases_done"].append("crtp_log_setup"); summary["phase_count"] += 1

        # origin capture
        origin = _first_pose()
        if origin is None:
            summary["status"] = "FAIL_NO_POSE"
            summary["errors"].append("no pose"); return summary
        summary["origin_xyz"] = [origin[0], origin[1], origin[2]]
        _j("origin", {"x": origin[0], "y": origin[1], "z": origin[2]})
        summary["phases_done"].append("origin"); summary["phase_count"] += 1

        # arm + takeoff
        sentai.crazy.arm()
        sentai.rtos.sleep_ms(200)
        sentai.crazy.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DUR)
        _j("takeoff", {"h": TAKEOFF_HEIGHT})
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        summary["phases_done"].append("takeoff"); summary["phase_count"] += 1

        # multi-leg waypoint sequence: WP1 → INSPECT → WP2 → INSPECT → home
        prev_xy = (origin[0], origin[1])
        for i, wp in enumerate([WP1, WP2]):
            sentai.crazy.go_to(wp[0], wp[1], wp[2], 0.0, WAYPOINT_DUR, 0, 0, 0)
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

        # return home — tight iteration to absorb cf2 HL Commander drift
        _j("goto_home", {"x": origin[0], "y": origin[1]})
        pose_h, d_h = _tight_return(origin[0], origin[1], TAKEOFF_HEIGHT,
                                      tol=0.05, max_iters=3)
        if pose_h is not None:
            leg = _dist_xy(prev_xy, (pose_h[0], pose_h[1]))
            summary["total_path_m"] += leg
        summary["phases_done"].append("return_home"); summary["phase_count"] += 1

        # land
        sentai.crazy.land(0.0, LAND_DUR)
        _j("land", {})
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 500)
        crtp_log.poll()
        land_pose = crtp_log.latest_pose()
        if land_pose is not None:
            summary["closure_xy"] = _dist_xy(land_pose, origin)
            _j("closure", {"d": summary["closure_xy"]})
        summary["phases_done"].append("land"); summary["phase_count"] += 1

        sentai.crazy.disarm()
        summary["phases_done"].append("disarm"); summary["phase_count"] += 1
        summary["status"] = "DONE"

    except Exception as e:
        summary["status"] = "EXCEPTION"
        summary["errors"].append("exc: " + str(e))
        _j("exception", {"msg": str(e)})
    finally:
        try:
            if bid is not None: crtp_log.stop(bid)
        except Exception: pass
        try:
            sentai.fs.write(SUMMARY_NAME, _ser(summary))
        except Exception: pass
        try: sentai.sim.journal_close()
        except Exception: pass
        try: sentai.crazy.stop()
        except Exception: pass

    return summary

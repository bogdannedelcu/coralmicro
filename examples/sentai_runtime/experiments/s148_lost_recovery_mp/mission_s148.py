# mission_s148.py — MP-only port of s138 LOST recovery.
#
# Demonstrates anomalous-state handling in pure MP: mid-mission we
# simulate "LOST" by interrupting normal flow and ascending manually,
# then signal recovery and continue.  Closure < 10 cm vs origin.
#
# Pattern: same shell as mission_s146 with a LOST_SIM phase inserted
# between approach and return_home.  No L6 explore FSM integration yet;
# that's Stage 4.
#
# Stage required upstream: build-sim/sentai_fs_root/crtp_log.py.

import sentai
import crtp_log

JOURNAL_NAME    = "mission_s148_journal.txt"
SUMMARY_NAME    = "mission_s148_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
# Slow velocity to keep flow EKF tracked (peak ~0.10 m/s per 0.3m hop / 5s).
WAYPOINT_DUR    = 5.0
LOST_ASCEND_DUR = 2.0
LOST_DWELL_MS   = 1500
POSE_PERIOD_MS  = 100

TARGET_XY       = (0.20, 0.10)
LOST_DELTA_Z    = 0.30          # +30 cm climb to simulate LOST recovery
CLOSURE_TOL_M   = 0.10


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
        if p is not None: return p
        sentai.rtos.sleep_ms(50); polled += 50
    return None


def _converge_to(tx, ty, label, tol=0.15, timeout_ms=8000):
    polled = 0; last = None
    while polled < timeout_ms:
        crtp_log.poll()
        p = crtp_log.latest_pose()
        if p is not None:
            last = p
            d = _dist_xy(p, (tx, ty))
            if d < tol:
                _j("converge_" + label, {"x": p[0], "y": p[1], "z": p[2],
                                          "dist": d, "ms": polled})
                return last, d
        sentai.rtos.sleep_ms(50); polled += 50
    return last, -1.0


def _dwell(ms):
    polled = 0
    while polled < ms:
        crtp_log.poll()
        sentai.rtos.sleep_ms(50); polled += 50


def _tight_return(tx, ty, z, tol=0.05, max_iters=3):
    """Iterate go_to + converge to absorb cf2 HL Commander undershoot."""
    pose, d = None, -1.0
    for it in range(max_iters):
        dur = 2.5 if it == 0 else 1.5
        sentai.crazy.go_to(tx, ty, z, 0.0, dur, 0, 0, 0)
        _j("tight_goto", {"iter": it, "dur": dur})
        pose, d = _converge_to(tx, ty, "tight%d" % it, tol=tol,
                                timeout_ms=int(dur*1000)+1500)
        if pose is not None and 0 <= d <= tol:
            return pose, d
    return pose, d


def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"version": sentai.version()})
    summary = {
        "name": "mission_s148", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "pose_pre_lost": None,
        "pose_at_lost_apex": None,
        "pose_after_recovery": None,
        "closure_xy": None,
        "lost_simulated": False,
        "errors": [],
    }

    bid = None
    try:
        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["status"] = "FAIL_INIT"; return summary
        summary["phases_done"].append("init"); summary["phase_count"] += 1

        crtp_log.reset()
        n = crtp_log.scan_toc(timeout_ms=10000)
        _j("toc_scan", {"n": n})
        if n < 50:
            summary["errors"].append("toc=%d" % n); summary["status"] = "FAIL_TOC"; return summary
        bid = crtp_log.create_pose_block(block_id=1, period_ms=POSE_PERIOD_MS)
        if bid <= 0:
            summary["errors"].append("subscribe rc=%d" % bid)
            summary["status"] = "FAIL_SUBSCRIBE"; return summary
        summary["phases_done"].append("crtp_log_setup"); summary["phase_count"] += 1

        origin = _first_pose()
        if origin is None:
            summary["status"] = "FAIL_NO_POSE"; return summary
        summary["origin_xyz"] = [origin[0], origin[1], origin[2]]
        _j("origin", {"x": origin[0], "y": origin[1]})
        summary["phases_done"].append("origin"); summary["phase_count"] += 1

        sentai.crazy.arm()
        sentai.rtos.sleep_ms(200)
        sentai.crazy.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DUR)
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        summary["phases_done"].append("takeoff"); summary["phase_count"] += 1

        # approach target
        sentai.crazy.go_to(TARGET_XY[0], TARGET_XY[1], TAKEOFF_HEIGHT,
                            0.0, WAYPOINT_DUR, 0, 0, 0)
        _j("goto_target", {"x": TARGET_XY[0], "y": TARGET_XY[1]})
        pose_t, _ = _converge_to(TARGET_XY[0], TARGET_XY[1], "target")
        if pose_t is not None:
            summary["pose_pre_lost"] = [pose_t[0], pose_t[1], pose_t[2]]
        summary["phases_done"].append("approach"); summary["phase_count"] += 1

        # === simulate LOST ===  cf2 ascends LOST_DELTA_Z m, dwells, recovers.
        # This emulates s138's force_lost → ascend → signal_marker_seen path.
        _j("lost_simulated_start", {"delta_z": LOST_DELTA_Z})
        target_z_lost = TAKEOFF_HEIGHT + LOST_DELTA_Z
        sentai.crazy.go_to(TARGET_XY[0], TARGET_XY[1], target_z_lost,
                            0.0, LOST_ASCEND_DUR, 0, 0, 0)
        sentai.rtos.sleep_ms(int(LOST_ASCEND_DUR * 1000) + 300)
        crtp_log.poll()
        p_apex = crtp_log.latest_pose()
        if p_apex is not None:
            summary["pose_at_lost_apex"] = [p_apex[0], p_apex[1], p_apex[2]]
            _j("lost_apex", {"z": p_apex[2]})
        _dwell(LOST_DWELL_MS)
        # recovery: descend back to normal cruise altitude
        sentai.crazy.go_to(TARGET_XY[0], TARGET_XY[1], TAKEOFF_HEIGHT,
                            0.0, LOST_ASCEND_DUR, 0, 0, 0)
        sentai.rtos.sleep_ms(int(LOST_ASCEND_DUR * 1000) + 300)
        crtp_log.poll()
        p_rec = crtp_log.latest_pose()
        if p_rec is not None:
            summary["pose_after_recovery"] = [p_rec[0], p_rec[1], p_rec[2]]
            _j("lost_recovered", {"x": p_rec[0], "y": p_rec[1], "z": p_rec[2]})
        summary["lost_simulated"] = True
        summary["phases_done"].append("lost_recovery"); summary["phase_count"] += 1

        # return home — tight iteration absorbs HL Commander drift
        _j("goto_home", {})
        _tight_return(origin[0], origin[1], TAKEOFF_HEIGHT, tol=0.05, max_iters=3)
        summary["phases_done"].append("return"); summary["phase_count"] += 1

        sentai.crazy.land(0.0, LAND_DUR)
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
        try: sentai.fs.write(SUMMARY_NAME, _ser(summary))
        except Exception: pass
        try: sentai.sim.journal_close()
        except Exception: pass
        try: sentai.crazy.stop()
        except Exception: pass

    return summary

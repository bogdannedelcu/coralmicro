# mission_s154.py — LOST recovery on the sentai.servo paradigm.
#
# Equivalent to mission_s148 (port of s138) but driving cf2 SITL through
# sentai.servo.* instead of sentai.crazy.* + crtp_log.py.  Mid-mission
# the drone ascends LOST_DELTA_Z m as the "LOST" trigger, dwells, then
# recovers back to cruise altitude.
#
# No crtp_log.py — sentai.servo.init(CF2) subscribes pose itself.

import sentai

JOURNAL_NAME    = "mission_s154_journal.txt"
SUMMARY_NAME    = "mission_s154_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
WAYPOINT_DUR    = 5.0
# Empirically cf2 HL Commander needs ~2× the requested duration to fully
# reach a 30 cm setpoint.  We request 3 s but wait 6 s before sampling
# apex pose, so we observe the polynomial actually settle.
LOST_ASCEND_DUR = 3.0
LOST_SETTLE_MS  = 6000
LOST_DWELL_MS   = 1500

TARGET_XY       = (0.20, 0.10)
LOST_DELTA_Z    = 0.30
CLOSURE_TOL_M   = 0.10
ORIGIN_TOL_M    = 0.05  # see [[experiments-start-from-origin]]


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


def _converge_to(tx, ty, label, tol=0.15, poll_ms=4000, settle_ms=3000):
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
    return last, -1.0


def _dwell(ms):
    polled = 0
    while polled < ms:
        sentai.servo.pose()
        sentai.rtos.sleep_ms(50)
        polled += 50


def _tight_return(tx, ty, z, tol=0.05, max_iters=3):
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
        "name": "mission_s154", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "pose_pre_lost": None,
        "pose_at_lost_apex": None,
        "pose_after_recovery": None,
        "closure_xy": None,
        "lost_simulated": False,
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
            summary["status"] = "FAIL_NO_POSE"; return summary
        summary["origin_xyz"] = [origin[0], origin[1], origin[2]]
        origin_d = (origin[0]**2 + origin[1]**2) ** 0.5
        if origin_d > ORIGIN_TOL_M:
            summary["status"] = "FAIL_ORIGIN_BIAS"
            summary["errors"].append("origin_xy=%.3f m from world (0,0) — restart SITL" % origin_d)
            _j("origin_bias", {"d": origin_d, "x": origin[0], "y": origin[1]})
            return summary
        _j("origin", {"x": origin[0], "y": origin[1]})
        summary["phases_done"].append("origin"); summary["phase_count"] += 1

        sentai.servo.arm()
        sentai.rtos.sleep_ms(200)
        sentai.servo.takeoff(TAKEOFF_HEIGHT)
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        summary["phases_done"].append("takeoff"); summary["phase_count"] += 1

        # ---- approach target ----
        # Hard-sleep until the polynomial fully completes (no early-exit
        # converge here — we need the drone at REST before triggering LOST,
        # otherwise the LOST climb gets folded into a 3D trajectory that
        # takes 2× the requested duration to complete).
        sentai.servo.go_to(TARGET_XY[0], TARGET_XY[1], TAKEOFF_HEIGHT, 0.0)
        _j("goto_target", {"x": TARGET_XY[0], "y": TARGET_XY[1]})
        sentai.rtos.sleep_ms(int(WAYPOINT_DUR * 1000) + 2000)
        pose_t = sentai.servo.pose()
        if pose_t is not None:
            summary["pose_pre_lost"] = [pose_t[0], pose_t[1], pose_t[2]]
            _j("converge_target", {"x": pose_t[0], "y": pose_t[1], "z": pose_t[2]})
        summary["phases_done"].append("approach"); summary["phase_count"] += 1

        # ---- simulate LOST: ascend LOST_DELTA_Z m ----
        _j("lost_simulated_start", {"delta_z": LOST_DELTA_Z})
        target_z_lost = TAKEOFF_HEIGHT + LOST_DELTA_Z
        sentai.servo.set_durations(TAKEOFF_DUR, LOST_ASCEND_DUR, LAND_DUR)
        sentai.servo.go_to(TARGET_XY[0], TARGET_XY[1], target_z_lost, 0.0)
        sentai.rtos.sleep_ms(LOST_SETTLE_MS)
        p_apex = sentai.servo.pose()
        if p_apex is not None:
            summary["pose_at_lost_apex"] = [p_apex[0], p_apex[1], p_apex[2]]
            _j("lost_apex", {"z": p_apex[2]})
        _dwell(LOST_DWELL_MS)

        # ---- recovery: descend back to cruise altitude ----
        sentai.servo.go_to(TARGET_XY[0], TARGET_XY[1], TAKEOFF_HEIGHT, 0.0)
        sentai.rtos.sleep_ms(LOST_SETTLE_MS)
        p_rec = sentai.servo.pose()
        if p_rec is not None:
            summary["pose_after_recovery"] = [p_rec[0], p_rec[1], p_rec[2]]
            _j("lost_recovered", {"x": p_rec[0], "y": p_rec[1], "z": p_rec[2]})
        summary["lost_simulated"] = True
        summary["phases_done"].append("lost_recovery"); summary["phase_count"] += 1

        # ---- return home — tight iteration absorbs HL Commander drift ----
        _j("goto_home", {})
        _tight_return(origin[0], origin[1], TAKEOFF_HEIGHT, tol=0.05, max_iters=3)
        summary["phases_done"].append("return"); summary["phase_count"] += 1

        sentai.servo.land()
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 500)
        land_pose = sentai.servo.pose()
        if land_pose is not None:
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

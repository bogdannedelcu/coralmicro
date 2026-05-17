# mission_s156.py — Real-frame loop closure on the sentai.servo paradigm.
#
# Equivalent to mission_s151 (port of s144) but driving cf2 SITL through
# sentai.servo.* instead of sentai.crazy.* + crtp_log.py.  Same shape as
# s155 BUT descriptors come from REAL Gazebo camera frames via
# sentai.camera.grab_gray() instead of synthetic hex_image.
#
# Prerequisite (live run): camera bridge UP (gz_to_uds_bridge plus Gazebo
# /downward_cam/image streaming into /tmp/sentai_cam.sock).  Without
# bridge, grab_gray() returns None and mission reports FAIL_NO_CAMERA.

import sentai
import hex_helpers

JOURNAL_NAME    = "mission_s156_journal.txt"
SUMMARY_NAME    = "mission_s156_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
WAYPOINT_DUR    = 6.0
SAMPLE_DWELL_MS = 800

FRAME_W = 80
FRAME_H = 60

WAYPOINTS = [
    (0.20, 0.00, TAKEOFF_HEIGHT),
    (0.20, 0.20, TAKEOFF_HEIGHT),
    (0.00, 0.20, TAKEOFF_HEIGHT),
]

CLOSURE_TOL_M = 0.12
ORIGIN_TOL_M  = 0.05  # see [[experiments-start-from-origin]]


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


def _converge_to(tx, ty, label, tol=0.15, poll_ms=4000, settle_ms=4000):
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
        "name": "mission_s156", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "lap1_real_stores": [],
        "lap2_real_matches": [],
        "matches_ok": 0,
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

        sentai.places.clear()
        _j("places_cleared", {})

        # Camera sanity probe
        probe = sentai.camera.grab_gray(FRAME_W, FRAME_H)
        if probe is None:
            summary["status"] = "FAIL_NO_CAMERA"
            summary["errors"].append("camera.grab_gray returned None (gz_to_uds_bridge down?)")
            _j("camera_probe_fail", {})
            return summary
        _j("camera_probe_ok", {"w": probe["w"], "h": probe["h"], "seq": probe["seq"]})
        summary["phases_done"].append("camera_probe"); summary["phase_count"] += 1

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

        # ─── Lap 1: real-frame store ────────────────────────────────
        _j("lap1_start", {})
        for i, wp in enumerate(WAYPOINTS):
            sentai.servo.go_to(wp[0], wp[1], wp[2], 0.0)
            _j("lap1_goto", {"i": i})
            pose, _ = _converge_to(wp[0], wp[1], "lap1_wp%d" % i)
            if pose is None:
                summary["lap1_real_stores"].append({"wp": i, "pid": -1,
                                                     "err": "converge"})
                continue
            _dwell(SAMPLE_DWELL_MS)
            pid = hex_helpers.real_capture_and_store(pose[0], pose[1], pose[2],
                                                      w=FRAME_W, h=FRAME_H)
            _j("lap1_real_store", {"i": i, "pid": pid})
            summary["lap1_real_stores"].append({"wp": i, "pid": pid,
                                                 "pose": [pose[0], pose[1]]})
        summary["phases_done"].append("lap1"); summary["phase_count"] += 1

        # ─── Lap 2: real-frame query ────────────────────────────────
        _j("lap2_start", {})
        for i, wp in enumerate(WAYPOINTS):
            sentai.servo.go_to(wp[0], wp[1], wp[2], 0.0)
            _j("lap2_goto", {"i": i})
            pose, _ = _converge_to(wp[0], wp[1], "lap2_wp%d" % i)
            if pose is None:
                summary["lap2_real_matches"].append({"wp": i, "match": None,
                                                      "err": "converge"})
                continue
            _dwell(SAMPLE_DWELL_MS)
            r = hex_helpers.real_query_at(pose[0], pose[1],
                                            w=FRAME_W, h=FRAME_H)
            _j("lap2_real_query", {"i": i, "match": r})
            summary["lap2_real_matches"].append({"wp": i, "match": r,
                                                  "pose": [pose[0], pose[1]]})

        ok = 0
        for st, q in zip(summary["lap1_real_stores"], summary["lap2_real_matches"]):
            spid = st.get("pid", -1)
            m = q.get("match")
            if m and isinstance(m, dict) and m.get("id") == spid and spid >= 0:
                ok += 1
        summary["matches_ok"] = ok
        summary["phases_done"].append("lap2"); summary["phase_count"] += 1

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

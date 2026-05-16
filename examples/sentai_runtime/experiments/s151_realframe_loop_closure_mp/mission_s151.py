# mission_s151.py — MP-only port of s144 real-frame loop closure.
#
# Same shape as s150 BUT descriptors come from REAL Gazebo camera
# frames via sentai.camera.grab_gray() instead of synthetic hex_image.
# Tests whether PHOG+GIST descriptor pipeline is robust to natural
# lighting/pose variation across two laps.
#
# Reuses: crtp_log.py (s146), hex_helpers.py (s142 — provides
# real_capture_and_store + real_query_at).
#
# Prerequisite (live run): camera bridge UP (gz_to_uds_bridge plus
# Gazebo /downward_cam/image topic streaming into /tmp/sentai_cam.sock).
# Without the bridge, sentai.camera.grab_gray() returns None and the
# mission gracefully reports FAIL_NO_FRAME.

import sentai
import crtp_log
import hex_helpers

JOURNAL_NAME    = "mission_s151_journal.txt"
SUMMARY_NAME    = "mission_s151_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
# Slow trajectories — peak velocity ≈ 0.10 m/s for flow + camera stability.
WAYPOINT_DUR    = 6.0
SAMPLE_DWELL_MS = 800        # extra time for fresh camera frame to land
POSE_PERIOD_MS  = 100

# Camera frame size for descriptor compute — small to fit MP heap budget.
FRAME_W = 80
FRAME_H = 60

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
                _j("converge_" + label, {"x": p[0], "y": p[1],
                                          "dist": d, "ms": polled})
                return last, d
        sentai.rtos.sleep_ms(50); polled += 50
    return last, -1.0


def _dwell(ms):
    polled = 0
    while polled < ms:
        crtp_log.poll(); sentai.rtos.sleep_ms(50); polled += 50


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
        "name": "mission_s151", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "lap1_real_stores": [],     # [{wp, pid, frame_seq}]
        "lap2_real_matches": [],    # [{wp, match, frame_seq}]
        "matches_ok": 0,
        "closure_xy": None,
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
            summary["status"] = "FAIL_TOC"
            summary["errors"].append("toc=%d" % n); return summary
        bid = crtp_log.create_pose_block(block_id=1, period_ms=POSE_PERIOD_MS)
        if bid <= 0:
            summary["status"] = "FAIL_SUBSCRIBE"
            summary["errors"].append("subscribe rc=%d" % bid); return summary
        summary["phases_done"].append("crtp_log_setup"); summary["phase_count"] += 1

        sentai.places.clear()
        _j("places_cleared", {})

        # Camera sanity probe: try one grab to verify bridge is up.
        probe = sentai.camera.grab_gray(FRAME_W, FRAME_H)
        if probe is None:
            summary["status"] = "FAIL_NO_CAMERA"
            summary["errors"].append("camera.grab_gray returned None (gz_to_uds_bridge down?)")
            _j("camera_probe_fail", {})
            return summary
        _j("camera_probe_ok", {"w": probe["w"], "h": probe["h"], "seq": probe["seq"]})
        summary["phases_done"].append("camera_probe"); summary["phase_count"] += 1

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

        # ─── Lap 1: real-frame store ────────────────────────────────
        _j("lap1_start", {})
        for i, wp in enumerate(WAYPOINTS):
            sentai.crazy.go_to(wp[0], wp[1], wp[2], 0.0, WAYPOINT_DUR, 0, 0, 0)
            _j("lap1_goto", {"i": i})
            pose, _ = _converge_to(wp[0], wp[1], "lap1_wp%d" % i)
            if pose is None:
                summary["lap1_real_stores"].append({"wp": i, "pid": -1, "err": "converge"})
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
            sentai.crazy.go_to(wp[0], wp[1], wp[2], 0.0, WAYPOINT_DUR, 0, 0, 0)
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

        # Count matches: lap-2.match.id == lap-1.pid at same wp idx.
        ok = 0
        for st, q in zip(summary["lap1_real_stores"], summary["lap2_real_matches"]):
            spid = st.get("pid", -1)
            m = q.get("match")
            if m and isinstance(m, dict) and m.get("id") == spid and spid >= 0:
                ok += 1
        summary["matches_ok"] = ok
        summary["phases_done"].append("lap2"); summary["phase_count"] += 1

        # return + land — tight iteration on home leg
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

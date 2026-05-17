# mission_s155.py — Loop closure on the sentai.servo paradigm.
#
# Equivalent to mission_s150 (port of s143) but driving cf2 SITL through
# sentai.servo.* instead of sentai.crazy.* + crtp_log.py.  Two-lap
# mission: lap-1 stores 3 synthetic descriptors at 3 waypoints, lap-2
# revisits and queries the L3 gallery — expects MATCH id=stored_pid.
# Demonstrates "drone CONSUMES memory" pattern.
#
# Stage required upstream: build-sim/sentai_fs_root/hex_helpers.py
# (from s142_hex_descriptor_patrol).  No crtp_log.py — servo.init(CF2)
# subscribes pose itself.

import sentai
import hex_helpers

JOURNAL_NAME    = "mission_s155_journal.txt"
SUMMARY_NAME    = "mission_s155_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
WAYPOINT_DUR    = 6.0
SAMPLE_DWELL_MS = 600

WAYPOINTS = [
    (0.20, 0.00, TAKEOFF_HEIGHT),
    (0.20, 0.20, TAKEOFF_HEIGHT),
    (0.00, 0.20, TAKEOFF_HEIGHT),
]

CLOSURE_TOL_M = 0.12  # relaxed for 2-lap (drift accumulates 2x)
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


def _lap(label, store):
    """Fly all WAYPOINTS in order.  If `store` is True, capture+store
    descriptors at the drone's REAL pose; else, query the gallery."""
    rows = []
    for i, wp in enumerate(WAYPOINTS):
        sentai.servo.go_to(wp[0], wp[1], wp[2], 0.0)
        _j("%s_goto" % label, {"i": i, "x": wp[0], "y": wp[1]})
        pose, _ = _converge_to(wp[0], wp[1], "%s_wp%d" % (label, i))
        if pose is None:
            rows.append({"wp": i, "pose": None, "ok": False})
            continue
        _dwell(SAMPLE_DWELL_MS)
        if store:
            pid = hex_helpers.capture_and_store(seed=i,
                                                  x=pose[0], y=pose[1], z=pose[2])
            _j("%s_store" % label, {"i": i, "pid": pid})
            rows.append({"wp": i, "pid": pid, "pose": [pose[0], pose[1]]})
        else:
            img = hex_helpers.hex_image(i)
            phog = sentai.places.compute_phog(img, hex_helpers.W, hex_helpers.H)
            gist = sentai.places.compute_gist(img, hex_helpers.W, hex_helpers.H)
            desc = hex_helpers.quantize(phog, gist)
            cell = hex_helpers.xy_to_h3(pose[0], pose[1], 15)
            r = sentai.places.query(desc, cell, 1, 0)
            _j("%s_query" % label, {"i": i, "match": r})
            rows.append({"wp": i, "match": r, "pose": [pose[0], pose[1]]})
    return rows


def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"version": sentai.version()})

    summary = {
        "name": "mission_s155", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "lap1_stores": [],
        "lap2_matches": [],
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

        # Lap 1 — store
        _j("lap1_start", {})
        summary["lap1_stores"] = _lap("lap1", store=True)
        summary["phases_done"].append("lap1"); summary["phase_count"] += 1

        # Lap 2 — query (consume memory)
        _j("lap2_start", {})
        summary["lap2_matches"] = _lap("lap2", store=False)

        ok = 0
        for store, q in zip(summary["lap1_stores"], summary["lap2_matches"]):
            spid = store.get("pid", -1)
            match = q.get("match")
            if match and isinstance(match, dict) and match.get("id") == spid and spid >= 0:
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

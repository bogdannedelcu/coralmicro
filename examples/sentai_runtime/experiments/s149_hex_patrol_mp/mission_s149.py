# mission_s149.py — MP-only port of s142 HexPatrol.
#
# Drone flies a 3-waypoint patrol; at each waypoint it computes
# PHOG+GIST descriptors from a synthetic image (seed=wp_idx) and adds
# them to the L3 places gallery with the corresponding H3 cell.  After
# the patrol completes, self-query each stored pid → must return same
# pid as best match.  Closure < 10 cm vs origin.
#
# This is the original s142 trajectory + descriptor pipeline running
# INSIDE sentai firmware (no host-side cflib orchestration).
#
# Stage required upstream:
#   build-sim/sentai_fs_root/crtp_log.py
#   build-sim/sentai_fs_root/hex_helpers.py  (from s142_hex_descriptor_patrol)

import gc
import sentai
import crtp_log
import hex_helpers      # synth image + quantize + xy_to_h3 + capture_and_store + self_query

# Subscribe target — used both by crtp_log.create_pose_block AND by the
# scan_toc early-exit set so we don't bloat MP heap with 360 unused TOC
# entries.
POSE_VARS = {
    ('stateEstimate', 'x'), ('stateEstimate', 'y'),
    ('stateEstimate', 'z'), ('stabilizer', 'yaw'),
}

JOURNAL_NAME    = "mission_s149_journal.txt"
SUMMARY_NAME    = "mission_s149_summary.json"

TAKEOFF_HEIGHT  = 0.75
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
# Slow trajectories — peak velocity ≈ 0.10 m/s for flow tracking accuracy.
WAYPOINT_DUR    = 6.0
SAMPLE_DWELL_MS = 600          # enough for a couple of pose updates
POSE_PERIOD_MS  = 100

# 3 patrol waypoints with smaller scale to keep peak velocity in flow budget.
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
                _j("converge_" + label, {"x": p[0], "y": p[1], "dist": d,
                                          "ms": polled})
                return last, d
        sentai.rtos.sleep_ms(50); polled += 50
    return last, -1.0


def _dwell(ms):
    polled = 0
    while polled < ms:
        crtp_log.poll(); sentai.rtos.sleep_ms(50); polled += 50


def _tight_return(tx, ty, z, tol=0.05, max_iters=3):
    """Iterate HL Commander go_to to absorb undershoot on the home leg.
    Low-level Position commander conflicts with HL Commander state in
    cf2 SITL (empirically: drone got stuck mid-mission), so we stick to
    HL Commander iterations even though they oscillate."""
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
        "name": "mission_s149", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "places_stored": [],     # [{wp, pid, x, y}]
        "self_queries":  [],     # [{pid, match_id, score, ok}]
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
        # Early-exit TOC scan keeps MP heap free for descriptors.
        n = crtp_log.scan_toc(timeout_ms=10000, stop_when=POSE_VARS)
        _j("toc_scan", {"n": n})
        gc.collect()
        if n < 50:
            summary["errors"].append("toc=%d" % n)
            summary["status"] = "FAIL_TOC"; return summary
        bid = crtp_log.create_pose_block(block_id=1, period_ms=POSE_PERIOD_MS)
        if bid <= 0:
            summary["errors"].append("subscribe rc=%d" % bid)
            summary["status"] = "FAIL_SUBSCRIBE"; return summary
        summary["phases_done"].append("crtp_log_setup"); summary["phase_count"] += 1

        # Clear L3 gallery so the test starts from a known state.
        sentai.places.clear()
        _j("places_cleared", {})

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

        # Patrol: HL Commander go_to to each waypoint, store descriptor
        # at the drone's ACTUAL pose (closed-loop in the verdict sense:
        # the place is keyed to where drone REALLY is, not where we
        # asked it to go).  cf2 HL Commander undershoots by 10-15 cm but
        # that's tolerable for descriptor matching since lap-2 uses the
        # same actual-pose lookup pattern.
        for i, wp in enumerate(WAYPOINTS):
            sentai.crazy.go_to(wp[0], wp[1], wp[2], 0.0, WAYPOINT_DUR, 0, 0, 0)
            _j("goto_wp", {"i": i, "x": wp[0], "y": wp[1]})
            pose, _ = _converge_to(wp[0], wp[1], "wp%d" % i)
            if pose is None:
                summary["errors"].append("wp%d converge failed" % i)
                continue
            _dwell(SAMPLE_DWELL_MS)
            pid = hex_helpers.capture_and_store(seed=i, x=pose[0], y=pose[1], z=pose[2])
            _j("place_add", {"wp": i, "pid": pid, "x": pose[0], "y": pose[1]})
            summary["places_stored"].append({
                "wp": i, "pid": pid, "x": pose[0], "y": pose[1],
            })
            gc.collect()  # release the 16KB hex_image scratch ASAP
        summary["phases_done"].append("patrol"); summary["phase_count"] += 1

        # Self-query: each stored pid should be its own best match.
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
        summary["phases_done"].append("self_queries"); summary["phase_count"] += 1

        # return + land — tight iteration on home leg (HL Commander only)
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

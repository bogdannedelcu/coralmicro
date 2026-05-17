# mission_s160.py — sentai.aruco altitude sweep over the static
# aruco_4x4_50 quartet from sentai_crazysim.sdf.
#
# Per [[missions-run-in-sentai-only]]: this whole script runs inside
# sentai_sim's MP VM.  Host launcher only stages + kicks off + reads
# the summary JSON afterwards.  Per [[sentai-sim-air-gapped-from-truth]]
# the ONLY data sources are CRTP LOG (drone pose) + camera bridge
# (frame buffer) — no Gazebo ground truth.
#
# Trajectory:
#   ARM → TAKEOFF(0.30) → for z in ALT_SWEEP:
#       go_to(0, 0, z) → settle → sample N frames via sentai.aruco
#       → record detection rate + per-marker tvec_z
#   → return_to(origin, 0.30) → LAND → DISARM
#
# Per CLAUDE.md compute-in-C principle: detect_from_camera() goes
# C-to-C via sentai_camera_grab_gray_zerocopy; the image buffer never
# crosses the MP binding.

import sentai
import crtp_log

# ─── Configuration ──────────────────────────────────────────────────
JOURNAL_NAME    = "mission_s160_journal.txt"
SUMMARY_NAME    = "mission_s160_summary.json"

TAKEOFF_HEIGHT  = 0.30
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
WAYPOINT_DUR    = 3.0
SETTLE_MS       = 2500
SAMPLE_MS       = 200      # period between consecutive frames
N_SAMPLES       = 10
APPROACH_TOL_M  = 0.10
SETTLE_TIMEOUT_MS = 10000
CLOSURE_TOL_M   = 0.15
POSE_PERIOD_MS  = 100

# Camera intrinsics for the SIM downward camera (320x240 after
# zerocopy resize from native 640x480; fx_eff = native_fx / 2).
# Default Gazebo down-cam has ~fx=480 at 640x480 -> 240 at 320x240.
CAM_FX = 240.0
CAM_FY = 240.0
CAM_CX = 160.0
CAM_CY = 120.0
# ArUco markers in sentai_crazysim.sdf are 0.08x0.08 m face at z=0.20 m
# (10 cm tall box on the floor, texture on the top face).
MARKER_SIZE_M = 0.08

# Altitudes (drone z above ground in meters).  Below 0.30 m we get
# danger from prop-wash + obstacle margin; above 1.2 m markers are
# too small in 320x240.
ALT_SWEEP = (0.30, 0.40, 0.50, 0.70, 1.00)


def _j(label, value=None):
    sentai.sim.journal_write(label, value)


def _wait_pose(timeout_ms=5000):
    polled = 0
    while polled < timeout_ms:
        crtp_log.poll()
        p = crtp_log.latest_pose()
        if p is not None:
            return p
        sentai.rtos.sleep_ms(50)
        polled += 50
    return None


def _dist_xy(p, target):
    if p is None:
        return 1.0e9
    dx = p[0] - target[0]
    dy = p[1] - target[1]
    return (dx * dx + dy * dy) ** 0.5


def _converge_xyz(tx, ty, tz, tol_m=APPROACH_TOL_M,
                   timeout_ms=SETTLE_TIMEOUT_MS, label=""):
    polled = 0
    last = None
    while polled < timeout_ms:
        crtp_log.poll()
        p = crtp_log.latest_pose()
        if p is not None:
            last = p
            d_xy = _dist_xy(p, (tx, ty))
            d_z = abs(p[2] - tz)
            if d_xy < tol_m and d_z < tol_m:
                _j("converge_" + label, {
                    "x": p[0], "y": p[1], "z": p[2],
                    "d_xy": d_xy, "d_z": d_z, "ms": polled,
                })
                return last, polled
        sentai.rtos.sleep_ms(50)
        polled += 50
    _j("converge_timeout_" + label, {
        "last_x": last[0] if last else None,
        "last_y": last[1] if last else None,
        "last_z": last[2] if last else None,
        "target": (tx, ty, tz),
    })
    return last, polled


def _settle(ms):
    polled = 0
    while polled < ms:
        crtp_log.poll()
        sentai.rtos.sleep_ms(50)
        polled += 50


def _sample_detection(z_nominal, n_samples=N_SAMPLES):
    """Run N detections back-to-back at the current pose; aggregate."""
    samples = []
    for i in range(n_samples):
        markers = sentai.aruco.detect_from_camera()
        crtp_log.poll()
        pose = crtp_log.latest_pose()
        samples.append({
            "i":       i,
            "n":       len(markers),
            "ids":     sorted([m['marker_id'] for m in markers]),
            "tvec_zs": [round(m['tvec_cam'][2], 4) for m in markers],
            "reprojs": [round(m['reproj_err_px'], 2) for m in markers],
            "z":       pose[2] if pose else None,
        })
        sentai.rtos.sleep_ms(SAMPLE_MS)
    return samples


def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"version": sentai.version()})

    summary = {
        "name":          "mission_s160",
        "version":       sentai.version(),
        "status":        "STARTED",
        "origin":        None,
        "by_altitude":   [],
        "closure_xy":    None,
        "errors":        [],
    }

    bid = None
    try:
        rc = sentai.crazy.init()
        _j("crazy_init", {"rc": rc})
        if rc != 0:
            summary["status"] = "FAIL_INIT"
            return summary

        crtp_log.reset()
        n = crtp_log.scan_toc(timeout_ms=10000)
        _j("toc_scan", {"n": n})
        if n < 50:
            summary["status"] = "FAIL_TOC"
            return summary
        bid = crtp_log.create_pose_block(block_id=1, period_ms=POSE_PERIOD_MS)
        if bid <= 0:
            summary["status"] = "FAIL_SUBSCRIBE"
            return summary

        sentai.aruco.init()
        sentai.aruco.set_intrinsics(CAM_FX, CAM_FY, CAM_CX, CAM_CY)
        sentai.aruco.set_marker_size(MARKER_SIZE_M)
        _j("aruco_init", {
            "fx": CAM_FX, "fy": CAM_FY,
            "cx": CAM_CX, "cy": CAM_CY,
            "size_m": MARKER_SIZE_M,
        })

        origin = _wait_pose(timeout_ms=5000)
        if origin is None:
            summary["status"] = "FAIL_NO_POSE"
            return summary
        summary["origin"] = [origin[0], origin[1], origin[2]]
        _j("origin", {"x": origin[0], "y": origin[1], "z": origin[2]})

        sentai.crazy.arm()
        sentai.rtos.sleep_ms(200)
        sentai.crazy.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DUR)
        _j("takeoff", {"h": TAKEOFF_HEIGHT})
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)

        for z in ALT_SWEEP:
            _j("alt_begin", {"z": z})
            # ABSOLUTE go_to (relative=0): drone moves to (origin_x,
            # origin_y, z) in world frame. Default of sentai.crazy.go_to
            # is relative=1, which would drift the drone upward by `z`
            # at every iteration (caught 2026-05-17 on first s160 run).
            sentai.crazy.go_to(origin[0], origin[1], z, 0.0,
                                WAYPOINT_DUR, 0)
            sentai.rtos.sleep_ms(int(WAYPOINT_DUR * 1000) + 500)
            pose, ms = _converge_xyz(origin[0], origin[1], z,
                                     tol_m=APPROACH_TOL_M,
                                     timeout_ms=SETTLE_TIMEOUT_MS,
                                     label="alt_%d" % int(z * 100))
            _settle(SETTLE_MS)
            samples = _sample_detection(z, N_SAMPLES)
            # Aggregate.
            hits = sum(1 for s in samples if s["n"] > 0)
            ids_seen = set()
            tvec_zs = []
            reprojs = []
            for s in samples:
                ids_seen.update(s["ids"])
                tvec_zs.extend(s["tvec_zs"])
                reprojs.extend(s["reprojs"])
            entry = {
                "z_nominal":      z,
                "z_actual":       pose[2] if pose else None,
                "n_samples":      len(samples),
                "n_with_detect":  hits,
                "detect_rate":    hits / float(len(samples)),
                "unique_ids":     sorted(list(ids_seen)),
                "tvec_z_min":     min(tvec_zs) if tvec_zs else None,
                "tvec_z_max":     max(tvec_zs) if tvec_zs else None,
                "reproj_mean":    sum(reprojs) / len(reprojs) if reprojs else None,
                "reproj_max":     max(reprojs) if reprojs else None,
            }
            summary["by_altitude"].append(entry)
            _j("alt_done", entry)

        # Return + land — ABSOLUTE go_to.
        sentai.crazy.go_to(origin[0], origin[1], TAKEOFF_HEIGHT, 0.0,
                            WAYPOINT_DUR, 0)
        sentai.rtos.sleep_ms(int(WAYPOINT_DUR * 1000) + 500)
        pose, _ = _converge_xyz(origin[0], origin[1], TAKEOFF_HEIGHT,
                                 tol_m=APPROACH_TOL_M, label="return")
        sentai.crazy.land(LAND_DUR)
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 500)
        last = crtp_log.latest_pose()
        summary["closure_xy"] = _dist_xy(last, (origin[0], origin[1]))
        _j("land_done", {"closure_xy": summary["closure_xy"]})
        sentai.crazy.disarm()
        summary["status"] = "OK"

    except Exception as e:
        summary["status"] = "ERROR"
        summary["errors"].append(repr(e))
        _j("exception", repr(e))
    finally:
        try:
            if bid is not None and bid > 0:
                crtp_log.delete_block(bid)
        except Exception:
            pass
        sentai.sim.journal_close()
        sentai.fs.write(SUMMARY_NAME, str(summary))

    return summary

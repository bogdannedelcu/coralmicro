# mission_s158.py — OP-S6-W1-T7: mission_s153 + takeoff calibration.
#
# Extends s153 (long-distance exploration on sentai.servo) with a
# pre-trajectory takeoff calibration phase that runs the camera-to-body
# Kabsch (sentai.calib) on real ArUco PnP samples captured via
# sentai.aruco.detect_from_camera() over the static aruco_4x4_50
# quartet from sentai_crazysim.sdf.
#
# This is the first time sentai.calib + sentai.aruco land together
# end-to-end in flight.  Per the air-gap rule [[sentai-sim-air-gapped-
# from-truth]] the calibration uses ONLY:
#   - tvec_cam from sentai.aruco (which reads camera frames)
#   - drone_W + yaw from sentai.servo.pose() (CRTP LOG telemetry)
#   - marker_W from the known landing-pad layout (passed in by mission)
# No Gazebo ground truth is used by the C code at any point.
#
# Trajectory:
#   ARM -> TAKEOFF(1.0) -> calib hover at origin -> collect K frames
#         -> kabsch+save -> proceed with s153 waypoints
#         -> tight_return -> LAND -> closure verdict.

import sentai

JOURNAL_NAME    = "mission_s158_journal.txt"
SUMMARY_NAME    = "mission_s158_summary.json"

# A4-pad layout: 6 cm flat markers (top at z=0.005).  Hover at 0.5 m
# gives projected 28.8 px per marker (~5 px/cell, decoder OK).  Higher
# altitudes shrink the marker below the decoder's reliable range
# given the heuristic quad extraction.
TAKEOFF_HEIGHT  = 0.50
TAKEOFF_DUR     = 2.0
LAND_DUR        = 2.0
WAYPOINT_DUR    = 8.0
INSPECT_DWELL_MS = 1500
CALIB_SETTLE_MS  = 2500
CALIB_FRAME_PERIOD_MS = 200
CALIB_N_FRAMES   = 8

# Waypoints scaled to fit the A4 layout (markers at ±0.06 in x, ±0.10
# in y).  Stay within ~25 cm of origin to keep the markers in FOV
# during the trajectory.
WP1 = (0.15, 0.00, TAKEOFF_HEIGHT)
WP2 = (0.15, 0.15, TAKEOFF_HEIGHT)

# Closure gate same as s153 (10 cm) with a relaxed budget for the
# extra calibration leg (single 5 cm bump per [[s147-s151-migrations]]).
CLOSURE_TOL_M   = 0.12
APPROACH_TOL_M  = 0.15
ORIGIN_TOL_M    = 0.05

# Camera intrinsics: 320x240 effective after camera_bridge downscale
# from native 640x480 Gazebo camera.  fx=fy=240 cx=160 cy=120 match
# s160 results (tvec_z = drone_z - marker_z to ~5% accuracy).
CAM_FX = 240.0
CAM_FY = 240.0
CAM_CX = 160.0
CAM_CY = 120.0
MARKER_SIZE_M = 0.06

# A4 takeoff/landing pad layout (sim identical to real-world A4 print).
# 4 markers, 6x6 cm flat face, top at z=0.005 m (=5 mm above ground —
# matches the cat-rug pose to stay in-plane).  Centres at (±60, ±100)
# mm from page centre.  Total spread 12x20 cm fits A4 portrait
# (210x297 mm) with ~4.5 cm L/R + 4.85 cm T/B margins.
#
# To print: see s158_calib_takeoff/README.md for the exact PDF spec.
# Real-world setup: lay the A4 sheet flat on a hard surface, drone
# takes off from page centre, hovers at z=0.5 m, samples markers.
MARKER_W = {
    0: (+0.06, +0.10, 0.005),
    1: (-0.06, +0.10, 0.005),
    2: (-0.06, -0.10, 0.005),
    3: (+0.06, -0.10, 0.005),
}

CALIB_ACCEPT_MEAN_RES_DEG = 3.0


def _j(ev, payload=None):
    sentai.sim.journal_write(ev, payload if payload is not None else {})


def _ser(v):
    if v is None: return "null"
    if isinstance(v, bool): return "true" if v else "false"
    if isinstance(v, (int, float)): return str(v)
    if isinstance(v, str):
        return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
    # MicroPython tuple has no __name__ attr — handle alongside list.
    if isinstance(v, (list, tuple)):
        return "[" + ", ".join(_ser(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{" + ", ".join('"%s": %s' % (k, _ser(val))
                                for k, val in v.items()) + "}"
    try:
        return '"<%s>"' % type(v).__name__
    except AttributeError:
        return '"<unknown>"'


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
                                          "z": p[2], "d": d, "ms": polled})
                return p, d
        sentai.rtos.sleep_ms(50)
        polled += 50
    _j("converge_timeout_" + label, {
        "last": last, "target": (tx, ty), "tol": tol,
    })
    return last, -1.0


def _dwell(ms):
    polled = 0
    while polled < ms:
        sentai.servo.pose()
        sentai.rtos.sleep_ms(50)
        polled += 50


def _collect_calib_samples(n_frames, period_ms):
    """Drone hovers in place; sample N frames + accumulate
    (tvec_cam, marker_W, drone_W, yaw) tuples per detected marker."""
    samples = []
    frame_log = []
    for i in range(n_frames):
        markers = sentai.aruco.detect_from_camera()
        pose = sentai.servo.pose()
        ids_this_frame = []
        for m in markers:
            mid = m['marker_id']
            if mid not in MARKER_W:
                continue
            samples.append((
                m['tvec_cam'],
                MARKER_W[mid],
                (pose[0], pose[1], pose[2]),
                pose[3],
            ))
            ids_this_frame.append(mid)
        frame_log.append({
            "i":   i,
            "ids": sorted(ids_this_frame),
            "pose_z": pose[2] if pose else None,
            "n":   len(markers),
        })
        sentai.rtos.sleep_ms(period_ms)
    return samples, frame_log


def run():
    sentai.sim.journal_open(JOURNAL_NAME)
    _j("mission_start", {"version": sentai.version()})

    summary = {
        "name":   "mission_s158", "status": "STARTED",
        "phases_done": [], "phase_count": 0,
        "origin_xyz": None,
        "calib": None,
        "calib_frames": [],
        "waypoints": [],
        "total_path_m": 0.0,
        "closure_xy": None,
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

        sentai.aruco.init()
        sentai.aruco.set_intrinsics(CAM_FX, CAM_FY, CAM_CX, CAM_CY)
        sentai.aruco.set_marker_size(MARKER_SIZE_M)
        sentai.calib.init()
        _j("calib_init", {
            "cam_intr": (CAM_FX, CAM_FY, CAM_CX, CAM_CY),
            "size_m":   MARKER_SIZE_M,
            "default_R": sentai.calib.get_R_cam_to_body(),
        })
        summary["phases_done"].append("init")
        summary["phase_count"] += 1

        # Capture origin.
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
            summary["errors"].append("origin_xy=%.3f from (0,0)" % origin_d)
            return summary
        _j("origin", {"x": origin[0], "y": origin[1], "z": origin[2]})
        summary["phases_done"].append("origin"); summary["phase_count"] += 1

        # ARM + TAKEOFF.
        sentai.servo.arm()
        sentai.rtos.sleep_ms(200)
        sentai.servo.takeoff(TAKEOFF_HEIGHT)
        _j("takeoff", {"h": TAKEOFF_HEIGHT})
        sentai.rtos.sleep_ms(int(TAKEOFF_DUR * 1000) + 500)
        summary["phases_done"].append("takeoff"); summary["phase_count"] += 1

        # ── Calibration phase ─────────────────────────────────────
        # Hover at origin, settle, then sample frames + run Kabsch.
        _dwell(CALIB_SETTLE_MS)
        samples, frame_log = _collect_calib_samples(
            CALIB_N_FRAMES, CALIB_FRAME_PERIOD_MS)
        summary["calib_frames"] = frame_log
        _j("calib_samples", {"n": len(samples)})

        # Persisted R is the on-load default (or last saved value).
        persisted = sentai.calib.get_R_cam_to_body()
        R_new, quality = sentai.calib.run_kabsch(samples, persisted)
        calib_info = {
            "n_samples":              quality["n_samples"],
            "det_R":                  quality["det_R"],
            "mean_residual_deg":      quality["mean_residual_deg"],
            "max_residual_deg":       quality["max_residual_deg"],
            "drift_from_persisted":   quality["drift_from_persisted_deg"],
            "accepted":               quality["accepted"],
            "reject_code":            quality["reject_code"],
            "R_new":                  R_new,
            "persisted_R":            persisted,
        }
        summary["calib"] = calib_info
        _j("kabsch", calib_info)

        if quality["accepted"]:
            rc_commit = sentai.calib.commit_R(R_new, None)
            rc_save   = sentai.calib.save()
            _j("calib_save", {"rc_commit": rc_commit, "rc_save": rc_save})
            calib_info["committed"] = (rc_commit == 0)
            calib_info["saved"]     = bool(rc_save)
            summary["phases_done"].append("calib_persisted")
        else:
            summary["errors"].append(
                "kabsch rejected: code=%d, mean_res=%.2f" % (
                    quality["reject_code"], quality["mean_residual_deg"]))
            summary["phases_done"].append("calib_attempted")
        summary["phase_count"] += 1

        # ── Continue s153 trajectory ──────────────────────────────
        prev_xy = (origin[0], origin[1])
        for i, wp in enumerate([WP1, WP2]):
            sentai.servo.go_to(wp[0], wp[1], wp[2], 0.0)
            _j("goto_wp%d" % i, {"x": wp[0], "y": wp[1], "z": wp[2]})
            pose, d = _converge_to(wp[0], wp[1], "wp%d" % i)
            _dwell(INSPECT_DWELL_MS)
            if pose is None or d < 0:
                summary["errors"].append("wp%d timeout d=%s" % (i, d))
                continue
            leg = _dist_xy(prev_xy, (pose[0], pose[1]))
            summary["total_path_m"] += leg
            summary["waypoints"].append({
                "i": i, "x": pose[0], "y": pose[1], "z": pose[2],
                "leg_m": leg, "d_xy": d,
            })
            prev_xy = (pose[0], pose[1])
        summary["phases_done"].append("waypoints")
        summary["phase_count"] += 1

        # Tight return + land.
        sentai.servo.set_durations(TAKEOFF_DUR, 2.5, LAND_DUR)
        sentai.servo.go_to(origin[0], origin[1], TAKEOFF_HEIGHT, 0.0)
        pose, d = _converge_to(origin[0], origin[1], "return",
                                settle_ms=3000, poll_ms=2000)
        leg = _dist_xy(prev_xy, (pose[0], pose[1])) if pose else 0.0
        summary["total_path_m"] += leg

        sentai.servo.land()
        sentai.rtos.sleep_ms(int(LAND_DUR * 1000) + 800)
        last = sentai.servo.pose()
        summary["closure_xy"] = _dist_xy(last, (origin[0], origin[1])) \
                                if last else None
        _j("land_done", {"closure_xy": summary["closure_xy"],
                          "last_z": last[2] if last else None})
        sentai.servo.disarm()
        summary["phases_done"].append("landed")
        summary["phase_count"] += 1

        if summary["closure_xy"] is not None \
                and summary["closure_xy"] < CLOSURE_TOL_M \
                and summary["calib"] is not None \
                and summary["calib"]["accepted"]:
            summary["status"] = "OK"
        else:
            summary["status"] = "FAIL_GATE"

    except Exception as e:
        summary["status"] = "ERROR"
        summary["errors"].append(repr(e))
        _j("exception", repr(e))
    finally:
        sentai.sim.journal_close()
        sentai.fs.write(SUMMARY_NAME, _ser(summary))

    return summary

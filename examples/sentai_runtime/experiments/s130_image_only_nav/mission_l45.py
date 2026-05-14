"""s130 4.5Baseline mission — image-only world-model navigation.

Drone visits 4 ArUco markers using only multi-marker PnP for pose +
yaw. `cf2.stateEstimate` is captured but NEVER consulted in the inner
control loop — image_localize() is the sole pose source. Validates
the image-only localization primitive that will become permanent at
takeoff calibration + Stage 6 yaw loop-closure.

Reuses harness components from s128/s129:
    ReplDriver, StepLog, j_int helper, sentai.sim.journal API,
    cflib MotionCommander + flow_forwarder thread, telemetry capture,
    ibvs_center_on_marker (s129).

Outputs (under /tmp/s130_image_only_nav/):
    summary.json        — verdict reads this
    journal.txt         — REPL-side structured journal
    image_vs_cf2.json   — per-outer-iter (image_pose, cf2_pose, yaws)
    ibvs_iter_log.json  — per-IBVS-iter convergence trace
    mission.log         — host-side step trace
    repl.transcript     — raw stdin/stdout dump
    cf2_telemetry.json  — stateEstimate samples
    servo_*.json        — final servo.status() / .trace() dumps
"""
from __future__ import annotations

import ast
import contextlib
import datetime as dt
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

S091_DIR = Path(__file__).resolve().parent.parent / "s091_aruco_lowalt"
S090_DIR = Path(__file__).resolve().parent.parent / "s090_hover_over_cat"
S128_DIR = Path(__file__).resolve().parent.parent / "s128_l41baseline_seeded"
S129_DIR = Path(__file__).resolve().parent.parent / "s129_l42baseline_centering"
sys.path.insert(0, str(S091_DIR))
sys.path.insert(0, str(S090_DIR))
sys.path.insert(0, str(S128_DIR))
sys.path.insert(0, str(S129_DIR))
import aruco_hover                                         # noqa: E402
from aruco_detector import (                               # noqa: E402
    detect_in_ppm, latest_ppm,
    CAM_CX, CAM_CY, CAM_FX, CAM_FY,
    CAM_OFFSET_BODY,
)
# NOTE: we deliberately do NOT use aruco_detector.estimate_drone_world_pose
# nor its _R_CAM_TO_BODY here.  Empirically (2026-05-14, build #113) the
# real Gazebo tvecs are reflected 180° around Z relative to what aruco
# detector's stale R assumes — using the stale R yields image-pose mirrored
# through origin + yaw 180° off.  s128/s129 IBVS empirically tuned their
# tvec→body sign convention to match the CURRENT camera, so they work; the
# breakage is hidden in the `estimate_drone_world_pose` code path (used
# only by s091/s108/s109 which haven't been re-validated since).  Rather
# than touch aruco_detector and risk regressing those, we use a local
# R_CAM_TO_BODY tuned for the CURRENT Gazebo state.
from mission_l41 import ReplDriver, StepLog                # noqa: E402
from mission_l42 import (                                  # noqa: E402
    ibvs_center_on_marker, wait_for_fresh_ppm,
    NAV_TOL_M as IBVS_NAV_TOL_M,
    NAV_MIN_STEP_M as IBVS_NAV_MIN_STEP_M,
    NAV_SETTLE_S as IBVS_NAV_SETTLE_S,
    MARKER_PIXEL_TIGHT_PX,
    IBVS_VEL_MPS, IBVS_GAIN_K,
)

WORKDIR = Path("/tmp/s130_image_only_nav")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
TRACE_JSON      = WORKDIR / "servo_trace.json"
STATUS_JSON     = WORKDIR / "servo_status.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
IBVS_LOG_JSON   = WORKDIR / "ibvs_iter_log.json"
IMG_VS_CF2_JSON = WORKDIR / "image_vs_cf2.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

JOURNAL_NAME      = "s130_journal.txt"
JOURNAL_HOST_PATH = lambda: SENTAI_FS_ROOT / JOURNAL_NAME    # noqa: E731

# ─── Markers seeded into sentai.objects at start ──────────────────────
# (class_id, world_x, world_y, world_z, label, aruco_dict_id)
# Same layout as s128/s129 — all 4 visible at z=1 m hover above origin.
# Class id == ArUco dict id by convention (used as the sentai.objects
# class field so we can look up world position by ArUco id).
ARUCO_MARKERS_FULL = [
    (0, +0.15, +0.10, 0.20, "id0_NE", 0),
    (1, -0.15, +0.10, 0.20, "id1_NW", 1),
    (2, -0.15, -0.10, 0.20, "id2_SW", 2),
    (3, +0.15, -0.10, 0.20, "id3_SE", 3),
]
# Single-target smoke mode: set env S130_SINGLE_TARGET=<aruco_id> to
# visit ONLY that marker.  Used for the first-run sign-convention check
# (operator request, verbose(1) + single target before full tour).
_single_tgt_env = os.environ.get("S130_SINGLE_TARGET")
if _single_tgt_env is not None:
    _stid = int(_single_tgt_env)
    ARUCO_MARKERS = [m for m in ARUCO_MARKERS_FULL if m[5] == _stid]
    if not ARUCO_MARKERS:
        raise RuntimeError(f"S130_SINGLE_TARGET={_stid} matches no marker")
else:
    ARUCO_MARKERS = ARUCO_MARKERS_FULL
# Built from the FULL marker list — image_localize needs ALL known
# marker positions to disambiguate, even when the tour visits only one.
MARKER_WORLD_MAP = {
    aid: (x, y, z) for (cid, x, y, z, label, aid) in ARUCO_MARKERS_FULL
}

# Flight params
HOVER_Z_M       = 1.00
TAKEOFF_Z_M     = 1.00
TAKEOFF_VEL_MPS = 0.6
NAV_VEL_MPS     = 0.20

# Outer-loop tolerances (image-only world-frame navigation)
NAV_OUTER_MAX_ITERS  = 8
NAV_OUTER_TIMEOUT_S  = 14.0
NAV_OUTER_TOL_M      = 0.05    # within 5 cm world XY → handoff to IBVS
NAV_OUTER_SETTLE_S   = 0.7
NAV_OUTER_MIN_STEP_M = 0.015
NAV_OUTER_GAIN_K     = 0.6     # damp world-delta moves

# Hand-off: pixel distance below which we switch from outer-loop world-
# nav to inner-loop IBVS pixel servoing.  At z=1 m, 1 px ≈ 1.7 mm so
# 80 px ≈ 14 cm — well within IBVS gain-stable range.
IBVS_HANDOFF_PX = 80

# Minimum known markers for image_localize to function.  Below this,
# yaw is not observable from baselines alone → abort.
MIN_KNOWN_MARKERS = 2


# ─────────────────────────────────────────────────────────────────
# Image-only localization helpers
# ─────────────────────────────────────────────────────────────────
# R_cam_to_body — empirically calibrated 2026-05-14 against the
# CURRENT Gazebo sentai_crazysim camera mount.  Verified by:
#   - drone at world (~0,~0,~1.05), yaw=0; 4 markers at ±0.15×±0.10.
#   - tvec id0 = (+0.078, +0.177, +0.881)  →  body = (+0.177, +0.078, -0.881)
#     ≈ world marker pose relative to drone (modulo cam-CoM offset).
# Convention: body.x = +tvec.y, body.y = +tvec.x, body.z = -tvec.z.
# This DIFFERS from aruco_detector._R_CAM_TO_BODY (which has negated
# X/Y rows — out-of-date relative to current SDF).  We keep this local
# to avoid regressing other consumers of aruco_detector.
_R_CAM_TO_BODY_LOCAL = np.array([[0.0, 1.0,  0.0],
                                  [1.0, 0.0,  0.0],
                                  [0.0, 0.0, -1.0]], dtype=np.float32)


def _R_body_to_world_2d(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0],
                      [s,  c, 0.0],
                      [0.0, 0.0, 1.0]], dtype=np.float32)


def yaw_from_baselines_2d(dets, marker_world_xy_map):
    """Estimate drone yaw via 2D rotation-only Procrustes.

    For each visible marker with known world XY, transform its tvec
    to body frame via R_cam_to_body. The optimal yaw mapping body XY
    onto world XY has closed-form `atan2(sum_cross, sum_dot)`.

    Args:
        dets: {marker_id: Marker} from detect_in_ppm(estimate_pose=True).
        marker_world_xy_map: {marker_id: (wx, wy)} known world positions.

    Returns:
        yaw in radians, or None if fewer than 2 valid markers.
    """
    body_xy: list[tuple[float, float]] = []
    world_xy: list[tuple[float, float]] = []
    for mid, m in dets.items():
        if mid not in marker_world_xy_map or m.tvec is None:
            continue
        t = np.asarray(m.tvec).reshape(-1)
        if t.size != 3 or not np.all(np.isfinite(t)):
            continue
        b = _R_CAM_TO_BODY_LOCAL @ t.astype(np.float32)
        body_xy.append((float(b[0]), float(b[1])))
        world_xy.append(tuple(marker_world_xy_map[mid]))
    if len(body_xy) < 2:
        return None
    b = np.asarray(body_xy, dtype=np.float64)
    w = np.asarray(world_xy, dtype=np.float64)
    b -= b.mean(axis=0)
    w -= w.mean(axis=0)
    sum_cross = float(np.sum(b[:, 0] * w[:, 1] - b[:, 1] * w[:, 0]))
    sum_dot   = float(np.sum(b[:, 0] * w[:, 0] + b[:, 1] * w[:, 1]))
    return math.atan2(sum_cross, sum_dot)


def image_localize(dets, marker_world_map):
    """Multi-marker image-only localization (drone CoM in world).

    Computes drone (x, y, z, yaw) using only the visible markers'
    known world positions + their PnP tvecs.  No IMU, no EKF.

    Algorithm:
        1. yaw = yaw_from_baselines_2d(dets) (Procrustes-2D).
        2. R_cam_to_world = R_body_to_world(yaw) @ R_cam_to_body.
        3. For each visible known marker i:
             cam_in_world_i = marker_world_i - R_cam_to_world @ tvec_i
        4. cam_world = mean across markers.
        5. drone_CoM_world = cam_world - R_body_to_world(yaw) @ cam_offset_body.

    Returns:
        (drone_world_xyz_np, drone_yaw_rad) or None on failure (<2 markers).
    """
    xy_map = {mid: pos[:2] for mid, pos in marker_world_map.items()}
    yaw = yaw_from_baselines_2d(dets, xy_map)
    if yaw is None:
        return None
    R_body_to_world = _R_body_to_world_2d(yaw)
    R_cam_to_world  = R_body_to_world @ _R_CAM_TO_BODY_LOCAL
    cam_estimates: list[np.ndarray] = []
    for mid, m in dets.items():
        if mid not in marker_world_map or m.tvec is None:
            continue
        t = np.asarray(m.tvec).reshape(-1).astype(np.float32)
        if t.size != 3 or not np.all(np.isfinite(t)):
            continue
        mw = np.asarray(marker_world_map[mid], dtype=np.float32)
        cam_in_world = mw - R_cam_to_world @ t
        cam_estimates.append(cam_in_world)
    if not cam_estimates:
        return None
    cam_world = np.mean(np.stack(cam_estimates), axis=0)
    cam_offset_world = R_body_to_world @ CAM_OFFSET_BODY
    drone_world = cam_world - cam_offset_world
    return drone_world, yaw


def world_to_body_delta(world_dx: float, world_dy: float,
                         yaw: float) -> tuple[float, float]:
    """Rotate a world-frame XY vector into body-frame XY by -yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    return (+c * world_dx + s * world_dy,
            -s * world_dx + c * world_dy)


# ─────────────────────────────────────────────────────────────────
# cf2 telemetry capture (for VERDICT comparison only; never read by
# the control loop — image_localize is the sole pose source).
# ─────────────────────────────────────────────────────────────────
_tel_lock = threading.Lock()
_tel_last = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
_tel_samples: list[dict] = []


def _tel_cb(_ts, data, _lc):
    with _tel_lock:
        _tel_last["x"]       = data["stateEstimate.x"]
        _tel_last["y"]       = data["stateEstimate.y"]
        _tel_last["z"]       = data["stateEstimate.z"]
        _tel_last["yaw_deg"] = data["stateEstimate.yaw"]
        _tel_samples.append({
            "t": time.monotonic(),
            "x": _tel_last["x"], "y": _tel_last["y"], "z": _tel_last["z"],
            "yaw_deg": _tel_last["yaw_deg"],
        })


def tel_snapshot() -> dict:
    with _tel_lock:
        return dict(_tel_last)


# ─────────────────────────────────────────────────────────────────
# Outer-loop image-only navigation toward a target marker.
# Runs until target appears within IBVS_HANDOFF_PX of image centre,
# or NAV_OUTER_TIMEOUT_S elapses, or NAV_OUTER_MAX_ITERS exhausted.
# ─────────────────────────────────────────────────────────────────
def image_only_outer_nav(log: StepLog, repl: ReplDriver, mc, j_int,
                          frames_dir: Path,
                          target_aruco_id: int,
                          target_world_xy: tuple[float, float],
                          image_vs_cf2: list) -> dict:
    """Drive drone toward target_world_xy using image-only localization.
    Returns dict with outcome + counters."""
    last_seq = -1
    t_start = time.monotonic()
    n_iters = 0
    n_localize_fail = 0
    last_pose = None
    last_yaw = None
    converged_for_ibvs = False

    for it in range(NAV_OUTER_MAX_ITERS):
        if (time.monotonic() - t_start) > NAV_OUTER_TIMEOUT_S:
            log.info(f"        outer it{it}: TIMEOUT "
                     f"({NAV_OUTER_TIMEOUT_S}s)")
            break

        got = wait_for_fresh_ppm(frames_dir, last_seq, deadline_s=3.0)
        if got is None:
            log.info(f"        outer it{it}: no fresh PPM in 3 s — abort")
            break
        ppm, last_seq = got
        try:
            dets = detect_in_ppm(ppm, estimate_pose=True)
        except Exception as e:
            log.info(f"        outer it{it}: detect failed {e}")
            continue
        # Filter to known markers only — unknown ones (e.g. spurious
        # detections) must not poison the Procrustes fit.
        dets_known = {k: v for k, v in dets.items() if k in MARKER_WORLD_MAP}
        if len(dets_known) < MIN_KNOWN_MARKERS:
            n_localize_fail += 1
            log.info(f"        outer it{it}: only {len(dets_known)} known "
                     f"markers visible (need {MIN_KNOWN_MARKERS})")
            continue

        loc = image_localize(dets_known, MARKER_WORLD_MAP)
        if loc is None:
            n_localize_fail += 1
            log.info(f"        outer it{it}: image_localize returned None")
            continue
        (image_pose, image_yaw) = loc
        last_pose, last_yaw = image_pose, image_yaw

        tel = tel_snapshot()
        image_vs_cf2.append({
            "t_s": time.monotonic() - t_start,
            "target_id": target_aruco_id,
            "ppm": ppm.name,
            "image":         list(map(float, image_pose)),
            "image_yaw_deg": math.degrees(image_yaw),
            "cf2":           [tel["x"], tel["y"], tel["z"]],
            "cf2_yaw_deg":   tel["yaw_deg"],
            "n_dets":        len(dets_known),
            "det_ids":       sorted(dets_known.keys()),
        })

        # If target is already in FOV and centred close enough → IBVS.
        if target_aruco_id in dets_known:
            m = dets_known[target_aruco_id]
            px = math.hypot(m.cx - CAM_CX, m.cy - CAM_CY)
            if px < IBVS_HANDOFF_PX:
                log.info(f"        outer it{it}: target id{target_aruco_id} "
                         f"in FOV at px_dist={px:.1f} < {IBVS_HANDOFF_PX} "
                         f"→ HANDOFF to IBVS")
                converged_for_ibvs = True
                break
            log.info(f"        outer it{it}: target id{target_aruco_id} "
                     f"in FOV at px_dist={px:.1f}  (≥ {IBVS_HANDOFF_PX}, "
                     f"continue outer nav)")

        # World-delta from image-derived pose to target.
        wdx = target_world_xy[0] - image_pose[0]
        wdy = target_world_xy[1] - image_pose[1]
        world_dist = math.hypot(wdx, wdy)
        if world_dist < NAV_OUTER_TOL_M:
            log.info(f"        outer it{it}: within outer tol "
                     f"(world_dist={world_dist:.3f} m < {NAV_OUTER_TOL_M})")
            converged_for_ibvs = True
            break

        body_dx, body_dy = world_to_body_delta(wdx, wdy, image_yaw)
        body_dx *= NAV_OUTER_GAIN_K
        body_dy *= NAV_OUTER_GAIN_K
        log.info(f"        outer it{it}: image_pose=({image_pose[0]:+.3f},"
                 f"{image_pose[1]:+.3f},{image_pose[2]:.3f})  "
                 f"yaw={math.degrees(image_yaw):+.1f}°  "
                 f"world_delta=({wdx:+.3f},{wdy:+.3f}) → "
                 f"body=({body_dx:+.3f},{body_dy:+.3f})  "
                 f"cf2=({tel['x']:+.3f},{tel['y']:+.3f},{tel['z']:.3f})  "
                 f"dets={sorted(dets_known.keys())}")

        sx = body_dx if abs(body_dx) >= NAV_OUTER_MIN_STEP_M else 0.0
        sy = body_dy if abs(body_dy) >= NAV_OUTER_MIN_STEP_M else 0.0
        if sx == 0.0 and sy == 0.0:
            log.info(f"        outer it{it}: both deltas under "
                     f"NAV_OUTER_MIN_STEP_M; converged enough")
            converged_for_ibvs = True
            break

        rc = j_int(f"servo_outer_id{target_aruco_id}_it{it}",
                   f"sentai.servo.move({sx}, {sy}, 0)")
        if rc != 0:
            raise RuntimeError(
                f"servo.move outer it{it} id{target_aruco_id} rc={rc}")
        mc.move_distance(sx, sy, 0.0, velocity=NAV_VEL_MPS)
        time.sleep(NAV_OUTER_SETTLE_S)
        n_iters = it + 1

    return {
        "n_iters":            n_iters,
        "n_localize_fail":    n_localize_fail,
        "converged_for_ibvs": converged_for_ibvs,
        "last_image_pose":    list(map(float, last_pose)) if last_pose is not None else None,
        "last_image_yaw_deg": math.degrees(last_yaw) if last_yaw is not None else None,
        "last_ppm_seq":       last_seq,
    }


# ─────────────────────────────────────────────────────────────────
# Mission
# ─────────────────────────────────────────────────────────────────
def fly(log: StepLog, repl: ReplDriver) -> dict:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    summary: dict = {}

    with log.step("cf2 link + Kalman reset + damp params"):
        sync = SyncCrazyflie("udp://127.0.0.1:19850",
                             cf=Crazyflie(rw_cache=None))
        sync.open_link()
        cf = sync.cf
        cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(0.5)
        for k, v in {
            "posCtlPid.xyKd":     0.5,
            "velCtlPid.vxKd":     0.05,
            "velCtlPid.vyKd":     0.05,
            "posCtlPid.xVelMax":  2.5,
            "posCtlPid.yVelMax":  2.5,
            "posCtlPid.xKp":      3.0,
            "posCtlPid.yKp":      3.0,
        }.items():
            try:
                cf.param.set_value(k, v)
            except Exception:
                pass
        time.sleep(0.3)
        cf.param.set_value("kalman.resetEstimation", 1)
        time.sleep(0.5)
        cf.param.set_value("kalman.resetEstimation", 0)
        time.sleep(2.0)

    with log.step("telemetry log @ 20 ms"):
        lc = LogConfig(name="att", period_in_ms=20)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                  "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
            lc.add_variable(v, "float")
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(_tel_cb)
        lc.start()

    with log.step("flow forwarder thread"):
        stop_evt = threading.Event()
        flow_stats = {"n_sent": 0, "fatal": None, "last_err": None}
        flow_th = threading.Thread(target=aruco_hover.flow_forwarder,
                                    args=(stop_evt, cf, flow_stats),
                                    daemon=True)
        flow_th.start()
        time.sleep(2.0)
        if flow_stats["fatal"]:
            raise RuntimeError(f"flow forwarder fatal: {flow_stats['fatal']}")
        log.info(f"        flow forwarder running (n_sent={flow_stats['n_sent']})")

    def j_int(label: str, cmd: str) -> int:
        rc = repl.exec_int(cmd)
        repl.exec_int(f"sentai.sim.journal_write('{label}', "
                       f"sentai.servo.status())")
        return rc

    verbose_level = int(os.environ.get("S130_VERBOSE", "0"))
    with log.step(f"REPL: import + verbose({verbose_level}) + journal_open"):
        repl.exec("import sentai")
        repl.exec(f"sentai.verbose({verbose_level})")
        if repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')") != 0:
            raise RuntimeError("journal_open failed")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")

    # aruco_id → object_id (object_id is allocated by sentai.objects.add
    # and is NOT necessarily equal to class_id — g_next_id persists across
    # clear() calls, so back-to-back runs get incrementing ids).
    # ALWAYS seed all 4 markers in sentai.objects, even if the tour
    # visits only one (single-target mode).  image_localize needs all
    # known marker world positions to disambiguate.
    aruco_to_objid: dict[int, int] = {}
    with log.step(f"REPL: clear + seed {len(ARUCO_MARKERS_FULL)} markers in sentai.objects"):
        repl.exec_int("sentai.objects.clear()")
        for cid, x, y, z, label, aid in ARUCO_MARKERS_FULL:
            rid = repl.exec_int(f"sentai.objects.add({cid}, {x}, {y}, {z})")
            if rid <= 0:
                raise RuntimeError(f"objects.add({label}) returned {rid}")
            aruco_to_objid[aid] = rid
            log.info(f"        seeded {label}: class_id={cid} aruco_id={aid} "
                     f"world=({x:+.2f},{y:+.2f},{z:.2f}) → object_id={rid}")
        n = repl.exec_int("sentai.objects.count()")
        if n != len(ARUCO_MARKERS_FULL):
            raise RuntimeError(f"objects.count()={n}, expected {len(ARUCO_MARKERS_FULL)}")
        summary["aruco_to_objid"] = aruco_to_objid

    with log.step("REPL: servo.init + arm + takeoff (intent)"):
        if j_int("servo_init", "sentai.servo.init('sim')") != 0:
            raise RuntimeError("servo.init failed")
        repl.exec_int("sentai.servo.clear_trace()")
        if j_int("servo_arm", "sentai.servo.arm()") != 0:
            raise RuntimeError("servo.arm failed")
        if j_int("servo_takeoff", f"sentai.servo.takeoff({TAKEOFF_Z_M})") != 0:
            raise RuntimeError("servo.takeoff failed")

    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(2.5)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        log.info(f"        post-takeoff cf2 @ ({tel['x']:+.2f},{tel['y']:+.2f},"
                 f"{tel['z']:.2f})  yaw={tel['yaw_deg']:+.1f}°  "
                 f"flow_n={flow_stats['n_sent']}")
        summary["cf2_post_takeoff"] = [tel["x"], tel["y"], tel["z"]]
        summary["cf2_yaw_deg_post_takeoff"] = tel["yaw_deg"]

    # ─── Image-only world-model tour ───
    # Per marker: validate seed via sentai.objects.get(), drive outer
    # loop (image-only world-frame nav) until target near image centre,
    # then IBVS for final centring.
    all_iter_logs: list = []
    image_vs_cf2_log: list = []
    per_marker: list = []
    with log.step(f"image-only tour of {len(ARUCO_MARKERS)} markers"):
        for cid, x, y, z, label, aruco_id in ARUCO_MARKERS:
            # Look up target world from sentai.objects (validates round-trip).
            objid = aruco_to_objid[aruco_id]
            target_obj = repl.exec_repr(f"sentai.objects.get({objid})")
            if not target_obj or "x" not in target_obj:
                raise RuntimeError(f"objects.get({objid}) bad: {target_obj!r}")
            t_xy = (float(target_obj["x"]), float(target_obj["y"]))
            log.info(f"        target id{aruco_id} ({label}) "
                     f"objid={objid} sentai.objects → world=({t_xy[0]:+.3f},"
                     f"{t_xy[1]:+.3f})")

            # ─── Outer loop: image-only world-frame navigation ───
            with log.step(f"outer image-only nav toward id{aruco_id} "
                          f"[max {NAV_OUTER_MAX_ITERS} iters, "
                          f"{NAV_OUTER_TIMEOUT_S}s timeout]"):
                outer = image_only_outer_nav(
                    log, repl, mc, j_int, repl.frames_dir,
                    aruco_id, t_xy, image_vs_cf2_log,
                )
                log.info(f"        outer id{aruco_id} result: {outer}")

            # ─── Inner loop: IBVS final centring (reused from s129) ───
            with log.step(f"IBVS centre on id{aruco_id} "
                          f"[max iters from s129, timeout from s129]"):
                iter_log: list = []
                ibvs = ibvs_center_on_marker(log, repl, mc, j_int,
                                              repl.frames_dir,
                                              aruco_id, iter_log)
                for ent in iter_log:
                    ent["target_id"] = aruco_id
                all_iter_logs.extend(iter_log)
                log.info(f"        IBVS id{aruco_id}: {ibvs}")

            m_record: dict = {
                "id":    aruco_id,
                "outer": outer,
                "ibvs":  ibvs,
            }
            if iter_log and ibvs.get("converged"):
                last = iter_log[-1]
                tel = tel_snapshot()
                m_record.update({
                    "final_pixel":     last["pixel"],
                    "final_pixel_err": last["pixel_err"],
                    "cf2_at_converge": [tel["x"], tel["y"], tel["z"]],
                })
            per_marker.append(m_record)

            if j_int(f"servo_hover_id{aruco_id}",
                      "sentai.servo.hover()") != 0:
                raise RuntimeError(f"servo.hover after id{aruco_id} failed")

    summary["per_marker"]   = per_marker
    summary["image_vs_cf2"] = image_vs_cf2_log
    n_conv = sum(1 for m in per_marker
                 if m.get("ibvs", {}).get("converged"))
    summary["n_markers_converged"] = n_conv
    summary["n_markers_total"]     = len(ARUCO_MARKERS)
    summary["n_localize_failures"] = sum(
        m.get("outer", {}).get("n_localize_fail", 0) for m in per_marker
    )
    log.info(f"        summary: {n_conv}/{len(ARUCO_MARKERS)} markers IBVS-converged, "
             f"{summary['n_localize_failures']} localize failures")

    with log.step("REPL: servo.land + disarm"):
        if j_int("servo_land", "sentai.servo.land()") != 0:
            raise RuntimeError("servo.land failed")
        if j_int("servo_disarm", "sentai.servo.disarm()") != 0:
            raise RuntimeError("servo.disarm failed")

    with log.step("cf2 land"):
        mc.land(velocity=0.3)
        time.sleep(2.0)

    with log.step("teardown telemetry + cf2 link"):
        stop_evt.set()
        flow_th.join(timeout=1.5)
        lc.stop()
        sync.close_link()
        summary["flow_n_sent"] = flow_stats["n_sent"]

    with log.step("REPL: dump servo.status / trace via fs.write"):
        repl.exec_int("sentai.sim.journal_write('mission_dump_begin', None)")
        status = repl.exec_via_fs("sentai.servo.status()", SENTAI_FS_ROOT)
        trace  = repl.exec_via_fs("sentai.servo.trace()",  SENTAI_FS_ROOT)
        STATUS_JSON.write_text(json.dumps(status, indent=2))
        TRACE_JSON.write_text(json.dumps(trace,  indent=2))
        summary["servo_status"] = status
        summary["servo_trace"]  = trace
        repl.exec_int("sentai.sim.journal_write('mission_end', None)")
        repl.exec_int("sentai.sim.journal_close()")

    # Surface journal + iter logs
    journal = JOURNAL_HOST_PATH()
    if journal.is_file():
        (WORKDIR / "journal.txt").write_text(journal.read_text())
    IBVS_LOG_JSON.write_text(json.dumps(all_iter_logs, indent=2))
    IMG_VS_CF2_JSON.write_text(json.dumps(image_vs_cf2_log, indent=2))

    return summary


def main() -> int:
    log = StepLog(MISSION_LOG)
    log.info(f"REPO_ROOT={REPO_ROOT}")
    if not SENTAI_SIM_BIN.is_file():
        log.info("FATAL — sentai_sim binary missing")
        return 1

    repl = ReplDriver(SENTAI_SIM_BIN, SENTAI_FS_ROOT, REPL_TRANSCRIPT,
                       frames_dir_base=WORKDIR)
    try:
        summary = fly(log, repl)
    except BaseException as e:
        log.info(f"MISSION FAILED: {type(e).__name__}: {e}")
        with contextlib.suppress(Exception):
            st = repl.exec_via_fs("sentai.servo.status()", SENTAI_FS_ROOT)
            STATUS_JSON.write_text(json.dumps(st, indent=2))
        with contextlib.suppress(Exception):
            tr = repl.exec_via_fs("sentai.servo.trace()", SENTAI_FS_ROOT)
            TRACE_JSON.write_text(json.dumps(tr, indent=2))
        with contextlib.suppress(Exception):
            repl.exec_int("sentai.sim.journal_write('mission_crash', None)")
            repl.exec_int("sentai.sim.journal_close()")
        journal = JOURNAL_HOST_PATH()
        if journal.is_file():
            (WORKDIR / "journal.txt").write_text(journal.read_text())
        SUMMARY_JSON.write_text(json.dumps({
            "_status":     "FAIL",
            "_exception":  f"{type(e).__name__}: {e}",
            "_last_run":   dt.datetime.now().isoformat(),
        }, indent=2))
        repl.close()
        log.close()
        return 1

    with _tel_lock:
        TELEMETRY_JSON.write_text(json.dumps(_tel_samples))
    summary["_status"]   = "OK"
    summary["_last_run"] = dt.datetime.now().isoformat()
    SUMMARY_JSON.write_text(json.dumps(summary, indent=2))
    log.info("MISSION OK")
    repl.close()
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

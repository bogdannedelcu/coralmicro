"""ArUco marker detector — ARM/x86 agnostic API.

Detects 4x4_50 ArUco markers in 640×480 RGB frames.  Returns a dict
keyed by marker_id with bbox (x1,y1,x2,y2 in pixel space), centroid,
and corner pixel positions.  Pose estimation is optional (requires
camera_matrix + dist_coeffs).

Architecture:
  - x86/SIM: this Python module uses cv2.aruco (opencv-contrib-python).
  - ARM/board: TODO — port a minimal C ArUco implementation to
    examples/sentai_runtime/sentai_aruco.{h,cc}.  Same API
    (`detect_markers(rgb_buf, w, h) -> list[Marker]`).  The s090
    calibration logic in hover_over_cat.py consumes the dict and
    is platform-agnostic.

This separation lets us iterate calibration logic without waiting
for the ARM port — same algorithm validates on both targets later.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

# ArUco lives in opencv-contrib-python (cv2.aruco namespace).
import cv2


# Known world positions of markers (matches SDF).  Used by the
# calibration loop to compute drone-to-marker offsets in world frame
# without needing to query Gazebo each tick.
# A4 takeoff/landing pad layout — re-aligned 2026-05-17 to match the
# CURRENT world SDF (sentai_crazysim.sdf) marker poses after the
# A4-pad redesign.  Old layout (tall posts at ±0.15, ±0.10, z=0.20)
# was ORPHANED — the world was updated 2026-05-17 to A4 ground-flat
# poses (±0.06, ±0.10, z=0.005) but this PnP table never followed.
# That divergence silently corrupted every PnP-derived pose until
# the [[cf2-sitl-cheat-odom-gt]] cheat plugin masked it; with the
# cheat removed in FlowBaseline, the drone spiralled out (3.17 m
# offset in s127 trial 2026-05-17).  Single source of truth lives
# alongside the world SDF + s158_calib_takeoff/mission_s158.py.
#
# OP-S10-W14 (2026-05-18) — bumped from 0.06 to 0.12 m face / from
# ±0.06,±0.10 to ±0.12,±0.20 positions alongside SDF doubling
# (operator: "imaginea RGB e prea mica, hai sa ii facem markerii de
# 2 ori mai mari").
# Marker: 12×12 cm box face, top at z = pose.z + thickness/2 = 0.005 +
# 0.005 = 0.010 m.  PnP uses top-face corners.
KNOWN_POSITIONS_M = {
    0: (+0.12, +0.20, 0.010),
    1: (-0.12, +0.20, 0.010),
    2: (-0.12, -0.20, 0.010),
    3: (+0.12, -0.20, 0.010),
}
# Effective ArUco square size — 12×12 cm box face fully textured with
# the 4x4_50 pattern.  solvePnP detects the outer black border = full
# 0.12 m face.
MARKER_SIZE_M = 0.12

# Camera intrinsics — derived from gz cam SDF FOV.  Used for pose
# estimation (estimatePoseSingleMarkers).
# Image 640×480, HFOV=58°  →  fx = 640 / (2 * tan(29°)) ≈ 577 px
#                              fy = 480 / (2 * tan(45°/2)) ≈ 579 px
CAM_W = 640
CAM_H = 480
CAM_FOV_H_RAD = np.deg2rad(58.0)
CAM_FOV_V_RAD = np.deg2rad(45.0)
CAM_FX = CAM_W / (2.0 * np.tan(CAM_FOV_H_RAD / 2))
CAM_FY = CAM_H / (2.0 * np.tan(CAM_FOV_V_RAD / 2))
CAM_CX = CAM_W / 2.0
CAM_CY = CAM_H / 2.0
CAM_MATRIX = np.array([[CAM_FX, 0,      CAM_CX],
                        [0,     CAM_FY, CAM_CY],
                        [0,     0,      1]], dtype=np.float32)
DIST_COEFFS = np.zeros((5,), dtype=np.float32)   # no distortion in sim


@dataclass
class Marker:
    """One detected marker."""
    id: int
    corners: np.ndarray         # shape (4, 2), pixel coords (x, y)
    cx: float                   # centroid x in image
    cy: float                   # centroid y in image
    # Optional pose-from-PnP (filled if estimate_pose=True)
    tvec: Optional[np.ndarray] = None   # (3,)  translation in camera frame, metres
    rvec: Optional[np.ndarray] = None   # (3,)  Rodrigues rotation


_DET_PARAMS = cv2.aruco.DetectorParameters()
_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_DETECTOR = cv2.aruco.ArucoDetector(_DICT, _DET_PARAMS)


# Camera offset from drone CoM (body frame).  From model.sdf.jinja:
# <pose>-0.04 0 -0.02 0 1.5707963 3.1415927</pose>
#   -0.04m: camera is 4cm BACK of CoM along body +X
#   0:      no Y offset
#   -0.02m: camera is 2cm BELOW CoM along body +Z
CAM_OFFSET_BODY = np.array([-0.04, 0.0, -0.02], dtype=np.float32)


_R_CAM_TO_BODY = np.array([[0.0, -1.0,  0.0],
                            [-1.0, 0.0,  0.0],
                            [0.0,  0.0, -1.0]], dtype=np.float32)
# Derived empirically (s092 axis calibration 2026-05-11):
# - +body_X motion ↔ -L0_dy (cam_Y axis = -body_X)
# - +body_Y motion ↔ -L0_dx (cam_X axis = -body_Y)
# - cam_Z (optical axis) points DOWN = -body_Z
# Verified: with this R and drone at (0,0,1), marker[0]@(0.15,0.10,0.20),
# computed tvec ≈ (-0.10, -0.19, +0.78) — matches observed solvePnP output.


def estimate_drone_world_pose(dets, known_positions=None, drone_yaw=0.0):
    """Compute drone world position from detected ArUco markers.

    Strategy: use solvePnP's tvec for each marker (well-defined under
    any drone attitude), and a KNOWN camera-to-world rotation derived
    from drone yaw + the fixed body-to-camera mount transform.  Avoids
    relying on solvePnP's rvec, which can be sign-ambiguous under tilt.

    For each detected marker i with known world position marker_w:
        tvec_i = marker_i_in_camera_frame
        cam_in_world = marker_w - R_cam_to_world @ tvec_i

    Average across visible markers, then subtract camera-CoM offset.

    Args:
        dets: {marker_id: Marker} from detect_markers(estimate_pose=True).
        known_positions: override module-level KNOWN_POSITIONS_M.
        drone_yaw: drone heading (radians) for body↔world rotation.

    Returns:
        (drone_x, drone_y, drone_z) in world frame, or None on failure.
    """
    if not dets:
        return None
    kp = known_positions or KNOWN_POSITIONS_M
    # Sanity-check drone_yaw — bounded to ±2π (any larger value indicates
    # unit confusion, e.g. degrees passed in by mistake).
    if not math.isfinite(drone_yaw) or abs(drone_yaw) > 2 * math.pi + 1e-3:
        return None
    # Camera-to-world rotation = (body-to-world by yaw) ∘ (cam-to-body).
    # Ignore drone roll/pitch — they're <5° even under wind, contribute
    # <10% projection error in tvec interpretation.  Yaw matters more
    # since it can rotate up to 360°.
    cy, sy = np.cos(drone_yaw), np.sin(drone_yaw)
    R_body_to_world = np.array([[cy, -sy, 0],
                                 [sy,  cy, 0],
                                 [0,   0,  1]], dtype=np.float32)
    R_cam_to_world = R_body_to_world @ _R_CAM_TO_BODY
    cam_estimates = []
    for mid, m in dets.items():
        if mid not in kp or m.tvec is None:
            continue
        # Validate tvec shape — solvePnP returns either (3,) or (3, 1).
        # A wrong shape would crash later; bail explicitly for this marker.
        tvec_raw = np.asarray(m.tvec).reshape(-1)
        if tvec_raw.size != 3 or not np.all(np.isfinite(tvec_raw)):
            continue
        mx, my, mz = kp[mid]
        tvec = tvec_raw.astype(np.float32)
        cam_in_world = np.array([mx, my, mz]) - R_cam_to_world @ tvec
        cam_estimates.append(cam_in_world)
    if not cam_estimates:
        return None
    cam_world = np.mean(cam_estimates, axis=0)
    # Drone CoM = camera world position - (body-to-world rotated cam offset)
    cam_offset_world = R_body_to_world @ CAM_OFFSET_BODY
    drone_world = cam_world - cam_offset_world
    return tuple(float(v) for v in drone_world)


def detect_markers(rgb_or_gray: np.ndarray,
                    estimate_pose: bool = False) -> Dict[int, Marker]:
    """Detect ArUco markers in an RGB or grayscale frame.

    Args:
        rgb_or_gray: H×W×3 uint8 RGB image, or H×W uint8 gray.
        estimate_pose: if True, run solvePnP to fill .tvec/.rvec.

    Returns:
        dict { marker_id: Marker } — empty if no detections.
    """
    if rgb_or_gray.ndim == 3:
        gray = cv2.cvtColor(rgb_or_gray, cv2.COLOR_RGB2GRAY)
    else:
        gray = rgb_or_gray
    corners, ids, _ = _DETECTOR.detectMarkers(gray)
    out: Dict[int, Marker] = {}
    if ids is None or len(ids) == 0:
        return out
    if estimate_pose:
        # Pose estimation per-marker (each marker is independent).
        obj_pts = np.array([
            [-MARKER_SIZE_M/2,  MARKER_SIZE_M/2, 0],
            [ MARKER_SIZE_M/2,  MARKER_SIZE_M/2, 0],
            [ MARKER_SIZE_M/2, -MARKER_SIZE_M/2, 0],
            [-MARKER_SIZE_M/2, -MARKER_SIZE_M/2, 0],
        ], dtype=np.float32)
        for c, mid in zip(corners, ids.flatten()):
            c4 = c.reshape(4, 2).astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(obj_pts, c4, CAM_MATRIX, DIST_COEFFS)
            cx = float(c4[:, 0].mean())
            cy = float(c4[:, 1].mean())
            out[int(mid)] = Marker(id=int(mid), corners=c4, cx=cx, cy=cy,
                                    tvec=tvec.flatten() if ok else None,
                                    rvec=rvec.flatten() if ok else None)
    else:
        for c, mid in zip(corners, ids.flatten()):
            c4 = c.reshape(4, 2).astype(np.float32)
            cx = float(c4[:, 0].mean())
            cy = float(c4[:, 1].mean())
            out[int(mid)] = Marker(id=int(mid), corners=c4, cx=cx, cy=cy)
    return out


def read_ppm(path: Path) -> np.ndarray:
    """Read a P6 PPM file into H×W×3 uint8 RGB array."""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P6":
            raise ValueError(f"not a P6 PPM: {path}")
        # skip comments
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline().strip())
        if maxval != 255:
            raise ValueError(f"unsupported maxval {maxval}")
        data = f.read(w * h * 3)
    return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)


def detect_in_ppm(ppm_path: Path,
                   estimate_pose: bool = False) -> Dict[int, Marker]:
    """Convenience wrapper: read a PPM and detect."""
    rgb = read_ppm(ppm_path)
    return detect_markers(rgb, estimate_pose=estimate_pose)


def latest_ppm(frames_dir: Path) -> Optional[Path]:
    """Most recent frame_NNNNNN.ppm in a directory, or None."""
    ppms = sorted(frames_dir.glob("frame_*.ppm"))
    return ppms[-1] if ppms else None


if __name__ == "__main__":
    # Smoke test from CLI: detect_in_ppm <ppm_file> [pose]
    import sys
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <frame.ppm> [pose]")
        sys.exit(1)
    do_pose = (len(sys.argv) > 2 and sys.argv[2] == "pose")
    dets = detect_in_ppm(Path(sys.argv[1]), estimate_pose=do_pose)
    for mid, m in dets.items():
        print(f"id={mid}  cxy=({m.cx:.1f}, {m.cy:.1f})  corners={m.corners.tolist()}")
        if m.tvec is not None:
            print(f"   tvec={m.tvec.tolist()}  rvec={m.rvec.tolist()}")

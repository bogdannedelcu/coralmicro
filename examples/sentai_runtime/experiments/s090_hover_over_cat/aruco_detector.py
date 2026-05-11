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

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

# ArUco lives in opencv-contrib-python (cv2.aruco namespace).
import cv2


# Known world positions of markers (matches SDF).  Used by the
# calibration loop to compute drone-to-marker offsets in world frame
# without needing to query Gazebo each tick.
KNOWN_POSITIONS_M = {
    0: (+0.7, +0.5, 0.005),  # NE corner
    1: (-0.7, +0.5, 0.005),  # NW corner
    2: (-0.7, -0.5, 0.005),  # SW corner
    3: (+0.7, -0.5, 0.005),  # SE corner
}
MARKER_SIZE_M = 0.30   # physical size of one marker's square on the ground

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

#!/usr/bin/env python3
"""s176 — Python local PnP rotation-invariance prototype.

Pipeline:
  1. Read s175 frame_original.pgm (clean Gazebo render).
  2. Run cv2.aruco detect → 4 corners per marker (run ONCE on the
     clean frame, so corner-detection accuracy is held constant).
  3. For each in-plane image rotation θ in a sweep:
     a. Rotate each marker's 4 corner pixel coords around image
        center by θ (no image warp, no bilinear noise).
     b. Run three PnP variants on the rotated corners:
        - REF   = cv2.solvePnP(SOLVEPNP_IPPE_SQUARE)
        - OURS1 = our homography-decomposition PnP (1 solution)
        - OURS2 = OURS1 + R_180(v) second-solution pick
  4. For each variant, report max |Δz| across the sweep and across
     the 4 markers.  Pass if < 5 mm.
"""
from __future__ import annotations
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np

# Camera intrinsics — match sentai_aruco.cc defaults for the SIM cam.
FX, FY = 240.0, 240.0
CX, CY = 160.0, 120.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
DIST = np.zeros((5,), dtype=np.float64)

# Marker size — match the SIM scene (0.094 m square, 12cm grid).
MARKER_SIZE = 0.094

# Marker corners in marker frame (y-up convention, matches our C):
#   TL = (-L/2, +L/2, 0)
#   TR = (+L/2, +L/2, 0)
#   BR = (+L/2, -L/2, 0)
#   BL = (-L/2, -L/2, 0)
L2 = MARKER_SIZE * 0.5
OBJ_PTS = np.array([
    [-L2, +L2, 0.0],
    [+L2, +L2, 0.0],
    [+L2, -L2, 0.0],
    [-L2, -L2, 0.0],
], dtype=np.float64)


def detect_aruco(img_gray: np.ndarray) -> dict[int, np.ndarray]:
    """Run cv2.aruco detector, return {marker_id: 4x2 corner array}."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(img_gray)
    out = {}
    if ids is None:
        return out
    for i in range(len(ids)):
        mid = int(ids[i, 0])
        c = corners[i].reshape(4, 2).astype(np.float64)
        out[mid] = c
    return out


def rotate_corners(corners: np.ndarray, theta_deg: float,
                   cx: float = CX, cy: float = CY) -> np.ndarray:
    """Rotate 4x2 corner pixel coords around image center by theta."""
    t = math.radians(theta_deg)
    c, s = math.cos(t), math.sin(t)
    out = np.empty_like(corners)
    for i in range(corners.shape[0]):
        x = corners[i, 0] - cx
        y = corners[i, 1] - cy
        out[i, 0] = c * x - s * y + cx
        out[i, 1] = s * x + c * y + cy
    return out


# =====================================================================
#  PnP variant REF — OpenCV SOLVEPNP_IPPE_SQUARE (literature reference)
# =====================================================================
def pnp_ref(corners_4x2: np.ndarray) -> tuple[np.ndarray, np.ndarray, float] | None:
    ok, rvec, tvec, reproj = cv2.solvePnPGeneric(
        OBJ_PTS, corners_4x2, K, DIST,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok or len(rvec) == 0:
        return None
    # solvePnPGeneric with IPPE_SQUARE returns BOTH solutions sorted
    # by reprojection error.  Take the best.
    return rvec[0].reshape(3), tvec[0].reshape(3), float(reproj[0, 0])


# =====================================================================
#  PnP variant OURS1 — pure Python port of sentai_aruco.cc
#  aruco_pnp_from_corners() PRE-T18 (single homography decomposition,
#  no 2-solution disambiguation).  Used as the "baseline" to show
#  the regression that motivates the T18 fix.
# =====================================================================
def _dlt_homography(obj_xy: np.ndarray, img_uv: np.ndarray) -> np.ndarray | None:
    """4-point DLT.  obj_xy 4x2, img_uv 4x2 → 3x3 H or None."""
    A = np.zeros((8, 9))
    for i in range(4):
        mx, my = obj_xy[i]
        u, v = img_uv[i]
        A[2*i + 0] = [mx, my, 1, 0, 0, 0, -mx*u, -my*u, -u]
        A[2*i + 1] = [0, 0, 0, mx, my, 1, -mx*v, -my*v, -v]
    _, _, Vt = np.linalg.svd(A)
    h = Vt[-1]
    if abs(h[8]) < 1e-12:
        return None
    H = h.reshape(3, 3) / h[8]
    return H


def _R_to_rvec(R: np.ndarray) -> np.ndarray:
    """Rotation matrix → axis-angle vector (Rodrigues)."""
    rvec, _ = cv2.Rodrigues(R)
    return rvec.reshape(3)


def _reproj_err(R: np.ndarray, t: np.ndarray,
                obj_pts: np.ndarray, img_pts: np.ndarray,
                fx: float, fy: float, cx: float, cy: float) -> float | None:
    err = 0.0
    for i in range(4):
        Xc = R[0, 0]*obj_pts[i, 0] + R[0, 1]*obj_pts[i, 1] + t[0]
        Yc = R[1, 0]*obj_pts[i, 0] + R[1, 1]*obj_pts[i, 1] + t[1]
        Zc = R[2, 0]*obj_pts[i, 0] + R[2, 1]*obj_pts[i, 1] + t[2]
        if Zc <= 1e-9:
            return None
        up = fx * (Xc / Zc) + cx
        vp = fy * (Yc / Zc) + cy
        du = up - img_pts[i, 0]
        dv = vp - img_pts[i, 1]
        err += math.sqrt(du*du + dv*dv)
    return err * 0.25


def _pnp_from_corners_ours_common(corners_4x2: np.ndarray):
    """Return (R1, t1, mx, my, u, v) from homography decomposition; or None."""
    mx = OBJ_PTS[:, 0]
    my = OBJ_PTS[:, 1]
    u = corners_4x2[:, 0]
    v = corners_4x2[:, 1]
    H = _dlt_homography(np.stack([mx, my], axis=1),
                        np.stack([u, v], axis=1))
    if H is None:
        return None
    # Normalize via K^{-1}.
    Kinv = np.linalg.inv(K)
    Hn = Kinv @ H
    h1 = Hn[:, 0]
    h2 = Hn[:, 1]
    h3 = Hn[:, 2]
    n_h1 = np.linalg.norm(h1)
    n_h2 = np.linalg.norm(h2)
    if n_h1 < 1e-9 or n_h2 < 1e-9:
        return None
    lam = 2.0 / (n_h1 + n_h2)
    r1 = h1 * lam
    r2 = h2 * lam
    t  = h3 * lam
    # Gram-Schmidt: r2 ⊥ r1, then renormalize.
    r2 = r2 - np.dot(r1, r2) * r1
    n_r1 = np.linalg.norm(r1)
    n_r2 = np.linalg.norm(r2)
    if n_r1 < 1e-9 or n_r2 < 1e-9:
        return None
    r1 = r1 / n_r1
    r2 = r2 / n_r2
    r3 = np.cross(r1, r2)
    if t[2] < 0.0:
        r1 = -r1
        r2 = -r2
        r3 = -r3
        t = -t
    R1 = np.column_stack([r1, r2, r3])
    return R1, t, mx, my, u, v


def pnp_ours1(corners_4x2: np.ndarray) -> tuple[np.ndarray, np.ndarray, float] | None:
    res = _pnp_from_corners_ours_common(corners_4x2)
    if res is None:
        return None
    R1, t, mx, my, u, v = res
    err = _reproj_err(R1, t,
                      OBJ_PTS, np.stack([u, v], axis=1),
                      FX, FY, CX, CY)
    if err is None:
        return None
    return _R_to_rvec(R1), t, err


# =====================================================================
#  PnP variant OURS2 — OURS1 + R_180(v) second-solution pick.
# =====================================================================
def pnp_ours2(corners_4x2: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, int] | None:
    res = _pnp_from_corners_ours_common(corners_4x2)
    if res is None:
        return None
    R1, t, mx, my, u, v = res
    img_pts = np.stack([u, v], axis=1)
    err1 = _reproj_err(R1, t, OBJ_PTS, img_pts, FX, FY, CX, CY)
    if err1 is None:
        return None
    picked = 1
    R_best, t_best, err_best = R1, t, err1

    # Solution 2: R_180(v) * R1 around line of sight v = t/||t||.
    t_norm = float(np.linalg.norm(t))
    if t_norm > 1e-9:
        vv = t / t_norm
        R180 = 2.0 * np.outer(vv, vv) - np.eye(3)
        R2 = R180 @ R1
        err2 = _reproj_err(R2, t, OBJ_PTS, img_pts, FX, FY, CX, CY)
        if err2 is not None and err2 < err_best:
            R_best, t_best, err_best = R2, t, err2
            picked = 2
    return _R_to_rvec(R_best), t_best, err_best, picked


# =====================================================================
#  Sweep over θ + verdict
# =====================================================================
def main():
    here = Path(__file__).parent
    img_path = here / "frame_original.pgm"
    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"FAIL — cannot read {img_path}")
    print(f"[s176] image {img.shape[1]}x{img.shape[0]} loaded")

    markers = detect_aruco(img)
    if not markers:
        sys.exit("FAIL — no markers detected")
    print(f"[s176] markers detected: {sorted(markers.keys())}")

    thetas = list(range(0, 91, 5))   # 0°, 5°, …, 90°
    results: dict[str, dict[int, list[float]]] = {
        "REF":   {mid: [] for mid in markers},
        "OURS1": {mid: [] for mid in markers},
        "OURS2": {mid: [] for mid in markers},
    }
    picks_ours2: dict[int, list[int]] = {mid: [] for mid in markers}

    for theta in thetas:
        for mid, corners in markers.items():
            rotc = rotate_corners(corners, theta)
            r = pnp_ref(rotc)
            o1 = pnp_ours1(rotc)
            o2 = pnp_ours2(rotc)
            results["REF"][mid].append(  r[1][2] if r  is not None else float("nan"))
            results["OURS1"][mid].append(o1[1][2] if o1 is not None else float("nan"))
            results["OURS2"][mid].append(o2[1][2] if o2 is not None else float("nan"))
            picks_ours2[mid].append(o2[3] if o2 is not None else -1)

    print()
    print(f"  {'θ°':>4} |  mid | "
          f"{'REF z':>9} | {'OURS1 z':>9} | {'OURS2 z':>9} | pick2")
    print("  " + "-" * 60)
    for ti, theta in enumerate(thetas):
        for mid in sorted(markers.keys()):
            print(f"  {theta:>4} | {mid:>4} | "
                  f"{results['REF'][mid][ti]:>9.4f} | "
                  f"{results['OURS1'][mid][ti]:>9.4f} | "
                  f"{results['OURS2'][mid][ti]:>9.4f} | "
                  f"{picks_ours2[mid][ti]}")
        if ti < len(thetas) - 1:
            print()

    print()
    print(f"  {'algo':>6} | max |Δz vs θ=0| across markers (mm)")
    print("  " + "-" * 50)
    pass_thresh_mm = 5.0
    overall_pass = True
    for algo in ("REF", "OURS1", "OURS2"):
        max_dz_mm = 0.0
        for mid in markers:
            z0 = results[algo][mid][0]
            for z in results[algo][mid]:
                if math.isnan(z) or math.isnan(z0):
                    continue
                dz_mm = abs(z - z0) * 1000.0
                if dz_mm > max_dz_mm:
                    max_dz_mm = dz_mm
        verdict = "PASS" if max_dz_mm < pass_thresh_mm else "FAIL"
        if max_dz_mm >= pass_thresh_mm:
            overall_pass = False
        print(f"  {algo:>6} | {max_dz_mm:>7.2f}    [{verdict}]")

    print()
    if overall_pass:
        print("[s176] PASS — all three algorithms rotation-invariant")
        return 0
    else:
        print("[s176] result captured; see table above for which "
              "algorithm failed and by how much")
        return 1


if __name__ == "__main__":
    sys.exit(main())

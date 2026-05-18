#!/usr/bin/env python3
"""s177 — Corner subpixel refinement prototype on real captured frames.

Compares cv2.aruco detection with CORNER_REFINE_NONE vs
CORNER_REFINE_SUBPIX on two real frames from a s174 yaw trial:
  - frame_horiz_n4.pgm: markers axis-aligned, n=4 detected on-board
  - frame_rot45_n1.pgm: markers rotated ~45° in image, n=1 on-board

Report per-marker corner shifts and PnP z deltas.  If subpix moves
corners by &gt; 0.2 px and z by &gt; 1 mm, the corner-noise hypothesis is
confirmed and we port refinement to sentai_aruco.cc as T18-C.
"""
from __future__ import annotations
import math
from pathlib import Path

import cv2
import numpy as np

# Camera intrinsics — match sentai_aruco.cc defaults (320x240).
FX, FY = 240.0, 240.0
CX, CY = 160.0, 120.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
DIST = np.zeros((5,), dtype=np.float64)

MARKER_SIZE = 0.094
L2 = MARKER_SIZE * 0.5
OBJ_PTS = np.array([
    [-L2, +L2, 0.0],
    [+L2, +L2, 0.0],
    [+L2, -L2, 0.0],
    [-L2, -L2, 0.0],
], dtype=np.float64)


def detect(img: np.ndarray, refine: int) -> dict[int, np.ndarray]:
    """Return {id: 4x2 corners} for the given refinement method."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = refine
    if refine == cv2.aruco.CORNER_REFINE_SUBPIX:
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 30
        params.cornerRefinementMinAccuracy = 0.01
    det = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = det.detectMarkers(img)
    out = {}
    if ids is None:
        return out
    for i in range(len(ids)):
        mid = int(ids[i, 0])
        out[mid] = corners[i].reshape(4, 2).astype(np.float64)
    return out


def pnp_z(corners_4x2: np.ndarray) -> tuple[float, float] | None:
    """Return (tvec_z, reproj_err_mean_px) via SOLVEPNP_IPPE_SQUARE."""
    ok, rvec, tvec, reproj = cv2.solvePnPGeneric(
        OBJ_PTS, corners_4x2, K, DIST,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok or len(rvec) == 0:
        return None
    return float(tvec[0].reshape(3)[2]), float(reproj[0, 0])


def analyze_frame(name: str, path: Path):
    print(f"\n=== {name} ({path.name}) ===")
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"  FAIL — cannot read {path}")
        return
    print(f"  shape: {img.shape[1]}x{img.shape[0]}")

    no_refine = detect(img, cv2.aruco.CORNER_REFINE_NONE)
    sub_refine = detect(img, cv2.aruco.CORNER_REFINE_SUBPIX)
    print(f"  detect REFINE_NONE   : ids={sorted(no_refine.keys())}")
    print(f"  detect REFINE_SUBPIX : ids={sorted(sub_refine.keys())}")

    common = sorted(set(no_refine.keys()) & set(sub_refine.keys()))
    if not common:
        print("  no common markers; nothing to compare")
        return

    print(f"\n  {'id':>2} | "
          f"{'max_shift':>9} | {'mean_shift':>10} | "
          f"{'z_none (m)':>11} | {'z_sub (m)':>10} | "
          f"{'dz (mm)':>8} | {'rp_n':>5} | {'rp_s':>5}")
    print("  " + "-" * 85)

    rows = []
    for mid in common:
        c_n = no_refine[mid]
        c_s = sub_refine[mid]
        delta = c_s - c_n
        shifts = np.linalg.norm(delta, axis=1)
        max_shift = float(shifts.max())
        mean_shift = float(shifts.mean())

        r_n = pnp_z(c_n)
        r_s = pnp_z(c_s)
        z_n, rp_n = (r_n if r_n is not None else (float('nan'), float('nan')))
        z_s, rp_s = (r_s if r_s is not None else (float('nan'), float('nan')))
        dz_mm = (z_s - z_n) * 1000.0
        rows.append((mid, max_shift, mean_shift, z_n, z_s, dz_mm, rp_n, rp_s))
        print(f"  {mid:>2} | "
              f"{max_shift:>9.3f} | {mean_shift:>10.3f} | "
              f"{z_n:>+11.4f} | {z_s:>+10.4f} | "
              f"{dz_mm:>+8.2f} | {rp_n:>5.2f} | {rp_s:>5.2f}")

    print()
    return rows


def render_overlay(img_gray: np.ndarray, no_refine: dict, sub_refine: dict,
                   out_path: Path):
    """Save a PNG with both corner sets overlaid (none = red, subpix = green)."""
    bgr = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    for mid, c in no_refine.items():
        for i in range(4):
            x, y = int(round(c[i, 0])), int(round(c[i, 1]))
            cv2.circle(bgr, (x, y), 3, (0, 0, 255), 1)
    for mid, c in sub_refine.items():
        for i in range(4):
            x, y = int(round(c[i, 0])), int(round(c[i, 1]))
            cv2.circle(bgr, (x, y), 2, (0, 255, 0), -1)
    cv2.imwrite(str(out_path), bgr)


def main():
    here = Path(__file__).parent
    frames = [
        ("HORIZONTAL n=4", here / "frame_horiz_n4.pgm"),
        ("ROTATED ~45° n=1", here / "frame_rot45_n1.pgm"),
    ]
    for name, p in frames:
        rows = analyze_frame(name, p)
        img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        if img is not None:
            no_r = detect(img, cv2.aruco.CORNER_REFINE_NONE)
            su_r = detect(img, cv2.aruco.CORNER_REFINE_SUBPIX)
            render_overlay(img, no_r, su_r,
                           p.with_name(p.stem + "_overlay.png"))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Apply 3 refinement modes + PnP to both frames; measure Z deltas.

For each marker per frame:
  - NONE     : cv2.aruco corners, no refinement
  - OURS     : our_subpix.refine_corners (Python port matching what
               we'd ship in C)
  - CV2_REF  : cv2.cornerSubPix (reference)

Compare per-marker Z (m) and reproj error.  If ours produces Z
within 5mm of cv2_ref across all markers, the C port will yield
similar accuracy.
"""
from __future__ import annotations
import math
from pathlib import Path

import cv2
import numpy as np

import our_subpix

FX, FY, CX, CY = 240.0, 240.0, 160.0, 120.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float64)
DIST = np.zeros((5,), dtype=np.float64)
MARKER_SIZE = 0.094
L2 = MARKER_SIZE * 0.5
OBJ = np.array([[-L2,+L2,0], [+L2,+L2,0], [+L2,-L2,0], [-L2,-L2,0]],
               dtype=np.float64)


def pnp(corners):
    ok, rvec, tvec, reproj = cv2.solvePnPGeneric(
        OBJ, corners, K, DIST, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok or not len(rvec):
        return None
    return float(tvec[0].reshape(3)[2]), float(reproj[0, 0])


def main():
    here = Path(__file__).parent
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    det = cv2.aruco.ArucoDetector(aruco_dict, params)

    for name, fn in [("HORIZONTAL", "frame_horiz_n4.pgm"),
                     ("ROTATED45",  "frame_rot45_n1.pgm")]:
        print(f"\n=== {name} ===")
        img = cv2.imread(str(here / fn), cv2.IMREAD_GRAYSCALE)
        corners_arr, ids, _ = det.detectMarkers(img)
        if ids is None:
            print("  no markers"); continue

        print(f"  {'id':>2} | {'mode':>8} | {'z (m)':>9} | {'reproj':>6} | "
              f"{'Δz vs NONE (mm)':>16} | {'Δz vs CV2 (mm)':>15}")
        print("  " + "-" * 80)

        for k in range(len(ids)):
            mid = int(ids[k, 0])
            c0 = corners_arr[k].reshape(4, 2).astype(np.float64)
            c_ours = our_subpix.refine_corners(img, c0)
            c_cv2  = our_subpix.cv2_refine(img, c0)
            res = {}
            for mode, cc in [("NONE", c0), ("OURS", c_ours), ("CV2_REF", c_cv2)]:
                r = pnp(cc)
                res[mode] = r if r else (float('nan'), float('nan'))

            z_n, _    = res["NONE"]
            z_cv2, _  = res["CV2_REF"]
            for mode in ("NONE", "OURS", "CV2_REF"):
                z, rp = res[mode]
                dz_n  = (z - z_n) * 1000.0 if not math.isnan(z) else float('nan')
                dz_cv = (z - z_cv2) * 1000.0 if not math.isnan(z) else float('nan')
                print(f"  {mid:>2} | {mode:>8} | {z:>+9.4f} | {rp:>6.3f} | "
                      f"{dz_n:>+16.2f} | {dz_cv:>+15.2f}")


if __name__ == "__main__":
    main()

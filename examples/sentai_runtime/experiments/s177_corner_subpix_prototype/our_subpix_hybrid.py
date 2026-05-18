#!/usr/bin/env python3
"""Hybrid test — use cv2.getRectSubPix for bilinear window sampling,
then OUR gradient + saddle-point algorithm.  If this MATCHES cv2's
own cornerSubPix, the bug was in our bilinear impl, not the
algorithm.  If it still doesn't match, the algorithm differs.
"""
from __future__ import annotations
import math
from pathlib import Path

import cv2
import numpy as np


def refine_hybrid(img, x, y, win_half=2, max_iter=30, eps=0.01):
    """Use cv2.getRectSubPix for bilinear window; our gradient+solve.
    Weights match the ACTUAL cv2.cornerSubPix source (cornersubpix.cpp,
    OpenCV 4.x):
      y = (i - win.height) / win.height           (NOT *coeff)
      vy = exp(-y²)                               # → 0.368 at corner
      mask[i,j] = vy * exp(-x²)
    Gradient is RAW central difference, no /2 (scales A,b uniformly).
    Includes cv2's safety revert: if final |c - c_init| > win_half on
    either axis, the function returns the INITIAL position.
    """
    x_init, y_init = x, y
    win = 2 * win_half + 1   # 5
    big = win + 2            # 7
    inv_h = 1.0 / win_half    # cv2's normalising factor (= 0.5 for win_half=2)
    for _ in range(max_iter):
        patch = cv2.getRectSubPix(img, (big, big), (float(x), float(y)),
                                   patchType=cv2.CV_32F)
        A00 = A01 = A11 = 0.0
        bb0 = bb1 = 0.0
        for di in range(win):
            ry = (di - win_half) * inv_h
            wy = math.exp(-ry * ry)
            for dj in range(win):
                rx = (dj - win_half) * inv_h
                w = wy * math.exp(-rx * rx)
                # Raw central difference (cv2 does NOT divide by 2).
                gx = (float(patch[di + 1, dj + 2]) -
                       float(patch[di + 1, dj]))
                gy = (float(patch[di + 2, dj + 1]) -
                       float(patch[di,     dj + 1]))
                # NOTE: bb0/bb1 use window-pixel indices (j - win_h_half,
                # i - win_h_half), NOT the scaled (rx, ry).  Same as cv2.
                pix_rx = dj - win_half
                pix_ry = di - win_half
                wgxgx = w * gx * gx
                wgxgy = w * gx * gy
                wgygy = w * gy * gy
                A00 += wgxgx
                A01 += wgxgy
                A11 += wgygy
                bb0 += wgxgx * pix_rx + wgxgy * pix_ry
                bb1 += wgxgy * pix_rx + wgygy * pix_ry
        det = A00 * A11 - A01 * A01
        if abs(det) < 1e-9:
            return x, y
        inv = 1.0 / det
        dx = ( A11 * bb0 - A01 * bb1) * inv
        dy = (-A01 * bb0 + A00 * bb1) * inv
        x += dx
        y += dy
        if dx*dx + dy*dy < eps*eps:
            break
    # cv2 safety: revert if final corner moved more than win_half on
    # either axis from the initial guess.
    if abs(x - x_init) > win_half or abs(y - y_init) > win_half:
        return x_init, y_init
    return x, y


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
        corners, ids, _ = det.detectMarkers(img)
        if ids is None:
            print("  no markers"); continue
        for k in range(len(ids)):
            mid = int(ids[k, 0])
            c0 = corners[k].reshape(4, 2).astype(np.float64)
            # Ours-hybrid
            c_h = np.empty_like(c0)
            for i in range(4):
                c_h[i, 0], c_h[i, 1] = refine_hybrid(img, c0[i, 0], c0[i, 1])
            # cv2 reference
            pts32 = c0.reshape(-1, 1, 2).astype(np.float32)
            crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            c_cv2 = cv2.cornerSubPix(img, pts32, (2, 2), (-1, -1), crit
                                      ).reshape(4, 2).astype(np.float64)
            d = np.linalg.norm(c_h - c_cv2, axis=1)
            print(f"  id={mid}: HYBRID vs cv2  max={d.max():.4f} mean={d.mean():.4f}")


if __name__ == "__main__":
    main()

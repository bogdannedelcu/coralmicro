#!/usr/bin/env python3
"""Step-by-step diff vs cv2 cornerSubPix on a SINGLE corner.

Prints cv2's intermediate corner positions (via 1-iteration-at-a-time
calls) alongside ours, isolating the iteration where we diverge.
"""
from __future__ import annotations
import math
from pathlib import Path

import cv2
import numpy as np


def cv2_one_iter(img: np.ndarray, x: float, y: float) -> tuple[float, float]:
    """Run cv2.cornerSubPix for EXACTLY ONE iteration."""
    pts = np.array([[[x, y]]], dtype=np.float32)
    crit = (cv2.TERM_CRITERIA_MAX_ITER, 1, 0.0001)
    out = cv2.cornerSubPix(img, pts, (2, 2), (-1, -1), crit)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def my_one_iter(img: np.ndarray, x: float, y: float) -> tuple[float, float]:
    """One iteration of our cv2-matching saddle algorithm."""
    win_half = 2
    win = 5
    big = 7
    coeff = 1.0 / (win_half * win_half * 2.0)
    win_h_half = win // 2

    patch = cv2.getRectSubPix(img, (big, big), (float(x), float(y)),
                               patchType=cv2.CV_32F)
    A00 = A01 = A11 = 0.0
    bb0 = bb1 = 0.0
    for di in range(win):
        ry = (di - win_h_half) * coeff
        wy = math.exp(-ry * ry)
        for dj in range(win):
            rx = (dj - win_h_half) * coeff
            w = wy * math.exp(-rx * rx)
            gx = 0.5 * (float(patch[di + 1, dj + 2]) -
                        float(patch[di + 1, dj]))
            gy = 0.5 * (float(patch[di + 2, dj + 1]) -
                        float(patch[di,     dj + 1]))
            pix_rx = dj - win_h_half
            pix_ry = di - win_h_half
            wgxgx = w * gx * gx
            wgxgy = w * gx * gy
            wgygy = w * gy * gy
            A00 += wgxgx
            A01 += wgxgy
            A11 += wgygy
            bb0 += wgxgx * pix_rx + wgxgy * pix_ry
            bb1 += wgxgy * pix_rx + wgygy * pix_ry
    det = A00 * A11 - A01 * A01
    inv = 1.0 / det
    new_x = x + (A11 * bb0 - A01 * bb1) * inv
    new_y = y + (-A01 * bb0 + A00 * bb1) * inv
    return new_x, new_y


def main():
    here = Path(__file__).parent
    img = cv2.imread(str(here / "frame_rot45_n1.pgm"), cv2.IMREAD_GRAYSCALE)
    # Pick a problematic corner — first corner of id=1 from rot45.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    det = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = det.detectMarkers(img)
    idx = list(ids.flatten()).index(1)
    c0 = corners[idx].reshape(4, 2)[0]
    print(f"initial corner: ({c0[0]:.4f}, {c0[1]:.4f})")

    x, y = float(c0[0]), float(c0[1])
    cx, cy = x, y
    print(f"  iter | cv2 ({'x':>9}, {'y':>9}) | mine ({'x':>9}, {'y':>9}) | diff")
    print("  " + "-" * 78)
    for it in range(15):
        # cv2 fresh-call style: simulate 1 step from current cv2 state
        nx_cv, ny_cv = cv2_one_iter(img, cx, cy)
        # my fresh-call style: 1 step from current mine state
        nx_mn, ny_mn = my_one_iter(img, x, y)
        d = math.hypot(nx_cv - nx_mn, ny_cv - ny_mn)
        print(f"  {it:>4} | cv2 ({nx_cv:>9.5f}, {ny_cv:>9.5f}) | "
              f"mine ({nx_mn:>9.5f}, {ny_mn:>9.5f}) | {d:.4f}")
        cx, cy = nx_cv, ny_cv
        x,  y  = nx_mn, ny_mn


if __name__ == "__main__":
    main()

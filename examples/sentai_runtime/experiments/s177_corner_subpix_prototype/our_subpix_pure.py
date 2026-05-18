#!/usr/bin/env python3
"""Pure-Python cornerSubPix port — no cv2.getRectSubPix dependency.

This is the C-portable version.  Implements bilinear interp inline,
gradient + saddle solve + safety revert exactly like
cv2.cornerSubPix (OpenCV 4.x cornersubpix.cpp).

Verified against cv2: match to 0.0000 px on both s177 test frames
(see __main__ block).
"""
from __future__ import annotations
import math
from pathlib import Path

import cv2
import numpy as np


def bilinear(img: np.ndarray, x: float, y: float) -> float:
    """uint8 image → float value at sub-pixel (x, y) via bilinear."""
    H, W = img.shape
    if x < 0: x = 0.0
    if y < 0: y = 0.0
    if x > W - 1: x = W - 1.0
    if y > H - 1: y = H - 1.0
    xi = int(math.floor(x))
    yi = int(math.floor(y))
    ax = x - xi
    ay = y - yi
    # Clamp +1 indices.
    xi1 = xi + 1 if xi + 1 < W else xi
    yi1 = yi + 1 if yi + 1 < H else yi
    i00 = float(img[yi,  xi ])
    i01 = float(img[yi,  xi1])
    i10 = float(img[yi1, xi ])
    i11 = float(img[yi1, xi1])
    return ((1 - ax) * (1 - ay) * i00 +
            ax       * (1 - ay) * i01 +
            (1 - ax) * ay       * i10 +
            ax       * ay       * i11)


def get_rect_subpix(img: np.ndarray, big: int, cx: float, cy: float) -> np.ndarray:
    """Sample a big×big sub-pixel-accurate patch centered at (cx, cy).
    Match cv2.getRectSubPix: patch[(big-1)/2, (big-1)/2] = img value
    at (cx, cy) by bilinear interp.
    """
    half = (big - 1) / 2.0
    out = np.empty((big, big), dtype=np.float64)
    for i in range(big):
        sy = cy + (i - half)
        for j in range(big):
            sx = cx + (j - half)
            out[i, j] = bilinear(img, sx, sy)
    return out


def refine_corner_pure(img: np.ndarray, x: float, y: float,
                        win_half: int = 2,
                        max_iter: int = 30,
                        eps: float = 0.01) -> tuple[float, float]:
    """Pure-Python cv2.cornerSubPix equivalent (no cv2 internals)."""
    x_init, y_init = x, y
    win = 2 * win_half + 1   # 5
    big = win + 2            # 7
    inv_h = 1.0 / win_half    # 0.5 for win_half=2

    # Pre-compute the 5x5 mask once (cv2's mask).
    mask = np.empty((win, win), dtype=np.float64)
    for i in range(win):
        ry = (i - win_half) * inv_h
        vy = math.exp(-ry * ry)
        for j in range(win):
            rx = (j - win_half) * inv_h
            mask[i, j] = vy * math.exp(-rx * rx)

    eps_sq = eps * eps
    for _ in range(max_iter):
        patch = get_rect_subpix(img, big, x, y)
        A00 = A01 = A11 = 0.0
        bb0 = bb1 = 0.0
        for di in range(win):
            pix_ry = di - win_half
            for dj in range(win):
                pix_rx = dj - win_half
                m = mask[di, dj]
                tgx = patch[di + 1, dj + 2] - patch[di + 1, dj]
                tgy = patch[di + 2, dj + 1] - patch[di,     dj + 1]
                gxx = tgx * tgx * m
                gxy = tgx * tgy * m
                gyy = tgy * tgy * m
                A00 += gxx
                A01 += gxy
                A11 += gyy
                bb0 += gxx * pix_rx + gxy * pix_ry
                bb1 += gxy * pix_rx + gyy * pix_ry
        det = A00 * A11 - A01 * A01
        if abs(det) < 1e-12:
            break
        inv = 1.0 / det
        dx = ( A11 * bb0 - A01 * bb1) * inv
        dy = (-A01 * bb0 + A00 * bb1) * inv
        x += dx
        y += dy
        if dx*dx + dy*dy < eps_sq:
            break
    # cv2 safety revert.
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
            c_pure = np.empty_like(c0)
            for i in range(4):
                c_pure[i, 0], c_pure[i, 1] = refine_corner_pure(
                    img, c0[i, 0], c0[i, 1])
            pts32 = c0.reshape(-1, 1, 2).astype(np.float32)
            crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            c_cv2 = cv2.cornerSubPix(img, pts32, (2, 2), (-1, -1), crit
                                      ).reshape(4, 2).astype(np.float64)
            d = np.linalg.norm(c_pure - c_cv2, axis=1)
            print(f"  id={mid}: PURE vs cv2  max={d.max():.4f} mean={d.mean():.4f}")


if __name__ == "__main__":
    main()

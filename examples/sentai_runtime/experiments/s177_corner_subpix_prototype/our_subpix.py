#!/usr/bin/env python3
"""Pure-Python implementation of Förstner-style corner subpixel
refinement, designed to match what we'd port to sentai_aruco.cc.

Algorithm (matches OpenCV cornerSubPix in spirit):
  For each corner c = (x, y):
    Loop up to max_iter:
      For each pixel q in (2k+1)² window around c:
        g_q = ∇I(q)  (central differences)
        w_q = exp(-||q - c||² / (2σ²))
        Outer product: M_q = w_q * g_q * g_q^T
      A = Σ M_q
      b = Σ M_q * q
      c_new = A⁻¹ b
      if ||c_new - c|| < eps: break
      c = c_new

Comparison vs cv2.cornerSubPix in __main__ block.
"""
from __future__ import annotations
import math
import sys
from pathlib import Path

import cv2
import numpy as np


def refine_corner(img: np.ndarray, x: float, y: float,
                  win_half: int = 2,
                  max_iter: int = 30,
                  eps: float = 0.01) -> tuple[float, float]:
    """Refine a single corner sub-pixel.

    img        : grayscale image as uint8 ndarray, shape (H, W)
    x, y       : initial corner position (pixels, possibly fractional)
    win_half   : half-width of refinement window (window is 2*k+1 sq)
    max_iter   : max iterations
    eps        : convergence threshold in pixels

    Returns (x_refined, y_refined).
    """
    H, W = img.shape
    two_sigma_sq = 2.0 * (win_half * win_half)
    for _ in range(max_iter):
        cx = int(round(x))
        cy = int(round(y))
        if cx - win_half - 1 < 0 or cx + win_half + 1 >= W:
            return x, y
        if cy - win_half - 1 < 0 or cy + win_half + 1 >= H:
            return x, y

        A00 = A01 = A11 = 0.0
        b0 = b1 = 0.0
        for dy in range(-win_half, win_half + 1):
            py = cy + dy
            for dx in range(-win_half, win_half + 1):
                px = cx + dx
                gx = 0.5 * (int(img[py, px + 1]) - int(img[py, px - 1]))
                gy = 0.5 * (int(img[py + 1, px]) - int(img[py - 1, px]))
                dist_sq = dx*dx + dy*dy
                w = math.exp(-dist_sq / two_sigma_sq)
                wgxgx = w * gx * gx
                wgxgy = w * gx * gy
                wgygy = w * gy * gy
                A00 += wgxgx
                A01 += wgxgy
                A11 += wgygy
                qx = float(px)
                qy = float(py)
                b0 += wgxgx * qx + wgxgy * qy
                b1 += wgxgy * qx + wgygy * qy

        det = A00 * A11 - A01 * A01
        if abs(det) < 1e-9:
            return x, y
        inv = 1.0 / det
        new_x = (A11 * b0 - A01 * b1) * inv
        new_y = (-A01 * b0 + A00 * b1) * inv
        step_sq = (new_x - x)**2 + (new_y - y)**2
        x, y = new_x, new_y
        if step_sq < eps * eps:
            break
    return x, y


def _bilinear(img: np.ndarray, x: float, y: float) -> float:
    """Bilinear interp at (x, y) into uint8 image; returns float."""
    H, W = img.shape
    x = max(0.0, min(W - 1.0001, x))
    y = max(0.0, min(H - 1.0001, y))
    xi = int(x)
    yi = int(y)
    ax = x - xi
    ay = y - yi
    i00 = float(img[yi,     xi    ])
    i01 = float(img[yi,     xi + 1])
    i10 = float(img[yi + 1, xi    ])
    i11 = float(img[yi + 1, xi + 1])
    return ((1 - ax) * (1 - ay) * i00 + ax * (1 - ay) * i01 +
            (1 - ax) * ay       * i10 + ax * ay       * i11)


def refine_corner_bilinear(img: np.ndarray, x: float, y: float,
                            win_half: int = 2,
                            max_iter: int = 30,
                            eps: float = 0.01) -> tuple[float, float]:
    """Match cv2.cornerSubPix: sample a (2k+1+2)² window at SUB-PIXEL
    position cI via bilinear interp, then central-difference gradient
    over the inner (2k+1)² grid.  Accumulator uses RELATIVE
    window-centered coords (numerically stable).
    """
    H, W = img.shape
    two_sigma_sq = 2.0 * (win_half * win_half)
    win = 2 * win_half + 1               # 5
    big = win + 2                         # 7 — leaves 1-pixel margin for ∇
    for _ in range(max_iter):
        # bounds check: need (big × big) pixels around (x, y)
        if x - big/2 < 1 or x + big/2 >= W - 1:
            return x, y
        if y - big/2 < 1 or y + big/2 >= H - 1:
            return x, y
        # Bilinear-sample the (big × big) window centered at (x, y).
        # row 0..big-1, col 0..big-1 correspond to offsets
        # (-(big-1)/2 + i, -(big-1)/2 + j) from (x, y).
        off = (big - 1) / 2.0
        # We only need samples that the inner 5x5 gradient uses:
        # for inner (i, j) ∈ [0..win-1]², the gradient at (i+1, j+1)
        # of the 7x7 needs samples at (i+1, j), (i+1, j+2), (i, j+1),
        # (i+2, j+1).  Easier: just fill the whole 7x7.
        patch = np.empty((big, big), dtype=np.float64)
        for i in range(big):
            for j in range(big):
                patch[i, j] = _bilinear(img, x - off + j, y - off + i)

        A00 = A01 = A11 = 0.0
        bb0 = bb1 = 0.0
        for di in range(win):
            for dj in range(win):
                # Inner pixel at patch[di+1, dj+1]
                gx = 0.5 * (patch[di + 1, dj + 2] - patch[di + 1, dj])
                gy = 0.5 * (patch[di + 2, dj + 1] - patch[di,     dj + 1])
                # Relative offset from window center
                rx = dj - win_half
                ry = di - win_half
                dist_sq = rx*rx + ry*ry
                w = math.exp(-dist_sq / two_sigma_sq)
                wgxgx = w * gx * gx
                wgxgy = w * gx * gy
                wgygy = w * gy * gy
                A00 += wgxgx
                A01 += wgxgy
                A11 += wgygy
                bb0 += wgxgx * rx + wgxgy * ry
                bb1 += wgxgy * rx + wgygy * ry
        det = A00 * A11 - A01 * A01
        if abs(det) < 1e-9:
            return x, y
        inv = 1.0 / det
        dx_step = ( A11 * bb0 - A01 * bb1) * inv
        dy_step = (-A01 * bb0 + A00 * bb1) * inv
        x_new = x + dx_step
        y_new = y + dy_step
        step_sq = dx_step*dx_step + dy_step*dy_step
        x, y = x_new, y_new
        if step_sq < eps * eps:
            break
    return x, y


def refine_corners_bilinear(img, corners, **kw):
    out = np.empty_like(corners)
    for i in range(corners.shape[0]):
        out[i, 0], out[i, 1] = refine_corner_bilinear(
            img, float(corners[i, 0]), float(corners[i, 1]), **kw)
    return out


def refine_corners(img: np.ndarray, corners: np.ndarray,
                   **kw) -> np.ndarray:
    """Apply refine_corner to each row of an N×2 array."""
    out = np.empty_like(corners)
    for i in range(corners.shape[0]):
        out[i, 0], out[i, 1] = refine_corner(img,
                                              float(corners[i, 0]),
                                              float(corners[i, 1]),
                                              **kw)
    return out


def cv2_refine(img: np.ndarray, corners: np.ndarray,
               win_half: int = 2, max_iter: int = 30,
               eps: float = 0.01) -> np.ndarray:
    """cv2.cornerSubPix wrapper — reference implementation."""
    pts = corners.reshape(-1, 1, 2).astype(np.float32)
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
            max_iter, eps)
    out = cv2.cornerSubPix(img, pts,
                            (win_half, win_half), (-1, -1), crit)
    return out.reshape(-1, 2).astype(np.float64)


if __name__ == "__main__":
    # Sanity check: detect markers on both frames with NO refine, then
    # refine with (a) our Python; (b) cv2.cornerSubPix.  Print deltas.
    here = Path(__file__).parent
    frames = [
        ("HORIZONTAL", here / "frame_horiz_n4.pgm"),
        ("ROTATED45",  here / "frame_rot45_n1.pgm"),
    ]
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    det = cv2.aruco.ArucoDetector(aruco_dict, params)

    for name, p in frames:
        print(f"\n=== {name} ===")
        img = cv2.imread(str(p), cv2.IMREAD_GRAYSCALE)
        corners, ids, _ = det.detectMarkers(img)
        if ids is None:
            print("  no markers")
            continue
        for i, mid_arr in enumerate(ids):
            mid = int(mid_arr[0])
            c0 = corners[i].reshape(4, 2).astype(np.float64)
            c_int  = refine_corners(img, c0)                # integer-center window
            c_bl   = refine_corners_bilinear(img, c0)       # cv2-style bilinear window
            c_cv2  = cv2_refine(img, c0)
            d_int  = np.linalg.norm(c_int - c_cv2, axis=1)
            d_bl   = np.linalg.norm(c_bl  - c_cv2, axis=1)
            print(f"  id={mid}")
            print(f"    init→cv2     max={np.linalg.norm(c_cv2 - c0, axis=1).max():.3f}")
            print(f"    INT vs cv2   max={d_int.max():.4f}  mean={d_int.mean():.4f}")
            print(f"    BL  vs cv2   max={d_bl.max():.4f}  mean={d_bl.mean():.4f}")

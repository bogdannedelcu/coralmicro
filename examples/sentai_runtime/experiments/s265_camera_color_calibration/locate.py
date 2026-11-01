#!/usr/bin/env python3
"""Locate the monitor in each camera: Gray-code structured light -> homography.

For every camera: show white, black, then NX vertical and NY horizontal Gray-code
stripe patterns, capture each, decode the monitor cell (cx, cy) per camera pixel,
fit a homography monitor_px -> camera_px on the cell centroids, and report the
screen rectangle the camera sees.  Finally the intersection of the cameras' rectangles
is written to <out>/locate.json together with per-camera H, inverse H and residuals.

Usage: python3 locate.py --out iter1 [--cams 0,1] [--nx 5 --ny 4] [--settle 1.0]
"""
import argparse
import io
import json
import os
import subprocess
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from board_repl import Repl  # noqa: E402

W, H = 1920, 1080


def gray_patterns(nx, ny, out):
    """Write the stripe PNGs; return [(name, path)] in display order."""
    os.makedirs(out, exist_ok=True)
    pats = [('white', 255), ('black', 0)]
    paths = []
    for name, v in pats:
        p = os.path.join(out, 'sl_%s.png' % name)
        Image.new('L', (W, H), v).save(p)
        paths.append((name, p))
    xs = np.arange(W)
    ys = np.arange(H)
    cx = (xs * (1 << nx)) // W           # cell index per column
    cy = (ys * (1 << ny)) // H
    gx = cx ^ (cx >> 1)                   # Gray code
    gy = cy ^ (cy >> 1)
    for b in range(nx - 1, -1, -1):       # MSB first
        row = (((gx >> b) & 1) * 255).astype(np.uint8)
        img = np.repeat(row[None, :], H, axis=0)
        p = os.path.join(out, 'sl_x%d.png' % b)
        Image.fromarray(img, 'L').save(p)
        paths.append(('x%d' % b, p))
    for b in range(ny - 1, -1, -1):
        col = (((gy >> b) & 1) * 255).astype(np.uint8)
        img = np.repeat(col[:, None], W, axis=1)
        p = os.path.join(out, 'sl_y%d.png' % b)
        Image.fromarray(img, 'L').save(p)
        paths.append(('y%d' % b, p))
    return paths


def show(path):
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--png', path],
                   check=True, stdout=subprocess.DEVNULL)


def luma(jpg):
    return np.asarray(Image.open(io.BytesIO(jpg)).convert('L'), dtype=np.float32)


def gray_to_bin(g):
    b = g.copy()
    shift = 1
    while True:
        s = b >> shift
        if not s.any():
            break
        b ^= s
        shift <<= 1
    return b


def decode(caps, nx, ny, min_contrast=40.0):
    white, black = caps['white'], caps['black']
    thr = (white + black) / 2.0
    valid = (white - black) > min_contrast
    gx = np.zeros(white.shape, np.int32)
    for b in range(nx - 1, -1, -1):
        gx = (gx << 1) | (caps['x%d' % b] > thr).astype(np.int32)
    gy = np.zeros(white.shape, np.int32)
    for b in range(ny - 1, -1, -1):
        gy = (gy << 1) | (caps['y%d' % b] > thr).astype(np.int32)
    return gray_to_bin(gx), gray_to_bin(gy), valid


def centroids(cx, cy, valid, nx, ny, min_px=60):
    """Return (monitor_xy, camera_xy) arrays of matched cell centres."""
    hh, ww = valid.shape
    yy, xx = np.mgrid[0:hh, 0:ww]
    code = (cx * (1 << ny) + cy)
    code = np.where(valid, code, -1).ravel()
    xs, ys = xx.ravel(), yy.ravel()
    mon, cam = [], []
    ncell = (1 << nx) * (1 << ny)
    counts = np.bincount(code[code >= 0], minlength=ncell)
    sx = np.bincount(code[code >= 0], weights=xs[code >= 0], minlength=ncell)
    sy = np.bincount(code[code >= 0], weights=ys[code >= 0], minlength=ncell)
    for c in np.nonzero(counts >= min_px)[0]:
        ix, iy = divmod(int(c), 1 << ny)
        # interior cells only: all 4 neighbours present (edge cells are clipped by the FOV)
        nb = [(ix + dx) * (1 << ny) + iy + dy for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
        if any(n < 0 or n >= ncell or counts[n] < min_px for n in nb):
            continue
        mon.append(((ix + 0.5) * W / (1 << nx), (iy + 0.5) * H / (1 << ny)))
        cam.append((sx[c] / counts[c], sy[c] / counts[c]))
    return np.array(mon), np.array(cam)


def fit_h(src, dst):
    """DLT homography src -> dst with Hartley normalisation."""
    def norm(p):
        m = p.mean(0)
        s = np.sqrt(2) / np.sqrt(((p - m) ** 2).sum(1)).mean()
        T = np.array([[s, 0, -s * m[0]], [0, s, -s * m[1]], [0, 0, 1]])
        return (T @ np.c_[p, np.ones(len(p))].T).T, T
    sn, Ts = norm(src)
    dn, Td = norm(dst)
    A = []
    for (x, y, _), (u, v, _) in zip(sn, dn):
        A.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        A.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, vt = np.linalg.svd(np.array(A))
    Hn = vt[-1].reshape(3, 3)
    Hm = np.linalg.inv(Td) @ Hn @ Ts
    return Hm / Hm[2, 2]


def apply_h(Hm, pts):
    p = np.c_[pts, np.ones(len(pts))] @ Hm.T
    return p[:, :2] / p[:, 2:3]


def robust_fit(mon, cam):
    Hm = fit_h(mon, cam)
    res = np.linalg.norm(apply_h(Hm, mon) - cam, axis=1)
    keep = res < max(3.0 * np.median(res), 2.0)
    Hm = fit_h(mon[keep], cam[keep])
    res = np.linalg.norm(apply_h(Hm, mon[keep]) - cam[keep], axis=1)
    return Hm, res, int(keep.sum()), len(mon)


def visible_rect(Hinv, cw, ch):
    """Screen rectangle fully inside the camera frame (inner axis-aligned box)."""
    corners = apply_h(Hinv, np.array([[0, 0], [cw, 0], [cw, ch], [0, ch]], float))
    xs, ys = corners[:, 0], corners[:, 1]
    # inner box: between the two left-most x's and two right-most x's, same for y
    xs_s, ys_s = np.sort(xs), np.sort(ys)
    x0, x1 = max(xs_s[1], 0), min(xs_s[2], W)
    y0, y1 = max(ys_s[1], 0), min(ys_s[2], H)
    return [float(x0), float(y0), float(x1 - x0), float(y1 - y0)], corners.tolist()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--cams', default='0,1')
    ap.add_argument('--nx', type=int, default=5)
    ap.add_argument('--ny', type=int, default=4)
    ap.add_argument('--settle', type=float, default=1.0, help='s between show and capture')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    pats = gray_patterns(a.nx, a.ny, os.path.join(HERE, 'patterns'))
    log = open(os.path.join(a.out, 'locate.log'), 'a')
    r = Repl(log=log)
    result = {'nx': a.nx, 'ny': a.ny, 'screen': [W, H], 'cams': {}}
    rects = []
    for cam in [int(c) for c in a.cams.split(',')]:
        r.select(cam, 1.5)
        caps = {}
        t0 = time.time()
        for name, path in pats:
            show(path)
            time.sleep(a.settle)
            jpg = r.snap_cur('/s265_sl.jpg', 90)
            open(os.path.join(a.out, 'sl_cam%d_%s.jpg' % (cam, name)), 'wb').write(jpg)
            caps[name] = luma(jpg)
        print('cam%d: %d captures in %.1f s' % (cam, len(pats), time.time() - t0))
        cx, cy, valid = decode(caps, a.nx, a.ny)
        mon, camxy = centroids(cx, cy, valid, a.nx, a.ny)
        if len(mon) < 6:
            print('cam%d: only %d cells decoded, giving up' % (cam, len(mon)))
            continue
        Hm, res, kept, total = robust_fit(mon, camxy)
        Hinv = np.linalg.inv(Hm)
        Hinv /= Hinv[2, 2]
        ch, cw = valid.shape
        rect, corners = visible_rect(Hinv, cw, ch)
        rects.append(rect)
        result['cams'][str(cam)] = {
            'H_mon_to_cam': Hm.tolist(), 'H_cam_to_mon': Hinv.tolist(),
            'cells_used': kept, 'cells_found': total,
            'residual_px_median': float(np.median(res)), 'residual_px_max': float(res.max()),
            'valid_fraction': float(valid.mean()),
            'cam_corners_on_screen': corners, 'visible_screen_rect': rect,
            'cam_size': [cw, ch]}
        print('cam%d: cells %d/%d, residual median %.2f px max %.2f px, valid %.0f %%, sees screen '
              'rect x=%.0f y=%.0f w=%.0f h=%.0f' % (cam, kept, total, np.median(res), res.max(),
                                                   100 * valid.mean(), *rect))
        # overlay: decoded cell map over the white capture
        img = Image.fromarray(caps['white'].astype(np.uint8)).convert('RGB')
        rgb = np.asarray(img).copy()
        hue = ((cx * 37 + cy * 91) % 255).astype(np.uint8)
        rgb[valid] = np.stack([hue[valid], 255 - hue[valid], (cx[valid] * 8) % 255], 1)
        img = Image.fromarray(rgb)
        d = ImageDraw.Draw(img)
        for p in camxy:
            d.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3], outline=(255, 255, 255))
        img.save(os.path.join(a.out, 'locate_cam%d_overlay.png' % cam))
    r.close()
    if len(rects) == 2:
        x0 = max(rects[0][0], rects[1][0])
        y0 = max(rects[0][1], rects[1][1])
        x1 = min(rects[0][0] + rects[0][2], rects[1][0] + rects[1][2])
        y1 = min(rects[0][1] + rects[0][3], rects[1][1] + rects[1][3])
        result['common_rect'] = [x0, y0, max(0.0, x1 - x0), max(0.0, y1 - y0)]
        print('common screen rect: x=%.0f y=%.0f w=%.0f h=%.0f' % tuple(result['common_rect']))
    json.dump(result, open(os.path.join(a.out, 'locate.json'), 'w'), indent=1)
    show(os.path.join(HERE, 'patterns', 'sl_black.png'))


if __name__ == '__main__':
    main()

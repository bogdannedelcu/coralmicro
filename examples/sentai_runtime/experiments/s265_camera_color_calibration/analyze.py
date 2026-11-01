#!/usr/bin/env python3
"""Measure the grid patches in a capture and compare them with the displayed sRGB.

Inputs: <out>/locate.json (homographies), patterns/pattern_grid.json (patch rectangles
on screen), <out>/<tag>_cam<N>.jpg.  For every patch the screen rectangle is mapped into
the camera image through H_mon_to_cam; the inner 40 % of the mapped quadrilateral (axis
aligned bbox of it, shrunk) is averaged.  Output: <out>/<tag>_patches_cam<N>.json,
<out>/<tag>_analysis_cam<N>.png (overlay) and a terminal table.

Error metric: per-patch RGB delta (measured - target) and CIEDE2000 (sRGB -> Lab, D65)
when the `colour` package is absent we use CIE76 (Euclidean Lab distance), labelled dE76.

Usage: python3 analyze.py --out iter1 --tag grid_before [--cams 0,1] [--shrink 0.3]
"""
import argparse
import io
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))


def srgb_to_lab(rgb):
    c = np.asarray(rgb, float) / 255.0
    lin = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    M = np.array([[0.4124564, 0.3575761, 0.1804375],
                  [0.2126729, 0.7151522, 0.0721750],
                  [0.0193339, 0.1191920, 0.9503041]])
    xyz = lin @ M.T
    xyz = xyz / np.array([0.95047, 1.0, 1.08883])
    f = np.where(xyz > 0.008856, np.cbrt(xyz), 7.787 * xyz + 16.0 / 116.0)
    L = 116 * f[..., 1] - 16
    a = 500 * (f[..., 0] - f[..., 1])
    b = 200 * (f[..., 1] - f[..., 2])
    return np.stack([L, a, b], -1)


def de76(rgb1, rgb2):
    return float(np.linalg.norm(srgb_to_lab(rgb1) - srgb_to_lab(rgb2)))


def de2000_lab(lab1, lab2, kL=1.0, kC=1.0, kH=1.0):
    """CIEDE2000 (Sharma, Wu, Dalal 2005).  Validated against their test data: pair 1
    (50, 2.6772, -79.7751) vs (50, 0, -82.7485) -> 2.0425; pair 17 (50, 2.5, 0) vs
    (73, 25, -18) -> 27.1492."""
    L1, a1, b1 = lab1
    L2, a2, b2 = lab2
    C1, C2 = np.hypot(a1, b1), np.hypot(a2, b2)
    Cm = (C1 + C2) / 2
    G = 0.5 * (1 - np.sqrt(Cm ** 7 / (Cm ** 7 + 25.0 ** 7)))
    a1p, a2p = (1 + G) * a1, (1 + G) * a2
    C1p, C2p = np.hypot(a1p, b1), np.hypot(a2p, b2)
    h1p = np.degrees(np.arctan2(b1, a1p)) % 360 if (a1p != 0 or b1 != 0) else 0.0
    h2p = np.degrees(np.arctan2(b2, a2p)) % 360 if (a2p != 0 or b2 != 0) else 0.0
    dLp = L2 - L1
    dCp = C2p - C1p
    if C1p * C2p == 0:
        dhp = 0.0
    elif abs(h2p - h1p) <= 180:
        dhp = h2p - h1p
    elif h2p - h1p > 180:
        dhp = h2p - h1p - 360
    else:
        dhp = h2p - h1p + 360
    dHp = 2 * np.sqrt(C1p * C2p) * np.sin(np.radians(dhp / 2))
    Lpm = (L1 + L2) / 2
    Cpm = (C1p + C2p) / 2
    if C1p * C2p == 0:
        hpm = h1p + h2p
    elif abs(h1p - h2p) <= 180:
        hpm = (h1p + h2p) / 2
    elif h1p + h2p < 360:
        hpm = (h1p + h2p + 360) / 2
    else:
        hpm = (h1p + h2p - 360) / 2
    T = (1 - 0.17 * np.cos(np.radians(hpm - 30)) + 0.24 * np.cos(np.radians(2 * hpm))
         + 0.32 * np.cos(np.radians(3 * hpm + 6)) - 0.20 * np.cos(np.radians(4 * hpm - 63)))
    dtheta = 30 * np.exp(-((hpm - 275) / 25) ** 2)
    RC = 2 * np.sqrt(Cpm ** 7 / (Cpm ** 7 + 25.0 ** 7))
    SL = 1 + 0.015 * (Lpm - 50) ** 2 / np.sqrt(20 + (Lpm - 50) ** 2)
    SC = 1 + 0.045 * Cpm
    SH = 1 + 0.015 * Cpm * T
    RT = -np.sin(np.radians(2 * dtheta)) * RC
    return float(np.sqrt((dLp / (kL * SL)) ** 2 + (dCp / (kC * SC)) ** 2 + (dHp / (kH * SH)) ** 2
                         + RT * (dCp / (kC * SC)) * (dHp / (kH * SH))))


def de2000(rgb1, rgb2):
    return de2000_lab(srgb_to_lab(rgb1), srgb_to_lab(rgb2))


def _selftest():
    assert abs(de2000_lab((50, 2.6772, -79.7751), (50, 0, -82.7485)) - 2.0425) < 1e-3
    assert abs(de2000_lab((50, 2.5, 0), (73, 25, -18)) - 27.1492) < 1e-3
    assert abs(de2000_lab((50, 2.5, 0), (50, 3.1736, 0.5854)) - 1.0) < 1e-3


_selftest()


def apply_h(Hm, pts):
    p = np.c_[pts, np.ones(len(pts))] @ np.asarray(Hm).T
    return p[:, :2] / p[:, 2:3]


def measure(img, Hm, patches, shrink=0.3):
    """Return list of dicts with mean RGB per patch; None if the patch is out of frame."""
    arr = np.asarray(img.convert('RGB'), dtype=np.float32)
    h, w = arr.shape[:2]
    out = []
    for p in patches:
        x, y, pw, ph = p['rect']
        # shrink the screen rectangle, then map its corners
        sx, sy = x + pw * shrink / 2, y + ph * shrink / 2
        sw, sh = pw * (1 - shrink), ph * (1 - shrink)
        quad = apply_h(Hm, np.array([[sx, sy], [sx + sw, sy], [sx + sw, sy + sh], [sx, sy + sh]]))
        x0, y0 = quad.min(0)
        x1, y1 = quad.max(0)
        if x0 < 2 or y0 < 2 or x1 > w - 2 or y1 > h - 2:
            out.append(None)
            continue
        roi = arr[int(y0):int(y1) + 1, int(x0):int(x1) + 1]
        mean = roi.reshape(-1, 3).mean(0)
        std = roi.reshape(-1, 3).std(0)
        out.append({'name': p['name'], 'target': p['srgb'], 'mean': mean.round(1).tolist(),
                    'std': std.round(1).tolist(), 'bbox': [float(x0), float(y0), float(x1), float(y1)],
                    'n_px': int(roi.shape[0] * roi.shape[1]),
                    'dE76': round(de76(mean, p['srgb']), 2),
                    'dE00': round(de2000(mean, p['srgb']), 2)})
    return out


def report(cam, meas):
    rows = [m for m in meas if m]
    print('cam%d: %d of %d patches in frame' % (cam, len(rows), len(meas)))
    print('  %-14s %-15s %-18s %-16s %5s %5s' % ('patch', 'target RGB', 'measured RGB', 'delta', 'dE76', 'dE00'))
    for m in rows:
        d = [round(a - b, 1) for a, b in zip(m['mean'], m['target'])]
        print('  %-14s %-15s %-18s %-16s %5.1f %5.1f' % (m['name'], tuple(m['target']),
                                                         tuple(int(round(v)) for v in m['mean']), tuple(d),
                                                         m['dE76'], m['dE00']))
    greys = [m for m in rows if m['name'] in ('white', 'neutral 8', 'neutral 6.5',
                                              'neutral 5', 'neutral 3.5', 'black')]
    if rows:
        e00 = sorted(m['dE00'] for m in rows)
        print('  mean dE76 all %.1f | greys %.1f | max %.1f (%s)' % (
            np.mean([m['dE76'] for m in rows]),
            np.mean([m['dE76'] for m in greys]) if greys else float('nan'),
            max(m['dE76'] for m in rows), max(rows, key=lambda m: m['dE76'])['name']))
        print('  mean dE00 all %.1f | greys %.1f | 95th pct %.1f | max %.1f (%s)' % (
            np.mean(e00), np.mean([m['dE00'] for m in greys]) if greys else float('nan'),
            e00[int(0.95 * (len(e00) - 1))], e00[-1], max(rows, key=lambda m: m['dE00'])['name']))
    return rows


def overlay(img, meas, path):
    im = img.convert('RGB').copy()
    d = ImageDraw.Draw(im)
    for m in meas:
        if not m:
            continue
        x0, y0, x1, y1 = m['bbox']
        d.rectangle([x0, y0, x1, y1], outline=(255, 255, 255), width=2)
        # target swatch in the corner of the box
        d.rectangle([x0, y0, x0 + 18, y0 + 18], fill=tuple(int(v) for v in m['target']))
        d.text((x0 + 22, y0 + 2), '%s dE %.0f' % (m['name'], m['dE76']), fill=(255, 255, 255))
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--cams', default='0,1')
    ap.add_argument('--shrink', type=float, default=0.3)
    ap.add_argument('--grid-json', default=os.path.join(HERE, 'patterns', 'pattern_grid.json'))
    a = ap.parse_args()
    loc = json.load(open(os.path.join(a.out, 'locate.json')))
    patches = json.load(open(a.grid_json))
    summary = {}
    for cam in [int(c) for c in a.cams.split(',')]:
        jp = os.path.join(a.out, '%s_cam%d.jpg' % (a.tag, cam))
        img = Image.open(jp)
        meas = measure(img, loc['cams'][str(cam)]['H_mon_to_cam'], patches, a.shrink)
        rows = report(cam, meas)
        json.dump(meas, open(os.path.join(a.out, '%s_patches_cam%d.json' % (a.tag, cam)), 'w'), indent=1)
        overlay(img, meas, os.path.join(a.out, '%s_analysis_cam%d.png' % (a.tag, cam)))
        summary['cam%d' % cam] = {'n': len(rows), 'mean_dE76': float(np.mean([m['dE76'] for m in rows])) if rows else None}
    json.dump(summary, open(os.path.join(a.out, '%s_summary.json' % a.tag), 'w'), indent=1)


if __name__ == '__main__':
    main()

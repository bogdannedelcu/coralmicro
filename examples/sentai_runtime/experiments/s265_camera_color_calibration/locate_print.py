#!/usr/bin/env python3
"""Locate the printed chart (patterns/colorchecker_a4.pdf) in each camera from its four
black corner fiducials (12 mm squares with a white centre dot) and write a locate.json in
the same format as locate.py (H_mon_to_cam maps *chart millimetres* to camera pixels), plus a
pattern json with the patch rectangles in mm so analyze.py / tune_colors.py work unchanged.

Detection: dark connected components of plausible size whose centre is bright (the white dot),
on a downscaled luma image; the four are ordered by their position relative to the centroid.
The chart may appear rotated by 180 degrees (cam0 is mounted upside down relative to cam1):
both orderings are tried and the one whose patch colours best match the reference wins.

Usage: python3 locate_print.py --out iter2_print [--cams 0,1]
"""
import argparse
import io
import json
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from board_repl import Repl  # noqa: E402
from locate import fit_h, apply_h  # noqa: E402
from analyze import measure  # noqa: E402

GEOM = json.load(open(os.path.join(HERE, 'patterns', 'colorchecker_a4.json')))


def chart_geometry():
    """Fiducial centres and patch rects in chart mm, origin = top-left of patch 1."""
    pm, gap, fid = GEOM['patch_mm'], GEOM['gap_mm'], GEOM['fiducial_mm']
    cw = GEOM['cols'] * pm + (GEOM['cols'] - 1) * gap
    ch = GEOM['rows'] * pm + (GEOM['rows'] - 1) * gap
    off = fid / 2 + 4
    fids = np.array([[-off, -off], [cw + off, -off], [-off, ch + off], [cw + off, ch + off]], float)  # TL TR BL BR
    patches = [{'name': p['name'], 'srgb': p['srgb'], 'rect': p['rect_mm']} for p in GEOM['patches']]
    return fids, patches


def label(mask):
    """4-connected component labelling without scipy (flood fill on a small image)."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    n = 0
    for y0 in range(h):
        for x0 in range(w):
            if mask[y0, x0] and lab[y0, x0] == 0:
                n += 1
                stack = [(y0, x0)]
                lab[y0, x0] = n
                while stack:
                    y, x = stack.pop()
                    for yy, xx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                        if 0 <= yy < h and 0 <= xx < w and mask[yy, xx] and lab[yy, xx] == 0:
                            lab[yy, xx] = n
                            stack.append((yy, xx))
    return lab, n


def find_fiducials(img, scale=0.5):
    """Dark squares with a bright centre dot, found by local contrast (the frame also holds
    large black areas and strong lighting gradients, so a global threshold does not work)."""
    g = np.asarray(img.convert('L').resize((int(img.width * scale), int(img.height * scale))), np.float32)
    k = 25
    pad = np.pad(g, k, mode='edge')
    cs = np.cumsum(np.cumsum(pad, 0), 1)
    cs = np.pad(cs, ((1, 0), (1, 0)))
    h, w = g.shape
    local = (cs[2 * k + 1:2 * k + 1 + h, 2 * k + 1:2 * k + 1 + w] - cs[:h, 2 * k + 1:2 * k + 1 + w]
             - cs[2 * k + 1:2 * k + 1 + h, :w] + cs[:h, :w]) / float((2 * k + 1) ** 2)
    dark = (g < 0.8 * local) & (local > 40)
    lab, n = label(dark)
    cands = []
    for i in range(1, n + 1):
        ys, xs = np.nonzero(lab == i)
        if not (30 <= len(xs) <= 600):
            continue
        hh, ww = np.ptp(ys) + 1, np.ptp(xs) + 1
        if not (0.65 < ww / hh < 1.5) or len(xs) < 0.55 * ww * hh:
            continue
        cands.append((xs.mean() / scale, ys.mean() / scale, len(xs)))
    if len(cands) < 4:
        return [(c[0], c[1]) for c in cands]
    # the fiducials are the four most outlying compact dark squares: take, for each corner of
    # the candidates' bounding box, the candidate closest to it (the white dot is too small to
    # be a reliable test at low light)
    P = np.array([(c[0], c[1]) for c in cands])
    x0, y0, x1, y1 = P[:, 0].min(), P[:, 1].min(), P[:, 0].max(), P[:, 1].max()
    picked = []
    for cx, cy in ((x0, y0), (x1, y0), (x0, y1), (x1, y1)):
        k = int(np.argmin((P[:, 0] - cx) ** 2 + (P[:, 1] - cy) ** 2))
        if k not in picked:
            picked.append(k)
    return [tuple(P[k]) for k in picked]


def order_corners(pts):
    pts = np.array(pts, float)
    c = pts.mean(0)
    tl = pts[np.argmin((pts - c).sum(1))]
    br = pts[np.argmax((pts - c).sum(1))]
    d = (pts - c)[:, 0] - (pts - c)[:, 1]
    tr = pts[np.argmax(d)]
    bl = pts[np.argmin(d)]
    return np.array([tl, tr, bl, br])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--cams', default='0,1')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    fids_mm, patches = chart_geometry()
    json.dump(patches, open(os.path.join(a.out, 'pattern_print.json'), 'w'), indent=1)
    r = Repl()
    res = {'screen': 'chart mm', 'cams': {}}
    for cam in [int(c) for c in a.cams.split(',')]:
        r.select(cam, 1.5)
        jpg = r.snap_cur('/s265_lp.jpg', 95)
        open(os.path.join(a.out, 'locate_print_cam%d.jpg' % cam), 'wb').write(jpg)
        img = Image.open(io.BytesIO(jpg))
        pts = find_fiducials(img)
        if len(pts) < 4:
            print('cam%d: only %d fiducials found %s — move the chart so all four corners are in view' % (cam, len(pts), pts))
            continue
        cam_pts = order_corners(pts)
        # orientation: the grey row must go white -> black in luminance (robust to any colour
        # cast, unlike a dE match); also report the dE of the identification
        best = None
        greys = ('white', 'neutral 8', 'neutral 6.5', 'neutral 5', 'neutral 3.5', 'black')
        for flip in (False, True):
            src = fids_mm if not flip else fids_mm[[3, 2, 1, 0]]
            Hm = fit_h(src, cam_pts)
            meas = {m['name']: m for m in measure(img, Hm, patches, 0.4) if m}
            lum = [np.mean(meas[g]['mean']) for g in greys if g in meas]
            mono = sum(1 for a, b in zip(lum, lum[1:]) if a > b)      # 5 = perfectly ordered
            score = np.mean([m['dE76'] for m in meas.values()]) if meas else 1e9
            if best is None or (mono, -score) > (best[0], -best[1]):
                best = (mono, score, Hm, flip)
        mono, score, Hm, flip = best
        Hinv = np.linalg.inv(Hm); Hinv /= Hinv[2, 2]
        res['cams'][str(cam)] = {'H_mon_to_cam': Hm.tolist(), 'H_cam_to_mon': Hinv.tolist(),
                                 'fiducials_cam': cam_pts.tolist(), 'rotated_180': flip,
                                 'cam_size': [img.width, img.height], 'match_dE76': float(score)}
        print('cam%d: 4 fiducials at %s, chart %s, grey row ordered %d/5, mean dE76 %.1f' % (
            cam, [(int(x), int(y)) for x, y in cam_pts], 'rotated 180' if flip else 'upright', mono, score))
    r.close()
    json.dump(res, open(os.path.join(a.out, 'locate.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Colour-shading map: show a uniform grey on the whole monitor, capture each camera and
compute per-cell mean R, G, B plus the R/G and B/G ratios over a cells_x x cells_y grid.
A camera with a flat response gives ratios constant across the frame; a lens / IR-filter
problem shows up as a spatial gradient in B/G or R/G.  Output: <out>/shading_cam<N>.jpg,
<out>/shading.json, <out>/shading_map.png (heat maps of B/G and R/G, both cameras).

Usage: python3 shading_map.py --out iter1 [--grey 150] [--cells 8x6]
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


def cell_stats(img, cx, cy):
    arr = np.asarray(img.convert('RGB'), np.float32)
    h, w = arr.shape[:2]
    out = np.zeros((cy, cx, 3))
    for j in range(cy):
        for i in range(cx):
            roi = arr[j * h // cy:(j + 1) * h // cy, i * w // cx:(i + 1) * w // cx]
            out[j, i] = roi.reshape(-1, 3).mean(0)
    return out


def heat(ratio, lo, hi, size):
    """Blue (low) .. grey (1.0) .. red (high) heat map, upscaled to ``size``."""
    t = np.clip((ratio - lo) / (hi - lo), 0, 1)
    r = (255 * t).astype(np.uint8)
    b = (255 * (1 - t)).astype(np.uint8)
    g = (255 * (1 - 2 * np.abs(t - 0.5))).astype(np.uint8)
    return Image.fromarray(np.stack([r, g, b], -1)).resize(size, Image.NEAREST)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--grey', type=int, default=150)
    ap.add_argument('--hex', help='solid colour instead of grey, e.g. b3b300 (yellow at 70 percent)')
    ap.add_argument('--tag', default='')
    ap.add_argument('--cells', default='8x6')
    ap.add_argument('--cams', default='0,1')
    a = ap.parse_args()
    cx, cy = (int(v) for v in a.cells.split('x'))
    colour = a.hex or '%02x%02x%02x' % ((a.grey,) * 3)
    tag = a.tag or ('' if not a.hex else '_' + a.hex)
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--solid', colour],
                   check=True, stdout=subprocess.DEVNULL)
    time.sleep(2.0)
    r = Repl()
    res = {'grey': a.grey, 'cells': [cx, cy], 'cams': {}}
    maps = []
    for cam in [int(c) for c in a.cams.split(',')]:
        r.select(cam, 1.5)
        jpg = r.snap_cur('/s265_sh.jpg', 95)
        open(os.path.join(a.out, 'shading%s_cam%d.jpg' % (tag, cam)), 'wb').write(jpg)
        st = cell_stats(Image.open(io.BytesIO(jpg)), cx, cy)
        rg = st[..., 0] / st[..., 1]
        bg = st[..., 2] / st[..., 1]
        res['cams'][str(cam)] = {'mean_rgb': st.round(1).tolist(), 'r_over_g': rg.round(3).tolist(),
                                 'b_over_g': bg.round(3).tolist(),
                                 'g_min_max': [float(st[..., 1].min()), float(st[..., 1].max())],
                                 'rg_min_max': [float(rg.min()), float(rg.max())],
                                 'bg_min_max': [float(bg.min()), float(bg.max())]}
        print('cam%d: G %.0f..%.0f (falloff %.0f %%), R/G %.3f..%.3f, B/G %.3f..%.3f' % (
            cam, st[..., 1].min(), st[..., 1].max(), 100 * (1 - st[..., 1].min() / st[..., 1].max()),
            rg.min(), rg.max(), bg.min(), bg.max()))
        maps.append((cam, rg, bg))
    r.close()
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--grid', '--region', '843,229,555,653',
                    '--layout', '4x6'], check=True, stdout=subprocess.DEVNULL)
    json.dump(res, open(os.path.join(a.out, 'shading%s.json' % tag), 'w'), indent=1)
    # sheet: per camera, the capture, B/G map, R/G map (ratio scale 0.7 .. 1.3)
    tw, th = 320, 240
    sheet = Image.new('RGB', (3 * tw + 40, len(maps) * (th + 30) + 10), (30, 30, 30))
    d = ImageDraw.Draw(sheet)
    for k, (cam, rg, bg) in enumerate(maps):
        y = 10 + k * (th + 30)
        img = Image.open(os.path.join(a.out, 'shading%s_cam%d.jpg' % (tag, cam))).resize((tw, th))
        sheet.paste(img, (10, y + 20)); d.text((10, y + 4), 'cam%d %s capture' % (cam, colour), fill=(255, 255, 0))
        sheet.paste(heat(bg, 0.7, 1.3, (tw, th)), (20 + tw, y + 20))
        d.text((20 + tw, y + 4), 'B/G %.2f..%.2f (blue<1<red)' % (bg.min(), bg.max()), fill=(255, 255, 0))
        sheet.paste(heat(rg, 0.7, 1.3, (tw, th)), (30 + 2 * tw, y + 20))
        d.text((30 + 2 * tw, y + 4), 'R/G %.2f..%.2f' % (rg.min(), rg.max()), fill=(255, 255, 0))
    sheet.save(os.path.join(a.out, 'shading%s_map.png' % tag))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Warp camera captures into monitor coordinates (via the locate.py homographies) and put
them next to the reference image, so the two cameras can be compared patch by patch in the
same orientation (cam1 is mounted rotated 180 degrees with respect to cam0).

Usage: python3 rectify.py --out iter1 --region 843,229,555,653 --title "run 3" \
           --cam0 iter1/x.jpg --cam1 iter1/y.jpg [--save iter1/rectified_run3.png]
"""
import argparse
import json
import os

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))


def rectify(img, Hm, region, scale=0.6):
    """Nearest-neighbour inverse warp of the screen ``region`` into a scaled image."""
    x0, y0, w, h = region
    ow, oh = int(w * scale), int(h * scale)
    xs = x0 + (np.arange(ow) + 0.5) / scale
    ys = y0 + (np.arange(oh) + 0.5) / scale
    X, Y = np.meshgrid(xs, ys)
    P = np.stack([X.ravel(), Y.ravel(), np.ones(X.size)])
    Q = np.asarray(Hm) @ P
    u = (Q[0] / Q[2]).round().astype(int)
    v = (Q[1] / Q[2]).round().astype(int)
    arr = np.asarray(img.convert('RGB'))
    ok = (u >= 0) & (u < arr.shape[1]) & (v >= 0) & (v < arr.shape[0])
    out = np.zeros((oh * ow, 3), np.uint8)
    out[ok] = arr[v[ok], u[ok]]
    return Image.fromarray(out.reshape(oh, ow, 3))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--region', default='843,229,555,653')
    ap.add_argument('--cam0', required=True)
    ap.add_argument('--cam1', required=True)
    ap.add_argument('--title', default='')
    ap.add_argument('--save')
    ap.add_argument('--pattern', default=os.path.join(HERE, 'patterns', 'pattern_grid.png'))
    a = ap.parse_args()
    region = tuple(int(v) for v in a.region.split(','))
    loc = json.load(open(os.path.join(a.out, 'locate.json')))
    scale = 0.6
    ref = Image.open(a.pattern).crop((region[0], region[1], region[0] + region[2], region[1] + region[3]))
    ref = ref.resize((int(region[2] * scale), int(region[3] * scale)))
    tiles = [('reference (sent to monitor)', ref)]
    for cam, path in ((0, a.cam0), (1, a.cam1)):
        tiles.append(('cam%d rectified %s' % (cam, a.title),
                      rectify(Image.open(path), loc['cams'][str(cam)]['H_mon_to_cam'], region, scale)))
    W = sum(t.width for _, t in tiles) + 10 * (len(tiles) + 1)
    H = tiles[0][1].height + 30
    sheet = Image.new('RGB', (W, H), (30, 30, 30))
    x = 10
    for label, t in tiles:
        sheet.paste(t, (x, 25))
        ImageDraw.Draw(sheet).text((x, 6), label, fill=(255, 255, 0))
        x += t.width + 10
    save = a.save or os.path.join(a.out, 'rectified.png')
    sheet.save(save)
    print(save)


if __name__ == '__main__':
    main()

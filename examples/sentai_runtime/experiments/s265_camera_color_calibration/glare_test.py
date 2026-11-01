#!/usr/bin/env python3
"""Stray-light (veiling glare) test.  A coloured square is shown at the centre of each
camera's field of view, once on a black background and once on a white background; the
square's measured colour must not depend on the background unless stray light (lens haze,
flare, reflection of the board in the glossy screen) is added to the image.

Usage: python3 glare_test.py --out iter1 [--colour e6c71f] [--size 220]
"""
import argparse
import io
import json
import os
import subprocess
import sys
import time

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from board_repl import Repl  # noqa: E402
from analyze import measure, apply_h  # noqa: E402

W, H = 1920, 1080


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--colour', default='e6c71f', help='square colour, default ColorChecker yellow')
    ap.add_argument('--size', type=int, default=220)
    ap.add_argument('--cams', default='0,1')
    a = ap.parse_args()
    rgb = tuple(int(a.colour[i:i + 2], 16) for i in (0, 2, 4))
    loc = json.load(open(os.path.join(a.out, 'locate.json')))
    r = Repl()
    res = {}
    for cam in [int(c) for c in a.cams.split(',')]:
        Hinv = loc['cams'][str(cam)]['H_cam_to_mon']
        cw, ch = loc['cams'][str(cam)]['cam_size']
        cx, cy = apply_h(Hinv, np.array([[cw / 2, ch / 2]]))[0]
        rect = [int(cx - a.size / 2), int(cy - a.size / 2), a.size, a.size]
        patches = [{'name': 'square', 'srgb': list(rgb), 'rect': rect}]
        r.select(cam, 1.5)
        res[cam] = {'rect': rect}
        for bg_name, bg in (('black', (0, 0, 0)), ('white', (255, 255, 255)), ('grey', (118, 118, 118))):
            img = Image.new('RGB', (W, H), bg)
            img.paste(Image.new('RGB', (a.size, a.size), rgb), (rect[0], rect[1]))
            p = os.path.join(HERE, 'patterns', 'glare_%s_%s_cam%d.png' % (a.colour, bg_name, cam))
            img.save(p)
            subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--png', p],
                           check=True, stdout=subprocess.DEVNULL)
            time.sleep(1.5)
            jpg = r.snap_cur('/s265_g.jpg', 95)
            open(os.path.join(a.out, 'glare_%s_%s_cam%d.jpg' % (a.colour, bg_name, cam)), 'wb').write(jpg)
            m = measure(Image.open(io.BytesIO(jpg)), loc['cams'][str(cam)]['H_mon_to_cam'], patches, shrink=0.4)[0]
            res[cam][bg_name] = m['mean'] if m else None
            print('cam%d square %s on %-5s bg -> RGB %s  (dE76 %.1f)' % (
                cam, rgb, bg_name, tuple(int(round(v)) for v in m['mean']) if m else None, m['dE76'] if m else -1))
    r.close()
    json.dump(res, open(os.path.join(a.out, 'glare_%s.json' % a.colour), 'w'), indent=1)
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--grid', '--region', '843,229,555,653',
                    '--layout', '4x6'], check=True, stdout=subprocess.DEVNULL)


if __name__ == '__main__':
    main()

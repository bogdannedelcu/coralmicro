#!/usr/bin/env python3
"""Primary-colour response: full-screen red, green, blue (and white, black), one capture
per camera and colour.  Prints the mean RGB of the central 40 % of the frame and of the
whole frame, saves <out>/prim_<colour>_cam<N>.jpg, <out>/primaries.json and a contact
sheet <out>/primaries.png (camera rows, colour columns).

Usage: python3 primaries.py --out iter1 [--cams 0,1] [--colours red,green,blue,white,black]
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

COLOURS = {'red': 'ff0000', 'green': '00ff00', 'blue': '0000ff', 'white': 'ffffff', 'black': '000000',
           'cyan': '00ffff', 'magenta': 'ff00ff', 'yellow': 'ffff00'}


def show_solid(hexrgb):
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--solid', hexrgb],
                   check=True, stdout=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--cams', default='0,1')
    ap.add_argument('--colours', default='red,green,blue,white,black')
    ap.add_argument('--settle', type=float, default=1.5)
    a = ap.parse_args()
    cams = [int(c) for c in a.cams.split(',')]
    cols = a.colours.split(',')
    r = Repl()
    res = {}
    for cam in cams:
        r.select(cam, 1.5)
        res[cam] = {}
        for c in cols:
            show_solid(COLOURS[c])
            time.sleep(a.settle)
            jpg = r.snap_cur('/s265_p.jpg', 95)
            open(os.path.join(a.out, 'prim_%s_cam%d.jpg' % (c, cam)), 'wb').write(jpg)
            arr = np.asarray(Image.open(io.BytesIO(jpg)).convert('RGB'), np.float32)
            h, w = arr.shape[:2]
            centre = arr[int(0.3 * h):int(0.7 * h), int(0.3 * w):int(0.7 * w)].reshape(-1, 3).mean(0)
            whole = arr.reshape(-1, 3).mean(0)
            res[cam][c] = {'centre': centre.round(1).tolist(), 'whole': whole.round(1).tolist()}
            print('cam%d %-7s centre RGB %3.0f %3.0f %3.0f   whole %3.0f %3.0f %3.0f' % (cam, c, *centre, *whole))
    r.close()
    json.dump(res, open(os.path.join(a.out, 'primaries.json'), 'w'), indent=1)
    tw, th = 240, 180
    sheet = Image.new('RGB', (len(cols) * (tw + 10) + 10, len(cams) * (th + 30) + 10), (30, 30, 30))
    d = ImageDraw.Draw(sheet)
    for i, cam in enumerate(cams):
        for j, c in enumerate(cols):
            im = Image.open(os.path.join(a.out, 'prim_%s_cam%d.jpg' % (c, cam))).resize((tw, th))
            x, y = 10 + j * (tw + 10), 10 + i * (th + 30)
            sheet.paste(im, (x, y + 20))
            d.text((x, y + 4), 'cam%d %s -> %s' % (cam, c, tuple(int(v) for v in res[cam][c]['centre'])), fill=(255, 255, 0))
    sheet.save(os.path.join(a.out, 'primaries.png'))
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--grid', '--region', '843,229,555,653',
                    '--layout', '4x6'], check=True, stdout=subprocess.DEVNULL)


if __name__ == '__main__':
    main()

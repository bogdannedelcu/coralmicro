#!/usr/bin/env python3
"""Empirical probe of the OV5640 LENC registers (datasheet gives the layout, not the gain
formula): on a uniform grey field, perturb one green-matrix cell (0x5800-0x5823, 6x6, 6 bit)
and one BR-matrix byte (0x5824-0x583C, 5x5, high nibble / low nibble) and measure the per-cell
change of R, G, B over a 6x6 grid of the frame.  Tells us (a) which image region each register
drives (the sensor is rotated 270 degrees in the housing), (b) the gain per register unit,
(c) which nibble is B and which is R.

Usage: python3 lenc_probe.py --out iter1/lenc [--cams 0,1] [--grey 150]
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
from shading_map import cell_stats  # noqa: E402

PROBES = [  # (label, reg, delta)
    ('G cell (0,0)  0x5800 +16', 0x5800, 16),
    ('G cell (2,2)  0x580E +16', 0x580E, 16),
    ('G cell (5,5)  0x5823 +16', 0x5823, 16),
    ('BR byte (0,0) 0x5824 hi +4', 0x5824, 0x40),
    ('BR byte (0,0) 0x5824 lo +4', 0x5824, 0x04),
    ('BR byte (2,2) 0x5830 hi +4', 0x5830, 0x40),
    ('BR byte (2,2) 0x5830 lo +4', 0x5830, 0x04),
]


def fmt(m):
    return '\n'.join('      ' + ' '.join('%+5.1f' % v for v in row) for row in m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--cams', default='0,1')
    ap.add_argument('--grey', type=int, default=150)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--solid', '%02x%02x%02x' % ((a.grey,) * 3)],
                   check=True, stdout=subprocess.DEVNULL)
    time.sleep(1.5)
    r = Repl()
    res = {}
    for cam in [int(c) for c in a.cams.split(',')]:
        r.select(cam, 1.5)
        print('cam%d LENC enable (0x5000 bit7): %d; G hscale/vscale %s BR hscale/vscale %s' % (
            cam, (r.reg(cam, 0x5000) >> 7) & 1,
            ['%02X' % v for v in r.regs(cam, [0x5846, 0x5847, 0x5848, 0x5849])],
            ['%02X' % v for v in r.regs(cam, [0x5842, 0x5843, 0x5844, 0x5845])]))
        base = cell_stats(Image.open(io.BytesIO(r.snap_cur('/s265_l.jpg', 95))), 6, 6)
        res[cam] = {'base': base.round(1).tolist(), 'probes': {}}
        for label, reg, delta in PROBES:
            old = r.reg(cam, reg)
            new = old + delta
            if (reg < 0x5824 and new > 63) or new > 255:
                new = old - delta
            r.regw(cam, reg, new)
            time.sleep(0.7)
            st = cell_stats(Image.open(io.BytesIO(r.snap_cur('/s265_l.jpg', 95))), 6, 6)
            r.regw(cam, reg, old)
            d = st - base
            res[cam]['probes'][label] = {'reg': '0x%04X' % reg, 'old': old, 'new': new, 'dRGB': d.round(1).tolist()}
            k = np.unravel_index(np.abs(d).sum(2).argmax(), d.shape[:2])
            print('  %s  (old %d -> %d): max change at image cell row %d col %d: dR %+.1f dG %+.1f dB %+.1f'
                  % (label, old, new, k[0], k[1], *d[k]))
            print('    dG map:\n' + fmt(d[..., 1]))
            if 'BR' in label:
                print('    dR map:\n' + fmt(d[..., 0]) + '\n    dB map:\n' + fmt(d[..., 2]))
        time.sleep(0.7)
    r.close()
    json.dump(res, open(os.path.join(a.out, 'lenc_probe.json'), 'w'), indent=1)
    subprocess.run([sys.executable, os.path.join(HERE, 'show_pattern.py'), '--grid', '--region', '843,229,555,653',
                    '--layout', '4x6'], check=True, stdout=subprocess.DEVNULL)


if __name__ == '__main__':
    main()

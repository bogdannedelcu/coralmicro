#!/usr/bin/env python3
"""Capture one JPEG per camera and dump the colour-relevant OV5640 registers.

Usage: python3 capture.py --out iter1 --tag initial [--cams 0,1] [--quality 90]

Writes <out>/<tag>_cam<N>.jpg and <out>/<tag>_regs.json.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_repl import Repl  # noqa: E402

# OV5640 registers that shape colour: AWB gains, AWB manual bit, AEC/AGC, SDE
REGS = {
    'awb_r_gain': (0x3400, 0x3401), 'awb_g_gain': (0x3402, 0x3403),
    'awb_b_gain': (0x3404, 0x3405), 'awb_manual_0x3406': (0x3406,),
    'aec_manual_0x3503': (0x3503,), 'agc_gain': (0x350A, 0x350B),
    'exposure': (0x3500, 0x3501, 0x3502),
    'isp_ctrl1_0x5001': (0x5001,), 'sde_ctrl0_0x5580': (0x5580,),
    'sde_sat_u_0x5583': (0x5583,), 'sde_sat_v_0x5584': (0x5584,),
    'sde_bright_0x5587': (0x5587,), 'sde_contrast_0x5586': (0x5586,),
    'sde_sign_0x5588': (0x5588,),
    'awb_ctrl_0x5180': (0x5180,), 'awb_ctrl_0x5183': (0x5183,),
    'cmx1_0x5381': (0x5381,), 'cmx2': (0x5382,), 'cmx3': (0x5383,),
    'cmx4': (0x5384,), 'cmx5': (0x5385,), 'cmx6': (0x5386,),
    'cmx7': (0x5387,), 'cmx8': (0x5388,), 'cmx9': (0x5389,), 'cmx_sign': (0x538A, 0x538B),
    'format_0x4300': (0x4300,),
}


def read_regs(r, cam):
    out = {}
    for name, addrs in REGS.items():
        vals = r.regs(cam, addrs)
        out[name] = {'regs': ['0x%04X' % a for a in addrs], 'vals': ['0x%02X' % v for v in vals]}
        if len(vals) == 2 and name.endswith('gain'):
            out[name]['u16'] = (vals[0] << 8) | vals[1]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--tag', required=True)
    ap.add_argument('--cams', default='0,1')
    ap.add_argument('--quality', type=int, default=90)
    ap.add_argument('--settle', type=float, default=1.5)
    ap.add_argument('--no-regs', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    log = open(os.path.join(a.out, a.tag + '_capture.log'), 'a')
    r = Repl(log=log)
    regs = {}
    for cam in [int(c) for c in a.cams.split(',')]:
        jpg = r.snap(cam, '/s265_%s_c%d.jpg' % (a.tag, cam), a.quality, a.settle)
        p = os.path.join(a.out, '%s_cam%d.jpg' % (a.tag, cam))
        open(p, 'wb').write(jpg)
        print('%s %d bytes' % (p, len(jpg)))
        if not a.no_regs:
            regs['cam%d' % cam] = read_regs(r, cam)
    if regs:
        p = os.path.join(a.out, a.tag + '_regs.json')
        json.dump(regs, open(p, 'w'), indent=1)
        for cam, d in regs.items():
            print(cam, 'AWB R/G/B gain', d['awb_r_gain']['u16'], d['awb_g_gain']['u16'],
                  d['awb_b_gain']['u16'], 'AGC', d['agc_gain']['vals'], 'exp', d['exposure']['vals'])
    r.close()


if __name__ == '__main__':
    main()

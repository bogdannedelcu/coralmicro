#!/usr/bin/env python3
"""Full OV5640 register scan of both camera modules in the same state, plus a careful OTP read,
and a classified diff (W24-T22 registry).  The read loop runs ON THE BOARD (one MicroPython
exec per 4 KB block) so the scan takes seconds, not minutes.

Usage: python3 scan_regs.py --out scans [--init 30] [--no-init]
Writes scans/<ID>_<date>.hex (one line per 16 registers) and prints the diff.
"""
import argparse
import datetime
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'experiments', 's265_camera_color_calibration'))
from board_repl import Repl  # noqa: E402

BOARD = r"""
def _scan(cam, a0, a1):
    import sentai
    rr = sentai.camera.reg_read
    out = []
    for base in range(a0, a1, 16):
        out.append(''.join('%02X' % (rr(cam, base + i) & 0xFF) for i in range(16)))
    return out
def _otp(cam):
    import sentai
    rr, rw = sentai.camera.reg_read, sentai.camera.reg_write
    r3000 = rr(cam, 0x3000); r3004 = rr(cam, 0x3004)
    rw(cam, 0x3000, r3000 & ~0x10)      # release OTP reset
    rw(cam, 0x3004, r3004 | 0x10)       # OTP clock on
    res = {}
    for speed in (0x01, 0x03):          # fast, slow read
        rw(cam, 0x3D21, speed)
        for _ in range(50):
            if not (rr(cam, 0x3D21) & 0x80): break
        import sentai as _s
        d = ''.join('%02X' % rr(cam, 0x3D00 + i) for i in range(32))
        rw(cam, 0x3D21, 0x00)
        res['speed_%d' % speed] = d
    res['r3000'] = r3000; res['r3004'] = r3004
    return res
"""

# registers whose value legitimately differs with the scene / auto loops / per-camera firmware setup
DYNAMIC = set(list(range(0x3500, 0x350C)) + list(range(0x3400, 0x3406)) + list(range(0x5690, 0x56A2)) +
              list(range(0x3C00, 0x3C20)) + [0x3A1C, 0x3A1D] + list(range(0x3A20, 0x3A26)) +
              list(range(0x5180, 0x51B0)) + list(range(0x3D00, 0x3D22)) + [0x3820, 0x3821, 0x3022, 0x3023, 0x3024, 0x3025, 0x3026, 0x3027, 0x3028, 0x3029])
RANGES = [(0x3000, 0x4000), (0x4000, 0x5000), (0x5000, 0x6000), (0x6000, 0x6100)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(HERE, 'scans'))
    ap.add_argument('--init', type=int, default=30)
    ap.add_argument('--no-init', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    reg = json.load(open(os.path.join(HERE, 'modules.json')))
    ids = {m['placement']['connector']: m['id'] for m in reg['modules']}
    r = Repl()
    print(r.value('sentai.version()'))
    if not a.no_init:
        print('init ->', r.cmd('print(sentai.camera.init(1, %d))' % a.init, 40).replace('\r', '').strip().split('\n')[-1])
        time.sleep(2)
    r.cmd('exec(%r)' % BOARD, 10)
    date = datetime.datetime.now().strftime('%Y-%m-%d')
    dumps = {}
    for cam in (0, 1):
        r.select(cam)
        time.sleep(1.0)
        lines = []
        for a0, a1 in RANGES:
            out = r.cmd('print(_scan(%d, %d, %d))' % (cam, a0, a1), 120)
            last = [l for l in out.replace('\r', '').split('\n') if l.strip().startswith('[')][-1]
            blk = eval(last)
            for k, hx in enumerate(blk):
                lines.append((a0 + 16 * k, hx))
        otp = eval([l for l in r.cmd('print(_otp(%d))' % cam, 30).replace('\r', '').split('\n') if l.strip().startswith('{')][-1])
        mid = ids.get('cam%d' % cam, 'cam%d' % cam)
        path = os.path.join(a.out, '%s_%s.hex' % (mid, date))
        with open(path, 'w') as f:
            f.write('# %s on cam%d, %s, %s\n' % (mid, cam, date, r.value('sentai.version()')))
            f.write('# OTP fast %s\n# OTP slow %s\n# 0x3000=%02X 0x3004=%02X\n' % (otp['speed_1'], otp['speed_3'], otp['r3000'], otp['r3004']))
            for addr, hx in lines:
                f.write('%04X %s\n' % (addr, hx))
        dumps[cam] = (mid, dict(lines), otp)
        print('cam%d (%s): %d registers, OTP fast %s' % (cam, mid, 16 * len(lines), otp['speed_1']))
        print('           OTP slow %s   0x3000=%02X 0x3004=%02X' % (otp['speed_3'], otp['r3000'], otp['r3004']))
    r.close()
    # ---- diff ----
    (id0, d0, _), (id1, d1, _) = dumps[0], dumps[1]
    static, dyn = [], []
    for addr in sorted(d0):
        h0, h1 = d0[addr], d1[addr]
        if h0 == h1:
            continue
        for i in range(16):
            b0, b1 = h0[2 * i:2 * i + 2], h1[2 * i:2 * i + 2]
            if b0 != b1:
                (dyn if (addr + i) in DYNAMIC else static).append((addr + i, b0, b1))
    print('\nregisters that differ: %d static-class, %d dynamic-class (AEC/AGC/AWB/AVG/flicker/OTP/flip)' % (len(static), len(dyn)))
    print('static-class differences (%s vs %s):' % (id0, id1))
    for addr, b0, b1 in static:
        print('  0x%04X  %s  %s' % (addr, b0, b1))
    print('dynamic-class differences:')
    for addr, b0, b1 in dyn:
        print('  0x%04X  %s  %s' % (addr, b0, b1))
    json.dump({'static': static, 'dynamic': dyn, 'ids': [id0, id1], 'date': date},
              open(os.path.join(a.out, 'diff_%s.json' % date), 'w'), indent=1)


if __name__ == '__main__':
    main()

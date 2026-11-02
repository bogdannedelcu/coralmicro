#!/usr/bin/env python3
"""EXP-s266 iter8: alternation cadence from the firmware FB-done ring WITH a consumer
dequeuing, per mode.  Checks the sensor period estimate, the genlock result, ratio_unit,
and the tag pattern / intervals of ratio(1,1) alternation.

Usage: python3 frame_log_check.py --out iter8_genlock_fix [--modes vga30,sxga15,sxga30p]
"""
import argparse
import glob
import json
import os
import re
import sys
import time

import numpy as np
import serial

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's265_camera_color_calibration'))
from board_repl import Repl  # noqa: E402

MODES = {
    'vga30':   dict(hw=None,        packed=0, fps=30, T=33333),
    'sxga15':  dict(hw=(1280, 960), packed=0, fps=15, T=66667),
    'sxga30p': dict(hw=(1280, 960), packed=1, fps=30, T=33333),
}
CONSUMER = r"""
def _c(n, byte):
    import sentai
    for i in range(n):
        sentai.camera.peek_col(40, byte)
"""


def reset_board():
    r = Repl(); r.ser.write(b'sentai.sys.reset()\r'); time.sleep(0.5); r.close()
    t0 = time.time()
    while time.time() - t0 < 40:
        time.sleep(1)
        try:
            s = serial.Serial(sorted(glob.glob('/dev/ttyACM*'))[0], 115200, timeout=0.5); s.close(); break
        except Exception:
            pass
    time.sleep(3)


def run(name, m, out, log):
    reset_board()
    r = Repl(log=log)
    ver = r.value('sentai.version()')
    print('== %s (%s)' % (name, ver))
    if m['hw']:
        r.value('sentai.camera.set_hw(%d, %d)' % m['hw'])
    if m['packed']:
        r.value('sentai.camera.packed420(1)')
    init = r.cmd('print(sentai.camera.init(1, %d))' % m['fps'], 40).replace('\r', '').strip().split('\n')[-1]
    time.sleep(3)
    st = r.value('sentai.camera.stats()')
    period = int(re.search(r"'period_us': (\d+)", st).group(1))
    unit = r.value('sentai.camera.ratio_unit()')
    glk = re.search(r"'genlock_ok': (\d+)", st); glk = glk.group(1) if glk else '?'
    dm = r.cmd('print(sentai.rtos.dmesg(6000))', 20)
    codes = sorted(set(re.findall(r'E:0A(0[CD]|0E|10|11|12):\d+', dm)))
    print('  init %s | period_us %d (nominal %d) | ratio_unit %s | genlock_ok %s | cam SERR %s' % (init.strip(), period, m['T'], unit, glk, codes))
    r.cmd('exec(%r)' % CONSUMER, 10)
    r.value('sentai.camera.ratio(1, 1)')
    time.sleep(0.5)
    r.cmd('_c(%d, %d)' % (120 if m['fps'] >= 30 else 60, 1 if m['packed'] else 0), 60)   # consumer dequeuing ~4 s
    out_log = r.cmd('print(sentai.camera.frame_log(64))', 20)
    log64 = eval([l for l in out_log.replace('\r', '').split('\n') if l.strip().startswith('[')][-1])
    r.value('sentai.camera.ratio(1, 0)')
    r.close()
    # the ring also holds entries from before the consumer started (the queue was full
    # and the log paused): keep only the entries after the last gap > 3 periods
    ts_all = np.array([e[0] for e in log64], np.int64)
    gaps = np.where(((np.diff(ts_all) & 0xFFFFFFFF) / 1000.0) > 3 * m['T'] / 1000.0)[0]
    start = int(gaps[-1] + 1) if len(gaps) else 0
    log64 = log64[start:]
    ts = np.array([e[0] for e in log64], np.int64); cams = [e[2] for e in log64]; seq = [e[3] for e in log64]
    dt = (np.diff(ts) & 0xFFFFFFFF) / 1000.0
    print('  ring entries analysed: %d (dropped %d before the last log gap)' % (len(log64), start))
    tags = ''.join(str(c) for c in cams)
    alt = sum(1 for a, b in zip(cams, cams[1:]) if a != b) / max(1, len(cams) - 1)
    seq_step = np.diff(np.array(seq, np.int64)) & 0xFFFF
    res = {'mode': name, 'version': ver, 'period_us': period, 'nominal_us': m['T'], 'ratio_unit': unit, 'genlock_ok': glk,
           'cam_serr': codes, 'tags': tags, 'alternation_fraction': alt,
           'interval_ms': {'mean': float(dt.mean()), 'sd': float(dt.std()), 'min': float(dt.min()), 'max': float(dt.max())},
           'seq_steps': {int(k): int(v) for k, v in zip(*np.unique(seq_step, return_counts=True))}}
    print('  tags %s' % tags)
    print('  FB-done interval %.2f ms sd %.2f (min %.2f max %.2f); flips between consecutive entries %.0f %%; seq steps %s' % (
        dt.mean(), dt.std(), dt.min(), dt.max(), 100 * alt, res['seq_steps']))
    ok = abs(period - m['T']) < 0.1 * m['T'] and unit == '1' and alt > 0.95 and dt.std() < 0.1 * dt.mean()
    res['pass'] = bool(ok)
    print('  -> %s' % ('PASS (1:1, uniform cadence, period plausible)' if ok else 'FAIL'))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--modes', default='sxga30p,sxga15,vga30')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    log = open(os.path.join(a.out, 'repl.log'), 'a')
    res = [run(n, MODES[n], a.out, log) for n in a.modes.split(',')]
    json.dump(res, open(os.path.join(a.out, 'verdict.json'), 'w'), indent=1)
    print('OVERALL:', 'PASS' if all(x['pass'] for x in res) else 'FAIL')


if __name__ == '__main__':
    main()

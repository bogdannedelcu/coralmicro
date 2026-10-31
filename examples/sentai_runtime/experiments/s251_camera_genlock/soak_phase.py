#!/usr/bin/env python3
"""EXP-s251 soak: hold ratio(1,1) alternation at the full sensor rate for N
minutes while consuming frames, and sample the inter-sensor phase every
30 s to show it does not drift after the C-side genlock (build >= 1654).

Each 30 s window runs on the board as one exec() line: consume frames with
peek_col(40) (cheap, dequeues one buffer), count consumed vs sensor frames,
then read camera.stats() (phase_us is refreshed by the watchdog phase
monitor once per kick while alternating).  Rows:

  CSV|t_s,consumed,sensor_frames,fps,phase_us,stale_skips,phase_drift,stall_count

Usage: python3 soak_phase.py [--minutes 15] [--tag soak15]
"""
import argparse
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402

WINDOW = ('exec("t0=sentai.rtos.ticks_ms()\\nf0=sentai.camera.sensor_frames()\\nn=0\\n'
          'while sentai.rtos.ticks_ms()-t0<30000:\\n  sentai.camera.peek_col(40); n+=1\\n'
          'dt=sentai.rtos.ticks_ms()-t0\\ns=sentai.camera.stats()\\n'
          'print(\'WIN\',n,sentai.camera.sensor_frames()-f0,dt,(n*1000)//dt,s[\'phase_us\'],'
          's[\'stale_skips\'],s[\'phase_drift\'],s[\'stall_count\'])")')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=15.0)
    ap.add_argument('--tag', default='soak15')
    ap.add_argument('--no-init', action='store_true', help='camera already initialised')
    a = ap.parse_args()
    ser = B.open_repl()
    B.cmd(ser, 'import sentai', 5)
    log = open(os.path.join(HERE, 'run_%s.log' % a.tag), 'w')
    csv = open(os.path.join(HERE, 'results_%s.csv' % a.tag), 'w')
    csv.write('t_s,consumed,sensor_frames,ms,fps,phase_us,stale_skips,phase_drift,stall_count\n')
    log.write(B.cmd(ser, 'print(sentai.version())', 8))
    if not a.no_init:
        out = B.cmd(ser, 'print("INIT", sentai.camera.init(1, 45))', 30)
        log.write(out)
        for l in out.split('\n'):
            if '[cam] genlock' in l or 'INIT' in l[:5]:
                print(l.strip())
    out = B.cmd(ser, 'print("STATS0", sentai.camera.stats())', 8)
    log.write(out)
    B.cmd(ser, 'sentai.camera.ratio(1, 1)', 5)
    t_start = time.monotonic()
    n_win = int(a.minutes * 2 + 0.5)
    for w in range(n_win):
        out = B.cmd(ser, WINDOW, 45)
        log.write(out)
        log.flush()
        m = re.search(r'WIN (\d+) (\d+) (\d+) (\d+) (-?\d+|None) (\d+) (\d+) (\d+)', out)
        t_s = int(time.monotonic() - t_start)
        if not m:
            print('window %d: no WIN row -> %s' % (w, out.strip()[-200:]))
            csv.write('%d,,,,,,,\n' % t_s)
            csv.flush()
            if 'Traceback' in out:
                break
            continue
        row = (t_s,) + m.groups()
        csv.write(','.join(str(x) for x in row) + '\n')
        csv.flush()
        print('t=%4ds consumed=%s sensor=%s fps=%s phase_us=%s stale=%s drift=%s stall=%s' %
              (t_s, m.group(1), m.group(2), m.group(4), m.group(5), m.group(6), m.group(7), m.group(8)))
    B.cmd(ser, 'sentai.camera.ratio(0, 0)', 5)
    out = B.cmd(ser, 'print("PHASE_END", sentai.camera.phase())', 20)
    log.write(out)
    print([l.strip() for l in out.split('\n') if 'PHASE_END' in l])
    out = B.cmd(ser, 'print(sentai.rtos.dmesg())', 10)
    log.write(out)
    print('dmesg CAM events:', ' '.join(re.findall(r'E:0A[0-9A-F]{2}:\d+', out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

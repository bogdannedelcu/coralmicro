#!/usr/bin/env python3
"""EXP-s251 F16 isolation: does plain 1:1 alternation at the as-found phase
wedge the CSI without any genlock code running?

Cold boot (caller RAM-loads first), genlock_auto(0) (default), init, verify
frames flow, then ratio(1,1) with drain=0/dirty=0 and a consumer for N
seconds (the same ISR flips the genlock measurement performs), then stats.
Only afterwards run genlock(0) by hand and re-check.

Usage: python3 alt_only_check.py [--tag iterN] [--seconds 6]
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    ap.add_argument('--seconds', type=int, default=6)
    ap.add_argument('--settle', type=float, default=0.0, help='seconds to wait after init before alternating')
    ap.add_argument('--no-genlock', action='store_true')
    ap.add_argument('--pre', default='', help='REPL line to run right after import (A/B knobs)')
    a = ap.parse_args()
    alt = ('exec("sentai.camera.switch_drain(0)\\nsentai.camera.dirty_skip_n(0)\\nsentai.camera.ratio(1,1)\\n'
           't0=sentai.rtos.ticks_ms()\\nf0=sentai.camera.sensor_frames()\\nn=0\\nerr=0\\n'
           'while sentai.rtos.ticks_ms()-t0<%d:\\n  try:\\n    sentai.camera.peek_col(40); n+=1\\n  except Exception as e:\\n    err+=1\\n    if err>3: break\\n'
           'dt=sentai.rtos.ticks_ms()-t0\\nprint(\'ALT consumed\',n,\'sensor\',sentai.camera.sensor_frames()-f0,\'ms\',dt,\'fps\',(n*1000)//max(dt,1),\'err\',err)\\n'
           'sentai.camera.ratio(0,0)")' % (a.seconds * 1000))
    steps = [
        ('import sentai', 5),
        ('print(sentai.version())', 8),
        (a.pre or 'None', 5),
        ('print("AUTO", sentai.camera.genlock_auto())', 5),
        ('print("INIT", sentai.camera.init(1, 45))', 30),
        ('sentai.rtos.sleep_ms(%d)' % int(a.settle * 1000), a.settle + 5),
        ('print("SF_A", sentai.camera.sensor_frames()); sentai.rtos.sleep_ms(500); print("SF_B", sentai.camera.sensor_frames())', 8),
        ('print("PEEK", sentai.camera.peek_col(40)[:2])', 8),
        (alt, a.seconds + 15),
        ('print("STATS_ALT", sentai.camera.stats())', 8),
        ('print("SF_C", sentai.camera.sensor_frames()); sentai.rtos.sleep_ms(500); print("SF_D", sentai.camera.sensor_frames())', 8),
    ] + ([] if a.no_genlock else [
        ('print("GENLOCK", sentai.camera.genlock(0))', 40),
        ('print("STATS_GL", sentai.camera.stats())', 8),
        ('print("PHASE", sentai.camera.phase())', 20),
    ]) + [
        ('print("DMESG", sentai.rtos.dmesg())', 10),
    ]
    ser = B.open_repl()
    log_path = os.path.join(HERE, 'run_%s.log' % a.tag)
    with open(log_path, 'w') as log:
        for line, t in steps:
            out = B.cmd(ser, line, t)
            log.write('>>> %s\n%s\n' % (line, out))
            for l in out.split('\n'):
                s = l.strip()
                if s and not s.startswith('>>>') and not s.startswith('[CAM') and not s.startswith('BOARD_') \
                        and not s.startswith('CAMERA_') and 'sentai.' not in s[:8] and not s.startswith('exec('):
                    print(s[:240])
    print('log=%s' % log_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())

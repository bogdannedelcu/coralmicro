#!/usr/bin/env python3
"""EXP-s251 iter8+: exercise the C-side genlock (build >= 1653).

Sequence over the REPL:
  1. camera.init(1, 45)  -> the firmware runs sentai_cam_genlock(0) itself;
     the [cam] genlock lines and camera.stats() are captured.
  2. camera.phase()      -> independent re-measurement of phi.
  3. ratio(1,1) with the defaults the genlock left (drain=0, dirty=0):
     count sensor frames / consumed frames for ~3 s -> fps.
  4. camera.genlock(1)   -> synchronous standby restart + trim; stats again.
  5. camera.phase() again.

Usage: python3 genlock_check.py [--tag iterN]
Writes run_<tag>.log next to this file.
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402

STEPS = [
    ('import sentai', 5),
    ('print(sentai.version())', 8),
    ('print("AUTO", sentai.camera.genlock_auto(1))', 5),   # build >= 1659: init-time genlock is opt-in (audit F16)
    ('print("INIT", sentai.camera.init(1, 45))', 30),
    ('print("STATS1", sentai.camera.stats())', 8),
    ('print("PHASE1", sentai.camera.phase())', 20),
    ('exec("sentai.camera.ratio(1,1)\\nt0=sentai.rtos.ticks_ms()\\nf0=sentai.camera.sensor_frames()\\nn=0\\n'
     'while sentai.rtos.ticks_ms()-t0<3000:\\n  sentai.camera.peek_col(40); n+=1\\n'
     'dt=sentai.rtos.ticks_ms()-t0\\nprint(\'ALT consumed\',n,\'sensor\',sentai.camera.sensor_frames()-f0,\'ms\',dt,\'fps\',(n*1000)//dt)\\n'
     'sentai.camera.ratio(0,0)")', 25),
    ('print("STATS2", sentai.camera.stats())', 8),
    ('print("GENLOCK_SYNC", sentai.camera.genlock(1))', 40),
    ('print("STATS3", sentai.camera.stats())', 8),
    ('print("PHASE2", sentai.camera.phase())', 20),
    ('print("DMESG"); sentai.rtos.dmesg()', 10),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    a = ap.parse_args()
    ser = B.open_repl()
    log_path = os.path.join(HERE, 'run_%s.log' % a.tag)
    with open(log_path, 'w') as log:
        for line, t in STEPS:
            out = B.cmd(ser, line, t)
            log.write('>>> %s\n%s\n' % (line, out))
            for l in out.split('\n'):
                s = l.strip()
                if s and not s.startswith('>>>') and 'sentai.' not in s[:8]:
                    print(s[:220])
    print('log=%s' % log_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())

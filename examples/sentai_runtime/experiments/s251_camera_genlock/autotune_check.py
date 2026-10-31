#!/usr/bin/env python3
"""EXP-s251: on-the-fly alignment (build >= 1660).

Cold boot (caller RAM-loads first):
  1. camera.init(1,45) with the default genlock_auto=1 -> open-loop PRE-TRIM
     on cam0 (no MUX flip); stats must show pretrim_lines > 0.
  2. ratio(1,1) with a consumer for 6 s -> full rate, phase small (monitor).
  3. Inject a phase error: one unsynchronised VTS+40 frame on cam1 (shifts
     cam1 by 40 or 80 lines = ~0.9 or 1.8 ms, outside the F16 danger zone).
  4. Keep alternating with a consumer; sample stats every 5 s for 40 s ->
     expect PHASE_DRIFT, one autotrim (SERR 0x0A11), PHASE_DRIFT_END and
     |phase_us| back under 60 us.

Usage: python3 autotune_check.py [--tag iterN]
"""
import argparse
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402

ALT = ('exec("sentai.camera.ratio(1,1)\\nt0=sentai.rtos.ticks_ms()\\nf0=sentai.camera.sensor_frames()\\nn=0\\nerr=0\\n'
       'while sentai.rtos.ticks_ms()-t0<%d:\\n  try:\\n    sentai.camera.peek_col(40); n+=1\\n  except Exception as e:\\n    err+=1\\n    if err>3: break\\n'
       'dt=sentai.rtos.ticks_ms()-t0\\ns=sentai.camera.stats()\\n'
       'print(\'WIN\',n,sentai.camera.sensor_frames()-f0,dt,(n*1000)//max(dt,1),err,s[\'phase_us\'],s[\'phase_drift\'],s[\'autotrims\'],s[\'stall_count\'])")')
INJECT = ('exec("v=(sentai.camera.reg_read(1,0x380E)<<8)|sentai.camera.reg_read(1,0x380F)\\n'
          'sentai.camera.reg_write(1,0x380E,(v+40)>>8); sentai.camera.reg_write(1,0x380F,(v+40)&255)\\n'
          'sentai.rtos.sleep_ms(50)\\nsentai.camera.reg_write(1,0x380E,v>>8); sentai.camera.reg_write(1,0x380F,v&255)\\nprint(\'INJECT vts\',v)")')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    a = ap.parse_args()
    ser = B.open_repl()
    log = open(os.path.join(HERE, 'run_%s.log' % a.tag), 'w')

    def q(line, t=10):
        out = B.cmd(ser, line, t)
        log.write('>>> %s\n%s\n' % (line, out))
        log.flush()
        for l in out.split('\n'):
            s = l.strip()
            if s and not s.startswith('>>>') and not s.startswith('[CAM') and not s.startswith('BOARD_') \
                    and not s.startswith('CAMERA_') and 'sentai.' not in s[:8] and not s.startswith('exec('):
                print(s[:240])
        return out

    q('import sentai', 5)
    q('print(sentai.version())', 8)
    q('print("MODE", sentai.camera.genlock_auto(), sentai.camera.autotune())', 5)
    q('print("INIT", sentai.camera.init(1, 45))', 30)
    q('print("STATS0", sentai.camera.stats())', 8)
    q(ALT % 6000, 25)
    q('print("STATS1", sentai.camera.stats())', 8)
    q(INJECT, 10)
    for w in range(8):
        q(ALT % 5000, 25)
    q('sentai.camera.ratio(0,0)', 5)
    q('print("STATS2", sentai.camera.stats())', 8)
    q('print("PHASE", sentai.camera.phase())', 20)
    out = q('print("DMESG", sentai.rtos.dmesg())', 10)
    print('CAM events:', ' '.join(re.findall(r'E:0A[0-9A-F]{2}:\d+', out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""EXP-s251 / OP-S10-W23-T14: per-frame alternation check (build >= 1684).

After camera.init() (pre-trim + genlock) ratio_unit must be 1; at ratio(1,1)
the FB-done log must then show the MUX owner alternating on EVERY frame
(cam 0,1,0,1,...) at the full sensor rate.  Also exercises the drift
degrade/restore path: an injected VTS error -> PHASE_DRIFT -> ratio_unit 2 +
dirty 1 -> autotrim -> PHASE_DRIFT_END -> ratio_unit 1.

Usage: python3 perframe_check.py [--tag iterN]
"""
import argparse, os, re, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402

ALT = ('exec("sentai.camera.ratio(1,1)\\nt0=sentai.rtos.ticks_ms()\\nf0=sentai.camera.sensor_frames()\\nn=0\\nerr=0\\n'
       'while sentai.rtos.ticks_ms()-t0<%d:\\n  try:\\n    sentai.camera.peek_col(40); n+=1\\n  except Exception as e:\\n    err+=1\\n    if err>3: break\\n'
       'dt=sentai.rtos.ticks_ms()-t0\\ns=sentai.camera.stats()\\nlg=sentai.camera.frame_log(32)\\ncams=[e[2] for e in lg]\\n'
       'alts=sum(1 for i in range(1,len(cams)) if cams[i]!=cams[i-1])\\n'
       'print(\'WIN\',n,sentai.camera.sensor_frames()-f0,dt,(n*1000)//max(dt,1),err,s[\'phase_us\'],s[\'phase_drift\'],s[\'autotrims\'],s[\'ratio_unit\'],\'alts\',alts,\'of\',len(cams)-1,\'cams\',\'\'.join(str(c) for c in cams[-16:]))")')
INJECT = ('exec("v=(sentai.camera.reg_read(1,0x380E)<<8)|sentai.camera.reg_read(1,0x380F)\\n'
          'sentai.camera.reg_write(1,0x380E,(v+40)>>8); sentai.camera.reg_write(1,0x380F,(v+40)&255)\\n'
          'sentai.rtos.sleep_ms(50)\\nsentai.camera.reg_write(1,0x380E,v>>8); sentai.camera.reg_write(1,0x380F,v&255)\\nprint(\'INJECT vts\',v)")')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S')); a = ap.parse_args()
    ser = B.open_repl(); log = open(os.path.join(HERE, 'run_%s.log' % a.tag), 'w')
    def q(line, t=15):
        out = B.cmd(ser, line, t); log.write('>>> %s\n%s\n' % (line, out)); log.flush()
        for l in out.split('\n'):
            s = l.strip()
            if s and not s.startswith('>>>') and not s.startswith('[CAM') and not s.startswith('BOARD_') and not s.startswith('CAMERA_') \
                    and 'sentai.' not in s[:8] and not s.startswith('exec('):
                print(s[:260])
        return out
    q('import sentai', 5); q('print(sentai.version())', 8)
    q('print("INIT", sentai.camera.init(1, 45))', 60)
    q('print("UNIT_after_init", sentai.camera.ratio_unit(), "stats", {k: v for k, v in sentai.camera.stats().items() if k in ("phase_us","genlock_ok","pretrim_lines","ratio_unit")})')
    q(ALT % 6000, 25)
    q(INJECT, 10)
    for w in range(8):
        q(ALT % 5000, 25)
    q('sentai.camera.ratio(0,0)', 5)
    out = q('print("DMESG", sentai.rtos.dmesg())', 10)
    print('CAM events:', ' '.join(re.findall(r'E:0A[0-9A-F]{2}:\d+', out)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

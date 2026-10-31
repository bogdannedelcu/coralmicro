#!/usr/bin/env python3
"""EXP-s256 -- camera -> SD card throughput and chain stability.

Operator ask (2026-10-31): evaluate the continuous chain camera -> (JPEG) ->
SD card, single camera and 1:1 alternation, saving a frame every N frames so
the card is not flooded; how many fps survive, how fast the card really is,
whether the card runs in high-speed mode, and whether anything in the chain
wedges over time.

The board side is orchestration only (MicroPython): the frame grab, the
JPEG encode (libjpeg on M7) and the FileX write are C.  Per 30 s window the
board loop consumes every frame with peek_col() (cheap dequeue) and every N
frames encodes one JPEG (camera.jpeg) and writes it (fs.write) into
/rec/fNNNNN.jpg, rotating over 100 names.

CSV row per window:
  t_s,consumed,sensor_frames,ms,fps,saves,enc_ms_avg,write_ms_avg,bytes,kBps,errs,stall,csi_faults,phase_us

Usage:
  python3 cam_to_sd.py --mode alt --every 45 --minutes 3 [--fps 45] [--quality 75] [--tag x]
"""
import argparse, os, re, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402

WINDOW = ('exec("t0=sentai.rtos.ticks_ms()\\nf0=sentai.camera.sensor_frames()\\nn=0\\nsv=0\\nem=0\\nwm=0\\nby=0\\ner=0\\n'
          'while sentai.rtos.ticks_ms()-t0<%(win)d:\\n'
          '  try:\\n    sentai.camera.peek_col(40); n+=1\\n  except Exception as e:\\n    er+=1\\n    if er>5: break\\n'
          '  if n %% %(every)d == 0:\\n'
          '    t1=sentai.rtos.ticks_ms(); j=sentai.camera.jpeg(%(q)d, %(yuv)d); t2=sentai.rtos.ticks_ms()\\n'
          '    ok=sentai.fs.write(\'/rec/f%%05d.jpg\' %% (sv %% 100), j); t3=sentai.rtos.ticks_ms()\\n'
          '    em+=t2-t1; wm+=t3-t2; by+=len(j); sv+=1\\n'
          '    if not ok: er+=1\\n'
          'dt=sentai.rtos.ticks_ms()-t0\\ns=sentai.camera.stats()\\n'
          'print(\'WIN\',n,sentai.camera.sensor_frames()-f0,dt,(n*1000)//max(dt,1),sv,em//max(sv,1),wm//max(sv,1),by,(by*1000)//max(dt,1)//1024,er,s[\'stall_count\'],s[\'csi_queue_faults\'],s[\'phase_us\'])")')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['single', 'alt'], default='alt')
    ap.add_argument('--every', type=int, default=45)
    ap.add_argument('--minutes', type=float, default=3.0)
    ap.add_argument('--window', type=int, default=30)
    ap.add_argument('--fps', type=int, default=45)
    ap.add_argument('--quality', type=int, default=75)
    ap.add_argument('--yuv', type=int, default=0, help='1 = PXP YCbCr 4:2:0 path into the codec (build >= 1690)')
    ap.add_argument('--no-init', action='store_true')
    ap.add_argument('--tag', default=None)
    a = ap.parse_args()
    tag = a.tag or ('%s_every%d_%dmin' % (a.mode, a.every, int(a.minutes)))
    ser = B.open_repl(); B.cmd(ser, 'import sentai', 5)
    log = open(os.path.join(HERE, 'run_%s.log' % tag), 'w')
    csv = open(os.path.join(HERE, 'results_%s.csv' % tag), 'w')
    csv.write('t_s,consumed,sensor_frames,ms,fps,saves,enc_ms_avg,write_ms_avg,bytes,kBps,errs,stall,csi_faults,phase_us\n')

    def q(line, t=20):
        out = B.cmd(ser, line, t); log.write('>>> %s\n%s\n' % (line, out)); log.flush(); return out
    print(q('print(sentai.version())', 8).strip().split('\n')[-2][:70])
    if not a.no_init:
        if a.yuv:
            q('print("YUVMODE", sentai.camera.yuv_mode(1))', 5)
        out = q('print("INIT", sentai.camera.init(1, %d))' % a.fps, 60)
        for l in out.split('\n'):
            if '[cam] genlock' in l or '[cam] pre-trim' in l or l.strip().startswith('INIT'): print(l.strip()[:120])
    print('SD bus:', q('print(sentai.sdcard.bus())', 8).strip().split('\n')[-2][:80])
    q('sentai.fs.mkdir("/rec")', 10)
    q('sentai.camera.ratio(%s)' % ('1, 1' if a.mode == 'alt' else '0, 0'), 5)
    t_start = time.monotonic(); n_win = int(a.minutes * 60 / a.window + 0.5)
    for w in range(n_win):
        out = q(WINDOW % {'win': a.window * 1000, 'every': a.every, 'q': a.quality, 'yuv': a.yuv}, a.window + 40)
        m = re.search(r'WIN (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (\d+) (-?\d+|None)', out)
        t_s = int(time.monotonic() - t_start)
        if not m:
            print('window %d: no WIN row -> %s' % (w, out.strip()[-240:])); csv.write('%d\n' % t_s); csv.flush()
            if 'Traceback' in out: break
            continue
        csv.write(','.join([str(t_s)] + list(m.groups())) + '\n'); csv.flush()
        g = m.groups()
        print('t=%4ds consumed=%s sensor=%s fps=%s saves=%s enc=%sms write=%sms kB/s=%s errs=%s stall=%s csi=%s phase=%s' %
              (t_s, g[0], g[1], g[3], g[4], g[5], g[6], g[8], g[9], g[10], g[11], g[12]))
    q('sentai.camera.ratio(0,0)', 5)
    print('sync:', q('print(sentai.fs.sync())', 20).strip().split('\n')[-2][:40])
    out = q('print(sentai.rtos.dmesg())', 10)
    print('events:', ' '.join(re.findall(r'E:0[AD][0-9A-F]{2}:\d+', out))[:300])
    print('fs.stats:', q('print(sentai.fs.stats())', 8).strip().split('\n')[-2][:120])
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""EXP-s259 -- at which line width does the packed YUV420 capture break?

The 2 B/px packed YUV420 path is perfect at 640x480 and scrambled at 1280x960 (the
bytes are the right values, in the wrong places: autocorrelation peaks at 8/64 B).
The OV5640 VGA timing row reads the array 2x2-binned (1296x972) and the ISP scaler
makes 640x480 from it, so the ISP output size (0x3808..0x380B) can be re-programmed
to any size up to 1280x960 on the SAME timing and MIPI rate.  This sweeps the output
width with the receiver told the new geometry (csi_geom), 24-bit receiver, half
width (two pixels per pixel-link cycle), sensor format 0x5F.

Per config (fresh boot): csi_geom(W/2, H); csi_mode(24,0,0,1); init(1,30); sensor
0x3808..0x380B <- WxH; 0x4300 <- 0x5F; colour bars on; frame rate; dump; analyse.

Usage: python3 w420_sweep.py [--sizes 640x480,800x600,...] [--tag x] [--natural]
"""
import argparse, os, subprocess, sys, time
import numpy as np
from PIL import Image
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_csi16 import Board, RESET  # noqa: E402
from grab_packed import fetch, packed420_to_rgb  # noqa: E402
from sxga_packed_probe import analyse  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sizes', default='640x480,800x600,960x720,1024x768,1280x960')
    ap.add_argument('--tag', default=''); ap.add_argument('--natural', action='store_true'); ap.add_argument('--row', default='vga', help='vga or sxga sensor timing row'); ap.add_argument('--regw', default='', help='extra sensor writes after the format, e.g. 4837=14'); ap.add_argument('--suffix', default=''); ap.add_argument('--fps', type=int, default=30); ap.add_argument('--settle', type=int, default=0, help='D-PHY T-HSSETTLE override (csi_settle) before init')
    a = ap.parse_args(); tag = a.tag or time.strftime('%H%M%S')
    with open(os.path.join(HERE, 'w420_%s.log' % tag), 'w') as log:
        for sz in [s for s in a.sizes.split(',') if s]:
            w, h = [int(x) for x in sz.split('x')]
            print('\n=== %dx%d from the %s timing row, packed YUV420' % (w, h, a.row.upper()))
            subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
            bd = Board(log)
            try:
                bd.q('sentai.camera.yuv_mode(1)', 5)
                if a.row == 'sxga':
                    bd.q('print("HW", sentai.camera.set_hw(1280, 960))', 10)
                bd.q('print("G", sentai.camera.csi_geom(%d, %d))' % (w // 2, h), 5)
                bd.q('sentai.camera.csi_mode(24, 0, 0, 1)', 5)
                if a.settle:
                    bd.q('print("SET", sentai.camera.csi_settle(%d))' % a.settle, 5)
                out = bd.q('print("INIT", sentai.camera.init(1, %d))' % a.fps, 90)
                print('  init', bd.last(out, 'INIT'))
                for l in out.split('\n'):
                    if 'CSI probe' in l: print('   ', l.strip())
                cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
                f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF'); time.sleep(1.0)
                f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
                print('  422 frame counter rate before the switch ~ %.1f/s' % ((int(f1.split()[1]) - int(f0.split()[1])) / 1.0))
                for reg, val in ((0x3808, w >> 8), (0x3809, w & 0xFF), (0x380A, h >> 8), (0x380B, h & 0xFF)):
                    bd.q('sentai.camera.reg_write(%s, 0x%04X, 0x%02X)' % (cam, reg, val), 5)
                bd.q('sentai.camera.reg_write(%s, 0x4300, 0x5F)' % cam, 5)
                for kv in [x for x in a.regw.split(',') if x]:
                    r_, v_ = kv.split('='); bd.q('sentai.camera.reg_write(%s, 0x%s, 0x%s)' % (cam, r_, v_), 5)
                if not a.natural:
                    bd.q('sentai.camera.test_pattern(%s, 1)' % cam, 5)
                time.sleep(1.0)
                f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF'); time.sleep(2.0)
                f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
                print('  frame counter rate ~ %.1f/s' % ((int(f1.split()[1]) - int(f0.split()[1])) / 2.0))
                st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST'); print('  ', st[:200])
                bd.q('print("IP", sentai.usb.ip(1))', 10); time.sleep(4)
                buf = fetch(bd, 's259_w%d' % w, w * h * 2)
                open(os.path.join(HERE, 'p420_%s_w%dx%d_%s%s.bin' % (a.row, w, h, tag, a.suffix)), 'wb').write(buf)
                analyse(buf, w, h)
                rgb, Y, U, V = packed420_to_rgb(buf, w, h)
                Image.fromarray(rgb).save(os.path.join(HERE, 'p420_%s_w%dx%d_%s%s.png' % (a.row, w, h, tag, a.suffix)))
            except Exception as e:
                print('  FAILED:', e)
            finally:
                bd.close()


if __name__ == '__main__':
    main()

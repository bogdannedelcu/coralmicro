#!/usr/bin/env python3
"""EXP-s259 -- why is the packed YUV420 capture garbage at SXGA when it is perfect at VGA?

Per config (fresh boot): set_hw(w,h); csi_mode(24,0,0,2); init(1,fps); 0x4300 <- fmt;
dump one full packed frame (kept as .bin), then measure the structure:
  * fraction of zero bytes in lane 3 (must be ~1.0 if the [C,Y1,Y2,0] alignment holds);
  * row-to-row correlation of the Y1 lane (a natural image: > 0.8; garbage: ~0);
  * lane means / stds; sensor binning + PLL registers.

Usage: python3 sxga_packed_probe.py [--configs ...] [--tag x]
"""
import argparse, os, subprocess, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_csi16 import Board, RESET  # noqa: E402
from grab_packed import fetch, packed420_to_rgb  # noqa: E402
from PIL import Image
RX = 0x40810000


# id: (w, h, fps, fmt, wdiv)
CONFIGS = {
    'sxga30_5f': (1280, 960, 30, 0x5F, 2),
    'sxga15_5f': (1280, 960, 15, 0x5F, 2),
    'sxga30_5e': (1280, 960, 30, 0x5E, 2),
    'vga30_5f':  (640, 480, 30, 0x5F, 2),
}
REGS = [0x3814, 0x3815, 0x3820, 0x3821, 0x3034, 0x3035, 0x3036, 0x3037, 0x3108, 0x4837, 0x4300, 0x501F, 0x3808, 0x3809, 0x380A, 0x380B, 0x380C, 0x380D, 0x380E, 0x380F]


def analyse(buf, w, h):
    g = np.frombuffer(buf, dtype=np.uint8).reshape(h, w // 2, 4)
    z3 = float((g[:, :, 3] == 0).mean())
    y1 = g[:, :, 1].astype(np.float32)
    rr = [float(np.corrcoef(y1[r], y1[r + 1])[0, 1]) for r in range(0, h - 1, max(1, h // 24))]
    rr = [c for c in rr if not np.isnan(c)]
    print('  lane3 zero fraction %.3f; Y1 row-to-row corr median %.2f (min %.2f max %.2f)' % (
        z3, float(np.median(rr)), min(rr), max(rr)))
    for k in range(4):
        print('    lane %d mean %.1f std %.1f' % (k, g[:, :, k].mean(), g[:, :, k].std()))
    # does any 4-byte alignment give lane3 == 0?  (detect a byte-offset shift)
    flat = np.frombuffer(buf, dtype=np.uint8)
    for s in range(4):
        zz = float((flat[s::4] == 0).mean())
        print('    byte lane %d zero fraction %.3f' % (s, zz))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--configs', default=','.join(CONFIGS)); ap.add_argument('--tag', default=''); ap.add_argument('--restart', action='store_true', help='toggle sensor streaming (0x3008) after setting the format'); ap.add_argument('--bars', action='store_true'); ap.add_argument('--regw', default='', help='extra sensor writes after the format, e.g. 3821=00,3820=41'); ap.add_argument('--suffix', default='')
    a = ap.parse_args(); tag = a.tag or time.strftime('%H%M%S')
    with open(os.path.join(HERE, 'sxgaprobe_%s.log' % tag), 'w') as log:
        for cid in [c.strip() for c in a.configs.split(',') if c.strip()]:
            w, h, fps, fmt, wdiv = CONFIGS[cid]
            print('\n=== %s: %dx%d @%d fmt 0x%02X wdiv %d' % (cid, w, h, fps, fmt, wdiv))
            subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
            bd = Board(log)
            try:
                bd.q('sentai.camera.yuv_mode(1)', 5)
                if (w, h) != (640, 480):
                    bd.q('print("HW", sentai.camera.set_hw(%d, %d))' % (w, h), 10)
                bd.q('sentai.camera.csi_mode(24, 0, 0, %d)' % wdiv, 5)
                print('  init', bd.last(bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 90), 'INIT'))
                cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
                bd.q('sentai.camera.reg_write(%s, 0x4300, 0x%02X)' % (cam, fmt), 5)
                for kv in [x for x in a.regw.split(',') if x]:
                    r_, v_ = kv.split('='); bd.q('sentai.camera.reg_write(%s, 0x%s, 0x%s)' % (cam, r_, v_), 5)
                if a.bars:
                    bd.q('sentai.camera.test_pattern(%s, 1)' % cam, 5)
                if a.restart:
                    bd.q('sentai.camera.reg_write(%s, 0x3008, 0x42)' % cam, 5); time.sleep(0.2)
                    bd.q('sentai.camera.reg_write(%s, 0x3008, 0x02)' % cam, 5)
                time.sleep(1.0)
                errs = bd.last(bd.q('print("RX", [hex(sentai.sys.peek32(0x%X + o)) for o in (0x108, 0x10C, 0x118, 0x11C, 0x120, 0x124, 0x128)])' % RX, 10), 'RX')
                print('  RX BIT_ERR IRQ_STATUS ERRSOT ERRSOTSYNC ERRESC ERRSYNCESC ERRCONTROL:', errs[3:])
                regs = bd.last(bd.q('print("RG", [hex(sentai.camera.reg_read(%s, r)) for r in %s])' % (cam, REGS), 15), 'RG')
                print('  regs', ' '.join('%04X=%s' % (r, v.strip("'")) for r, v in zip(REGS, regs[3:].strip('[]').split(', '))))
                f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF'); time.sleep(2.0)
                f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
                print('  frame counter rate ~ %.1f/s' % ((int(f1.split()[1]) - int(f0.split()[1])) / 2.0))
                bd.q('print("IP", sentai.usb.ip(1))', 10); time.sleep(4)
                buf = fetch(bd, 's259_%s' % cid, w * h * 2)
                open(os.path.join(HERE, 'p420_%s_%s.bin' % (cid, tag)), 'wb').write(buf)
                analyse(buf, w, h)
                rgb, Y, U, V = packed420_to_rgb(buf, w, h)
                Image.fromarray(rgb).save(os.path.join(HERE, 'p420_%s_%s%s.png' % (cid, tag, ('_restart' if a.restart else '') + a.suffix)))
            finally:
                bd.close()


if __name__ == '__main__':
    main()

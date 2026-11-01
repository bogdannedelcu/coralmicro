#!/usr/bin/env python3
"""EXP-s259 / OP-S10-W24-T14 -- which CSI-2 data type does the OV5640 actually send?

The MIPI CSI-2 RX core has one payload-disable bit per supported data type
(MIPI_CSI2RX->CFG_DISABLE_PAYLOAD_0/1 at 0x40810000 + 0x12C/0x130).  Disabling the
type the sensor uses stops the frames, so the bits are a data-type detector:
  1. legacy XYUV init, baseline frame rate;
  2. disable YUV422_8BIT (bit 14) -> frames must stop if the tag is 0x1E;
  3. with YUV422 still disabled, try the 0x4814 data-type override (0x22 RGB565, 0x24
     RGB888, 0x30 user-defined) -> frames resume only if the tag really changed;
  4. restore, then switch the sensor to real RAW (0x4300=0x00, 0x501F=0x01) and look
     at the receiver with YUV422 disabled / UNSUPPORTED disabled (PAYLOAD_1 bit 16).

Single boot, legacy 24-bit receiver.  Usage: python3 dt_probe.py [--tag x]
"""
import argparse, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_csi16 import Board, RESET, peek, lanes, transitions, bar_means  # noqa: E402

RX = 0x40810000
PAY0 = RX + 0x12C
PAY1 = RX + 0x130
YUV422_8BIT = 1 << 14
YUV420 = 1 << 10
RGB565 = 1 << 18
RGB888 = 1 << 20
UDEF30 = 1 << 0
UNSUPPORTED = 1 << 16


def fps(bd, dt=2.0):
    f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
    time.sleep(dt)
    f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
    try: return (int(f1.split()[1]) - int(f0.split()[1])) / dt
    except Exception: return -1


def poke(bd, addr, val):
    bd.q('sentai.sys.poke32(0x%X, 0x%X)' % (addr, val), 5)


def peek32(bd, addr):
    return bd.last(bd.q('print("PV", hex(sentai.sys.peek32(0x%X)))' % addr, 5), 'PV').split()[-1]


def lanes_report(bd, w=640, bpp=4, tag=''):
    b = peek(bd)
    if len(b) < w * bpp:
        print('    %s: no frame (%d B)' % (tag, len(b))); return
    L = lanes(b, bpp, w)
    print('    %s: first 16 B %s' % (tag, b[:16].hex()))
    for k, l in enumerate(L):
        print('      lane %d t=%d means=%s' % (k, transitions(l), [int(x) for x in bar_means(l)]))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--tag', default=''); a = ap.parse_args()
    logp = os.path.join(HERE, 'dtprobe_%s.log' % (a.tag or time.strftime('%H%M%S')))
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    with open(logp, 'w') as log:
        bd = Board(log)
        try:
            bd.q('sentai.camera.yuv_mode(1)', 5)
            print('init', bd.last(bd.q('print("INIT", sentai.camera.init(1, 30))', 60), 'INIT'))
            cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
            print('PAYLOAD_0 =', peek32(bd, PAY0), ' PAYLOAD_1 =', peek32(bd, PAY1))
            bd.q('sentai.camera.test_pattern(%s, 1)' % cam, 5); time.sleep(0.5)
            print('baseline fps %.1f' % fps(bd)); lanes_report(bd, tag='baseline')

            print('\n-- disable YUV422_8BIT payload')
            poke(bd, PAY0, YUV422_8BIT); time.sleep(0.3)
            print('fps %.1f (0 => the stream is tagged 0x1E)' % fps(bd))

            for dt, name, enable_bit in ((0x22, 'RGB565', RGB565), (0x24, 'RGB888', RGB888), (0x30, 'UDEF30', None)):
                print('\n-- 0x4814 <- 0x%02X (%s) with YUV422 disabled' % (dt, name))
                print('   wr', bd.last(bd.q('print("WR", sentai.camera.reg_write(%s, 0x4814, 0x%02X))' % (cam, dt), 5), 'WR'),
                      'rd', bd.last(bd.q('print("RD", hex(sentai.camera.reg_read(%s, 0x4814)))' % cam, 5), 'RD'))
                time.sleep(0.5)
                print('   fps %.1f' % fps(bd)); lanes_report(bd, tag=name)
                bd.q('sentai.camera.reg_write(%s, 0x4814, 0x2A)' % cam, 5)

            print('\n-- restore payload enable; 0x4814 back to 0x2A')
            poke(bd, PAY0, 0); time.sleep(0.3); print('fps %.1f' % fps(bd))

            print('\n-- sensor RAW mode (0x4300=0x00, 0x501F=0x01), YUV422 payload enabled')
            print('   3034 (bit mode) =', bd.last(bd.q('print("B", hex(sentai.camera.reg_read(%s, 0x3034)))' % cam, 5), 'B'))
            bd.q('sentai.camera.reg_write(%s, 0x4300, 0x00)' % cam, 5)
            bd.q('sentai.camera.reg_write(%s, 0x501F, 0x01)' % cam, 5)
            time.sleep(0.5); print('   fps %.1f' % fps(bd)); lanes_report(bd, tag='RAW')
            print('   PAYLOAD_1 UNSUPPORTED disabled:'); poke(bd, PAY1, UNSUPPORTED); time.sleep(0.3)
            print('   fps %.1f' % fps(bd)); poke(bd, PAY1, 0)
            print('   RX status regs: BIT_ERR', peek32(bd, RX + 0x108), 'IRQ_STATUS', peek32(bd, RX + 0x10C))
            bd.q('sentai.camera.reg_write(%s, 0x501F, 0x00)' % cam, 5)
            bd.q('sentai.camera.reg_write(%s, 0x4300, 0x3F)' % cam, 5)
            time.sleep(0.5); print('\nback to YUV422: fps %.1f' % fps(bd)); lanes_report(bd, tag='YUV422 again')
            bd.q('sentai.camera.test_pattern(%s, 0)' % cam, 5)
        finally:
            bd.close()
    print('log:', logp)


if __name__ == '__main__':
    main()

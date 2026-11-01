#!/usr/bin/env python3
"""EXP-s264 follow-up: after the SXGA fps re-keying (keys = measured fps; SDK patch 0007, build 1756)
check (1) legacy XYUV capture at SXGA key 30 is refused with -12 (that mode hung the board in s260),
(2) legacy key 22 initialises and the CSI frame counter measures 22.6/s, (3) packed key 30 is accepted.
The lockstep run itself is run_lockstep.py --keys 15,22,30."""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
CR3 = 0x40800008


def frm(bd):
    return (int(bd.last(bd.q('print("V", sentai.sys.peek32(0x%X))' % CR3, 5), 'V').split()[1]) >> 16) & 0xFFFF


def boot(log):
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    bd = Board(log); bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('print("HW", sentai.camera.set_hw(1280, 960))', 10)
    return bd


with open(os.path.join(HERE, 'rekey_check.log'), 'w') as log:
    bd = boot(log)
    print('version:', bd.q('print(sentai.version())', 8).strip().split('\n')[-2][:60])
    r = bd.last(bd.q('print("INIT", sentai.camera.init(1, 30))', 30), 'INIT'); print('legacy SXGA init(1, 30):', r, '(expect -12)')
    r = bd.last(bd.q('print("INIT", sentai.camera.init(1, 36))', 30), 'INIT'); print('legacy SXGA init(1, 36):', r, '(expect -10, key gone)')
    r = bd.last(bd.q('print("INIT", sentai.camera.init(1, 22))', 90), 'INIT'); print('legacy SXGA init(1, 22):', r, '(expect 0)')
    time.sleep(2.0); f0 = frm(bd); t0 = time.time(); time.sleep(4.0); f1 = frm(bd)
    print('  CSI FRMCNT rate: %.2f/s (expect 22.6)' % (((f1 - f0) & 0xFFFF) / (time.time() - t0)))
    bd.close()
    bd = boot(log)
    bd.q('sentai.camera.packed420(1)', 5)
    r = bd.last(bd.q('print("INIT", sentai.camera.init(1, 30))', 90), 'INIT'); print('packed SXGA init(1, 30):', r, '(expect 0)')
    time.sleep(2.0); f0 = frm(bd); t0 = time.time(); time.sleep(4.0); f1 = frm(bd)
    print('  CSI FRMCNT rate: %.2f/s (expect 30.0)' % (((f1 - f0) & 0xFFFF) / (time.time() - t0)))
    bd.close()

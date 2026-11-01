#!/usr/bin/env python3
"""EXP-s261 helper: does the CSI really overwrite a poisoned padding byte, and where?
Poke 0xFFFFFFFF into words of the in-flight buffer (DMASA_FB1) and read them back a few frames later."""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
CSI = 0x40800000
subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
with open(os.path.join(HERE, 'marker_probe.log'), 'w') as log:
    bd = Board(log)
    bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('sentai.camera.packed420(1)', 5)
    print('init', bd.last(bd.q('print("INIT", sentai.camera.init(1, 45))', 90), 'INIT')); time.sleep(3)
    def r32(a): return int(bd.last(bd.q('print("V", sentai.sys.peek32(0x%X))' % a, 5), 'V').split()[1])
    cr3 = r32(CSI + 0x08); cr18 = r32(CSI + 0x48); imag = r32(CSI + 0x34); fbuf = r32(CSI + 0x30)
    fb1 = r32(CSI + 0x28); fb2 = r32(CSI + 0x2C)
    print('CR3 %08x (FRMCNT %d) CR18 %08x IMAG %08x FBUF %08x DMASA_FB1 %08x FB2 %08x' % (cr3, cr3 >> 16, cr18, imag, fbuf, fb1, fb2))
    pitch = 320 * 4
    for name, buf in (('FB1', fb1), ('FB2', fb2)):
        addrs = {'row100 last word': buf + 100 * pitch + pitch - 4, 'row100 first word': buf + 100 * pitch,
                 'row479 last word': buf + 479 * pitch + pitch - 4, 'row0 first word': buf}
        for k, a in addrs.items():
            bd.q('sentai.sys.poke32(0x%X, 0xFFFFFFFF)' % a, 5)
        t0 = time.time(); vals = {}
        time.sleep(0.2)
        for k, a in addrs.items():
            vals[k] = r32(a)
        fc = r32(CSI + 0x08) >> 16
        print('%s (%08x) after 200 ms (FRMCNT now %d): %s' % (name, buf, fc, {k: '%08x' % v for k, v in vals.items()}))
    # grab a frame via the camera task so the queue moves, then re-check both
    bd.q('print("PK", sentai.camera.peek_row(16))', 10)
    fb1b = r32(CSI + 0x28); fb2b = r32(CSI + 0x2C); print('after a grab: DMASA_FB1 %08x FB2 %08x' % (fb1b, fb2b))
    bd.close()

#!/usr/bin/env python3
"""W24-T15 step 4 feasibility: does the CSI DMA alternate FB1/FB2 every N lines when IMAG_PARA.height
is set smaller than the sensor frame?  Packed YUV420 SXGA (true-30 row), receiver geometry
csi_geom(640, H) for H in (960, 480, 128): the CSI frame counter rate should scale 1x / 2x / 7.5x if
the DMA restarts mid-frame; camera.stats fault counters and a peek of the first bytes are printed."""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
for H in (960, 480, 128):
    print('\n=== receiver height %d (sensor 1280x960 @30, packed YUV420, 640 cycles/line)' % H)
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    with open(os.path.join(HERE, 'csi_slice_h%d.log' % H), 'w') as log:
        bd = Board(log)
        bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('sentai.camera.set_hw(1280, 960)', 10)
        bd.q('sentai.camera.packed420(1)', 5); bd.q('print("G", sentai.camera.csi_geom(640, %d))' % H, 5)
        out = bd.q('print("INIT", sentai.camera.init(1, 36))', 90); print('  init', bd.last(out, 'INIT'))
        for l in out.split('\n'):
            if 'CSI probe' in l: print('   ', l.strip()[:150])
        time.sleep(2)
        def frm(): return (int(bd.last(bd.q('print("V", sentai.sys.peek32(0x40800008))', 5), 'V').split()[1]) >> 16) & 0xFFFF
        f0 = frm(); t0 = time.time(); time.sleep(3); f1 = frm(); dt = time.time() - t0
        print('  CSI FRMCNT rate: %.1f /s' % (((f1 - f0) & 0xFFFF) / dt))
        sr = int(bd.last(bd.q('print("V", sentai.sys.peek32(0x40800018))', 5), 'V').split()[1])
        print('  CSI SR 0x%08x  frames_total/sensor_frames %s' % (sr, bd.last(bd.q('print("SF", sentai.camera.sensor_frames(), sentai.camera.frame_count())', 5), 'SF')))
        st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST'); print('  ', st[:230])
        pk = bd.last(bd.q('print("PK", list(sentai.camera.peek_row(16)))', 10), 'PK'); print('  peek:', pk[:80])
        bd.close()

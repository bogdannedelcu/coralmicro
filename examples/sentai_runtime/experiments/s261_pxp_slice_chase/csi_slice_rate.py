#!/usr/bin/env python3
"""W24-T15 step 4: slice-completion rate.  Receiver height H (960 = full frame, 128 = slice).
sensor_frames() counts CSI DMA frame-done events (only while a consumer dequeues), so run the no-TPU
pipeline as the consumer and measure the counter rate: ~30/s = one DMA 'frame' per sensor frame
(the DMA stops until the next SOF); ~225/s = the DMA restarts on FB1/FB2 every 128 lines."""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
for H in (960, 128, 480):
    print('\n=== receiver height %d' % H)
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    with open(os.path.join(HERE, 'csi_slice_rate_h%d.log' % H), 'w') as log:
        bd = Board(log)
        bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('sentai.camera.set_hw(1280, 960)', 10)
        bd.q('sentai.camera.packed420(1)', 5); bd.q('sentai.camera.csi_geom(640, %d)' % H, 5)
        print('  init', bd.last(bd.q('print("INIT", sentai.camera.init(1, 36))', 90), 'INIT'))
        bd.q('print("LOAD", sentai.tpu.load("/models/c2f_yuv420_sxga_relu_int8_edgetpu.tflite"))', 90)
        bd.q('sentai.pipeline.debug_no_invoke(1)', 5)
        bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(2)
        a = bd.last(bd.q('print("C", sentai.camera.sensor_frames(), sentai.camera.frame_count(), (sentai.sys.peek32(0x40800008) >> 16) & 0xFFFF)', 5), 'C')
        t0 = time.time(); time.sleep(3.0)
        b = bd.last(bd.q('print("C", sentai.camera.sensor_frames(), sentai.camera.frame_count(), (sentai.sys.peek32(0x40800008) >> 16) & 0xFFFF)', 5), 'C')
        dt = time.time() - t0
        a_ = [int(x) for x in a.split()[1:]]; b_ = [int(x) for x in b.split()[1:]]
        print('  rates over %.1f s: sensor_frames %.1f/s  frame_count %.1f/s  CSI FRMCNT %.1f/s' % (dt, (b_[0]-a_[0])/dt, (b_[1]-a_[1])/dt, ((b_[2]-a_[2]) & 0xFFFF)/dt))
        pst = bd.last(bd.q('print("P", sentai.pipeline.prep_stats())', 10), 'P'); print('  prep:', pst[:160])
        bd.q('print("STOP", sentai.pipeline.stop())', 30)
        st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST'); print('  cam:', st[:200])
        bd.close()

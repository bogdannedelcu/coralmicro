#!/usr/bin/env python3
"""EXP-s260 helper: is the VGA45 pipeline fps limited by the scene (AEC exposure) or by software?
Legacy VGA45, same boot: pipeline 10 s with colour bars (bright, short exposure) vs natural scene (dark room);
also prints sensor_frames / frame_count deltas and camera.fps() while running."""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import pipeline, MODELS  # noqa: E402

subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
with open(os.path.join(HERE, 'probe_fps.log'), 'w') as log:
    bd = Board(log)
    bd.q('sentai.camera.yuv_mode(1)', 5)
    print('init', bd.last(bd.q('print("INIT", sentai.camera.init(1, 45))', 90), 'INIT'))
    cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
    for reg in (0x3A00, 0x3A02, 0x3A03, 0x3A14, 0x3A15, 0x380E, 0x380F, 0x3500, 0x3501, 0x3502, 0x350B):
        print('  %04X = %s' % (reg, bd.last(bd.q('print("R", hex(sentai.camera.reg_read(%s, 0x%04X)))' % (cam, reg), 5), 'R').split()[-1]))
    def counters(tag):
        a = bd.q('print("C", sentai.camera.sensor_frames(), sentai.camera.frame_count())', 5)
        time.sleep(2.0)
        b = bd.q('print("C", sentai.camera.sensor_frames(), sentai.camera.frame_count())', 5)
        print('  %s counters: %s -> %s (2 s)' % (tag, bd.last(a, 'C'), bd.last(b, 'C')))
        print('  camera.fps():', bd.last(bd.q('print("F", sentai.camera.fps())', 5), 'F'))
    counters('idle')
    print('load', bd.last(bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(640, 480)], 90), 'LOAD'))
    bd.q('sentai.tpu.invoke()', 30)
    bd.q('sentai.camera.test_pattern(%s, 1)' % cam, 5); time.sleep(0.5)
    p = pipeline(bd, 10); print('  BARS   pipeline fps %.1f invoke %.1f pxp %.1f grab %.1f sem %.1f' % (p['fps'], p['invoke_ms'], p['pxp_ms'], p['grab_ms'], p['sem_ms']))
    bd.q('sentai.camera.test_pattern(%s, 0)' % cam, 5); time.sleep(1.0)
    for reg in (0x3500, 0x3501, 0x3502, 0x350B, 0x380E, 0x380F):
        print('  %04X = %s' % (reg, bd.last(bd.q('print("R", hex(sentai.camera.reg_read(%s, 0x%04X)))' % (cam, reg), 5), 'R').split()[-1]))
    p = pipeline(bd, 10); print('  NATURAL pipeline fps %.1f invoke %.1f pxp %.1f grab %.1f sem %.1f' % (p['fps'], p['invoke_ms'], p['pxp_ms'], p['grab_ms'], p['sem_ms']))
    bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(1.0); counters('pipeline running'); bd.q('print("STOP", sentai.pipeline.stop())', 30)
    print('  stats:', bd.last(bd.q('print("S", sentai.pipeline.stats())', 10), 'S')[:300])
    bd.close()

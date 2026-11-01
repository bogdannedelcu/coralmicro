#!/usr/bin/env python3
"""EXP-s262: (1) is the TPU output in OCRAM-ring mode identical to staging mode for the same
input (colour bars, SXGA option B)?  (2) chunk size A/B for the ring (36 KB vs 120 KB)."""
import os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS, pipeline  # noqa: E402
subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
with open(os.path.join(HERE, 'ring_check.log'), 'w') as log:
    bd = Board(log)
    bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('sentai.camera.set_hw(1280, 960)', 10); bd.q('sentai.camera.packed420(1)', 5)
    print('init', bd.last(bd.q('print("INIT", sentai.camera.init(1, 30))', 90), 'INIT'))
    print('load', bd.last(bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(1280, 960)], 90), 'LOAD'))
    cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
    bd.q('sentai.tpu.invoke()', 30); time.sleep(10)
    def fp(scene_mode, ring, secs=4.0):
        bd.q('sentai.camera.test_pattern(%s, %d)' % (cam, scene_mode), 5); time.sleep(0.8)
        bd.q('print("RG", sentai.pipeline.ocram_ring(%d))' % ring, 10)
        bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(secs)
        bd.q('print("STOP", sentai.pipeline.stop())', 30)
        o = bd.q('o = sentai.tpu.output(0); o1 = sentai.tpu.output(1); print("H", len(o), sum(o), sum(o[:4000]), len(o1), sum(o1), list(o[100:108]))', 20)
        return bd.last(o, 'H')[2:].strip()
    print('  bars, staging :', fp(1, 0))
    print('  bars, ring    :', fp(1, 1))
    print('  bars, staging :', fp(1, 0))
    print('  bars, ring    :', fp(1, 1))
    print('  hband, staging:', fp(2, 0))
    print('  hband, ring   :', fp(2, 1))
    bd.q('sentai.camera.test_pattern(%s, 0)' % cam, 5)
    for chunk in (36864, 122880):
        bd.q('print("CS", sentai.tpu.chunk_size(%d))' % chunk, 5)
        bd.q('sentai.pipeline.ocram_ring(1)', 10); bd.q('sentai.pipeline.ring_stats(1)', 5)
        p = pipeline(bd, 12)
        rs = bd.last(bd.q('print("RS", sentai.pipeline.ring_stats())', 5), 'RS')
        print('  ring chunk %6d: fps %.1f invoke %.1f ms cycle %.1f pxp %.1f fail %d | %s' % (chunk, p['fps'], p['invoke_ms'], p['cycle_ms'], p['pxp_ms'], p['fail'], rs))
    bd.close()

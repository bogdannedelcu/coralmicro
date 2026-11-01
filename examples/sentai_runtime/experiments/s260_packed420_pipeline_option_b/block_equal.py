#!/usr/bin/env python3
"""EXP-s260: does the PXP 16x16 block size change the output?  VGA, legacy and option B:
run the no-TPU pipeline with 8x8 then 16x16 blocks, dump staging slot 0 each time (colour bars
then natural scene) and compare on the host."""
import os, subprocess, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS  # noqa: E402

def dump(bd, n):
    bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(1.5)
    bd.q('print("STOP", sentai.pipeline.stop())', 30)
    bd.q('print("DS", sentai.pipeline.dump_staging("/s260_eq.bin", 0, %d))' % n, 60)
    p = subprocess.run(['curl', '-s', '-m', '60', 'http://10.0.0.1/api/raw/s260_eq.bin'], capture_output=True)
    bd.q('sentai.fs.remove("/s260_eq.bin")', 10)
    return np.frombuffer(p.stdout, dtype=np.uint8) if len(p.stdout) == n else None

for packed in (False, True):
    print('\n=== VGA45 %s' % ('option B' if packed else 'legacy'))
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    with open(os.path.join(HERE, 'block_equal_%s.log' % ('b' if packed else 'legacy')), 'w') as log:
        bd = Board(log)
        bd.q('sentai.camera.yuv_mode(1)', 5)
        if packed: bd.q('sentai.camera.packed420(1)', 5)
        print('  init', bd.last(bd.q('print("INIT", sentai.camera.init(1, 45))', 90), 'INIT'))
        bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(640, 480)], 90)
        bd.q('sentai.pipeline.debug_no_invoke(1)', 5)
        bd.q('print("IP", sentai.usb.ip(1))', 10); time.sleep(4)
        cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
        n = 640 * 480 * 3 // 2
        for scene in ('bars', 'hband', 'natural'):
            bd.q('sentai.camera.test_pattern(%s, %d)' % (cam, {'bars': 1, 'hband': 2, 'natural': 0}[scene]), 5); time.sleep(0.8)
            bd.q('sentai.camera.pxp_block16(0)', 5); a = dump(bd, n)
            bd.q('sentai.camera.pxp_block16(1)', 5); b = dump(bd, n)
            if a is None or b is None: print('  %s: dump failed' % scene); continue
            d = np.abs(a.astype(np.int16) - b.astype(np.int16))
            print('  %s: identical=%s  bytes differing=%d / %d  mean|diff|=%.3f  max=%d' % (scene, bool((d == 0).all()), int((d > 0).sum()), n, float(d.mean()), int(d.max())))
        bd.close()

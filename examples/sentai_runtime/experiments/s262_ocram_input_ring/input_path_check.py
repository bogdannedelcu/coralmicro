#!/usr/bin/env python3
"""Does the camera frame actually reach the Edge TPU in the YUV420 pipeline?
VGA45, legacy XYUV, c2f YUV420 model: run the pipeline on colour bars, stop, read tpu.output_hash();
repeat on the natural scene and on the HBAND pattern.  Equal hashes => the TPU input is not the staged frame.
Also: fill input slots explicitly via tpu.set_input_slot and compare."""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS  # noqa: E402
subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
with open(os.path.join(HERE, 'input_path_check.log'), 'w') as log:
    bd = Board(log)
    bd.q('sentai.camera.yuv_mode(1)', 5)
    print('init', bd.last(bd.q('print("INIT", sentai.camera.init(1, 45))', 90), 'INIT'))
    print('load', bd.last(bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(640, 480)], 90), 'LOAD'))
    cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
    print('direct_tensor flag:', bd.q('print(dir(sentai.pipeline))', 5).count('direct'))
    for scene, mode in (('bars', 1), ('hband', 2), ('natural', 0), ('bars again', 1)):
        bd.q('sentai.camera.test_pattern(%s, %d)' % (cam, mode), 5); time.sleep(0.8)
        bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(3.0)
        st = bd.last(bd.q('print("ST", sentai.pipeline.stats())', 10), 'ST')
        bd.q('print("STOP", sentai.pipeline.stop())', 30)
        h = bd.last(bd.q('print("H", sentai.tpu.output_hash())', 10), 'H')
        print('  %-11s pipeline stats %s  output_hash %s' % (scene, st[3:60], h[2:]))
    # explicit input: zeros vs 0x80 via set_input_slot (if it accepts bytes)
    out = bd.q('import sentai; b0 = bytes(640*480); r = sentai.tpu.set_input_slot(1, b0); print("SI", r)', 30)
    print('  set_input_slot(1, zeros):', bd.last(out, 'SI'), [l for l in out.split('\n') if 'Error' in l][-1:] )
    out = bd.q('print("INV", sentai.tpu.invoke()); print("H", sentai.tpu.output_hash())', 30)
    print('  invoke zeros:', bd.last(out, 'INV'), bd.last(out, 'H'))
    out = bd.q('b1 = bytes([0x80])*(640*480); print("SI", sentai.tpu.set_input_slot(1, b1)); print("INV", sentai.tpu.invoke()); print("H", sentai.tpu.output_hash())', 60)
    print('  invoke 0x80:', bd.last(out, 'SI'), bd.last(out, 'INV'), bd.last(out, 'H'))
    bd.close()

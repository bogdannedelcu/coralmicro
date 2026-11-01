#!/usr/bin/env python3
"""Decisive check: does the staged camera frame reach the Edge TPU (YUV420 dual-input, direct pointer-patch path)?
1. pipeline on colour bars -> tpu.output_hash(0); pipeline on natural scene -> hash (must differ if frames reach the TPU)
2. set_input_slot(0, zeros / 0x80) on the UV tensor + invoke -> hashes (arena path, reference)
3. pipeline again -> hash: if it equals the last arena-content hash, the pipeline never fed the TPU
4. same with pipeline.direct_tensor(0) (legacy memcpy path) if the binding exists."""
import os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS  # noqa: E402
subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
with open(os.path.join(HERE, 'input_path_check2.log'), 'w') as log:
    bd = Board(log)
    bd.q('sentai.camera.yuv_mode(1)', 5)
    print('init', bd.last(bd.q('print("INIT", sentai.camera.init(1, 45))', 90), 'INIT'))
    print('load', bd.last(bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(640, 480)], 90), 'LOAD'))
    cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
    d = bd.q('print(dir(sentai.pipeline))', 5); print('direct binding:', [w for w in ('direct_tensor', 'direct') if w in d])
    def run_pipe(scene, mode, secs=3.0):
        bd.q('sentai.camera.test_pattern(%s, %d)' % (cam, mode), 5); time.sleep(0.8)
        bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(secs)
        bd.q('print("STOP", sentai.pipeline.stop())', 30)
        o = bd.q('o = sentai.tpu.output(0); print("H", len(o), sum(o), sum(o[:4000]), list(o[100:112]))', 20); return bd.last(o, 'H')[2:].strip() or [l for l in o.split('\n') if 'Error' in l][-1:]
    print('  pipeline bars    :', run_pipe('bars', 1))
    print('  pipeline natural :', run_pipe('natural', 0))
    print('  pipeline hband   :', run_pipe('hband', 2))
    out = bd.q('print("INV", sentai.tpu.invoke()); o = sentai.tpu.output(0); print("H", len(o), sum(o), sum(o[:4000]), list(o[100:112]))', 60)
    print('  bare tpu.invoke() (arena as is):', bd.last(out, 'INV'), bd.last(out, 'H'))
    out = bd.q('print("INV", sentai.tpu.invoke()); o = sentai.tpu.output(0); print("H", len(o), sum(o), sum(o[:4000]), list(o[100:112]))', 60)
    print('  bare tpu.invoke() again         :', bd.last(out, 'INV'), bd.last(out, 'H'))
    print('  pipeline bars    :', run_pipe('bars', 1))
    print('  pipeline natural :', run_pipe('natural', 0))
    if 'direct_tensor' in d:
        print('  direct_tensor(0):', bd.last(bd.q('print("DT", sentai.pipeline.direct_tensor(0))', 5), 'DT'))
        print('  pipeline bars    (legacy path):', run_pipe('bars', 1))
        print('  pipeline natural (legacy path):', run_pipe('natural', 0))
    bd.q('sentai.camera.test_pattern(%s, 0)' % cam, 5)
    bd.close()

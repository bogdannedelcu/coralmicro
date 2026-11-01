#!/usr/bin/env python3
"""EXP-s264 helper: why does the pipeline process ~92 % of the camera frames at SXGA key 15
(camera 11.2/s, pipeline 10.2/s) instead of every frame?  Snapshots of the pipeline trace ring
(128 events) in steady state: camera frame_seq of every grab (kind 3), sem waits (1->2),
infer waits (10->11), invoke spans (12->14) and TPU input done (13).

Usage: python3 trace_probe.py [key=15] [snapshots=3]
"""
import ast, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
sys.path.insert(0, os.path.join(HERE, '..', 's260_packed420_pipeline_option_b'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS, kv  # noqa: E402
KEY = int(sys.argv[1]) if len(sys.argv) > 1 else 15
SNAPS = int(sys.argv[2]) if len(sys.argv) > 2 else 3
NAMES = {1: 'P_wait_free', 2: 'P_got_free', 3: 'P_grab', 4: 'P_pxp_start', 5: 'P_pxp_end', 6: 'P_done',
         7: 'C_sync', 8: 'C_slice', 10: 'I_wait_prep', 11: 'I_got_prep', 12: 'I_invoke_start', 13: 'I_input_done', 14: 'I_invoke_end'}


def trace(bd):
    out = bd.q('print("TR", sentai.pipeline.trace())', 10)
    m = re.search(r'TR (\(.*\)|\[.*\])', out, re.S)
    return list(ast.literal_eval(m.group(1))) if m else []


subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
with open(os.path.join(HERE, 'trace_probe_k%d.log' % KEY), 'w') as log:
    bd = Board(log)
    bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('print("HW", sentai.camera.set_hw(1280, 960))', 10)
    bd.q('sentai.camera.packed420(1)', 5)
    print('init', bd.last(bd.q('print("INIT", sentai.camera.init(1, %d))' % KEY, 90), 'INIT')); time.sleep(1.0)
    cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
    v = int(bd.last(bd.q('print("NM", sentai.camera.reg_read(%s, 0x3A00))' % cam, 5), 'NM').split()[-1])
    bd.q('sentai.camera.reg_write(%s, 0x3A00, %d)' % (cam, v & ~0x04), 5)
    bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(1280, 960)], 90); bd.q('sentai.tpu.invoke()', 30)
    bd.q('print("RG", sentai.pipeline.ocram_ring(1))', 10)
    bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(10.0)   # warm-up / autotune
    for s in range(SNAPS):
        bd.q('sentai.pipeline.trace(1)', 5); time.sleep(1.3)
        ev = trace(bd)
        if not ev:
            print('snapshot %d: no events' % s); continue
        ev.sort(key=lambda e: e[0])
        t0 = ev[0][0]
        print('\n--- snapshot %d: %d events over %.0f ms' % (s, len(ev), (ev[-1][0] - t0) / 1000.0))
        grabs = [e for e in ev if e[1] == 3]
        seqs = [e[3] for e in grabs]
        print('  grab frame_seq:', seqs)
        print('  grab dt ms:', [round((b[0] - a[0]) / 1000.0, 1) for a, b in zip(grabs, grabs[1:])])
        gaps = [(a, b) for a, b in zip(seqs, seqs[1:]) if b - a != 1]
        print('  seq gaps (skipped camera frames):', gaps if gaps else 'none')
        for e in ev:
            print('   %8.1f  %-15s slot %d seq %d' % ((e[0] - t0) / 1000.0, NAMES.get(e[1], str(e[1])), e[2], e[3]))
    bd.q('print("STOP", sentai.pipeline.stop())', 30)
    st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST'); d = kv(st)
    print('\ncam stats:', ' '.join('%s=%s' % (k, d[k]) for k in ('stale_skips', 'grab_retry', 'producer_overruns', 'frames_total', 'csi_queue_faults') if k in d))
    bd.close()

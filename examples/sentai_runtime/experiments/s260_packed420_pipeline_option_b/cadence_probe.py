#!/usr/bin/env python3
"""EXP-s260 follow-up: why 22.5 and not 30 prepared fps at SXGA30 option B?
Measures the CSI hardware frame counter rate (CR3.FRMCNT) in legacy vs packed mode, then runs the
no-TPU pipeline and prints the camera stats and the pipeline trace (grab/PXP cadence)."""
import os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS, kv  # noqa: E402
from prep_only import prep_run  # noqa: E402
CR3 = 0x40800008
FPS = int(sys.argv[1]) if len(sys.argv) > 1 else 30
for packed in (False, True):
    print('\n=== SXGA fps-key %d %s' % (FPS, 'option B packed' if packed else 'legacy'))
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    with open(os.path.join(HERE, 'cadence_%s.log' % ('b' if packed else 'legacy')), 'w') as log:
        bd = Board(log)
        bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('print("HW", sentai.camera.set_hw(1280, 960))', 10)
        if packed: bd.q('sentai.camera.packed420(1)', 5)
        print('  init', bd.last(bd.q('print("INIT", sentai.camera.init(1, %d)' % FPS + ')', 90), 'INIT')); time.sleep(3)
        def frm(): return (int(bd.last(bd.q('print("V", sentai.sys.peek32(0x%X))' % CR3, 5), 'V').split()[1]) >> 16) & 0xFFFF
        f0 = frm(); t0 = time.time(); time.sleep(4.0); f1 = frm(); dt = time.time() - t0
        print('  CSI FRMCNT rate idle: %.1f frames/s' % (((f1 - f0) & 0xFFFF) / dt))
        bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(1280, 960)], 90)
        bd.q('sentai.pipeline.debug_no_invoke(1)', 5)
        prep_run(bd, 8)
        bd.q('sentai.pipeline.trace(1)', 5)   # reset trace ring
        f0 = frm(); t0 = time.time()
        p = prep_run(bd, 10)
        f1 = frm(); dt = time.time() - t0
        print('  during prep run: CSI FRMCNT rate %.1f/s | prepared %.1f fps | pxp %.1f grab %.1f sem %.1f total %.1f ms' % (
            ((f1 - f0) & 0xFFFF) / dt, p['prep_fps'], p['pxp_ms'], p['grab_ms'], p['sem_ms'], p['total_ms']))
        st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST')
        d = kv(st); print('  cam stats: ' + ' '.join('%s=%s' % (k, d[k]) for k in ('stale_skips', 'grab_retry', 'grab_fatal', 'drain_timeout', 'producer_overruns', 'frames_total', 'period_us', 'vblank_us', 'csi_queue_faults') if k in d))
        tr = bd.q('print("TR", sentai.pipeline.trace())', 10)
        m = re.search(r'TR (\(.*\)|\[.*\])', tr, re.S)
        if m:
            import ast
            ev = list(ast.literal_eval(m.group(1)))
            # events are tuples (kind, slot, seq, t_us)? print the last 24 raw
            print('  trace (last 24 events):'); 
            for e in ev[-24:]: print('    ', e)
        bd.close()

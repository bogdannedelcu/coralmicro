#!/usr/bin/env python3
"""EXP-s260: why 28.6 and not 30 prepared fps on the true-30 SXGA row in option B?
Runs the no-TPU pipeline and reads the pipeline trace ring (GPT1 us timestamps): per-frame PXP
duration (kind 4 -> 5), grab wait (kind 1/2 -> 3), and the interval between consecutive prepared
frames, to see how often a frame boundary is missed.  Also for the 22.6 fps row for comparison."""
import ast, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS  # noqa: E402
from prep_only import prep_run  # noqa: E402

def trace(bd):
    out = bd.q('print("TR", sentai.pipeline.trace())', 10)
    m = re.search(r'TR (\(.*\)|\[.*\])', out, re.S)
    return list(ast.literal_eval(m.group(1))) if m else []

B16 = os.environ.get('B16', '0')
for fps in (int(a) for a in (sys.argv[1:] or ['36', '30'])):
    print('\n=== SXGA fps key %d, option B, no TPU, pxp_block16=%s' % (fps, B16))
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    with open(os.path.join(HERE, 'jitter_f%d.log' % fps), 'w') as log:
        bd = Board(log)
        bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('sentai.camera.set_hw(1280, 960)', 10); bd.q('sentai.camera.packed420(1)', 5)
        print('  init', bd.last(bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 90), 'INIT')); time.sleep(2)
        bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(1280, 960)], 90)
        bd.q('sentai.pipeline.debug_no_invoke(1)', 5)
        bd.q('print("B16", sentai.camera.pxp_block16(%s))' % B16, 5)
        prep_run(bd, 8)
        bd.q('sentai.pipeline.trace(1)', 5)
        p = prep_run(bd, 6)
        ev = trace(bd)
        print('  prepared %.1f fps | pxp %.1f grab %.1f total %.1f ms | %d trace events' % (p['prep_fps'], p['pxp_ms'], p['grab_ms'], p['total_ms'], len(ev)))
        # events: (t_us, kind, slot, seq); kinds: 1/2 grab start/end?, 3 grab done, 4 pxp start, 5 pxp end, 6 prep done
        kinds = {}
        for e in ev: kinds.setdefault(e[1], []).append(e)
        print('  kinds present:', {k: len(v) for k, v in sorted(kinds.items())})
        pxp = []; starts = {}
        for t, k, slot, seq in ev:
            if k == 4: starts[seq] = t
            if k == 5 and seq in starts: pxp.append((t - starts[seq]) / 1e3)
        if pxp: print('  PXP per frame (ms): n=%d min %.1f mean %.1f max %.1f  values %s' % (len(pxp), min(pxp), sum(pxp)/len(pxp), max(pxp), [round(x, 1) for x in pxp]))
        done = sorted(t for t, k, s, q in ev if k == 5)
        iv = [(b - a) / 1e3 for a, b in zip(done, done[1:])]
        if iv: print('  interval between prepared frames (ms): %s' % [round(x, 1) for x in iv])
        g3 = sorted(t for t, k, s, q in ev if k == 3)
        g4 = sorted(t for t, k, s, q in ev if k == 4)
        bd.close()

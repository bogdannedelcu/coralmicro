#!/usr/bin/env python3
"""EXP-s260 follow-up (operator question): without the USB limit, how many frames per
second can camera + PXP deliver, ready for the Edge TPU, in option B vs legacy?

Runs the detection pipeline with sentai.pipeline.debug_no_invoke(1): PrepTask grabs
frames and runs the PXP exactly as in production, InferTask skips the TPU.  Reports
prepared frames per second and the per-frame PXP / grab / total prep times.

Usage: python3 prep_only.py [--seconds 15] [--phases vga_legacy,vga_b,sxga_legacy,sxga_b]
"""
import argparse, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import PHASES, MODELS, kv  # noqa: E402


def prep_run(bd, seconds):
    bd.q('sentai.pipeline.prep_reset()', 5); bd.q('sentai.pipeline.infer_reset()', 5)
    bd.q('print("START", sentai.pipeline.start(0.5))', 30)
    t0 = time.time(); time.sleep(seconds)
    pst = kv(bd.q('print("PSTATS", sentai.pipeline.prep_stats())', 10))
    wall = time.time() - t0
    bd.q('print("STOP", sentai.pipeline.stop())', 30)
    f = pst.get('frames', 0) or 1
    return dict(prep_fps=round(pst.get('frames', 0) / wall, 1), frames=pst.get('frames', 0),
                pxp_ms=round(pst.get('pxp_ms_sum', 0) / f, 1), grab_ms=round(pst.get('cam_grab_ms_sum', 0) / f, 1),
                sem_ms=round(pst.get('sem_wait_ms_sum', 0) / f, 1), total_ms=round(pst.get('total_ms_sum', 0) / f, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--phases', default='vga_legacy,vga_b,sxga_legacy,sxga_b')
    ap.add_argument('--tag', default=time.strftime('%H%M%S'))
    a = ap.parse_args(); rows = []
    with open(os.path.join(HERE, 'prep_only_%s.log' % a.tag), 'w') as log:
        for name in [p.strip() for p in a.phases.split(',') if p.strip()]:
            w, h, fps, packed = PHASES[name]
            print('\n=== %s: %dx%d @%d, %s, NO TPU invoke' % (name, w, h, fps, 'option B' if packed else 'legacy'))
            subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
            bd = Board(log)
            try:
                bd.q('sentai.camera.yuv_mode(1)', 5)
                if (w, h) != (640, 480): bd.q('print("HW", sentai.camera.set_hw(%d, %d))' % (w, h), 10)
                if packed: bd.q('print("PK", sentai.camera.packed420(1))', 5)
                out = bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 90); print('  init', bd.last(out, 'INIT'))
                out = bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(w, h)], 90)
                if 'LOAD 0' not in out: print('  LOAD FAILED'); continue
                print('  no_invoke:', bd.last(bd.q('print("NI", sentai.pipeline.debug_no_invoke(1))', 5), 'NI'))
                b16 = os.environ.get('B16', '0'); print('  pxp_block16:', bd.last(bd.q('print("B16", sentai.camera.pxp_block16(%s))' % b16, 5), 'B16'))
                wu = prep_run(bd, 8); print('  warm-up: prep fps %.1f (discarded)' % wu['prep_fps'])
                p = prep_run(bd, a.seconds)
                print('  prepared frames/s %.1f | pxp %.1f ms  grab %.1f ms  sem %.1f ms  prep total %.1f ms (%d frames)' % (
                    p['prep_fps'], p['pxp_ms'], p['grab_ms'], p['sem_ms'], p['total_ms'], p['frames']))
                rows.append((name, p))
            finally:
                bd.close()
    print('\n=== summary: camera + PXP only (no TPU)')
    for name, p in rows:
        print('%-12s prepared %5.1f fps  pxp %5.1f ms  grab %5.1f ms  total %5.1f ms' % (name, p['prep_fps'], p['pxp_ms'], p['grab_ms'], p['total_ms']))


if __name__ == '__main__':
    main()

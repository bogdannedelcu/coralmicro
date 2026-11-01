#!/usr/bin/env python3
"""EXP-s261 / OP-S10-W24-T15 step 1 -- PXP repack that chases the CSI writer.

Question (operator): must the PXP wait for the whole frame, or can it repack slice
by slice while the camera is still writing (start at ~25 %)?  `sentai.camera.
chase_test(n)` (firmware) syncs to a frame boundary, finds the buffer the CSI is
filling (padding-byte markers), waits for each of n slices to be written, runs the
PXP on that slice only, then compares the result with a full-frame PXP of the same
buffer.  Reported per run: detect time, frame time, latency from end-of-frame to
repack-done, per-slice wait/PXP times, output equality.

Phases: VGA45 and SXGA30, legacy XYUV->YUV420 and option-B packed byte-drop,
n_slices in {1 (baseline = full frame after EOF), 2, 4, 8}, N runs each.

Usage: python3 run_chase.py [--runs 8] [--phases vga_legacy,vga_b,sxga_legacy,sxga_b] [--tag x]
"""
import argparse, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
from run_csi16 import Board, RESET  # noqa: E402

PHASES = {
    'vga_legacy': (640, 480, 45, False),
    'vga_b': (640, 480, 45, True),
    'sxga_legacy': (1280, 960, 30, False),
    'sxga_b': (1280, 960, 30, True),
}


def parse(line):
    import ast
    try:
        return list(ast.literal_eval(line.split(' ', 1)[1].strip()))
    except Exception:
        return None


def phase(name, runs, log):
    w, h, fps, packed = PHASES[name]
    print('\n=== %s: %dx%d @%d, %s' % (name, w, h, fps, 'option B packed' if packed else 'legacy XYUV'))
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    bd = Board(log); rows = []
    try:
        bd.q('sentai.camera.yuv_mode(1)', 5)
        if (w, h) != (640, 480):
            bd.q('print("HW", sentai.camera.set_hw(%d, %d))' % (w, h), 10)
        if packed:
            bd.q('print("PK", sentai.camera.packed420(1))', 5)
        out = bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 90)
        print('  init', bd.last(out, 'INIT'))
        if 'INIT 0' not in out: return rows
        time.sleep(12.0)   # let the camera autotune/genlock finish (s260: it steals frames early on)
        cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
        bd.q('sentai.camera.test_pattern(%s, 1)' % cam, 5); time.sleep(0.5)   # static content: frames compare equal
        for n in (1, 2, 4, 8):
            res = []
            for i in range(runs):
                out = bd.q('print("CT", sentai.camera.chase_test(%d))' % n, 20)
                l = bd.last(out, 'CT')
                v = parse(l) if l else None
                if v and v[0] == 0 and v[1] == 2:
                    print('  n=%d run %d: slice %d never completed; detect %.1f ms, FRMCNT %d -> %d, still poisoned %d, chased fb %d, DMASA %08x %08x' % (n, i, v[3], v[2] / 1e3, v[4], v[5], v[6], v[7], v[8], v[9]))
                    continue
                if v and v[0] == 0 and v[1] == 3:
                    print('  n=%d run %d: neither candidate completed slice 0 (cand %08x %08x, FRMCNT %d -> %d)' % (n, i, v[2], v[3], v[5], v[4]))
                    continue
                if v and v[0] == 0 and v[1] == 1:
                    print('  n=%d run %d: DMASA_FB1 %08x != FB2 %08x or not a framebuffer' % (n, i, v[2], v[3]))
                    continue
                if v and v[0] == 0:
                    print('  n=%d run %d: no in-flight buffer detected; DMASA_FB1 %08x FB2 %08x fb %s still_poisoned %d' % (n, i, v[1], v[2], ['%08x' % x for x in v[3:7]], v[7]))
                    continue
                if not v:
                    err = [x for x in out.split('\n') if 'RuntimeError' in x]
                    print('  n=%d run %d: FAIL %s' % (n, i, err[-1][:80] if err else out[-120:].strip()))
                    continue
                res.append(v)
            if not res:
                continue
            det = [v[2] for v in res]; eof = [v[3] for v in res]; lat = [v[4] for v in res]
            eq = sum(v[5] for v in res); same = sum(v[6] for v in res)
            pxp = [sum(v[9 + 2 * k] for k in range(n)) for v in res]
            last_pxp = [v[9 + 2 * (n - 1)] for v in res]
            ready = res[0][8::2][:n]
            print('  n=%d: runs %d | detect %4.1f ms | frame %5.1f ms | latency after EOF %5.1f ms (min %5.1f max %5.1f) | PXP total %5.1f ms, last slice %4.1f | equal %d/%d same_buf %d/%d | slice ready (ms) %s' % (
                n, len(res), sum(det) / len(det) / 1e3, sum(eof) / len(eof) / 1e3, sum(lat) / len(lat) / 1e3,
                min(lat) / 1e3, max(lat) / 1e3, sum(pxp) / len(pxp) / 1e3, sum(last_pxp) / len(last_pxp) / 1e3,
                eq, len(res), same, len(res), [round(x / 1e3, 1) for x in ready]))
            rows.append((name, n, len(res), sum(lat) / len(lat) / 1e3, sum(pxp) / len(pxp) / 1e3, eq, same))
    finally:
        bd.close()
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=8)
    ap.add_argument('--phases', default='vga_legacy,vga_b,sxga_legacy,sxga_b')
    ap.add_argument('--tag', default=time.strftime('%H%M%S'))
    a = ap.parse_args()
    allrows = []
    with open(os.path.join(HERE, 'chase_%s.log' % a.tag), 'w') as log:
        for ph in [p.strip() for p in a.phases.split(',') if p.strip()]:
            allrows += phase(ph, a.runs, log)
    print('\n=== summary: latency end-of-frame -> repack done (ms), PXP total (ms), output equal')
    for r in allrows:
        print('%-12s n=%d runs=%d latency %6.1f  pxp %6.1f  equal %d/%d same %d/%d' % (r[0], r[1], r[2], r[3], r[4], r[5], r[2], r[6], r[2]))
    with open(os.path.join(HERE, 'results_%s.txt' % a.tag), 'w') as f:
        for r in allrows: f.write(repr(r) + '\n')


if __name__ == '__main__':
    main()

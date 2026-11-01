#!/usr/bin/env python3
"""EXP-s263 / OP-S10-W24-T15 step 2 -- PrepTask repacks the frame the CSI is still writing.

SXGA option B + OCRAM ring (s262), with and without pipeline.chase_prep(1).  Reports pipeline fps,
invoke, failures, chase counters, and the latency from the camera frame to the TPU input being on the
wire, from the pipeline trace ring (t_us, kind, slot, seq):
  kind 7  = chase sync (vertical blank before the frame starts)      [chase mode]
  kind 8  = slice ready (slot field = slice index)                    [chase mode]
  kind 3  = frame grabbed (complete frame returned by the camera)     [normal mode]
  kind 13 = TPU input done (all input bytes transferred)
  kind 14 = invoke returned
Latency normal = t13 - t3 (+ the frame write time already elapsed before the grab);
latency chase  = t13 - t7 (frame start) and t13 - t8[last] (frame end).

Usage: python3 run_chase_prep.py [--seconds 15] [--fps 22|30]  (keys = measured fps since 2026-11-01: 22 = the old "30" row at 22.6 fps, 30 = true 30; the logs here were taken with the old keys 30/36) [--tag x]
"""
import argparse, ast, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
sys.path.insert(0, os.path.join(HERE, '..', 's260_packed420_pipeline_option_b'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS, pipeline  # noqa: E402


def trace(bd):
    out = bd.q('print("TR", sentai.pipeline.trace())', 10)
    m = re.search(r'TR (\(.*\)|\[.*\])', out, re.S)
    return list(ast.literal_eval(m.group(1))) if m else []


def latencies(ev, chase):
    """Per TPU-input-done event (kind 13, seq s): pair with the prep events of the same frame.
    PrepTask kinds 4/5/7/8 share the InferTask seq counter; kind 3 (grab) carries the camera
    frame_seq, so it is paired by time (latest grab before the frame's PXP start, kind 4)."""
    k13 = {seq: t for t, k, slot, seq in ev if k == 13}
    k4 = {}
    for t, k, slot, seq in ev:
        if k == 4 and seq not in k4: k4[seq] = t
    k7 = {seq: t for t, k, slot, seq in ev if k == 7}
    k8 = {}
    for t, k, slot, seq in ev:
        if k == 8: k8.setdefault(seq, []).append((slot, t))
    grabs = sorted(t for t, k, slot, seq in ev if k == 3)
    rows = []
    for seq, t13 in sorted(k13.items()):
        if chase:
            if seq not in k7 or seq not in k8: continue
            sl = sorted(k8[seq]); t_first = sl[0][1]; t_last = sl[-1][1]
            rows.append((seq, (t13 - t_first) / 1e3, (t13 - t_last) / 1e3, (t_last - t_first) / 1e3))
        else:
            if seq not in k4: continue
            g = [t for t in grabs if t <= k4[seq]]
            if not g: continue
            rows.append((seq, (t13 - g[-1]) / 1e3, None, None))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--tag', default=time.strftime('%H%M%S'))
    a = ap.parse_args()
    with open(os.path.join(HERE, 'run_%s.log' % a.tag), 'w') as log:
        for chase in (0, 1):
            print('\n=== SXGA fps-key %d, option B + OCRAM ring, chase_prep=%d' % (a.fps, chase))
            subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
            bd = Board(log)
            try:
                bd.q('sentai.camera.yuv_mode(1)', 5); bd.q('sentai.camera.set_hw(1280, 960)', 10); bd.q('sentai.camera.packed420(1)', 5)
                print('  init', bd.last(bd.q('print("INIT", sentai.camera.init(1, %d))' % a.fps, 90), 'INIT'))
                out = bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(1280, 960)], 90)
                if 'LOAD 0' not in out: print('  LOAD FAILED'); continue
                bd.q('sentai.tpu.invoke()', 30)
                bd.q('print("RG", sentai.pipeline.ocram_ring(1))', 10)
                bd.q('print("CP", sentai.pipeline.chase_prep(%d))' % chase, 5)
                wu = pipeline(bd, 8); print('  warm-up fps %.1f (discarded)' % wu['fps'])
                bd.q('sentai.pipeline.trace(1)', 5); bd.q('sentai.pipeline.ring_stats(1)', 5)
                p = pipeline(bd, a.seconds)
                ev = trace(bd)
                print('  pipeline: fps %.1f invoke %.1f ms cycle %.1f pxp(+waits) %.1f fail %d' % (p['fps'], p['invoke_ms'], p['cycle_ms'], p['pxp_ms'], p['fail']))
                print('  ring_stats:', bd.last(bd.q('print("RS", sentai.pipeline.ring_stats())', 5), 'RS')[3:], ' chase_stats (ok, begin_fail, slice_fail):', bd.last(bd.q('print("CS", sentai.pipeline.chase_stats())', 5), 'CS')[3:])
                rows = latencies(ev, chase)
                if rows:
                    l1 = [r[1] for r in rows]
                    if chase:
                        l2 = [r[2] for r in rows]; fw = [r[3] for r in rows]
                        print('  latency slice-0 written -> TPU input done: mean %.1f ms (min %.1f max %.1f) | last line written -> input done: mean %.1f ms | first..last slice %.1f ms  (n=%d)' % (
                            sum(l1) / len(l1), min(l1), max(l1), sum(l2) / len(l2), sum(fw) / len(fw), len(rows)))
                    else:
                        print('  latency complete frame grabbed (~last line) -> TPU input done: mean %.1f ms (min %.1f max %.1f)  (n=%d)' % (sum(l1) / len(l1), min(l1), max(l1), len(rows)))
                else:
                    print('  no matching trace events; kinds present:', sorted(set(e[1] for e in ev))); print('  last events:', ev[-16:])
                print('  health:', bd.q('print(sentai.sys.health())', 8).strip().split('\n')[-2][:100])
            finally:
                bd.close()


if __name__ == '__main__':
    main()

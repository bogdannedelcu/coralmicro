#!/usr/bin/env python3
"""EXP-s264 / OP-S10-W24-T16 -- does the SXGA pipeline process EVERY camera frame when the
sensor runs at the pipeline's own rate (~15 fps)?

Operator question (2026-11-01): the option B + OCRAM ring pipeline reaches 15.6 fps with the
camera at a true 30 fps (every other frame dropped).  If the camera itself is set to SXGA
15 fps, does the TPU see every frame (lockstep, no slip -> no desync on camera switch)?

Per fps key: fresh boot, one camera, packed YUV420 + ring, SXGA model, warm-up run, then a
measured run during which the CSI hardware frame counter (CR3.FRMCNT, +1 per frame the CSI
completes) is sampled at start and end.  processed / csi_frames is the lockstep ratio.

Usage: python3 run_lockstep.py [--seconds 20] [--keys 15,36] [--tag x]
"""
import argparse, os, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
sys.path.insert(0, os.path.join(HERE, '..', 's260_packed420_pipeline_option_b'))
from run_csi16 import Board, RESET  # noqa: E402
from run_option_b import MODELS, kv, pipeline, urb  # noqa: E402
CR3 = 0x40800008


def frm(bd):
    return (int(bd.last(bd.q('print("V", sentai.sys.peek32(0x%X))' % CR3, 5), 'V').split()[1]) >> 16) & 0xFFFF


def run_key(key, seconds, log, chase=False):
    print('\n=== SXGA fps key %d, option B + OCRAM ring%s' % (key, ', chase_prep' if chase else ''))
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    bd = Board(log)
    res = {'key': key, 'chase': chase}
    try:
        print(' ', bd.q('print(sentai.version())', 8).strip().split('\n')[-2][:60])
        bd.q('sentai.camera.yuv_mode(1)', 5)
        bd.q('print("HW", sentai.camera.set_hw(1280, 960))', 10)
        bd.q('print("PK", sentai.camera.packed420(1))', 5)
        out = bd.q('print("INIT", sentai.camera.init(1, %d))' % key, 90)
        print('  init', bd.last(out, 'INIT'))
        if 'INIT 0' not in out:
            res['error'] = 'init'; return res
        time.sleep(1.0)
        cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
        v = int(bd.last(bd.q('print("NM", sentai.camera.reg_read(%s, 0x3A00))' % cam, 5), 'NM').split()[-1])
        bd.q('sentai.camera.reg_write(%s, 0x3A00, %d)' % (cam, v & ~0x04), 5)   # AEC night mode off
        for reg in (0x3035, 0x3036, 0x380E, 0x380F):
            print('  reg %04X = %s' % (reg, bd.last(bd.q('print("R", hex(sentai.camera.reg_read(%s, 0x%04X)))' % (cam, reg), 5), 'R').split()[-1]))
        time.sleep(2.0)
        f0 = frm(bd); t0 = time.time(); time.sleep(4.0); f1 = frm(bd); dt = time.time() - t0
        res['csi_idle_rate'] = ((f1 - f0) & 0xFFFF) / dt
        print('  CSI FRMCNT rate idle: %.2f frames/s' % res['csi_idle_rate'])
        out = bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(1280, 960)], 90)
        if 'LOAD 0' not in out:
            print('  LOAD FAILED', out[-200:]); res['error'] = 'load'; return res
        bd.q('sentai.tpu.invoke()', 30)
        print('  ocram_ring:', bd.last(bd.q('print("RG", sentai.pipeline.ocram_ring(1))', 10), 'RG'))
        if chase:
            print('  chase_prep:', bd.last(bd.q('print("CP", sentai.pipeline.chase_prep(1))', 10), 'CP'))
        wu = pipeline(bd, 8); print('  warm-up run: fps %.1f (discarded)' % wu['fps'])
        # Steady-state window: the pipeline keeps running while both counters (CSI FRMCNT and
        # the InferTask ok count) are read in ONE REPL statement at t0 and again at t1, so the
        # start/stop transients and the REPL round-trips stay outside the window (the earlier
        # start..stop window under-counted by ~8 % for that reason).
        bd.q('sentai.pipeline.infer_reset()', 5); bd.q('sentai.pipeline.prep_reset()', 5)
        bd.q('sentai.pipeline.ring_stats(1)', 5)
        bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(3.0)
        def snap():
            s = bd.last(bd.q('print("SN", sentai.sys.peek32(0x%X) >> 16, sentai.pipeline.infer_stats()["ok"], sentai.camera.stats()["stale_skips"])' % CR3, 8), 'SN').split()
            return int(s[1]) & 0xFFFF, int(s[2]), int(s[3])
        c0, k0, s0 = snap(); t0 = time.time(); time.sleep(seconds); c1, k1, s1 = snap(); dt = time.time() - t0
        ist = kv(bd.q('print("ISTATS", sentai.pipeline.infer_stats())', 10))
        pst = kv(bd.q('print("PSTATS", sentai.pipeline.prep_stats())', 10))
        bd.q('print("STOP", sentai.pipeline.stop())', 30)
        csi = (c1 - c0) & 0xFFFF; done = k1 - k0
        frames = ist.get('ok', 0) or 1; pf = pst.get('frames', 0) or 1
        res.update(csi_frames=csi, csi_rate=csi / dt, frames=done, fps=done / dt, wall=dt,
                   ratio=done / csi if csi else 0, stale_skips_win=s1 - s0, fail=ist.get('fail', 0),
                   invoke_ms=ist.get('ms_sum', 0) / frames, cycle_ms=ist.get('cycle_ms_sum', 0) / frames,
                   pxp_ms=pst.get('pxp_ms_sum', 0) / pf, grab_ms=pst.get('cam_grab_ms_sum', 0) / pf, sem_ms=pst.get('sem_wait_ms_sum', 0) / pf)
        res['urb'] = urb(bd)
        print('  steady state %.1f s: CSI frames %d (%.2f/s) | processed %d (%.2f/s) | processed/captured = %.3f | stale skips in window %d' % (
            dt, csi, res['csi_rate'], done, res['fps'], res['ratio'], res['stale_skips_win']))
        print('  invoke %.1f ms  cycle %.1f ms | prep: pxp %.1f grab %.1f sem %.1f | fail %d | urb %s' % (
            res['invoke_ms'], res['cycle_ms'], res['pxp_ms'], res['grab_ms'], res['sem_ms'], res['fail'], res['urb']))
        st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST')
        d = kv(st); print('  cam stats: ' + ' '.join('%s=%s' % (k, d[k]) for k in ('stale_skips', 'grab_retry', 'grab_fatal', 'producer_overruns', 'frames_total', 'period_us', 'vblank_us', 'csi_queue_faults') if k in d))
        print('  ring_stats:', bd.last(bd.q('print("RS", sentai.pipeline.ring_stats())', 5), 'RS'))
        print('  health:', bd.q('print(sentai.sys.health())', 8).strip().split('\n')[-2][:120])
    finally:
        bd.close()
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--keys', default='15,36')
    ap.add_argument('--chase', action='store_true')
    ap.add_argument('--tag', default=time.strftime('%H%M%S'))
    a = ap.parse_args()
    rows = []
    with open(os.path.join(HERE, 'run_%s.log' % a.tag), 'w') as log:
        for k in [int(x) for x in a.keys.split(',') if x.strip()]:
            rows.append(run_key(k, a.seconds, log, chase=a.chase))
    print('\n=== summary (%s)' % a.tag)
    print('%-4s %8s %8s %7s %7s %6s %7s %6s %5s' % ('key', 'csi/s', 'fps', 'invoke', 'cycle', 'pxp', 'ratio', 'stale', 'fail'))
    for r in rows:
        if 'error' in r:
            print('%-4d ERROR %s' % (r['key'], r['error'])); continue
        print('%-4d %8.2f %8.2f %7.1f %7.1f %6.1f %7.3f %6d %5d' % (
            r['key'], r['csi_rate'], r['fps'], r['invoke_ms'], r['cycle_ms'], r['pxp_ms'], r['ratio'], r['stale_skips_win'], r['fail']))
    with open(os.path.join(HERE, 'results_%s.txt' % a.tag), 'w') as f:
        for r in rows: f.write(repr(r) + '\n')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""EXP-s260 / OP-S10-W24-T16 -- option B speed test: packed YUV420 capture + PXP byte-drop.

Compares, on the same build and the same static scene, the detection pipeline in
  legacy mode : camera XYUV8888 (4 B/px) -> PXP XYUV->YUV420 repack -> TPU (Y + UV)
  option B    : camera packed YUV420 (2 B/px, EXP-s259) -> PXP RGB888->RGB888P byte-drop
                (1.5 B/px [C,Y1,Y2]) -> TPU (same byte count; the existing 2-input
                YUV420 models read the same bytes, detections are meaningless)
at VGA 45 fps and SXGA 30 fps.  Reports pipeline fps, invoke ms, PXP ms per frame,
camera-grab ms, failures and URB counters.  Each phase is a fresh boot.  Also dumps
the staging buffer after a colour-bar run in option B to verify the byte-drop layout.

Usage: python3 run_option_b.py [--seconds 15] [--tag x] [--phases vga_legacy,vga_b,sxga_legacy,sxga_b]
"""
import argparse, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
sys.path.insert(0, os.path.join(HERE, '..', 's259_csi_mipi_16bit_yuv422'))
import board_serial as B  # noqa: E402
from run_csi16 import Board, RESET  # noqa: E402

MODELS = {
    (640, 480): '/models/c2f_yuv420_int8_edgetpu.tflite',
    (1280, 960): '/models/c2f_yuv420_sxga_relu_int8_edgetpu.tflite',
}
# SXGA fps keys were re-keyed to the MEASURED frame rate on 2026-11-01 (EXP-s264, SDK patch 0007):
# the old key 30 (22.6 fps) is now 22, the old true-30 key 36 is now 30, the old key 15 (11.2 fps)
# is gone and key 15 is a true 15 fps.  The phase names below keep their historical meaning
# ("sxga36" = true 30) so the logs in this folder stay readable; only the init keys changed.
PHASES = {
    'vga_legacy':  (640, 480, 45, False),
    'vga_b':       (640, 480, 45, True),
    'sxga_legacy': (1280, 960, 22, False),
    'sxga_b':      (1280, 960, 22, True),
    'sxga15_legacy': (1280, 960, 15, False),   # 15.0 fps now (was 11.2 before the re-keying)
    'sxga36_legacy': (1280, 960, 30, False),   # init returns -12: legacy 4 B/px at a true 30 fps hangs the board
    'sxga36_b':      (1280, 960, 30, True),
    'sxga_b_ring':   (1280, 960, 22, 'ring'),
    'sxga36_b_ring': (1280, 960, 30, 'ring'),
    'sxga15_b':      (1280, 960, 15, True),
}


def kv(s, pre=''):
    d = {}
    m = re.search(r"\{.*\}", s, re.S)
    if m:
        for k, v in re.findall(r"'(\w+)': (-?\d+)", m.group(0)):
            d[pre + k] = int(v)
    return d


def pipeline(bd, seconds):
    bd.q('sentai.pipeline.infer_reset()', 5)
    bd.q('sentai.pipeline.prep_reset()', 5)
    bd.q('print("START", sentai.pipeline.start(0.5))', 30)
    t0 = time.time(); time.sleep(seconds)
    ist = kv(bd.q('print("ISTATS", sentai.pipeline.infer_stats())', 10))
    pst = kv(bd.q('print("PSTATS", sentai.pipeline.prep_stats())', 10))
    wall = time.time() - t0
    bd.q('print("STOP", sentai.pipeline.stop())', 30)
    frames = ist.get('ok', 0); fail = ist.get('fail', 0)
    pf = pst.get('frames', 0) or 1
    return dict(fps=round(frames / wall, 1), frames=frames, fail=fail,
                invoke_ms=round(ist.get('ms_sum', 0) / frames, 1) if frames else 0,
                cycle_ms=round(ist.get('cycle_ms_sum', 0) / frames, 1) if frames else 0,
                pxp_ms=round(pst.get('pxp_ms_sum', 0) / pf, 1),
                grab_ms=round(pst.get('cam_grab_ms_sum', 0) / pf, 1),
                sem_ms=round(pst.get('sem_wait_ms_sum', 0) / pf, 1),
                prep_total_ms=round(pst.get('total_ms_sum', 0) / pf, 1),
                prep_frames=pst.get('frames', 0), first_fail=ist.get('first_fail_idx', -1))


def urb(bd):
    d = kv(bd.q('print(sentai.tpu.urb_sync_stats())', 8))
    return {k: d.get(k, 0) for k in ('take_failed', 'csr_timeouts', 'desync_count', 'usb_detach')}


def ehci(bd):
    d = kv(bd.q('print(sentai.tpu.urb_sync_stats())', 8))
    return d.get('ehci', None)


def phase(name, seconds, log, bars_dump=False):
    w, h, fps, packed = PHASES[name]
    print('\n=== %s: %dx%d @%d fps, %s' % (name, w, h, fps, 'option B (packed YUV420 + byte-drop)' if packed else 'legacy XYUV + repack'))
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    bd = Board(log)
    res = {'phase': name}
    try:
        print(' ', bd.q('print(sentai.version())', 8).strip().split('\n')[-2][:60])
        bd.q('sentai.camera.yuv_mode(1)', 5)
        if (w, h) != (640, 480):
            print('  set_hw', bd.last(bd.q('print("HW", sentai.camera.set_hw(%d, %d))' % (w, h), 10), 'HW'))
        if packed:
            print('  packed420', bd.last(bd.q('print("PK", sentai.camera.packed420(1))', 5), 'PK'))
        out = bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 90)
        print('  init', bd.last(out, 'INIT'), [l.strip()[:90] for l in out.split('\n') if 'packed420' in l])
        if 'INIT 0' not in out:
            res['error'] = 'init'; return res
        time.sleep(1.0)
        # The room may be dark: the OV5640 AEC 'night mode' (0x3A00 bit 2) then stretches
        # the frame period, which would confound the comparison.  Clear it so the sensor
        # keeps its nominal frame rate (darker image, same timing in every phase).
        cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
        v = int(bd.last(bd.q('print("NM", sentai.camera.reg_read(%s, 0x3A00))' % cam, 5), 'NM').split()[-1])
        bd.q('sentai.camera.reg_write(%s, 0x3A00, %d)' % (cam, v & ~0x04), 5)
        print('  0x3A00 %#04x -> %#04x (night mode off)' % (v, v & ~0x04))
        time.sleep(0.5)
        f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF'); time.sleep(2.0)
        f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
        res['cam_rate'] = (int(f1.split()[1]) - int(f0.split()[1])) / 2.0
        print('  sensor frame counter ~ %.1f/s' % res['cam_rate'])
        out = bd.q('print("LOAD", sentai.tpu.load("%s"))' % MODELS[(w, h)], 90)
        if 'LOAD 0' not in out:
            print('  LOAD FAILED', out[-200:]); res['error'] = 'load'; return res
        bd.q('sentai.tpu.invoke()', 30)   # parameter upload
        if packed == 'ring':
            print('  ocram_ring:', bd.last(bd.q('print("RG", sentai.pipeline.ocram_ring(1))', 10), 'RG'))
            bd.q('sentai.pipeline.ring_stats(1)', 5)
        # warm-up: the camera autotune/genlock started by init() runs for several
        # seconds and steals frames (probe_fps.py: first run 24 fps, second 41 fps)
        wu = pipeline(bd, 8); print('  warm-up run: fps %.1f grab %.1f ms (discarded)' % (wu['fps'], wu['grab_ms']))
        p = pipeline(bd, seconds)
        res.update(p); res['urb'] = urb(bd)
        print('  pipeline: fps %.1f  invoke %.1f ms  cycle %.1f ms | prep: pxp %.1f ms grab %.1f sem %.1f total %.1f (%d frames) | fail %d first_fail %d | urb %s' % (
            p['fps'], p['invoke_ms'], p['cycle_ms'], p['pxp_ms'], p['grab_ms'], p['sem_ms'], p['prep_total_ms'], p['prep_frames'], p['fail'], p['first_fail'], res['urb']))
        print('  health:', bd.q('print(sentai.sys.health())', 8).strip().split('\n')[-2][:120])
        if packed == 'ring':
            print('  ring_stats (frames, urbs, cons_wait_us, cons_waits, cons_timeouts, prod_timeouts):', bd.last(bd.q('print("RS", sentai.pipeline.ring_stats())', 5), 'RS'))
        st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST'); print('  cam:', st[:200])
        if bars_dump and packed:
            cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
            bd.q('sentai.camera.test_pattern(%s, 1)' % cam, 5); time.sleep(0.6)
            bd.q('print("START", sentai.pipeline.start(0.5))', 30); time.sleep(2.0)
            bd.q('print("STOP", sentai.pipeline.stop())', 30)
            n = w * h * 3 // 2
            out = bd.q('print("DS", sentai.pipeline.dump_staging("/s260_stage.bin", 0, %d))' % n, 60)
            print('  dump_staging:', bd.last(out, 'DS'))
            bd.q('sentai.camera.test_pattern(%s, 0)' % cam, 5)
            bd.q('print("IP", sentai.usb.ip(1))', 10); time.sleep(4)
            pr = subprocess.run(['curl', '-s', '-m', '60', 'http://10.0.0.1/api/raw/s260_stage.bin'], capture_output=True)
            if pr.returncode == 0 and len(pr.stdout) == n:
                open(os.path.join(HERE, 'stage_%s.bin' % name), 'wb').write(pr.stdout)
                check_stage(pr.stdout, w, h, name)
            else:
                print('  staging fetch failed rc %d len %d' % (pr.returncode, len(pr.stdout)))
            bd.q('sentai.fs.remove("/s260_stage.bin")', 10)
    finally:
        bd.close()
    return res


def check_stage(buf, w, h, name):
    """Staging buffer in option B = [C,Y1,Y2] per pixel pair, rows of w/2 groups."""
    import numpy as np
    from PIL import Image
    g = np.frombuffer(buf, dtype=np.uint8).reshape(h, w // 2, 3)
    for r in (0, 1):
        row = g[r]
        means = [[int(x) for x in np.array_split(row[:, k].astype(np.float32), 8).__iter__().__next__()[:1]] for k in range(3)]
        bars = [[int(np.mean(seg)) for seg in np.array_split(row[:, k].astype(np.float32), 8)] for k in range(3)]
        print('  stage row %d: C %s | Y1 %s | Y2 %s' % (r, bars[0], bars[1], bars[2]))
    Y = np.empty((h, w), dtype=np.uint8); Y[:, 0::2] = g[:, :, 1]; Y[:, 1::2] = g[:, :, 2]
    U = np.repeat(np.repeat(g[0::2, :, 0].astype(np.float32), 2, 0), 2, 1)[:h, :w]
    V = np.repeat(np.repeat(g[1::2, :, 0].astype(np.float32), 2, 0), 2, 1)[:h, :w]
    c = Y.astype(np.float32) - 16; d = U - 128; e = V - 128
    rgb = np.clip(np.dstack([1.164 * c + 1.596 * e, 1.164 * c - 0.392 * d - 0.813 * e, 1.164 * c + 2.017 * d]), 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(os.path.join(HERE, 'stage_%s.png' % name))
    print('  staging image written: stage_%s.png' % name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=15.0)
    ap.add_argument('--tag', default=time.strftime('%H%M%S'))
    ap.add_argument('--phases', default='vga_legacy,vga_b,sxga_legacy,sxga_b,sxga15_legacy,sxga15_b')
    ap.add_argument('--bars-dump', action='store_true', help='after each option-B phase, dump the staging buffer with colour bars on')
    a = ap.parse_args()
    rows = []
    with open(os.path.join(HERE, 'run_%s.log' % a.tag), 'w') as log:
        for ph in [p.strip() for p in a.phases.split(',') if p.strip()]:
            rows.append(phase(ph, a.seconds, log, bars_dump=a.bars_dump))
    print('\n=== summary (%s)' % a.tag)
    print('%-14s %5s %6s %7s %7s %6s %6s %5s' % ('phase', 'cam/s', 'fps', 'invoke', 'cycle', 'pxp', 'grab', 'fail'))
    for r in rows:
        if 'error' in r:
            print('%-14s ERROR %s' % (r['phase'], r['error'])); continue
        print('%-14s %5.1f %6.1f %7.1f %7.1f %6.1f %6.1f %5d' % (
            r['phase'], r.get('cam_rate', 0), r['fps'], r['invoke_ms'], r['cycle_ms'], r['pxp_ms'], r['grab_ms'], r['fail']))
    with open(os.path.join(HERE, 'results_%s.txt' % a.tag), 'w') as f:
        for r in rows: f.write(repr(r) + '\n')


if __name__ == '__main__':
    main()

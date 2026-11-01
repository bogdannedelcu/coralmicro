#!/usr/bin/env python3
"""EXP-s259 / OP-S10-W24-T14 -- visual check of the 2 B/px packed YUV420 capture.

Sensor 0x4300=0x5F ("YUV420 for MIPI only", UYY/VYY) + 24-bit CSI receiver at half
width stores, per pair of pixels, [C, Y1, Y2, 0] with C = U on even rows and V on odd
rows (fmt/y8 sweeps).  This script dumps whole frames with sentai.camera.save_raw(),
pulls them over HTTP (/api/raw), rebuilds Y / U / V planes and writes PNGs next to a
legacy XYUV capture of the same scene, for VGA and SXGA.  It also reports the sensor
frame rate and the camera fault counters in packed mode at SXGA 30 fps.

Usage: python3 grab_packed.py [--tag x] [--skip-sxga]
"""
import argparse, os, subprocess, sys, time
import numpy as np
from PIL import Image
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from run_csi16 import Board, RESET  # noqa: E402

CHUNK = 900 * 1024   # /api/raw POST buffer is 1 MB; keep file slices under it


def fetch(bd, name, nbytes):
    data = b''
    off = 0
    i = 0
    while off < nbytes:
        n = min(CHUNK, nbytes - off)
        fn = '/%s_%d.bin' % (name, i)
        out = bd.q('print("SV", sentai.camera.save_raw("%s", %d, %d))' % (fn, off, n), 60)
        sv = bd.last(out, 'SV')
        if not sv or int(sv.split()[1]) != n:
            raise RuntimeError('save_raw: %r' % sv)
        p = subprocess.run(['curl', '-s', '-m', '60', 'http://10.0.0.1/api/raw%s' % fn], capture_output=True)
        if p.returncode != 0 or len(p.stdout) != n:
            raise RuntimeError('curl %s: rc %d len %d' % (fn, p.returncode, len(p.stdout)))
        data += p.stdout
        bd.q('sentai.fs.remove("%s")' % fn, 10)
        off += n; i += 1
    return data


def packed420_to_rgb(buf, w, h):
    g = np.frombuffer(buf, dtype=np.uint8).reshape(h, w // 2, 4)
    Y = np.empty((h, w), dtype=np.uint8)
    Y[:, 0::2] = g[:, :, 1]
    Y[:, 1::2] = g[:, :, 2]
    U = g[0::2, :, 0].astype(np.float32)     # even rows carry U (0x5F: UYY / VYY)
    V = g[1::2, :, 0].astype(np.float32)     # odd rows carry V
    Uf = np.repeat(np.repeat(U, 2, axis=0), 2, axis=1)[:h, :w]
    Vf = np.repeat(np.repeat(V, 2, axis=0), 2, axis=1)[:h, :w]
    return yuv_to_rgb(Y.astype(np.float32), Uf, Vf), Y, U.astype(np.uint8), V.astype(np.uint8)


def xyuv_to_rgb(buf, w, h):
    g = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4).astype(np.float32)
    V, U, Y = g[:, :, 0], g[:, :, 1], g[:, :, 2]   # memory order [V,U,Y,X] (s259 lanes)
    return yuv_to_rgb(Y, U, V), Y.astype(np.uint8)


def yuv_to_rgb(Y, U, V):
    c = Y - 16.0; d = U - 128.0; e = V - 128.0
    r = 1.164 * c + 1.596 * e
    gch = 1.164 * c - 0.392 * d - 0.813 * e
    b = 1.164 * c + 2.017 * d
    return np.clip(np.dstack([r, gch, b]), 0, 255).astype(np.uint8)


def session(log, w, h, packed, fps=30):
    subprocess.run(['python3', RESET], check=False, capture_output=True); time.sleep(1.0)
    bd = Board(log)
    try:
        bd.q('sentai.camera.yuv_mode(1)', 5)
        if (w, h) != (640, 480):
            print('  set_hw', bd.last(bd.q('print("HW", sentai.camera.set_hw(%d, %d))' % (w, h), 10), 'HW'))
        if packed:
            bd.q('sentai.camera.csi_mode(24, 0, 0, 2)', 5)
        print('  init', bd.last(bd.q('print("INIT", sentai.camera.init(1, %d))' % fps, 90), 'INIT'))
        cam = bd.last(bd.q('print("CI", sentai.camera.current_id())', 5), 'CI').split()[-1]
        if packed:
            bd.q('sentai.camera.reg_write(%s, 0x4300, 0x5F)' % cam, 5)
        time.sleep(1.0)
        f0 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
        time.sleep(3.0)
        f1 = bd.last(bd.q('print("SF", sentai.camera.sensor_frames())', 5), 'SF')
        rate = (int(f1.split()[1]) - int(f0.split()[1])) / 3.0
        print('  sensor frame counter rate ~ %.1f/s' % rate)
        bd.q('print("IP", sentai.usb.ip(1))', 10); time.sleep(4)
        nbytes = w * h * (2 if packed else 4)
        t0 = time.time()
        buf = fetch(bd, 's259_%s_%dx%d' % ('p420' if packed else 'xyuv', w, h), nbytes)
        print('  fetched %d bytes in %.1f s' % (len(buf), time.time() - t0))
        st = bd.last(bd.q('print("ST", sentai.camera.stats())', 10), 'ST')
        print('  ', st[:220])
        return buf, rate, st
    finally:
        bd.close()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--tag', default=''); ap.add_argument('--skip-sxga', action='store_true')
    a = ap.parse_args()
    tag = a.tag or time.strftime('%H%M%S')
    with open(os.path.join(HERE, 'grab_%s.log' % tag), 'w') as log:
        for (w, h) in ([(640, 480)] + ([] if a.skip_sxga else [(1280, 960)])):
            print('\n=== %dx%d legacy XYUV' % (w, h))
            ref, _, _ = session(log, w, h, packed=False)
            rgb_ref, Yref = xyuv_to_rgb(ref, w, h)
            Image.fromarray(rgb_ref).save(os.path.join(HERE, 'xyuv_%dx%d_%s.png' % (w, h, tag)))
            print('\n=== %dx%d packed YUV420 (2 B/px)' % (w, h))
            pk, rate, st = session(log, w, h, packed=True)
            open(os.path.join(HERE, 'p420_%dx%d_%s.bin' % (w, h, tag)), 'wb').write(pk)
            rgb, Y, U, V = packed420_to_rgb(pk, w, h)
            Image.fromarray(rgb).save(os.path.join(HERE, 'p420_%dx%d_%s.png' % (w, h, tag)))
            Image.fromarray(Y).save(os.path.join(HERE, 'p420_Y_%dx%d_%s.png' % (w, h, tag)))
            # luma agreement with the legacy capture of the same static scene
            a_ = Yref.astype(np.float32).ravel(); b_ = Y.astype(np.float32).ravel()
            c = float(np.corrcoef(a_, b_)[0, 1])
            print('  Y(packed) vs Y(legacy): mean %.1f vs %.1f, corr %.3f, mean|diff| %.1f' % (
                b_.mean(), a_.mean(), c, float(np.abs(a_ - b_).mean())))
            print('  U mean %.1f std %.1f, V mean %.1f std %.1f' % (U.mean(), U.std(), V.mean(), V.std()))
    print('PNGs written in', HERE)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""EXP-s268: measure the colour chart in the domain the Edge TPU models receive — the OV5640's
own YUV output (camera.yuv_mode(1), CSI stores XYUV 4 B/px) — instead of the RGB/JPEG path.

Steps (board must not move between the RGB reference run and this one; the patch bboxes are
reused from --bbox-from, a tune_colors measurement directory with locate.json):
  A. RGB mode: reset, init (production register sets apply), tune_colors --stages expo
     (manual AGC so 'neutral 8' lands at the target) -> read the frozen exposure / gain.
  B. YUV mode: reset, yuv_mode(1), init, write the same AEC/AGC freeze, show the same grid,
     save_raw() one frame per camera, download, record 0x4300 / 0x501F / 0x5000 / 0x5001.
  C. Host: identify the byte roles (X, Y, U, V) from the white / black / red / blue patches,
     per-patch Y U V means, range evidence (white Y, black Y, grey U/V), YUV->RGB with BT.601
     full and limited range, dE00 vs the sRGB targets for both, converted frame saved as JPEG so
     compare_patches.py can render reference | cam0 | cam1 from the YUV captures.

Usage: python3 yuv_check.py --out iter1 [--region 243,162,1133,756] [--bbox-from <dir>]
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
S265 = os.path.join(HERE, '..', 's265_camera_color_calibration')
sys.path.insert(0, S265)
from board_repl import Repl  # noqa: E402
from analyze import srgb_to_lab, de2000_lab, de76  # noqa: E402

import glob
import serial


def reset_board():
    r = Repl(); r.ser.write(b'sentai.sys.reset()\r'); time.sleep(0.5); r.close()
    t0 = time.time()
    while time.time() - t0 < 40:
        time.sleep(1)
        try:
            s = serial.Serial(sorted(glob.glob('/dev/ttyACM*'))[0], 115200, timeout=0.5); s.close(); break
        except Exception:
            pass
    time.sleep(3)


def yuv_to_rgb(y, u, v, limited):
    y = y.astype(float); u = u.astype(float) - 128.0; v = v.astype(float) - 128.0
    if limited:
        y = (y - 16.0) * 255.0 / 219.0; u = u * 255.0 / 224.0; v = v * 255.0 / 224.0
    r = y + 1.402 * v
    g = y - 0.344136 * u - 0.714136 * v
    b = y + 1.772 * u
    return np.clip(np.stack([r, g, b], -1), 0, 255)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--region', default='243,162,1133,756')
    ap.add_argument('--layout', default='4x6')
    ap.add_argument('--bbox-from', default=os.path.join(HERE, '..', '..', 'camera_modules', 'out', 'measure_dist45b'))
    ap.add_argument('--skip-expo', action='store_true', help='reuse exposure.json from a previous run')
    a = ap.parse_args()
    a.out = os.path.abspath(a.out)
    os.makedirs(a.out, exist_ok=True)
    shutil.copy(os.path.join(a.bbox_from, 'locate.json'), os.path.join(a.out, 'locate.json'))
    bbox = {c: json.load(open(os.path.join(a.bbox_from, 'measure_patches_cam%d.json' % c))) for c in (0, 1)}
    expo_path = os.path.join(a.out, 'exposure.json')

    # ---- A: exposure normalisation in RGB mode -------------------------------------------
    if not (a.skip_expo and os.path.exists(expo_path)):
        print('== A: RGB mode, exposure normalisation (tune_colors --stages expo)')
        reset_board()
        r = Repl(log=open(os.path.join(a.out, 'repl.log'), 'a'))
        print('  init ->', r.cmd('print(sentai.camera.init(1, 30))', 40).replace('\r', '').strip().split('\n')[-1]); r.close()
        time.sleep(2)
        subprocess.run(['python3', os.path.join(S265, 'tune_colors.py'), '--out', a.out, '--cams', '0,1', '--stages', 'expo',
                        '--region', a.region, '--avg', '2'], cwd=S265, stdout=open(os.path.join(a.out, 'expo.out'), 'w'), stderr=subprocess.STDOUT)
        subprocess.run(['python3', os.path.join(S265, 'show_pattern.py'), '--quit'], cwd=S265, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        r = Repl(log=open(os.path.join(a.out, 'repl.log'), 'a'))
        expo = {}
        for c in (0, 1):
            v = r.regs(c, [0x3503, 0x3500, 0x3501, 0x3502, 0x350A, 0x350B])
            expo[c] = v
            print('  cam%d frozen: 0x3503=%02X exposure %d lines gain %.2fx' % (c, v[0], (v[1] << 16 | v[2] << 8 | v[3]) >> 4, (v[4] << 8 | v[5]) / 16))
        r.close()
        json.dump(expo, open(expo_path, 'w'))
    expo = {int(k): v for k, v in json.load(open(expo_path)).items()}

    # ---- B: YUV mode capture ------------------------------------------------------------
    print('== B: YUV mode capture')
    reset_board()
    r = Repl(log=open(os.path.join(a.out, 'repl.log'), 'a'))
    print('  yuv_mode(1) ->', r.value('sentai.camera.yuv_mode(1)'))
    print('  init ->', r.cmd('print(sentai.camera.init(1, 30))', 40).replace('\r', '').strip().split('\n')[-1])
    time.sleep(2)
    fmt = {}
    for c in (0, 1):
        v = expo[c]
        for addr, val in zip([0x3503, 0x3500, 0x3501, 0x3502, 0x350A, 0x350B], v):
            r.regw(c, addr, val)
        fmt[c] = dict(zip(['0x4300', '0x501F', '0x5000', '0x5001', '0x3406', '0x3503'], r.regs(c, [0x4300, 0x501F, 0x5000, 0x5001, 0x3406, 0x3503])))
        print('  cam%d format regs: %s' % (c, {k: '%02X' % x for k, x in fmt[c].items()}))
    subprocess.Popen(['python3', os.path.join(S265, 'show_pattern.py'), '--grid', '--region', a.region, '--layout', a.layout],
                     cwd=S265, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)
    raws = {}
    PITCH = 640 * 4
    for c in (0, 1):
        r.select(c, 1.5)
        # only the row band that holds the chart: fs.read_base64 is capped at 256 KB, so two
        # save_raw() slices of <= 90 rows (static scene, frozen exposure -> frames identical)
        ys = [m['bbox'][1] for m in bbox[c].values()]; ye = [m['bbox'][3] for m in bbox[c].values()]
        row0 = max(0, int(min(ys)) - 2); row1 = min(480, int(max(ye)) + 3)
        band = np.zeros((480, 640, 4), np.uint8)
        rr = row0
        t0 = time.time()
        while rr < row1:
            rows = min(90, row1 - rr)
            n = r.value('sentai.camera.save_raw("/s268_c%d.raw", %d, %d)' % (c, rr * PITCH, rows * PITCH), 60)
            data = r.download('/s268_c%d.raw' % c, timeout=300)
            if len(data) != rows * PITCH:
                print('  cam%d: slice rows %d..%d -> save_raw %s, got %d bytes (expected %d)' % (c, rr, rr + rows, n, len(data), rows * PITCH))
            band[rr:rr + rows] = np.frombuffer(data[:rows * PITCH], np.uint8).reshape(rows, 640, 4)
            rr += rows
        print('  cam%d rows %d..%d captured in %.0f s' % (c, row0, row1, time.time() - t0))
        np.save(os.path.join(a.out, 'cam%d_band.npy' % c), band)
        raws[c] = (band, row0, row1)
    r.close()
    subprocess.run(['python3', os.path.join(S265, 'show_pattern.py'), '--quit'], cwd=S265, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # ---- C: analysis -------------------------------------------------------------------
    print('== C: analysis')
    result = {'region': a.region, 'exposure': expo, 'format_regs': {str(k): v for k, v in fmt.items()}, 'cams': {}}
    for c in (0, 1):
        fr, row0, row1 = raws[c]
        band_px = fr[row0:row1]

        def patch_mean(name, plane=None):
            x0, y0, x1, y1 = [int(round(v)) for v in bbox[c][name]['bbox']]
            w, h = x1 - x0, y1 - y0
            sx, sy = int(w * 0.15), int(h * 0.15)   # central 70 %
            p = fr[y0 + sy:y1 - sy, x0 + sx:x1 - sx]
            return p.reshape(-1, 4).mean(0)

        mw, mb, mr, mbl = patch_mean('white'), patch_mean('black'), patch_mean('red'), patch_mean('blue')
        spread = band_px.reshape(-1, 4).std(0)
        bx = int(np.argmin(spread))                       # the constant byte = X
        rest = [i for i in range(4) if i != bx]
        by = max(rest, key=lambda i: mw[i] - mb[i])       # largest white-black contrast = Y
        cu, cv = [i for i in rest if i != by]
        # V (Cr) is high on red and low on blue; U (Cb) the opposite
        if (mr[cu] - mbl[cu]) > (mr[cv] - mbl[cv]):
            cu, cv = cv, cu
        roles = {'X': bx, 'Y': by, 'U': cu, 'V': cv}
        print('  cam%d byte roles %s (X byte std %.1f, value %.0f); white Y %.1f U %.1f V %.1f | black Y %.1f U %.1f V %.1f' % (
            c, roles, spread[bx], band_px[..., bx].mean(), mw[by], mw[cu], mw[cv], mb[by], mb[cu], mb[cv]))
        Y, U, V = fr[..., by], fr[..., cu], fr[..., cv]
        per = {}
        for lim in (False, True):
            rgb = yuv_to_rgb(Y, U, V, lim)
            meas = {}
            des = []
            for name, m in bbox[c].items():
                x0, y0, x1, y1 = [int(round(v)) for v in m['bbox']]
                w, h = x1 - x0, y1 - y0
                sx, sy = int(w * 0.15), int(h * 0.15)
                p = rgb[y0 + sy:y1 - sy, x0 + sx:x1 - sx].reshape(-1, 3)
                mean = p.mean(0); tgt = np.array(m['target'], float)
                d00 = de2000_lab(srgb_to_lab(mean), srgb_to_lab(tgt)); d76 = de76(mean, tgt)
                yuvm = np.array([Y[y0 + sy:y1 - sy, x0 + sx:x1 - sx].mean(), U[y0 + sy:y1 - sy, x0 + sx:x1 - sx].mean(), V[y0 + sy:y1 - sy, x0 + sx:x1 - sx].mean()])
                meas[name] = {'name': name, 'target': m['target'], 'mean': [float(v) for v in mean], 'yuv': [float(v) for v in yuvm],
                              'std': [float(v) for v in p.std(0)], 'bbox': m['bbox'], 'n_px': int(p.shape[0]), 'dE76': float(d76), 'dE00': float(d00)}
                des.append(d00)
            per['limited' if lim else 'full'] = {'mean_dE00': float(np.mean(des)), 'patches': meas}
            Image.fromarray(rgb.astype(np.uint8)).save(os.path.join(a.out, 'cam%d_yuv_%s.png' % (c, 'limited' if lim else 'full')))
        best = min(per, key=lambda k: per[k]['mean_dE00'])
        greys = np.mean([per[best]['patches'][n]['dE00'] for n in per[best]['patches'] if 'neutral' in n or n in ('white', 'black')])
        print('  cam%d dE00 vs sRGB targets: full-range conversion %.1f, limited-range %.1f -> %s fits (greys %.1f)' % (
            c, per['full']['mean_dE00'], per['limited']['mean_dE00'], best, greys))
        gu = np.mean([per[best]['patches'][n]['yuv'][1] for n in per[best]['patches'] if 'neutral' in n])
        gv = np.mean([per[best]['patches'][n]['yuv'][2] for n in per[best]['patches'] if 'neutral' in n])
        print('  cam%d grey patches: mean U %.1f V %.1f (128 = neutral)' % (c, gu, gv))
        # files for compare_patches.py (reference | cam0 | cam1 from the YUV captures)
        json.dump(per[best]['patches'], open(os.path.join(a.out, 'measure_patches_cam%d.json' % c), 'w'), indent=1)
        Image.open(os.path.join(a.out, 'cam%d_yuv_%s.png' % (c, best))).save(os.path.join(a.out, 'tune_cam%d_001_yuv.jpg' % c), quality=95)
        result['cams'][str(c)] = {'roles': roles, 'x_value': float(band_px[..., bx].mean()), 'rows': [row0, row1], 'white_yuv': [float(mw[by]), float(mw[cu]), float(mw[cv])],
                                  'black_yuv': [float(mb[by]), float(mb[cu]), float(mb[cv])], 'grey_uv': [float(gu), float(gv)],
                                  'dE00_full': per['full']['mean_dE00'], 'dE00_limited': per['limited']['mean_dE00'], 'best': best, 'greys_dE00': float(greys)}
    json.dump(result, open(os.path.join(a.out, 'result.json'), 'w'), indent=1)
    # comparison image from the YUV captures
    cmp_tool = os.path.join(HERE, '..', '..', 'camera_modules', 'compare_patches.py')
    subprocess.run(['python3', cmp_tool, a.out, '--out', os.path.join(a.out, 'compare_yuv.png'),
                    '--title', 's268 YUV path (sensor YUV -> BT.601) — reference | cam0 | cam1'])


if __name__ == '__main__':
    main()

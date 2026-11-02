#!/usr/bin/env python3
"""EXP-s269: show reference frames (with ground truth) from the EDAPA synthetic datasets on the
Kamvas, let each camera look at them, run the on-board YUV420 Edge TPU detector and compare
the detections with the ground truth projected into the camera image through the s265
homography.

Per reference image: a 1920x1080 pattern canvas (the coordinate system of locate.json) with
the 1280x960 frame scaled into the screen rect both cameras see; shown through show_pattern.py.
Board: camera.yuv_mode(1), init, tpu.load(model), pipeline.start(conf, iou, 50); for each
camera (select, no alternation) and each image: settle, read K detection frames
(pipeline.get_ex), score each frame against the projected GT (greedy IoU match), save an
overlay on the reference image (GT green, detections red/orange by camera).

Usage: python3 run_display_eval.py --out iter1 [--model /models/c2f_thick_yuv420_int8_edgetpu.tflite]
       [--locate ../../camera_modules/out/measure_dist45b/locate.json] [--frames 6] [--conf 0.25]
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
S265 = os.path.join(HERE, '..', 's265_camera_color_calibration')
sys.path.insert(0, S265)
from board_repl import Repl  # noqa: E402
import serial  # noqa: E402

CANVAS = (1920, 1080)


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


def apply_h(H, pts):
    p = np.c_[pts, np.ones(len(pts))] @ np.asarray(H).T
    return p[:, :2] / p[:, 2:3]


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def match(gt, dets, thr):
    """greedy by confidence; returns (tp, fp, fn, matched gt indices)"""
    used = set(); tp = 0; fp = 0
    for d in sorted(dets, key=lambda d: -d[4]):
        best, bi = 0.0, -1
        for i, g in enumerate(gt):
            if i in used: continue
            v = iou(d[:4], g)
            if v > best: best, bi = v, i
        if best >= thr: used.add(bi); tp += 1
        else: fp += 1
    return tp, fp, len(gt) - len(used), used


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--model', default='/models/c2f_thick_yuv420_int8_edgetpu.tflite')
    ap.add_argument('--locate', default=os.path.join(HERE, '..', '..', 'camera_modules', 'out', 'measure_dist45b', 'locate.json'))
    ap.add_argument('--frames', type=int, default=6, help='detection frames read per image and camera')
    ap.add_argument('--conf', type=float, default=0.25, help='pipeline confidence threshold (recall also reported at 0.5)')
    ap.add_argument('--iou', type=float, default=0.3, help='IoU for a match (boxes are ~10-25 px in the camera)')
    ap.add_argument('--settle', type=float, default=1.5)
    ap.add_argument('--crop', type=float, default=1.0, help='show only the central <crop> fraction of the frame, scaled to the screen rect (people appear 1/crop larger)')
    ap.add_argument('--aec', default='', help='hi,lo AEC target window written with camera.aec_set (e.g. 0x1c,0x14; default = production)')
    ap.add_argument('--rgb', action='store_true', help='RGB pipeline (yuv_mode 0) for a single-input RGB model')
    ap.add_argument('--bg', type=int, default=0, help='canvas background grey level (mid grey keeps the AEC from over-exposing the picture)')
    ap.add_argument('--top', type=int, default=0, help='use only the N reference frames with the highest luminance contrast (0 = all)')
    ap.add_argument('--snap', action='store_true', help='save a camera JPEG of the first canvas per camera before the pipeline starts')
    a = ap.parse_args()
    a.out = os.path.abspath(a.out); os.makedirs(a.out, exist_ok=True)
    loc = json.load(open(a.locate))
    cx, cy, cw, ch = loc['common_rect']
    # fit 1280x960 into the common rect, centred (keeps the 4:3 aspect)
    crop_w, crop_h = 1280 * a.crop, 960 * a.crop
    crop_x0, crop_y0 = (1280 - crop_w) / 2.0, (960 - crop_h) / 2.0
    s = min(cw / crop_w, ch / crop_h); iw, ih = crop_w * s, crop_h * s
    ox, oy = cx + (cw - iw) / 2.0, cy + (ch - ih) / 2.0
    print('crop %.2f (%.0fx%.0f from (%.0f,%.0f)) on canvas: scale %.3f at (%.0f, %.0f), %.0fx%.0f of the %dx%d pattern space' % (
        a.crop, crop_w, crop_h, crop_x0, crop_y0, s, ox, oy, iw, ih, *CANVAS))
    refs = sorted(glob.glob(os.path.join(HERE, 'refs', '*.jpg')))
    if a.top:
        def contrast(f):
            g = np.asarray(Image.open(f).convert('L'), float); return float(g.std())
        refs = sorted(sorted(refs, key=contrast, reverse=True)[:a.top])
        print('top-%d contrast frames: %s' % (a.top, [os.path.basename(f)[:-4].split('__')[1][-6:] for f in refs]))
    # canvases
    canv_dir = os.path.join(a.out, 'canvas'); os.makedirs(canv_dir, exist_ok=True)
    gts = {}
    for f in refs:
        name = os.path.basename(f)[:-4]
        im = Image.open(f).convert('RGB').crop((int(round(crop_x0)), int(round(crop_y0)), int(round(crop_x0 + crop_w)), int(round(crop_y0 + crop_h)))).resize((int(round(iw)), int(round(ih))), Image.LANCZOS)
        canvas = Image.new('RGB', CANVAS, (a.bg, a.bg, a.bg)); canvas.paste(im, (int(round(ox)), int(round(oy))))
        canvas.save(os.path.join(canv_dir, name + '.png'))
        boxes = []
        for l in open(f[:-4] + '.txt'):
            p = l.split()
            if len(p) < 5: continue
            xc, yc, w, h = [float(v) for v in p[1:5]]
            b = ((xc - w / 2) * 1280 - crop_x0, (yc - h / 2) * 960 - crop_y0, (xc + w / 2) * 1280 - crop_x0, (yc + h / 2) * 960 - crop_y0)
            # keep boxes at least half inside the crop; clip to it
            bx0, by0, bx1, by1 = max(0, b[0]), max(0, b[1]), min(crop_w, b[2]), min(crop_h, b[3])
            if bx1 <= bx0 or by1 <= by0: continue
            if (bx1 - bx0) * (by1 - by0) < 0.5 * (b[2] - b[0]) * (b[3] - b[1]): continue
            boxes.append((bx0, by0, bx1, by1))
        gts[name] = boxes
    # ---- board -------------------------------------------------------------------
    reset_board()
    r = Repl(log=open(os.path.join(a.out, 'repl.log'), 'a'))
    print('version', r.value('sentai.version()'))
    print('yuv_mode', r.value('sentai.camera.yuv_mode(%d)' % (0 if a.rgb else 1)), 'init', r.cmd('print(sentai.camera.init(1, 30))', 40).replace('\r', '').strip().split('\n')[-1])
    time.sleep(2)
    if a.aec:
        hi, lo = [int(v, 0) for v in a.aec.split(',')]
        for c in (0, 1): print('aec_set cam%d ->' % c, r.value('sentai.camera.aec_set(%d, %d, %d)' % (c, hi, lo)))
    print('tpu.load ->', r.cmd('print(sentai.tpu.load(%r))' % a.model, 60).replace('\r', '').strip().split('\n')[-1], ' inputs', r.value('sentai.tpu.num_inputs()'))
    if a.snap:
        first = os.path.basename(refs[0])[:-4]
        subprocess.run(['python3', os.path.join(S265, 'show_pattern.py'), '--png', os.path.join(canv_dir, first + '.png')], cwd=S265, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)
        for c in (0, 1):
            r.value('sentai.camera.select(%d)' % c); time.sleep(1.5)
            data = r.cmd('print(len(sentai.camera.jpeg(90, %d)))' % (0 if a.rgb else 1), 30)
            r.value('sentai.fs.write("/s269_snap.jpg", sentai.camera.jpeg(90, %d))' % (0 if a.rgb else 1))
            open(os.path.join(a.out, 'view_cam%d.jpg' % c), 'wb').write(r.download('/s269_snap.jpg'))
            v = r.regs(c, [0x3500, 0x3501, 0x3502, 0x350A, 0x350B, 0x56A1])
            print('  view cam%d saved: exposure %d lines gain %.2fx avg_lum %d' % (c, (v[0] << 16 | v[1] << 8 | v[2]) >> 4, (v[3] << 8 | v[4]) / 16, v[5]))
    r.value('sentai.pipeline.direct_tensor(1)'); r.value('sentai.pipeline.invokes_per_frame(1)')
    print('pipeline.start ->', r.value('sentai.pipeline.start(%.3f, 0.45, 50, False)' % a.conf))
    time.sleep(1.0)
    results = {'model': a.model, 'conf': a.conf, 'iou': a.iou, 'scale': s, 'offset': [ox, oy], 'crop': a.crop, 'aec': a.aec, 'rgb': a.rgb, 'images': {}}
    for cam in (0, 1):
        H = np.asarray(loc['cams'][str(cam)]['H_mon_to_cam'])
        Hinv = np.asarray(loc['cams'][str(cam)]['H_cam_to_mon'])
        r.value('sentai.camera.select(%d)' % cam); time.sleep(1.0)
        for f in refs:
            name = os.path.basename(f)[:-4]
            subprocess.run(['python3', os.path.join(S265, 'show_pattern.py'), '--png', os.path.join(canv_dir, name + '.png')],
                           cwd=S265, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(a.settle)
            # GT -> canvas -> camera pixels
            gt_cam = []
            for (x0, y0, x1, y1) in gts[name]:
                c = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float) * s + [ox, oy]
                q = apply_h(H, c)
                gt_cam.append((q[:, 0].min(), q[:, 1].min(), q[:, 0].max(), q[:, 1].max()))
            frames = []
            for k in range(a.frames):
                out = r.cmd('print(sentai.pipeline.get_ex(2000))', 5)
                line = [l for l in out.replace('\r', '').split('\n') if l.strip().startswith('(')]
                if not line: frames.append(None); continue
                tup = eval(line[-1])
                frames.append({'dets': tup[0], 'invoke_ms': tup[1], 'seq': tup[3], 'cam_id': tup[6]})
            per = []
            for fr in frames:
                if fr is None: continue
                dets = [d for d in fr['dets']]
                tp, fp, fn, used = match(gt_cam, dets, a.iou)
                dets50 = [d for d in dets if d[4] >= 0.5]
                tp5, fp5, fn5, _ = match(gt_cam, dets50, a.iou)
                per.append({'seq': fr['seq'], 'cam_id': fr['cam_id'], 'n_det': len(dets), 'tp': tp, 'fp': fp, 'fn': fn,
                            'tp50': tp5, 'fp50': fp5, 'fn50': fn5, 'invoke_ms': fr['invoke_ms']})
            ngt = len(gt_cam)
            rec = np.mean([p['tp'] / ngt for p in per]) if per and ngt else 0.0
            rec50 = np.mean([p['tp50'] / ngt for p in per]) if per and ngt else 0.0
            fpm = np.mean([p['fp'] for p in per]) if per else 0.0
            cam_ids = sorted(set(p['cam_id'] for p in per))
            print('cam%d %-52s GT %2d | recall@%.2f %.2f  recall@0.5 %.2f  FP/frame %.1f | frames %d cam_id %s invoke %s ms' % (
                cam, name, ngt, a.conf, rec, rec50, fpm, len(per), cam_ids, per[0]['invoke_ms'] if per else '-'))
            results['images'].setdefault(name, {})[str(cam)] = {'n_gt': ngt, 'recall': float(rec), 'recall50': float(rec50), 'fp_per_frame': float(fpm),
                                                                 'frames': per, 'gt_cam': gt_cam, 'dets_last': frames[-1]['dets'] if frames and frames[-1] else []}
            # overlay on the reference image: GT green, detections (last frame) back-projected to the canvas
            ov = Image.open(f).convert('RGB').crop((int(round(crop_x0)), int(round(crop_y0)), int(round(crop_x0 + crop_w)), int(round(crop_y0 + crop_h)))); dr = ImageDraw.Draw(ov)
            for (x0, y0, x1, y1) in gts[name]: dr.rectangle([x0, y0, x1, y1], outline=(0, 255, 0), width=2)
            if frames and frames[-1]:
                for d in frames[-1]['dets']:
                    c = np.array([[d[0], d[1]], [d[2], d[1]], [d[2], d[3]], [d[0], d[3]]], float)
                    q = (apply_h(Hinv, c) - [ox, oy]) / s
                    col = (255, 64, 64) if cam == 0 else (255, 160, 0)
                    dr.rectangle([q[:, 0].min(), q[:, 1].min(), q[:, 0].max(), q[:, 1].max()], outline=col, width=2)
                    dr.text((q[:, 0].min(), max(0, q[:, 1].min() - 12)), '%.2f' % d[4], fill=col)
            ov.save(os.path.join(a.out, 'overlay_%s_cam%d.jpg' % (name, cam)), quality=88)
    r.value('sentai.pipeline.stop()')
    r.close()
    subprocess.run(['python3', os.path.join(S265, 'show_pattern.py'), '--quit'], cwd=S265, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    json.dump(results, open(os.path.join(a.out, 'results.json'), 'w'), indent=1)
    # ---- summary per dataset / camera ------------------------------------------------
    print('\n%-40s %-5s %5s %8s %8s %8s' % ('dataset', 'cam', 'GT', 'rec@%.2f' % a.conf, 'rec@0.5', 'FP/fr'))
    for ds in sorted(set(n.split('__')[0] for n in results['images'])):
        for cam in ('0', '1'):
            rows = [v[cam] for n, v in results['images'].items() if n.startswith(ds + '__') and cam in v]
            ngt = sum(x['n_gt'] for x in rows)
            rec = sum(x['recall'] * x['n_gt'] for x in rows) / ngt if ngt else 0
            rec50 = sum(x['recall50'] * x['n_gt'] for x in rows) / ngt if ngt else 0
            print('%-40s cam%s %5d %8.2f %8.2f %8.1f' % (ds[:40], cam, ngt, rec, rec50, np.mean([x['fp_per_frame'] for x in rows])))


if __name__ == '__main__':
    main()

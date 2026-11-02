#!/usr/bin/env python3
"""EXP-s269 host control: run the CPU int8 versions of the detectors on the 640x480 reference
frames (no board, no display) with the reference decoder (s236/host_decode.py) and score
against the GT.  RGB models take the frame directly; YUV420 models take Y [480,640,1] +
UV [240,320,2] made from the frame with BT.601 FULL range (what the OV5640 outputs, s268).

Run: venv-coral312/bin/python3 host_eval.py [--conf 0.25]
"""
import argparse, glob, json, os, sys
import numpy as np
from PIL import Image
HERE = os.path.dirname(os.path.abspath(__file__)); E = os.path.join(HERE, '..')
sys.path.insert(0, os.path.join(E, 's236_model_speed_sweep_640x480'))
import host_decode as hd  # noqa
from ai_edge_litert.interpreter import Interpreter


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0])); iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy; ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def match(gt, dets, thr):
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

RGB_MODELS = sorted(glob.glob(os.path.join(E, 's238_relu_v1_board_speed_sweep', 'host_int8_models', '*.tflite')))
YUV_MODELS = sorted(glob.glob(os.path.join(E, 's241_relu_yuv_vga45_stats', 'models_yuv', '*.tflite')))


def rgb_to_yuv420(img):
    f = img.astype(np.float32)
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    y = 0.299 * r + 0.587 * g + 0.114 * b
    u = -0.168736 * r - 0.331264 * g + 0.5 * b + 128
    v = 0.5 * r - 0.418688 * g - 0.081312 * b + 128
    u2 = u.reshape(240, 2, 320, 2).mean((1, 3)); v2 = v.reshape(240, 2, 320, 2).mean((1, 3))
    return np.clip(y, 0, 255).astype(np.uint8)[..., None], np.stack([np.clip(u2, 0, 255), np.clip(v2, 0, 255)], -1).astype(np.uint8)


def run(model_path, img, conf, iou):
    it = Interpreter(model_path=model_path); it.allocate_tensors()
    ins = it.get_input_details()
    if len(ins) == 1:
        it.set_tensor(ins[0]['index'], img[None].astype(np.uint8))
    else:
        y, uv = rgb_to_yuv420(img)
        for d in ins:
            shp = tuple(d['shape'])
            arr = y if shp[1] == 480 else uv
            if d['dtype'] == np.int8: arr = (arr.astype(np.int16) - 128).astype(np.int8)
            it.set_tensor(d['index'], arr[None])
    it.invoke()
    allb = []
    for od in it.get_output_details():
        arr = it.get_tensor(od['index'])[0]
        stride, anchor = hd.HEAD_BY_GH[arr.shape[0]]
        scale, zp = od['quantization']
        allb.append(hd.decode_head(arr, scale, zp, stride, anchor))
    boxes = np.concatenate(allb, 0); boxes = boxes[boxes[:, 4] >= conf]
    boxes = hd._nms(boxes, iou)
    return [(float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(b[4]), 0) for b in boxes]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--conf', type=float, default=0.25); ap.add_argument('--iou', type=float, default=0.3)
    ap.add_argument('--models', default='c2f,c2f_thick', help='models_yuv/*.tflite are edgetpu-compiled (custom op) and cannot run on the CPU'); a = ap.parse_args()
    want = a.models.split(',')
    models = [m for m in RGB_MODELS + YUV_MODELS if os.path.basename(m)[:-7] in want]
    refs = sorted(glob.glob(os.path.join(HERE, 'refs_640', '*.jpg')))
    ctrl = np.fromfile(os.path.join(E, 's238_relu_v1_board_speed_sweep', 'test.bin'), np.uint8).reshape(480, 640, 3)
    print('%-22s %-30s %4s %4s %8s %5s' % ('model', 'dataset', 'GT', 'det', 'recall', 'FP'))
    for m in models:
        name = os.path.basename(m)[:-7]
        c = run(m, ctrl, a.conf, a.iou); print('%-22s %-30s %4s %4d %8s %5s   (s238 control: board/host had ~17-26 dets)' % (name, 'CONTROL test.bin', '-', len(c), '-', '-'))
        per = {}
        for f in refs:
            n = os.path.basename(f)[:-4]; ds = n.split('__')[0][:28]
            img = np.asarray(Image.open(f).convert('RGB'))
            gt = json.load(open(f[:-4] + '.json'))
            dets = run(m, img, a.conf, a.iou)
            tp, fp, fn, _ = match(gt, dets, a.iou)
            d = per.setdefault(ds, [0, 0, 0, 0]); d[0] += len(gt); d[1] += len(dets); d[2] += tp; d[3] += fp
        for ds, (ngt, nd, tp, fp) in per.items():
            print('%-22s %-30s %4d %4d %8.2f %5d' % (name, ds, ngt, nd, tp / ngt if ngt else 0, fp))


if __name__ == '__main__':
    main()

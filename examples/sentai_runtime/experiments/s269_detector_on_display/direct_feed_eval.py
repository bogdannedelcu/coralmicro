#!/usr/bin/env python3
"""EXP-s269 control: feed the 640x480 reference JPEGs straight into the on-board model
(tpu.load_image -> tpu.invoke -> tpu.detect, no camera, no display) and score against the
ground truth.  Separates the detector / post-processing from the display + camera chain.

Usage: python3 direct_feed_eval.py --out iter_direct [--model /models/iarna_c2f.tflite] [--conf 0.25]
Requires the JPEGs on the card under /refs640/ (copied over USB mass storage) and
refs_640/<name>.json with the GT boxes in 640x480 pixels.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's265_camera_color_calibration'))
from board_repl import Repl  # noqa: E402
sys.path.insert(0, HERE)
from run_display_eval import match  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--model', default='/models/iarna_c2f.tflite')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--iou', type=float, default=0.3)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    r = Repl(log=open(os.path.join(a.out, 'repl.log'), 'a'))
    print(r.value('sentai.version()'))
    print('tpu.load ->', r.cmd('print(sentai.tpu.load(%r))' % a.model, 60).replace('\r', '').strip().split('\n')[-1])
    res = {'model': a.model, 'conf': a.conf, 'images': {}}
    short = json.load(open(os.path.join(HERE, 'refs_640', 'short_names.json')))   # /r/NN.jpg -> long name (FAT path length limit on the board)
    for k, name in sorted(short.items()):
        jf = os.path.join(HERE, 'refs_640', name + '.json')
        gt = json.load(open(jf))
        li = r.value('sentai.tpu.load_image("/r/%s.jpg")' % k)
        inv = r.value('sentai.tpu.invoke()')
        out = r.cmd('print(sentai.tpu.detect(%.3f, 0.45, 50))' % a.conf, 20).replace('\r', '').strip().split('\n')[-1]
        try:
            dets = eval(out)
        except Exception:
            print('%-60s load_image %s invoke %s detect -> %s' % (name, li, inv, out[:80])); continue
        tp, fp, fn, _ = match(gt, dets, a.iou)
        d50 = [d for d in dets if d[4] >= 0.5]
        tp5, fp5, fn5, _ = match(gt, d50, a.iou)
        res['images'][name] = {'n_gt': len(gt), 'dets': dets, 'tp': tp, 'fp': fp, 'tp50': tp5, 'fp50': fp5}
        print('%-60s GT %2d det %2d | recall@%.2f %.2f FP %d | recall@0.5 %.2f FP %d | conf %s' % (
            name[-26:], len(gt), len(dets), a.conf, tp / len(gt) if gt else 0, fp, tp5 / len(gt) if gt else 0, fp5,
            [round(d[4], 2) for d in sorted(dets, key=lambda d: -d[4])[:5]]))
    r.close()
    json.dump(res, open(os.path.join(a.out, 'results.json'), 'w'), indent=1)
    print('\n%-40s %5s %8s %6s %8s %6s' % ('dataset', 'GT', 'rec@%.2f' % a.conf, 'FP', 'rec@0.5', 'FP'))
    for ds in sorted(set(n.split('__')[0] for n in res['images'])):
        rows = [v for n, v in res['images'].items() if n.startswith(ds + '__')]
        ngt = sum(x['n_gt'] for x in rows)
        print('%-40s %5d %8.2f %6d %8.2f %6d' % (ds[:40], ngt, sum(x['tp'] for x in rows) / ngt if ngt else 0, sum(x['fp'] for x in rows),
                                                sum(x['tp50'] for x in rows) / ngt if ngt else 0, sum(x['fp50'] for x in rows)))


if __name__ == '__main__':
    main()

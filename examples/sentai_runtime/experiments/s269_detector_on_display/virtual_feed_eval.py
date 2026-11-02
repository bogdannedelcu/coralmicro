#!/usr/bin/env python3
"""EXP-s269 control B: feed the reference frames through the VIRTUAL camera (BMP on the card,
camera.select(-1, path)) into the real detection pipeline (PrepTask -> PXP -> direct tensor ->
Edge TPU -> firmware decode), i.e. the same path the live camera uses, without display or
sensor.  /vb/t238.bmp is the s238 positive control (c2f: 17 detections on board and host, July).

Usage: python3 virtual_feed_eval.py --out iter_virtual_rgb [--model /models/iarna_c2f.tflite] [--conf 0.25]
"""
import argparse
import json
import os
import sys
import time

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
    ap.add_argument('--frames', type=int, default=3)
    a = ap.parse_args()
    a.out = os.path.abspath(a.out); os.makedirs(a.out, exist_ok=True)
    short = json.load(open(os.path.join(HERE, 'refs_640', 'short_names.json')))
    r = Repl(log=open(os.path.join(a.out, 'repl.log'), 'a'))
    print(r.value('sentai.version()'))
    print('select(-1, /vb/t238.bmp) ->', r.value('sentai.camera.select(-1, "/vb/t238.bmp")'), ' native_res', r.value('sentai.camera.native_res()'))
    print('tpu.load ->', r.cmd('print(sentai.tpu.load(%r))' % a.model, 60).replace('\r', '').strip().split('\n')[-1])
    print('pipeline.start ->', r.value('sentai.pipeline.start(%.3f, 0.45, 50, False)' % a.conf)); time.sleep(1.0)

    def read_dets(n):
        out = []
        for _ in range(n):
            o = r.cmd('print(sentai.pipeline.get_ex(3000))', 6).replace('\r', '').strip().split('\n')[-1]
            try:
                t = eval(o); out.append(t[0])
            except Exception:
                out.append(None)
        return out

    # positive control
    fr = read_dets(a.frames)
    ok = [d for d in fr if d is not None]
    print('CONTROL t238 (s238 test image): detections per frame %s, max conf %s' % ([len(d) for d in ok], max([max([x[4] for x in d] or [0]) for d in ok] or [0])))
    res = {'model': a.model, 'conf': a.conf, 'control_t238': ok[-1] if ok else None, 'images': {}}
    for k, name in sorted(short.items()):
        gt = json.load(open(os.path.join(HERE, 'refs_640', name + '.json')))
        sel = r.value('sentai.camera.select(-1, "/vb/%s.bmp")' % k); time.sleep(0.4)
        fr = [d for d in read_dets(a.frames) if d is not None]
        dets = fr[-1] if fr else []
        tp, fp, fn, _ = match(gt, dets, a.iou)
        d50 = [d for d in dets if d[4] >= 0.5]; tp5, fp5, _, _ = match(gt, d50, a.iou)
        res['images'][name] = {'n_gt': len(gt), 'dets': dets, 'tp': tp, 'fp': fp, 'tp50': tp5, 'fp50': fp5, 'select_rc': sel}
        print('%-28s GT %2d det %2d | recall@%.2f %.2f FP %d | recall@0.5 %.2f FP %d | conf %s' % (
            name[-28:], len(gt), len(dets), a.conf, tp / len(gt) if gt else 0, fp, tp5 / len(gt) if gt else 0, fp5,
            [round(d[4], 2) for d in sorted(dets, key=lambda d: -d[4])[:6]]))
    r.value('sentai.pipeline.stop()'); r.value('sentai.camera.select(0)'); r.close()
    json.dump(res, open(os.path.join(a.out, 'results.json'), 'w'), indent=1)
    print('\n%-40s %5s %8s %6s %8s %6s' % ('dataset', 'GT', 'rec@%.2f' % a.conf, 'FP', 'rec@0.5', 'FP'))
    for ds in sorted(set(n.split('__')[0] for n in res['images'])):
        rows = [v for n, v in res['images'].items() if n.startswith(ds + '__')]
        ngt = sum(x['n_gt'] for x in rows)
        print('%-40s %5d %8.2f %6d %8.2f %6d' % (ds[:40], ngt, sum(x['tp'] for x in rows) / ngt if ngt else 0, sum(x['fp'] for x in rows),
                                                sum(x['tp50'] for x in rows) / ngt if ngt else 0, sum(x['fp50'] for x in rows)))


if __name__ == '__main__':
    main()

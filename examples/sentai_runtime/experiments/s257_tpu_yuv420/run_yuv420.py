#!/usr/bin/env python3
"""EXP-s257 / OP-S10-W24-T3 -- Edge TPU inference on the YUV420 (dual-input) detectors.

The firmware already has the path (s240 lineage): camera in native YUV
(`camera.yuv_mode(1)` before init), PXP repacks the XYUV frame into Y (640x480)
+ UV (320x240x2) planes, the loader sets `g_sentai_yuv_dual` when the model has
two inputs and the detection pipeline invokes with both planes.  This runner:

  1. pushes the YUV420 EdgeTPU models over HTTP (`/api/write/models/<name>`,
     1 MB cap -> all of them fit) -- needs `sentai.usb.ip(1)` on the board;
  2. on ONE boot: yuv_mode(1), camera.init(1, fps), then for each model:
     tpu.load -> pipeline.start -> run N seconds -> pipeline.infer_stats()
     -> pipeline.stop();
  3. prints ms/invoke (min/avg/max), pipeline fps and detections per model.

Usage: python3 run_yuv420.py [--models DIR] [--seconds 20] [--fps 45] [--no-upload] [--tag x]
"""
import argparse, glob, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402

DEFAULT_MODELS = '/home/bogdan/work/ch569w/experiments/exp021_yolov5_640_board/models'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--models', default=DEFAULT_MODELS)
    ap.add_argument('--pattern', default='*yuv420*_edgetpu.tflite')
    ap.add_argument('--seconds', type=int, default=20)
    ap.add_argument('--fps', type=int, default=45)
    ap.add_argument('--no-upload', action='store_true')
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    a = ap.parse_args()
    files = sorted(glob.glob(os.path.join(a.models, a.pattern)), key=os.path.getsize)
    if not files:
        print('no models match', a.pattern, 'in', a.models); return 2
    ser = B.open_repl(); B.cmd(ser, 'import sentai', 5)
    log = open(os.path.join(HERE, 'run_%s.log' % a.tag), 'w')

    def q(line, t=20):
        out = B.cmd(ser, line, t); log.write('>>> %s\n%s\n' % (line, out)); log.flush(); return out
    print(q('print(sentai.version())', 8).strip().split('\n')[-2][:70])
    if not a.no_upload:
        q('print("IP", sentai.usb.ip(1))', 10); time.sleep(4)
        q('sentai.fs.mkdir("/models")', 10)
        for f in files:
            name = os.path.basename(f)
            r = subprocess.run(['curl', '-s', '-m', '120', '-X', 'POST', '--data-binary', '@' + f,
                                'http://10.0.0.1/api/write/models/' + name], capture_output=True, text=True)
            print('upload %-44s %7d B -> %s' % (name, os.path.getsize(f), (r.stdout.strip() or r.stderr.strip())[:60]))
    q('print("YUVMODE", sentai.camera.yuv_mode(1))', 5)
    out = q('print("INIT", sentai.camera.init(1, %d))' % a.fps, 60)
    print([l.strip()[:80] for l in out.split('\n') if l.strip().startswith('INIT') or '[cam] genlock' in l])
    print('%-40s %8s %8s %8s %8s %6s %6s' % ('model', 'ms_min', 'ms_avg', 'ms_max', 'fps', 'frames', 'dets'))
    rows = []
    for f in files:
        name = os.path.basename(f)
        out = q('print("LOAD", sentai.tpu.load("/models/%s"))' % name, 90)
        ld = [l.strip() for l in out.split('\n') if l.strip().startswith(('LOAD', 'Inputs:', 'in[', 'Arena'))]
        log.write('\n'.join(ld) + '\n')
        if 'LOAD 0' not in out:
            print('%-40s LOAD FAILED / REPL silent -> stop sweep' % name[:40]); break
        q('sentai.pipeline.infer_reset()', 5)
        t_start = time.time()
        q('print("START", sentai.pipeline.start(0.5))', 30)
        time.sleep(a.seconds)
        st = q('print("STATS", sentai.pipeline.stats())', 10)
        st2 = q('print("ISTATS", sentai.pipeline.infer_stats())', 10)
        wall = time.time() - t_start
        q('print("STOP", sentai.pipeline.stop())', 30)
        d = {}
        for src, pre in ((st, ''), (st2, 'i_')):
            m = re.search(r"STATS (\{.*\})", src)
            if m:
                for k, v in re.findall(r"'(\w+)': (-?\d+)", m.group(1)):
                    d[pre + k] = int(v)
        frames = d.get('frames', d.get('i_ok', 0)); inv_sum = d.get('invoke_ms_sum', d.get('i_ms_sum', 0))
        avg = inv_sum / frames if frames else 0
        fps = d.get('fps_x100', 0) / 100.0 if d.get('fps_x100') else (frames / wall if wall else 0)
        row = (name[:40], d.get('invoke_ms_min', 0), round(avg, 1), d.get('invoke_ms_max', 0), round(fps, 1), frames, d.get('detections_total', 0))
        rows.append(row)
        print('%-40s %8s %8s %8s %8s %6s %6s   i_ok=%s i_fail=%s last_rc=%s' % (row + (d.get('i_ok'), d.get('i_fail'), d.get('i_last_rc'))))
        for l in ld[:3]: print('    ', l[:100])
        if not st.strip() or 'STATS' not in st:
            print('REPL silent after %s -> stop sweep' % name); break
    out = q('print(sentai.rtos.dmesg())', 10)
    print('events:', ' '.join(re.findall(r'E:0[AB4][0-9A-F]{2}:\d+', out))[:300])
    # W24-T6/T7 counters: synchronous-URB timeouts and in-pipeline recoveries.
    out = q('print("URB", sentai.tpu.urb_sync_stats())', 10)
    print([l.strip()[:200] for l in out.split('\n') if l.strip().startswith('URB')])
    with open(os.path.join(HERE, 'results_%s.csv' % a.tag), 'w') as f:
        f.write('model,ms_min,ms_avg,ms_max,fps,frames,detections\n')
        for r in rows: f.write(','.join(str(x) for x in r) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

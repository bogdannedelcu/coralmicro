#!/usr/bin/env python3
"""EXP-s257 / OP-S10-W24-T10 -- fewer URBs per invoke: sweep the bulk-OUT chunk size.

Per invoke the single-EP stream is ~80 KB instructions + 307 200 B Y + 153 600 B UV,
cut into `tpu.chunk_size()` pieces (default 36 KB -> ~21 URBs incl. headers,
outputs, event).  Larger chunks mean fewer URB submit/complete round-trips
(each ~50-100 us of host-task + EHCI latency).  This measures the real gain.

One boot, camera YUV at --fps; per chunk size and per model:
  isolated:  N `tpu.invoke()` (GPT1-timed ms returned by the firmware) -> min/avg/max
  pipeline:  `pipeline.start` for --seconds -> fps, invoke ms avg, fails, URB counters

Usage: python3 chunk_sweep.py [--sizes 36,64,96,128,160] [--models c3,c2f_thick]
                              [--n 40] [--seconds 10] [--fps 45] [--tag x]
"""
import argparse, os, re, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402


def kv(s, pre=''):
    d = {}
    m = re.search(r"\{.*\}", s, re.S)
    if m:
        for k, v in re.findall(r"'(\w+)': (-?\d+)", m.group(0)):
            d[pre + k] = int(v)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sizes', default='36,64,96,128,160')
    ap.add_argument('--models', default='c3,c2f_thick')
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--fps', type=int, default=45)
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    a = ap.parse_args()
    sizes = [int(x) for x in a.sizes.split(',')]
    models = a.models.split(',')
    ser = B.open_repl(); B.cmd(ser, 'import sentai', 5)
    log = open(os.path.join(HERE, 'chunk_%s.log' % a.tag), 'w')

    def q(line, t=20):
        out = B.cmd(ser, line, t); log.write('>>> %s\n%s\n' % (line, out)); log.flush(); return out

    print(q('print(sentai.version())', 8).strip().split('\n')[-2][:70])
    q('print("YUVMODE", sentai.camera.yuv_mode(1))', 5)
    print([l.strip()[:80] for l in q('print("INIT", sentai.camera.init(1, %d))' % a.fps, 60).split('\n')
           if l.strip().startswith('INIT')])
    print('%-10s %6s %6s | %8s %8s %8s | %7s %8s %6s %6s %9s' %
          ('model', 'chunkKB', 'URB/inv', 'iso_min', 'iso_avg', 'iso_max', 'pip_fps', 'pip_ms', 'frames', 'fail', 'urb_to'))
    rows = []
    for name in models:
        out = q('print("LOAD", sentai.tpu.load("/models/%s_yuv420_int8_edgetpu.tflite"))' % name, 90)
        if 'LOAD 0' not in out:
            print('LOAD FAILED', name); break
        for kb in sizes:
            q('print("CS", sentai.tpu.chunk_size(%d))' % (kb * 1024), 5)
            # isolated invokes (first one warms the parameter cache after a load)
            q('sentai.tpu.invoke()', 20)
            u0 = kv(q('print(sentai.tpu.urb_sync_stats())', 8))
            out = q('print("ISO", [sentai.tpu.invoke() for _ in range(%d)])' % a.n, 120)
            m = re.search(r'ISO \[(.*)\]', out)
            iso = [int(x) for x in m.group(1).split(',')] if m else []
            u1 = kv(q('print(sentai.tpu.urb_sync_stats())', 8))
            urb_per = (u1.get('entered', 0) - u0.get('entered', 0)) / max(1, len(iso))
            # pipeline
            q('sentai.pipeline.infer_reset()', 5)
            q('print("START", sentai.pipeline.start(0.5))', 30)
            time.sleep(a.seconds)
            st = q('print("STATS", sentai.pipeline.stats())', 10)
            ist = kv(q('print("ISTATS", sentai.pipeline.infer_stats())', 10), 'i_')
            q('print("STOP", sentai.pipeline.stop())', 30)
            u2 = kv(q('print(sentai.tpu.urb_sync_stats())', 8))
            frames = ist.get('i_ok', 0); fail = ist.get('i_fail', 0)
            pms = round(ist.get('i_ms_sum', 0) / frames, 2) if frames else 0
            mfps = re.search(r'STATS \((\d+), (\d+), ([\d.]+)\)', st)
            pfps = float(mfps.group(3)) if mfps else round(frames / a.seconds, 1)
            to = u2.get('take_failed', 0) + u2.get('csr_timeouts', 0) + u2.get('desync_count', 0)
            fails = [x for x in iso if x < 0]
            okv = [x for x in iso if x >= 0]
            row = (name, kb, round(urb_per, 1), min(okv) if okv else -1, round(sum(okv) / len(okv), 2) if okv else -1,
                   max(okv) if okv else -1, pfps, pms, frames, fail + len(fails), to)
            rows.append(row)
            print('%-10s %6d %6.1f | %8d %8.2f %8d | %7.1f %8.2f %6d %6d %9d' % row)
    q('print("CS", sentai.tpu.chunk_size(%d))' % (36 * 1024), 5)   # restore default
    print(q('print(sentai.tpu.urb_sync_stats())', 8).strip()[-700:])
    with open(os.path.join(HERE, 'chunk_%s.csv' % a.tag), 'w') as f:
        f.write('model,chunk_kb,urb_per_invoke,iso_min_ms,iso_avg_ms,iso_max_ms,pipeline_fps,pipeline_invoke_ms,frames,fails,urb_timeouts\n')
        for r in rows: f.write(','.join(str(x) for x in r) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())

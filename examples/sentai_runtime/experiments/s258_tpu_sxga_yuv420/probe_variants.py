#!/usr/bin/env python3
"""EXP-s258 / OP-S10-W24-T11 -- time a list of Edge TPU models on the board with
isolated `tpu.invoke()` (arena content as input; the TPU time does not depend
on pixel values).  Uploads them over HTTP/NCM first unless --no-upload.

Usage: python3 probe_variants.py models/*_edgetpu.tflite [--n 20] [--no-upload] [--tag x]
Prints: model, compiled bytes, invoke min/avg/max ms, fails, URB timeouts/desync.
The trained s257 c2f (/models/c2f_yuv420_int8_edgetpu.tflite) is always timed
first as the reference.
"""
import argparse, glob, os, re, subprocess, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 's236_model_speed_sweep_640x480'))
import board_serial as B  # noqa: E402


def kv(s):
    d = {}
    m = re.search(r"\{.*\}", s, re.S)
    if m:
        for k, v in re.findall(r"'(\w+)': (-?\d+)", m.group(0)):
            d[k] = int(v)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+')
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--no-upload', action='store_true')
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M%S'))
    a = ap.parse_args()
    B.PORT = (sorted(glob.glob('/dev/ttyACM*')) or [B.PORT])[0]
    ser = B.open_repl(); B.cmd(ser, 'import sentai', 5)
    log = open(os.path.join(HERE, 'probe_%s.log' % a.tag), 'w')

    def q(line, t=20):
        out = B.cmd(ser, line, t); log.write('>>> %s\n%s\n' % (line, out)); log.flush(); return out

    print(q('print(sentai.version())', 8).strip().split('\n')[-2][:70])
    if not a.no_upload:
        q('print("IP", sentai.usb.ip(1))', 10); time.sleep(4)
        for f in a.files:
            n = os.path.basename(f)
            r = subprocess.run(['curl', '-s', '-m', '120', '-X', 'POST', '--data-binary', '@' + f,
                                'http://10.0.0.1/api/write/models/' + n], capture_output=True, text=True)
            print('upload %-52s %7d B -> %s' % (n, os.path.getsize(f), r.stdout.strip()[:50]))
    print('%-52s %8s %6s %7s %6s %5s %6s' % ('model', 'bytes', 'min', 'avg', 'max', 'fail', 'urb'))
    names = ['/models/c2f_yuv420_int8_edgetpu.tflite'] + ['/models/' + os.path.basename(f) for f in a.files]
    for path in names:
        out = q('print("LOAD", sentai.tpu.load("%s"))' % path, 90)
        if 'LOAD 0' not in out:
            print('%-52s LOAD FAILED: %s' % (os.path.basename(path), out.strip()[-80:])); continue
        size = re.search(r'Model loaded: (\d+) bytes', out)
        q('sentai.tpu.invoke()', 30)
        out = q('print("ISO", [sentai.tpu.invoke() for _ in range(%d)])' % a.n, 300)
        m = re.search(r'ISO \[(.*)\]', out)
        v = [int(x) for x in m.group(1).split(',')] if m else []
        ok = [x for x in v if x >= 0]
        u = kv(q('print(sentai.tpu.urb_sync_stats())', 8))
        print('%-52s %8s %6s %7s %6s %5d %6d' % (
            os.path.basename(path)[:52], size.group(1) if size else '?',
            min(ok) if ok else -1, round(sum(ok) / len(ok), 1) if ok else -1, max(ok) if ok else -1,
            len(v) - len(ok), u.get('take_failed', 0) + u.get('desync_count', 0)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

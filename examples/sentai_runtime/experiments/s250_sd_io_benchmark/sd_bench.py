#!/usr/bin/env python3
"""EXP-s250 — SD card (FileX over USDHC1) I/O benchmark, driven over the REPL.

Measures, on the board (sentai.rtos.ticks_ms around each call):
  B1  sentai.fs.write(path, N bytes)        — file create + write + close
  B2  sentai.fs.read(path)                  — whole-file read (single copy)
  B3  sentai.fs.append(path, 4 KB) x K      — small-append pattern (logs)
  B4  sentai.fs.sync()                      — fx_media_flush cost
and from the host:
  B5  HTTP GET /api/raw/<file> over CDC-NCM — end-to-end transfer rate

Sizes default to 64 KB / 256 KB (fs.read is capped at 512 KB; bulk reads
belong on HTTP).  Each size runs `--reps` times; the table reports the
median.  Results are appended to results.md by the operator.

Usage: python3 sd_bench.py [--port /dev/ttyACM0] [--reps 3] [--no-http]
"""
import argparse
import statistics
import subprocess
import sys
import time

import serial


class Repl:
    def __init__(self, port):
        self.s = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(0.5)
        self.s.write(b'\x03\r')
        time.sleep(0.5)
        self.s.read(65536)

    def one(self, code, wait=30.0):
        self.s.write(code.encode() + b'\r')
        t0 = time.time()
        out = b''
        last = time.time()
        while time.time() - t0 < wait:
            d = self.s.read(65536)
            if d:
                out += d
                last = time.time()
            if out.rstrip().endswith(b'>>>') and time.time() - last > 0.3:
                break
        txt = out.decode(errors='replace')
        res = [l for l in txt.strip().split('\n')
               if l.strip() and not l.startswith(code[:12]) and '>>>' not in l]
        return res[-1].strip() if res else '(none)'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--no-http', action='store_true')
    ap.add_argument('--sizes', default='65536,262144')
    a = ap.parse_args()
    sizes = [int(x) for x in a.sizes.split(',')]

    r = Repl(a.port)
    print(r.one('print(sentai.version())'))
    print(r.one('print(sentai.sdcard.info())', 10))
    r.one('T=sentai.rtos.ticks_ms')
    r.one('blk=bytes(range(256))*16')          # 4 KB pattern

    rows = []
    r.one('import gc')
    for n in sizes:
        # keep only one copy of the payload live on the 1 MB MP heap
        r.one('data=None; b=None; gc.collect(); data=blk*%d' % (n // 4096), 20)
        w, rd, ok = [], [], True
        for _ in range(a.reps):
            out = r.one('t=T(); sentai.fs.write("/s250_%d.bin", data); print(T()-t)' % n, 60)
            w.append(float(out)) if out.replace('.', '').isdigit() else None
            out = r.one('b=None; gc.collect(); t=T(); b=sentai.fs.read("/s250_%d.bin"); dt=T()-t; print(dt if len(b)==%d else -1)' % (n, n), 60)
            if out.lstrip('-').isdigit() and int(out) >= 0:
                rd.append(float(out))
            else:
                ok = False
                print('  read %d KB failed: %s' % (n // 1024, out))
        r.one('b=None; sentai.fs.remove("/s250_%d.bin")' % n, 20)
        if w and rd:
            wm, rm = statistics.median(w), statistics.median(rd)
            rows.append((n, wm, n / 1024.0 / (wm / 1000.0), rm, n / 1024.0 / (rm / 1000.0), ok))
        else:
            rows.append((n, -1, 0, -1, 0, False))

    # B3 — small appends (log pattern): 64 x 4 KB
    out = r.one('t=T()\rfor i in range(64): sentai.fs.append("/s250_log.bin", blk)\r\rprint(T()-t)', 90)
    app_ms = float(out) if out.replace('.', '').isdigit() else -1
    # B4 — sync
    sync_ms = r.one('t=T(); sentai.fs.sync(); print(T()-t)', 60)
    r.one('sentai.fs.remove("/s250_log.bin")', 20)

    print('\n| size | write ms | write KB/s | read ms | read KB/s | ok |')
    print('|---|---|---|---|---|---|')
    for n, wm, wk, rm, rk, ok in rows:
        print('| %d KB | %.0f | %.0f | %.0f | %.0f | %s |' % (n // 1024, wm, wk, rm, rk, ok))
    print('\nB3 64 x 4 KB append: %.0f ms (%.1f ms/append, %.0f KB/s)' %
          (app_ms, app_ms / 64 if app_ms > 0 else -1, 256.0 / (app_ms / 1000.0) if app_ms > 0 else 0))
    print('B4 fs.sync(): %s ms' % sync_ms)

    if not a.no_http:
        # B5 — HTTP over NCM: write a 256 KB file, fetch it 3x
        r.one('data=blk*64; sentai.fs.write("/s250_http.bin", data); data=None', 60)
        r.one('print(sentai.usb.ip(1))', 8)
        time.sleep(6.0)
        ts = []
        for _ in range(3):
            p = subprocess.run(['curl', '-s', '-m', '30', '-o', '/dev/null', '-w', '%{size_download} %{time_total}',
                                'http://10.0.0.1/api/raw/s250_http.bin'], capture_output=True, text=True)
            try:
                sz, t = p.stdout.split()
                if int(sz) == 262144:
                    ts.append(float(t))
            except ValueError:
                pass
        r.one('sentai.fs.remove("/s250_http.bin")', 20)
        if ts:
            m = statistics.median(ts)
            print('B5 HTTP /api/raw 256 KB x%d: median %.3f s = %.0f KB/s' % (len(ts), m, 256.0 / m))
        else:
            print('B5 HTTP: no complete transfer')
    return 0


if __name__ == '__main__':
    sys.exit(main())

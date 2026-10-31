#!/usr/bin/env python3
"""EXP-s249 — SentAI firmware hardening regression (host side).

Drives the board over USB CDC-ACM (/dev/ttyACM0) and, optionally, the HTTP
server over CDC-NCM (10.0.0.1) and checks the behaviours fixed during the
2026-10-30/31 hardening batches.  Every check prints PASS/FAIL; exit code is
the number of failures.

Usage:
    python3 repl_regression.py [--port /dev/ttyACM0] [--no-http] [--no-camera]

Checks (see README.md for the defect each one pins down):
    R1  big int literal after a longer token on the same line parses
    R2  OverflowError (not garbage) for a literal > 2^63
    R3  int("-big") keeps its sign
    R4  CRLF-pasted indented block runs every body line
    R5  20 KB print arrives complete
    R6  line > 1023 chars is rejected and the REPL stays alive
    R7  Ctrl-C interrupts a busy loop (KeyboardInterrupt) and REPL recovers
    R8  deep recursion raises RecursionError (no board reset)
    R9  rtos.suspend() refuses tcpip_thread / hw_wdog
    R10 camera init + capture of both sensors to the SD card (size > 0)
    R11 IMU init + read returns a dict with x/y/z
    R12 HTTP /api/ls and /api/raw over NCM return the capture, 3x identical
"""
import argparse
import hashlib
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

    def raw(self, data, wait=6.0):
        self.s.write(data)
        t0 = time.time()
        out = b''
        last = time.time()
        while time.time() - t0 < wait:
            d = self.s.read(65536)
            if d:
                out += d
                last = time.time()
            if out.rstrip().endswith(b'>>>') and time.time() - last > 0.4:
                break
        return out.decode(errors='replace')

    def one(self, code, wait=6.0):
        """Run one line, return the last non-echo, non-prompt output line."""
        o = self.raw(code.encode() + b'\r', wait)
        res = [l for l in o.strip().split('\n')
               if l.strip() and not l.startswith(code[:12]) and '>>>' not in l]
        return res[-1].strip() if res else '(none)'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--no-http', action='store_true')
    ap.add_argument('--no-camera', action='store_true')
    a = ap.parse_args()

    r = Repl(a.port)
    fails = 0

    def check(name, ok, detail=''):
        nonlocal fails
        print('%-4s %s %s' % (name, 'PASS' if ok else 'FAIL', detail[:110]))
        if not ok:
            fails += 1

    print(r.one('print(sentai.version())'))

    # R1 — literal > 2^30 after a longer token (the s248 probe line)
    out = r.one('print("SBMR1=0x%08X SBMR2=0x%08X SRSR=0x%08X" % '
                '(sentai.sys.peek32(0x40C04008), sentai.sys.peek32(0x40C0400C), '
                'sentai.sys.peek32(0x40C04010)))')
    check('R1', out.startswith('SBMR1=0x') and 'Error' not in out, out)
    out = r.one('print("SBMR1=0x%08X SBMR2=0x%08X SRSR=0x%08X zzzzzzzzzzzzz", 1086341128)')
    check('R1b', out.endswith('1086341128'), out)

    # R2 — overflow is an exception, not a wrapped value
    out = r.one('print(99999999999999999999)')
    check('R2', 'OverflowError' in out, out)

    # R3 — sign of int(str) on the long-int path
    out = r.one('print(int("-1086341128"), int("1086341128"), 0x4000_0000_0)')
    check('R3', out == '-1086341128 1086341128 17179869184', out)

    # R4 — CRLF block paste runs all body lines
    blk = (b'for i in range(3):\r\n    print("row", i)\r\n'
           b'    print("tail", i*10)\r\n\r\nprint("after")\r\n')
    o = r.raw(blk, 8)
    check('R4', all(k in o for k in ('row 2', 'tail 20', 'after')),
          ' '.join(l.strip() for l in o.split('\n') if 'tail' in l))

    # R5 — 20 KB print complete
    o = r.raw(b'print("A"*20000 + "END")\r', 20)
    check('R5', o.count('A') >= 20000 and 'END' in o, 'A-count=%d' % o.count('A'))

    # R6 — over-long line rejected, REPL alive
    o = r.raw(('x=1;' * 300 + 'print("NOPE")').encode() + b'\r', 12)
    alive = r.one('print("alive", 1+1)')
    check('R6', 'line too long' in o and 'NOPE' not in o and alive == 'alive 2', alive)

    # R7 — Ctrl-C interrupts a busy loop
    r.s.write(b'while True: pass\r')
    time.sleep(1.0)
    r.s.write(b'\x03')
    o = r.raw(b'', 4)
    alive = r.one('print("alive", 2+2)')
    check('R7', 'KeyboardInterrupt' in o and alive == 'alive 4', alive)

    # R8 — recursion limit instead of a reset
    out = r.one('def f(n): return f(n+1)\r\rprint(f(0))', 10)
    check('R8', 'RecursionError' in out or 'maximum recursion' in out, out)

    # R9 — protected tasks
    o1 = r.one('print(sentai.rtos.suspend("tcpip_thread"))')
    o2 = r.one('print(sentai.rtos.suspend("hw_wdog"))')
    check('R9', o1 == '0' and o2 == '0', '%s %s' % (o1, o2))

    if not a.no_camera:
        # R10 — both sensors capture to the card
        rc = r.one('print(sentai.camera.init())', 15)
        time.sleep(2.0)
        s0 = r.one('print(sentai.camera.save_jpeg("/s249_cam0.jpg", 0))', 15)
        s1 = r.one('print(sentai.camera.save_jpeg("/s249_cam1.jpg", 1))', 15)
        ok = rc == '0' and s0.isdigit() and s1.isdigit() and int(s0) > 0 and int(s1) > 0
        check('R10', ok, 'init=%s cam0=%s cam1=%s' % (rc, s0, s1))

    # R11 — IMU
    rc = r.one('print(sentai.imu.init())', 8)
    out = r.one('print(sentai.imu.read())', 8)
    check('R11', rc == '0' and all(k in out for k in ("'x'", "'y'", "'z'")), out)

    if not a.no_http and not a.no_camera:
        # R12 — HTTP over NCM, 3 identical downloads
        r.one('print(sentai.usb.ip(1))', 8)
        time.sleep(6.0)
        ls = subprocess.run(['curl', '-s', '-m', '8', 'http://10.0.0.1/api/ls/'],
                            capture_output=True, text=True).stdout
        sums = set()
        for _ in range(3):
            p = subprocess.run(['curl', '-s', '-m', '15', 'http://10.0.0.1/api/raw/s249_cam0.jpg'],
                               capture_output=True)
            sums.add(hashlib.md5(p.stdout).hexdigest() + ':%d' % len(p.stdout))
        check('R12', 's249_cam0.jpg' in ls and len(sums) == 1 and not next(iter(sums)).endswith(':0'),
              'ls=%d B sums=%s' % (len(ls), sums))

    # R13 — fs.read single-copy path + readback; dmesg now has content
    # (bare expression statements echo their value at the REPL — is_repl
    # compile — so keep print() last or search the whole output)
    r.one('sentai.fs.write("/s249_rw.bin", bytes(range(256))*40)', 10)
    o = r.raw(b'b=sentai.fs.read("/s249_rw.bin"); sentai.fs.remove("/s249_rw.bin"); print(len(b), b[:4], b[-2:])\r', 10)
    check('R13', "10240 b'\\x00\\x01\\x02\\x03' b'\\xfe\\xff'" in o, o.strip().split('\n')[-2].strip() if '\n' in o else o)
    out = r.one('print(len(sentai.rtos.dmesg()) > 0)', 8)
    check('R13b', out == 'True', 'dmesg non-empty: ' + out)

    print('--- task stack high-water marks (words free) ---')
    print(r.one('print(sentai.rtos.tasks())', 10))
    print('failures:', fails)
    return fails


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""_host_run_with_var.py - run a diag driver after setting REPL globals.

Some drivers (e.g. _t_fps_bench.py) expect a `_target_fps` global to be
defined in the REPL before exec. This wrapper sends `_target_fps = N`
(or any --set KEY=VAL) over the REPL first, then exec's the file body
from LFS, then streams output until the '=== done ===' sentinel.
"""
import argparse
import serial
import sys
import time


def _drain(ser, t=0.3):
    end = time.monotonic() + t
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            ser.read(n); end = time.monotonic() + t
        else:
            time.sleep(0.02)


def _enter_repl(ser):
    ser.write(b"\r\x03\x03"); time.sleep(0.3); _drain(ser, 0.4)
    ser.write(b"\r\n"); time.sleep(0.1); _drain(ser, 0.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--file", required=True, help="remote /lib/diag/...")
    ap.add_argument("--set", action="append", default=[],
                    help="REPL var assignment, repeatable. Example: --set _target_fps=30")
    ap.add_argument("--timeout", type=float, default=240.0)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.5)
    _enter_repl(ser)

    for assign in args.set:
        ser.write((assign + "\r\n").encode()); ser.flush()
        time.sleep(0.2); _drain(ser, 0.3)

    cmd = "import sentai; exec(sentai.fs.read(%r))\r\n" % args.file
    ser.write(cmd.encode()); ser.flush()

    deadline = time.monotonic() + args.timeout
    idle_since = time.monotonic()
    saw_done = False
    acc = b""
    try:
        while time.monotonic() < deadline:
            chunk = ser.read(1024)
            if chunk:
                sys.stdout.write(chunk.decode(errors="replace"))
                sys.stdout.flush()
                acc += chunk
                idle_since = time.monotonic()
                if b"=== done ===" in acc:
                    saw_done = True
                    until = time.monotonic() + 1.0
                    while time.monotonic() < until:
                        t = ser.read(ser.in_waiting or 1)
                        if not t:
                            time.sleep(0.05); continue
                        sys.stdout.write(t.decode(errors="replace"))
                        sys.stdout.flush()
                        until = time.monotonic() + 0.3
                    break
            else:
                if time.monotonic() - idle_since > 60.0:
                    print("\n[host] no output 60s - aborting"); break
        if not saw_done:
            print("\n[host] timeout without '=== done ==='")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

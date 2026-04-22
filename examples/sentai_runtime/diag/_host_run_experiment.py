#!/usr/bin/env python3
"""_host_run_experiment.py — run a diag driver over the REPL and stream output."""
import argparse
import serial
import sys
import time


def _drain(ser, timeout=0.3):
    end = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            end = time.monotonic() + timeout
        else:
            time.sleep(0.02)
    return buf


def _enter_repl(ser):
    ser.write(b"\r\x03\x03")
    time.sleep(0.2)
    _drain(ser, 0.4)
    ser.write(b"\r\n")
    time.sleep(0.1)
    _drain(ser, 0.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--file", required=True, help="remote path, e.g. /lib/diag/_e21_vga_capture.py")
    ap.add_argument("--timeout", type=float, default=180.0, help="max run time in s")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.5)
    _enter_repl(ser)

    # This MicroPython is stripped — no open(), use sentai.fs.read() instead.
    cmd = "import sentai; exec(sentai.fs.read(%r))\r\n" % args.file
    ser.write(cmd.encode())
    ser.flush()

    deadline = time.monotonic() + args.timeout
    idle_since = time.monotonic()
    try:
        while time.monotonic() < deadline:
            chunk = ser.read(1024)
            if chunk:
                sys.stdout.write(chunk.decode(errors="replace"))
                sys.stdout.flush()
                idle_since = time.monotonic()
                if chunk.rstrip().endswith(b">>>"):
                    time.sleep(1.0)
                    tail = ser.read(ser.in_waiting or 1)
                    if not tail:
                        break
                    sys.stdout.write(tail.decode(errors="replace"))
                    sys.stdout.flush()
            else:
                if time.monotonic() - idle_since > 30.0:
                    print("\n[host] no output for 30s — aborting")
                    break
    finally:
        ser.close()


if __name__ == "__main__":
    main()

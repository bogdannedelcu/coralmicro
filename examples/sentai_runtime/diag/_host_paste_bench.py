#!/usr/bin/env python3
"""_host_paste_bench.py - run a diag .py via REPL exec(_d) without fs.write.

Why: sentai.fs.write hangs USB CDC for files > ~3KB on this firmware.
This script chunks the local file into a `_d` bytes accumulator over the
REPL (same protocol as _host_upload_repl.py) but skips the final fs.write
and does exec(_d) instead, so the bench script runs straight from RAM.

Sets `_target_fps = N` first, then streams output until '=== done ==='.
"""
import argparse
import pathlib
import serial
import sys
import time

CHUNK = 48


def _drain(ser, t=0.3):
    end = time.monotonic() + t
    buf = b""
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            end = time.monotonic() + t
        else:
            time.sleep(0.02)
    return buf


def _recv_until(ser, term, timeout):
    end = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < end:
        c = ser.read(512)
        if c:
            buf.extend(c)
            if term in bytes(buf):
                return bytes(buf)
        else:
            time.sleep(0.01)
    raise TimeoutError("waiting for %r last=%r" % (term, bytes(buf[-160:])))


def _send_line(ser, line, timeout=5.0):
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(line.encode() + b"\r\n")
    return _recv_until(ser, b"\r\n>>> ", timeout=timeout)


def _open_repl(ser):
    ser.write(b"\r\x03\x03")
    time.sleep(0.2)
    _drain(ser, 0.4)
    ser.write(b"\r\n")
    _recv_until(ser, b"\r\n>>> ", timeout=3.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--file", required=True)
    ap.add_argument("--fps", type=int, required=True)
    ap.add_argument("--timeout", type=float, default=240.0)
    args = ap.parse_args()

    body = pathlib.Path(args.file).read_bytes()
    n = len(body)

    ser = serial.Serial(args.port, 115200, timeout=0.5)
    _open_repl(ser)
    _send_line(ser, "import sentai")
    _send_line(ser, "_target_fps = %d" % args.fps)
    _send_line(ser, "_d = b''")
    resp = _send_line(ser, "print('LEN:%d' % len(_d))").decode(errors="replace")
    if "LEN:0" not in resp:
        print("ERR: _d not empty"); sys.exit(1)

    print("[host] streaming %d bytes ..." % n)
    for i in range(0, n, CHUNK):
        piece = body[i : i + CHUNK]
        _send_line(ser, "_d = _d + " + repr(piece), timeout=3.0)
        if (i // CHUNK) % 10 == 9:
            chk = _send_line(ser, "print('LEN:%d' % len(_d))").decode(errors="replace")
            expected = min(i + CHUNK, n)
            if ("LEN:%d" % expected) not in chk:
                print("ERR: drift at %d: %s" % (expected, chk[-200:]))
                sys.exit(1)
            print("  ... %d/%d" % (expected, n))
    chk = _send_line(ser, "print('LEN:%d' % len(_d))").decode(errors="replace")
    if ("LEN:%d" % n) not in chk:
        print("ERR: final drift: %s" % chk[-200:]); sys.exit(1)
    print("[host] running exec(_d) ...")

    # Don't use _send_line: exec runs for tens of seconds and emits its own
    # output; we just stream until sentinel.
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(b"exec(_d)\r\n")
    ser.flush()

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
                if time.monotonic() - idle_since > 30.0:
                    print("\n[host] no output 30s - aborting"); break
        if not saw_done:
            print("\n[host] timeout without '=== done ==='")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

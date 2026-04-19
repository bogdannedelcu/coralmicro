#!/usr/bin/env python3
# repl_run.py — send commands to SentAI custom MicroPython REPL
# This REPL is line-at-a-time + ':'-triggered multi-line blocks.
# No raw/paste REPL. So we send one line at a time and wait for "\n>>> " or "\n... ".
#
# Usage:
#   python3 repl_run.py --line "import diag" --line "diag.e10_memory('idle')"
#   python3 repl_run.py --script snippet.py           # one-liners separated by \n
#   python3 repl_run.py --file_exec /diags/_run.py    # exec(open(...).read()) on-device
import sys, time, argparse, serial, re

PROMPT = b">>> "
PROMPT_CONT = b"... "


def _recv_until(ser, terminator, timeout=30.0, echo=False):
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
            if echo:
                sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                sys.stdout.flush()
            # look at tail to match prompts
            tail = bytes(buf[-8:])
            if terminator in tail:
                return bytes(buf)
        else:
            time.sleep(0.02)
    raise TimeoutError("timeout waiting for %r; last: %r" % (terminator, bytes(buf[-200:])))


def _send_interrupt(ser):
    ser.write(b"\x03")
    time.sleep(0.1)
    ser.write(b"\x03")
    time.sleep(0.2)


def open_repl(port, baud=115200):
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.2)
    # drain and get to a clean prompt
    ser.reset_input_buffer()
    _send_interrupt(ser)
    ser.write(b"\r\n")
    try:
        _recv_until(ser, PROMPT, timeout=2.0)
    except TimeoutError:
        # try once more
        _send_interrupt(ser)
        ser.write(b"\r\n")
        _recv_until(ser, PROMPT, timeout=3.0)
    return ser


def send_line(ser, line, timeout=60.0, echo=True):
    """Send one REPL line, wait for next >>> prompt. Returns captured text."""
    ser.write((line + "\r\n").encode())
    return _recv_until(ser, PROMPT, timeout=timeout, echo=echo).decode("utf-8", errors="replace")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--line", action="append", default=[],
                    help="one REPL line (repeatable)")
    ap.add_argument("--script", help="file containing REPL lines (one per line)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    lines = list(args.line)
    if args.script:
        with open(args.script) as f:
            for raw in f.read().splitlines():
                s = raw.rstrip()
                if s and not s.startswith("#"):
                    lines.append(s)
    if not lines:
        ap.error("need --line or --script")

    ser = open_repl(args.port)
    try:
        for line in lines:
            if not args.quiet:
                sys.stdout.write("\n>>> %s\n" % line)
                sys.stdout.flush()
            send_line(ser, line, timeout=args.timeout, echo=not args.quiet)
    finally:
        ser.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Host-side driver that runs E20 TPU raw invoke benchmark over the REPL.

Full-buffer terminator match for ">>> " per agent.md §5 to survive
the REPL's per-line mode.  Prints whatever the board prints and also
captures the three summary lines ("E20 TPU RAW BENCH", "invoke ms:",
"effective FPS:") so a caller can diff across firmware builds.
"""

import argparse, re, serial, sys, time


def _drain(s):
    while s.in_waiting:
        s.read(s.in_waiting)


def send(s, line, timeout=60):
    _drain(s)
    s.write(line.encode() + b"\r\n")
    deadline = time.time() + timeout
    buf = bytearray()
    last_nl = 0
    while time.time() < deadline:
        chunk = s.read(4096)
        if chunk:
            # Live-print newly-arrived complete lines.
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n", last_nl)
                if nl < 0: break
                line_txt = bytes(buf[last_nl:nl+1]).decode(errors="replace")
                sys.stdout.write(line_txt)
                sys.stdout.flush()
                last_nl = nl + 1
        # Look for the fresh prompt anywhere — tail-matching fails when
        # the board prints extra text after the final newline.
        if b"\r\n>>> " in bytes(buf) and time.time() - (deadline - timeout) > 0.1:
            # small grace period to pick up any trailing output
            break
    # Flush any residual partial line
    if last_nl < len(buf):
        sys.stdout.write(bytes(buf[last_nl:]).decode(errors="replace"))
        sys.stdout.flush()
    return bytes(buf).decode(errors="replace")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=int, default=120,
                    help="seconds to wait per REPL step")
    args = ap.parse_args()

    s = serial.Serial(args.port, args.baud, timeout=0.3,
                      rtscts=False, xonxoff=False, dsrdtr=False)
    time.sleep(0.3)
    _drain(s)
    s.write(b"\x03\r\n"); time.sleep(0.3); _drain(s)
    # Get a clean prompt
    s.write(b"\r\n"); time.sleep(0.3)

    # Quiet firmware per-frame prints.
    send(s, "import sentai", timeout=5)
    send(s, "sentai.verbose(0)", timeout=5)

    # Import the driver.  E20 does warmup + measurement in module body.
    full = send(s, "from diag.drivers import _e20_tpu_raw", timeout=args.timeout)

    # Extract the summary for the caller's convenience.
    m = re.search(r"invoke ms: min=(\d+) mean=([\d.]+) max=(\d+)", full)
    if m:
        print("\n--- PARSED ---")
        print("min_ms=%s mean_ms=%s max_ms=%s" % (m.group(1), m.group(2), m.group(3)))
    fps = re.search(r"effective FPS \(pure invoke\): ([\d.]+)", full)
    if fps:
        print("fps=%s" % fps.group(1))

    s.close()


if __name__ == "__main__":
    main()

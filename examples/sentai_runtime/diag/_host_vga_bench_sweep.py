#!/usr/bin/env python3
"""_host_vga_bench_sweep.py — sweep VGA30/45/60, restart between.

For each fps in (30, 45, 60):
  1. Reset the board via sentai.sys.reset() (clean slate; avoids
     the set_hw second-call hang noted in
     memory/project_runtime_camera_hw.md).
  2. Wait for NXP enum + REPL ready.
  3. Set `_target_fps = <fps>` in REPL globals.
  4. exec(sentai.fs.read('/lib/diag/_t_vga_bench.py')) — bench
     reads `_target_fps` and runs all 5 modes.
  5. Tail the output until `=== done ===`.

Aggregate the per-fps summary lines into a single table.
"""
import argparse
import re
import sys
import time

import serial


def _drain(ser, timeout=0.4):
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


def _reset(ser):
    ser.write(b"\r\x03\x03")
    time.sleep(0.3)
    _drain(ser, 0.4)
    ser.write(b"import sentai; sentai.sys.reset()\r\n")
    # board reboots — serial port may briefly disappear; the caller
    # handles re-enumeration via lsusb + REPL probe.


def _wait_ready(ser, port, deadline_s=60):
    """Wait until /dev/ttyACM0 re-appears and REPL responds.

    NVIC_SystemReset takes the USB device offline for ~5-10 s while
    the firmware reboots and CDC-ACM re-enumerates.  We close the
    handle, wait for the device to disappear AND reappear, and only
    then attempt to talk to it.  Bounded by deadline_s. """
    import os
    ser.close()
    end = time.monotonic() + deadline_s
    # 1) Wait for the port to disappear (device is rebooting).
    while time.monotonic() < end and os.path.exists(port):
        time.sleep(0.2)
    # 2) Wait for the port to come back.
    while time.monotonic() < end and not os.path.exists(port):
        time.sleep(0.2)
    # 3) Settle for USB CDC stack to be ready.
    time.sleep(2.0)
    # 4) Probe REPL.
    while time.monotonic() < end:
        try:
            ser2 = serial.Serial(port, 115200, timeout=0.5)
            time.sleep(0.3)
            ser2.write(b"\r\n")
            time.sleep(0.6)
            d = ser2.read(800)
            if b">>>" in d:
                return ser2
            ser2.close()
        except Exception:
            time.sleep(0.5)
    raise TimeoutError("REPL did not come back within %ds" % deadline_s)


def _send_line(ser, line, to=10.0):
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(line.encode() + b"\r\n")
    end = time.monotonic() + to
    buf = b""
    while time.monotonic() < end:
        c = ser.read(4096)
        if c:
            buf += c
            if b"\r\n>>> " in buf:
                break
    return buf.decode(errors="replace")


def _run_bench(ser, fps, deadline_s=120):
    # Set fps then exec bench driver.  Bench reads _target_fps from
    # the REPL globals.  We exec via a single line so we don't
    # accidentally enter multi-line mode.
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    cmd = (
        "_target_fps=%d; import sentai; "
        "exec(sentai.fs.read('/lib/diag/_t_vga_bench.py'))\r\n" % fps
    )
    ser.write(cmd.encode())
    end = time.monotonic() + deadline_s
    buf = b""
    output = []
    while time.monotonic() < end:
        c = ser.read(8192)
        if c:
            sys.stdout.write(c.decode(errors="replace"))
            sys.stdout.flush()
            buf += c
            output.append(c)
            if b"=== done ===" in c:
                # consume trailing prompt
                time.sleep(0.5)
                tail = ser.read(ser.in_waiting or 1)
                if tail:
                    sys.stdout.write(tail.decode(errors="replace"))
                    sys.stdout.flush()
                    output.append(tail)
                break
    return b"".join(output).decode(errors="replace")


_summary_re = re.compile(
    r"\s*(single_cam0|single_cam1|alt_1_1|alt_2_1|alt_3_1)\s*\|\s*"
    r"(\d+)/\d+\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*"
    r"(\d+):(\d+)\s*\|\s*(\d+)/\s*(\d+)/\s*(\d+)\s*\|\s*(\d+)"
)


def _parse(out):
    rows = []
    for m in _summary_re.finditer(out):
        rows.append({
            "mode": m.group(1),
            "correct": int(m.group(2)),
            "scrambled": int(m.group(3)),
            "wrong": int(m.group(4)),
            "cam0": int(m.group(5)),
            "cam1": int(m.group(6)),
            "ms_avg": int(m.group(7)),
            "ms_p50": int(m.group(8)),
            "ms_p99": int(m.group(9)),
            "fps": int(m.group(10)),
        })
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--fps", type=int, action="append", default=None,
                    help="repeatable; defaults to 30,45,60")
    args = ap.parse_args()
    fps_list = args.fps if args.fps else [30, 45, 60]

    ser = serial.Serial(args.port, 115200, timeout=0.5)
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"\r\x03\x03\r\n")
    time.sleep(0.3)
    _drain(ser, 0.4)

    all_rows = {}  # fps -> list of mode rows
    for fps in fps_list:
        print("\n========== VGA%d sweep ==========" % fps)
        # Soft-reset board for a clean set_hw path.
        _reset(ser)
        # Wait for re-enum + REPL.
        ser = _wait_ready(ser, args.port, deadline_s=30)
        time.sleep(2)  # extra settle for camera power sequencing
        out = _run_bench(ser, fps, deadline_s=200)
        rows = _parse(out)
        all_rows[fps] = rows

    ser.close()

    # Aggregate table
    print("\n\n========== AGGREGATE: VGA{30,45,60} cam_id+timing ==========")
    print(
        "%-13s | %-3s | %3s/%-3s | %5s | %5s | %5s | %5s | %3s" %
        ("mode", "fps", "ok", "N", "scrm", "wrng", "ms_av", "ms_p99", "FPS")
    )
    for fps in fps_list:
        rows = all_rows.get(fps, [])
        if not rows:
            print("%-13s | %3d | (no data)" % ("--", fps))
            continue
        for r in rows:
            print(
                "%-13s | %3d | %3d/%-3d |  %3d  |  %3d  | %5d | %5d | %3d" %
                (r["mode"], fps, r["correct"], 100, r["scrambled"], r["wrong"],
                 r["ms_avg"], r["ms_p99"], r["fps"])
            )


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Run s234: unchanged diag/_t_timing.py timing recheck on real SentAI HW."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import serial


REPO = Path(__file__).resolve().parents[4]
EXP_DIR = Path(__file__).resolve().parent
RUNTIME_DIR = REPO / "examples" / "sentai_runtime"
PORT = "/dev/ttyACM0"
BAUD = 115200


def run_cmd(cmd: list[str], out: Path, cwd: Path = REPO, timeout: float | None = None) -> int:
    with out.open("w", encoding="utf-8", errors="replace") as f:
        p = subprocess.run(
            cmd,
            cwd=str(cwd),
            stdout=f,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
        )
    return p.returncode


def drain(ser: serial.Serial, quiet_s: float = 0.25) -> bytes:
    end = time.monotonic() + quiet_s
    buf = bytearray()
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
            end = time.monotonic() + quiet_s
        else:
            time.sleep(0.02)
    return bytes(buf)


def enter_repl(ser: serial.Serial) -> bytes:
    ser.write(b"\r\x03\x03")
    time.sleep(0.3)
    buf = bytearray(drain(ser, 0.4))
    ser.write(b"\r\n")
    time.sleep(0.2)
    buf.extend(drain(ser, 0.4))
    return bytes(buf)


def send_line(ser: serial.Serial, line: str, timeout_s: float = 5.0) -> bytes:
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(line.encode() + b"\r\n")
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(1024)
        if chunk:
            buf.extend(chunk)
            if b"\r\n>>> " in bytes(buf):
                break
        else:
            time.sleep(0.02)
    return bytes(buf)


def probe_board(out: Path) -> None:
    lines = []
    with serial.Serial(PORT, BAUD, timeout=0.2, rtscts=False, xonxoff=False, dsrdtr=False) as ser:
        lines.append(enter_repl(ser).decode(errors="replace"))
        for line in [
            "import sentai",
            "sentai.verbose(1)",
            "print('VERSION', sentai.version())",
            "print('TPU_CHUNK', sentai.diag.tpu_chunk_size())",
            "print('TPU_URB_TIMEOUT', sentai.diag.tpu_urb_timeout())",
            "print('TPU_DESC_CACHE', sentai.diag.tpu_desc_cache())",
        ]:
            lines.append(send_line(ser, line, 5.0).decode(errors="replace"))
    out.write_text("".join(lines), encoding="utf-8", errors="replace")


def run_driver(out: Path, timeout_s: float = 90.0) -> bool:
    saw_done = False
    with serial.Serial(PORT, BAUD, timeout=0.5, rtscts=False, xonxoff=False, dsrdtr=False) as ser:
        with out.open("w", encoding="utf-8", errors="replace") as f:
            f.write(enter_repl(ser).decode(errors="replace"))
            cmd = 'import sentai; exec(sentai.fs.read_str("/lib/diag/_t_timing.py"))\r\n'
            ser.write(cmd.encode())
            deadline = time.monotonic() + timeout_s
            acc = bytearray()
            while time.monotonic() < deadline:
                chunk = ser.read(4096)
                if chunk:
                    text = chunk.decode(errors="replace")
                    f.write(text)
                    f.flush()
                    acc.extend(chunk)
                    if b"=== done ===" in acc:
                        saw_done = True
                        f.write(drain(ser, 1.0).decode(errors="replace"))
                        break
                else:
                    time.sleep(0.05)
            if not saw_done:
                f.write("\n[host] timeout without === done ===\n")
    return saw_done


def parse_summary(run_log: Path) -> dict[str, object]:
    text = run_log.read_text(encoding="utf-8", errors="replace")
    summary: dict[str, object] = {
        "saw_pure": "=== PURE TPU variance" in text,
        "saw_pipeline": "=== PIPELINE variance" in text,
        "saw_done": "=== done ===" in text,
    }
    runs = []
    for m in re.finditer(
        r"run (\d+): total=([0-9.]+)ms\s+input=([0-9.]+)\s+params=([0-9.]+)\s+ins=([0-9.]+)\s+output=([0-9.]+)\s+event=([0-9.]+)",
        text,
    ):
        runs.append({
            "run": int(m.group(1)),
            "total_ms": float(m.group(2)),
            "input_ms": float(m.group(3)),
            "params_ms": float(m.group(4)),
            "ins_ms": float(m.group(5)),
            "output_ms": float(m.group(6)),
            "event_ms": float(m.group(7)),
        })
    summary["pure_runs"] = runs
    if runs:
        keys = ["total_ms", "input_ms", "params_ms", "ins_ms", "output_ms", "event_ms"]
        summary["pure_avg"] = {
            k: sum(float(r[k]) for r in runs) / len(runs)
            for k in keys
        }
    b = re.search(r"bytes/invoke: input=(\d+)B ins=(\d+)B output=(\d+)B", text)
    if b:
        summary["bytes_per_invoke"] = {
            "input": int(b.group(1)),
            "instructions": int(b.group(2)),
            "output": int(b.group(3)),
        }
    return summary


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iter", default="iter1")
    args = ap.parse_args()

    out_dir = EXP_DIR / args.iter
    out_dir.mkdir(parents=True, exist_ok=True)

    run_cmd(["lsusb"], out_dir / "lsusb_before.txt")
    run_cmd(["git", "status", "--short"], out_dir / "git_status.txt")
    probe_board(out_dir / "board_probe.log")

    upload_log = out_dir / "upload_t_timing.log"
    rc = run_cmd(
        ["python3", "diag/_host_upload_repl.py", "--file", "_t_timing.py"],
        upload_log,
        cwd=RUNTIME_DIR,
        timeout=120,
    )
    if rc != 0:
        print(f"upload failed; see {upload_log}", file=sys.stderr)
        return rc

    run_log = out_dir / "run_t_timing.log"
    ok = run_driver(run_log)
    summary = parse_summary(run_log)
    (out_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if ok and summary.get("saw_pure") and summary.get("saw_pipeline") else 2


if __name__ == "__main__":
    raise SystemExit(main())

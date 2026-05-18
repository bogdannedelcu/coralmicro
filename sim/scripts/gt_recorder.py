#!/usr/bin/env python3
"""gt_recorder.py — canonical Gazebo ground-truth pose recorder.

Subscribes to `/world/<world>/dynamic_pose/info` from inside the
`crazysim-garden` distrobox via `gz topic -e`, parses the protobuf-text
dump, and writes JSONL records of one model's pose tagged with HOST
monotonic time.  Sim-time (`sec`/`nsec`) is also included so verdicts
can choose either clock for joining with other recorders.

## Anti-cheat positioning

This tool runs HOST-SIDE ONLY and is the canonical mechanism for
post-mortem ground-truth comparison in this project.  Anti-cheat
invariants:

  - GT JSONL output is consumed only by verdict / plot scripts.
  - It is NEVER injected back into `sentai_sim`, the cf2 firmware,
    or any MicroPython binding.  Doing so would re-introduce the
    `[[cf2-sitl-cheat-odom-gt]]` class of fault.
  - The reader (verdict script) MUST tag what it does with the GT as
    "post-mortem / forensic" so a reviewer can audit at a glance.

Reference: `Sim.md` §10t / §10aa anti-cheat; `[[op-s8-w1-cf2-sim-honest]]`.

## Calling convention

CLI:
    python3 sim/scripts/gt_recorder.py [--world W] [--model M] [--out PATH]
                                       [--distrobox NAME]

Env (CLI flags take precedence):
    GT_RECORDER_WORLD      default sentai_crazysim
    GT_RECORDER_MODEL      default crazyflie_0
    GT_RECORDER_OUT        default /tmp/gt_recorder/gt_poses.jsonl
    GT_RECORDER_DISTROBOX  default crazysim-garden

Backwards-compat env aliases (recognised, lower priority than the
canonical names so legacy runners keep working):
    S165_GT_*, S166_GT_*, S167_GT_*  →  GT_RECORDER_*

## JSONL record shape (one per line)

    {"t_wall": <host monotonic float>,
     "t_unix": <host wall-clock float>,
     "gz_sec": <int>, "gz_nsec": <int>,
     "x": <float>, "y": <float>, "z": <float>}

Append-only; the recorder process holds the file open with line
buffering so an interrupted run still leaves the data flushed.

## Lifecycle / cleanup

The tool spawns `distrobox enter crazysim-garden -- gz topic -e ...`.
Killing the wrapper via SIGTERM forwards a `terminate()` to the
Popen, but distrobox + podman do not always propagate signals to the
container-side `gz topic` process — so the orchestrator that started
this recorder should ALSO run

    distrobox enter crazysim-garden -- pkill -9 -f \
        "gz topic -e -t /world/<world>/dynamic_pose"

after the trial to make sure no orphan subscriber is left running.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time


def _env_chain(*names: str, default: str) -> str:
    """Return the first defined env var from names, else default."""
    for n in names:
        v = os.environ.get(n)
        if v is not None:
            return v
    return default


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--world",
                    default=_env_chain("GT_RECORDER_WORLD",
                                       "S165_GT_WORLD",
                                       "S166_GT_WORLD",
                                       "S167_GT_WORLD",
                                       default="sentai_crazysim"))
    ap.add_argument("--model",
                    default=_env_chain("GT_RECORDER_MODEL",
                                       "S165_GT_MODEL",
                                       "S166_GT_MODEL",
                                       "S167_GT_MODEL",
                                       default="crazyflie_0"))
    ap.add_argument("--out",
                    default=_env_chain("GT_RECORDER_OUT",
                                       "S165_GT_OUT",
                                       "S166_GT_OUT",
                                       "S167_GT_OUT",
                                       default="/tmp/gt_recorder/gt_poses.jsonl"))
    ap.add_argument("--distrobox",
                    default=_env_chain("GT_RECORDER_DISTROBOX",
                                       default="crazysim-garden"))
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    topic = f"/world/{args.world}/dynamic_pose/info"
    cmd = ["distrobox", "enter", args.distrobox, "--",
           "gz", "topic", "-e", "-t", topic]
    sys.stderr.write(
        f"[gt_recorder] world={args.world} model={args.model} "
        f"distrobox={args.distrobox}\n"
        f"[gt_recorder] subscribing to {topic}\n"
        f"[gt_recorder] out → {args.out}\n")

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             text=True, bufsize=1)

    def _sigterm(_sig, _frm):
        try:
            proc.terminate()
        except Exception:
            pass
    signal.signal(signal.SIGINT,  _sigterm)
    signal.signal(signal.SIGTERM, _sigterm)

    rx_name = re.compile(r'^\s*name:\s*"([^"]+)"')
    rx_x    = re.compile(r'^\s*x:\s*([-\d.eE+]+)')
    rx_y    = re.compile(r'^\s*y:\s*([-\d.eE+]+)')
    rx_z    = re.compile(r'^\s*z:\s*([-\d.eE+]+)')
    rx_w    = re.compile(r'^\s*w:\s*([-\d.eE+]+)')  # quaternion w
    rx_sec  = re.compile(r'^\s*sec:\s*(\d+)')
    rx_nsec = re.compile(r'^\s*nsec:\s*(\d+)')

    in_position    = False
    in_orientation = False     # OP-S10-W14-T13 — capture quaternion
    saw_target     = False
    gz_sec         = None
    gz_nsec        = None
    record: dict   = {}
    n_written      = 0

    with open(args.out, "w", buffering=1) as fp:
        for line in proc.stdout:
            line = line.rstrip("\n")
            m = rx_sec.match(line)
            if m and not in_position:
                gz_sec = int(m.group(1))
                continue
            m = rx_nsec.match(line)
            if m and not in_position:
                gz_nsec = int(m.group(1))
                continue
            m = rx_name.match(line)
            if m:
                saw_target  = (m.group(1) == args.model)
                in_position = False
                record = {}
                continue
            if saw_target and line.strip().startswith("position {"):
                in_position    = True
                in_orientation = False
                continue
            if saw_target and line.strip().startswith("orientation {"):
                in_orientation = True
                in_position    = False
                continue
            if in_position:
                for rx, key in ((rx_x, "x"), (rx_y, "y"), (rx_z, "z")):
                    m = rx.match(line)
                    if m:
                        record[key] = float(m.group(1))
                        break
                if line.strip() == "}":
                    in_position = False
            elif in_orientation:
                for rx, key in ((rx_x, "qx"), (rx_y, "qy"),
                                 (rx_z, "qz"), (rx_w, "qw")):
                    m = rx.match(line)
                    if m:
                        record[key] = float(m.group(1))
                        break
                if line.strip() == "}":
                    in_orientation = False
                    # Pose complete (position + orientation) — derive
                    # yaw and emit one record per pose update.
                    if all(k in record for k in ("x", "y", "z",
                                                   "qx","qy","qz","qw")):
                        import math as _math
                        qx, qy = record["qx"], record["qy"]
                        qz, qw = record["qz"], record["qw"]
                        yaw_rad = _math.atan2(
                            2.0 * (qw * qz + qx * qy),
                            1.0 - 2.0 * (qy * qy + qz * qz))
                        yaw_deg = yaw_rad * 180.0 / _math.pi
                        rec = {
                            "t_wall":  time.monotonic(),
                            "t_unix":  time.time(),
                            "gz_sec":  gz_sec,
                            "gz_nsec": gz_nsec,
                            "x": record["x"],
                            "y": record["y"],
                            "z": record["z"],
                            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
                            "yaw_deg": yaw_deg,
                        }
                        fp.write(json.dumps(rec) + "\n")
                        n_written += 1
                    saw_target = False
                    record = {}
    proc.wait(timeout=2.0)
    sys.stderr.write(f"[gt_recorder] wrote {n_written} records → {args.out}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

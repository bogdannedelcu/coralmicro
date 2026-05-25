#!/usr/bin/env python3
"""s166 — Gazebo ground-truth recorder for cf2 (post-cheat baseline).

Subscribes to `/world/<world>/dynamic_pose/info` via the distrobox
`gz topic -e` CLI, parses the protobuf-text dump, writes JSON-lines
records of cf2 pose tagged with HOST monotonic time so verdict can
join EKF samples and GT samples on a shared clock.

Anti-cheat note ([[sentai-sim-air-gapped-from-truth]] +
[[cf2-sitl-cheat-odom-gt]]): runs HOST-SIDE ONLY for post-mortem
comparison.  GT never crosses into sentai_sim or the cf2 firmware.

Adapted from s165/gt_recorder.py — same parser, S166_* env vars.

Env:
  S166_GT_MODEL  (default: crazyflie_0)
  S166_GT_WORLD  (default: sentai_crazysim)
  S166_GT_OUT    (default: /tmp/s166_flowbaseline_gt/gt_poses.jsonl)
"""
from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import sys
import time


def main() -> int:
    model    = os.environ.get("S166_GT_MODEL", "crazyflie_0")
    world    = os.environ.get("S166_GT_WORLD", "sentai_crazysim")
    out_path = os.environ.get("S166_GT_OUT",
                              "/tmp/s166_flowbaseline_gt/gt_poses.jsonl")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    topic = f"/world/{world}/dynamic_pose/info"
    cmd = ["distrobox", "enter", "crazysim-garden", "--",
           "gz", "topic", "-e", "-t", topic]

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
    rx_sec  = re.compile(r'^\s*sec:\s*(\d+)')
    rx_nsec = re.compile(r'^\s*nsec:\s*(\d+)')

    in_position = False
    saw_target  = False
    gz_sec      = None
    gz_nsec     = None
    record      = {}
    n_written   = 0

    with open(out_path, "w", buffering=1) as fp:
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
                saw_target  = (m.group(1) == model)
                in_position = False
                record = {}
                continue
            if saw_target and line.strip().startswith("position {"):
                in_position = True
                record = {}
                continue
            if in_position:
                for rx, key in ((rx_x, "x"), (rx_y, "y"), (rx_z, "z")):
                    m = rx.match(line)
                    if m:
                        record[key] = float(m.group(1))
                        break
                if line.strip() == "}":
                    if all(k in record for k in ("x", "y", "z")):
                        rec = {
                            "t_wall":  time.monotonic(),
                            "t_unix":  time.time(),
                            "gz_sec":  gz_sec,
                            "gz_nsec": gz_nsec,
                            "x": record["x"],
                            "y": record["y"],
                            "z": record["z"],
                        }
                        fp.write(json.dumps(rec) + "\n")
                        n_written += 1
                    in_position = False
                    saw_target  = False
                    record = {}
    proc.wait(timeout=2.0)
    sys.stderr.write(f"[gt_recorder] wrote {n_written} records → {out_path}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

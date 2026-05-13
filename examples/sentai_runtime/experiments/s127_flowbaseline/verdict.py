#!/usr/bin/env python3
"""verdict.py — read hover_log.json from s091_aruco_lowalt and decide PASS/FAIL.

Pass criteria (per s127 README):
  dist_mean_m < 0.15  AND  all4_rate >= 0.5  AND  n_samples >= 5

Exit 0 on PASS, 1 on FAIL.  Prints a one-line summary line then the verdict.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

LOG = Path(__file__).resolve().parents[1] / "s091_aruco_lowalt" / "hover_log.json"

DIST_MEAN_MAX = 0.15
ALL4_MIN      = 0.50
N_SAMPLES_MIN = 5


def main() -> int:
    if not LOG.is_file():
        print(f"FAIL — no hover_log.json at {LOG}")
        return 1

    d = json.loads(LOG.read_text())
    dist = d.get("dist_mean_m")
    a4   = d.get("all4_rate")
    n    = d.get("n_samples")
    flow_n  = d.get("flow_n")
    flow_hz = d.get("flow_hz")
    z_mean  = d.get("z_mean_cm")
    ts      = d.get("_last_run")

    print(f"[verdict] hover_log.json _last_run={ts}")
    print(f"          dist_mean_m={dist}  all4_rate={a4}  n_samples={n}")
    print(f"          z_mean_cm={z_mean}  flow_n={flow_n}  flow_hz={flow_hz}")

    fails = []
    if dist is None or dist >= DIST_MEAN_MAX:
        fails.append(f"dist_mean_m={dist} >= {DIST_MEAN_MAX}")
    if a4 is None or a4 < ALL4_MIN:
        fails.append(f"all4_rate={a4} < {ALL4_MIN}")
    if n is None or n < N_SAMPLES_MIN:
        fails.append(f"n_samples={n} < {N_SAMPLES_MIN}")

    if fails:
        print(f"FAIL — {'; '.join(fails)}")
        return 1
    print("PASS — sentai.flow baseline holds vs s091 #14")
    return 0


if __name__ == "__main__":
    sys.exit(main())

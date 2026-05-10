#!/usr/bin/env python3
"""
analyze_drift.py — compare RMS X/Y drift between no_flow and with_flow runs.

Pass criterion (printed at the end):
    rms_x(with_flow) < 0.5 * rms_x(no_flow)
    rms_y(with_flow) < 0.5 * rms_y(no_flow)
"""
import csv
import math
import sys
from pathlib import Path

WINDOW_START_S = 10.0   # skip takeoff transient
WINDOW_END_S   = 30.0   # exclude landing


def load(path: Path) -> list[tuple[float, float, float, float]]:
    rows = []
    with open(path) as f:
        r = csv.reader(f)
        next(r)  # header
        for ts, x, y, z in r:
            rows.append((float(ts) / 1000.0, float(x), float(y), float(z)))
    if not rows:
        return rows
    t0 = rows[0][0]
    return [(t - t0, x, y, z) for (t, x, y, z) in rows]


def rms(samples: list[float]) -> float:
    if not samples: return 0.0
    return math.sqrt(sum(s * s for s in samples) / len(samples))


def report(label: str, path: Path) -> tuple[float, float]:
    rows = load(path)
    if not rows:
        print(f"[{label}] EMPTY ({path})")
        return (0.0, 0.0)
    win = [(t, x, y, z) for (t, x, y, z) in rows
           if WINDOW_START_S <= t <= WINDOW_END_S]
    rx = rms([x for (_, x, _, _) in win])
    ry = rms([y for (_, _, y, _) in win])
    rz = rms([z - 1.0 for (_, _, _, z) in win])  # error vs target z=1
    print(f"[{label}] n={len(win)}  rms_x={rx:.3f} m  rms_y={ry:.3f} m  "
          f"rms_z_err={rz:.3f} m")
    return (rx, ry)


def main():
    if len(sys.argv) != 3:
        print("usage: analyze_drift.py csv/no_flow.csv csv/with_flow.csv")
        return 2
    no_path   = Path(sys.argv[1])
    with_path = Path(sys.argv[2])

    rx_n, ry_n = report("no_flow",   no_path)
    rx_w, ry_w = report("with_flow", with_path)

    print()
    print("=== DRIFT REDUCTION ===")
    if rx_n > 0 and ry_n > 0:
        red_x = 100.0 * (1.0 - rx_w / rx_n)
        red_y = 100.0 * (1.0 - ry_w / ry_n)
        print(f"  X drift reduced by {red_x:+.1f}%   "
              f"({rx_n:.3f} -> {rx_w:.3f} m)")
        print(f"  Y drift reduced by {red_y:+.1f}%   "
              f"({ry_n:.3f} -> {ry_w:.3f} m)")
    else:
        print("  no_flow run too short for ratio")

    pass_x = rx_w < 0.5 * rx_n
    pass_y = ry_w < 0.5 * ry_n
    print()
    print(f"  Pass criterion X (with < 0.5 * no): {'PASS' if pass_x else 'FAIL'}")
    print(f"  Pass criterion Y (with < 0.5 * no): {'PASS' if pass_y else 'FAIL'}")
    return 0 if (pass_x and pass_y) else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""s185 verdict -- OP-S10-W19-T7 yaw smoke (Gazebo mission-level).

Parses the per-tick journal produced by mission_s185.py, pairs each
tick's cf2_yaw with the picker's recovered yaw, computes MAE / max
error / mirror-flip rate, and emits paired CSV + a yaw-error plot.

Pass criteria (see README.md): yaw MAE <= 5 deg, flip rate < 1%.
This first-cut tool just prints the numbers; treat as informational
until the tolerance budget is locked in.
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path


TICK_RE = re.compile(r'^\d+\s+tick\s+(\{.*\})\s*$')


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def parse_journal(journal_path):
    """Yield each {"phase","k","yaw_cmd","cf2","n","pose"} payload."""
    if not journal_path.exists():
        return
    for line in journal_path.read_text(errors="replace").splitlines():
        m = TICK_RE.match(line)
        if not m:
            continue
        try:
            payload = json.loads(m.group(1))
        except json.JSONDecodeError:
            continue
        yield payload


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--journal", required=True, type=Path)
    ap.add_argument("--gt",      required=False, type=Path)
    ap.add_argument("--out-dir", required=True, type=Path)
    args = ap.parse_args()

    rows = []     # (phase, k, cf2_yaw, recovered_yaw, yaw_err_rad, n, flips, pos_err)
    n_total      = 0
    n_with_pose  = 0
    n_flip_xy    = 0

    for tick in parse_journal(args.journal):
        n_total += 1
        cf2 = tick.get("cf2")
        if cf2 is None:
            continue
        cf2_x, cf2_y, cf2_z, cf2_yaw = cf2
        pose = tick.get("pose")
        if pose is None:
            continue
        n_with_pose += 1
        x, y, z, yaw_est, res_max, n_used, flip_x, flip_y, flip_z = pose
        yaw_err = wrap_pi(yaw_est - cf2_yaw)
        if flip_x or flip_y:
            n_flip_xy += 1
        pos_err = math.sqrt((x - cf2_x) ** 2 +
                             (y - cf2_y) ** 2 +
                             (z - cf2_z) ** 2)
        rows.append((
            tick.get("phase", "?"),
            tick.get("k", -1),
            cf2_yaw, yaw_est, yaw_err,
            n_used, flip_x, flip_y, flip_z,
            pos_err,
        ))

    args.out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.out_dir / "s185_yaw_paired.csv"
    with csv_path.open("w") as f:
        f.write("phase,k,cf2_yaw_rad,est_yaw_rad,yaw_err_rad,"
                "n_used,flip_x,flip_y,flip_z,pos_err_m\n")
        for r in rows:
            f.write(",".join(str(v) for v in r) + "\n")

    sep = "=" * 60
    print(sep)
    print("s185 verdict -- OP-S10-W19-T7 yaw smoke (Gazebo)")
    print(sep)
    print(f"  Ticks in journal       : {n_total}")
    print(f"  Ticks with pose        : {n_with_pose}")
    if not rows:
        print(f"  VERDICT: NO_DATA -- picker never fired")
        sys.exit(1)

    abs_errs = [abs(r[4]) for r in rows]
    mae_deg  = math.degrees(sum(abs_errs) / len(abs_errs))
    max_deg  = math.degrees(max(abs_errs))
    flip_rate = n_flip_xy / n_with_pose

    pos_errs = [r[9] for r in rows]
    pos_mae  = sum(pos_errs) / len(pos_errs)
    pos_max  = max(pos_errs)

    print(f"  Yaw MAE                : {mae_deg:.2f} deg")
    print(f"  Yaw max abs error      : {max_deg:.2f} deg")
    print(f"  Mirror flip rate (X/Y) : {flip_rate*100:.2f} %")
    print(f"  Position MAE           : {pos_mae*100:.2f} cm")
    print(f"  Position max abs error : {pos_max*100:.2f} cm")
    print(f"  Per-tick CSV           : {csv_path}")

    # Provisional pass criteria from README.
    pass_yaw  = mae_deg  <= 5.0
    pass_flip = flip_rate < 0.01
    pass_pos  = pos_mae  <= 0.03

    if pass_yaw and pass_flip and pass_pos:
        print(f"  VERDICT: PASS (yaw <= 5deg, flip < 1%, pos <= 3cm)")
    else:
        why = []
        if not pass_yaw:  why.append(f"yaw_MAE={mae_deg:.2f} > 5")
        if not pass_flip: why.append(f"flip={flip_rate*100:.2f} >= 1%")
        if not pass_pos:  why.append(f"pos_MAE={pos_mae*100:.2f} > 3cm")
        print(f"  VERDICT: FAIL ({', '.join(why)})")
        sys.exit(2)


if __name__ == "__main__":
    main()

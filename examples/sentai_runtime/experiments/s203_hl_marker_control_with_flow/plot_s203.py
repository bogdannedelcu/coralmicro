#!/usr/bin/env python3
"""Post-mortem plots for s203.

GT is forensic only.  This script reads artifacts after the mission completed
and writes two plots:

- s203_xy_gt_vs_est.png
- s203_z_gt_vs_est.png
"""

from __future__ import annotations

import ast
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load_gt(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8", errors="replace") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return rows


def load_est_from_journal(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8", errors="replace") as fp:
        for line in fp:
            parts = line.split(" ", 2)
            if len(parts) < 3:
                continue
            try:
                t = float(parts[0]) / 1000.0
                data = ast.literal_eval(parts[2])
            except Exception:
                continue
            if not isinstance(data, dict):
                continue
            est = data.get("est_pose") or data.get("last_est_pose")
            if est is None or len(est) < 3:
                continue
            try:
                rows.append({
                    "t": t,
                    "x": float(est[0]),
                    "y": float(est[1]),
                    "z": float(est[2]),
                    "event": parts[1],
                })
            except Exception:
                pass
    return rows


def rel_time(vals: list[float]) -> list[float]:
    if not vals:
        return []
    t0 = vals[0]
    return [v - t0 for v in vals]


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: plot_s203.py ITER_DIR", file=sys.stderr)
        return 2
    workdir = Path(argv[1])
    gt = load_gt(workdir / "fr_current" / "gt.jsonl")
    est = load_est_from_journal(workdir / "mission_s203_journal.txt")
    if not gt:
        print("no GT rows; plots skipped")
        return 1
    if not est:
        print("no estimator rows; plots skipped")
        return 1

    gt_x = [float(r["x"]) for r in gt if all(k in r for k in ("x", "y", "z", "t_wall"))]
    gt_y = [float(r["y"]) for r in gt if all(k in r for k in ("x", "y", "z", "t_wall"))]
    gt_z = [float(r["z"]) for r in gt if all(k in r for k in ("x", "y", "z", "t_wall"))]
    gt_t_abs = [float(r["t_wall"]) for r in gt if all(k in r for k in ("x", "y", "z", "t_wall"))]
    est_x = [r["x"] for r in est]
    est_y = [r["y"] for r in est]
    est_z = [r["z"] for r in est]
    est_t_abs = [r["t"] for r in est]

    plt.figure(figsize=(7, 7))
    plt.plot(gt_x, gt_y, "-", lw=1.2, label="GT xy")
    plt.plot(est_x, est_y, ".-", lw=0.8, ms=3, label="CF estimator xy")
    plt.scatter([gt_x[0]], [gt_y[0]], c="black", s=25, label="GT start")
    plt.scatter([gt_x[-1]], [gt_y[-1]], c="tab:green", s=25, label="GT end")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("s203 XY post-mortem: GT vs CF estimator")
    plt.legend()
    xy_path = workdir / "s203_xy_gt_vs_est.png"
    plt.tight_layout()
    plt.savefig(xy_path, dpi=140)
    plt.close()

    plt.figure(figsize=(9, 4.8))
    plt.plot(rel_time(gt_t_abs), gt_z, "-", lw=1.2, label="GT z")
    plt.plot(rel_time(est_t_abs), est_z, ".-", lw=0.8, ms=3, label="CF estimator z")
    plt.grid(True, alpha=0.3)
    plt.xlabel("time [s, relative]")
    plt.ylabel("Z [m]")
    plt.title("s203 Z post-mortem: GT vs CF estimator")
    plt.legend()
    z_path = workdir / "s203_z_gt_vs_est.png"
    plt.tight_layout()
    plt.savefig(z_path, dpi=140)
    plt.close()

    print(f"wrote {xy_path}")
    print(f"wrote {z_path}")
    print(f"gt_rows={len(gt)} est_rows={len(est)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

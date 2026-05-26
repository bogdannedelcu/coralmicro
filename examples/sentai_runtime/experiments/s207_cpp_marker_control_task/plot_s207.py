#!/usr/bin/env python3
"""Post-mortem plots for s207.

GT is forensic only.  This script reads sentai.fr artifacts after the mission
completed and writes two plots:

- s207_xy_gt_vs_est.png
- s207_z_gt_vs_est.png
"""

from __future__ import annotations

import csv
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


def load_scalar_series(path: Path, label: str) -> list[tuple[float, float]]:
    rows: list[tuple[float, float]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8", errors="replace", newline="") as fp:
        reader = csv.reader(fp)
        for row in reader:
            if len(row) < 3 or row[0].startswith("#") or row[1] != label:
                continue
            try:
                rows.append((float(row[0]) / 1000.0, float(row[2])))
            except ValueError:
                pass
    return rows


def rel_time(vals: list[float]) -> list[float]:
    if not vals:
        return []
    t0 = vals[0]
    return [v - t0 for v in vals]


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: plot_s207.py ITER_DIR", file=sys.stderr)
        return 2
    workdir = Path(argv[1])
    gt = load_gt(workdir / "fr_current" / "gt.jsonl")
    est_xs = load_scalar_series(workdir / "fr_current" / "scalars.csv", "marker_est_x")
    est_ys = load_scalar_series(workdir / "fr_current" / "scalars.csv", "marker_est_y")
    est_zs = load_scalar_series(workdir / "fr_current" / "scalars.csv", "marker_est_z")
    if not gt:
        print("no GT rows; plots skipped")
        return 1
    n_est = min(len(est_xs), len(est_ys), len(est_zs))
    if n_est <= 0:
        print("no estimator scalar rows; plots skipped")
        return 1

    gt_rows = [r for r in gt if all(k in r for k in ("x", "y", "z", "t_wall"))]
    if not gt_rows:
        print("no usable GT rows; plots skipped")
        return 1
    gt_x = [float(r["x"]) for r in gt_rows]
    gt_y = [float(r["y"]) for r in gt_rows]
    gt_z = [float(r["z"]) for r in gt_rows]
    gt_t_abs = [float(r["t_wall"]) for r in gt_rows]
    t0 = gt_t_abs[0]
    gt_t = [t - t0 for t in gt_t_abs]

    est_t = [est_zs[i][0] - t0 for i in range(n_est)]
    valid_est = [
        i for i in range(n_est)
        if est_zs[i][1] > 0.2
    ]
    if not valid_est:
        print("no usable estimator rows; plots skipped")
        return 1
    est_t = [est_t[i] for i in valid_est]
    est_x = [est_xs[i][1] for i in valid_est]
    est_y = [est_ys[i][1] for i in valid_est]
    est_z = [est_zs[i][1] for i in valid_est]

    plt.figure(figsize=(7, 7))
    plt.plot(gt_x, gt_y, "-", lw=1.2, label="GT xy")
    plt.plot(est_x, est_y, ".-", lw=0.8, ms=3, label="CF estimator xy")
    plt.scatter([gt_x[0]], [gt_y[0]], c="black", s=25, label="GT start")
    plt.scatter([gt_x[-1]], [gt_y[-1]], c="tab:green", s=25, label="GT end")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("s207 XY post-mortem: GT vs CF estimator")
    plt.legend()
    xy_path = workdir / "s207_xy_gt_vs_est.png"
    plt.tight_layout()
    plt.savefig(xy_path, dpi=140)
    plt.close()

    plt.figure(figsize=(9, 4.8))
    plt.plot(gt_t, gt_z, "-", lw=1.2, label="GT z")
    plt.plot(est_t, est_z, ".-", lw=0.8, ms=3, label="CF estimator z")
    plt.grid(True, alpha=0.3)
    plt.xlabel("time [s, relative]")
    plt.ylabel("Z [m]")
    plt.title("s207 Z post-mortem: GT vs CF estimator")
    plt.legend()
    z_path = workdir / "s207_z_gt_vs_est.png"
    plt.tight_layout()
    plt.savefig(z_path, dpi=140)
    plt.close()

    print(f"wrote {xy_path}")
    print(f"wrote {z_path}")
    print(f"gt_rows={len(gt_rows)} est_rows={len(valid_est)}")
    print("time_alignment=host_monotonic")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

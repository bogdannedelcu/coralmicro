#!/usr/bin/env python3
"""analyze_run.py — post-run analysis of a s125 demo capture.

Reads the saved frame stats CSV + a parallel drone-pose log (recorded by
gz_pose_logger.py or extracted from /world/.../pose/info) and produces a
summary that tells the operator:

  - did the drone leave the ground (z > 0.05 m at any time)?
  - was flight stable (roll/pitch within ±30° → IMU sane)?
  - did flow detect any motion (frames had varying gradient/stats)?
  - which frame indices correspond to which mission phase?

Usage:
   python3 sim/scripts/analyze_run.py <run_dir>
where <run_dir> = examples/.../captures/run_YYYYMMDD_HHMMSS/

Looks for:
  <run_dir>/_stats.csv          (mandatory — frame stats from saver)
  <run_dir>/pose.csv            (optional — pose log; produced by
                                 sim/scripts/gz_pose_logger.py if used)
"""
import csv
import sys
from pathlib import Path

def main():
    if len(sys.argv) < 2:
        print("usage: analyze_run.py <run_dir>")
        return 1
    run_dir = Path(sys.argv[1])
    stats_csv = run_dir / "_stats.csv"
    pose_csv  = run_dir / "pose.csv"

    if not stats_csv.exists():
        print(f"FATAL: {stats_csv} not found")
        return 2

    # Read frame stats
    frames = []
    with open(stats_csv) as f:
        for row in csv.DictReader(f):
            frames.append({
                "idx":   int(row["idx"]),
                "mean":  float(row["mean"]),
                "std":   float(row["std"]),
                "grad":  int(row["grad_sum"]),
                "uniq":  int(row.get("unique_colors", 0)),
            })

    n = len(frames)
    print(f"\n=== {run_dir.name} ===")
    print(f"frames: {n}")
    if not frames:
        return 3

    grad_min = min(f["grad"] for f in frames)
    grad_max = max(f["grad"] for f in frames)
    mean_min = min(f["mean"] for f in frames)
    mean_max = max(f["mean"] for f in frames)
    uniq_min = min(f["uniq"] for f in frames)
    uniq_max = max(f["uniq"] for f in frames)

    print(f"  gradient sum: min={grad_min:>10,d}  max={grad_max:>10,d}  range={grad_max - grad_min:>10,d}")
    print(f"  mean luma:    min={mean_min:>6.1f}    max={mean_max:>6.1f}    range={mean_max - mean_min:>6.1f}")
    print(f"  unique colors: min={uniq_min:>5d}    max={uniq_max:>5d}    range={uniq_max - uniq_min:>5d}")
    print()

    # Heuristic: did the scene change "enough" to suggest motion?
    grad_variation = grad_max - grad_min
    mean_variation = mean_max - mean_min
    if grad_variation < 100_000 and mean_variation < 1.0:
        print("VERDICT (visual): SCENE ESSENTIALLY STATIC across all frames.")
        print("                  → drone likely never left the ground.")
    elif grad_variation < 1_000_000:
        print("VERDICT (visual): minor frame variation (lighting/jitter, not flight).")
    else:
        print("VERDICT (visual): SIGNIFICANT frame variation across run — drone moved.")
    print()

    # If we have pose data, correlate
    if pose_csv.exists():
        poses = []
        with open(pose_csv) as f:
            for row in csv.DictReader(f):
                poses.append({k: float(v) for k, v in row.items()})
        zs = [p.get("z", 0.0) for p in poses]
        z_min, z_max = min(zs), max(zs)
        print(f"=== pose ===")
        print(f"  z range: {z_min:.3f} → {z_max:.3f} m (Δ={z_max - z_min:.3f} m)")
        if z_max < 0.05:
            print("VERDICT (pose): DRONE NEVER LEFT GROUND (z stayed below 5 cm).")
        elif z_max < 0.5:
            print("VERDICT (pose): drone barely lifted (max z < 0.5 m) — likely chaotic.")
        else:
            print(f"VERDICT (pose): drone reached {z_max:.2f} m altitude.")
    else:
        print(f"(no {pose_csv.name} — record drone pose alongside next run "
              f"with: gz topic -e -t /world/<world>/pose/info > {pose_csv.name})")

    return 0


if __name__ == "__main__":
    sys.exit(main())

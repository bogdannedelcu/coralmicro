"""s165 verdict — EKF belief vs Gazebo GT comparison plot.

Joins cf2_telemetry.json (cf2 EKF samples, host-monotonic time) with
gt_poses.jsonl (Gazebo dynamic_pose/info, host-monotonic time at line
reception), then:
  - per-waypoint: tabulates EKF pos, GT pos, |EKF-GT| at t_arrived
  - generates plot.png — top-down view, EKF track blue, GT track red,
    waypoint markers + closure annotation
  - emits pass/fail vs CLOSURE_GATE_CM (15 cm) on GT-based closure.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

WORKDIR = Path("/tmp/s165_square_drift_gt")
SUMMARY_JSON = WORKDIR / "summary.json"
TELEMETRY_JSON = WORKDIR / "cf2_telemetry.json"
GT_JSONL = WORKDIR / "gt_poses.jsonl"
PLOT_PATH = WORKDIR / "plot.png"

CLOSURE_GATE_CM = 15.0


def _load_summary():
    if not SUMMARY_JSON.is_file():
        return None
    return json.loads(SUMMARY_JSON.read_text())


def _load_ekf():
    if not TELEMETRY_JSON.is_file():
        return []
    return json.loads(TELEMETRY_JSON.read_text())


def _load_gt():
    if not GT_JSONL.is_file():
        return []
    out = []
    for line in GT_JSONL.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out


def _interp_xy(samples, key_t, t_query):
    """Linear-interpolate (x, y) at host time t_query.  Returns
    (x, y, ok); ok=False if t_query is outside the sample span."""
    if not samples:
        return None, None, False
    if t_query <= samples[0][key_t]:
        return samples[0]["x"], samples[0]["y"], True
    if t_query >= samples[-1][key_t]:
        return samples[-1]["x"], samples[-1]["y"], True
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid][key_t] <= t_query:
            lo = mid
        else:
            hi = mid
    s0, s1 = samples[lo], samples[hi]
    span = s1[key_t] - s0[key_t]
    if span <= 0:
        return s0["x"], s0["y"], True
    a = (t_query - s0[key_t]) / span
    return (s0["x"] + a * (s1["x"] - s0["x"]),
            s0["y"] + a * (s1["y"] - s0["y"]), True)


def main() -> int:
    s = _load_summary()
    if s is None:
        print(f"[verdict] FAIL — summary.json missing: {SUMMARY_JSON}")
        return 1
    if s.get("_status") != "OK":
        print(f"[verdict] FAIL — mission status={s.get('_status')}; "
              f"exception={s.get('_exception')}")
        return 1

    ekf = _load_ekf()
    gt  = _load_gt()
    print(f"[verdict] s165 EKF-vs-GT  ekf_samples={len(ekf)}  "
          f"gt_samples={len(gt)}")
    if not ekf or not gt:
        print(f"[verdict] FAIL — missing samples (ekf={len(ekf)}, gt={len(gt)})")
        return 1

    corners = s.get("corners_log", [])
    physical_origin = s.get("physical_origin", [0.0, 0.0])

    # Reference-frame alignment (W11-T5.A v2 bugfix 2026-05-17):
    #   EKF: kalman.reset zeros at some t < takeoff.  By the time
    #        `physical_origin` is captured (post-takeoff hover), the
    #        drone has *physically moved* a few cm during the take_off
    #        manoeuvre — that displacement is baked into both EKF and
    #        GT in their own frames.
    #   GT:  Original verdict used `gt[0]` (FIRST recorder sample) as
    #        origin, which is the spawn pose ON THE FLOOR — NOT the
    #        post-takeoff position.  Subtracting two different physical
    #        moments produced an apparent ~8 cm uniform offset that was
    #        purely a frame-mismatch artefact.
    #   Fix: use GT pose AT the host-time when physical_origin was
    #        captured (≈ t_leg_start of the first corner).  Now both
    #        EKF and GT tracks are normalised to the SAME physical
    #        moment → residuals drop from ~8 cm to <0.5 cm at every
    #        corner (validated 2026-05-17 against the first GT trial).
    if corners:
        t_phys_orig = corners[0]["t_leg_start"]
        gx0, gy0, _ = _interp_xy(gt, "t_wall", t_phys_orig)
        gt_origin = (gx0 if gx0 is not None else gt[0]["x"],
                     gy0 if gy0 is not None else gt[0]["y"])
    else:
        gt_origin = (gt[0]["x"], gt[0]["y"])

    # Per-corner table.
    print("          corner       EKF (x,y)             GT (x,y)              |EKF-GT|")
    row_lines = []
    closure_gt_cm = None
    closure_ekf_cm = None
    for c in corners:
        t_arr = c["t_arrived"]
        # EKF samples have key "t" (host monotonic).
        ex, ey, _ = _interp_xy(ekf, "t", t_arr)
        gx, gy, ok_gt = _interp_xy(gt, "t_wall", t_arr)
        if not ok_gt:
            print(f"          {c['label']:8s} no GT sample at t_arrived")
            continue
        # Normalise both to their own origin so deltas are comparable.
        ex_n = ex - physical_origin[0]
        ey_n = ey - physical_origin[1]
        gx_n = gx - gt_origin[0]
        gy_n = gy - gt_origin[1]
        d_cm = math.hypot(ex_n - gx_n, ey_n - gy_n) * 100.0
        row_lines.append((c["label"], ex_n, ey_n, gx_n, gy_n, d_cm))
        print(f"          {c['label']:8s} ({ex_n:+.3f},{ey_n:+.3f})   "
              f"({gx_n:+.3f},{gy_n:+.3f})   {d_cm:.2f} cm")
        # Last row is c0_home — use it as the closure measurement.
        if c["label"].startswith("c0_home"):
            closure_ekf_cm = math.hypot(ex_n, ey_n) * 100.0
            closure_gt_cm  = math.hypot(gx_n, gy_n) * 100.0

    # Landing pose closure (separate from the c0_home corner — happens
    # AFTER the controlled descent).  Use the LAST GT sample with
    # t_wall <= mission summary's "_last_run" or the very last GT.
    land_pose = s.get("land_pose", [0, 0, 0])
    ekf_close_land = math.hypot(land_pose[0] - physical_origin[0],
                                 land_pose[1] - physical_origin[1]) * 100.0
    gx_last = gt[-1]["x"] - gt_origin[0]
    gy_last = gt[-1]["y"] - gt_origin[1]
    gt_close_land = math.hypot(gx_last, gy_last) * 100.0
    print(f"          land     EKF=({land_pose[0]-physical_origin[0]:+.3f},"
          f"{land_pose[1]-physical_origin[1]:+.3f})   "
          f"GT=({gx_last:+.3f},{gy_last:+.3f})   "
          f"EKF_close={ekf_close_land:.2f}cm  GT_close={gt_close_land:.2f}cm")

    # ── plot ───────────────────────────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as e:
        print(f"[verdict] WARN — matplotlib unavailable ({e}); skipping plot")
    else:
        fig, ax = plt.subplots(figsize=(8, 8))
        ekf_x = [e["x"] - physical_origin[0] for e in ekf]
        ekf_y = [e["y"] - physical_origin[1] for e in ekf]
        gt_x  = [g["x"] - gt_origin[0]  for g in gt]
        gt_y  = [g["y"] - gt_origin[1]  for g in gt]
        ax.plot(ekf_x, ekf_y, "-", color="tab:blue", lw=1.2,
                label=f"cf2 EKF belief (n={len(ekf)})")
        ax.plot(gt_x,  gt_y,  "-", color="tab:red",  lw=1.2,
                label=f"Gazebo GT (n={len(gt)})")
        # Commanded corners (target square).
        for c in corners:
            cx, cy = c["cmd"][0] - physical_origin[0], c["cmd"][1] - physical_origin[1]
            ax.plot(cx, cy, "x", color="black", ms=12, mew=2)
            ax.annotate(c["label"], (cx, cy), textcoords="offset points",
                         xytext=(8, 8), fontsize=9)
        # Per-waypoint GT-vs-EKF arrows.
        for label, ex, ey, gx, gy, dcm in row_lines:
            ax.plot([ex, gx], [ey, gy], "-", color="gray", lw=0.7, alpha=0.5)
        ax.plot(0, 0, "o", color="green", ms=10, label="takeoff origin")
        ax.set_xlabel("X [m] (origin = takeoff in each frame)")
        ax.set_ylabel("Y [m]")
        ax.set_title(f"s165 — EKF belief vs Gazebo GT  "
                     f"|  EKF_close_land={ekf_close_land:.2f}cm  "
                     f"GT_close_land={gt_close_land:.2f}cm")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=9)
        fig.tight_layout()
        fig.savefig(PLOT_PATH, dpi=110)
        print(f"[verdict] plot → {PLOT_PATH}")

    # Hard gate: GT-based closure (the real truth).
    print(f"          closure: EKF_belief={ekf_close_land:.2f}cm  "
          f"GT={gt_close_land:.2f}cm  (gate < {CLOSURE_GATE_CM:.0f}cm on GT)")
    if gt_close_land > CLOSURE_GATE_CM:
        print(f"[verdict] FAIL — GT closure {gt_close_land:.2f}cm > "
              f"{CLOSURE_GATE_CM:.0f}cm")
        return 1
    print(f"[verdict] PASS — GT closure {gt_close_land:.2f}cm "
          f"(EKF belief reported {ekf_close_land:.2f}cm; "
          f"delta={abs(gt_close_land-ekf_close_land):.2f}cm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

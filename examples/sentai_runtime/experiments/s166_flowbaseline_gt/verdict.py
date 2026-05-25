#!/usr/bin/env python3
"""s166 verdict — FlowBaseline EKF belief vs Gazebo GT.

Reads:
  /tmp/s166_flowbaseline_gt/gt_poses.jsonl         (host-side GT recorder)
  examples/.../s091_aruco_lowalt/hover_log.json    (aruco_hover output)

Aligns both timelines on host monotonic time (`t_wall`), then:
  - normalises each track to its own takeoff origin (first GT sample
    after the hover loop began, vs (0,0) for EKF which kalman.reset'd
    to zero pre-takeoff)
  - prints per-sample tabulation of EKF vs GT and |EKF-GT|
  - plot.png — top-down XY trajectories with distinct colours
       blue   = cf2 EKF belief (high-rate)
       red    = Gazebo GT
       black  = setpoint (origin)
       grey   = per-sample EKF→GT residual arrows
  - error.png — |EKF-GT| per-axis + magnitude vs time
  - gate: GT-based dist_max < GT_DIST_MAX_M ; PASS/FAIL

Anti-cheat: GT loaded host-side, plot-only — never read by mission.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

WORKDIR  = Path("/tmp/s166_flowbaseline_gt")
HOVER_LOG = Path(__file__).resolve().parents[1] / "s091_aruco_lowalt" / "hover_log.json"
GT_JSONL = WORKDIR / "gt_poses.jsonl"
PLOT_XY  = WORKDIR / "plot_xy.png"
PLOT_ERR = WORKDIR / "plot_err.png"
SUMMARY  = WORKDIR / "summary.json"

# Tentative gates — first post-cheat run; numbers will calibrate.
GT_DIST_MAX_M = 0.30   # max |xy| from origin under GT (was 0.15 pre-cheat)
GT_DIST_MEAN_MAX_M = 0.20


def _load_hover():
    if not HOVER_LOG.is_file():
        print(f"FAIL — no hover_log.json at {HOVER_LOG}")
        return None
    return json.loads(HOVER_LOG.read_text())


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


def _interp_xyz(samples, key_t, t_query):
    if not samples:
        return None
    if t_query <= samples[0][key_t]:
        s = samples[0]
        return (s["x"], s["y"], s["z"])
    if t_query >= samples[-1][key_t]:
        s = samples[-1]
        return (s["x"], s["y"], s["z"])
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
        return (s0["x"], s0["y"], s0["z"])
    a = (t_query - s0[key_t]) / span
    return (s0["x"] + a * (s1["x"] - s0["x"]),
            s0["y"] + a * (s1["y"] - s0["y"]),
            s0["z"] + a * (s1["z"] - s0["z"]))


def main() -> int:
    h = _load_hover()
    if h is None:
        return 1
    gt = _load_gt()
    ekf_trace = h.get("ekf_trace", [])
    samples   = h.get("samples", [])
    t0_hover  = h.get("hover_t0_wall")

    print(f"[verdict] s166 FlowBaseline-GT  "
          f"ekf_trace={len(ekf_trace)}  samples={len(samples)}  gt={len(gt)}")
    if not ekf_trace or not gt or t0_hover is None:
        print(f"FAIL — missing data (ekf={len(ekf_trace)}, gt={len(gt)}, "
              f"t0={t0_hover})")
        return 1

    # ── Frame alignment ─────────────────────────────────────────────
    # EKF was kalman.reset'd to (0,0) pre-takeoff; the take_off() leg
    # moves the drone +z 1m AND drifts a few cm in XY before the hover
    # loop starts (t0_hover marks hover-loop start, AFTER takeoff).
    # GT records absolute Gazebo coordinates; we subtract the GT sample
    # at t0_hover so both tracks are referenced to the same physical
    # moment (the start of the hover phase).  Matches the s165 v2 fix
    # ([[op-s8-w1-cf2-sim-honest]]).
    gt_origin = _interp_xyz(gt, "t_wall", t0_hover)
    if gt_origin is None:
        gt_origin = (gt[0]["x"], gt[0]["y"], gt[0]["z"])
    gx0, gy0, gz0 = gt_origin

    # EKF "origin" for plot is its physical position at t0_hover — which
    # by kalman.reset + takeoff is roughly (0, 0, 1) m.  We plot raw EKF
    # values (origin = takeoff pad in EKF frame) so any drift is visible
    # directly.  GT is normalised to its t0_hover sample so both share
    # the SAME (0, 0) at hover start.
    ekf_x = [e["x"] for e in ekf_trace]
    ekf_y = [e["y"] for e in ekf_trace]
    ekf_z = [e["z"] for e in ekf_trace]
    gt_xn = [g["x"] - gx0 for g in gt]
    gt_yn = [g["y"] - gy0 for g in gt]
    gt_zn = [g["z"] - gz0 for g in gt]
    ekf_t = [e["t_wall"] for e in ekf_trace]
    gt_t  = [g["t_wall"] for g in gt]

    # ── Per-sample residual table (5 Hz EKF samples) ────────────────
    print("          t[s]   EKF (x,y)             GT (x,y)              "
          "|EKF-GT|cm  EKF_z   GT_z")
    rows = []
    for s in samples:
        t = s["t_wall"]
        ex, ey, ez = s["ekf"]
        g = _interp_xyz(gt, "t_wall", t)
        if g is None:
            continue
        gx, gy, gz = g[0] - gx0, g[1] - gy0, g[2] - gz0
        d_cm = math.hypot(ex - gx, ey - gy) * 100.0
        rows.append({
            "t_rel": t - t0_hover, "ex": ex, "ey": ey, "ez": ez,
            "gx": gx, "gy": gy, "gz": gz, "d_cm": d_cm,
        })
    # Print sparsified — every 3rd row to keep stdout readable.
    for i, r in enumerate(rows):
        if i % 3 == 0:
            print(f"          {r['t_rel']:5.1f}  "
                  f"({r['ex']:+.3f},{r['ey']:+.3f})   "
                  f"({r['gx']:+.3f},{r['gy']:+.3f})   "
                  f"{r['d_cm']:6.2f}     {r['ez']:5.2f}  {r['gz']:5.2f}")

    # ── Drift metrics from GT (the gate) ─────────────────────────────
    if not rows:
        print("FAIL — no overlapping samples")
        return 1
    # Restrict to hover window (t > 0 since hover loop start).
    hover_rows = [r for r in rows if r["t_rel"] >= 0]
    gt_dist = [math.hypot(r["gx"], r["gy"]) for r in hover_rows]
    ekf_dist = [math.hypot(r["ex"], r["ey"]) for r in hover_rows]
    ekf_gt_err_cm = [r["d_cm"] for r in hover_rows]
    gt_dist_mean = sum(gt_dist) / len(gt_dist)
    gt_dist_max  = max(gt_dist)
    ekf_dist_mean = sum(ekf_dist) / len(ekf_dist)
    ekf_dist_max  = max(ekf_dist)
    err_mean_cm  = sum(ekf_gt_err_cm) / len(ekf_gt_err_cm)
    err_max_cm   = max(ekf_gt_err_cm)

    print(f"\n[verdict] hover samples         : {len(hover_rows)}")
    print(f"          GT  dist:  mean={gt_dist_mean*100:.2f}cm  max={gt_dist_max*100:.2f}cm")
    print(f"          EKF dist:  mean={ekf_dist_mean*100:.2f}cm  max={ekf_dist_max*100:.2f}cm")
    print(f"          |EKF-GT|:  mean={err_mean_cm:.2f}cm  max={err_max_cm:.2f}cm")

    # ── Plot 1: top-down XY trajectory ───────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as e:
        print(f"[verdict] WARN — matplotlib unavailable ({e}); skipping plots")
    else:
        fig, ax = plt.subplots(figsize=(9, 9))
        ax.plot(ekf_x, ekf_y, "-",  color="tab:blue", lw=1.4, alpha=0.85,
                label=f"cf2 EKF belief (n={len(ekf_trace)})")
        ax.plot(gt_xn, gt_yn, "-",  color="tab:red",  lw=1.4, alpha=0.85,
                label=f"Gazebo GT (n={len(gt)})")
        # Per-sample EKF→GT residuals.
        for r in hover_rows:
            ax.plot([r["ex"], r["gx"]], [r["ey"], r["gy"]],
                    "-", color="grey", lw=0.5, alpha=0.4)
        ax.plot(0, 0, "o", color="black", ms=10, label="setpoint (0,0)")
        ax.plot(ekf_x[0], ekf_y[0], "^", color="tab:blue", ms=10,
                label="EKF start")
        ax.plot(gt_xn[0], gt_yn[0], "^", color="tab:red",  ms=10,
                label="GT start")
        ax.plot(ekf_x[-1], ekf_y[-1], "s", color="tab:blue", ms=10,
                label="EKF end")
        ax.plot(gt_xn[-1], gt_yn[-1], "s", color="tab:red",  ms=10,
                label="GT end")
        ax.set_xlabel("X [m]  (each track origin = hover-loop start)")
        ax.set_ylabel("Y [m]")
        ax.set_title(
            f"s166 FlowBaseline — EKF belief vs Gazebo GT  |  "
            f"GT_dist mean={gt_dist_mean*100:.1f}cm max={gt_dist_max*100:.1f}cm  |  "
            f"|EKF-GT| mean={err_mean_cm:.1f}cm")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        fig.savefig(PLOT_XY, dpi=110)
        print(f"[verdict] XY plot → {PLOT_XY}")

        # ── Plot 2: error vs time ────────────────────────────────────
        fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        # X / Y / Z error
        t_rel = [r["t_rel"] for r in rows]
        ex_err = [(r["ex"] - r["gx"]) * 100 for r in rows]
        ey_err = [(r["ey"] - r["gy"]) * 100 for r in rows]
        ez_err = [(r["ez"] - r["gz"]) * 100 for r in rows]
        d_xy   = [r["d_cm"] for r in rows]
        axes[0].plot(t_rel, ex_err, "-", color="tab:red",   label="X err")
        axes[0].plot(t_rel, ey_err, "-", color="tab:blue",  label="Y err")
        axes[0].plot(t_rel, ez_err, "-", color="tab:green", label="Z err")
        axes[0].axhline(0, color="black", lw=0.5)
        axes[0].set_ylabel("EKF - GT [cm]")
        axes[0].grid(True, alpha=0.3); axes[0].legend(loc="upper right")
        axes[0].set_title("EKF - GT per-axis error")
        # |EKF-GT|
        axes[1].plot(t_rel, d_xy, "-", color="black", label="|EKF-GT| XY")
        axes[1].set_ylabel("|EKF-GT| [cm]")
        axes[1].grid(True, alpha=0.3); axes[1].legend(loc="upper right")
        # dist from origin (each frame)
        axes[2].plot(t_rel,
                     [math.hypot(r["ex"], r["ey"]) * 100 for r in rows],
                     "-", color="tab:blue", label="|EKF| from setpoint")
        axes[2].plot(t_rel,
                     [math.hypot(r["gx"], r["gy"]) * 100 for r in rows],
                     "-", color="tab:red",  label="|GT|  from setpoint")
        axes[2].set_xlabel("t since hover start [s]")
        axes[2].set_ylabel("dist from (0,0) [cm]")
        axes[2].grid(True, alpha=0.3); axes[2].legend(loc="upper right")
        fig.tight_layout()
        fig.savefig(PLOT_ERR, dpi=110)
        print(f"[verdict] err plot → {PLOT_ERR}")

    # ── Summary JSON ─────────────────────────────────────────────────
    summary = {
        "n_ekf_trace": len(ekf_trace),
        "n_samples": len(samples),
        "n_gt": len(gt),
        "n_hover_rows": len(hover_rows),
        "gt_dist_mean_cm": round(gt_dist_mean * 100, 2),
        "gt_dist_max_cm":  round(gt_dist_max * 100, 2),
        "ekf_dist_mean_cm": round(ekf_dist_mean * 100, 2),
        "ekf_dist_max_cm":  round(ekf_dist_max * 100, 2),
        "ekf_minus_gt_mean_cm": round(err_mean_cm, 2),
        "ekf_minus_gt_max_cm":  round(err_max_cm, 2),
        "gt_origin_world": [gx0, gy0, gz0],
        "gate": {"gt_dist_max_m": GT_DIST_MAX_M,
                 "gt_dist_mean_max_m": GT_DIST_MEAN_MAX_M},
    }
    SUMMARY.write_text(json.dumps(summary, indent=2))
    print(f"[verdict] summary → {SUMMARY}")

    # ── Gate on GT, not EKF ──────────────────────────────────────────
    fails = []
    if gt_dist_max > GT_DIST_MAX_M:
        fails.append(f"GT dist_max={gt_dist_max*100:.2f}cm > "
                     f"{GT_DIST_MAX_M*100:.0f}cm")
    if gt_dist_mean > GT_DIST_MEAN_MAX_M:
        fails.append(f"GT dist_mean={gt_dist_mean*100:.2f}cm > "
                     f"{GT_DIST_MEAN_MAX_M*100:.0f}cm")
    if fails:
        print(f"[verdict] FAIL — {'; '.join(fails)}")
        return 1
    print(f"[verdict] PASS — GT-based drift within gate "
          f"(mean={gt_dist_mean*100:.2f}cm, max={gt_dist_max*100:.2f}cm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

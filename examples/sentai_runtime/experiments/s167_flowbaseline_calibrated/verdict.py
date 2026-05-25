#!/usr/bin/env python3
"""s167 verdict — 4-phase FlowBaseline2 vs Gazebo GT.

Reads:
  /tmp/s167_flowbaseline_calibrated/phases.json   (mission output)
  /tmp/s167_flowbaseline_calibrated/gt_poses.jsonl (canonical recorder)
  examples/.../s091_aruco_lowalt/hover_log.json    (F3 mirror for plot)

Produces:
  plot_phases.png  — XY trajectory coloured by phase
  plot_xy_F3.png   — F3-only top-down (same shape as s166)
  plot_err_F3.png  — F3 per-axis error vs time
  summary.json     — per-phase + overall metrics
  Gate: F3 GT-based dist_max_m < GT_F3_DIST_MAX_M.

Anti-cheat: GT loaded host-side, plot-only — never read by mission.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

WORKDIR  = Path("/tmp/s167_flowbaseline_calibrated")
PHASES   = WORKDIR / "phases.json"
GT_JSONL = WORKDIR / "gt_poses.jsonl"
HOVER_LOG = Path(__file__).resolve().parents[1] / "s091_aruco_lowalt" / "hover_log.json"
PLOT_PHASES = WORKDIR / "plot_phases.png"
PLOT_XY_F3  = WORKDIR / "plot_xy_F3.png"
PLOT_ERR_F3 = WORKDIR / "plot_err_F3.png"
SUMMARY  = WORKDIR / "summary.json"

# Gates (tentative — F3 only; F1/F2 are calibration, not measurement).
GT_F3_DIST_MAX_M  = 0.30
GT_F3_DIST_MEAN_M = 0.20


def _load_json(p, default=None):
    if not p.is_file():
        return default
    return json.loads(p.read_text())


def _load_jsonl(p):
    if not p.is_file():
        return []
    out = []
    for line in p.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            out.append(json.loads(line))
        except Exception:
            pass
    return out


def _interp(samples, key_t, t_query):
    if not samples:
        return None
    if t_query <= samples[0][key_t]:
        s = samples[0]; return (s["x"], s["y"], s["z"])
    if t_query >= samples[-1][key_t]:
        s = samples[-1]; return (s["x"], s["y"], s["z"])
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid][key_t] <= t_query: lo = mid
        else: hi = mid
    s0, s1 = samples[lo], samples[hi]
    span = s1[key_t] - s0[key_t]
    if span <= 0: return (s0["x"], s0["y"], s0["z"])
    a = (t_query - s0[key_t]) / span
    return (s0["x"] + a * (s1["x"] - s0["x"]),
            s0["y"] + a * (s1["y"] - s0["y"]),
            s0["z"] + a * (s1["z"] - s0["z"]))


def main() -> int:
    ph = _load_json(PHASES)
    if ph is None:
        print(f"FAIL — phases.json missing at {PHASES}")
        return 1
    gt = _load_jsonl(GT_JSONL)
    hlog = _load_json(HOVER_LOG, default={})
    ekf_trace = hlog.get("ekf_trace", [])

    print(f"[verdict] s167 FlowBaseline2  phases={len(ph['phases'])}  "
          f"gt={len(gt)}  ekf_trace={len(ekf_trace)}")
    print(f"          BX default        = {ph['body_xform_default']}")
    print(f"          BX after F2b      = {ph['body_xform_final']}")

    # Locate phase blocks of interest.
    phases = ph["phases"]
    by_name = {p.get("name"): p for p in phases if isinstance(p, dict)}
    f2a = by_name.get("F2a_static_pin")
    # Iter #3 introduced F2_iter_calib (PnP-in-loop); fall back to iter #1/#2
    # F2b_axes_probe schema for archival data.
    f2b = by_name.get("F2_iter_calib") or by_name.get("F2b_axes_probe")
    f3  = by_name.get("F3_measure")

    # ── F2a metrics ─────────────────────────────────────────────────
    if f2a:
        print(f"\n[verdict] F2a static-pin @ z=0.60m")
        print(f"          captures = {f2a['n_captures']}")
        z = f2a.get('pnp_z_mean')
        zs = f2a.get('pnp_z_std')
        print(f"          PnP-z mean = "
              f"{('%.3f m' % z) if z is not None else 'n/a'}  "
              f"std = {('%.3f m' % zs) if zs is not None else 'n/a'}")
        e = f2a.get('ekf_xy_mean', [None, None])
        p = f2a.get('pnp_xy_mean', [None, None])
        print(f"          EKF xy mean = ({e[0]!r}, {e[1]!r})")
        print(f"          PnP xy mean = ({p[0]!r}, {p[1]!r})")

    # ── F2 axes-probe — three schema versions supported ────────────
    if f2b:
        iter3_schema = (f2b.get("name") == "F2_iter_calib")
        new_schema   = "vs_list" in f2b        # iter #2 (impulse)
        if iter3_schema:
            hist = f2b.get("history", [])
            print(f"\n[verdict] F2-iter PnP closed-loop  z={f2b.get('z_hold')}m  "
                  f"vs={f2b.get('vs_probe_mps')}m/s  iters_run={f2b.get('n_iterations_run')}  "
                  f"converged_at={f2b.get('converged_at')}  status={f2b.get('status')}")
            for h in hist:
                print(f"          i{h['iter']:02d} {h['label']:<3s}  "
                      f"pnp_body=({h['pnp_dx_body']:+.3f},{h['pnp_dy_body']:+.3f})m  "
                      f"flow=({h['flow_int_dx_grid']:+.1f},"
                      f"{h['flow_int_dy_grid']:+.1f})grid  "
                      f"drift={h['drift_from_anchor_m']*100:.1f}cm  "
                      f"ret={h['did_return']}  n_pp={h['n_pp']}")
            print(f"          BX raw fit       = {f2b['BX_raw_fit']}")
            print(f"          BX raw det       = {f2b.get('BX_raw_det')}")
            print(f"          BX sign-snap     = {f2b['BX_normalised_sign']}")
            print(f"          BX safe_to_swap  = {f2b.get('BX_safe_to_swap')}")
        elif new_schema:
            vs_list = f2b.get("vs_list", [])
            print(f"\n[verdict] F2b IMPULSE axes-probe  vs={vs_list} m/s  "
                  f"impulse={f2b.get('impulse_s')}s  settle={f2b.get('settle_s')}s")
            for leg in f2b["legs"]:
                vx = leg.get("vx_body_cmd", 0.0)
                vy = leg.get("vy_body_cmd", 0.0)
                print(f"          {leg['label']:<14s} "
                      f"v_cmd=({vx:+.3f},{vy:+.3f})m/s  "
                      f"ekf_body=({leg['ekf_dx_body']:+.3f},"
                      f"{leg['ekf_dy_body']:+.3f})m  "
                      f"flow_int=({leg['flow_int_dx_grid']:+.1f},"
                      f"{leg['flow_int_dy_grid']:+.1f})grid  "
                      f"ekf_z=({leg.get('ekf_z_start',0):.2f}->"
                      f"{leg.get('ekf_z_end',0):.2f})  n_pp={leg['n_pp']}")
        else:
            print(f"\n[verdict] F2b STEP axes-probe (±{f2b.get('step_m',0)*100:.0f}cm × 4 legs)")
            for leg in f2b["legs"]:
                print(f"          {leg['label']:<10s} "
                      f"cmd=({leg.get('cmd_dx',0):+.3f},{leg.get('cmd_dy',0):+.3f})  "
                      f"ekf_body=({leg['ekf_dx_body']:+.3f},{leg['ekf_dy_body']:+.3f})m  "
                      f"flow_int=({leg['flow_int_dx_grid']:+.1f},"
                      f"{leg['flow_int_dy_grid']:+.1f})grid  n_pp={leg['n_pp']}")
        print(f"          BX raw fit       = {f2b['BX_raw_fit']}")
        print(f"          BX raw det       = {f2b.get('BX_raw_det')}")
        print(f"          BX sign-snap     = {f2b['BX_normalised_sign']}")
        print(f"          BX safe_to_swap  = {f2b.get('BX_safe_to_swap')}")

    # ── F3 measurement metrics (GT-based gate) ─────────────────────
    f3_metrics = None
    if f3 and gt and f3.get("samples"):
        samples = f3["samples"]
        t0_hover = f3["t0_wall"]
        gt_origin = _interp(gt, "t_wall", t0_hover)
        gx0, gy0, gz0 = gt_origin or (gt[0]["x"], gt[0]["y"], gt[0]["z"])
        gt_dists, ekf_dists, errs_cm = [], [], []
        rows = []
        for s in samples:
            t = s["t_wall"]
            ex, ey, ez = s["ekf"]
            g = _interp(gt, "t_wall", t)
            if g is None: continue
            gx, gy, gz = g[0] - gx0, g[1] - gy0, g[2] - gz0
            d = math.hypot(ex - gx, ey - gy) * 100
            gt_dists.append(math.hypot(gx, gy))
            ekf_dists.append(math.hypot(ex, ey))
            errs_cm.append(d)
            rows.append({"t_rel": t - t0_hover,
                          "ex": ex, "ey": ey, "ez": ez,
                          "gx": gx, "gy": gy, "gz": gz, "d_cm": d})
        f3_metrics = {
            "n_samples": len(rows),
            "gt_origin_world": [gx0, gy0, gz0],
            "gt_dist_mean_cm": sum(gt_dists)/len(gt_dists)*100 if gt_dists else None,
            "gt_dist_max_cm":  max(gt_dists)*100 if gt_dists else None,
            "ekf_dist_mean_cm": sum(ekf_dists)/len(ekf_dists)*100 if ekf_dists else None,
            "ekf_dist_max_cm":  max(ekf_dists)*100 if ekf_dists else None,
            "ekf_minus_gt_mean_cm": sum(errs_cm)/len(errs_cm) if errs_cm else None,
            "ekf_minus_gt_max_cm":  max(errs_cm) if errs_cm else None,
        }
        print(f"\n[verdict] F3 measurement @ z=1.00m  ({len(rows)} samples)")
        print(f"          GT  dist: mean={f3_metrics['gt_dist_mean_cm']:.2f}cm  "
              f"max={f3_metrics['gt_dist_max_cm']:.2f}cm")
        print(f"          EKF dist: mean={f3_metrics['ekf_dist_mean_cm']:.2f}cm  "
              f"max={f3_metrics['ekf_dist_max_cm']:.2f}cm")
        print(f"          |EKF-GT|: mean={f3_metrics['ekf_minus_gt_mean_cm']:.2f}cm  "
              f"max={f3_metrics['ekf_minus_gt_max_cm']:.2f}cm")

    # ── Plots ──────────────────────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as e:
        print(f"[verdict] WARN — matplotlib unavailable ({e})")
        plt = None

    if plt is not None:
        # ── plot_phases.png — colour by phase, both EKF + GT ──────
        fig, ax = plt.subplots(figsize=(10, 10))
        if gt:
            t0 = ph["overall_t0_wall"]
            gxs = [g["x"] - gt[0]["x"] for g in gt]
            gys = [g["y"] - gt[0]["y"] for g in gt]
            ax.plot(gxs, gys, "-", color="lightcoral", alpha=0.5, lw=1.0,
                    label=f"GT full track (n={len(gt)})")
        # Phase-segmented EKF trace using time windows from each phase.
        if ekf_trace and isinstance(f2a, dict) and isinstance(f3, dict):
            t_f2a_start = f2a["t0_wall"]
            t_f2a_end   = t_f2a_start + f2a["duration_s"]
            t_f3_start  = f3["t0_wall"]
            t_f3_end    = t_f3_start + f3["duration_s"]
            colors = {"F1_takeoff": "tab:gray",
                       "F2a_pin":   "tab:green",
                       "F2b_probe": "tab:orange",
                       "F3_meas":   "tab:blue",
                       "F4_land":   "tab:purple"}
            def _seg(t_lo, t_hi, color, label):
                pts = [(e["x"], e["y"]) for e in ekf_trace
                        if t_lo <= e["t_wall"] < t_hi]
                if pts:
                    xs, ys = zip(*pts)
                    ax.plot(xs, ys, "-", color=color, lw=1.3, label=label)
            _seg(0, t_f2a_start,        colors["F1_takeoff"], "EKF F1 takeoff")
            _seg(t_f2a_start, t_f2a_end, colors["F2a_pin"],   "EKF F2a pin")
            _seg(t_f2a_end, t_f3_start,  colors["F2b_probe"], "EKF F2b probe")
            _seg(t_f3_start, t_f3_end,   colors["F3_meas"],   "EKF F3 hover")
            _seg(t_f3_end, 9e18,         colors["F4_land"],   "EKF F4 land")
        ax.plot(0, 0, "o", color="black", ms=9, label="setpoint (0,0)")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=8)
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
        bx_note = f"BX default {ph['body_xform_default']} → final {ph['body_xform_final']}"
        ax.set_title(f"s167 — EKF coloured by phase + GT  |  {bx_note}")
        fig.tight_layout(); fig.savefig(PLOT_PHASES, dpi=110)
        print(f"[verdict] phases plot → {PLOT_PHASES}")

        # ── plot_xy_F3.png + plot_err_F3.png — F3-only ────────────
        if f3_metrics is not None and rows:
            # Reuse the s166 plotting shape on F3 alone.
            fig, ax = plt.subplots(figsize=(9, 9))
            ekf_x = [e["x"] for e in ekf_trace
                     if f3["t0_wall"] <= e["t_wall"] <= f3["t0_wall"] + f3["duration_s"]]
            ekf_y = [e["y"] for e in ekf_trace
                     if f3["t0_wall"] <= e["t_wall"] <= f3["t0_wall"] + f3["duration_s"]]
            gt_x = [g["x"] - f3_metrics["gt_origin_world"][0] for g in gt
                     if f3["t0_wall"] <= g["t_wall"] <= f3["t0_wall"] + f3["duration_s"]]
            gt_y = [g["y"] - f3_metrics["gt_origin_world"][1] for g in gt
                     if f3["t0_wall"] <= g["t_wall"] <= f3["t0_wall"] + f3["duration_s"]]
            ax.plot(ekf_x, ekf_y, "-", color="tab:blue", lw=1.3, label="cf2 EKF (F3 only)")
            ax.plot(gt_x,  gt_y,  "-", color="tab:red",  lw=1.3, label="Gazebo GT (F3 only)")
            for r in rows:
                ax.plot([r["ex"], r["gx"]], [r["ey"], r["gy"]],
                        "-", color="grey", lw=0.4, alpha=0.4)
            ax.plot(0, 0, "o", color="black", ms=9, label="setpoint")
            ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend()
            ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
            ax.set_title(
                f"s167 F3 hover (z=1m)  |  "
                f"GT mean={f3_metrics['gt_dist_mean_cm']:.1f}cm "
                f"max={f3_metrics['gt_dist_max_cm']:.1f}cm  |  "
                f"|EKF-GT| mean={f3_metrics['ekf_minus_gt_mean_cm']:.1f}cm")
            fig.tight_layout(); fig.savefig(PLOT_XY_F3, dpi=110)
            print(f"[verdict] F3 XY plot → {PLOT_XY_F3}")

            # Error per axis vs time.
            fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
            t = [r["t_rel"] for r in rows]
            axes[0].plot(t, [(r["ex"]-r["gx"])*100 for r in rows], color="tab:red",   label="X err")
            axes[0].plot(t, [(r["ey"]-r["gy"])*100 for r in rows], color="tab:blue",  label="Y err")
            axes[0].plot(t, [(r["ez"]-r["gz"])*100 for r in rows], color="tab:green", label="Z err")
            axes[0].axhline(0, color="black", lw=0.5); axes[0].grid(True, alpha=0.3)
            axes[0].set_ylabel("EKF - GT [cm]"); axes[0].legend(loc="upper right")
            axes[0].set_title("F3 hover — EKF - GT per-axis error")
            axes[1].plot(t, [r["d_cm"] for r in rows], color="black", label="|EKF-GT| XY")
            axes[1].set_ylabel("|EKF-GT| [cm]"); axes[1].grid(True, alpha=0.3); axes[1].legend(loc="upper right")
            axes[2].plot(t, [math.hypot(r["ex"],r["ey"])*100 for r in rows], color="tab:blue", label="|EKF| from setpoint")
            axes[2].plot(t, [math.hypot(r["gx"],r["gy"])*100 for r in rows], color="tab:red",  label="|GT|  from setpoint")
            axes[2].set_xlabel("t since hover start [s]")
            axes[2].set_ylabel("dist from (0,0) [cm]")
            axes[2].grid(True, alpha=0.3); axes[2].legend(loc="upper right")
            fig.tight_layout(); fig.savefig(PLOT_ERR_F3, dpi=110)
            print(f"[verdict] F3 err plot → {PLOT_ERR_F3}")

    # ── Summary + gate ──────────────────────────────────────────────
    summary = {
        "body_xform_default": ph["body_xform_default"],
        "body_xform_final":   ph["body_xform_final"],
        "F2a": ({"pnp_z_mean": f2a.get("pnp_z_mean"),
                  "pnp_z_std": f2a.get("pnp_z_std"),
                  "n_captures": f2a.get("n_captures")} if f2a else None),
        "F2b": ({"BX_raw_fit": f2b.get("BX_raw_fit"),
                  "BX_raw_det": f2b.get("BX_raw_det"),
                  "BX_normalised_sign": f2b.get("BX_normalised_sign"),
                  "BX_safe_to_swap": f2b.get("BX_safe_to_swap"),
                  "converged_at": f2b.get("converged_at"),
                  "n_iterations_run": f2b.get("n_iterations_run"),
                  "legs": [{"label": l.get("label"),
                              "cmd": [l.get("vx_body_cmd", l.get("cmd_dx", 0.0)),
                                       l.get("vy_body_cmd", l.get("cmd_dy", 0.0))],
                              "ekf_body": [l.get("ekf_dx_body", 0), l.get("ekf_dy_body", 0)],
                              "pnp_body": [l.get("pnp_dx_body"), l.get("pnp_dy_body")],
                              "flow_int": [l["flow_int_dx_grid"], l["flow_int_dy_grid"]],
                              "drift_cm": l.get("drift_from_anchor_m", 0) * 100,
                              "did_return": l.get("did_return")}
                             for l in (f2b.get("legs", []) or f2b.get("history", []))]}
                 if f2b else None),
        "F3": f3_metrics,
        "gate": {"f3_gt_dist_max_m": GT_F3_DIST_MAX_M,
                  "f3_gt_dist_mean_max_m": GT_F3_DIST_MEAN_M},
    }
    SUMMARY.write_text(json.dumps(summary, indent=2))
    print(f"[verdict] summary → {SUMMARY}")

    if f3_metrics is None:
        print("[verdict] FAIL — no F3 metrics (F3 didn't run or no GT)")
        return 1
    fails = []
    if f3_metrics["gt_dist_max_cm"] / 100 > GT_F3_DIST_MAX_M:
        fails.append(f"F3 GT dist_max={f3_metrics['gt_dist_max_cm']:.2f}cm > "
                     f"{GT_F3_DIST_MAX_M*100:.0f}cm")
    if f3_metrics["gt_dist_mean_cm"] / 100 > GT_F3_DIST_MEAN_M:
        fails.append(f"F3 GT dist_mean={f3_metrics['gt_dist_mean_cm']:.2f}cm > "
                     f"{GT_F3_DIST_MEAN_M*100:.0f}cm")
    if fails:
        print(f"[verdict] FAIL — {'; '.join(fails)}")
        return 1
    print(f"[verdict] PASS — F3 GT drift within gate")
    return 0


if __name__ == "__main__":
    sys.exit(main())

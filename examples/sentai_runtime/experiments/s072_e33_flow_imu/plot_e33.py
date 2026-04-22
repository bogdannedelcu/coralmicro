#!/usr/bin/env python3
"""plot_e33.py — flow + IMU trace with per-phase coloring.

Reads 001_e33_flow_imu_square.csv, produces:
  trace_phases.png   — 2D (pos_left, pos_fw) coloured by phase
  flow_vs_imu.png    — time series of body_fw / body_left / imu_xyz
  per_phase.txt      — numeric summary
"""
import csv
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = pathlib.Path(__file__).parent
CSV_PATH = next(HERE.glob("001_e33_*.csv"))


def load():
    rows = []
    with CSV_PATH.open() as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    return rows


def to_arrays(rows):
    t   = np.array([int(r["t_ms"])       for r in rows])
    ph  = np.array([r["phase"]            for r in rows])
    pi  = np.array([int(r["phase_idx"])  for r in rows])
    bfw = np.array([int(r["body_fw"])    for r in rows])
    blf = np.array([int(r["body_left"])  for r in rows])
    pfw = np.array([int(r["pos_fw"])     for r in rows])
    plf = np.array([int(r["pos_left"])   for r in rows])
    conf= np.array([int(r["confidence"]) for r in rows])
    imx = np.array([int(r["imu_x_mg"])   for r in rows])
    imy = np.array([int(r["imu_y_mg"])   for r in rows])
    imz = np.array([int(r["imu_z_mg"])   for r in rows])
    return t, ph, pi, bfw, blf, pfw, plf, conf, imx, imy, imz


PHASE_COLORS = {
    "WAIT_start": "#888",
    "MOVE_FW":    "#1f77b4",
    "WAIT_c1":    "#666",
    "MOVE_R":     "#2ca02c",
    "WAIT_c2":    "#555",
    "MOVE_BACK":  "#d62728",
    "WAIT_c3":    "#444",
    "MOVE_L":     "#ff7f0e",
    "WAIT_end":   "#333",
}


def main():
    rows = load()
    t, ph, pi, bfw, blf, pfw, plf, conf, imx, imy, imz = to_arrays(rows)
    print(f"samples: {len(t)}  duration: {t[-1]/1000:.2f}s")
    print(f"close-loop pos_fw={pfw[-1]}, pos_left={plf[-1]}")

    # ── 2D trace coloured by phase ───────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 10))
    for name, c in PHASE_COLORS.items():
        mask = ph == name
        if not mask.any(): continue
        ax.scatter(plf[mask], pfw[mask], c=c, s=20, label=name,
                   edgecolor="k", linewidths=0.2)
    ax.plot(plf, pfw, "-", color="gray", lw=0.5, alpha=0.4)
    ax.axhline(0, color="k", lw=0.3); ax.axvline(0, color="k", lw=0.3)
    ax.set_xlabel("pos_left  ← R      L →")
    ax.set_ylabel("pos_fw    ↓ BACK     FW ↑")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.set_title(f"E33 — phase-coloured square trace  ({len(t)} samples)")
    ax.legend(loc="upper left", fontsize=8)
    # Annotate start/end
    ax.annotate("start", (plf[0], pfw[0]), xytext=(8, 8),
                textcoords="offset points",
                bbox=dict(boxstyle="round", fc="lightgreen", alpha=0.7))
    ax.annotate("end",   (plf[-1], pfw[-1]), xytext=(8, -12),
                textcoords="offset points",
                bbox=dict(boxstyle="round", fc="lightpink", alpha=0.7))
    fig.tight_layout()
    out = HERE / "trace_phases.png"
    fig.savefig(out, dpi=110); plt.close(fig)
    print(f"wrote {out}")

    # ── Time series: flow vs imu ─────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    ts = t / 1000.0

    # Phase overlay: shaded bands for each SOLID phase.
    phase_starts = [0]
    for i in range(1, len(t)):
        if pi[i] != pi[i-1]:
            phase_starts.append(i)
    phase_starts.append(len(t))
    for j in range(len(phase_starts) - 1):
        a, b = phase_starts[j], phase_starts[j+1]
        name = ph[a]
        if name.startswith("MOVE_"):
            for ax in axes:
                ax.axvspan(ts[a], ts[b-1], alpha=0.08,
                           color=PHASE_COLORS.get(name, "#ccc"))
                ax.annotate(name.replace("MOVE_", ""),
                            xy=(ts[a], ax.get_ylim()[1] if False else 0),
                            xytext=(ts[a] + 0.1, 0),
                            fontsize=7, color="gray",
                            ha="left", va="bottom")

    axes[0].plot(ts, bfw, color="tab:blue", lw=0.8, label="body_fw")
    axes[0].plot(ts, blf, color="tab:orange", lw=0.8, label="body_left")
    axes[0].axhline(0, color="k", lw=0.3); axes[0].set_ylabel("Δ px/frame")
    axes[0].legend(loc="upper right"); axes[0].grid(alpha=0.3)
    axes[0].set_title("Flow per-frame delta")

    axes[1].plot(ts, pfw, color="tab:blue", label="pos_fw")
    axes[1].plot(ts, plf, color="tab:orange", label="pos_left")
    axes[1].axhline(0, color="k", lw=0.3); axes[1].set_ylabel("cum. px")
    axes[1].legend(loc="upper right"); axes[1].grid(alpha=0.3)
    axes[1].set_title("Flow cumulative position")

    # IMU — if all zeros, mark as "IMU not recorded"
    imu_nonzero = (imx.any() or imy.any() or imz.any())
    if imu_nonzero:
        axes[2].plot(ts, imx, color="tab:red",   lw=0.6, label="imu_x")
        axes[2].plot(ts, imy, color="tab:green", lw=0.6, label="imu_y")
        axes[2].plot(ts, imz, color="tab:purple",lw=0.6, label="imu_z")
        axes[2].legend(loc="upper right"); axes[2].grid(alpha=0.3)
        axes[2].set_ylabel("accel  (mg)")
        axes[2].set_title("IMU accelerometer")
    else:
        axes[2].text(0.5, 0.5,
                     "IMU data not recorded in this session\n"
                     "(imu.read() returned dict in a tuple context)",
                     ha="center", va="center",
                     transform=axes[2].transAxes, fontsize=11,
                     bbox=dict(boxstyle="round", fc="mistyrose"))
        axes[2].set_axis_off()

    axes[3].plot(ts, conf, color="tab:green", lw=0.6)
    axes[3].set_ylim(0, 260); axes[3].set_ylabel("confidence")
    axes[3].set_xlabel("time (s)")
    axes[3].grid(alpha=0.3); axes[3].set_title("SAD confidence")

    fig.tight_layout()
    out = HERE / "flow_vs_imu.png"
    fig.savefig(out, dpi=110); plt.close(fig)
    print(f"wrote {out}")

    # ── Per-phase numeric summary ────────────────────────────────
    summary = []
    for name in ("WAIT_start", "MOVE_FW", "WAIT_c1", "MOVE_R",
                 "WAIT_c2", "MOVE_BACK", "WAIT_c3", "MOVE_L", "WAIT_end"):
        mask = ph == name
        if not mask.any(): continue
        bfw_mean = bfw[mask].mean()
        blf_mean = blf[mask].mean()
        pfw_start = pfw[mask][0]; pfw_end = pfw[mask][-1]
        plf_start = plf[mask][0]; plf_end = plf[mask][-1]
        summary.append(
            f"{name:12s}  N={mask.sum():3d}  "
            f"mean body=({bfw_mean:+.2f}, {blf_mean:+.2f})  "
            f"Δpos=({pfw_end - pfw_start:+d}, {plf_end - plf_start:+d})"
        )
    summary_text = "\n".join(summary)
    print("\n" + summary_text)
    (HERE / "per_phase.txt").write_text(summary_text + "\n")


if __name__ == "__main__":
    main()

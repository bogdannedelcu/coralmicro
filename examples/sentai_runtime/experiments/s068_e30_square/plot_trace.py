#!/usr/bin/env python3
"""plot_trace.py — render the E30 flow trace as a 2D plot.

Reads 001_e30_flow_trace.csv in this directory, produces:
  trace_2d.png     — 2D scatter+line of (pos_left, pos_fw), drone frame
  components.png   — per-axis time series of body_fw / body_left / cumulative
"""
import csv
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = pathlib.Path(__file__).parent
CSV = HERE / "001_e30_flow_trace.csv"


def load():
    t, bfw, blf, pfw, plf, conf = [], [], [], [], [], []
    with CSV.open() as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(int(row["t_ms"]))
            bfw.append(int(row["body_fw"]))
            blf.append(int(row["body_left"]))
            pfw.append(int(row["pos_fw"]))
            plf.append(int(row["pos_left"]))
            conf.append(int(row["confidence"]))
    return (np.array(t), np.array(bfw), np.array(blf),
            np.array(pfw), np.array(plf), np.array(conf))


def main():
    t, bfw, blf, pfw, plf, conf = load()
    print(f"samples: {len(t)}")
    print(f"duration: {(t[-1] - t[0]) / 1000:.2f} s")
    print(f"pos_fw range:   [{pfw.min()}, {pfw.max()}]  "
          f"(swing {pfw.max() - pfw.min()})")
    print(f"pos_left range: [{plf.min()}, {plf.max()}]  "
          f"(swing {plf.max() - plf.min()})")
    print(f"confidence mean: {conf.mean():.0f}  min: {conf.min()}")

    # ── 2D trace plot ────────────────────────────────────────────
    # Convention: FW up, L left.  Use color = time for readability.
    fig, ax = plt.subplots(figsize=(9, 9))
    sc = ax.scatter(plf, pfw, c=t / 1000.0, cmap="viridis",
                    s=20, edgecolor="k", linewidths=0.2)
    ax.plot(plf, pfw, "-", color="gray", lw=0.6, alpha=0.5)
    ax.axhline(0, color="k", lw=0.3); ax.axvline(0, color="k", lw=0.3)
    ax.set_xlabel("pos_left  (grid-px cumulated)   "
                  "← R      L →")
    ax.set_ylabel("pos_fw    (grid-px cumulated)   "
                  "↓ BACK     FW ↑")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.set_title(f"E30 — Drone 2D trace from M4 optical flow "
                 f"({len(t)} samples, {(t[-1]-t[0])/1000:.1f}s)")
    cb = plt.colorbar(sc, ax=ax, shrink=0.7)
    cb.set_label("t  (seconds since LED_ON)")
    # Annotate start and end.
    ax.annotate("start", (plf[0], pfw[0]), xytext=(10, 10),
                textcoords="offset points", fontsize=9,
                bbox=dict(boxstyle="round,pad=0.2", fc="lightgreen",
                          alpha=0.8))
    ax.annotate("end", (plf[-1], pfw[-1]), xytext=(10, -10),
                textcoords="offset points", fontsize=9,
                bbox=dict(boxstyle="round,pad=0.2", fc="lightpink",
                          alpha=0.8))
    out = HERE / "trace_2d.png"
    fig.tight_layout(); fig.savefig(out, dpi=110); plt.close(fig)
    print(f"wrote {out}")

    # ── per-axis time series ─────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t / 1000, bfw, lw=0.8, color="tab:blue",
                 label="body_fw  (per-frame)")
    axes[0].plot(t / 1000, blf, lw=0.8, color="tab:orange",
                 label="body_left  (per-frame)")
    axes[0].axhline(0, color="k", lw=0.3)
    axes[0].set_ylabel("Δ grid-px / frame")
    axes[0].legend(loc="upper right"); axes[0].grid(alpha=0.3)
    axes[0].set_title("E30 — per-frame body-frame delta")

    axes[1].plot(t / 1000, pfw, color="tab:blue", label="pos_fw  (cum.)")
    axes[1].plot(t / 1000, plf, color="tab:orange",
                 label="pos_left  (cum.)")
    axes[1].axhline(0, color="k", lw=0.3)
    axes[1].set_ylabel("cumulative grid-px")
    axes[1].legend(loc="upper right"); axes[1].grid(alpha=0.3)
    axes[1].set_title("E30 — cumulative position (integrated from flow)")

    axes[2].plot(t / 1000, conf, color="tab:green", lw=0.7)
    axes[2].set_ylabel("SAD confidence (0-255)")
    axes[2].set_xlabel("time  (s)")
    axes[2].grid(alpha=0.3); axes[2].set_ylim(0, 260)
    axes[2].set_title("E30 — match confidence")

    out2 = HERE / "components.png"
    fig.tight_layout(); fig.savefig(out2, dpi=110); plt.close(fig)
    print(f"wrote {out2}")


if __name__ == "__main__":
    main()

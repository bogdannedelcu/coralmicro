#!/usr/bin/env python3
"""s181 — plot WhyCon SIM eval results (X/Y/Z estimated vs GT).

Reads `s181_corrected.csv` (post annulus-correction factor) and
generates three side-by-side panels for the thesis chapter.

WBS: OP-S10-W19-T4.
"""

import argparse
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")  # headless host
import matplotlib.pyplot as plt   # noqa: E402


HERE = pathlib.Path(__file__).resolve().parent


def parse_csv(path):
    """Return dict {tag: [(gt, est_x, est_y, est_z), ...]}."""
    rows = {"Z": [], "X": [], "Y": []}
    for line in path.read_text().splitlines():
        if not line.startswith("S181:") or line.startswith("S181:#"):
            continue
        body = line[5:]
        parts = body.split(",")
        if len(parts) < 13:
            continue
        tag = parts[0]
        try:
            Xgt = float(parts[1]); Ygt = float(parts[2]); Zgt = float(parts[3])
            status = parts[7]
            if status != "OK":
                continue
            Xest = float(parts[8]); Yest = float(parts[9]); Zest = float(parts[10])
        except (ValueError, IndexError):
            continue
        rows.setdefault(tag, []).append((Xgt, Ygt, Zgt, Xest, Yest, Zest))
    return rows


def plot_axis_compare(rows, axis_name, gt_index, est_index, ax, color,
                        title_suffix=""):
    """One panel: estimated vs GT for the chosen axis."""
    pts = [(r[gt_index], r[est_index]) for r in rows]
    if not pts:
        ax.text(0.5, 0.5, "no data", transform=ax.transAxes, ha="center")
        ax.set_title(f"{axis_name} {title_suffix}")
        return
    gt = [p[0] for p in pts]
    est = [p[1] for p in pts]
    # Identity reference line (where est == gt).
    lo = min(min(gt), min(est))
    hi = max(max(gt), max(est))
    pad = 0.05 * (hi - lo + 1e-9)
    ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad],
             "k--", linewidth=0.8, label="ideal (est = gt)")
    ax.scatter(gt, est, c=color, s=30, alpha=0.85,
                edgecolor="black", linewidth=0.4, label="measured")
    ax.set_xlabel(f"{axis_name}_gt  (m)")
    ax.set_ylabel(f"{axis_name}_est (m)")
    ax.set_title(f"{axis_name} {title_suffix}")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="upper left", fontsize=8)
    # Annotate the mean absolute error so the plot is self-describing.
    errs = [abs(e - g) for g, e in zip(gt, est)]
    mae = sum(errs) / len(errs)
    ax.text(0.98, 0.02, f"MAE = {mae*100:.2f} cm  (n={len(pts)})",
             transform=ax.transAxes, ha="right", va="bottom",
             fontsize=9,
             bbox=dict(boxstyle="round,pad=0.3", fc="white",
                        ec="gray", alpha=0.85))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--corrected", default=str(HERE / "s181_corrected.csv"),
                     help="CSV with annulus-correction applied")
    ap.add_argument("--uncorrected", default=str(HERE / "s181_uncorrected.csv"),
                     help="CSV without annulus correction (for compare plot)")
    ap.add_argument("--out", default=str(HERE / "s181_xyz_compare.png"))
    ap.add_argument("--out-z", default=str(HERE / "s181_z_uncorrected_vs_corrected.png"),
                     help="Second figure: Z bias before/after correction")
    args = ap.parse_args()

    corrected_path = pathlib.Path(args.corrected)
    uncorrected_path = pathlib.Path(args.uncorrected)
    if not corrected_path.exists():
        sys.exit(f"missing: {corrected_path}")

    corrected = parse_csv(corrected_path)

    # ----- Figure 1: estimated vs GT for X, Y, Z (corrected) ----------
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))
    # Z sweep uses tag "Z"; Z values are in row[2] (Zgt) and row[5] (Zest).
    plot_axis_compare(corrected.get("Z", []), "Z",
                        gt_index=2, est_index=5, ax=axes[0],
                        color="tab:blue",
                        title_suffix="(sweep, X=0 Y=0)")
    plot_axis_compare(corrected.get("X", []), "X",
                        gt_index=0, est_index=3, ax=axes[1],
                        color="tab:orange",
                        title_suffix="(sweep, Y=0 Z=0.5 m)")
    plot_axis_compare(corrected.get("Y", []), "Y",
                        gt_index=1, est_index=4, ax=axes[2],
                        color="tab:green",
                        title_suffix="(sweep, X=0 Z=0.5 m)")
    fig.suptitle(
        "WhyCon SIM eval — closed-form PnP (annulus-corrected), "
        "single Krajník marker, fx=fy=240, d=8 cm",
        fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(args.out, dpi=150)
    print(f"[s181] wrote {args.out}")

    # ----- Figure 2: Z bias before/after correction -------------------
    if uncorrected_path.exists():
        uncorrected = parse_csv(uncorrected_path)
        fig2, axes2 = plt.subplots(1, 2, figsize=(11, 4.6))
        plot_axis_compare(uncorrected.get("Z", []), "Z",
                            gt_index=2, est_index=5, ax=axes2[0],
                            color="tab:red",
                            title_suffix="(uncorrected — solid-disc PnP)")
        plot_axis_compare(corrected.get("Z", []), "Z",
                            gt_index=2, est_index=5, ax=axes2[1],
                            color="tab:blue",
                            title_suffix="(annulus-corrected, ×1.166)")
        fig2.suptitle(
            "WhyCon SIM eval — Z systematic bias before/after annulus "
            "correction (Krajník inner/outer = 0.6)",
            fontsize=11)
        fig2.tight_layout(rect=(0, 0, 1, 0.93))
        fig2.savefig(args.out_z, dpi=150)
        print(f"[s181] wrote {args.out_z}")


if __name__ == "__main__":
    main()

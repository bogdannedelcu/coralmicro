#!/usr/bin/env python3
"""Generate paper-grade figures from a TD-S10-B2 WhyCon ablation run.

Usage:
    ./venv/bin/python sim/scripts/plot_whycon_ablation_figures.py \
        dataset/TD-S10-B2/<dataset_id>/validation_YYYYMMDD_HHMMSS

Outputs PNG figures into <validation_dir>/figures/.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


OPENCV_COLOR = "#1f77b4"
SENTAI_COLOR = "#d62728"


def load_results(validation_dir: Path) -> tuple[list[dict], dict]:
    results = [json.loads(line) for line in (validation_dir / "results.jsonl").read_text().splitlines() if line.strip()]
    summary = json.loads((validation_dir / "summary.json").read_text())
    return results, summary


def fig_recall_by_z(results: list[dict], out: Path) -> None:
    by_z: dict[float, dict[str, list[int]]] = defaultdict(lambda: {"expected": [], "opencv": [], "sentai": []})
    for r in results:
        z = round(r["pose_world_camera"]["z"], 2)
        bo = r["backends"].get("opencv", {})
        bs = r["backends"].get("sentai_sim", {}) or {}
        exp = r["expected_count"]
        by_z[z]["expected"].append(exp)
        by_z[z]["opencv"].append(bo.get("matched_count", 0))
        by_z[z]["sentai"].append(bs.get("matched_count", 0) if bs.get("available", False) else 0)
    zs = sorted(by_z.keys())
    opencv_rec = [sum(by_z[z]["opencv"]) / max(1, sum(by_z[z]["expected"])) for z in zs]
    sentai_rec = [sum(by_z[z]["sentai"]) / max(1, sum(by_z[z]["expected"])) for z in zs]

    fig, ax = plt.subplots(figsize=(7, 4.2))
    x = np.arange(len(zs))
    w = 0.36
    ax.bar(x - w / 2, opencv_rec, w, label="OpenCV reference", color=OPENCV_COLOR)
    ax.bar(x + w / 2, sentai_rec, w, label="sentai_sim (embedded port)", color=SENTAI_COLOR)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{z:.2f} m" for z in zs])
    ax.set_ylim(0.95, 1.005)
    ax.set_ylabel("Marker recall (matched / expected)")
    ax.set_xlabel("Camera altitude")
    ax.set_title("WhyCon marker recall by altitude — 368-frame Gazebo dataset")
    ax.axhline(1.0, color="0.5", linewidth=0.7, linestyle=":")
    ax.legend(loc="lower left")
    ax.grid(axis="y", alpha=0.3)
    for i, (o, s) in enumerate(zip(opencv_rec, sentai_rec)):
        ax.text(i - w / 2, o + 0.001, f"{o:.4f}", ha="center", va="bottom", fontsize=8, color=OPENCV_COLOR)
        ax.text(i + w / 2, s + 0.001, f"{s:.4f}", ha="center", va="bottom", fontsize=8, color=SENTAI_COLOR)
    fig.tight_layout()
    fig.savefig(out / "fig1_recall_by_altitude.png", dpi=140)
    plt.close(fig)


def _collect_centroid(results: list[dict], backend: str) -> np.ndarray:
    out: list[float] = []
    for r in results:
        b = r["backends"].get(backend, {}) or {}
        if not b.get("available", True):
            continue
        for m in b.get("matches", []) or []:
            if "centroid_error_px" in m:
                out.append(m["centroid_error_px"])
    return np.asarray(out)


def fig_centroid_distribution(results: list[dict], out: Path) -> None:
    o = _collect_centroid(results, "opencv")
    s = _collect_centroid(results, "sentai_sim")

    fig, (ax_hist, ax_cdf) = plt.subplots(1, 2, figsize=(11, 4.2))
    bins = np.linspace(0, max(o.max(), s.max()) * 1.05, 40)
    ax_hist.hist(o, bins=bins, alpha=0.55, label=f"OpenCV  (n={len(o)})", color=OPENCV_COLOR)
    ax_hist.hist(s, bins=bins, alpha=0.55, label=f"sentai_sim  (n={len(s)})", color=SENTAI_COLOR)
    ax_hist.set_xlabel("Per-match centroid error (px)")
    ax_hist.set_ylabel("Count of matched detections")
    ax_hist.set_title("Centroid error histogram")
    ax_hist.legend(loc="upper right")
    ax_hist.grid(alpha=0.3)

    for arr, lab, c in [(o, "OpenCV", OPENCV_COLOR), (s, "sentai_sim", SENTAI_COLOR)]:
        if len(arr) == 0:
            continue
        xs = np.sort(arr)
        ys = np.arange(1, len(xs) + 1) / len(xs)
        ax_cdf.plot(xs, ys, label=f"{lab}  p95={np.percentile(arr, 95):.2f} px", color=c, linewidth=1.6)
    ax_cdf.set_xlabel("Centroid error (px)")
    ax_cdf.set_ylabel("CDF")
    ax_cdf.set_title("Centroid error CDF")
    ax_cdf.axvline(1.0, color="0.5", linewidth=0.7, linestyle=":")
    ax_cdf.legend(loc="lower right")
    ax_cdf.grid(alpha=0.3)
    ax_cdf.set_xlim(0, max(np.percentile(o, 99), np.percentile(s, 99)) * 1.05)

    fig.suptitle("Centroid error: OpenCV reference vs sentai_sim embedded port", y=1.02)
    fig.tight_layout()
    fig.savefig(out / "fig2_centroid_distribution.png", dpi=140, bbox_inches="tight")
    plt.close(fig)


def _collect_translation(results: list[dict], backend: str) -> np.ndarray:
    out = []
    for r in results:
        b = r["backends"].get(backend, {}) or {}
        if not b.get("available", True):
            continue
        pe = b.get("pose_eval", {}) or {}
        if not pe.get("available", False):
            continue
        if "translation_error_m" in pe:
            out.append(pe["translation_error_m"])
    return np.asarray(out)


def fig_translation_cdf(results: list[dict], out: Path) -> None:
    o = _collect_translation(results, "opencv")
    s = _collect_translation(results, "sentai_sim")

    fig, ax = plt.subplots(figsize=(7, 4.6))
    for arr, lab, c in [(o, "OpenCV  (exhaustive perm.)", OPENCV_COLOR),
                        (s, "sentai_sim  (prior-guided)", SENTAI_COLOR)]:
        if len(arr) == 0:
            continue
        xs = np.sort(arr)
        ys = np.arange(1, len(xs) + 1) / len(xs)
        ax.plot(xs, ys, label=f"{lab}\n  n={len(arr)}  RMSE={np.sqrt(np.mean(arr**2)):.3f} m  p95={np.percentile(arr,95):.3f} m",
                color=c, linewidth=1.8)
    ax.set_xscale("log")
    ax.set_xlabel("Camera translation error vs ground truth (m, log scale)")
    ax.set_ylabel("CDF over pose-valid frames")
    ax.set_title("Pose translation CDF — exposes mirror-branch outliers")
    ax.axvline(0.05, color="0.5", linewidth=0.7, linestyle=":")
    ax.text(0.05, 0.02, "5 cm", color="0.4", fontsize=8, rotation=0, ha="left", va="bottom")
    ax.axvline(1.0, color="0.8", linewidth=0.7, linestyle=":")
    ax.text(1.0, 0.02, "1 m  (mirror branch)", color="0.4", fontsize=8, ha="left", va="bottom")
    ax.set_xlim(1e-3, 2.0)
    ax.set_ylim(0, 1.02)
    ax.legend(loc="lower right", fontsize=9)
    ax.grid(alpha=0.3, which="both")
    fig.tight_layout()
    fig.savefig(out / "fig3_translation_cdf.png", dpi=140)
    plt.close(fig)


def fig_translation_vs_marker_count(results: list[dict], out: Path) -> None:
    bins = [4, 5, 6, 7]
    data = {b: {"opencv": [], "sentai": []} for b in bins}
    for r in results:
        n = r["expected_count"]
        if n not in bins:
            continue
        for backend, key in [("opencv", "opencv"), ("sentai_sim", "sentai")]:
            b = r["backends"].get(backend, {}) or {}
            if not b.get("available", True):
                continue
            pe = b.get("pose_eval", {}) or {}
            if not pe.get("available", False):
                continue
            if "translation_error_m" in pe:
                data[n][key].append(pe["translation_error_m"])

    fig, ax = plt.subplots(figsize=(8, 4.6))
    positions = np.arange(len(bins))
    width = 0.36
    bp_o = ax.boxplot([data[b]["opencv"] for b in bins],
                      positions=positions - width / 2, widths=width * 0.85, patch_artist=True,
                      boxprops=dict(facecolor=OPENCV_COLOR, alpha=0.6), showfliers=True,
                      medianprops=dict(color="black"))
    bp_s = ax.boxplot([data[b]["sentai"] for b in bins],
                      positions=positions + width / 2, widths=width * 0.85, patch_artist=True,
                      boxprops=dict(facecolor=SENTAI_COLOR, alpha=0.6), showfliers=True,
                      medianprops=dict(color="black"))
    ax.set_yscale("log")
    ax.set_ylim(1e-3, 2)
    ax.set_xticks(positions)
    ax.set_xticklabels([f"{b}-marker" for b in bins])
    ax.set_xlabel("Number of evaluation-visible markers per frame")
    ax.set_ylabel("Translation error vs GT (m, log)")
    ax.set_title("Pose error by visible-marker count — weak-geometry penalty")
    ax.axhline(0.05, color="0.5", linewidth=0.7, linestyle=":")
    ax.axhline(1.0, color="0.8", linewidth=0.7, linestyle=":")
    legend_handles = [bp_o["boxes"][0], bp_s["boxes"][0]]
    ax.legend(legend_handles, ["OpenCV (exhaustive perm.)", "sentai_sim (prior-guided)"], loc="upper right")
    ax.grid(alpha=0.3, which="both", axis="y")
    fig.tight_layout()
    fig.savefig(out / "fig4_translation_by_marker_count.png", dpi=140)
    plt.close(fig)


def fig_false_positives_by_z(results: list[dict], out: Path) -> None:
    by_z: dict[float, dict[str, list[int]]] = defaultdict(lambda: {"opencv": [], "sentai": [], "frames": []})
    for r in results:
        z = round(r["pose_world_camera"]["z"], 2)
        bo = r["backends"].get("opencv", {}) or {}
        bs = r["backends"].get("sentai_sim", {}) or {}
        by_z[z]["opencv"].append(bo.get("false_positive_count", 0))
        by_z[z]["sentai"].append(bs.get("false_positive_count", 0) if bs.get("available", False) else 0)
        by_z[z]["frames"].append(1)
    zs = sorted(by_z.keys())
    opencv_fp = [sum(by_z[z]["opencv"]) for z in zs]
    sentai_fp = [sum(by_z[z]["sentai"]) for z in zs]
    nframes = [sum(by_z[z]["frames"]) for z in zs]

    fig, ax = plt.subplots(figsize=(7, 4.2))
    x = np.arange(len(zs))
    w = 0.36
    ax.bar(x - w / 2, opencv_fp, w, label=f"OpenCV  ({sum(opencv_fp)} total)", color=OPENCV_COLOR)
    ax.bar(x + w / 2, sentai_fp, w, label=f"sentai_sim  ({sum(sentai_fp)} total)", color=SENTAI_COLOR)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{z:.2f} m\n({n} frames)" for z, n in zip(zs, nframes)])
    ax.set_ylabel("False positives (count)")
    ax.set_xlabel("Camera altitude")
    ax.set_title("False positives by altitude — sentai_sim FP rises at z=1.0 m")
    ax.legend(loc="upper left")
    ax.grid(axis="y", alpha=0.3)
    for i, (o, s) in enumerate(zip(opencv_fp, sentai_fp)):
        if o:
            ax.text(i - w / 2, o + 0.2, str(o), ha="center", va="bottom", fontsize=9, color=OPENCV_COLOR)
        if s:
            ax.text(i + w / 2, s + 0.2, str(s), ha="center", va="bottom", fontsize=9, color=SENTAI_COLOR)
    fig.tight_layout()
    fig.savefig(out / "fig5_false_positives_by_altitude.png", dpi=140)
    plt.close(fig)


def fig_centroid_parity_scatter(results: list[dict], out: Path) -> None:
    pairs: list[tuple[float, float, float]] = []
    for r in results:
        bo = r["backends"].get("opencv", {}) or {}
        bs = r["backends"].get("sentai_sim", {}) or {}
        if not bs.get("available", False):
            continue
        z = r["pose_world_camera"]["z"]
        sentai_by_id = {m["marker_id"]: m for m in (bs.get("matches", []) or []) if "marker_id" in m}
        for m in (bo.get("matches", []) or []):
            mid = m.get("marker_id")
            if mid is None or mid not in sentai_by_id:
                continue
            o_err = m.get("centroid_error_px")
            s_err = sentai_by_id[mid].get("centroid_error_px")
            if o_err is None or s_err is None:
                continue
            pairs.append((o_err, s_err, z))
    if not pairs:
        return
    arr = np.array(pairs)
    o, s, z = arr[:, 0], arr[:, 1], arr[:, 2]

    fig, ax = plt.subplots(figsize=(6, 5.6))
    sc = ax.scatter(o, s, c=z, cmap="viridis", s=10, alpha=0.6, edgecolors="none")
    lim = max(o.max(), s.max()) * 1.05
    ax.plot([0, lim], [0, lim], color="0.4", linewidth=0.8, linestyle="--", label="y = x")
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    ax.set_xlabel("OpenCV centroid error (px)")
    ax.set_ylabel("sentai_sim centroid error (px)")
    ax.set_title(f"Per-marker centroid parity — n={len(pairs)} matched pairs")
    ax.set_aspect("equal")
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("Camera z (m)")
    ax.legend(loc="upper left")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "fig6_centroid_parity_scatter.png", dpi=140)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("validation_dir", type=Path)
    p.add_argument("--out", type=Path, default=None,
                   help="Output directory (default: <validation_dir>/figures/)")
    args = p.parse_args()

    out = args.out if args.out is not None else args.validation_dir / "figures"
    out.mkdir(parents=True, exist_ok=True)

    results, summary = load_results(args.validation_dir)
    print(f"loaded {len(results)} frames from {args.validation_dir}")

    fig_recall_by_z(results, out)
    fig_centroid_distribution(results, out)
    fig_translation_cdf(results, out)
    fig_translation_vs_marker_count(results, out)
    fig_false_positives_by_z(results, out)
    fig_centroid_parity_scatter(results, out)

    print(f"wrote 6 figures to {out}")
    for f in sorted(out.glob("fig*.png")):
        print(f"  {f.name}")


if __name__ == "__main__":
    main()

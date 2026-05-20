#!/usr/bin/env python3
"""s182_replay — local detection vs drone-side comparison.

WBS: OP-S10-W19-T4 iter-6.  Pattern mirrors s178_cv2_vs_ours_dump but
inverted: instead of digging into ONE frame at sub-stage level, we
sweep ALL frames captured by sentai.fr during the mission and compare
the drone-side WhyCon count against a cv2-based ground-truth detector
running locally on the same PGMs.

Inputs:
  --frames-dir : path to fr_s182/frames/ containing tNNNNNNNN_nN_fNNNNNN.pgm
                 The 'n<N>' in the filename is the drone-side n_dets
                 (sentai_markers_detect_frame's s_cache_n at the
                 moment of FR push).
  --out-dir    : where to dump comparison CSV + side-by-side PNGs.
  --tag        : a string appended to output filenames.
  --limit      : optional cap on number of frames to process.
  --viz-every  : write a viz PNG every Nth frame (default: 10).

Outputs:
  s182_replay_<tag>.csv  — one row per frame: ts_ms, seq, n_drone,
                            n_cv2, n_match, drone_circles_json
  s182_replay_<tag>_dist.png — histogram of (n_drone - n_cv2)
  s182_replay_<tag>_overlay_*.png — per-frame visualisations

The cv2 detector here is HoughCircles tuned for the bilinear-rendered
Krajník annulus that Gazebo + PBR shading actually produces (NOT a
sharp annulus — see SOTA_CITATIONS.md "annulus correction factor"
caveat).  This is intentionally NOT a re-implementation of the C
WhyCon pipeline (Bradley + 8-conn + moments) — that would require
porting hundreds of lines of C to Python.  Instead, cv2.HoughCircles
gives us an INDEPENDENT, well-validated baseline: if drone and cv2
agree, both are right; if they diverge, we have a frame to inspect.

This is the same methodology as s178 (cv2 ArUco baseline) but for
WhyCon-class markers.
"""
from __future__ import annotations
import argparse
import pathlib
import re
import sys

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


FNAME_RE = re.compile(
    r"t(\d+)_n(-?\d+)_f(\d+)\.pgm$",
)


def read_pgm(path: pathlib.Path) -> np.ndarray:
    """Read P5 PGM into uint8 H×W array.  Robust to embedded comments."""
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"not a P5 PGM: {path}")
        # Skip comment lines.
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = (int(x) for x in line.split())
        maxv = int(f.readline().strip())
        if maxv != 255:
            raise ValueError(f"unexpected maxval {maxv} in {path}")
        data = f.read(w * h)
        if len(data) != w * h:
            raise ValueError(
                f"short read on {path}: got {len(data)}, expected {w*h}")
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w)


def detect_cv2_hough(img: np.ndarray) -> list[tuple[float, float, float]]:
    """Detect WhyCon-class circular markers via HoughCircles.

    Returns: list of (cx, cy, radius) in pixels.

    Parameters chosen for Krajník-cross @ 320×240, marker outer
    diameter 0.108 m, drone altitude 0.4-1.0 m → projected radius
    range ~15-40 px.  PBR + bilinear renders the annulus as a near-
    solid disc, so we use HOUGH_GRADIENT (edge-based) with a moderate
    accumulator threshold.
    """
    blurred = cv2.GaussianBlur(img, (5, 5), sigmaX=1.0)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=15,
        param1=80,    # Canny upper threshold
        param2=18,    # Accumulator threshold (lower → more detections)
        minRadius=8,
        maxRadius=50,
    )
    if circles is None:
        return []
    return [(float(c[0]), float(c[1]), float(c[2]))
             for c in circles[0]]


def parse_filename(path: pathlib.Path) -> tuple[int, int, int] | None:
    """Extract (ts_ms, n_drone, seq) from FR filename, or None."""
    m = FNAME_RE.search(path.name)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2)), int(m.group(3))


def viz_one(img: np.ndarray, cv2_circles, n_drone: int,
             out_path: pathlib.Path, title: str):
    """Draw the original frame + cv2-detected circles, side by side."""
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].imshow(img, cmap="gray", vmin=0, vmax=255)
    axes[0].set_title(f"raw  ({img.shape[1]}×{img.shape[0]})")
    axes[0].axis("off")

    axes[1].imshow(img, cmap="gray", vmin=0, vmax=255)
    for (cx, cy, r) in cv2_circles:
        circ = plt.Circle((cx, cy), r, fill=False, color="lime",
                           linewidth=1.3)
        axes[1].add_patch(circ)
        axes[1].plot(cx, cy, "+", color="red", markersize=6)
    axes[1].set_title(
        f"cv2 HoughCircles: {len(cv2_circles)}   "
        f"drone n={n_drone}")
    axes[1].axis("off")
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--frames-dir", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--tag", default="iter6")
    ap.add_argument("--limit", type=int, default=0,
                     help="cap frames processed (0 = all)")
    ap.add_argument("--viz-every", type=int, default=10,
                     help="save side-by-side PNG every Nth frame")
    args = ap.parse_args()

    frames_dir = pathlib.Path(args.frames_dir)
    out_dir = pathlib.Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    pgms = sorted(p for p in frames_dir.glob("*.pgm")
                   if FNAME_RE.search(p.name))
    if not pgms:
        sys.exit(f"no PGMs matching FR pattern under {frames_dir}")

    if args.limit > 0:
        pgms = pgms[:args.limit]

    print(f"[s182-replay] {len(pgms)} frames under {frames_dir}")
    rows = []
    for idx, pgm in enumerate(pgms):
        meta = parse_filename(pgm)
        if meta is None:
            continue
        ts_ms, n_drone, seq = meta
        try:
            img = read_pgm(pgm)
        except Exception as ex:
            print(f"[s182-replay] skip {pgm.name}: {ex}")
            continue
        cv2_circles = detect_cv2_hough(img)
        n_cv2 = len(cv2_circles)

        rows.append({
            "ts_ms":   ts_ms,
            "seq":     seq,
            "n_drone": n_drone,
            "n_cv2":   n_cv2,
            "delta":   n_drone - n_cv2,
            "fname":   pgm.name,
        })

        if args.viz_every > 0 and (idx % args.viz_every == 0):
            viz_path = out_dir / (
                f"s182_replay_{args.tag}_overlay_{seq:06d}.png")
            viz_one(img, cv2_circles, n_drone, viz_path,
                     title=f"{pgm.name}  drone={n_drone}  cv2={n_cv2}")
        if idx % 50 == 0:
            print(f"[s182-replay] {idx}/{len(pgms)} "
                  f"(last: drone={n_drone} cv2={n_cv2})")

    # ---- CSV ---------------------------------------------------------
    csv_path = out_dir / f"s182_replay_{args.tag}.csv"
    cols = ["ts_ms", "seq", "n_drone", "n_cv2", "delta", "fname"]
    with csv_path.open("w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r[c]) for c in cols) + "\n")
    print(f"[s182-replay] wrote {csv_path}")
    if not rows:
        return

    # ---- Summary stats ----------------------------------------------
    n_total = len(rows)
    n_agree = sum(1 for r in rows if r["delta"] == 0)
    n_drone_more = sum(1 for r in rows if r["delta"] > 0)
    n_drone_less = sum(1 for r in rows if r["delta"] < 0)
    avg_drone = sum(r["n_drone"] for r in rows) / n_total
    avg_cv2 = sum(r["n_cv2"] for r in rows) / n_total
    print(f"[s182-replay] frames={n_total}  agree={n_agree}  "
          f"drone>cv2={n_drone_more}  drone<cv2={n_drone_less}")
    print(f"[s182-replay] avg drone n={avg_drone:.2f}  "
          f"avg cv2 n={avg_cv2:.2f}")

    # ---- Histogram + time-series ------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    deltas = [r["delta"] for r in rows]
    bins = range(min(deltas) - 1, max(deltas) + 2)
    axes[0].hist(deltas, bins=bins, color="tab:blue",
                  edgecolor="black", alpha=0.7)
    axes[0].axvline(0, color="red", lw=1, ls="--")
    axes[0].set_xlabel("n_drone − n_cv2  (per frame)")
    axes[0].set_ylabel("frame count")
    axes[0].set_title(
        f"agreement: {n_agree}/{n_total} = "
        f"{100*n_agree/n_total:.0f} %")
    axes[0].grid(True, alpha=0.3)

    xs = list(range(len(rows)))
    axes[1].plot(xs, [r["n_drone"] for r in rows], "-",
                  color="tab:blue", lw=1.0, label="drone (WhyCon)")
    axes[1].plot(xs, [r["n_cv2"] for r in rows], "-",
                  color="tab:orange", lw=1.0, label="cv2 (Hough)")
    axes[1].set_xlabel("frame index")
    axes[1].set_ylabel("n_dets")
    axes[1].set_title("per-frame marker count over mission")
    axes[1].legend(loc="upper right", fontsize=9)
    axes[1].grid(True, alpha=0.3)

    fig.suptitle(
        f"s182 replay {args.tag} — drone WhyCon vs cv2 HoughCircles "
        f"({n_total} frames)",
        fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    dist_path = out_dir / f"s182_replay_{args.tag}_dist.png"
    fig.savefig(dist_path, dpi=130)
    print(f"[s182-replay] wrote {dist_path}")


if __name__ == "__main__":
    main()

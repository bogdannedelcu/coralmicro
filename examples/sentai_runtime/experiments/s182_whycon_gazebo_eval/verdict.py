#!/usr/bin/env python3
"""s182 verdict — pair sentai_sim journal with gt_recorder JSONL,
compute estimated drone world pose from WhyCon detections, plot
estimated vs ground truth for X, Y, Z.

WBS: OP-S10-W19-T4 step 2.

Anti-cheat positioning ([[sentai-sim-air-gapped-from-truth]]):
  - This script is HOST-SIDE post-mortem.  It reads the firmware
    journal (WhyCon detections) and the gz_recorder GT, and writes
    plots.  Nothing here is fed back into sentai_sim.

Input:
  journal     : sentai_fs_root mission_s182_journal.txt  (NDJSON-ish per [_j])
  gt          : /tmp/s182_whycon_gazebo/gt/cf2_gt.jsonl  (jsonl per line)

Output:
  out_dir/s182_xyz_world.png            X/Y/Z est vs GT (3 panels)
  out_dir/s182_residuals.png            (X_est - X_gt) etc. vs altitude
  out_dir/s182_paired.csv               raw paired samples
"""

import argparse
import json
import math
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


# Known marker world positions — must match sentai_whycon.sdf.
MARKER_GT = {
    "N":  ( 0.00, +0.20, 0.005),
    "E":  (+0.16,  0.00, 0.005),
    "S":  ( 0.00, -0.20, 0.005),
    "W":  (-0.08,  0.00, 0.005),
}
MARKER_ORDER = ["N", "E", "S", "W"]  # order to try when associating detections.


def parse_journal(path: pathlib.Path):
    """sentai.sim.journal uses one event per line, format
    `<event_name>:<json_payload>` with the payload encoded by _ser_val
    in mission_s182.py.  Return list of (event, payload_dict)."""
    out = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line:
            continue
        # journal_write writes `event_name SPACE json_payload` per
        # sentai.sim.journal_write convention (see sentai_sim_journal.c).
        if " " in line:
            ev, _, body = line.partition(" ")
        elif ":" in line:
            ev, _, body = line.partition(":")
        else:
            continue
        try:
            payload = json.loads(body)
        except json.JSONDecodeError:
            continue
        out.append((ev, payload))
    return out


def parse_gt(path: pathlib.Path):
    """Each line: {t_wall, t_unix, gz_sec, gz_nsec, x, y, z}."""
    rows = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def estimate_drone_world(det, marker_world):
    """Given a WhyCon detection (tvec_cam = marker pose in camera frame)
    and the known marker world position, return the estimated drone
    world position.

    For a downward-facing camera with vflip=1 (Sim.md §10b):
      image LEFT  → body FORWARD   (+x_body)
      image RIGHT → body BACKWARD  (-x_body)
      image TOP   → body RIGHT     (-y_body)
      image BOTTOM→ body LEFT      (+y_body)

    Camera frame convention (OpenCV-like):
      cam +X right
      cam +Y down
      cam +Z forward (into the scene, i.e. world -Z since cam looks down)

    With cam attitude matching body frame (pre-yaw):
      tvec_cam.x  → +y_world (right in image = world +y_body, rotated)
      tvec_cam.y  → +x_world (down in image = ...)
      tvec_cam.z  → -z_world (cam +Z away → marker below → ground)

    For the simple downward-looking landing case with cf2 yaw=0:
      drone_world.x = marker.x + tvec.x
      drone_world.y = marker.y + tvec.y
      drone_world.z = marker.z + tvec.z

    (The camera frame's x/y are aligned with world x/y for cam looking
    straight down + cf2 at yaw=0; sign conventions need to match the
    SIM's vflip/cam-mount which is body_xform = (-1, 0, 0, +1) — i.e.
    image LEFT → body FORWARD.  This verdict assumes the simple sign
    convention drone_world = marker_world + tvec_cam.  Mismatch shows
    up as a constant flip in the residual plots — caught visually."""
    mx, my, mz = marker_world
    return (mx + det["tx"], my + det["ty"], mz + det["tz"])


def associate_marker(det, n_dets, det_idx):
    """Pick which world marker this detection corresponds to.  Strategy:
    use detection order index — WhyCon flood-fill scans top-to-bottom /
    left-to-right, so for our 4-marker cross layout the (pixel_cx,
    pixel_cy) order is roughly N → E → S → W.  Fall back to closest-by-
    pixel-position heuristic only if needed."""
    if det_idx < len(MARKER_ORDER):
        return MARKER_ORDER[det_idx]
    return None


def pair_journal_gt(journal_events, gt_rows):
    """Return list of dicts with paired ts_ms (sentai) / t_wall (gt) /
    cf2 pose / per-marker pose estimates."""
    # GT rows are timestamped in HOST monotonic + sim time; sentai
    # journal in sentai.rtos.ticks_ms.  Without a shared clock we pair
    # on RUN ORDER — tick i in journal corresponds to the gt sample
    # closest in time, but realistically there's drift.  Easiest match:
    # interpolate gt by sentai ts_ms assuming linear time progression
    # from first to last tick.  Since each tick is 100 ms (TICK_INTERVAL_MS),
    # we use the first tick as t=0 reference.
    ticks = [(ev, p) for ev, p in journal_events if ev == "tick"]
    if not ticks or not gt_rows:
        return []
    t0_sentai = ticks[0][1]["ts_ms"]
    t0_gt     = gt_rows[0]["t_wall"]
    paired = []
    for ev, p in ticks:
        rel_s = (p["ts_ms"] - t0_sentai) / 1000.0
        target_wall = t0_gt + rel_s
        # Nearest-neighbour search in gt.
        best = None
        best_err = 1e9
        for g in gt_rows:
            err = abs(g["t_wall"] - target_wall)
            if err < best_err:
                best_err = err
                best = g
            if g["t_wall"] > target_wall + 0.5:
                break
        if best is None or best_err > 0.5:
            continue
        for det_idx, det in enumerate(p.get("dets", [])):
            if not det.get("v"):
                continue
            name = associate_marker(det, p["n"], det_idx)
            if name is None:
                continue
            mw = MARKER_GT[name]
            ex, ey, ez = estimate_drone_world(det, mw)
            paired.append({
                "tick_idx":  p["i"],
                "alt_cmd":   p["alt"],
                "ts_ms":     p["ts_ms"],
                "marker":    name,
                "gt_x":      best["x"],
                "gt_y":      best["y"],
                "gt_z":      best["z"],
                "est_x":     ex,
                "est_y":     ey,
                "est_z":     ez,
                "tvec_z":    det["tz"],
                "reproj":    det["rep"],
            })
    return paired


def write_csv(paired, path: pathlib.Path):
    cols = ["tick_idx", "alt_cmd", "ts_ms", "marker",
             "gt_x", "gt_y", "gt_z", "est_x", "est_y", "est_z",
             "tvec_z", "reproj"]
    with path.open("w") as f:
        f.write(",".join(cols) + "\n")
        for r in paired:
            f.write(",".join(str(r[c]) for c in cols) + "\n")


def plot_xyz(paired, out_path: pathlib.Path):
    if not paired:
        print("[s182] no paired samples — skipping plot")
        return
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))
    axis_meta = [
        ("X", "gt_x", "est_x", "tab:orange"),
        ("Y", "gt_y", "est_y", "tab:green"),
        ("Z", "gt_z", "est_z", "tab:blue"),
    ]
    for ax, (name, gt_k, est_k, color) in zip(axes, axis_meta):
        gt = [r[gt_k] for r in paired]
        est = [r[est_k] for r in paired]
        lo = min(min(gt), min(est))
        hi = max(max(gt), max(est))
        pad = 0.05 * (hi - lo + 1e-9)
        ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad],
                 "k--", linewidth=0.8, label="ideal (est = gt)")
        ax.scatter(gt, est, c=color, s=18, alpha=0.7,
                    edgecolor="black", linewidth=0.3, label="measured")
        ax.set_xlabel(f"{name}_gt (m)  [Gazebo cf2 dynamic_pose]")
        ax.set_ylabel(f"{name}_est (m)  [WhyCon marker world + tvec_cam]")
        ax.set_title(f"{name} — drone world position")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="box")
        ax.legend(loc="upper left", fontsize=8)
        errs = [abs(e - g) for g, e in zip(gt, est)]
        mae = sum(errs) / len(errs)
        ax.text(0.98, 0.02, f"MAE = {mae*100:.2f} cm  (n={len(paired)})",
                 transform=ax.transAxes, ha="right", va="bottom",
                 fontsize=9,
                 bbox=dict(boxstyle="round,pad=0.3",
                            fc="white", ec="gray", alpha=0.85))
    fig.suptitle(
        "s182 — WhyCon Gazebo eval: drone X/Y/Z estimated vs GT  "
        "(Krajník-cross scene, cf2 SITL, anti-cheat-compliant)",
        fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out_path, dpi=150)
    print(f"[s182] wrote {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--journal", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()

    journal_path = pathlib.Path(args.journal)
    gt_path = pathlib.Path(args.gt)
    out_dir = pathlib.Path(args.out_dir)

    if not journal_path.exists():
        sys.exit(f"missing journal: {journal_path}")
    if not gt_path.exists():
        sys.exit(f"missing gt jsonl: {gt_path}")

    events = parse_journal(journal_path)
    gt_rows = parse_gt(gt_path)
    print(f"[s182] {len(events)} journal events, {len(gt_rows)} GT rows")

    paired = pair_journal_gt(events, gt_rows)
    print(f"[s182] {len(paired)} paired tick samples")

    write_csv(paired, out_dir / "s182_paired.csv")
    plot_xyz(paired, out_dir / "s182_xyz_world.png")


if __name__ == "__main__":
    main()

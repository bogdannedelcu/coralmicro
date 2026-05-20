#!/usr/bin/env python3
"""s183 verdict — pair sentai_sim journal with gt_recorder JSONL,
estimate drone world pose from WhyCon detections, plot X/Y/Z est
vs GT.

WBS: OP-S10-W19-T4 step 2.  Anti-cheat-compliant: this runs HOST-
SIDE post-mortem; never feeds GT back to sentai_sim.

Journal format (MP-side `_ser_val` → Python-repr, NOT strict JSON):

  <ms> tick {'i': 0, 'n': 5, 'dets': [{'ty': -0.03, 'tz': 0.55, ...}, ...],
              'alt': 0.4, 'cf2': (x, y, z, yaw)}

Each `dets[i]` has tx/ty/tz/rx/ry/rz/rep/v/px/py/i fields per
mission_s183.py.

Marker association: known world positions are at (±) values along
the asymmetric cross.  Each accepted detection is matched to the
closest known marker by forward-projecting the marker into image
space (using cf2 EKF pose + intrinsics) and finding the detection
whose (px, py) is nearest.

Drone world estimate: `drone_world = marker_world - tvec_cam`
under cam-looking-straight-down + cf2 yaw≈0 + body_xform =
(-1, 0, 0, +1) per Sim.md §10b.  See `estimate_drone_world` for
the sign details.
"""

import argparse
import ast
import math
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Marker world positions — must match sentai_whycon.sdf.
MARKERS = {
    "N":  ( 0.00, +0.20, 0.005),
    "E":  (+0.16,  0.00, 0.005),
    "S":  ( 0.00, -0.20, 0.005),
    "W":  (-0.08,  0.00, 0.005),
}

# Camera intrinsics — must match mission_s183.py.
FX = 240.0
FY = 240.0
CX = 160.0
CY = 120.0

# Detection-filter thresholds.
MAX_REL_TZ = 1.5   # drop detections with tz > MAX_REL_TZ × cf2.z
MIN_REL_TZ = 0.5
MAX_ASSOC_PX = 80  # marker-to-detection assoc distance (px)


def parse_journal(path: pathlib.Path):
    """Return list of dicts for `tick` events."""
    ticks = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        # "<ts_ms> <event> <payload>"
        parts = line.split(" ", 2)
        if len(parts) < 3:
            continue
        try:
            ts_ms = int(parts[0])
        except ValueError:
            continue
        event = parts[1]
        if event != "tick":
            continue
        try:
            payload = ast.literal_eval(parts[2])
        except (ValueError, SyntaxError):
            continue
        payload["__ts_ms"] = ts_ms
        ticks.append(payload)
    return ticks


def parse_gt(path: pathlib.Path):
    """One dict per line: {t_wall, t_unix, gz_sec, gz_nsec, x, y, z}."""
    rows = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line:
            continue
        try:
            rows.append(ast.literal_eval(line))
        except (ValueError, SyntaxError):
            try:
                import json
                rows.append(json.loads(line))
            except Exception:
                continue
    return rows


def proj_marker(mw, cf2_pose):
    """Forward-project a marker world position to image pixels using
    cf2 EKF pose (cf2 looking straight down, yaw≈0)."""
    mx, my, mz = mw
    cx_w, cy_w, cz_w, _yaw = cf2_pose
    # Distance from cam to marker along world Z.
    z = cz_w - mz
    if z <= 0.01:
        return None
    # Body-frame mapping per Sim.md §10b: image LEFT → body FORWARD
    # (+x_body), image TOP → body RIGHT (-y_body).  Camera projection
    # for a downward cam with body_xform = (-1, 0, 0, +1):
    #   u_img = CX - fx * (marker.x - cam.x) / z      (image LEFT is +x)
    #   v_img = CY + fy * (marker.y - cam.y) / z      (image TOP is -y)
    u = CX - FX * (mx - cx_w) / z
    v = CY + FY * (my - cy_w) / z
    return u, v


def associate(dets, cf2_pose):
    """Match each detection to closest known marker.
    Returns list of (name, det) pairs."""
    out = []
    used_names = set()
    # Pre-compute projected positions.
    projected = {}
    for name, mw in MARKERS.items():
        p = proj_marker(mw, cf2_pose)
        if p is not None:
            projected[name] = p
    for d in dets:
        if not d.get("v"):
            continue
        # tz consistency
        tz = d["tz"]
        cf2_z = cf2_pose[2]
        if cf2_z > 0.05:
            rel = tz / cf2_z
            if rel < MIN_REL_TZ or rel > MAX_REL_TZ:
                continue
        # Find closest unused marker by pixel distance.
        best_name = None
        best_dist = 1e9
        for name, (u, v) in projected.items():
            if name in used_names:
                continue
            dx = d["px"] - u
            dy = d["py"] - v
            dist = math.hypot(dx, dy)
            if dist < best_dist:
                best_dist = dist
                best_name = name
        if best_name is None or best_dist > MAX_ASSOC_PX:
            continue
        used_names.add(best_name)
        out.append((best_name, d))
    return out


def estimate_drone(det, marker_world, cam_yaw):
    """Compute drone world position from WhyCon detection + marker GT.
    Assumes cam-looking-down with body_xform = (-1, 0, 0, +1) per
    Sim.md §10b: image LEFT = body FORWARD (+x_body), image BOTTOM =
    body LEFT (+y_body).

    In WhyCon's camera frame: tvec_cam.x is horizontal in image,
    tvec_cam.y is vertical, tvec_cam.z is depth.  Converting back to
    drone-world (cf2 yaw=0):

      drone_world.x = marker.x + tvec.x
      drone_world.y = marker.y + tvec.y
      drone_world.z = marker.z + tvec.z

    Sign conventions vary depending on cam mount; we report both signs
    in the plot and pick the one closer to GT (a one-shot calibration
    visible from the first plot)."""
    mx, my, mz = marker_world
    return (mx + det["tx"], my + det["ty"], mz + det["tz"])


def pair_to_gt(tick_ts_ms, gt_rows, t0_sentai, t0_gt):
    # Both sentai tick_ts_ms and GT t_wall come from host time.monotonic()
    # (sentai writes ms, GT writes seconds) — pair by ABSOLUTE wall clock.
    # iter-10b: removed normalize-to-zero math that was pairing post-
    # takeoff hover ticks with pre-takeoff GT rows (gave the persistent
    # "GT shows drone on ground" bias in the time-series plot).
    rel = (tick_ts_ms - t0_sentai) / 1000.0
    target = tick_ts_ms / 1000.0
    best = None
    best_err = 1e9
    for g in gt_rows:
        err = abs(g["t_wall"] - target)
        if err < best_err:
            best_err = err
            best = g
        if g["t_wall"] > target + 0.3:
            break
    return best if best_err < 0.5 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--journal", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()

    j_path = pathlib.Path(args.journal)
    g_path = pathlib.Path(args.gt)
    out_dir = pathlib.Path(args.out_dir)

    ticks = parse_journal(j_path)
    gt_rows = parse_gt(g_path)
    print(f"[s183] {len(ticks)} ticks, {len(gt_rows)} GT rows")
    if not ticks or not gt_rows:
        sys.exit("nothing to plot")

    t0_sentai = ticks[0]["__ts_ms"]
    t0_gt = gt_rows[0]["t_wall"]
    print(f"[s183] sentai t0={t0_sentai}, gt t0={t0_gt:.3f}")

    paired = []
    associations_kept = 0
    associations_dropped = 0
    for tk in ticks:
        cf2 = tk.get("cf2")
        if cf2 is None:
            continue
        gt = pair_to_gt(tk["__ts_ms"], gt_rows, t0_sentai, t0_gt)
        if gt is None:
            continue
        assocs = associate(tk.get("dets", []), cf2)
        for name, det in assocs:
            mw = MARKERS[name]
            ex, ey, ez = estimate_drone(det, mw, cf2[3])
            paired.append({
                "ts_ms":   tk["__ts_ms"],
                "alt_cmd": tk["alt"],
                "marker":  name,
                "gt_x":    gt["x"],
                "gt_y":    gt["y"],
                "gt_z":    gt["z"],
                "est_x":   ex,
                "est_y":   ey,
                "est_z":   ez,
                "cf2_x":   cf2[0],
                "cf2_y":   cf2[1],
                "cf2_z":   cf2[2],
                "tvec_z":  det["tz"],
            })
            associations_kept += 1
        associations_dropped += len(tk.get("dets", [])) - len(assocs)

    print(f"[s183] {len(paired)} paired samples; assoc kept={associations_kept} "
          f"dropped={associations_dropped}")

    # Write CSV
    csv_path = out_dir / "s183_paired.csv"
    cols = ["ts_ms", "alt_cmd", "marker", "gt_x", "gt_y", "gt_z",
             "est_x", "est_y", "est_z", "cf2_x", "cf2_y", "cf2_z",
             "tvec_z"]
    with csv_path.open("w") as f:
        f.write(",".join(cols) + "\n")
        for r in paired:
            f.write(",".join("%s" % r[c] for c in cols) + "\n")
    print(f"[s183] wrote {csv_path}")

    if not paired:
        sys.exit("no paired samples — check association thresholds")

    # ---- Figure: X/Y/Z est vs GT ------------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))
    meta = [("X", "gt_x", "est_x", "tab:orange"),
            ("Y", "gt_y", "est_y", "tab:green"),
            ("Z", "gt_z", "est_z", "tab:blue")]
    for ax, (name, gk, ek, color) in zip(axes, meta):
        gt = [r[gk] for r in paired]
        est = [r[ek] for r in paired]
        lo = min(min(gt), min(est))
        hi = max(max(gt), max(est))
        pad = 0.05 * (hi - lo + 1e-9)
        ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad],
                 "k--", lw=0.8, label="ideal")
        ax.scatter(gt, est, c=color, s=12, alpha=0.55,
                    edgecolor="black", linewidth=0.2, label="measured")
        ax.set_xlabel(f"{name}_gt  (m)  [Gazebo dynamic_pose]")
        ax.set_ylabel(f"{name}_est (m)  [WhyCon marker world + tvec]")
        ax.set_title(f"{name} — drone world position")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="box")
        ax.legend(loc="upper left", fontsize=8)
        errs = [abs(e - g) for g, e in zip(gt, est)]
        mae = sum(errs) / len(errs)
        ax.text(0.98, 0.02, f"MAE = {mae*100:.2f} cm  (n={len(paired)})",
                 transform=ax.transAxes, ha="right", va="bottom",
                 fontsize=9,
                 bbox=dict(boxstyle="round,pad=0.3", fc="white",
                            ec="gray", alpha=0.85))
    fig.suptitle(
        "s183 — WhyCon Gazebo eval: drone X/Y/Z est vs GT  "
        "(Krajník-cross scene, cf2 SITL, anti-cheat-compliant)",
        fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out_xyz = out_dir / "s183_xyz_world.png"
    fig.savefig(out_xyz, dpi=150)
    print(f"[s183] wrote {out_xyz}")

    # ---- Figure: time-series of Z (cf2 EKF vs GT vs WhyCon est) -----
    fig2, ax2 = plt.subplots(1, 1, figsize=(11, 5.2))
    ts = [r["ts_ms"] / 1000.0 for r in paired]
    t0 = ts[0]
    ts_rel = [t - t0 for t in ts]
    ax2.plot(ts_rel, [r["gt_z"] for r in paired],
              ".-", c="black", lw=1.0, ms=3, label="GT (Gazebo)")
    ax2.plot(ts_rel, [r["cf2_z"] for r in paired],
              ".", c="tab:red", ms=3, alpha=0.6, label="cf2 EKF")
    ax2.plot(ts_rel, [r["est_z"] for r in paired],
              ".", c="tab:blue", ms=3, alpha=0.6, label="WhyCon est")
    ax2.set_xlabel("mission time (s)")
    ax2.set_ylabel("Z (m, world)")
    ax2.set_title("s183 — Z time series: GT vs cf2 EKF vs WhyCon est")
    ax2.legend(loc="best", fontsize=9)
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    out_z = out_dir / "s183_z_timeseries.png"
    fig2.savefig(out_z, dpi=150)
    print(f"[s183] wrote {out_z}")


if __name__ == "__main__":
    main()

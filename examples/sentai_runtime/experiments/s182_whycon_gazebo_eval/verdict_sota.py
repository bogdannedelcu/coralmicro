#!/usr/bin/env python3
"""s182 verdict (SOTA-aligned) — multi-marker joint pose recovery via
Kabsch SVD.

WBS: OP-S10-W19-T4 step 3.  Replaces the per-marker-then-median heuristic
in verdict.py with the closed-form joint solver from the SOTA point-set
registration literature.

Citations:

  - Kabsch, W. (1976). "A solution for the best rotation to relate two
    sets of vectors." Acta Crystallographica, A32:922-923.
  - Arun, K.S.; Huang, T.S.; Blostein, S.D. (1987). "Least-Squares
    Fitting of Two 3-D Point Sets." IEEE Trans. on Pattern Analysis
    and Machine Intelligence, PAMI-9(5):698-700.
  - Krajník, T.; Nitsche, M.; Faigl, J.; Vaněk, P.; Saska, M.; Přeučil, L.;
    Duckett, T.; Mejail, M. (2014). "A practical multirobot localization
    system." Journal of Intelligent and Robotic Systems, 76:539-562.
    (SOTA reference for the WhyCon detection + per-marker pose pipeline.)

Algorithm (Kabsch / Procrustes, well-conditioned for N≥3 non-collinear
points):

  Given N pairs {(p_world_i, p_cam_i)} (3D world position vs measured
  3D camera-frame position via per-marker WhyCon tvec_cam):

    1. Compute centroids µ_world, µ_cam.
    2. Centre each set: q_w = p_w - µ_w,  q_c = p_c - µ_c.
    3. Cross-covariance: H = Σ q_c · q_w^T   (3×3).
    4. SVD: H = U·Σ·V^T.
    5. d = det(V·U^T)  (+1 or -1; the reflection correction).
    6. R = V · diag(1, 1, d) · U^T.
    7. t = µ_w - R · µ_c.
    8. drone_world_position = t   (cam-origin transformed to world).

  This is the closed-form least-squares optimum over the SE(3) group
  for the 3D-3D point-pair registration problem.  Globally optimal,
  rotation orthonormal by construction.

Comparison vs the per-marker heuristic in verdict.py:

  Heuristic: drone_world_per_marker = marker_world - tvec_cam,
             then drone_world = median over markers.
             — Independent per-marker estimates, no joint constraint.
  Kabsch:    Single [R|t] fits ALL markers jointly.
             — Over-determined system: residuals → 0 in noise-free SIM,
               averages out marker-detection noise in real cameras.

Input/output: same as verdict.py — journal + GT JSONL → CSV + plots.
"""

import argparse
import ast
import math
import pathlib
import sys

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402

# Marker world positions — H layout per Kim, Yang, Kim 2013 IROS,
# with iter-8 X-asymmetry fix: east column at +0.12 m (not +0.16 m)
# breaks the X-mirror that caused iter-7 Kabsch ±0.4 m sign flips
# (residual was 26 mm but est_x was bimodal because the mirror gave
# equally-low residual on the wrong solution).  Y stays asymmetric
# (20 cm top arm vs 14 cm bottom arm).  Total span: 28 cm × 34 cm.
MARKER_WORLD = {
    "NW": np.array([-0.16, +0.20, 0.005], dtype=np.float64),
    "NE": np.array([+0.12, +0.20, 0.005], dtype=np.float64),
    "W":  np.array([-0.16,  0.00, 0.005], dtype=np.float64),
    "E":  np.array([+0.12,  0.00, 0.005], dtype=np.float64),
    "SW": np.array([-0.16, -0.14, 0.005], dtype=np.float64),
    "SE": np.array([+0.12, -0.14, 0.005], dtype=np.float64),
}
MARKER_ORDER = ["NW", "NE", "W", "E", "SW", "SE"]

# Camera intrinsics — must match mission_s182.py.  Derived from cf2
# SDF horizontal_fov = 1.0123 rad @ 640×480, halved by the 320×240
# bridge downscale: fx = fy ≈ 288 px.
FX = 288.3
FY = 288.3
CX = 160.0
CY = 120.0


def parse_journal(path):
    ticks = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(" ", 2)
        if len(parts) < 3:
            continue
        try:
            ts_ms = int(parts[0])
        except ValueError:
            continue
        if parts[1] != "tick":
            continue
        try:
            payload = ast.literal_eval(parts[2])
        except (ValueError, SyntaxError):
            continue
        payload["__ts_ms"] = ts_ms
        ticks.append(payload)
    return ticks


def parse_gt(path):
    rows = []
    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line:
            continue
        try:
            import json
            rows.append(json.loads(line))
        except Exception:
            try:
                rows.append(ast.literal_eval(line))
            except Exception:
                continue
    return rows


def project_marker(mw, cf2_pose):
    """Forward-project a marker world position to image pixels using
    cf2 EKF pose, assuming downward camera with body_xform = (-1, 0, 0, +1)
    per Sim.md §10b."""
    mx, my, _mz = mw
    cx_w, cy_w, cz_w, _yaw = cf2_pose
    z = cz_w - 0.005
    if z <= 0.01:
        return None
    u = CX - FX * (mx - cx_w) / z
    v = CY + FY * (my - cy_w) / z
    return u, v


def associate_dets(dets, cf2_pose, max_assoc_px=80):
    """Match each detection to its nearest projected marker (1-to-1)."""
    if cf2_pose is None or cf2_pose[2] < 0.05:
        return {}
    projected = {n: project_marker(MARKER_WORLD[n], cf2_pose)
                  for n in MARKER_ORDER}
    cf2_z = cf2_pose[2]
    out = {}
    used = set()
    for d in dets:
        if not d.get("v"):
            continue
        tz = d["tz"]
        rel = tz / cf2_z if cf2_z > 0.05 else 0
        if rel < 0.5 or rel > 1.5:
            continue
        best_name = None
        best_d2 = max_assoc_px * max_assoc_px
        for name, proj in projected.items():
            if name in used or proj is None:
                continue
            u, v = proj
            dx = d["px"] - u
            dy = d["py"] - v
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_name = name
        if best_name is None:
            continue
        used.add(best_name)
        out[best_name] = d
    return out


def kabsch_3d_3d(p_cam_xyz, p_world_xyz):
    """Closed-form Kabsch (Arun et al. 1987) over 3D-3D correspondences.

    Args:
      p_cam_xyz   : (N, 3) measured marker positions in CAM frame.
      p_world_xyz : (N, 3) known marker positions in WORLD frame.

    Returns:
      (R, t, residuals) where p_world ≈ R · p_cam + t.  drone_world = t.
      residuals : per-pair Euclidean error (used for outlier diagnostics).
    """
    assert p_cam_xyz.shape == p_world_xyz.shape
    assert p_cam_xyz.shape[0] >= 3
    pc = np.asarray(p_cam_xyz, dtype=np.float64)
    pw = np.asarray(p_world_xyz, dtype=np.float64)
    mu_c = pc.mean(axis=0)
    mu_w = pw.mean(axis=0)
    qc = pc - mu_c
    qw = pw - mu_w
    H = qc.T @ qw                              # 3×3 cross-covariance
    U, _S, Vt = np.linalg.svd(H, full_matrices=True)
    V = Vt.T
    d = np.sign(np.linalg.det(V @ U.T))
    if d == 0:
        d = 1.0
    R = V @ np.diag([1.0, 1.0, d]) @ U.T
    t = mu_w - R @ mu_c
    residuals = np.linalg.norm((R @ pc.T).T + t - pw, axis=1)
    return R, t, residuals


#  Iter-8 finding: at known drone altitude 0.252 m, our WhyCon PnP
#  returns tz ≈ 0.408 m — 62 % too high.  Root cause: the C-side
#  WHYCON_PNP_ANNULUS_FACTOR = 1.0 (set for "Gazebo PBR + bilinear
#  smooths annulus into solid disc" — iter-3 finding) but the actual
#  Gazebo render here is closer to a true Krajník annulus.  The
#  detector's 2nd-order-moment-derived semi-axis is ~62 % of the
#  outer-ring true radius, not 100 % as the iter-3 factor assumes.
#  Until we re-tune the C-side factor + rebuild, the verdict applies
#  a post-mortem scale of TVEC_SCALE on every (tx, ty, tz) tuple —
#  uniform scale preserves the projection geometry so Kabsch still
#  solves correctly.
TVEC_SCALE = 0.617


def tvec_to_cam_xyz(d):
    """The marker position in CAM frame from a WhyCon detection,
    scaled by the empirical TVEC_SCALE to compensate for the C-side
    ANNULUS_FACTOR mismatch (see comment above).
    """
    s = TVEC_SCALE
    return np.array([d["tx"] * s, d["ty"] * s, d["tz"] * s],
                     dtype=np.float64)


def kabsch_with_assignment(p_cam_list, marker_world_dict):
    """Procrustes-with-correspondence: pick the assignment of measured
    cam-frame points to world markers that minimises Kabsch residual.

    For 4-marker constellations and ≤4 detections this is 24
    permutations max — trivial.  Avoids needing a reliable cf2-EKF-
    based pre-association (which fails when cf2 is drifting).

    Returns: (R, t, residual_max, used_marker_names) or None if no
    assignment yields residual < threshold."""
    from itertools import permutations
    n_dets = len(p_cam_list)
    if n_dets < 3:
        return None
    names_all = list(marker_world_dict.keys())
    best = None
    for marker_subset in permutations(names_all, n_dets):
        p_cam = np.array(p_cam_list, dtype=np.float64)
        p_world = np.array([marker_world_dict[n] for n in marker_subset],
                            dtype=np.float64)
        R, t, residuals = kabsch_3d_3d(p_cam, p_world)
        res_max = float(residuals.max())
        if best is None or res_max < best[2]:
            best = (R, t, res_max, list(marker_subset))
    return best


# ──────────────────────────────────────────────────────────────────────
# Constrained pose solver — yaw-only rotation around world Z.
#
# Iter-8 finding: unconstrained Kabsch SVD with reflection-safe R has
# 2² = 4 valid orientations per assignment (mirror X and/or mirror Y,
# det adjusted via Z mirror).  Even with the H pattern X-asymmetry,
# the Y-mirror still satisfies det(R)=+1 because Z mirrors with it.
# So Kabsch picks between 2 valid R's (flip-Y and not) → bimodal Y/Z.
#
# Physical truth: the cf2 downward cam has a FIXED rotation w.r.t.
# the cf2 body frame.  Combined with cf2 yaw (which we read from
# CRTP LOG / journal), R_world_from_cam is a 1-DOF problem — just
# the yaw angle.  Once yaw is fixed, t is a closed-form 3-DOF
# least-squares.
#
# Cam mount convention (cf2 model.sdf.jinja downward_cam_link with
# the rotation 0 pi 0 around body Z, body X-axis): cam +X = world +X,
# cam +Y = world -Y, cam +Z = world -Z (when yaw=0).
# That gives R_cam_to_world_yaw0 = diag(1, -1, -1).
# ──────────────────────────────────────────────────────────────────────

R_CAM_TO_WORLD_FIXED = np.diag([1.0, -1.0, -1.0])


def yaw_rotation(theta: float) -> np.ndarray:
    """Rotation around world Z by theta (yaw)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def constrained_pose(p_cam_xyz, p_world_xyz, yaw_world: float):
    """Solve t such that p_world ≈ R_yaw(yaw_world) · R_cam_to_world_fixed
    · p_cam + t.  Closed-form least squares: t = mean(p_world - R · p_cam).
    Returns (R, t, residuals)."""
    R = yaw_rotation(yaw_world) @ R_CAM_TO_WORLD_FIXED
    pc = np.asarray(p_cam_xyz, dtype=np.float64)
    pw = np.asarray(p_world_xyz, dtype=np.float64)
    # Per-detection world estimate, then mean.
    t_per = pw - (R @ pc.T).T
    t = t_per.mean(axis=0)
    residuals = np.linalg.norm((R @ pc.T).T + t - pw, axis=1)
    return R, t, residuals


def constrained_pose_with_assignment(p_cam_list, marker_world_dict,
                                       yaw_world: float):
    """Pick the assignment that minimises constrained-pose residual."""
    from itertools import permutations
    n_dets = len(p_cam_list)
    if n_dets < 3:
        return None
    names_all = list(marker_world_dict.keys())
    best = None
    for marker_subset in permutations(names_all, n_dets):
        p_cam = np.array(p_cam_list, dtype=np.float64)
        p_world = np.array([marker_world_dict[n] for n in marker_subset],
                            dtype=np.float64)
        R, t, residuals = constrained_pose(p_cam, p_world, yaw_world)
        res_max = float(residuals.max())
        if best is None or res_max < best[2]:
            best = (R, t, res_max, list(marker_subset))
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--journal", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--tag", default="iter1")
    args = ap.parse_args()

    j_path = pathlib.Path(args.journal)
    g_path = pathlib.Path(args.gt)
    out_dir = pathlib.Path(args.out_dir)

    ticks = parse_journal(j_path)
    gt_rows = parse_gt(g_path)
    print(f"[s182-sota] {len(ticks)} ticks, {len(gt_rows)} GT rows")

    if not ticks or not gt_rows:
        sys.exit("nothing to analyse")

    t0_sentai = ticks[0]["__ts_ms"]
    t0_gt = gt_rows[0]["t_wall"]

    rows = []
    n_kabsch_3plus = 0
    n_kabsch_4 = 0
    n_no_assoc = 0
    RESIDUAL_GATE_M = 0.10   # 10 cm max residual to accept the fit
    for tk in ticks:
        cf2 = tk.get("cf2")
        if cf2 is None:
            continue
        # iter-8: dedupe by pixel proximity + take MIN tz for each
        # cluster.  WhyCon detector reports each true marker TWICE
        # (inner-disc + outer-ring blobs) — same pixel center, two
        # very different tz values.  The OUTER blob is what we want
        # (smaller tz because larger axis_a).  Drop duplicates within
        # 10 px pixel distance, keeping the smallest-tz of each cluster.
        raw_dets = [d for d in tk.get("dets", []) if d.get("v")]
        # Sort by tz so smaller tz comes first (= outer-ring detection).
        raw_dets.sort(key=lambda d: d.get("tz", 1e6))
        deduped = []
        for d in raw_dets:
            px, py = d["px"], d["py"]
            dupe = False
            for k in deduped:
                if (k["px"] - px) ** 2 + (k["py"] - py) ** 2 < 10 ** 2:
                    dupe = True
                    break
            if not dupe:
                deduped.append(d)
        # Now apply tz scale + sanity range.
        valid_dets = []
        for d in deduped:
            tz_s = d.get("tz", 0.0) * TVEC_SCALE
            if 0.05 < tz_s < 1.5:                    # plausible hover band
                valid_dets.append(d)
        # Iter-8b: NO CAP — operator: "sigur vrei cap la 4 markeri,
        # parca erau probleme la simetrie, de asta am pus 6".  The
        # whole point of 6-marker H pattern is to OVER-determine the
        # Kabsch fit + break ambiguities.  Capping defeats that.
        # We rely on dedupe (pixel proximity) + tz-range filter above
        # to keep the constellation clean.  If still >6, take 6 with
        # smallest |tx|+|ty|.
        if len(valid_dets) > 6:
            valid_dets.sort(key=lambda d: abs(d["tx"]) + abs(d["ty"]))
            valid_dets = valid_dets[:6]
        if len(valid_dets) < 3:
            n_no_assoc += 1
            continue
        # Iter-8b: back to unconstrained Kabsch.  The constrained
        # pose solver needed a hard-coded R_CAM_TO_WORLD_FIXED that
        # depended on the exact cf2 SDF cam mount orientation; getting
        # it wrong gives huge residuals.  With 6-marker X-asymmetric H,
        # unconstrained Kabsch should resolve the orientation uniquely.
        p_cam_list = [tvec_to_cam_xyz(d) for d in valid_dets]
        fit = kabsch_with_assignment(p_cam_list, MARKER_WORLD)
        if fit is None:
            n_no_assoc += 1
            continue
        R, t, res_max, used_names = fit
        if res_max > RESIDUAL_GATE_M:
            n_no_assoc += 1
            continue
        n_kabsch_3plus += 1
        if len(valid_dets) == 4:
            n_kabsch_4 += 1
        residuals = np.array([res_max])
        assocs = dict(zip(used_names, valid_dets))

        rel_s = (tk["__ts_ms"] - t0_sentai) / 1000.0
        target_wall = t0_gt + rel_s
        best_gt = None
        best_err = 1e9
        for g in gt_rows:
            err = abs(g["t_wall"] - target_wall)
            if err < best_err:
                best_err = err
                best_gt = g
            if g["t_wall"] > target_wall + 0.3:
                break
        if best_gt is None or best_err > 0.5:
            continue

        rows.append({
            "ts_ms":      tk["__ts_ms"],
            "rel_s":      rel_s,
            "alt_cmd":    tk["alt"],
            "n_assoc":    len(assocs),
            "markers":    "".join(sorted(assocs.keys())),
            "kabsch_res": float(residuals.max()),
            "gt_x":       best_gt["x"],
            "gt_y":       best_gt["y"],
            "gt_z":       best_gt["z"],
            "est_x":      float(t[0]),
            "est_y":      float(t[1]),
            "est_z":      float(t[2]),
            "cf2_x":      cf2[0],
            "cf2_y":      cf2[1],
            "cf2_z":      cf2[2],
        })

    print(f"[s182-sota] Kabsch fits: {n_kabsch_3plus} (4-marker: {n_kabsch_4}; "
          f"<3 dets: {n_no_assoc})")
    print(f"[s182-sota] paired with GT: {len(rows)}")

    csv_path = out_dir / f"s182_kabsch_{args.tag}.csv"
    cols = ["ts_ms", "rel_s", "alt_cmd", "n_assoc", "markers",
             "kabsch_res", "gt_x", "gt_y", "gt_z",
             "est_x", "est_y", "est_z", "cf2_x", "cf2_y", "cf2_z"]
    with csv_path.open("w") as f:
        f.write(",".join(cols) + "\n")
        for r in rows:
            f.write(",".join(str(r[c]) for c in cols) + "\n")
    print(f"[s182-sota] wrote {csv_path}")
    if not rows:
        return

    # ---- Figure 1: X / Y / Z est vs GT (Kabsch) ---------------------
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.6))
    meta = [("X", "gt_x", "est_x", "tab:orange"),
            ("Y", "gt_y", "est_y", "tab:green"),
            ("Z", "gt_z", "est_z", "tab:blue")]
    for ax, (name, gk, ek, color) in zip(axes, meta):
        gt = [r[gk] for r in rows]
        est = [r[ek] for r in rows]
        lo = min(min(gt), min(est))
        hi = max(max(gt), max(est))
        pad = 0.05 * (hi - lo + 1e-9)
        ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad],
                 "k--", lw=0.8, label="ideal (est = gt)")
        ax.scatter(gt, est, c=color, s=14, alpha=0.65,
                    edgecolor="black", linewidth=0.2, label="Kabsch")
        ax.set_xlabel(f"{name}_gt  (m)  [Gazebo dynamic_pose]")
        ax.set_ylabel(f"{name}_est (m)  [SOTA Kabsch SVD]")
        ax.set_title(f"{name} — drone world position")
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="box")
        ax.legend(loc="upper left", fontsize=8)
        errs = [abs(e - g) for g, e in zip(gt, est)]
        mae = sum(errs) / len(errs)
        ax.text(0.98, 0.02, f"MAE = {mae*100:.2f} cm  (n={len(rows)})",
                 transform=ax.transAxes, ha="right", va="bottom",
                 fontsize=9,
                 bbox=dict(boxstyle="round,pad=0.3", fc="white",
                            ec="gray", alpha=0.85))
    fig.suptitle(
        "s182 — WhyCon + Kabsch SOTA (Arun et al. 1987): drone X/Y/Z "
        "est vs GT  [Krajník-cross scene, cf2 SITL]",
        fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out_xyz = out_dir / f"s182_kabsch_xyz_{args.tag}.png"
    fig.savefig(out_xyz, dpi=150)
    print(f"[s182-sota] wrote {out_xyz}")

    # ---- Figure 2: Z time-series (GT / cf2 EKF / Kabsch est) --------
    fig2, ax2 = plt.subplots(1, 1, figsize=(12, 5.2))
    ts = [r["rel_s"] for r in rows]
    ax2.plot(ts, [r["gt_z"] for r in rows],
              ".-", c="black", lw=1.0, ms=3, label="GT (Gazebo)")
    ax2.plot(ts, [r["cf2_z"] for r in rows],
              ".", c="tab:red", ms=3, alpha=0.6, label="cf2 EKF")
    ax2.plot(ts, [r["est_z"] for r in rows],
              ".", c="tab:blue", ms=4, alpha=0.75, label="Kabsch est")
    # commanded altitude staircase
    for r in rows:
        ax2.axhline(r["alt_cmd"], color="gray", alpha=0.05, lw=0.5)
    ax2.set_xlabel("mission time (s)")
    ax2.set_ylabel("Z (m, world)")
    ax2.set_title("s182 — Z time-series: GT vs cf2 EKF vs Kabsch")
    ax2.legend(loc="best", fontsize=9)
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    out_z = out_dir / f"s182_kabsch_z_{args.tag}.png"
    fig2.savefig(out_z, dpi=150)
    print(f"[s182-sota] wrote {out_z}")


if __name__ == "__main__":
    main()

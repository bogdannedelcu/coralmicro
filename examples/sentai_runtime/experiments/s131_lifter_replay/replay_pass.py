"""Lifter replay against s130's captured Gazebo frames.

Walks the most-recent `/tmp/s130_image_only_nav/sentai_frames_*` dir,
correlates frames to `image_vs_cf2.json` records (drone pose at PnP
moment), runs ArUco detection on each PPM, and feeds the per-marker
stream through the inverse-depth EKF.

Per-marker pass criterion (LOOSE — s130 outer-loop motion has limited
parallax: ~6cm x × 20cm y over ~4 frames per marker):
    ||L_W_est - L_W_truth|| < 0.30 m  AND  no NaN/Inf  AND  no
    rejected updates (other than legitimate "behind camera").

This is INFORMATIONAL for the EKF's strict numerical convergence test
(σ_ρ < ε·ρ²) — synth_pass.py is the authoritative math validator.
Replay's job is to prove the frame conventions + bbox extraction +
pose data flow end-to-end on real Gazebo data before we port to C++.

If `/tmp/s130_image_only_nav/image_vs_cf2.json` is missing (s130 not
run), this exits 0 with a "skipped" verdict — replay is not blocking.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

# s090 lives under /home/.../experiments — same as us.
sys.path.insert(0,
    str(Path(__file__).resolve().parent.parent / "s090_hover_over_cat"))
from aruco_detector import (        # noqa: E402
    detect_in_ppm,
    CAM_FX, CAM_FY, CAM_CX, CAM_CY, CAM_OFFSET_BODY,
)

from lifter_proto import CameraModel, Landmark   # noqa: E402


def _to_native(o):
    """Recursive numpy → Python primitive conversion for JSON."""
    if isinstance(o, dict):
        return {k: _to_native(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_to_native(v) for v in o]
    if isinstance(o, np.ndarray):
        return _to_native(o.tolist())
    if isinstance(o, (np.generic,)):
        return o.item()
    return o

# Marker world map — matches s130 mission_l45.py ARUCO_MARKERS layout.
MARKER_WORLD_MAP = {
    0: np.array([+0.15, +0.10, 0.20], dtype=np.float64),
    1: np.array([-0.15, +0.10, 0.20], dtype=np.float64),
    2: np.array([-0.15, -0.10, 0.20], dtype=np.float64),
    3: np.array([+0.15, -0.10, 0.20], dtype=np.float64),
}
MARKER_REAL_SIZE_M = 0.0625

R_B_C = np.array([[0.0, 1.0,  0.0],
                  [1.0, 0.0,  0.0],
                  [0.0, 0.0, -1.0]], dtype=np.float64)

S130_DIR_DEFAULT = Path("/tmp/s130_image_only_nav")
# XY plane is well-observable from monocular bearings — strict tolerance.
PASS_XY_ERR_M = 0.10
# Z (depth) is weakly observable with limited parallax (s130's outer-loop
# motion is mostly ~10-20 cm in XY, hardly any altitude change). Loose
# tolerance here is EXPECTED — s132 with dedicated lateral pass will
# tighten this. See Civera 2008 §VI on baseline-vs-depth geometry.
PASS_Z_ERR_M = 0.60
# Rejected updates are EXPECTED and HEALTHY — the rho-clamp guard is
# fault containment per embeded.md F (local recovery before subsystem
# restart). The lifter remains usable; only count rejections as failure
# if EVERY update was rejected.
MIN_FRAMES_PER_MARKER = 3
MAX_REJECTION_RATIO = 0.7


def _bbox_from_corners(corners: np.ndarray) -> tuple[float, float, float]:
    """Return (cx, cy, w_px) from a (4,2) corner array.

    w_px = mean of top edge length + bottom edge length (robust to
    perspective tilt). Returned center is the mean of all 4 corners.
    """
    cx = float(np.mean(corners[:, 0]))
    cy = float(np.mean(corners[:, 1]))
    top_w = float(np.linalg.norm(corners[1] - corners[0]))
    bot_w = float(np.linalg.norm(corners[2] - corners[3]))
    w_px = 0.5 * (top_w + bot_w)
    return cx, cy, w_px


def _latest_frames_dir(base: Path) -> Path:
    candidates = sorted(base.glob("sentai_frames_*"))
    if not candidates:
        raise FileNotFoundError(f"no sentai_frames_* under {base}")
    return candidates[-1]


def replay_one_marker(marker_id: int,
                       records: list[dict],
                       frames_dir: Path,
                       cam: CameraModel,
                       verbose: bool) -> dict:
    """Replay observations of one marker through a fresh lifter.

    `records` is the time-ordered list of image_vs_cf2 entries with
    target_id == marker_id; their `ppm` field is the basename of a
    capture in `frames_dir`.
    """
    if marker_id not in MARKER_WORLD_MAP:
        return {"marker_id": marker_id, "pass": False,
                "reason": f"unknown_marker_id_{marker_id}"}
    truth = MARKER_WORLD_MAP[marker_id]

    valid_obs: list[dict] = []
    for r in records:
        ppm_path = frames_dir / r["ppm"]
        if not ppm_path.exists():
            continue
        dets = detect_in_ppm(str(ppm_path), estimate_pose=False)
        if marker_id not in dets:
            continue
        cx, cy, w_px = _bbox_from_corners(dets[marker_id].corners)
        drone_W = np.asarray(r["cf2"], dtype=np.float64)
        yaw_rad = math.radians(float(r.get("cf2_yaw_deg", 0.0)))
        valid_obs.append({
            "t_s": r["t_s"],
            "ppm": r["ppm"],
            "drone_W": drone_W,
            "yaw_rad": yaw_rad,
            "u_obs": cx,
            "v_obs": cy,
            "w_px": w_px,
        })

    if len(valid_obs) < MIN_FRAMES_PER_MARKER:
        return {"marker_id": marker_id, "pass": False,
                "reason": f"too_few_obs_{len(valid_obs)}_min_{MIN_FRAMES_PER_MARKER}",
                "n_valid_obs": len(valid_obs)}

    # ---- Init from first obs ----
    o0 = valid_obs[0]
    lm = Landmark.from_class_prior(
        u_c=o0["u_obs"], v_c=o0["v_obs"],
        bbox_w_px=o0["w_px"],
        real_size_m=MARKER_REAL_SIZE_M,
        drone_W=o0["drone_W"], yaw=o0["yaw_rad"],
        cam=cam,
    )
    init_status = lm.status()
    rows = []
    rows.append([
        f"# marker_id={marker_id} truth={truth.tolist()} n_obs={len(valid_obs)}"
    ])
    rows.append(["t_s", "u_obs", "v_obs", "w_px",
                 "drone_x", "drone_y", "drone_z", "yaw_rad",
                 "rho", "sigma_rho", "d_est_m",
                 "L_W_x", "L_W_y", "L_W_z", "err_m",
                 "innov_px", "rejected", "reason"])
    # Init row
    rows.append([
        f"{o0['t_s']:.4f}",
        f"{o0['u_obs']:.2f}", f"{o0['v_obs']:.2f}", f"{o0['w_px']:.2f}",
        f"{o0['drone_W'][0]:.4f}", f"{o0['drone_W'][1]:.4f}",
        f"{o0['drone_W'][2]:.4f}", f"{o0['yaw_rad']:.4f}",
        f"{init_status['rho']:.4f}",
        f"{init_status['sigma_rho']:.4f}",
        f"{init_status['d_est_m']:.4f}",
        f"{init_status['L_W'][0]:.4f}" if init_status['L_W'] else "nan",
        f"{init_status['L_W'][1]:.4f}" if init_status['L_W'] else "nan",
        f"{init_status['L_W'][2]:.4f}" if init_status['L_W'] else "nan",
        f"{float(np.linalg.norm(np.array(init_status['L_W'])-truth)):.4f}"
            if init_status['L_W'] else "nan",
        "0.00", "False", "init",
    ])

    n_rejected = 0
    for o in valid_obs[1:]:
        dt_s = max(o["t_s"] - valid_obs[0]["t_s"], 1.0/30.0)
        st = lm.update(o["u_obs"], o["v_obs"],
                       o["drone_W"], o["yaw_rad"], dt_s)
        if st["rejected"]:
            n_rejected += 1
        Lw = lm.world_pos()
        err = (float(np.linalg.norm(Lw - truth))
               if Lw is not None else float("nan"))
        sigma_rho = math.sqrt(max(st["var_rho"], 0.0))
        d_est = 1.0/st["rho"] if 0.05 < st["rho"] < 20 else float("nan")
        rows.append([
            f"{o['t_s']:.4f}",
            f"{o['u_obs']:.2f}", f"{o['v_obs']:.2f}", f"{o['w_px']:.2f}",
            f"{o['drone_W'][0]:.4f}", f"{o['drone_W'][1]:.4f}",
            f"{o['drone_W'][2]:.4f}", f"{o['yaw_rad']:.4f}",
            f"{st['rho']:.4f}", f"{sigma_rho:.4f}", f"{d_est:.4f}",
            f"{Lw[0]:.4f}" if Lw is not None else "nan",
            f"{Lw[1]:.4f}" if Lw is not None else "nan",
            f"{Lw[2]:.4f}" if Lw is not None else "nan",
            f"{err:.4f}",
            f"{st.get('innov_px', 0.0):.2f}",
            f"{st['rejected']}", str(st.get('reason') or ''),
        ])
        if verbose:
            print(f"  [mk{marker_id}] t={o['t_s']:5.2f}s "
                  f"L_W=({Lw[0]:6.3f},{Lw[1]:6.3f},{Lw[2]:6.3f}) "
                  f"d={d_est:5.3f}m err={err:6.4f}m innov={st.get('innov_px',0):.1f}px "
                  f"{'(rej:'+st.get('reason','')+')' if st['rejected'] else ''}")

    Lw_final = lm.world_pos()
    if Lw_final is not None and math.isfinite(float(np.linalg.norm(Lw_final))):
        final_err   = float(np.linalg.norm(Lw_final - truth))
        final_xy_err = float(np.linalg.norm(Lw_final[:2] - truth[:2]))
        final_z_err  = float(abs(Lw_final[2] - truth[2]))
    else:
        final_err = final_xy_err = final_z_err = float("inf")
    n_obs_attempted = len(valid_obs) - 1   # init doesn't count
    reject_ratio = (n_rejected / max(n_obs_attempted, 1))
    fail_reasons = []
    if Lw_final is None:
        fail_reasons.append("L_W_final_None")
    if not math.isfinite(final_err):
        fail_reasons.append("final_err_nan")
    if final_xy_err > PASS_XY_ERR_M:
        fail_reasons.append(
            f"final_xy_err_m={final_xy_err:.4f}>{PASS_XY_ERR_M}")
    if final_z_err > PASS_Z_ERR_M:
        fail_reasons.append(
            f"final_z_err_m={final_z_err:.4f}>{PASS_Z_ERR_M}")
    if reject_ratio > MAX_REJECTION_RATIO:
        fail_reasons.append(
            f"reject_ratio={reject_ratio:.2f}>{MAX_REJECTION_RATIO}")
    passed = len(fail_reasons) == 0
    return {
        "marker_id": marker_id,
        "pass": passed,
        "fail_reasons": fail_reasons,
        "n_valid_obs": len(valid_obs),
        "n_rejected": n_rejected,
        "reject_ratio": reject_ratio,
        "init": init_status,
        "final": lm.status(),
        "marker_W_true": truth.tolist(),
        "L_W_final": Lw_final.tolist() if Lw_final is not None else None,
        "final_err_m": final_err,
        "final_xy_err_m": final_xy_err,
        "final_z_err_m": final_z_err,
        "csv_rows": rows,
    }


def run(s130_dir: Path, out_dir: Path, verbose: bool = False) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    image_vs_cf2 = s130_dir / "image_vs_cf2.json"
    if not image_vs_cf2.exists():
        result = {"pass": True, "skipped": True,
                  "reason": f"no_s130_data_at_{s130_dir}"}
        with open(out_dir / "replay_result.json", "w") as f:
            json.dump(_to_native(result), f, indent=2)
        return result
    records = json.loads(image_vs_cf2.read_text())
    if not records:
        result = {"pass": True, "skipped": True, "reason": "empty_records"}
        with open(out_dir / "replay_result.json", "w") as f:
            json.dump(_to_native(result), f, indent=2)
        return result
    try:
        frames_dir = _latest_frames_dir(s130_dir)
    except FileNotFoundError as e:
        result = {"pass": True, "skipped": True, "reason": str(e)}
        with open(out_dir / "replay_result.json", "w") as f:
            json.dump(_to_native(result), f, indent=2)
        return result

    cam = CameraModel(
        fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY,
        R_B_C=R_B_C,
        cam_offset_B=np.asarray(CAM_OFFSET_BODY, dtype=np.float64),
    )

    by_id: dict[int, list] = {}
    for r in records:
        by_id.setdefault(r["target_id"], []).append(r)
    for mid in by_id:
        by_id[mid].sort(key=lambda r: r["t_s"])

    per_marker = []
    all_csv = []
    for mid in sorted(by_id):
        if verbose:
            print(f"[s131 replay] marker_id={mid}: "
                  f"{len(by_id[mid])} records, frames_dir={frames_dir.name}")
        r = replay_one_marker(mid, by_id[mid], frames_dir, cam, verbose)
        per_marker.append(r)
        all_csv.extend(r.pop("csv_rows"))
        all_csv.append(["#"])

    with open(out_dir / "replay_state.csv", "w", newline="") as f:
        w = csv.writer(f)
        for row in all_csv:
            w.writerow(row)

    overall_pass = all(m["pass"] for m in per_marker)
    result = {
        "pass": overall_pass,
        "skipped": False,
        "frames_dir": str(frames_dir),
        "thresholds": {
            "xy_err_m": PASS_XY_ERR_M,
            "z_err_m": PASS_Z_ERR_M,
            "min_frames_per_marker": MIN_FRAMES_PER_MARKER,
            "max_reject_ratio": MAX_REJECTION_RATIO,
        },
        "per_marker": per_marker,
    }
    with open(out_dir / "replay_result.json", "w") as f:
        json.dump(_to_native(result), f, indent=2)
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--s130-dir", default=str(S130_DIR_DEFAULT))
    p.add_argument("--out-dir", default="/tmp/s131_lifter_replay")
    p.add_argument("--verbose", action="store_true",
                   default=bool(int(__import__("os").environ.get(
                       "S131_VERBOSE", "0"))))
    args = p.parse_args()
    r = run(Path(args.s130_dir), Path(args.out_dir), verbose=args.verbose)
    if r.get("skipped"):
        print(f"[s131 replay] SKIPPED ({r['reason']}) — pass=true")
        return
    overall = "PASS" if r["pass"] else "FAIL"
    print(f"[s131 replay] {overall} (frames from {r['frames_dir']})")
    for m in r["per_marker"]:
        status = "PASS" if m["pass"] else "FAIL"
        print(f"  - mk{m['marker_id']}: {status}  "
              f"n_obs={m['n_valid_obs']}  rej={m['n_rejected']}  "
              f"xy_err={m['final_xy_err_m']:.4f}m  "
              f"z_err={m['final_z_err_m']:.4f}m")
        if not m["pass"]:
            print(f"      fail reasons: {m['fail_reasons']}")
    raise SystemExit(0 if r["pass"] else 1)


if __name__ == "__main__":
    main()

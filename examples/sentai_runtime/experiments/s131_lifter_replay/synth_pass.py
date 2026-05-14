"""Synthetic lateral-pass test of the inverse-depth EKF.

No Gazebo, no Python random-seed surprises (seed=42). Generates a
deterministic trajectory: drone holds z=1.0 m, yaw=0, executes lateral
motion x ∈ [-0.30, +0.30] over 6 s @ 30 fps. A single ArUco-sized
marker (real_size_m = 0.0625) sits at world (0.15, 0.10, 0.20).

For each frame: project marker world → camera pixel, add Gaussian
noise (σ = 1 px), feed to lifter. Records per-frame state. Pass:
- σ_ρ < 0.5 · ρ² before t = 5.0 s (linearization-valid landmark)
- final ||L_W_est - L_W_true|| < 0.10 m
- 0 rejected updates
- 0 NaN / Inf
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from lifter_proto import (
    CameraModel, Landmark,
    EPS_LIN_DEFAULT, OBS_PX_STD_DEFAULT,
)

# Camera intrinsics — match s130 sentai_crazysim 640×480 / 58° HFOV.
CAM_W, CAM_H = 640, 480
FOV_H_RAD = math.radians(58.0)
FOV_V_RAD = math.radians(45.0)
CAM_FX = CAM_W / (2.0 * math.tan(FOV_H_RAD / 2))
CAM_FY = CAM_H / (2.0 * math.tan(FOV_V_RAD / 2))
CAM_CX = CAM_W / 2.0
CAM_CY = CAM_H / 2.0

# Body-from-camera rotation — empirical s130 calibration (will be
# replaced by sentai.calib output on hardware; see objects_plan.md §21).
R_B_C = np.array([[0.0, 1.0,  0.0],
                  [1.0, 0.0,  0.0],
                  [0.0, 0.0, -1.0]], dtype=np.float64)

# Camera offset from drone CoM, body frame (from SDF model.sdf.jinja).
CAM_OFFSET_B = np.array([-0.04, 0.0, -0.02], dtype=np.float64)

# Test marker — NEAR scenario (d ≈ 0.8 m at hover height 1 m).
MARKER_W = np.array([0.15, 0.10, 0.20], dtype=np.float64)
REAL_SIZE_M = 0.0625

# Test marker — FAR scenario (d ≈ 6 m, ρ ≈ 0.16 → ρ² ≈ 0.027;
# σ_ρ₀ = 0.5·ρ₀ ≈ 0.08 > ε·ρ² = 0.013 → NOT-ready at init, must
# converge via parallax. Validates that the linearization gate
# actually fires the right way around.)
MARKER_W_FAR = np.array([0.0, 0.0, -5.0], dtype=np.float64)   # below drone
REAL_SIZE_M_FAR = 0.30        # red cube (per class table in objects_plan.md)

# Trajectory params.
TRAJ_DURATION_S = 6.0
TRAJ_FPS = 30.0
TRAJ_X_AMP_M = 0.30   # ±0.30 m lateral

# Pass criteria.
PASS_FINAL_ERR_M = 0.10
PASS_READY_BY_S = 5.0

# Sim params.
OBS_NOISE_PX_STD = 1.0          # Gaussian per-axis
INIT_BBOX_W_PX = None           # computed at runtime from projection size


def drone_pose_at(t_s: float) -> tuple[np.ndarray, float]:
    """Smooth lateral pass: x(t) = AMP · sin(2π · t / period).

    period = TRAJ_DURATION_S → exactly one full cycle.
    z hold 1.0 m, yaw = 0.0 throughout.
    """
    x = TRAJ_X_AMP_M * math.sin(2.0 * math.pi * t_s / TRAJ_DURATION_S)
    return np.array([x, 0.0, 1.0], dtype=np.float64), 0.0


def synth_bbox_w_px(L_W: np.ndarray, drone_W: np.ndarray, yaw: float,
                    cam: CameraModel, real_size_m: float) -> float:
    """Approximate apparent bbox width: w_px ≈ fx · real_size / d.

    Realistic for fronto-parallel markers; for tilted ones, would
    need projection of marker corners. Sufficient for class-prior init.
    """
    R_W_B = np.eye(3, dtype=np.float64)   # yaw=0 here
    c_W = drone_W + R_W_B @ cam.cam_offset_B
    d = float(np.linalg.norm(L_W - c_W))
    if d < 0.05:
        return 1.0
    return cam.fx * real_size_m / d


def run_scenario(name: str,
                 marker_W: np.ndarray,
                 real_size_m: float,
                 final_err_tol_m: float,
                 require_ready: bool,
                 verbose: bool, rng) -> tuple[dict, list]:
    """Run one synthetic scenario; returns (result, csv_rows)."""
    cam = CameraModel(
        fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY,
        R_B_C=R_B_C, cam_offset_B=CAM_OFFSET_B,
    )
    drone_W0, yaw0 = drone_pose_at(0.0)
    proj0 = cam.project(marker_W, drone_W0, yaw0)
    if proj0 is None:
        return ({"name": name, "pass": False,
                 "reason": "init_marker_behind_camera"}, [])
    u0_clean, v0_clean, _ = proj0
    u0 = u0_clean + rng.normal(0.0, OBS_NOISE_PX_STD)
    v0 = v0_clean + rng.normal(0.0, OBS_NOISE_PX_STD)
    bbox_w_px = synth_bbox_w_px(marker_W, drone_W0, yaw0, cam, real_size_m)
    lm = Landmark.from_class_prior(
        u_c=u0, v_c=v0,
        bbox_w_px=bbox_w_px, real_size_m=real_size_m,
        drone_W=drone_W0, yaw=yaw0, cam=cam,
    )
    init_status = lm.status()
    ready_at_init = init_status["ready"]

    n_frames = int(round(TRAJ_DURATION_S * TRAJ_FPS))
    dt = 1.0 / TRAJ_FPS
    n_rejected = 0
    n_nan = 0
    ready_at_s = 0.0 if ready_at_init else None
    csv_rows = []
    csv_rows.append([f"# scenario={name}"])
    csv_rows.append([
        "t_s", "drone_x", "drone_y", "drone_z",
        "u_obs", "v_obs", "u_pred", "v_pred",
        "rho", "sigma_rho", "d_est_m",
        "L_W_x", "L_W_y", "L_W_z", "err_m",
        "n_obs", "ready", "rejected", "reason",
    ])

    for f in range(1, n_frames):
        t_s = f * dt
        drone_W, yaw = drone_pose_at(t_s)
        proj = cam.project(marker_W, drone_W, yaw)
        if proj is None:
            continue
        u_clean, v_clean, _ = proj
        u = u_clean + rng.normal(0.0, OBS_NOISE_PX_STD)
        v = v_clean + rng.normal(0.0, OBS_NOISE_PX_STD)
        st = lm.update(u, v, drone_W, yaw, dt)
        if st["rejected"]:
            n_rejected += 1
        if not all(math.isfinite(x) for x in (st["rho"], st["var_rho"])):
            n_nan += 1
        sigma_rho = math.sqrt(max(st["var_rho"], 0.0))
        Lw = lm.world_pos()
        err = (float(np.linalg.norm(Lw - marker_W))
               if Lw is not None else float("nan"))
        d_est = (1.0 / st["rho"] if 0.01 < st["rho"] < 100 else float("nan"))
        if ready_at_s is None and lm.ready():
            ready_at_s = t_s
        csv_rows.append([
            f"{t_s:.4f}",
            f"{drone_W[0]:.4f}", f"{drone_W[1]:.4f}", f"{drone_W[2]:.4f}",
            f"{u:.2f}", f"{v:.2f}",
            f"{st.get('u_pred', float('nan')):.2f}",
            f"{st.get('v_pred', float('nan')):.2f}",
            f"{st['rho']:.4f}", f"{sigma_rho:.4f}", f"{d_est:.4f}",
            f"{Lw[0]:.4f}" if Lw is not None else "nan",
            f"{Lw[1]:.4f}" if Lw is not None else "nan",
            f"{Lw[2]:.4f}" if Lw is not None else "nan",
            f"{err:.4f}",
            f"{st['n_obs']}", f"{st.get('ready', False)}",
            f"{st['rejected']}", str(st.get('reason') or ''),
        ])
        if verbose and f % 30 == 0:
            print(f"[{name}] t={t_s:5.2f}s rho={st['rho']:6.3f} "
                  f"σ_ρ={sigma_rho:6.4f} d={d_est:5.3f}m err={err:6.4f}m "
                  f"ready={st.get('ready', False)}")

    final = lm.status()
    Lw_final = lm.world_pos()
    final_err = (float(np.linalg.norm(Lw_final - marker_W))
                 if Lw_final is not None else float("inf"))
    fail_reasons = []
    if n_rejected != 0:
        fail_reasons.append(f"n_rejected={n_rejected}")
    if n_nan != 0:
        fail_reasons.append(f"n_nan={n_nan}")
    if require_ready and (ready_at_s is None or ready_at_s > PASS_READY_BY_S):
        fail_reasons.append(f"not_ready_by_{PASS_READY_BY_S}s "
                            f"(ready_at_s={ready_at_s})")
    if final_err > final_err_tol_m:
        fail_reasons.append(
            f"final_err_m={final_err:.4f} > {final_err_tol_m}")
    passed = len(fail_reasons) == 0
    result = {
        "name": name,
        "pass": passed,
        "fail_reasons": fail_reasons,
        "n_frames": n_frames,
        "n_obs_total": final["n_obs"],
        "n_rejected": n_rejected,
        "n_nan": n_nan,
        "ready_at_s": ready_at_s,
        "ready_at_init": ready_at_init,
        "init": init_status,
        "final": final,
        "marker_W_true": marker_W.tolist(),
        "L_W_final": Lw_final.tolist() if Lw_final is not None else None,
        "final_err_m": final_err,
        "thresholds": {
            "final_err_m": final_err_tol_m,
            "ready_by_s": PASS_READY_BY_S,
            "require_ready": require_ready,
        },
    }
    return result, csv_rows


def run(out_dir: Path, verbose: bool = False, seed: int = 42) -> dict:
    rng = np.random.default_rng(seed)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "synth_state.csv"
    json_path = out_dir / "synth_result.json"

    near_r, near_csv = run_scenario(
        "near_aruco_d0p8m", MARKER_W, REAL_SIZE_M,
        final_err_tol_m=PASS_FINAL_ERR_M, require_ready=True,
        verbose=verbose, rng=rng,
    )
    far_r, far_csv = run_scenario(
        "far_cube_d6m", MARKER_W_FAR, REAL_SIZE_M_FAR,
        final_err_tol_m=0.50,             # 50 cm at d=6 m is reasonable
        require_ready=True, verbose=verbose, rng=rng,
    )

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        for row in near_csv + [["#"]] + far_csv:
            w.writerow(row)

    overall_pass = near_r["pass"] and far_r["pass"]
    result = {
        "pass": overall_pass,
        "scenarios": [near_r, far_r],
        "seed": seed,
    }
    with open(json_path, "w") as f:
        json.dump(result, f, indent=2)
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out-dir", default="/tmp/s131_lifter_replay")
    p.add_argument("--verbose", action="store_true",
                   default=bool(int(__import__("os").environ.get(
                       "S131_VERBOSE", "0"))))
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()
    out_dir = Path(args.out_dir)
    r = run(out_dir, verbose=args.verbose, seed=args.seed)
    overall = "PASS" if r["pass"] else "FAIL"
    print(f"[s131 synth] {overall}")
    for s in r["scenarios"]:
        status = "PASS" if s["pass"] else "FAIL"
        print(f"  - {s['name']}: {status}  "
              f"final_err={s['final_err_m']:.4f}m  "
              f"ready_at_s={s['ready_at_s']}  "
              f"ready_at_init={s['ready_at_init']}  "
              f"n_obs={s['n_obs_total']}  rejected={s['n_rejected']}")
        if not s["pass"]:
            print(f"      fail reasons: {s['fail_reasons']}")
    raise SystemExit(0 if r["pass"] else 1)


if __name__ == "__main__":
    main()

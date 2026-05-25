#!/usr/bin/env python3
"""s195 post-mortem verdict.

Reads mission artifacts plus host-side Gazebo GT.  GT is forensic only and is
never fed back to sentai_sim or the mission control loop.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path


def _load_json(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as fp:
        return json.load(fp)


def _load_gt(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def _span(vals: list[float]) -> tuple[float, float, float]:
    if not vals:
        return (math.nan, math.nan, math.nan)
    return (min(vals), max(vals), max(vals) - min(vals))


def _angle_delta_deg(a: float, b: float) -> float:
    d = b - a
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: verdict_s195.py ITER_DIR", file=sys.stderr)
        return 2

    workdir = Path(argv[1])
    summary = _load_json(workdir / "mission_s194val_summary.json")
    gt = _load_gt(workdir / "fr_current" / "gt.jsonl")

    print("s195 verdict: post-mortem only; GT is not used by sentai_sim")
    print(f"mission_status={summary.get('status', '?')}")
    print(f"mission_phase={summary.get('phase', '?')}")

    acq = summary.get("marker_acquisition") or {}
    brake = summary.get("post_lock_brake") or {}
    zhold = summary.get("visual_z_hold") or {}
    axis = summary.get("axis_response_smoke") or {}
    centroid_val = summary.get("centroid_pd_validation") or {}
    candidate = summary.get("candidate_scoring") or {}
    final_val = summary.get("final_candidate_validation") or {}
    final_recenter = summary.get("final_centroid_recenter") or {}
    print(f"marker_locked={acq.get('locked', False)}")
    print(f"lock_tick={acq.get('lock_tick')}")
    print(f"post_lock_brake_ok={brake.get('ok', False)}")
    print(f"post_lock_brake_vz_last_m_s={float(brake.get('vz_filt_last_m_s') or 0.0):.4f}")
    print(f"post_lock_brake_z_last_m={float(brake.get('z_cam_last_m') or 0.0):.4f}")
    print(f"visual_z_hold_ok={zhold.get('ok', False)}")
    print(f"visual_z_hold_abort={zhold.get('abort_reason', '')}")
    print(f"axis_response_ok={axis.get('ok', False)}")
    orth = axis.get("orthogonality") or {}
    if orth:
        print(
            "axis_orthogonality "
            f"dot_norm={float(orth.get('dot_norm') or 0.0):.3f} "
            f"ok={orth.get('ok')}"
        )
    for r in axis.get("results") or []:
        print(
            "axis_response "
            f"axis={r.get('axis')} "
            f"dominant={r.get('dominant_image_axis')} "
            f"sign={r.get('dominant_sign')} "
            f"dominance={float(r.get('dominance_ratio') or 0.0):.2f} "
            f"sign_ok={r.get('axis_sign_ok')} "
            f"pos_seg_px={r.get('pos_segment_delta_px')} "
            f"neg_seg_px={r.get('neg_segment_delta_px')} "
            f"comp_px={r.get('complementary_delta_px')} "
            f"pre_return_err_px={r.get('pre_recenter_return_err_px')} "
            f"settle_delta_px={r.get('settle_delta_from_axis_baseline_px')} "
            f"return_err_px={r.get('return_err_px')} "
            f"return_ok={r.get('return_ok')} "
            f"recenter_passes={(r.get('recenter') or {}).get('passes')} "
            f"min_full={r.get('min_full_markers')}"
        )
    print(f"centroid_validation_ok={centroid_val.get('ok', False)}")
    print(f"centroid_validation_reason={centroid_val.get('reason', '')}")
    print(f"centroid_validation_initial_err_px={float(centroid_val.get('initial_err_px') or 0.0):.2f}")
    print(f"centroid_validation_final_err_px={float(centroid_val.get('final_err_px') or 0.0):.2f}")
    print(f"centroid_validation_min_err_px={float(centroid_val.get('min_err_px') or 0.0):.2f}")
    print(f"centroid_validation_improvement_px={float(centroid_val.get('improvement_px') or 0.0):.2f}")
    print(f"centroid_validation_n_full_min={int(centroid_val.get('n_full_min') or 0)}")
    print(f"candidate_scoring_ok={candidate.get('ok', False)}")
    print(f"candidate_scoring_reason={candidate.get('reason', '')}")
    print(f"candidate_margin={float(candidate.get('margin') or 0.0):.2f}")
    best = candidate.get("best") or {}
    second = candidate.get("second") or {}
    if best:
        print(
            "candidate_best "
            f"idx={best.get('idx')} "
            f"score={float(best.get('score') or 0.0):.2f} "
            f"R={best.get('R_cam_to_body')}"
        )
    if second:
        print(
            "candidate_second "
            f"idx={second.get('idx')} "
            f"score={float(second.get('score') or 0.0):.2f} "
            f"R={second.get('R_cam_to_body')}"
        )
    print(f"final_validation_ok={final_val.get('ok', False)}")
    print(f"final_validation_reason={final_val.get('reason', '')}")
    for r in final_val.get("results") or []:
        print(
            "final_validation_axis "
            f"axis={r.get('axis')} "
            f"expected={r.get('expected_image_axis')}:{r.get('expected_sign')} "
            f"observed={r.get('observed_image_axis')}:{r.get('observed_sign')} "
            f"dominance={float(r.get('dominance_ratio') or 0.0):.2f} "
            f"comp_px={r.get('complementary_delta_px')} "
            f"return_err_px={float(r.get('return_err_px') or 0.0):.2f} "
            f"ok={r.get('ok')}"
        )
    print(f"final_recenter_ok={final_recenter.get('ok', False)}")
    print(f"final_recenter_reason={final_recenter.get('reason', '')}")
    print(f"final_recenter_passes={int(final_recenter.get('passes') or 0)}")
    print(f"final_recenter_initial_err_px={float(final_recenter.get('initial_err_px') or 0.0):.2f}")
    print(f"final_recenter_err_px={float(final_recenter.get('err_px') or 0.0):.2f}")
    z_src = zhold if zhold else acq
    z_alg_max = float(z_src.get("z_cam_max_m") or 0.0)
    z_alg_last = float(z_src.get("z_cam_last_m") or 0.0)
    print(f"alg_z_cam_max_m={z_alg_max:.4f}")
    print(f"alg_z_cam_last_m={z_alg_last:.4f}")
    if zhold:
        print(f"alg_z_hold_target_m={float(zhold.get('target_z_m') or 0.0):.4f}")
        print(f"alg_z_hold_valid_ticks={int(zhold.get('valid_ticks') or 0)}")
        print(f"alg_z_hold_thrust_min={int(zhold.get('thrust_min') or 0)}")
        print(f"alg_z_hold_thrust_max={int(zhold.get('thrust_max') or 0)}")

    if not gt:
        print("gt_records=0")
        print("gt_note=missing fr_current/gt.jsonl; run launch_sim with GT recorder")
        return 0

    xs = [float(r["x"]) for r in gt if "x" in r]
    ys = [float(r["y"]) for r in gt if "y" in r]
    zs = [float(r["z"]) for r in gt if "z" in r]
    yaws = [float(r["yaw_deg"]) for r in gt if "yaw_deg" in r]
    x0 = xs[0] if xs else 0.0
    y0 = ys[0] if ys else 0.0
    z0 = zs[0] if zs else 0.0
    x_last = xs[-1] if xs else 0.0
    y_last = ys[-1] if ys else 0.0
    z_last = zs[-1] if zs else 0.0
    xy_drift_last = math.sqrt((x_last - x0) ** 2 + (y_last - y0) ** 2)
    xy_radius_max = 0.0
    for x, y in zip(xs, ys):
        r = math.sqrt((x - x0) ** 2 + (y - y0) ** 2)
        if r > xy_radius_max:
            xy_radius_max = r

    z_min, z_max, z_span = _span(zs)
    x_min, x_max, x_span = _span(xs)
    y_min, y_max, y_span = _span(ys)
    yaw_delta_last = (
        _angle_delta_deg(yaws[0], yaws[-1]) if len(yaws) >= 2 else math.nan)
    yaw_abs_span = 0.0
    if yaws:
        yaw0 = yaws[0]
        for yaw in yaws:
            d = abs(_angle_delta_deg(yaw0, yaw))
            if d > yaw_abs_span:
                yaw_abs_span = d
    print(f"gt_records={len(gt)}")
    print(f"gt_x_min_m={x_min:.4f}")
    print(f"gt_x_max_m={x_max:.4f}")
    print(f"gt_x_span_m={x_span:.4f}")
    print(f"gt_y_min_m={y_min:.4f}")
    print(f"gt_y_max_m={y_max:.4f}")
    print(f"gt_y_span_m={y_span:.4f}")
    print(f"gt_z_min_m={z_min:.4f}")
    print(f"gt_z_max_m={z_max:.4f}")
    print(f"gt_z_span_m={z_span:.4f}")
    print(f"gt_z_last_m={z_last:.4f}")
    print(f"gt_xy_drift_last_m={xy_drift_last:.4f}")
    print(f"gt_xy_radius_max_m={xy_radius_max:.4f}")
    print(f"gt_yaw_delta_last_deg={yaw_delta_last:.2f}")
    print(f"gt_yaw_abs_span_deg={yaw_abs_span:.2f}")
    if z_alg_max > 0.0:
        print(f"err_z_max_alg_minus_gt_m={z_alg_max - z_max:.4f}")
    else:
        print("err_z_max_alg_minus_gt_m=nan")
    if z_alg_last > 0.0:
        print(f"err_z_last_alg_minus_gt_m={z_alg_last - z_last:.4f}")
    else:
        print("err_z_last_alg_minus_gt_m=nan")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

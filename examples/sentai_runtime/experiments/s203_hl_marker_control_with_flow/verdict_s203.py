#!/usr/bin/env python3
"""s203 post-mortem verdict.

GT is forensic only.  The mission success gate is based on the on-drone
summary: calibration load, marker lock, ExtPos warmup, handoff hover, landing.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path


def load_json(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as fp:
        return json.load(fp)


def load_gt(path: Path) -> list[dict]:
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
                pass
    return rows


def span(vals: list[float]) -> tuple[float, float, float]:
    if not vals:
        return (math.nan, math.nan, math.nan)
    return min(vals), max(vals), max(vals) - min(vals)


def quat_to_rpy_deg(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def load_phase_intervals(journal_path: Path) -> dict[str, tuple[float, float | None]]:
    intervals: dict[str, tuple[float, float | None]] = {}
    current = None
    current_t = None
    if not journal_path.exists():
        return intervals
    with journal_path.open("r", encoding="utf-8", errors="replace") as fp:
        for line in fp:
            parts = line.split(" ", 2)
            if len(parts) < 3 or parts[1] != "phase_transition":
                continue
            try:
                t_s = float(parts[0]) / 1000.0
            except ValueError:
                continue
            marker = "'to': '"
            idx = parts[2].find(marker)
            if idx < 0:
                continue
            rest = parts[2][idx + len(marker):]
            end = rest.find("'")
            if end < 0:
                continue
            phase = rest[:end]
            if current is not None and current_t is not None:
                intervals[current] = (current_t, t_s)
            current = phase
            current_t = t_s
    if current is not None and current_t is not None:
        intervals[current] = (current_t, None)
    return intervals


def attitude_stats_from_gt(gt: list[dict], t0: float, t1: float | None) -> dict:
    vals = []
    for r in gt:
        t = r.get("t_wall")
        if t is None:
            continue
        t = float(t)
        if t < t0 or (t1 is not None and t >= t1):
            continue
        if not all(k in r for k in ("qx", "qy", "qz", "qw")):
            continue
        roll, pitch, _ = quat_to_rpy_deg(
            float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"]))
        vals.append((roll, pitch))
    if not vals:
        return {"samples": 0}
    roll_abs = [abs(v[0]) for v in vals]
    pitch_abs = [abs(v[1]) for v in vals]
    tilt = [math.hypot(v[0], v[1]) for v in vals]
    return {
        "samples": len(vals),
        "roll_abs_max_deg": max(roll_abs),
        "pitch_abs_max_deg": max(pitch_abs),
        "tilt_abs_max_deg": max(tilt),
        "roll_rms_deg": math.sqrt(sum(v[0] * v[0] for v in vals) / len(vals)),
        "pitch_rms_deg": math.sqrt(sum(v[1] * v[1] for v in vals) / len(vals)),
    }


def load_generic_segment_intervals(journal_path: Path) -> list[dict]:
    segs: list[dict] = []
    current = None
    start_t = None
    target = None
    from_target = None
    if not journal_path.exists():
        return segs
    with journal_path.open("r", encoding="utf-8", errors="replace") as fp:
        lines = fp.readlines()
    for line in lines:
        parts = line.split(" ", 2)
        if len(parts) < 3:
            continue
        try:
            t_s = float(parts[0]) / 1000.0
        except ValueError:
            continue
        if parts[1] == "generic_axis_motion_tick":
            try:
                data = __import__("ast").literal_eval(parts[2])
            except Exception:
                continue
            label = data.get("segment")
            if label != current:
                if current is not None and start_t is not None:
                    segs.append({
                        "label": current,
                        "t0": start_t,
                        "t1": t_s,
                        "target": target,
                        "from_target": from_target,
                    })
                current = label
                start_t = t_s
                target = data.get("target")
                from_target = None
        elif parts[1] == "generic_axis_motion_segment":
            try:
                data = __import__("ast").literal_eval(parts[2])
            except Exception:
                continue
            if data.get("label") == current:
                from_target = data.get("from_target", from_target)
                target = data.get("target", target)
    end_t = None
    for line in lines:
        parts = line.split(" ", 2)
        if len(parts) >= 3 and parts[1] == "phase_transition":
            try:
                t_s = float(parts[0]) / 1000.0
                data = __import__("ast").literal_eval(parts[2])
            except Exception:
                continue
            if data.get("to") == "land":
                end_t = t_s
                break
    if current is not None and start_t is not None:
        segs.append({
            "label": current,
            "t0": start_t,
            "t1": end_t,
            "target": target,
            "from_target": from_target,
        })
    return segs


def gt_axis_motion_stats(gt: list[dict], segs: list[dict]) -> tuple[list[dict], bool]:
    rows = []
    ok_all = bool(segs)
    for seg in segs:
        t0 = seg.get("t0")
        t1 = seg.get("t1")
        ss = [
            r for r in gt
            if t0 is not None and t1 is not None and t0 <= float(r.get("t_wall", -1)) < t1
        ]
        if len(ss) < 3:
            out = {"label": seg.get("label"), "ok": False, "reason": "too_few_gt_samples"}
            rows.append(out)
            ok_all = False
            continue
        target = seg.get("target") or (0.0, 0.0, 0.0)
        if target and isinstance(target[0], str):
            rows.append({
                "label": seg.get("label"),
                "ok": True,
                "target_frame": "image",
                "note": "world-frame GT axis gate skipped for image-frame target",
            })
            continue
        from_target = seg.get("from_target") or (0.0, 0.0, target[2] if len(target) > 2 else 0.0)
        cmd_dx = float(target[0]) - float(from_target[0])
        cmd_dy = float(target[1]) - float(from_target[1])
        axis = "x" if abs(cmd_dx) >= abs(cmd_dy) else "y"
        cmd_delta = cmd_dx if axis == "x" else cmd_dy
        vals_axis = [float(r[axis]) for r in ss]
        other = "y" if axis == "x" else "x"
        vals_other = [float(r[other]) for r in ss]
        actual_delta = vals_axis[-1] - vals_axis[0]
        required = 0.40 * abs(cmd_delta)
        same_sign = (cmd_delta == 0.0) or (actual_delta * cmd_delta > 0.0)
        enough_motion = abs(actual_delta) >= required
        increments = [b - a for a, b in zip(vals_axis, vals_axis[1:])]
        useful = [d for d in increments if abs(d) > 0.0005]
        if useful and cmd_delta != 0.0:
            monotonic_ratio = sum(1 for d in useful if d * cmd_delta > 0.0) / len(useful)
        else:
            monotonic_ratio = 0.0
        other_span = max(vals_other) - min(vals_other)
        axis_span = max(vals_axis) - min(vals_axis)
        smooth_enough = monotonic_ratio >= 0.60
        other_bounded = other_span <= max(0.04, abs(cmd_delta) * 0.75)
        ok = same_sign and enough_motion and smooth_enough and other_bounded
        if not ok:
            ok_all = False
        rows.append({
            "label": seg.get("label"),
            "ok": ok,
            "axis": axis,
            "cmd_delta_m": cmd_delta,
            "actual_delta_m": actual_delta,
            "required_delta_m": required,
            "axis_span_m": axis_span,
            "other_axis": other,
            "other_span_m": other_span,
            "same_sign": same_sign,
            "enough_motion": enough_motion,
            "monotonic_ratio": monotonic_ratio,
            "smooth_enough": smooth_enough,
            "other_bounded": other_bounded,
            "samples": len(ss),
        })
    return rows, ok_all


def fmt_float(v, fallback: float = 0.0) -> str:
    if v is None:
        return f"{fallback:.6f}"
    try:
        return f"{float(v):.6f}"
    except (TypeError, ValueError):
        return f"{fallback:.6f}"


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: verdict_s203.py ITER_DIR", file=sys.stderr)
        return 2
    workdir = Path(argv[1])
    summary = load_json(workdir / "mission_s203_summary.json")
    gt = load_gt(workdir / "fr_current" / "gt.jsonl")
    phase_intervals = load_phase_intervals(workdir / "mission_s203_journal.txt")

    print("s203 verdict: GT is post-mortem only")
    print(f"mission_status={summary.get('status', '?')}")
    print(f"mission_phase={summary.get('phase', '?')}")
    print(f"calib_load_ok={summary.get('calib_load_ok', False)}")
    print(f"calib_layout_ok={summary.get('calib_layout_ok', False)}")
    print(f"R_loaded_matches_seed_diagnostic={summary.get('R_loaded_matches_seed', False)}")
    print(f"R_expected_max_abs_diff={fmt_float(summary.get('R_expected_max_abs_diff'), 999.0)}")
    print(f"pose_subscribe_rc={summary.get('pose_subscribe_rc')}")
    print(f"pose_subscribe_attempts={summary.get('pose_subscribe_attempts')}")

    acq = summary.get("acquire") or {}
    center = summary.get("center_hold") or {}
    extpos = summary.get("extpos_warmup") or {}
    hover = summary.get("handoff_hover") or {}
    generic_motion = summary.get("generic_axis_motion") or {}
    land = summary.get("land") or {}
    kreset = summary.get("kalman_reset_before_extpos") or {}

    print(f"acquire_ok={acq.get('ok', False)}")
    print(f"acquire_z_lock_m={float(acq.get('z_lock_m') or 0):.3f}")
    print(f"acquire_z_target_scale={float(acq.get('z_target_scale') or 0):.3f}")
    print(f"acquire_z_target_m={float(acq.get('z_target_m') or 0):.3f}")
    print(f"center_hold_ok={center.get('ok', False)}")
    print(f"center_hold_err_last_px={float(center.get('err_last_px') or 0):.2f}")
    print(f"center_hold_n_full_min={center.get('n_full_min', '?')}")
    print(f"center_hold_attitude_stats={center.get('attitude_stats')}")
    print(f"kalman_reset_before_extpos_ok={kreset.get('ok', False)}")
    print(f"kalman_reset_before_extpos={kreset}")
    print(f"extpos_warmup_ok={extpos.get('ok', False)}")
    print(f"extpos_converged={extpos.get('converged', False)}")
    print(f"extpos_diverged={extpos.get('diverged', False)}")
    print(f"extpos_convergence_reason={extpos.get('convergence_reason', '')}")
    print(f"extpos_ticks={extpos.get('ticks', '?')}")
    print(f"extpos_n_full_recent_avg={extpos.get('n_full_recent_avg', '?')}")
    print(f"extpos_send_ok={extpos.get('send_ok', 0)}")
    print(f"extpos_pose_rejects={extpos.get('pose_rejects', 0)}")
    print(f"extpos_pose_reject_reasons={extpos.get('pose_reject_reasons', {})}")
    print(f"extpos_last_pose={extpos.get('last_pose')}")
    print(f"extpos_last_sent={extpos.get('last_extpos')}")
    print(f"extpos_signs={extpos.get('extpos_signs')}")
    print(f"extpos_last_est_pose={extpos.get('last_est_pose')}")
    kx = extpos.get("kalman_crosscheck") or {}
    print(f"kalman_samples={kx.get('samples', 0)}")
    print(f"kalman_err_first_m={float(kx.get('err_first_m') or 0):.4f}")
    print(f"kalman_err_last_m={float(kx.get('err_last_m') or 0):.4f}")
    print(f"kalman_err_mean_m={float(kx.get('err_mean_m') or 0):.4f}")
    print(f"kalman_err_recent10_mean_m={float(kx.get('err_recent10_mean_m') or 0):.4f}")
    print(f"kalman_err_min_m={float(kx.get('err_min_m') or 0):.4f}")
    print(f"kalman_err_max_m={float(kx.get('err_max_m') or 0):.4f}")
    print(f"kalman_trend_improving={kx.get('trend_improving', kx.get('trend_converging', False))}")
    print(f"kalman_velocity_last_m_s={kx.get('velocity_last_m_s')}")
    print(f"kalman_attitude_last_deg={kx.get('attitude_last_deg')}")
    print(f"handoff_hover_ok={hover.get('ok', False)}")
    print(f"handoff_release_rc={hover.get('release_rc')}")
    print(f"handoff_mode={hover.get('mode')}")
    print(f"handoff_release_rc={hover.get('release_rc')}")
    print(f"handoff_hold_pose={hover.get('hold_pose')}")
    print(f"handoff_hold_pose_source={hover.get('hold_pose_source')}")
    print(f"handoff_last_est_pose={hover.get('last_est_pose')}")
    print(f"handoff_extpos_send_ok={hover.get('extpos_send_ok')}")
    print(f"handoff_pose_rejects={hover.get('pose_rejects')}")
    print(f"handoff_pose_reject_reasons={hover.get('pose_reject_reasons')}")
    print(f"handoff_hover_z={hover.get('hover_z')}")
    print(f"handoff_hover_z_source={hover.get('hover_z_source')}")
    print(f"handoff_hover_timeout_s={hover.get('hover_timeout_s')}")
    print(f"handoff_hover_n_full_min={hover.get('n_full_min', '?')}")
    print(f"handoff_hover_n_full_recent_avg={hover.get('n_full_recent_avg', '?')}")
    print(f"handoff_hover_attitude_stats={hover.get('attitude_stats')}")
    print(f"generic_image_motion_ok={generic_motion.get('ok', False)}")
    print(f"generic_image_motion_abort_reason={generic_motion.get('abort_reason', '')}")
    print(f"generic_image_motion_mode={generic_motion.get('mode')}")
    print(f"generic_image_motion_target_tol_frac={generic_motion.get('target_tol_frac')}")
    print(f"generic_image_motion_target_tol_px_xy={generic_motion.get('target_tol_px_xy')}")
    print(f"generic_image_motion_perp_tol_frac={generic_motion.get('perp_tol_frac')}")
    print(f"generic_image_motion_perp_tol_px_xy={generic_motion.get('perp_tol_px_xy')}")
    print(f"generic_image_motion_image_envelope={generic_motion.get('image_envelope')}")
    print(f"generic_image_motion_edge_margin_px={generic_motion.get('edge_margin_px')}")
    print(f"generic_image_motion_vmax_m_s={generic_motion.get('vmax_m_s')}")
    print(f"generic_image_motion_hover_default_timeout_s={generic_motion.get('hover_default_timeout_s')}")
    print(f"generic_image_motion_n_full_min={generic_motion.get('n_full_min', '?')}")
    print(f"generic_image_motion_n_full_recent_avg={generic_motion.get('n_full_recent_avg', '?')}")
    print(f"generic_image_motion_extpos_send_ok={generic_motion.get('extpos_send_ok')}")
    print(f"generic_image_motion_pose_rejects={generic_motion.get('pose_rejects')}")
    print(f"generic_image_motion_last_est_pose={generic_motion.get('last_est_pose')}")
    for seg in generic_motion.get("segments", []):
        print(
            "generic_image_segment "
            f"label={seg.get('label')} "
            f"target={seg.get('target')} "
            f"max_s={seg.get('max_s')} "
            f"timeout_s={seg.get('timeout_s')} "
            f"hover_rc={seg.get('hover_rc_last')} "
            f"reached={seg.get('reached')} "
            f"image_err_first_px={seg.get('image_err_first_px')} "
            f"image_err_last_px={seg.get('image_err_last_px')} "
            f"perp_abs_max_px={seg.get('perp_abs_max_px')} "
            f"axis_guard_last={seg.get('axis_guard_last')} "
            f"n_recent={seg.get('n_full_recent_avg')} "
            f"err_first_m={seg.get('est_target_err_first_m')} "
            f"err_last_m={seg.get('est_target_err_last_m')} "
            f"attitude={seg.get('attitude_stats')}"
        )
    print(f"land_ok={land.get('ok', False)}")
    print(f"land_rc={land.get('land_rc')}")
    print(f"land_timeout_s={land.get('land_timeout_s')}")
    print(f"pre_land_relax_rc={land.get('pre_land_relax_rc')}")
    print(f"land_extpos_send_ok={land.get('extpos_send_ok')}")
    print(f"land_pose_rejects={land.get('pose_rejects')}")
    print(f"land_n_full_min={land.get('n_full_min', '?')}")
    print(f"land_n_full_recent_avg={land.get('n_full_recent_avg', '?')}")
    print(f"land_last_est_pose={land.get('last_est_pose')}")
    print(f"land_attitude_stats={land.get('attitude_stats')}")

    mission_ok = summary.get("status") in (
        "HANDOFF_SMOKE_OK", "GENERIC_AXIS_MOTION_OK")
    generic_ok = False
    if summary.get("status") == "GENERIC_AXIS_MOTION_OK":
        segments = generic_motion.get("segments", [])
        generic_ok = bool(segments) and all(
            bool(seg.get("reached", False)) and seg.get("hover_rc_last") == 0
            for seg in segments)

    if gt:
        xs = [float(r.get("x", 0.0)) for r in gt if "x" in r]
        ys = [float(r.get("y", 0.0)) for r in gt if "y" in r]
        zs = [float(r.get("z", 0.0)) for r in gt if "z" in r]
        print(f"gt_samples={len(gt)}")
        print("gt_x_span min={:.3f} max={:.3f} span={:.3f}".format(*span(xs)))
        print("gt_y_span min={:.3f} max={:.3f} span={:.3f}".format(*span(ys)))
        print("gt_z_span min={:.3f} max={:.3f} span={:.3f}".format(*span(zs)))
        for phase in ("center_hold", "extpos_warmup", "handoff_hover",
                      "generic_axis_motion", "land"):
            if phase in phase_intervals:
                t0, t1 = phase_intervals[phase]
                print(f"gt_attitude_{phase}={attitude_stats_from_gt(gt, t0, t1)}")
        seg_rows, gt_axis_ok = gt_axis_motion_stats(
            gt, load_generic_segment_intervals(workdir / "mission_s203_journal.txt"))
        for row in seg_rows:
            print(f"gt_axis_segment={row}")
        print(f"gt_axis_motion_ok={gt_axis_ok}")
    else:
        gt_axis_ok = False

    ok = mission_ok
    if summary.get("status") == "GENERIC_AXIS_MOTION_OK":
        ok = ok and generic_ok
    print(f"success={ok}")

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

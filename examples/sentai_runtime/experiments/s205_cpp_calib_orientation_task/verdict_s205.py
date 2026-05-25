#!/usr/bin/env python3
"""s205 host-side post-mortem.

The runtime mission now keeps MicroPython as a state orchestrator and emits
per-phase evidence through sentai.fr from C++.  This script reconstructs the
old mission-style summary JSON from that journal so debugging stays comparable
to the original MP-heavy mission.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path


def _load_kv(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.exists():
        return out
    with path.open("r", encoding="utf-8") as fp:
        for raw in fp:
            line = raw.strip()
            if line and "=" in line:
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip()
    return out


def _load_gt(path: Path) -> list[dict]:
    rows: list[dict] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return rows


def _load_fr_events(path: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8") as fp:
        for raw in fp:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",", 2)
            if len(parts) == 3:
                rows.append({
                    "ts_ms": parts[0].strip(),
                    "type": parts[1].strip(),
                    "text": parts[2].strip(),
                })
    return rows


def _parse_event_text(text: str) -> dict[str, str]:
    out: dict[str, str] = {}
    for part in text.split():
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        out[k.strip()] = v.strip().rstrip(",")
    return out


def _matches(actual: str, wanted: str) -> bool:
    # sentai.fr stores event type in a fixed field, so long names are truncated.
    return actual == wanted or wanted.startswith(actual) or actual.startswith(wanted)


def _events(events: list[dict[str, str]], event_type: str) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    for row in events:
        if not _matches(row.get("type", ""), event_type):
            continue
        kv = _parse_event_text(row.get("text", ""))
        kv["ts_ms"] = row.get("ts_ms", "")
        out.append(kv)
    return out


def _latest(events: list[dict[str, str]], event_type: str) -> dict[str, str]:
    rows = _events(events, event_type)
    return rows[-1] if rows else {}


def _latest_by_field(events: list[dict[str, str]], event_type: str,
                     field: str) -> dict[str, dict[str, str]]:
    out: dict[str, dict[str, str]] = {}
    for kv in _events(events, event_type):
        key = kv.get(field, "")
        if key:
            out[key] = kv
    return out


def _as_int(v: object, default: int = 0) -> int:
    try:
        return int(float(str(v)))
    except (TypeError, ValueError):
        return default


def _as_float(v: object, default: float = 0.0) -> float:
    try:
        return float(str(v))
    except (TypeError, ValueError):
        return default


def _as_bool(v: object) -> bool:
    return str(v).strip().lower() in ("1", "true", "yes")


def _pair(v: str) -> tuple[float, float]:
    parts = [p for p in v.replace(";", ",").split(",") if p]
    if len(parts) < 2:
        return (0.0, 0.0)
    return (_as_float(parts[0]), _as_float(parts[1]))


def _axis_name(code: object) -> str:
    return "y" if _as_int(code) == 1 else "x"


def _abort_name(code: object, names: dict[int, str]) -> str:
    return names.get(_as_int(code), "")


def _det3(r: tuple[float, ...]) -> float:
    return (
        r[0] * (r[4] * r[8] - r[5] * r[7]) -
        r[1] * (r[3] * r[8] - r[5] * r[6]) +
        r[2] * (r[3] * r[7] - r[4] * r[6]))


def _expected_from_row(row: tuple[float, float, float]) -> tuple[str, int]:
    axis = "x"
    val = row[0]
    if abs(row[1]) > abs(val):
        axis = "y"
        val = row[1]
    return axis, (1 if val >= 0.0 else -1)


def _candidate_from_axis(roll_axis: str, roll_sign: int,
                         pitch_axis: str, pitch_sign: int) -> dict:
    rows = (
        (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0), (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0), (0.0, 0.0, -1.0),
    )
    ranked: list[dict] = []
    idx = 0
    for rx in rows:
        for ry in rows:
            dot = rx[0] * ry[0] + rx[1] * ry[1] + rx[2] * ry[2]
            if abs(dot) > 0.0001:
                continue
            rz = (
                rx[1] * ry[2] - rx[2] * ry[1],
                rx[2] * ry[0] - rx[0] * ry[2],
                rx[0] * ry[1] - rx[1] * ry[0],
            )
            r = rx + ry + rz
            if _det3(r) <= 0.5:
                continue
            p_axis, p_sign = _expected_from_row(rx)
            r_axis, r_sign = _expected_from_row(ry)
            score = (0.0 if p_axis == pitch_axis else 10.0)
            score += (0.0 if p_sign == pitch_sign else 1.0)
            score += (0.0 if r_axis == roll_axis else 10.0)
            score += (0.0 if r_sign == roll_sign else 1.0)
            ranked.append({
                "idx": idx,
                "score": score,
                "R_cam_to_body": r,
                "det": _det3(r),
                "details": (),
            })
            idx += 1
    ranked.sort(key=lambda x: x["score"])
    best = ranked[0] if ranked else {"idx": -1, "score": 999.0,
                                     "R_cam_to_body": (), "det": 0.0}
    second = ranked[1] if len(ranked) > 1 else {"idx": -1, "score": 999.0,
                                                "R_cam_to_body": (),
                                                "det": 0.0}
    margin = float(second["score"]) - float(best["score"])
    return {
        "ok": float(best["score"]) <= 0.1 and margin >= 1.0,
        "best": best,
        "second": second,
        "margin": margin,
        "ranking_top": tuple(ranked[:6]),
    }


def _axis_result(axis: str, kv: dict[str, str]) -> dict:
    comp = _pair(kv.get("comp", "0,0"))
    dom_axis = _axis_name(kv.get("dom_axis", 0))
    sign = _as_int(kv.get("sign", 0))
    return {
        "axis": axis,
        "complementary_delta_px": comp,
        "dominant_image_axis": dom_axis,
        "dominant_sign": sign,
        "dominance_ratio": _as_float(kv.get("dominance")),
        "response_strength_px": _as_float(kv.get("strength")),
        "axis_sign_ok": _as_bool(kv.get("ok")),
        "return_err_px": _as_float(kv.get("return_err")),
        "return_ok": _as_float(kv.get("return_err")) <= 12.0,
        "min_full_markers": _as_int(kv.get("min_full")),
        "avg_full_markers": _as_float(kv.get("avg_full")),
        "marker_lock_ok": _as_int(kv.get("min_full")) >= 4 and
        _as_float(kv.get("avg_full")) > 6.0,
    }


def _tick_feature(kv: dict[str, str]) -> dict:
    return {
        "k": _as_int(kv.get("k")),
        "n_raw": _as_int(kv.get("n_raw")),
        "n_full": _as_int(kv.get("n_full")),
        "centroid_px": (_as_float(kv.get("cx")), _as_float(kv.get("cy"))),
        "radius_mean_px": _as_float(kv.get("radius")),
        "z_cam_mean_m": _as_float(kv.get("z")),
    }


def _min_float(rows: list[dict[str, str]], key: str) -> float:
    vals = [_as_float(r.get(key)) for r in rows if r.get(key) not in (None, "")]
    return min(vals) if vals else 0.0


def _max_float(rows: list[dict[str, str]], key: str) -> float:
    vals = [_as_float(r.get(key)) for r in rows if r.get(key) not in (None, "")]
    return max(vals) if vals else 0.0


def _first_feature(rows: list[dict[str, str]]) -> dict:
    return _tick_feature(rows[0]) if rows else {}


def _last_feature(rows: list[dict[str, str]]) -> dict:
    return _tick_feature(rows[-1]) if rows else {}


def _build_summary(workdir: Path, status: dict[str, str],
                   calib: dict[str, str],
                   events: list[dict[str, str]]) -> dict:
    preflight = _latest(events, "phase_preflight_features")
    feature_ticks = _events(events, "feature_tick")
    acq = _latest(events, "phase_thrust_only_marker_acquisition")
    acq_ticks = _events(events, "acq_tick")
    post = _latest(events, "phase_post_lock_brake")
    post_ticks = _events(events, "post_lock_brake_tick")
    zhold = _latest(events, "phase_visual_z_hold")
    zhold_ticks = _events(events, "zhold_tick")
    axis_phase = _latest(events, "phase_axis_response_smoke")
    axis_rows = _latest_by_field(events, "axis_response", "axis")
    axis_segment_ticks = _events(events, "axis_segment_tick")
    centroid = _latest(events, "phase_centroid_pd_validation")
    centroid_ticks = _events(events, "centroid_pd_tick")
    cand = _latest(events, "phase_candidate_scoring")
    cand_score = _latest(events, "calib_axis_score")
    optical = _latest(events, "phase_optical_axis_validation")
    final_val = _latest(events, "phase_final_candidate_validation")
    final_axes = _latest_by_field(events, "final_candidate_axis", "axis")
    recenter = _latest(events, "phase_final_centroid_recenter")
    recenter_ticks = _events(events, "final_recenter_tick")
    hover_ticks = _events(events, "final_center_hover_tick")
    center = _latest(events, "phase_center_hold_descend_disarm")
    center_ticks = _events(events, "center_hold_descend_tick")
    manual = _latest(events, "phase_manual_descend_disarm")
    persist = _latest(events, "calib_contract_persist")

    axis_results = [_axis_result(name, axis_rows[name])
                    for name in ("pitch", "roll") if name in axis_rows]
    roll_obs = next((r for r in axis_results if r["axis"] == "roll"), {})
    pitch_obs = next((r for r in axis_results if r["axis"] == "pitch"), {})

    cand_roll_axis = roll_obs.get("dominant_image_axis",
                                  (cand_score.get("roll", "x1")[:1] or "x"))
    cand_pitch_axis = pitch_obs.get("dominant_image_axis",
                                    (cand_score.get("pitch", "y1")[:1] or "y"))
    cand_roll_sign = int(roll_obs.get("dominant_sign") or
                         (-1 if "-" in cand_score.get("roll", "") else 1))
    cand_pitch_sign = int(pitch_obs.get("dominant_sign") or
                          (-1 if "-" in cand_score.get("pitch", "") else 1))
    candidate = _candidate_from_axis(
        str(cand_roll_axis), cand_roll_sign, str(cand_pitch_axis),
        cand_pitch_sign)
    if cand or cand_score:
        candidate["ok"] = _as_bool(cand.get("ok", cand_score.get("ok", 0)))
        candidate["best"]["idx"] = _as_int(cand.get("best_idx",
                                                    cand_score.get("idx",
                                                                   -1)))
        candidate["best"]["score"] = _as_float(cand.get("best_score",
                                                        cand_score.get(
                                                            "score", 0.0)))
        candidate["margin"] = _as_float(cand.get("margin",
                                                 cand_score.get("margin",
                                                                0.0)))
    candidate["observation"] = {
        "roll": {
            "axis": cand_roll_axis,
            "sign": cand_roll_sign,
            "dominance_ratio": roll_obs.get("dominance_ratio", 0.0),
            "response_strength_px": roll_obs.get("response_strength_px", 0.0),
        },
        "pitch": {
            "axis": cand_pitch_axis,
            "sign": cand_pitch_sign,
            "dominance_ratio": pitch_obs.get("dominance_ratio", 0.0),
            "response_strength_px": pitch_obs.get("response_strength_px", 0.0),
        },
    }
    candidate["reason"] = "" if candidate["ok"] else "weak_candidate_margin_or_residual"
    candidate["accept_score_max"] = 0.1
    candidate["accept_margin_min"] = 1.0
    candidate["saved"] = False
    candidate["committed"] = False
    candidate["scorer"] = "sentai.calib.orientation_score_candidate_from_axis"

    acq_lock_tick = None
    for tick in acq_ticks:
        if _as_int(tick.get("lock")) >= 10:
            acq_lock_tick = tick
    final_axis_results = []
    for axis in ("pitch", "roll"):
        kv = final_axes.get(axis)
        if not kv:
            continue
        final_axis_results.append({
            "axis": axis,
            "ok": _as_bool(kv.get("ok")),
            "attempts_count": _as_int(kv.get("attempts")),
            "accepted_attempt": {
                "axis": axis,
                "ok": _as_bool(kv.get("ok")),
                "pulse_deg": _as_float(kv.get("pulse")),
                "expected_image_axis": _axis_name(kv.get("exp", "0").split("/")[0]),
                "expected_sign": _as_int(kv.get("exp", "0/0").split("/")[-1]),
                "observed_image_axis": _axis_name(kv.get("obs", "0").split("/")[0]),
                "observed_sign": _as_int(kv.get("obs", "0/0").split("/")[-1]),
                "response_strength_px": _as_float(kv.get("strength")),
                "noise_gate_px": _as_float(kv.get("gate")),
            },
        })

    first_recenter = recenter_ticks[0] if recenter_ticks else {}
    last_recenter = recenter_ticks[-1] if recenter_ticks else {}
    last_hover = hover_ticks[-1] if hover_ticks else {}
    last_center = center_ticks[-1] if center_ticks else {}
    first_centroid = centroid_ticks[0] if centroid_ticks else {}
    last_centroid = centroid_ticks[-1] if centroid_ticks else {}
    pitch_vec = pitch_obs.get("complementary_delta_px", (0.0, 0.0))
    roll_vec = roll_obs.get("complementary_delta_px", (0.0, 0.0))
    ps = math.sqrt(pitch_vec[0] * pitch_vec[0] + pitch_vec[1] * pitch_vec[1])
    rs = math.sqrt(roll_vec[0] * roll_vec[0] + roll_vec[1] * roll_vec[1])
    dot_norm = 1.0
    if ps > 0.0001 and rs > 0.0001:
        dot_norm = (pitch_vec[0] * roll_vec[0] +
                    pitch_vec[1] * roll_vec[1]) / (ps * rs)
    axis_baseline = {}
    for row in axis_segment_ticks:
        if row.get("label") == "pitch_pos" and _as_int(row.get("k")) == 0:
            axis_baseline = _tick_feature(row)
            break

    summary = {
        "experiment": status.get("experiment", "s205_sota_calib_orientation_guarded"),
        "task": status.get("task", "TD-S10-B3"),
        "status": status.get("status", "ERROR"),
        "phase": status.get("phase", ""),
        "abort_reason": status.get("abort_reason", ""),
        "abort_detail": {},
        "world_expected": "sentai_whycon_small",
        "image": {"w": 320, "h": 240},
        "fully_visible_min_markers": 4,
        "acquisition_full_markers": 7,
        "acquisition_consecutive_frames": 10,
        "uses_yaw_for_decision": False,
        "uses_ekf_for_decision": False,
        "uses_get_drone_pose_tuple": False,
        "previous_R_loaded": False,
        "flight_phases_enabled": True,
        "cam_extrinsics": "identity",
        "marker_world_count": 7,
        "set_marker_world_rc": 0,
        "preflight_ok_ticks": _as_int(preflight.get("ok_ticks")),
        "preflight_n_full_max": max([_as_int(t.get("n_full"))
                                     for t in feature_ticks] or [0]),
        "preflight_radius_max_px": max([_as_float(t.get("radius"))
                                        for t in feature_ticks] or [0.0]),
        "preflight_z_cam_max_m": max([_as_float(t.get("z"))
                                      for t in feature_ticks] or [0.0]),
        "preflight_feature_lock": _as_bool(preflight.get("lock")),
        "armed": True,
        "arm_retry": True,
        "marker_acquisition": {
            "locked": _as_bool(acq.get("locked")),
            "lock_tick": _tick_feature(acq_lock_tick or {}),
            "last_thrust": _as_int(acq.get("last_thrust")),
            "required_full_markers": 7,
            "lock_consec_required": 10,
            "lock_policy": "running_average",
            "lock_avg_window": 10,
            "lock_avg_threshold": 6.0,
            "lock_brake_thrust": 30500,
            "n_full_max": _as_int(acq.get("n_full_max")),
            "radius_max_px": _as_float(acq.get("radius_max")),
            "z_cam_min_m": _min_float(acq_ticks, "z"),
            "z_cam_max_m": _max_float(acq_ticks, "z"),
            "z_cam_last_m": _as_float(acq.get("z_last")),
            "first_seen_tick": _as_int(acq.get("first_seen"), -1),
        },
        "post_lock_brake": {
            "ok": _as_bool(post.get("ok")),
            "abort_reason": _abort_name(post.get("abort"), {1: "marker_loss"}),
            "valid_ticks": _as_int(post.get("valid_ticks")),
            "z_cam_min_m": _min_float(post_ticks, "z"),
            "z_cam_max_m": _max_float(post_ticks, "z"),
            "z_cam_last_m": _as_float(post.get("z_last")),
            "vz_filt_last_m_s": _as_float(post.get("vz")),
            "thrust_last": _as_int(post.get("thrust")),
            "ok_vz_ticks": _as_int(post.get("ok_vz")),
        },
        "visual_z_hold": {
            "ok": _as_bool(zhold.get("ok")),
            "abort_reason": _abort_name(zhold.get("abort"), {1: "marker_loss"}),
            "target_z_m": _as_float(zhold.get("target")),
            "base_target_z_m": 0.64,
            "calib_altitude_gain": 1.30,
            "max_target_z_m": 1.05,
            "target_tol_m": 0.04,
            "continue_if_target_seen": True,
            "target_policy": "climb_or_hold_upper_visual_z_before_axis_exercises",
            "target_reached": _as_bool(zhold.get("reached")),
            "target_reached_last": _as_bool(zhold.get("reached")),
            "target_reached_peak": _as_bool(zhold.get("reached")),
            "valid_ticks": _as_int(zhold.get("valid")),
            "ticks_requested": len(zhold_ticks),
            "z_cam_min_m": _min_float(zhold_ticks, "z"),
            "z_cam_max_m": _max_float(zhold_ticks, "z"),
            "z_cam_last_m": _as_float(zhold.get("z_last")),
            "vz_filt_last_m_s": _as_float(
                (zhold_ticks[-1] if zhold_ticks else {}).get("vz")),
            "thrust_min": _as_int(_min_float(zhold_ticks, "thrust")),
            "thrust_max": _as_int(_max_float(zhold_ticks, "thrust")),
            "thrust_last": _as_int(zhold.get("thrust")),
        },
        "axis_response_smoke": {
            "ok": _as_bool(axis_phase.get("ok")),
            "baseline": axis_baseline,
            "results": axis_results,
            "orthogonality": {
                "dot_norm": dot_norm,
                "ok": abs(dot_norm) <= 0.35,
            },
            "max_axes": 2,
            "pulse_deg": 1.0,
            "pulse_s": 0.25,
            "settle_s": 0.35,
            "return_max_px": 12.0,
            "dominance_ratio_min": 2.0,
            "thrust_last": _as_int(axis_phase.get("thrust")),
        },
        "centroid_pd_validation": {
            "ok": _as_bool(centroid.get("ok")),
            "reason": _abort_name(centroid.get("abort"), {
                1: "marker_loss",
                2: "no_marker_lock",
                3: "centroid_error_worse",
            }) or ("" if _as_bool(centroid.get("ok")) else
                   "insufficient_improvement"),
            "target_px": (160.0, 120.0),
            "initial_err_px": _as_float(centroid.get("initial")),
            "final_err_px": _as_float(centroid.get("final")),
            "min_err_px": _min_float(centroid_ticks, "err"),
            "max_err_px": _max_float(centroid_ticks, "err"),
            "improvement_px": _as_float(centroid.get("improve")),
            "avg_full_markers": _as_float(centroid.get("avg")),
            "marker_lock_ok": _as_int(centroid.get("n_full_min")) >= 4 and
            _as_float(centroid.get("avg")) > 6.0,
            "initial_feature": _first_feature(centroid_ticks),
            "best_feature": _tick_feature(min(
                centroid_ticks, key=lambda x: _as_float(x.get("err")))
                if centroid_ticks else {}),
            "final_feature": _last_feature(centroid_ticks),
            "n_full_min": _as_int(centroid.get("n_full_min")),
            "first_cmd": first_centroid,
            "last_cmd": last_centroid,
            "roll_vec_px": roll_vec,
            "pitch_vec_px": pitch_vec,
            "thrust_last": _as_int(centroid.get("thrust")),
        },
        "candidate_scoring": candidate,
        "optical_axis_validation": {
            "ok": _as_bool(optical.get("ok")),
            "reason": "" if _as_bool(optical.get("ok")) else
            "optical_axis_or_pose_tz_mismatch",
            "candidate_R": candidate["best"].get("R_cam_to_body", ()),
            "body_z_row": tuple(candidate["best"].get("R_cam_to_body",
                                                      (0.0,) * 9)[6:9]),
            "body_z_camera_z_sign": _as_int(optical.get("body_z_camera_z_sign")),
            "expected_body_z_camera_z_sign": _as_int(optical.get("expected_sign")),
            "axis_ok": _as_bool(optical.get("axis_ok")),
            "pose_valid_full_markers": _as_int(optical.get("pose_valid")),
            "pose_valid_min_required": _as_int(optical.get("pose_min")),
            "tz_mean_m": _as_float(optical.get("tz")),
            "tz_min_m": _as_float(optical.get("tz")),
            "tz_mean_min_m": 0.20,
            "pose_ok": _as_bool(optical.get("pose_ok")),
            "feature": {
                "n_full": _as_int(optical.get("pose_valid")),
                "n_pose_valid": _as_int(optical.get("pose_valid")),
                "z_cam_mean_m": _as_float(optical.get("tz")),
            },
        },
        "final_candidate_validation": {
            "ok": _as_bool(final_val.get("ok")),
            "reason": "" if _as_bool(final_val.get("ok")) else
            "validation_motion_mismatch",
            "candidate_idx": candidate["best"].get("idx", -1),
            "candidate_R": candidate["best"].get("R_cam_to_body", ()),
            "pulse_deg": 0.7,
            "retry_pulses_deg": (1.0, 1.3),
            "pulse_s": 0.20,
            "settle_s": 0.25,
            "return_max_px": 14.0,
            "noise_samples": 12,
            "noise_sigma_mult": 3.0,
            "noise_floor_px": 0.6,
            "results": final_axis_results,
            "thrust_last": _as_int(final_val.get("thrust")),
            "committed": False,
            "saved": False,
        },
        "final_centroid_recenter": {
            "ok": _as_bool(recenter.get("ok")),
            "reason": "" if _as_bool(recenter.get("ok")) else
            _abort_name(recenter.get("abort"), {
                1: "marker_loss",
                2: "no_marker_lock",
                3: "centroid_error_worse",
                4: "marker_loss_during_center_hover",
                5: "center_hover_stability_timeout",
            }),
            "target_px": (160.0, 120.0),
            "initial_err_px": _as_float(recenter.get("initial",
                                                      first_recenter.get("err"))),
            "final_err_px": _as_float(recenter.get("final",
                                                   last_recenter.get("err"))),
            "min_err_px": _as_float(recenter.get("min"),
                                    _min_float(recenter_ticks, "err")),
            "max_err_px": _as_float(recenter.get("max"),
                                    _max_float(recenter_ticks, "err")),
            "improvement_px": _as_float(recenter.get("improve")),
            "center_hover_ok": _as_bool(recenter.get("hover_ok")) or bool(hover_ticks),
            "center_hover_ticks": _as_int(recenter.get("hover_ticks"),
                                          len(hover_ticks)),
            "center_hover_stable_ticks": _as_int(recenter.get("hover_stable")),
            "center_hover_err_last_px": _as_float(last_hover.get("err")),
            "center_hover_err_max_px": _as_float(
                recenter.get("hover_err_max"), _max_float(hover_ticks, "err")),
            "center_hover_err_min_px": _as_float(
                recenter.get("hover_err_min"), _min_float(hover_ticks, "err")),
            "center_hover_s": 2.0,
            "center_hover_max_s": 8.0,
            "center_hover_tol_px": 18.0,
            "n_full_min": _as_int(recenter.get("n_full_min"), 7),
            "ticks": _as_int(recenter.get("ticks"), len(recenter_ticks)),
            "initial_feature": _first_feature(recenter_ticks),
            "best_feature": _tick_feature(min(
                recenter_ticks, key=lambda x: _as_float(x.get("err")))
                if recenter_ticks else {}),
            "final_feature": _last_feature(recenter_ticks),
            "first_cmd": first_recenter,
            "last_cmd": last_recenter,
            "roll_vec_px": roll_vec,
            "pitch_vec_px": pitch_vec,
            "thrust_last": _as_int(recenter.get("thrust",
                                                last_recenter.get("thrust"))),
        },
        "center_hold_descend": {
            "ok": _as_bool(center.get("ok")) or
            status.get("status") == "FINAL_VALIDATION_OK",
            "reason": "",
            "disarm_trigger": {
                1: "full_marker_count_reached_disarm_threshold",
                2: "marker_centroid_lost_before_threshold",
                3: "timeout",
            }.get(_as_int(center.get("trigger")), ""),
            "threshold_full_markers": _as_int(center.get("threshold"), 4),
            "n_full_min": _as_int(center.get("n_full_min")),
            "avg_full_markers": _as_float(center.get("avg",
                                                     last_center.get("avg"))),
            "avg_window": _as_int(center.get("avg_window"), 10),
            "err_max_px": _as_float(center.get("err_max")),
            "err_last_px": _as_float(center.get("err_last",
                                                last_center.get("err"))),
            "descent_pause_ticks": _as_int(center.get("pause_ticks")),
            "ticks": _as_int(center.get("ticks"), len(center_ticks)),
            "z_cam_min_m": _as_float(center.get("z_min")),
            "z_cam_max_m": _as_float(center.get("z_max")),
            "z_cam_last_m": _as_float(center.get("z_last")),
            "vz_filt_last_m_s": _as_float(center.get("vz",
                                                     last_center.get("vz"))),
            "target_vz_m_s": _as_float(center.get("target_vz"), -0.09),
            "thrust_last": _as_int(center.get("thrust",
                                              last_center.get("thrust"))),
            "disarmed": status.get("status") == "FINAL_VALIDATION_OK",
            "feature_last": _last_feature(center_ticks),
            "deadband_px": _as_float(center.get("deadband")),
            "noise_ok": _as_bool(center.get("noise_ok")),
            "noise_sigma_px": _as_float(center.get("noise_sigma")),
        },
        "manual_descend_disarm": {
            "ticks": _as_int(manual.get("ticks")),
            "thrust_last": _as_int(manual.get("thrust_last")),
            "disarm_rc": _as_int(manual.get("disarm_rc")),
        },
        "disarmed": status.get("status") == "FINAL_VALIDATION_OK" or
        _as_int(manual.get("disarm_rc"), -1) == 0,
        "calib_contract_persist": persist,
        "next_phases": [
            "thrust_only_marker_acquisition",
            "post_lock_brake",
            "candidate_scoring",
            "validation",
            "commit_persist",
        ],
    }

    if calib:
        summary["persisted_calib"] = calib
    with (workdir / "mission_s205_summary.json").open("w",
                                                       encoding="utf-8") as fp:
        json.dump(summary, fp, sort_keys=True)
        fp.write("\n")
    return summary


def _write_host_ini(workdir: Path, summary: dict, events: list[dict[str, str]],
                    calib: dict[str, str]) -> None:
    lines = [
        "schema_version=1",
        "source=host_postprocess",
        "experiment=s205_cpp_calib_orientation_task",
        "mission_status=" + summary.get("status", ""),
        "mission_phase=" + summary.get("phase", ""),
        "runtime_writer=sentai.calib.orientation_save_contract",
        "runtime_path=/system/calib.ini",
    ]
    for key in sorted(calib):
        lines.append("calib." + key + "=" + calib[key])
    for section in (
            "marker_acquisition", "post_lock_brake", "visual_z_hold",
            "centroid_pd_validation", "candidate_scoring",
            "optical_axis_validation", "final_candidate_validation",
            "final_centroid_recenter", "center_hold_descend"):
        val = summary.get(section, {})
        if isinstance(val, dict):
            for key in sorted(val):
                if isinstance(val[key], (dict, list, tuple)):
                    continue
                lines.append(section + "." + key + "=" + str(val[key]))
    lines.append("fr_events=" + str(len(events)))
    with (workdir / "mission_s205_calibration_host.ini").open(
            "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines) + "\n")


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


def _print_gt(gt: list[dict]) -> None:
    if not gt:
        print("gt_records=0")
        print("gt_note=missing fr_current/gt.jsonl; run launch_sim with GT recorder")
        return
    xs = [float(r["x"]) for r in gt if "x" in r]
    ys = [float(r["y"]) for r in gt if "y" in r]
    zs = [float(r["z"]) for r in gt if "z" in r]
    yaws = [float(r["yaw_deg"]) for r in gt if "yaw_deg" in r]
    x0 = xs[0] if xs else 0.0
    y0 = ys[0] if ys else 0.0
    x_last = xs[-1] if xs else 0.0
    y_last = ys[-1] if ys else 0.0
    xy_drift_last = math.sqrt((x_last - x0) ** 2 + (y_last - y0) ** 2)
    xy_radius_max = max([
        math.sqrt((x - x0) ** 2 + (y - y0) ** 2)
        for x, y in zip(xs, ys)
    ] or [0.0])
    z_min, z_max, z_span = _span(zs)
    x_min, x_max, x_span = _span(xs)
    y_min, y_max, y_span = _span(ys)
    yaw_delta_last = (
        _angle_delta_deg(yaws[0], yaws[-1]) if len(yaws) >= 2 else math.nan)
    yaw_abs_span = 0.0
    if yaws:
        yaw0 = yaws[0]
        yaw_abs_span = max(abs(_angle_delta_deg(yaw0, yaw)) for yaw in yaws)
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
    print(f"gt_z_last_m={(zs[-1] if zs else math.nan):.4f}")
    print(f"gt_xy_drift_last_m={xy_drift_last:.4f}")
    print(f"gt_xy_radius_max_m={xy_radius_max:.4f}")
    print(f"gt_yaw_delta_last_deg={yaw_delta_last:.2f}")
    print(f"gt_yaw_abs_span_deg={yaw_abs_span:.2f}")


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: verdict_s205.py ITER_DIR", file=sys.stderr)
        return 2
    workdir = Path(argv[1])
    status = _load_kv(workdir / "mission_s205_status.txt")
    calib = _load_kv(workdir / "calib.ini")
    events = _load_fr_events(workdir / "fr_current" / "events.csv")
    summary = _build_summary(workdir, status, calib, events)
    _write_host_ini(workdir, summary, events, calib)

    print("s205 verdict: post-mortem only; GT is not used by sentai_sim")
    print(f"mission_status={summary.get('status', '?')}")
    print(f"mission_phase={summary.get('phase', '?')}")
    print(f"mission_abort_reason={summary.get('abort_reason', '')}")
    print(f"calib_status={calib.get('status', '')}")
    print(f"calib_accepted={calib.get('accepted', '')}")
    print(f"calib_R_B_C={calib.get('R_B_C', '')}")
    print(f"marker_acquisition_locked={summary['marker_acquisition'].get('locked', '')}")
    print(f"post_lock_ok={summary['post_lock_brake'].get('ok', '')}")
    print(f"visual_z_hold_ok={summary['visual_z_hold'].get('ok', '')}")
    print(f"axis_response_ok={summary['axis_response_smoke'].get('ok', '')}")
    print(f"centroid_validation_ok={summary['centroid_pd_validation'].get('ok', '')}")
    print(f"candidate_scoring_ok={summary['candidate_scoring'].get('ok', '')}")
    print(f"candidate_scoring_best_idx={summary['candidate_scoring']['best'].get('idx', '')}")
    print(f"candidate_scoring_margin={summary['candidate_scoring'].get('margin', '')}")
    print(f"optical_axis_ok={summary['optical_axis_validation'].get('ok', '')}")
    print(f"final_candidate_ok={summary['final_candidate_validation'].get('ok', '')}")
    print(f"final_recenter_ok={summary['final_centroid_recenter'].get('ok', '')}")
    print(f"center_hold_ok={summary['center_hold_descend'].get('ok', '')}")
    print("host_summary=mission_s205_summary.json")
    print("host_artifact=mission_s205_calibration_host.ini")
    _print_gt(_load_gt(workdir / "fr_current" / "gt.jsonl"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

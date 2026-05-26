#!/usr/bin/env python3
"""s207 post-mortem summary from sentai.fr.

MicroPython only orchestrates phases and writes the small status file.  The
rich debug artifact is reconstructed host-side from sentai.fr events, matching
the s203 workflow closely enough for regression/debug comparisons.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path


TARGET_NAMES = {
    "0": ("center",),
    "1": ("min_x",),
    "2": ("max_x",),
    "3": ("min_y",),
    "4": ("max_y",),
}


def parse_value(v: str):
    if v in ("", "None", "null"):
        return None
    if v in ("True", "true"):
        return True
    if v in ("False", "false"):
        return False
    try:
        if any(c in v for c in ".eE"):
            return float(v)
        return int(v)
    except ValueError:
        return v


def parse_kv(text: str) -> dict:
    out = {}
    for part in text.strip().split():
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        out[k] = parse_value(v)
    return out


def load_status(path: Path) -> dict:
    out = {}
    if not path.exists():
        return out
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k] = v
    return out


def load_events(path: Path) -> list[dict]:
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith("#"):
            continue
        parts = line.split(",", 2)
        if len(parts) < 3:
            continue
        try:
            ts_ms = int(parts[0])
        except ValueError:
            continue
        rows.append({
            "ts_ms": ts_ms,
            "event": parts[1],
            "text": parts[2],
            "kv": parse_kv(parts[2]),
        })
    return rows


def last_event(events: list[dict], name: str) -> dict:
    for row in reversed(events):
        if row["event"] == name:
            return row
    return {}


def first_event(events: list[dict], name: str) -> dict:
    for row in events:
        if row["event"] == name:
            return row
    return {}


def target_tol_px():
    return [320.0 * 0.05, 240.0 * 0.05]


def build_summary(workdir: Path) -> dict:
    events = load_events(workdir / "fr_current" / "events.csv")
    status = load_status(workdir / "mission_s207_status.txt")

    summary = {
        "experiment": status.get("experiment", "s207_cpp_marker_control_task"),
        "task": status.get("task", "TD-S10-B4"),
        "status": status.get("status", "?"),
        "phase": status.get("phase", "?"),
        "abort_reason": status.get("abort_reason", ""),
        "source": "sentai.fr events.csv",
        "goal": (
            "load strict calib.ini -> B4 marker acquire -> center hold -> "
            "sentai.flow consumer -> ExtPos warmup -> Generic hover handoff -> "
            "image-frame envelope motion -> land"
        ),
    }

    acq = last_event(events, "marker_acquire").get("kv", {})
    if acq:
        summary["acquire"] = {
            "ok": bool(acq.get("ok")),
            "ticks": acq.get("ticks"),
            "thrust_last": acq.get("thrust_last"),
            "n_full_max": acq.get("n_full_max"),
            "radius_max_px": acq.get("radius_max"),
            "n_full_avg_best": acq.get("n_avg_best"),
            "z_target_m": acq.get("z_target"),
            "z_cam_lock_m": acq.get("z_lock"),
            "z_cam_peak_m": acq.get("z_peak"),
            "centroid_avg_min_markers": acq.get("centroid_avg_min"),
            "centroid_seen_ticks": acq.get("centroid_seen"),
            "centroid_cmd_ticks": acq.get("centroid_cmd"),
            "centroid_err_last_px": acq.get("centroid_err_last"),
            "centroid_err_max_px": acq.get("centroid_err_max"),
            "first_seen_tick": acq.get("first_seen_tick"),
            "first_full_tick": acq.get("first_full_tick"),
        }

    setup = last_event(events, "marker_setup").get("kv", {})
    calib_setup = last_event(events, "calib_setup").get("kv", {})
    summary["calib_load_ok"] = bool(setup.get("ok"))
    summary["calib_layout_ok"] = bool(calib_setup.get("ok"))
    summary["calib_ini_layout_id"] = calib_setup.get("layout")

    center = last_event(events, "marker_center_hold").get("kv", {})
    if center:
        summary["center_hold"] = {
            "ok": bool(center.get("ok")),
            "ticks": center.get("ticks"),
            "n_full_min": center.get("n_full_min"),
            "n_full_recent_avg": center.get("avg"),
            "err_initial_px": center.get("err_initial"),
            "err_last_px": center.get("err_last"),
            "err_max_px": center.get("err_max"),
            "thrust_last": center.get("thrust"),
        }

    prep = last_event(events, "marker_estimator_prep").get("kv", {})
    if prep:
        summary["kalman_reset_before_extpos"] = {
            "ok": prep.get("kalman_reset_rc") == 0,
            "reset_rc": prep.get("kalman_reset_rc"),
            "bootstrap_hold_rc": prep.get("hold_rc"),
            "bootstrap_hold_ticks": prep.get("hold_ticks"),
        }
        summary["extpos_stddev"] = {
            "ok": prep.get("bootstrap_stddev_rc") == 0,
            "write_rc": prep.get("bootstrap_stddev_rc"),
            "requested_m": prep.get("bootstrap_stddev"),
        }

    extpos = last_event(events, "marker_extpos_warmup").get("kv", {})
    if extpos:
        summary["extpos_warmup"] = {
            "ok": bool(extpos.get("ok")),
            "ticks": extpos.get("ticks"),
            "send_ok": extpos.get("send_ok"),
            "pose_rejects": extpos.get("pose_rej"),
            "n_full_min": extpos.get("n_full_min"),
            "n_full_recent_avg": extpos.get("avg"),
            "converged": bool(extpos.get("converged")),
            "diverged": bool(extpos.get("diverged")),
            "marker_visibility_lost": bool(extpos.get("marker_lost")),
            "last_pose_reject_reason_code": extpos.get("reject_reason"),
            "flow": {
                "send_ok": extpos.get("flow"),
                "read_errors": extpos.get("flow_err"),
            },
            "kalman_crosscheck": {
                "err_first_m": extpos.get("err_first"),
                "err_last_m": extpos.get("err_last"),
                "err_recent10_mean_m": extpos.get("err_recent"),
                "err_min_m": extpos.get("err_min"),
                "err_max_m": extpos.get("err_max"),
                "trend_improving": bool(extpos.get("converged")),
            },
        }
        summary["extpos_stddev_flow_assisted"] = {
            "ok": extpos.get("std1") == 0,
            "write_rc": extpos.get("std1"),
            "requested_m": 0.12,
        }

    handoff = last_event(events, "marker_handoff_hover").get("kv", {})
    if handoff:
        summary["handoff_hover"] = {
            "ok": bool(handoff.get("ok")),
            "mode": "generic_hover_after_rpyt_release",
            "release_rc": handoff.get("release_rc"),
            "hover_rc_last": handoff.get("hover_rc"),
            "ticks": handoff.get("ticks"),
            "n_full_min": handoff.get("n_full_min"),
            "n_full_recent_avg": handoff.get("avg"),
            "hover_z": handoff.get("hover_z"),
            "hover_z_source": "cf_state_estimate_z_or_visual_pose",
            "extpos_send_ok": handoff.get("send_ok"),
            "pose_rejects": handoff.get("pose_rej"),
            "flow": {
                "send_ok": handoff.get("flow"),
                "read_errors": handoff.get("flow_err"),
            },
        }

    envelope = last_event(events, "marker_axis_envelope").get("kv", {})
    image_envelope = None
    if envelope:
        image_envelope = {
            "frame": "image_centroid_symmetric_safe_envelope",
            "min_x_cx": envelope.get("min_x"),
            "max_x_cx": envelope.get("max_x"),
            "min_y_cy": envelope.get("min_y"),
            "max_y_cy": envelope.get("max_y"),
            "amp_x_cap_px": envelope.get("cap_x"),
            "amp_y_cap_px": envelope.get("cap_y"),
        }

    segments = []
    for row in events:
        if row["event"] != "marker_axis_motion_segment":
            continue
        kv = row["kv"]
        target = TARGET_NAMES.get(str(kv.get("target")), (kv.get("target"),))
        max_ms = kv.get("max_ms") or 0
        segments.append({
            "label": kv.get("label"),
            "target": target,
            "target_frame": "image",
            "target_tol_frac": 0.05,
            "target_tol_px_xy": target_tol_px(),
            "perp_tol_frac": 0.05,
            "perp_tol_px_xy": target_tol_px(),
            "max_s": float(max_ms) / 1000.0,
            "timeout_s": float(max_ms + 600) / 1000.0,
            "reached": bool(kv.get("reached")),
            "reached_k": kv.get("reached_k"),
            "hover_rc_last": kv.get("hover_rc"),
            "n_full_min": kv.get("n_full_min"),
            "n_full_recent_avg": kv.get("avg"),
            "image_err_last_px": kv.get("err_last"),
            "extpos_send_ok_total": kv.get("send_ok"),
            "pose_rejects_total": kv.get("pose_rej"),
            "flow_send_ok_total": kv.get("flow"),
            "flow_read_errors_total": kv.get("flow_err"),
            "image_envelope": image_envelope,
        })

    axis = last_event(events, "marker_axis_motion").get("kv", {})
    if axis:
        summary["generic_axis_motion"] = {
            "ok": bool(axis.get("ok")),
            "mode": "generic_hover_image_frame",
            "abort_reason": "" if axis.get("ok") else "segment_not_reached",
            "segments": segments,
            "image_envelope": image_envelope,
            "target_tol_frac": 0.05,
            "target_tol_px_xy": target_tol_px(),
            "perp_tol_frac": 0.05,
            "perp_tol_px_xy": target_tol_px(),
            "edge_margin_px": 10.0,
            "vmax_m_s": 0.08,
            "hover_default_timeout_s": 10.0,
            "n_full_min": axis.get("n_full_min"),
            "n_full_recent_avg": axis.get("avg"),
            "extpos_send_ok": axis.get("send_ok"),
            "pose_rejects": axis.get("pose_rej"),
            "last_err_px": axis.get("err"),
            "flow": {
                "send_ok": axis.get("flow"),
                "read_errors": axis.get("flow_err"),
            },
        }

    land = last_event(events, "marker_land").get("kv", {})
    if land:
        summary["land"] = {
            "ok": bool(land.get("ok")),
            "pre_land_relax_rc": land.get("release_rc"),
            "hover_rc_last": land.get("hover_rc"),
            "disarm_rc": land.get("disarm_rc"),
            "ticks": land.get("ticks"),
            "n_full_min": land.get("n_full_min"),
            "n_full_recent_avg": land.get("avg"),
            "extpos_send_ok": land.get("send_ok"),
            "pose_rejects": land.get("pose_rej"),
            "land_timeout_s": 3.8,
            "start_z_m": land.get("z_start"),
            "last_z_cmd_m": land.get("z_last"),
            "flow": {
                "send_ok": land.get("flow"),
                "read_errors": land.get("flow_err"),
            },
        }

    return summary


def print_verdict(summary: dict) -> None:
    print("s207 verdict: host-side summary from sentai.fr")
    print(f"mission_status={summary.get('status')}")
    print(f"mission_phase={summary.get('phase')}")
    print(f"calib_load_ok={summary.get('calib_load_ok')}")
    print(f"calib_layout_ok={summary.get('calib_layout_ok')}")
    for key in ("acquire", "center_hold", "extpos_warmup",
                "handoff_hover", "generic_axis_motion", "land"):
        block = summary.get(key) or {}
        print(f"{key}_ok={block.get('ok', False)}")
    axis = summary.get("generic_axis_motion") or {}
    print(f"generic_image_motion_segments={len(axis.get('segments') or [])}")
    print(f"generic_image_motion_flow={axis.get('flow')}")
    land = summary.get("land") or {}
    print(f"land_flow={land.get('flow')}")


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: verdict_s207.py ITER_DIR", file=sys.stderr)
        return 2
    workdir = Path(argv[1])
    summary = build_summary(workdir)
    out_path = workdir / "mission_s207_summary.json"
    out_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print_verdict(summary)
    return 0 if summary.get("status") == "MARKER_CONTROL_OK" else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

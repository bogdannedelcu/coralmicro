#!/usr/bin/env python3
"""Convert an accepted A3 calibration JSON artifact to a single calib.ini."""

from __future__ import annotations

import json
import sys
from pathlib import Path


EXPECTED_LAYOUT = "sentai_whycon_small_centroid_7_marker_v1"
DEFAULT_CAM_OFFSET_B = (-0.04, 0.0, -0.02)


def fmt9(v: float) -> str:
    return f"{float(v):.9f}"


def fmt6(v: float) -> str:
    return f"{float(v):.6f}"


def main() -> int:
    if len(sys.argv) != 3:
        print(
            "usage: make_calib_ini.py <mission_calibration.json> <calib.ini>",
            file=sys.stderr)
        return 2

    src = Path(sys.argv[1])
    dst = Path(sys.argv[2])
    data = json.loads(src.read_text(encoding="utf-8"))

    if not data.get("accepted"):
        raise SystemExit(f"refusing unaccepted calibration artifact: {src}")
    if data.get("status") != "FINAL_VALIDATION_OK":
        raise SystemExit(
            f"refusing calibration status {data.get('status')!r}: {src}")
    if data.get("layout_id") != EXPECTED_LAYOUT:
        raise SystemExit(
            f"unexpected layout {data.get('layout_id')!r}: {src}")

    r = data.get("R_cam_to_body")
    if not isinstance(r, list) or len(r) != 9:
        raise SystemExit(f"missing 9-value R_cam_to_body: {src}")

    marker = data.get("marker") or {}
    image = data.get("image") or {}
    validation = data.get("validation") or {}
    safety = data.get("safety_envelope") or {}
    axis = data.get("axis_response") or {}
    roll = axis.get("roll") or {}
    pitch = axis.get("pitch") or {}

    lines = [
        "# Generated for TD-S10-B4/s201 from accepted A3/s197 artifact.",
        "schema=2",
        "R_B_C=" + ",".join(fmt9(x) for x in r),
        "cam_offset_B=" + ",".join(fmt9(x) for x in DEFAULT_CAM_OFFSET_B),
        "kp_x=-1.000000",
        "kp_y=-1.000000",
        "kp_yaw=-1.000000",
        f"source_json={src}",
        "source_experiment=s197_sota_calib_orientation_guarded",
        "source_iter=iter27_camera_landing_contract",
        f"source_status={data.get('status', '')}",
        f"layout_id={data.get('layout_id', '')}",
        f"camera_referenced_landing_ok={validation.get('camera_referenced_landing_ok')}",
        f"marker_count={marker.get('count', '')}",
        f"marker_diameter_m={marker.get('diameter_m', '')}",
        f"fx={image.get('fx', '')}",
        f"fy={image.get('fy', '')}",
        f"cx={image.get('cx', '')}",
        f"cy={image.get('cy', '')}",
        f"disarm_full_markers={safety.get('disarm_full_markers', '')}",
        "extpos_sign_x=-1.000000",
        "extpos_sign_y=-1.000000",
        "extpos_sign_z=1.000000",
        f"axis_roll_image_axis={roll.get('dominant_image_axis', '')}",
        f"axis_roll_sign={roll.get('dominant_sign', '')}",
        f"axis_roll_strength_px={roll.get('response_strength_px', '')}",
        "axis_roll_vec_px=" + ",".join(
            fmt6(x) for x in (roll.get("complementary_delta_px") or (0.0, 0.0))),
        f"axis_pitch_image_axis={pitch.get('dominant_image_axis', '')}",
        f"axis_pitch_sign={pitch.get('dominant_sign', '')}",
        f"axis_pitch_strength_px={pitch.get('response_strength_px', '')}",
        "axis_pitch_vec_px=" + ",".join(
            fmt6(x) for x in (pitch.get("complementary_delta_px") or (0.0, 0.0))),
        f"axis_pulse_deg={fmt6(axis.get('pulse_deg', 0.0) or 0.0)}",
        "",
    ]
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {dst} ({dst.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

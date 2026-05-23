#!/usr/bin/env python3
"""verdict_s193 — calib bringup acceptance gate (OP-S10-W21-T4 phase-3).

Same thresholds as s187 verdict, but accepts the iter directory as a
single positional argument (matching the s192/s193 run.sh convention)
and pulls land-error from the GT recorder JSONL.

Pass criteria (from OP-S10-W21_calib_unified_bringup.md):
    R_cam_to_body drift from SDF GT  < 1.0 deg
    cam_offset_B vs SDF GT (inf)     < 5 mm
    kp_x, kp_y                       in [0.30, 0.50]
    hold rms drift                   < 30 mm
    hold max drift                   < 80 mm
"""

import json
import math
import sys
from pathlib import Path


SDF_R_GT = [
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
]
SDF_OFFSET_GT = (-0.04, 0.0, -0.02)

GATE_R_DEG     = 1.0
GATE_OFF_M     = 0.005
GATE_KP_LO     = 0.30
GATE_KP_HI     = 0.50
GATE_HOLD_RMS  = 0.030
GATE_HOLD_MAX  = 0.080


def _mat_from_flat(flat9):
    return [list(flat9[3*i:3*i+3]) for i in range(3)]


def rotation_angle_deg(R1, R2):
    tr = 0.0
    for i in range(3):
        s = 0.0
        for k in range(3):
            s += R1[k][i] * R2[k][i]
        tr += s
    cos_th = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
    return math.degrees(math.acos(cos_th))


def check(label, value, predicate, threshold_str):
    ok = predicate(value)
    print("  [{0}] {1:34s} = {2:>10}   gate: {3}".format(
        "PASS" if ok else "FAIL", label,
        "{:.4f}".format(value) if isinstance(value, float) else str(value),
        threshold_str))
    return ok


def main():
    if len(sys.argv) < 2:
        print("usage: verdict_s193.py <iter_dir>", file=sys.stderr)
        sys.exit(2)
    iter_dir = Path(sys.argv[1])
    sp = iter_dir / "mission_s193_summary.json"
    gt_p = iter_dir / "fr_current" / "gt.jsonl"

    if not sp.exists():
        print(f"[verdict_s193] FATAL: {sp} not found", file=sys.stderr)
        sys.exit(2)
    summary = json.loads(sp.read_text())

    print("=" * 72)
    print(f"s193 calib bringup verdict — {iter_dir.name}")
    print("=" * 72)
    print(f"  mission status     : {summary.get('status')}")
    print(f"  phase_reached      : {summary.get('phase_reached')}")
    print(f"  reject             : {summary.get('reject_code')} {summary.get('reject_name','')}")
    print(f"  n_samples          : {summary.get('n_samples')}")
    print(f"  duration (ms)      : {summary.get('duration_ms')}")
    print(f"  assert_calibrated  : {summary.get('assert_calibrated')}")
    print()
    print("Acceptance checks:")

    all_pass = True
    drift_deg = None
    inf_norm = None

    R = summary.get("R")
    if R and len(R) == 9:
        R_mat = _mat_from_flat(R)
        drift_deg = rotation_angle_deg(R_mat, SDF_R_GT)
        all_pass &= check("R_cam_to_body drift (deg)", drift_deg,
                            lambda v: v < GATE_R_DEG,
                            f"< {GATE_R_DEG} deg")
    else:
        print("  [FAIL] R_cam_to_body missing")
        all_pass = False

    off = summary.get("cam_offset_B")
    if off and len(off) == 3:
        inf_norm = max(abs(off[i] - SDF_OFFSET_GT[i]) for i in range(3))
        all_pass &= check("cam_offset inf-norm err (m)", inf_norm,
                            lambda v: v < GATE_OFF_M,
                            f"< {GATE_OFF_M} m")
    else:
        print("  [FAIL] cam_offset_B missing")
        all_pass = False

    kp_x = summary.get("kp_x", 0.0)
    kp_y = summary.get("kp_y", 0.0)
    all_pass &= check("kp_x", kp_x,
                        lambda v: GATE_KP_LO <= v <= GATE_KP_HI,
                        f"in [{GATE_KP_LO}, {GATE_KP_HI}]")
    all_pass &= check("kp_y", kp_y,
                        lambda v: GATE_KP_LO <= v <= GATE_KP_HI,
                        f"in [{GATE_KP_LO}, {GATE_KP_HI}]")

    all_pass &= check("hold rms (m)", summary.get("hold_rms", 0.0),
                        lambda v: v < GATE_HOLD_RMS,
                        f"< {GATE_HOLD_RMS} m")
    all_pass &= check("hold max (m)", summary.get("hold_max", 0.0),
                        lambda v: v < GATE_HOLD_MAX,
                        f"< {GATE_HOLD_MAX} m")

    cal_expected = bool(summary.get("accepted"))
    cal_got      = bool(summary.get("assert_calibrated"))
    all_pass &= check("assert_calibrated matches accepted",
                        cal_got == cal_expected,
                        lambda v: v, "True")

    # Land error from GT.
    land_err = None
    if gt_p.exists():
        rows = [json.loads(l) for l in gt_p.read_text().splitlines() if l.strip()]
        if rows:
            f, lr = rows[0], rows[-1]
            land_err = math.sqrt((lr["x"] - f["x"]) ** 2 + (lr["y"] - f["y"]) ** 2)
            print(f"  [INFO] land XY err          = {land_err*100:.1f} cm  (soft)")

    print()
    print("=" * 72)
    print(f"VERDICT: {'PASS' if all_pass else 'FAIL'}")
    print("=" * 72)

    verdict_obj = {
        "status":              summary.get("status"),
        "phase_reached":       summary.get("phase_reached"),
        "reject_name":         summary.get("reject_name"),
        "R_drift_deg":         drift_deg,
        "cam_offset_inf_err_m": inf_norm,
        "kp_x":                kp_x,
        "kp_y":                kp_y,
        "hold_rms":            summary.get("hold_rms"),
        "hold_max":            summary.get("hold_max"),
        "n_samples":           summary.get("n_samples"),
        "land_err_m":          land_err,
        "pass":                all_pass,
    }
    (iter_dir / "verdict_s193.json").write_text(json.dumps(verdict_obj, indent=2))
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()

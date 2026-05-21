#!/usr/bin/env python3
"""verdict_s187 — post-mortem acceptance gate for OP-S10-W21-T4 phase-2.

Compares the recovered calibration (from sentai_sim's bringup run) to
the SDF ground truth and prints PASS/FAIL per the design-doc thresholds:

    | R_cam_to_body drift from SDF | < 1.0°            |
    | cam_offset_B vs SDF (inf-norm) | < 5 mm          |
    | kp_x, kp_y                    | ∈ [0.30, 0.50]   |
    | Hold-validation rms drift     | < 30 mm          |
    | Hold-validation max drift     | < 80 mm          |

SDF ground truth values are mirrored from sentai_calib.cc:
    SENTAI_CALIB_DEFAULT_R_SIM           = [[0,1,0],[1,0,0],[0,0,-1]]
    SENTAI_CALIB_DEFAULT_CAM_OFFSET_SIM  = (-0.04, 0.0, -0.02)

Host-only.  Reads summary.json + journal.txt produced by the mission.
"""

import argparse
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
    """[r00 r01 r02 r10 r11 r12 r20 r21 r22] -> [[...],[...],[...]]."""
    return [list(flat9[3*i:3*i+3]) for i in range(3)]


def rotation_angle_deg(R1, R2):
    """Geodesic angle on SO(3) between two rotations (deg).  Matches
    the C-side sentai_calib_rotation_angle_deg."""
    # tr(R1^T R2)
    t = 0.0
    for i in range(3):
        for j in range(3):
            for k in range(3):
                t += R1[k][i] * R2[k][j] * (1.0 if i == j else 0.0)
    # Simpler — accumulate trace of R1.T @ R2.
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
    ap = argparse.ArgumentParser()
    ap.add_argument("--summary", required=True,
                     help="mission_s187_summary.json from sentai_sim")
    ap.add_argument("--journal", required=False,
                     help="mission_s187_journal.txt (for context only)")
    ap.add_argument("--out-dir", required=False, default=".",
                     help="where to write verdict.json + .txt")
    args = ap.parse_args()

    sp = Path(args.summary)
    if not sp.exists():
        print("[verdict_s187] FATAL: summary file not found:", sp,
              file=sys.stderr)
        return 2
    summary = json.loads(sp.read_text())

    print("=" * 72)
    print("s187 acceptance gate — OP-S10-W21-T4 phase-2")
    print("=" * 72)
    print("  mission status     :", summary.get("status"))
    print("  phase_reached      :", summary.get("phase_reached"))
    print("  reject             :", summary.get("reject_code"),
          summary.get("reject_name"))
    print("  n_samples          :", summary.get("n_samples"))
    print("  duration (ms)      :", summary.get("duration_ms"))
    print("  assert_calibrated  :", summary.get("assert_calibrated"))
    print()
    print("Acceptance checks:")

    all_pass = True

    # R drift.
    R = summary.get("R")
    if R and len(R) == 9:
        R_mat = _mat_from_flat(R)
        drift_deg = rotation_angle_deg(R_mat, SDF_R_GT)
        all_pass &= check("R_cam_to_body drift (deg)", drift_deg,
                            lambda v: v < GATE_R_DEG,
                            "< {} deg".format(GATE_R_DEG))
    else:
        print("  [FAIL] R_cam_to_body missing")
        all_pass = False
        drift_deg = None

    # cam_offset_B inf-norm.
    off = summary.get("cam_offset_B")
    if off and len(off) == 3:
        inf_norm = max(abs(off[i] - SDF_OFFSET_GT[i]) for i in range(3))
        all_pass &= check("cam_offset inf-norm err (m)", inf_norm,
                            lambda v: v < GATE_OFF_M,
                            "< {} m".format(GATE_OFF_M))
    else:
        print("  [FAIL] cam_offset_B missing")
        all_pass = False
        inf_norm = None

    # Kp gains.
    kp_x = summary.get("kp_x", 0.0)
    kp_y = summary.get("kp_y", 0.0)
    all_pass &= check("kp_x", kp_x,
                        lambda v: GATE_KP_LO <= v <= GATE_KP_HI,
                        "in [{}, {}]".format(GATE_KP_LO, GATE_KP_HI))
    all_pass &= check("kp_y", kp_y,
                        lambda v: GATE_KP_LO <= v <= GATE_KP_HI,
                        "in [{}, {}]".format(GATE_KP_LO, GATE_KP_HI))

    # Hold metrics.
    all_pass &= check("hold rms (m)", summary.get("hold_rms", 0.0),
                        lambda v: v < GATE_HOLD_RMS,
                        "< {} m".format(GATE_HOLD_RMS))
    all_pass &= check("hold max (m)", summary.get("hold_max", 0.0),
                        lambda v: v < GATE_HOLD_MAX,
                        "< {} m".format(GATE_HOLD_MAX))

    # assert_calibrated guard must pass IFF bringup accepted.
    cal_expected = bool(summary.get("accepted"))
    cal_got      = bool(summary.get("assert_calibrated"))
    all_pass &= check("assert_calibrated matches accepted",
                        cal_got == cal_expected,
                        lambda v: v, "True")

    print()
    print("=" * 72)
    print("OVERALL:", "PASS" if all_pass else "FAIL")
    print("=" * 72)

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    verdict = {
        "overall":              "PASS" if all_pass else "FAIL",
        "R_drift_deg":          drift_deg,
        "cam_offset_inf_err_m": inf_norm,
        "kp_x":                 kp_x,
        "kp_y":                 kp_y,
        "hold_rms":             summary.get("hold_rms"),
        "hold_max":             summary.get("hold_max"),
        "summary":              summary,
    }
    (out / "verdict_s187.json").write_text(json.dumps(verdict, indent=2))
    (out / "verdict_s187.txt").write_text(
        "OP-S10-W21-T4 s187 phase-2 verdict: {}\n".format(verdict["overall"]))
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())

"""s158 verdict — OP-S6-W1-T7 PASS gate.

PASS criteria (combine s153 closure + calibration acceptance):
  * Mission status == "OK"
  * calib.accepted is True (uses the firmware's QUALITY_RES_DEG = 5° gate;
    the gap from the s131 host-Python reference (~1°) is documented in
    s159 README — DLT PnP + heuristic quad corner extraction account for
    the extra noise.  Upgrades to IPPE + Douglas-Peucker would tighten
    this back down).
  * calib.mean_residual_deg < 5.0
  * calib.n_samples >= 12 (8 frames * 4 markers minus a few drops)
  * cam_calib.json exists in fs_root
  * closure_xy < 12 cm (slightly relaxed vs s153's 10 cm because of
    the extra calibration leg)
  * R_new differs from default by <= 10° (sanity gate — SIM camera is
    near-identity, so any wild deviation indicates a bug).
"""
from __future__ import annotations

import ast
import json
import math
import sys
from pathlib import Path

DEFAULT_R = (0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0)


def geodesic_deg(R1, R2):
    # M = R1^T R2; angle = acos((tr(M) - 1)/2).  Hand-rolled.
    M = [0.0] * 9
    for i in range(3):
        for j in range(3):
            s = 0.0
            for k in range(3):
                s += R1[k * 3 + i] * R2[k * 3 + j]
            M[i * 3 + j] = s
    tr = M[0] + M[4] + M[8]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    return math.degrees(math.acos(c))


def main(summary_path: str) -> int:
    p = Path(summary_path)
    if not p.exists():
        print(f"FAIL — summary not found: {summary_path}")
        return 1
    raw = p.read_text()
    try:
        summary = json.loads(raw)
    except json.JSONDecodeError:
        try:
            summary = ast.literal_eval(raw)
        except (ValueError, SyntaxError) as e:
            print(f"FAIL — summary unparseable: {e}\n--- raw ---\n{raw[:400]}")
            return 1

    print("=" * 60)
    print("s158 verdict — OP-S6-W1-T7 sentai.calib + sentai.aruco in flight")
    print("=" * 60)
    print(f"  status            : {summary.get('status')}")
    print(f"  origin            : {summary.get('origin_xyz')}")
    print(f"  closure_xy_m      : {summary.get('closure_xy')}")
    print(f"  total_path_m      : {summary.get('total_path_m')}")
    print(f"  errors            : {summary.get('errors')}")
    calib = summary.get("calib") or {}
    print(f"  calib accepted    : {calib.get('accepted')}")
    print(f"  calib n_samples   : {calib.get('n_samples')}")
    print(f"  calib mean_res    : {calib.get('mean_residual_deg')}")
    print(f"  calib drift_deg   : {calib.get('drift_from_persisted')}")
    print(f"  calib committed   : {calib.get('committed')}")
    print(f"  calib saved       : {calib.get('saved')}")
    R_new = calib.get("R_new")
    if isinstance(R_new, (list, tuple)) and len(R_new) == 9:
        try:
            dev = geodesic_deg(tuple(float(x) for x in R_new), DEFAULT_R)
            print(f"  R_new vs default  : {dev:.3f}°")
        except Exception as e:
            print(f"  R_new vs default  : <err {e}>")
            dev = 999.0
    else:
        dev = None
        print(f"  R_new vs default  : <missing>")

    fs_root = p.parent
    cam_calib = fs_root / "cam_calib.json"
    has_calib = cam_calib.exists()
    print(f"  cam_calib.json    : {'present' if has_calib else 'MISSING'}")

    # PASS gate.
    ok = summary.get("status") == "OK"
    ok = ok and (calib.get("accepted") is True)
    ok = ok and (calib.get("mean_residual_deg") is not None
                 and calib.get("mean_residual_deg") < 5.0)
    ok = ok and (calib.get("n_samples") is not None
                 and calib.get("n_samples") >= 12)
    closure = summary.get("closure_xy")
    ok = ok and isinstance(closure, (int, float)) and closure < 0.12
    ok = ok and has_calib
    if dev is not None:
        ok = ok and (dev < 10.0)

    print()
    print(f"VERDICT: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: verdict.py <mission_s158_summary.json>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))

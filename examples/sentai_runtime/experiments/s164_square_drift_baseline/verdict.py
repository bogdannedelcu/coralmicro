"""s164 verdict — 1m square drift baseline (OP-S10-W11-T5.A)."""
from __future__ import annotations

import json
import sys
from pathlib import Path

SUMMARY_JSON = Path("/tmp/s164_square_drift_baseline/summary.json")

# Hard gate per [[sim-test-must-return-home]] universal rule.  Single-lap
# canonical limit is 10 cm; the 1m square is ~4 m of travel + 4 turns,
# we relax to 15 cm (matches s142 HexPatrol gate).
CLOSURE_GATE_CM = 15.0


def main() -> int:
    if not SUMMARY_JSON.is_file():
        print(f"[verdict] FAIL — summary.json missing: {SUMMARY_JSON}")
        return 1
    s = json.loads(SUMMARY_JSON.read_text())
    if s.get("_status") != "OK":
        print(f"[verdict] FAIL — mission status={s.get('_status')}; "
              f"exception={s.get('_exception')}")
        return 1

    corners = s.get("corners_log", [])
    if len(corners) != 4:
        print(f"[verdict] FAIL — expected 4 corners visited, got {len(corners)}")
        return 1

    # Informational metrics — drift envelope characterisation.
    print("[verdict] s164 1m square drift baseline (T5.A)")
    print(f"          physical_origin = "
          f"({s['physical_origin'][0]:+.3f},{s['physical_origin'][1]:+.3f})")
    print(f"          land_pose       = "
          f"({s['land_pose'][0]:+.3f},{s['land_pose'][1]:+.3f},"
          f"{s['land_pose'][2]:.3f})")
    print(f"          mission_dur     = {s['mission_duration_s']:.1f} s")
    print("          per-corner:")
    for c in corners:
        ar = c.get("aruco", {})
        print(f"            {c['label']:8s} cmd=({c['cmd'][0]:+.2f},"
              f"{c['cmd'][1]:+.2f})  arrived={c['arrived']}  "
              f"goto={c['goto_elapsed_s']:.2f}s  "
              f"err_xy={c['cmd_err_xy_cm']:.2f}cm  "
              f"aruco n={ar.get('n_markers',0)} ids={ar.get('ids',[])}")

    closure_cm = s["land_err_vs_origin_cm"]
    print(f"          closure         = {closure_cm:.2f} cm "
          f"(gate < {CLOSURE_GATE_CM:.0f} cm)")

    # Drift envelope summary — useful for T5.B planning.
    cmd_errs = [c["cmd_err_xy_cm"] for c in corners]
    cmd_err_mean = sum(cmd_errs) / len(cmd_errs)
    cmd_err_max  = max(cmd_errs)
    print(f"          per-corner cmd_err: mean={cmd_err_mean:.2f}cm  "
          f"max={cmd_err_max:.2f}cm")

    aruco_counts = [c["aruco"].get("n_markers", 0) for c in corners]
    aruco_zero_corners = sum(1 for n in aruco_counts if n == 0)
    print(f"          aruco visibility: counts={aruco_counts}  "
          f"corners_blind={aruco_zero_corners}/4")

    # Hard gate.
    if closure_cm > CLOSURE_GATE_CM:
        print(f"[verdict] FAIL — closure {closure_cm:.2f}cm > "
              f"{CLOSURE_GATE_CM:.0f}cm gate")
        return 1

    # All corners should have at least attempted to arrive — failure
    # here means the controller got stuck or timed out.
    not_arrived = [c["label"] for c in corners if not c["arrived"]]
    if not_arrived:
        print(f"[verdict] FAIL — corners did not reach goto tolerance: "
              f"{not_arrived}")
        return 1

    print(f"[verdict] PASS — drift baseline established "
          f"(closure={closure_cm:.2f}cm, "
          f"cmd_err mean={cmd_err_mean:.2f}cm)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

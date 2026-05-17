#!/usr/bin/env python3
"""verdict.py — gates mission_s153 (long-distance exploration on servo paradigm)."""
import json
import sys
from pathlib import Path

SUMMARY_NAME = "mission_s153_summary.json"
JOURNAL_NAME = "mission_s153_journal.txt"

EXPECTED_PHASES = ["init", "origin", "takeoff", "waypoints",
                    "return_home", "land", "disarm"]
EXPECTED_EVENTS = ["mission_start", "crazy_init", "servo_init", "origin",
                    "takeoff", "goto_wp0", "converge_wp0",
                    "inspect_dwell_start", "inspect_dwell_end",
                    "goto_wp1", "converge_wp1", "goto_home", "land", "closure"]

GATE_CLOSURE_M = 0.10
# Path budget: theoretical (0,0)→(0.40,0)→(0.40,0.40)→home = 1.37 m,
# but cf2 HL Commander undershoots ~10-15 cm per leg even with 8 s
# WAYPOINT_DUR.  Empirical: 0.94-1.04 m measured.  Gate at 0.85 m still
# proves multi-leg traversal happened (single-leg would be < 0.5 m).
GATE_TOTAL_PATH_MIN_M = 0.85


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parents[4] / "build-sim" / "sentai_fs_root"
    sp = root / SUMMARY_NAME
    jp = root / JOURNAL_NAME

    if not sp.exists():
        print(f"FAIL missing summary at {sp}"); return 1
    s = json.loads(sp.read_text())
    j = jp.read_text() if jp.exists() else ""

    fails = []
    # G0 — reproducibility: cf2 must have been respawned at world origin
    # per [[experiments-start-from-origin]].  Mission already aborts on
    # FAIL_ORIGIN_BIAS; this gate double-checks the captured origin.
    orig = s.get("origin_xyz")
    if not orig or (orig[0]**2 + orig[1]**2) ** 0.5 > 0.05:
        fails.append(f"G0 origin_xy not at world (0,0): {orig}")
    if s.get("status") != "DONE":
        fails.append(f"G1 status={s.get('status')}")
    mp_ = [p for p in EXPECTED_PHASES if p not in s.get("phases_done", [])]
    if mp_: fails.append(f"G2 missing phases {mp_}")
    if len(s.get("waypoints", [])) < 2: fails.append("G3 waypoints<2")
    tp = s.get("total_path_m", 0)
    if tp < GATE_TOTAL_PATH_MIN_M:
        fails.append(f"G4 total_path={tp:.3f} < {GATE_TOTAL_PATH_MIN_M}")
    cx = s.get("closure_xy")
    if cx is None or cx > GATE_CLOSURE_M:
        fails.append(f"G5 closure_xy={cx}, gate < {GATE_CLOSURE_M}")
    if s.get("errors"): fails.append(f"G6 errors {s['errors']}")
    # servo-paradigm specific gates (mirror s152)
    st = s.get("servo_status_final") or {}
    if st.get("actions_ok", 0) < 7:
        fails.append(f"G7 actions_ok={st.get('actions_ok')} < 7")
    if st.get("faults_oob", 0) != 0:
        fails.append(f"G8 faults_oob={st.get('faults_oob')} != 0")
    me = [e for e in EXPECTED_EVENTS if e not in j]
    if me: fails.append(f"G9 missing events {me}")

    print(f"[s153 verdict] {len(fails)} FAIL")
    for f in fails: print("  FAIL", f)
    if not fails:
        print("[s153 verdict] PASS — long-distance exploration on sentai.servo paradigm")
        print(f"  closure_xy        = {cx*100:.2f} cm  (gate < {GATE_CLOSURE_M*100:.0f})")
        print(f"  total_path        = {tp*100:.1f} cm")
        print(f"  waypoints         = {len(s['waypoints'])}")
        print(f"  servo.actions_ok  = {st.get('actions_ok')}")
        print(f"  servo.faults_oob  = {st.get('faults_oob')}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""verdict.py — gates mission_s156 (real-frame loop closure on servo paradigm).

Gate G6 (matches_ok) is RELAXED from 100% to ≥1/3 because real Gazebo
camera frames have natural variation across laps (lighting, pose drift,
texture aliasing).  s144 README documents this empirically — synthetic
descriptors round-trip 3/3 but real frames typically 1-2/3."""
import json, sys
from pathlib import Path

SUMMARY_NAME = "mission_s156_summary.json"
JOURNAL_NAME = "mission_s156_journal.txt"

EXPECTED_PHASES = ["init", "camera_probe", "origin", "takeoff",
                    "lap1", "lap2", "return", "land", "disarm"]
EXPECTED_EVENTS = ["mission_start", "crazy_init", "servo_init",
                    "camera_probe_ok", "lap1_start", "lap1_real_store",
                    "lap2_start", "lap2_real_query", "goto_home", "closure"]
GATE_CLOSURE_M = 0.12
GATE_MATCHES_MIN = 1


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parents[4] / "build-sim" / "sentai_fs_root"
    sp, jp = root / SUMMARY_NAME, root / JOURNAL_NAME
    if not sp.exists():
        print(f"FAIL missing summary at {sp}"); return 1
    s = json.loads(sp.read_text()); j = jp.read_text() if jp.exists() else ""

    fails = []
    orig = s.get("origin_xyz")
    if not orig or (orig[0]**2 + orig[1]**2) ** 0.5 > 0.05:
        fails.append(f"G0 origin_xy not at world (0,0): {orig}")
    if s.get("status") != "DONE":
        fails.append(f"G1 status={s.get('status')}")
        if s.get("status") == "FAIL_NO_CAMERA":
            fails.append("  HINT: launch gz_to_uds_bridge before mission")
    mp_ = [p for p in EXPECTED_PHASES if p not in s.get("phases_done", [])]
    if mp_: fails.append(f"G2 missing phases {mp_}")
    if len(s.get("lap1_real_stores", [])) < 3:
        fails.append(f"G3 lap1 stored {len(s.get('lap1_real_stores', []))}")
    bad_pids = [r for r in s.get("lap1_real_stores", []) if r.get("pid", -1) < 0]
    if bad_pids: fails.append(f"G4 lap1 store failed for {bad_pids}")
    if len(s.get("lap2_real_matches", [])) < 3:
        fails.append(f"G5 lap2 matched {len(s.get('lap2_real_matches', []))}")
    ok = s.get("matches_ok", 0)
    if ok < GATE_MATCHES_MIN:
        fails.append(f"G6 matches_ok={ok} < {GATE_MATCHES_MIN}")
    cx = s.get("closure_xy")
    if cx is None or cx > GATE_CLOSURE_M:
        fails.append(f"G7 closure_xy={cx} > {GATE_CLOSURE_M}")
    if s.get("errors"): fails.append(f"G8 errors {s['errors']}")
    # servo-paradigm specific
    st = s.get("servo_status_final") or {}
    if st.get("actions_ok", 0) < 7:
        fails.append(f"G9 actions_ok={st.get('actions_ok')} < 7")
    if st.get("faults_oob", 0) != 0:
        fails.append(f"G10 faults_oob={st.get('faults_oob')} != 0")
    me = [e for e in EXPECTED_EVENTS if e not in j]
    if me: fails.append(f"G11 missing events {me}")

    print(f"[s156 verdict] {len(fails)} FAIL")
    for f in fails: print("  FAIL", f)
    if not fails:
        print("[s156 verdict] PASS — real-frame loop closure on sentai.servo paradigm")
        print(f"  closure_xy        = {cx*100:.2f} cm")
        print(f"  matches           = {ok}/3 round-trip (real frames)")
        print(f"  servo.actions_ok  = {st.get('actions_ok')}")
        print(f"  servo.faults_oob  = {st.get('faults_oob')}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

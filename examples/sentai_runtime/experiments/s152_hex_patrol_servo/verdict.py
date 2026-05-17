#!/usr/bin/env python3
"""verdict.py — gates mission_s152 (HexPatrol on sentai.servo paradigm)."""
import json, sys
from pathlib import Path

SUMMARY_NAME = "mission_s152_summary.json"
JOURNAL_NAME = "mission_s152_journal.txt"

EXPECTED_PHASES = ["init", "origin", "takeoff", "patrol",
                    "self_queries", "return", "land", "disarm"]
EXPECTED_EVENTS = ["mission_start", "crazy_init", "servo_init",
                    "origin", "places_cleared",
                    "goto_wp", "place_add", "self_query",
                    "goto_home", "closure"]
GATE_CLOSURE_M = 0.10


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parents[4] / "build-sim" / "sentai_fs_root"
    sp, jp = root / SUMMARY_NAME, root / JOURNAL_NAME
    if not sp.exists():
        print(f"FAIL missing summary at {sp}"); return 1
    s = json.loads(sp.read_text())
    j = jp.read_text() if jp.exists() else ""

    fails = []
    if s.get("status") != "DONE":
        fails.append(f"G1 status={s.get('status')}")
    mp_ = [p for p in EXPECTED_PHASES if p not in s.get("phases_done", [])]
    if mp_: fails.append(f"G2 missing phases {mp_}")
    if len(s.get("places_stored", [])) != 3:
        fails.append(f"G3 places_stored={len(s.get('places_stored', []))}")
    bad_pids = [r for r in s.get("places_stored", []) if r.get("pid", -1) < 0]
    if bad_pids: fails.append(f"G4 store failed for {bad_pids}")
    sq = s.get("self_queries", [])
    if len(sq) != 3 or not all(r.get("ok") for r in sq):
        fails.append(f"G5 self_queries={sq}")
    cx = s.get("closure_xy")
    if cx is None or cx > GATE_CLOSURE_M:
        fails.append(f"G6 closure_xy={cx} > {GATE_CLOSURE_M}")
    # New gates specific to servo paradigm.
    st = s.get("servo_status_final") or {}
    if st.get("actions_ok", 0) < 8:
        fails.append(f"G7 actions_ok={st.get('actions_ok')} < 8")
    if st.get("faults_oob", 0) != 0:
        fails.append(f"G8 faults_oob={st.get('faults_oob')} != 0")
    if s.get("errors"): fails.append(f"G9 errors {s['errors']}")
    me = [e for e in EXPECTED_EVENTS if e not in j]
    if me: fails.append(f"G10 missing events {me}")

    print(f"[s152 verdict] {len(fails)} FAIL")
    for f in fails: print("  FAIL", f)
    if not fails:
        print("[s152 verdict] PASS — HexPatrol on sentai.servo paradigm")
        print(f"  closure_xy        = {cx*100:.2f} cm")
        print(f"  servo.actions_ok  = {st.get('actions_ok')}")
        print(f"  servo.faults_oob  = {st.get('faults_oob')}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

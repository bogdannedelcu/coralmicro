#!/usr/bin/env python3
"""verdict.py — gates mission_s150 (loop closure MP-only)."""
import json, sys
from pathlib import Path

SUMMARY_NAME = "mission_s150_summary.json"
JOURNAL_NAME = "mission_s150_journal.txt"

EXPECTED_PHASES = ["init", "crtp_log_setup", "origin", "takeoff",
                    "lap1", "lap2", "return", "land", "disarm"]
EXPECTED_EVENTS = ["mission_start", "lap1_start", "lap1_store",
                    "lap2_start", "lap2_query", "goto_home", "closure"]
GATE_CLOSURE_M = 0.12  # relaxed for 2-lap (drift accumulates 2x)
GATE_MATCHES_MIN = 2     # at least 2 of 3 waypoints must round-trip


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parents[4] / "build-sim" / "sentai_fs_root"
    sp, jp = root / SUMMARY_NAME, root / JOURNAL_NAME
    if not sp.exists():
        print(f"FAIL missing summary at {sp}"); return 1
    s = json.loads(sp.read_text()); j = jp.read_text() if jp.exists() else ""

    fails = []
    if s.get("status") != "DONE": fails.append(f"G1 status={s.get('status')}")
    mp_ = [p for p in EXPECTED_PHASES if p not in s.get("phases_done", [])]
    if mp_: fails.append(f"G2 missing phases {mp_}")
    if len(s.get("lap1_stores", [])) < 3:
        fails.append(f"G3 lap1 stored {len(s.get('lap1_stores', []))}")
    if len(s.get("lap2_matches", [])) < 3:
        fails.append(f"G4 lap2 matched {len(s.get('lap2_matches', []))}")
    ok = s.get("matches_ok", 0)
    if ok < GATE_MATCHES_MIN:
        fails.append(f"G5 matches_ok={ok} < {GATE_MATCHES_MIN}")
    cx = s.get("closure_xy")
    if cx is None or cx > GATE_CLOSURE_M:
        fails.append(f"G6 closure_xy={cx} > {GATE_CLOSURE_M}")
    if s.get("errors"): fails.append(f"G7 errors {s['errors']}")
    me = [e for e in EXPECTED_EVENTS if e not in j]
    if me: fails.append(f"G8 missing events {me}")

    print(f"[s150 verdict] {len(fails)} FAIL")
    for f in fails: print("  FAIL", f)
    if not fails:
        print("[s150 verdict] PASS — loop closure (consume L3 memory)")
        print(f"  closure_xy = {cx*100:.2f} cm")
        print(f"  matches    = {ok}/3 round-trip")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

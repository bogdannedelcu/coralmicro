#!/usr/bin/env python3
"""verdict.py — gates mission_s149 (HexPatrol MP-only)."""
import json, sys
from pathlib import Path

SUMMARY_NAME = "mission_s149_summary.json"
JOURNAL_NAME = "mission_s149_journal.txt"

EXPECTED_PHASES = ["init", "crtp_log_setup", "origin", "takeoff",
                    "patrol", "self_queries", "return", "land", "disarm"]
EXPECTED_EVENTS = ["mission_start", "crazy_init", "places_cleared",
                    "origin", "goto_wp", "place_add", "self_query",
                    "goto_home", "closure"]
GATE_CLOSURE_M = 0.10
GATE_MIN_PLACES = 3


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
    places = s.get("places_stored", [])
    if len(places) < GATE_MIN_PLACES:
        fails.append(f"G3 stored {len(places)} < {GATE_MIN_PLACES}")
    bad_pids = [p for p in places if p.get("pid", -1) < 0]
    if bad_pids: fails.append(f"G4 places.add failed for {bad_pids}")
    queries = s.get("self_queries", [])
    if len(queries) < GATE_MIN_PLACES:
        fails.append(f"G5 self_queries {len(queries)} < {GATE_MIN_PLACES}")
    bad_q = [q for q in queries if not q.get("ok")]
    if bad_q: fails.append(f"G6 self_query failures {bad_q}")
    cx = s.get("closure_xy")
    if cx is None or cx > GATE_CLOSURE_M:
        fails.append(f"G7 closure_xy={cx} > {GATE_CLOSURE_M}")
    if s.get("errors"): fails.append(f"G8 errors {s['errors']}")
    me = [e for e in EXPECTED_EVENTS if e not in j]
    if me: fails.append(f"G9 missing events {me}")

    print(f"[s149 verdict] {len(fails)} FAIL")
    for f in fails: print("  FAIL", f)
    if not fails:
        print("[s149 verdict] PASS — 3 places stored + self-query + closure gate")
        print(f"  closure_xy   = {cx*100:.2f} cm")
        print(f"  places       = {len(places)}")
        print(f"  self queries = {len(queries)} all OK")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

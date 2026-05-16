#!/usr/bin/env python3
"""verdict.py — gates mission_s147 (long-distance exploration MP-only)."""
import json
import sys
from pathlib import Path

SUMMARY_NAME = "mission_s147_summary.json"
JOURNAL_NAME = "mission_s147_journal.txt"

EXPECTED_PHASES = ["init", "crtp_log_setup", "origin", "takeoff",
                    "waypoints", "return_home", "land", "disarm"]
EXPECTED_EVENTS = ["mission_start", "crazy_init", "toc_scan",
                    "pose_subscribe", "origin", "takeoff", "goto_wp0",
                    "converge_wp0", "inspect_dwell_start", "inspect_dwell_end",
                    "goto_wp1", "converge_wp1", "goto_home", "land", "closure"]

GATE_CLOSURE_M = 0.10
GATE_TOTAL_PATH_MIN_M = 1.0   # must traverse at least 1 m total


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
    if s.get("status") != "DONE":
        fails.append(f"G1 status={s.get('status')}")
    missing_phases = [p for p in EXPECTED_PHASES if p not in s.get("phases_done", [])]
    if missing_phases: fails.append(f"G2 missing phases {missing_phases}")
    if len(s.get("waypoints", [])) < 2: fails.append("G3 waypoints<2")
    tp = s.get("total_path_m", 0)
    if tp < GATE_TOTAL_PATH_MIN_M:
        fails.append(f"G4 total_path={tp:.3f} < {GATE_TOTAL_PATH_MIN_M}")
    cx = s.get("closure_xy")
    if cx is None or cx > GATE_CLOSURE_M:
        fails.append(f"G5 closure_xy={cx}, gate < {GATE_CLOSURE_M}")
    if s.get("errors"): fails.append(f"G6 errors {s['errors']}")
    missing_events = [e for e in EXPECTED_EVENTS if e not in j]
    if missing_events: fails.append(f"G7 missing events {missing_events}")

    print(f"[s147 verdict] {len(fails)} FAIL")
    for f in fails: print("  FAIL", f)
    if not fails:
        print("[s147 verdict] PASS")
        print(f"  closure_xy   = {cx*100:.2f} cm  (gate < {GATE_CLOSURE_M*100:.0f})")
        print(f"  total_path   = {tp*100:.1f} cm")
        print(f"  waypoints    = {len(s['waypoints'])}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

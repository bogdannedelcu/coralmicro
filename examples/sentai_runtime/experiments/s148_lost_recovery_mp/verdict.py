#!/usr/bin/env python3
"""verdict.py — gates mission_s148 (LOST recovery MP-only)."""
import json, sys
from pathlib import Path

SUMMARY_NAME = "mission_s148_summary.json"
JOURNAL_NAME = "mission_s148_journal.txt"

EXPECTED_PHASES = ["init", "crtp_log_setup", "origin", "takeoff",
                    "approach", "lost_recovery", "return", "land", "disarm"]
EXPECTED_EVENTS = ["mission_start", "crazy_init", "toc_scan", "origin",
                    "goto_target", "lost_simulated_start", "lost_apex",
                    "lost_recovered", "goto_home", "closure"]
GATE_CLOSURE_M = 0.10
GATE_LOST_DELTA_MIN = 0.15   # apex Z should be > TAKEOFF_HEIGHT by ~30cm


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        Path(__file__).resolve().parents[4] / "build-sim" / "sentai_fs_root"
    sp, jp = root / SUMMARY_NAME, root / JOURNAL_NAME
    if not sp.exists():
        print(f"FAIL missing summary at {sp}"); return 1
    s = json.loads(sp.read_text()); j = jp.read_text() if jp.exists() else ""
    fails = []
    if s.get("status") != "DONE": fails.append(f"G1 status={s.get('status')}")
    mp = [p for p in EXPECTED_PHASES if p not in s.get("phases_done", [])]
    if mp: fails.append(f"G2 missing phases {mp}")
    if not s.get("lost_simulated"): fails.append("G3 lost_simulated=false")
    apex = s.get("pose_at_lost_apex")
    pre  = s.get("pose_pre_lost")
    if apex is None or pre is None:
        fails.append("G4 lost apex/pre poses not captured")
    else:
        dz = apex[2] - pre[2]
        if dz < GATE_LOST_DELTA_MIN:
            fails.append(f"G4 lost ascend Δz={dz:.3f} < {GATE_LOST_DELTA_MIN}")
    rec = s.get("pose_after_recovery")
    if rec is None: fails.append("G5 pose_after_recovery missing")
    cx = s.get("closure_xy")
    if cx is None or cx > GATE_CLOSURE_M:
        fails.append(f"G6 closure_xy={cx} > {GATE_CLOSURE_M}")
    if s.get("errors"): fails.append(f"G7 errors {s['errors']}")
    me = [e for e in EXPECTED_EVENTS if e not in j]
    if me: fails.append(f"G8 missing events {me}")

    print(f"[s148 verdict] {len(fails)} FAIL")
    for f in fails: print("  FAIL", f)
    if not fails:
        print("[s148 verdict] PASS — LOST recovery survived + closure gate hit")
        print(f"  closure_xy = {cx*100:.2f} cm")
        if apex and pre:
            print(f"  lost ascend Δz = {(apex[2]-pre[2])*100:.1f} cm")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

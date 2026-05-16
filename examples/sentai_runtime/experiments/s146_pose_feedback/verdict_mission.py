#!/usr/bin/env python3
"""verdict_mission.py — gates s146 mission_s146 (pose-feedback closure).

HARD gates per [[test-must-be-relevant-to-claim]] +
[[sim-test-must-return-home]] + [[missions-run-in-sentai-only]]:

  G1  summary["status"] == "DONE"
  G2  every expected phase fired
  G3  pose feedback worked (origin captured, all pose_* fields populated)
  G4  approach_dist < 0.15 m  (cf2 HL Commander got close to target)
  G5  return_dist   < 0.15 m  (cf2 returned near origin)
  G6  closure_xy    < 0.10 m  (xy distance origin↔land_pose) — THE BIG ONE
  G7  no unhandled exceptions
  G8  journal contains all expected events

Usage:
    python3 verdict_mission.py [/path/to/sentai_fs_root]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECTED_PHASES = [
    "init", "crtp_log_setup", "origin_captured", "takeoff",
    "approach", "dwell", "return", "land", "disarm",
]
EXPECTED_EVENTS = [
    "mission_start", "crazy_init", "log_reset", "toc_scan",
    "pose_subscribe", "origin_captured", "crazy_takeoff",
    "crazy_goto_target", "converge_target", "crazy_goto_origin",
    "converge_return", "crazy_land", "land_pose", "closure_xy",
    "crazy_disarm",
]

SUMMARY_NAME = "mission_s146_summary.json"
JOURNAL_NAME = "mission_s146_journal.txt"

GATE_CLOSURE_M  = 0.10   # canonical [[sim-test-must-return-home]]
GATE_APPROACH_M = 0.15   # cf2 HL Commander tolerance
GATE_RETURN_M   = 0.15


def _default_root() -> Path:
    return Path(__file__).resolve().parents[4] / "build-sim" / "sentai_fs_root"


def main() -> int:
    fs_root = Path(sys.argv[1]) if len(sys.argv) > 1 else _default_root()
    summary_path = fs_root / SUMMARY_NAME
    journal_path = fs_root / JOURNAL_NAME

    print(f"[verdict] fs_root = {fs_root}")

    fails = []

    if not summary_path.exists():
        print(f"  FAIL summary.json missing at {summary_path}")
        return 1
    if not journal_path.exists():
        print(f"  FAIL journal.txt missing at {journal_path}")
        return 1

    summary = json.loads(summary_path.read_text())
    journal = journal_path.read_text()

    # G1 — terminal status
    if summary.get("status") != "DONE":
        fails.append(f'G1: status != "DONE" (got {summary.get("status")!r})')

    # G2 — every expected phase fired
    phases = summary.get("phases_done", [])
    missing_phases = [p for p in EXPECTED_PHASES if p not in phases]
    if missing_phases:
        fails.append(f"G2: missing phases: {missing_phases}")

    # G3 — pose feedback populated
    for k in ("origin_xyz", "pose_at_target", "pose_at_return", "pose_at_land"):
        v = summary.get(k)
        if v is None or not isinstance(v, list) or len(v) != 3:
            fails.append(f"G3: pose field {k} missing or malformed")

    # G4 — approach distance
    ad = summary.get("approach_dist")
    if ad is None or ad < 0 or ad > GATE_APPROACH_M:
        fails.append(f"G4: approach_dist={ad}, expected 0..{GATE_APPROACH_M}")

    # G5 — return distance
    rd = summary.get("return_dist")
    if rd is None or rd < 0 or rd > GATE_RETURN_M:
        fails.append(f"G5: return_dist={rd}, expected 0..{GATE_RETURN_M}")

    # G6 — CLOSURE (the big one)
    cx = summary.get("closure_xy")
    if cx is None or cx > GATE_CLOSURE_M:
        fails.append(f"G6: closure_xy={cx}, expected < {GATE_CLOSURE_M}")

    # G7 — no unhandled exceptions
    errors = summary.get("errors", [])
    if errors:
        fails.append(f"G7: errors: {errors}")

    # G8 — journal coverage
    missing_events = [e for e in EXPECTED_EVENTS if e not in journal]
    if missing_events:
        fails.append(f"G8: missing journal events: {missing_events}")

    print(f"[verdict] gates: {len(fails)} FAIL")
    for f in fails:
        print("  FAIL", f)

    if not fails:
        print("[verdict] PASS — pure-MP closed-loop mission with closure gate")
        print(f"  closure_xy     = {cx*100:.2f} cm  (gate < {GATE_CLOSURE_M*100:.0f} cm)")
        print(f"  approach_dist  = {ad*100:.2f} cm")
        print(f"  return_dist    = {rd*100:.2f} cm")
        print(f"  pose_at_land   = ({summary['pose_at_land'][0]*100:+.2f}, "
              f"{summary['pose_at_land'][1]*100:+.2f}, "
              f"{summary['pose_at_land'][2]*100:.2f}) cm")
        print(f"  phases         = {phases}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

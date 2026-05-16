#!/usr/bin/env python3
"""verdict.py — gates s145 mission_template run (offline or live).

Reads summary.json + journal.txt from the SIM virtual FS, applies the
canonical hard gates derived from `[[test-must-be-relevant-to-claim]]`
+ `[[sim-test-must-return-home]]`:

  G1  summary["status"] == "DONE"
  G2  every expected phase fired (init / arm / takeoff / waypoints /
      land / disarm)
  G3  waypoints_visited == len(WAYPOINTS_EXPECTED)
  G4  crazy_init_rc == 0 (transport up)
  G5  every expected journal event present
  G6  no UNHANDLED exceptions (summary["errors"] empty *outside* the
      template's tolerated send-rc=-3 chatter)

Closure-vs-physical-origin (`[[sim-test-must-return-home]]`) cannot be
gated by this verdict because the template has no pose-feedback — the
mission ends the trajectory at xy=(0, 0) but we don't know where cf2
actually landed.  Once Task #41 adds `sentai.crazy.pose()`, gate G7
land_xy_err < 10 cm becomes mandatory.

Usage:
    python3 verdict.py [/path/to/sentai_fs_root]

Default fs_root is build-sim/sentai_fs_root/ relative to repo.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECTED_PHASES = ["init", "arm", "takeoff", "waypoints", "land", "disarm"]
EXPECTED_WAYPOINTS = 2
EXPECTED_EVENTS = [
    "crazy_init", "crazy_arm", "crazy_takeoff",
    "waypoint_start", "waypoint_done",
    "crazy_land", "crazy_disarm",
]

SUMMARY_NAME = "mission_template_summary.json"
JOURNAL_NAME = "mission_template_journal.txt"


def _default_root() -> Path:
    here = Path(__file__).resolve()
    repo = here.parents[4]
    return repo / "build-sim" / "sentai_fs_root"


def main() -> int:
    fs_root = Path(sys.argv[1]) if len(sys.argv) > 1 else _default_root()
    summary_path = fs_root / SUMMARY_NAME
    journal_path = fs_root / JOURNAL_NAME

    print(f"[verdict] fs_root = {fs_root}")
    print(f"[verdict] summary = {summary_path}")
    print(f"[verdict] journal = {journal_path}")

    fails = []

    # G0 — artifacts exist
    if not summary_path.exists():
        fails.append(f"summary.json missing at {summary_path}")
    if not journal_path.exists():
        fails.append(f"journal.txt missing at {journal_path}")
    if fails:
        for f in fails:
            print("  FAIL", f)
        return 1

    summary = json.loads(summary_path.read_text())
    journal = journal_path.read_text()

    # G1 — terminal status
    status = summary.get("status")
    if status != "DONE":
        fails.append(f'status != "DONE" (got {status!r})')

    # G2 — every expected phase fired
    phases = summary.get("phases_done", [])
    missing_phases = [p for p in EXPECTED_PHASES if p not in phases]
    if missing_phases:
        fails.append(f"missing phases: {missing_phases}")

    # G3 — waypoint count
    wp = summary.get("waypoints_visited", -1)
    if wp != EXPECTED_WAYPOINTS:
        fails.append(f"waypoints_visited = {wp}, expected {EXPECTED_WAYPOINTS}")

    # G4 — transport up
    init_rc = summary.get("crazy_init_rc")
    if init_rc != 0:
        fails.append(f"crazy_init_rc = {init_rc}, expected 0")

    # G5 — journal events
    missing_events = [e for e in EXPECTED_EVENTS if e not in journal]
    if missing_events:
        fails.append(f"missing journal events: {missing_events}")

    # G6 — no unhandled exceptions
    errors = summary.get("errors", [])
    if errors:
        fails.append(f"unhandled errors: {errors}")

    print(f"[verdict] gates: {len(fails)} FAIL")
    for f in fails:
        print("  FAIL", f)
    if not fails:
        print("[verdict] PASS — all gates green")
        print(f"  phases:    {phases}")
        print(f"  waypoints: {wp}")
        print(f"  version:   {summary.get('version', '?')}")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())

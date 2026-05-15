"""s134 verdict — PASS/FAIL gate for L6 Gazebo integration.

Read summary.json + apply pass criteria.  Exit 0 PASS / 1 FAIL.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

WORKDIR = Path("/tmp/s134_explore_gazebo")
SUMMARY = WORKDIR / "summary.json"

LAND_ERR_MAX_M     = 0.30
MIN_TRANSITIONS    = 8           # at least IDLE→ARMING→TAKEOFF→HOVERING→APPROACH→INSPECT→HOVERING→RETURNING→HOVERING→LANDING→DONE (some collapses ok)
MISSION_TIMEOUT_S  = 90.0


def verdict() -> int:
    if not SUMMARY.is_file():
        print("[verdict] FAIL — summary.json missing")
        return 1
    s = json.loads(SUMMARY.read_text())
    print(f"[verdict] summary _last_run={s.get('_last_run','?')}")
    if s.get("_status") != "OK":
        print(f"[verdict] FAIL — mission status='{s.get('_status')}' "
              f"exc={s.get('_exception')}")
        return 1

    state_final = s.get("state_final", "")
    aborts = (s.get("explore_metrics_final") or {}).get("aborts", -1)
    transitions = (s.get("explore_metrics_final") or {}).get("transitions", -1)
    land_err_xy = s.get("land_err_xy_m", 999.0)
    mission_s = s.get("mission_duration_s", 999.0)
    transitions_seen = s.get("transitions_seen", [])
    gotos_completed = (s.get("explore_metrics_final") or {}).get(
        "gotos_completed", 0)

    print(f"          state_final='{state_final}'  aborts={aborts}")
    print(f"          transitions_counter={transitions}  "
          f"len(transitions_seen)={len(transitions_seen)}")
    print(f"          gotos_completed={gotos_completed}")
    print(f"          land_err_xy={land_err_xy:.3f} m  "
          f"mission_duration={mission_s:.1f} s")
    print(f"          transitions_seen={transitions_seen}")

    ok = True
    if state_final != "DONE":
        print(f"[verdict] FAIL state_final != DONE")
        ok = False
    if aborts != 0:
        print(f"[verdict] FAIL aborts != 0")
        ok = False
    if transitions < MIN_TRANSITIONS:
        print(f"[verdict] FAIL transitions_counter < {MIN_TRANSITIONS}")
        ok = False
    if land_err_xy > LAND_ERR_MAX_M:
        print(f"[verdict] FAIL land_err_xy > {LAND_ERR_MAX_M} m")
        ok = False
    if mission_s > MISSION_TIMEOUT_S:
        print(f"[verdict] FAIL mission > {MISSION_TIMEOUT_S} s")
        ok = False

    # Soft expectations (informational, not gating)
    if "INSPECT" not in transitions_seen:
        print("[verdict] WARN  INSPECT never entered (could mean stop_dist "
              "was effectively zero distance)")
    if "RETURNING" not in transitions_seen:
        print("[verdict] WARN  RETURNING never entered")

    if ok:
        print("[verdict] PASS — L6 explore completes Gazebo mission end-to-end")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(verdict())

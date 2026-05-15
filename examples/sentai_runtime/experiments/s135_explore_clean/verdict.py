"""s135 verdict — STRICT origin closure gate.

Per [[sim-test-must-return-home]]: drone must land within
LAND_ERR_MAX_M of the PHYSICAL takeoff origin.  Anywhere else =
FAILED = drone LOST.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

WORKDIR = Path("/tmp/s135_explore_clean")
SUMMARY = WORKDIR / "summary.json"

LAND_ERR_MAX_M     = 0.10        # strict closure threshold
LAND_Z_MAX_M       = 0.10        # confirm actually landed (not midair)
MIN_TRANSITIONS    = 8
MISSION_TIMEOUT_S  = 60.0


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
    m = s.get("explore_metrics_final") or {}
    aborts = m.get("aborts", -1)
    transitions = m.get("transitions", -1)
    gotos = m.get("gotos_completed", 0)
    land_err = s.get("land_err_xy_vs_origin_m", 999.0)
    land_pose = s.get("land_pose", [0, 0, 0])
    physical_origin = s.get("physical_origin", [0, 0])
    mission_s = s.get("mission_duration_s", 999.0)
    transitions_seen = s.get("transitions_seen", [])
    l6_home_vs_origin = s.get("l6_home_vs_origin_m", -1)

    print(f"          PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
          f"{physical_origin[1]:+.3f})")
    print(f"          LAND_POSE       = ({land_pose[0]:+.3f},"
          f"{land_pose[1]:+.3f},{land_pose[2]:.3f})")
    print(f"          land_err_xy_vs_origin = {land_err*100:.1f} cm  "
          f"(threshold {LAND_ERR_MAX_M*100:.0f} cm)")
    print(f"          L6 home vs origin     = {l6_home_vs_origin*100:.1f} cm  "
          f"(informational)")
    print(f"          state_final='{state_final}'  aborts={aborts}")
    print(f"          transitions_counter={transitions}  "
          f"len(transitions_seen)={len(transitions_seen)}")
    print(f"          gotos_completed={gotos}")
    print(f"          mission_duration={mission_s:.1f} s")
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
    if land_err > LAND_ERR_MAX_M:
        print(f"[verdict] FAIL DRONE LOST — land_err_xy_vs_origin "
              f"({land_err*100:.1f} cm) > {LAND_ERR_MAX_M*100:.0f} cm")
        ok = False
    if land_pose[2] > LAND_Z_MAX_M:
        print(f"[verdict] FAIL land_z ({land_pose[2]:.2f} m) > "
              f"{LAND_Z_MAX_M} m — drone didn't actually touch down")
        ok = False
    if mission_s > MISSION_TIMEOUT_S:
        print(f"[verdict] FAIL mission > {MISSION_TIMEOUT_S} s")
        ok = False

    if "INSPECT" not in transitions_seen:
        print("[verdict] WARN  INSPECT never entered")
    if "RETURNING" not in transitions_seen:
        print("[verdict] WARN  RETURNING never entered")

    if ok:
        print("[verdict] PASS — drone returned to origin and landed cleanly")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(verdict())

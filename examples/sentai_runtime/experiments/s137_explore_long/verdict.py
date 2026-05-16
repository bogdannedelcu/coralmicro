"""s137 verdict — HARD RELEVANCE gates with at-scale thresholds."""
from __future__ import annotations
import json, sys, math
from pathlib import Path

WORKDIR = Path("/tmp/s137_explore_long")
SUMMARY = WORKDIR / "summary.json"

LAND_ERR_MAX_M    = 0.15
LAND_Z_MAX_M      = 0.10
GOTO1_MIN_M       = 1.20
GOTO2_MIN_M       = 1.50
RETURN_MIN_M      = 1.00
MIN_TRANSITIONS   = 8
REQUIRED_STATES   = {"APPROACH", "INSPECT", "RETURNING", "LANDING"}
MISSION_TIMEOUT_S = 150.0


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
    land_pose = s.get("land_pose", [0, 0, 999])
    physical_origin = s.get("physical_origin", [0, 0])
    mission_s = s.get("mission_duration_s", 999.0)
    transitions_seen = s.get("transitions_seen", [])

    d1 = s.get("displacement_goto1_m", 0)
    d2 = s.get("displacement_goto2_m", 0)
    dr = s.get("displacement_return_m", 0)

    print("─── Closure ───")
    print(f"  PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
          f"{physical_origin[1]:+.3f})")
    print(f"  LAND_POSE       = ({land_pose[0]:+.3f},"
          f"{land_pose[1]:+.3f},{land_pose[2]:.3f})")
    print(f"  land_err_xy_vs_origin = {land_err*100:.1f} cm  "
          f"(threshold {LAND_ERR_MAX_M*100:.0f} cm)")
    print("─── Behavior — at-scale motion ───")
    print(f"  goto1 displacement   = {d1*100:.1f} cm  (≥ {GOTO1_MIN_M*100:.0f})")
    print(f"  goto2 displacement   = {d2*100:.1f} cm  (≥ {GOTO2_MIN_M*100:.0f})")
    print(f"  return displacement  = {dr*100:.1f} cm  (≥ {RETURN_MIN_M*100:.0f})")
    print(f"  total mission path   = {(d1+d2+dr)*100:.1f} cm")
    print("─── FSM ───")
    print(f"  state_final='{state_final}'  aborts={aborts}")
    print(f"  transitions_counter={transitions}  "
          f"len(transitions_seen)={len(transitions_seen)}")
    print(f"  transitions_seen={transitions_seen}")
    print(f"  gotos_completed={gotos}  mission_duration={mission_s:.1f}s")

    ok = True
    def fail(msg):
        nonlocal ok
        print(f"[verdict] FAIL — {msg}")
        ok = False

    if state_final != "DONE":
        fail(f"state_final != DONE (got '{state_final}')")
    if aborts != 0:
        fail("aborts != 0")
    if land_err > LAND_ERR_MAX_M:
        fail(f"DRONE LOST — land_err {land_err*100:.1f} cm > {LAND_ERR_MAX_M*100:.0f} cm")
    if land_pose[2] > LAND_Z_MAX_M:
        fail(f"land_z {land_pose[2]:.2f} > {LAND_Z_MAX_M} m")
    if d1 < GOTO1_MIN_M:
        fail(f"DRONE DID NOT REACH TARGET 1 — disp {d1*100:.1f} cm < {GOTO1_MIN_M*100:.0f} cm")
    if d2 < GOTO2_MIN_M:
        fail(f"DRONE DID NOT REACH TARGET 2 — disp {d2*100:.1f} cm < {GOTO2_MIN_M*100:.0f} cm")
    if dr < RETURN_MIN_M:
        fail(f"DRONE DID NOT RETURN — disp {dr*100:.1f} cm < {RETURN_MIN_M*100:.0f} cm")
    missing = REQUIRED_STATES - set(transitions_seen)
    if missing:
        fail(f"REQUIRED FSM STATES NEVER ENTERED: {sorted(missing)}")
    if transitions < MIN_TRANSITIONS:
        fail(f"transitions_counter < {MIN_TRANSITIONS}")
    if gotos < 2:
        fail(f"gotos_completed < 2 (got {gotos})")
    if mission_s > MISSION_TIMEOUT_S:
        fail(f"mission > {MISSION_TIMEOUT_S} s")

    if ok:
        print(f"[verdict] PASS — drone traversed {(d1+d2+dr)*100:.0f} cm "
              f"and returned within {land_err*100:.0f} cm of origin")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(verdict())

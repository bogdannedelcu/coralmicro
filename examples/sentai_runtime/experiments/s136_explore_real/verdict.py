"""s136 verdict — HARD RELEVANCE gate.

Per [[test-must-be-relevant-to-claim]]: every test must prove the
behavior it claims, not just match output shape.  This verdict refuses
PASS unless the drone actually moved ≥ 12 cm to the marker, ≥ 12 cm
back, the lifter converged within 8 cm of GT, RETURNING was entered,
and closure vs PHYSICAL_ORIGIN is within 10 cm.

If a hard gate fails, the verdict prints the precise reason ("DRONE
NEVER MOVED to target", "RETURNING NEVER ENTERED", etc.) so logs are
unambiguous.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

WORKDIR = Path("/tmp/s136_explore_real")
SUMMARY = WORKDIR / "summary.json"

# ─── Closure (universal rule) ───
LAND_ERR_MAX_M               = 0.10
LAND_Z_MAX_M                 = 0.10

# ─── Relevance gates (behavior must have actually occurred) ───
DISPLACEMENT_TO_TARGET_MIN_M = 0.12     # ≥ 12 cm forward motion
DISPLACEMENT_BACK_MIN_M      = 0.12     # ≥ 12 cm backward motion
LIFTER_XY_ERR_MAX_M          = 0.08     # converged on real marker
REQUIRED_STATES              = {"APPROACH", "INSPECT", "RETURNING", "LANDING"}

# ─── Other ───
MIN_TRANSITIONS              = 8
MISSION_TIMEOUT_S            = 120.0


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

    disp_to_target = s.get("displacement_to_target_xy_m", 0)
    disp_back      = s.get("displacement_back_to_origin_xy_m", 0)
    lifter_err     = s.get("lifter_world_pos_xy_err_vs_GT_m", 999)
    lifter_status  = s.get("lifter_status", -1)
    l6_home_vs_origin = s.get("l6_home_vs_origin_m", -1)

    print("─── Closure ───")
    print(f"  PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
          f"{physical_origin[1]:+.3f})")
    print(f"  LAND_POSE       = ({land_pose[0]:+.3f},"
          f"{land_pose[1]:+.3f},{land_pose[2]:.3f})")
    print(f"  land_err_xy_vs_origin = {land_err*100:.1f} cm  "
          f"(threshold {LAND_ERR_MAX_M*100:.0f} cm)")
    print(f"  L6 home vs origin     = {l6_home_vs_origin*100:.1f} cm  "
          f"(informational)")
    print("─── Behavior (relevance) ───")
    print(f"  displacement to target   = {disp_to_target*100:.1f} cm  "
          f"(threshold ≥ {DISPLACEMENT_TO_TARGET_MIN_M*100:.0f} cm)")
    print(f"  displacement back        = {disp_back*100:.1f} cm  "
          f"(threshold ≥ {DISPLACEMENT_BACK_MIN_M*100:.0f} cm)")
    print(f"  lifter world_pos err     = {lifter_err*100:.1f} cm vs GT "
          f"(threshold ≤ {LIFTER_XY_ERR_MAX_M*100:.0f} cm)")
    print(f"  lifter_status            = {lifter_status} "
          f"(2=LIFTED, 1=TRACKING)")
    print("─── FSM ───")
    print(f"  state_final='{state_final}'  aborts={aborts}")
    print(f"  transitions_counter={transitions}  "
          f"len(transitions_seen)={len(transitions_seen)}")
    print(f"  transitions_seen={transitions_seen}")
    print(f"  gotos_completed={gotos}  mission_duration={mission_s:.1f}s")

    ok = True

    def fail(msg: str):
        nonlocal ok
        print(f"[verdict] FAIL — {msg}")
        ok = False

    # ─── Closure gates ───
    if state_final != "DONE":
        fail(f"state_final != DONE (got '{state_final}')")
    if aborts != 0:
        fail(f"aborts != 0")
    if land_err > LAND_ERR_MAX_M:
        fail(f"DRONE LOST — land_err_xy_vs_origin {land_err*100:.1f} cm > "
             f"{LAND_ERR_MAX_M*100:.0f} cm")
    if land_pose[2] > LAND_Z_MAX_M:
        fail(f"land_z {land_pose[2]:.2f} m > {LAND_Z_MAX_M} m — "
             f"drone did not touch down")

    # ─── Relevance gates ───
    if disp_to_target < DISPLACEMENT_TO_TARGET_MIN_M:
        fail(f"DRONE NEVER MOVED TO TARGET — displacement_to_target_xy "
             f"{disp_to_target*100:.1f} cm < "
             f"{DISPLACEMENT_TO_TARGET_MIN_M*100:.0f} cm.  Test claim 'L6 "
             f"navigates drone to marker' was NOT exercised.")
    if disp_back < DISPLACEMENT_BACK_MIN_M:
        fail(f"DRONE NEVER MOVED BACK — displacement_back "
             f"{disp_back*100:.1f} cm < {DISPLACEMENT_BACK_MIN_M*100:.0f} cm. "
             f"Test claim 'drone returns from marker' was NOT exercised.")
    if lifter_err > LIFTER_XY_ERR_MAX_M:
        fail(f"LIFTER DID NOT CONVERGE ON REAL MARKER — world_pos err "
             f"{lifter_err*100:.1f} cm > {LIFTER_XY_ERR_MAX_M*100:.0f} cm. "
             f"Test claim 'L5 lifter sees real marker' was NOT exercised.")
    missing_states = REQUIRED_STATES - set(transitions_seen)
    if missing_states:
        fail(f"REQUIRED FSM STATES NEVER ENTERED: {sorted(missing_states)}.  "
             f"Transitions seen: {transitions_seen}")

    # ─── Sanity ───
    if transitions < MIN_TRANSITIONS:
        fail(f"transitions_counter < {MIN_TRANSITIONS}")
    if mission_s > MISSION_TIMEOUT_S:
        fail(f"mission > {MISSION_TIMEOUT_S} s")

    if ok:
        print("[verdict] PASS — drone navigated to marker, returned, "
              "landed at origin")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(verdict())

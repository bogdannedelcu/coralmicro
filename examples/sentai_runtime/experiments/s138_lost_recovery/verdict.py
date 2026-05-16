"""s138 verdict — LOST recovery behavior gates."""
from __future__ import annotations
import json, sys
from pathlib import Path

WORKDIR = Path("/tmp/s138_lost_recovery")
SUMMARY = WORKDIR / "summary.json"

LAND_ERR_MAX_M    = 0.15
LAND_Z_MAX_M      = 0.10
ASCEND_MIN_M      = 1.0       # drone must rise ≥ 1 m during LOST
MISSION_TIMEOUT_S = 90.0


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
    land_err = s.get("land_err_xy_vs_origin_m", 999.0)
    land_pose = s.get("land_pose", [0, 0, 999])
    physical_origin = s.get("physical_origin", [0, 0])
    takeoff_z = s.get("physical_takeoff_z", 1.5)
    max_z = s.get("max_z_during_lost", 0)
    transitions_seen = s.get("transitions_seen", [])
    state_post_recovery = s.get("state_post_recovery", "")
    mission_s = s.get("mission_duration_s", 999.0)

    ascend = max_z - takeoff_z

    print("─── Closure ───")
    print(f"  PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},{physical_origin[1]:+.3f})")
    print(f"  LAND_POSE       = ({land_pose[0]:+.3f},{land_pose[1]:+.3f},"
          f"{land_pose[2]:.3f})")
    print(f"  land_err_xy_vs_origin = {land_err*100:.1f} cm  "
          f"(threshold {LAND_ERR_MAX_M*100:.0f} cm)")
    print("─── LOST behavior ───")
    print(f"  takeoff_z = {takeoff_z:.3f} m")
    print(f"  max_z_during_lost = {max_z:.3f} m  → ascend = {ascend*100:.0f} cm")
    print(f"     (threshold ≥ {ASCEND_MIN_M*100:.0f} cm)")
    print(f"  state_post_recovery = '{state_post_recovery}'")
    print("─── FSM ───")
    print(f"  state_final='{state_final}'  aborts={aborts}")
    print(f"  transitions_counter={transitions}")
    print(f"  transitions_seen={transitions_seen}")
    print(f"  mission_duration={mission_s:.1f}s")

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
        fail(f"land_z {land_pose[2]:.2f} m > {LAND_Z_MAX_M} m")

    if "LOST" not in transitions_seen:
        fail("LOST state never entered")
    if ascend < ASCEND_MIN_M:
        fail(f"DRONE DID NOT ASCEND — ascend {ascend*100:.0f} cm < {ASCEND_MIN_M*100:.0f} cm")
    if not state_post_recovery or state_post_recovery == "LOST":
        fail(f"LOST not exited on signal_marker_seen (state='{state_post_recovery}')")

    if mission_s > MISSION_TIMEOUT_S:
        fail(f"mission > {MISSION_TIMEOUT_S} s")

    if ok:
        print(f"[verdict] PASS — LOST entered, ascended {ascend*100:.0f} cm, "
              f"recovered to '{state_post_recovery}', landed within "
              f"{land_err*100:.1f} cm of origin")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(verdict())

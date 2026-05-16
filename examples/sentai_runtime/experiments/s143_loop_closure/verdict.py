"""s143 verdict — Loop closure match HARD gates."""
from __future__ import annotations
import json, sys
from pathlib import Path

WORKDIR = Path("/tmp/s143_loop_closure")
SUMMARY = WORKDIR / "summary.json"

LAND_ERR_MAX_M    = 0.15
LAND_Z_MAX_M      = 0.10
MIN_MATCH_SCORE   = 95          # deterministic seeds → near-100 expected
MISSION_TIMEOUT_S = 120.0


def verdict() -> int:
    if not SUMMARY.is_file():
        print("[verdict] FAIL — summary.json missing")
        return 1
    s = json.loads(SUMMARY.read_text())
    print(f"[verdict] summary _last_run={s.get('_last_run','?')}")
    if s.get("_status") != "OK":
        print(f"[verdict] FAIL — mission status='{s.get('_status')}'  "
              f"exc={s.get('_exception')}")
        return 1

    state_final = s.get("state_final", "")
    land_err = s.get("land_err_xy_vs_origin_m", 999.0)
    land_pose = s.get("land_pose", [0, 0, 999])
    origin = s.get("physical_origin", [0, 0])

    g_lap1 = s.get("gallery_count_after_lap1", 0)
    g_lap2 = s.get("gallery_count_after_lap2", 0)
    match_id = s.get("match_lap2_id", 0)
    match_score = s.get("match_lap2_score", 0)
    match_l1 = s.get("match_lap2_l1", 999)
    expected_id = s.get("expected_match_id", -1)

    mission_s = s.get("mission_duration_s", 999.0)
    transitions = s.get("transitions_seen", [])

    print("─── Closure ───")
    print(f"  PHYSICAL_ORIGIN = ({origin[0]:+.3f},{origin[1]:+.3f})")
    print(f"  LAND_POSE       = ({land_pose[0]:+.3f},{land_pose[1]:+.3f},"
          f"{land_pose[2]:.3f})")
    print(f"  land_err = {land_err*100:.1f} cm  (< {LAND_ERR_MAX_M*100:.0f} cm)")
    print("─── Gallery discipline ───")
    print(f"  count after lap1 = {g_lap1}")
    print(f"  count after lap2 = {g_lap2}  (must equal lap1: no store in lap2)")
    print("─── LOOP CLOSURE MATCH (key claim) ───")
    print(f"  expected match id = {expected_id}")
    print(f"  actual  match id  = {match_id}")
    print(f"  score = {match_score}%   (≥ {MIN_MATCH_SCORE}%)")
    print(f"  L1 dist between descriptors = {match_l1}")
    print("─── FSM ───")
    print(f"  state_final='{state_final}'")
    print(f"  mission_duration = {mission_s:.1f}s")
    print(f"  transitions = {transitions}")

    ok = True
    def fail(msg):
        nonlocal ok
        print(f"[verdict] FAIL — {msg}")
        ok = False

    if state_final != "DONE":
        fail(f"state_final != DONE ('{state_final}')")
    if land_err > LAND_ERR_MAX_M:
        fail(f"DRONE LOST — land_err {land_err*100:.1f} cm > "
             f"{LAND_ERR_MAX_M*100:.0f} cm")
    if land_pose[2] > LAND_Z_MAX_M:
        fail(f"land_z {land_pose[2]:.2f} > {LAND_Z_MAX_M} m")

    # Key loop closure gates
    if match_id != expected_id:
        fail(f"WRONG PLACE MATCHED — got id={match_id}, expected {expected_id}")
    if match_score < MIN_MATCH_SCORE:
        fail(f"LOW CONFIDENCE — score {match_score}% < {MIN_MATCH_SCORE}%")

    # Storage discipline
    if g_lap2 != g_lap1:
        fail(f"GALLERY GREW IN LAP2 — count went {g_lap1} → {g_lap2} "
             f"(lap-2 should query, not store)")

    if mission_s > MISSION_TIMEOUT_S:
        fail(f"mission > {MISSION_TIMEOUT_S} s")

    if ok:
        print(f"[verdict] PASS — drone recognized lap-1 place "
              f"(id={match_id}, {match_score}% confidence)")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(verdict())

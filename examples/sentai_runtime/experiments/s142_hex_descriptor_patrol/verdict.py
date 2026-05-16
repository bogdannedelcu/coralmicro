"""s142 verdict — HexPatrol HARD gates (descriptor + H3 + closure)."""
from __future__ import annotations
import json, sys
from pathlib import Path

WORKDIR = Path("/tmp/s142_hex_descriptor_patrol")
SUMMARY = WORKDIR / "summary.json"

LAND_ERR_MAX_M    = 0.15
LAND_Z_MAX_M      = 0.10
MIN_PLACES        = 3              # home + 2 targets
MIN_DISTINCT_H3   = 3              # each at unique cell
MISSION_TIMEOUT_S = 120.0


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
    land_err = s.get("land_err_xy_vs_origin_m", 999.0)
    land_pose = s.get("land_pose", [0, 0, 999])
    origin = s.get("physical_origin", [0, 0])
    gallery_count = s.get("gallery_count", 0)
    distinct_cells = s.get("distinct_h3_cells", 0)
    all_descs_set = s.get("all_descs_set", False)
    match_results = s.get("match_results", [])
    mission_s = s.get("mission_duration_s", 999.0)
    transitions_seen = s.get("transitions_seen", [])

    print("─── Closure ───")
    print(f"  PHYSICAL_ORIGIN = ({origin[0]:+.3f},{origin[1]:+.3f})")
    print(f"  LAND_POSE       = ({land_pose[0]:+.3f},{land_pose[1]:+.3f},"
          f"{land_pose[2]:.3f})")
    print(f"  land_err vs origin = {land_err*100:.1f} cm  "
          f"(threshold {LAND_ERR_MAX_M*100:.0f} cm)")
    print("─── L3 gallery ───")
    print(f"  count = {gallery_count}    (≥ {MIN_PLACES})")
    print(f"  distinct H3 cells = {distinct_cells}    (≥ {MIN_DISTINCT_H3})")
    print(f"  all descriptors set = {all_descs_set}")
    print(f"  self-match results = {match_results}")
    print("─── FSM ───")
    print(f"  state_final='{state_final}'")
    print(f"  transitions = {transitions_seen}")
    print(f"  mission_duration = {mission_s:.1f}s")

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
    if gallery_count < MIN_PLACES:
        fail(f"GALLERY UNDERPOPULATED — count {gallery_count} < {MIN_PLACES}")
    if distinct_cells < MIN_DISTINCT_H3:
        fail(f"H3 CELLS NOT DISTINCT — only {distinct_cells} unique "
             f"out of {MIN_DISTINCT_H3} expected")
    if not all_descs_set:
        fail("DESCRIPTORS NOT POPULATED — at least one desc_set == 0")
    for pid, mid in match_results:
        if pid != mid:
            fail(f"SELF-MATCH BROKEN — query(desc[{pid}]) returned {mid}, "
                 f"expected {pid}")
    if mission_s > MISSION_TIMEOUT_S:
        fail(f"mission > {MISSION_TIMEOUT_S} s")

    if ok:
        print(f"[verdict] PASS — {gallery_count} places stored at {distinct_cells} "
              f"distinct H3 cells; closure {land_err*100:.1f} cm")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(verdict())

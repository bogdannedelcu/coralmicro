"""s144 verdict — Real-frame loop closure HARD gates.

Per [[test-must-be-relevant-to-claim]]: each claim has its HARD gate.
Includes new telemetry health gate (tel_max_gap_ms < 5s).
"""
from __future__ import annotations
import json, sys
from pathlib import Path

WORKDIR = Path("/tmp/s144_realframe_loop_closure")
SUMMARY = WORKDIR / "summary.json"

LAND_ERR_MAX_M     = 0.15
LAND_Z_MAX_M       = 0.10
MIN_MATCH_SCORE    = 60          # relaxed from s143's 95% (natural noise)
TEL_MAX_GAP_MS_OK  = 5000        # tel callbacks never silent > 5s
MISSION_TIMEOUT_S  = 150.0


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

    match = s.get("match_lap2", {}) or {}
    match_id    = match.get("id", 0)
    match_score = match.get("score_pct", 0)
    match_l1    = match.get("l1_dist", 0)
    frame_seq_2 = match.get("frame_seq_lap2", 0)
    expected_id = s.get("expected_match_id", -1)

    tel = s.get("tel_summary", {}) or {}
    tel_n  = tel.get("n_callbacks", 0)
    tel_gap = tel.get("max_gap_ms", 999999)

    mission_s = s.get("mission_duration_s", 999.0)
    transitions = s.get("transitions_seen", [])

    print("─── Closure ───")
    print(f"  PHYSICAL_ORIGIN = ({origin[0]:+.3f},{origin[1]:+.3f})")
    print(f"  LAND_POSE       = ({land_pose[0]:+.3f},{land_pose[1]:+.3f},"
          f"{land_pose[2]:.3f})")
    print(f"  land_err = {land_err*100:.1f} cm  (< {LAND_ERR_MAX_M*100:.0f} cm)")
    print("─── Gallery discipline ───")
    print(f"  lap1 count = {g_lap1}    lap2 count = {g_lap2}  (must equal lap1)")
    print("─── REAL-FRAME LOOP CLOSURE (key claim) ───")
    print(f"  expected match id = {expected_id}")
    print(f"  actual  match id  = {match_id}")
    print(f"  match score = {match_score}%   (≥ {MIN_MATCH_SCORE}%)")
    print(f"  match L1 dist = {match_l1}    (MUST BE > 0 — proves NEW frame "
          "differs from stored)")
    print(f"  lap2 frame_seq = {frame_seq_2}")
    print("─── Telemetry observability ───")
    print(f"  tel n_callbacks = {tel_n}")
    print(f"  tel max_gap_ms  = {tel_gap}  (< {TEL_MAX_GAP_MS_OK} ms)")
    print("─── FSM ───")
    print(f"  state_final = '{state_final}'   mission = {mission_s:.1f}s")
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

    # Loop closure claim
    if match_id != expected_id:
        fail(f"WRONG PLACE — got id={match_id}, expected {expected_id}")
    if match_score < MIN_MATCH_SCORE:
        fail(f"LOW CONFIDENCE — score {match_score}% < {MIN_MATCH_SCORE}%")
    if match_l1 <= 0:
        fail("DESCRIPTOR BIT-IDENTICAL — l1=0 means lap-2 frame matched "
             "lap-1 byte-for-byte (synthetic or stale).  Real frames "
             "should differ slightly.")

    if g_lap1 != g_lap2:
        fail(f"GALLERY GREW IN LAP2 — {g_lap1} → {g_lap2}")

    # Telemetry health
    if tel_gap > TEL_MAX_GAP_MS_OK:
        fail(f"TELEMETRY STALENESS — max_gap {tel_gap} ms > "
             f"{TEL_MAX_GAP_MS_OK} ms (callback went silent mid-mission; "
             f"investigate cf2 log channel or cflib state)")

    if mission_s > MISSION_TIMEOUT_S:
        fail(f"mission > {MISSION_TIMEOUT_S} s")

    if ok:
        print(f"[verdict] PASS — real frame recognized as place id={match_id} "
              f"@ {match_score}% confidence (L1={match_l1}); "
              f"tel max_gap={tel_gap}ms")
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(verdict())

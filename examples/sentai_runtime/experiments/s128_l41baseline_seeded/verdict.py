#!/usr/bin/env python3
"""verdict.py — read summary.json + telemetry from /tmp/s128_l41baseline/
and decide PASS/FAIL.  Cross-checks L4 trace ring against expected
sequence, L2 storage, cf2 waypoint arrival, and prints REPL transcript
tail on failure (per operator request — so you know WHERE and WHY it
broke at runtime).

Exit 0 on PASS, 1 on FAIL.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

WORKDIR = Path("/tmp/s128_l41baseline")
SUMMARY = WORKDIR / "summary.json"
TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG = WORKDIR / "mission.log"
JOURNAL = WORKDIR / "journal.txt"

# ─── Action / backend / flight enums (must match sentai_servo.h) ───
ACT_NONE, ACT_INIT, ACT_ARM, ACT_DISARM = 0, 1, 2, 3
ACT_TAKEOFF, ACT_MOVE, ACT_HOVER, ACT_LAND = 4, 5, 6, 7
BACKEND_SIM = 1
FLIGHT_GROUND, FLIGHT_AIRBORNE = 0, 1

# s128 (2026-05-14, 4-marker tour, closed-loop EKF): MOVE count is
# variable because each waypoint may need 1..NAV_MAX_ITERS corrections.
# So we drop the exact-sequence check and validate STRUCTURAL invariants:
#   - first non-trace-ring-rolled action is ARM
#   - last action is DISARM
#   - HOVER count == N_MARKERS (one dwell per waypoint)
#   - every MOVE entry has result == 0
N_MARKERS             = 4
EXPECTED_HOVER_COUNT  = N_MARKERS
# trace ring depth = 16; if we hit too many corrections per waypoint,
# entries roll off and `trace_count` clamps at 16 (overwrites > 0).
# That's still a valid run if the visual + ground-truth gates pass.
WAYPOINT_NEAR_M       = 0.30
# Visual baseline (operator request, 2026-05-14): the chosen ArUco
# marker MUST appear within MARKER_PIXEL_NEAR_PX of the image centre
# in the downward camera at end-of-hover.  640×480, fy≈fx≈579 px →
# 80 px ≈ 14 cm physical at z=1 m hover (wider than EKF/flow settle
# band, so this is the loose first-pass gate).
MARKER_PIXEL_NEAR_PX  = 80


def _print_journal_tail(n: int = 12) -> None:
    """The sentai.sim journal is the canonical 'where did it stop and why'
    artifact.  Format (one entry per line):
        <t_ms> <label> <repr(value)|'-'>
    Lines beginning with '#' are header/footer annotations.  The LAST
    non-comment line is the last successfully completed step before
    crash (or normal mission end)."""
    if not JOURNAL.is_file():
        print(f"[verdict] (no journal at {JOURNAL})")
        return
    lines = JOURNAL.read_text(errors="replace").splitlines()
    print(f"\n── Last {min(n, len(lines))} journal entries ({JOURNAL}) ──")
    for ln in lines[-n:]:
        print(f"  | {ln}")
    print("──")


def _print_transcript_tail(n: int = 30) -> None:
    if not TRANSCRIPT.is_file():
        print(f"[verdict] (no transcript at {TRANSCRIPT})")
        return
    print(f"\n── Last {n} lines of {TRANSCRIPT} ──")
    lines = TRANSCRIPT.read_text(errors="replace").splitlines()
    for ln in lines[-n:]:
        print(f"  | {ln}")
    print("──")


def _print_mission_tail(n: int = 20) -> None:
    if not MISSION_LOG.is_file():
        print(f"[verdict] (no mission.log at {MISSION_LOG})")
        return
    print(f"\n── Last {n} lines of {MISSION_LOG} ──")
    lines = MISSION_LOG.read_text(errors="replace").splitlines()
    for ln in lines[-n:]:
        print(f"  | {ln}")
    print("──")


def main() -> int:
    if not SUMMARY.is_file():
        print(f"FAIL — no summary.json at {SUMMARY}")
        _print_journal_tail()
        _print_mission_tail()
        _print_transcript_tail()
        return 1

    d = json.loads(SUMMARY.read_text())
    status   = d.get("_status")
    waypoints= d.get("waypoints", [])
    servo_st = d.get("servo_status", {})
    trace    = d.get("servo_trace", [])
    objects  = d.get("objects_list", [])
    ts       = d.get("_last_run")

    print(f"[verdict] summary.json _last_run={ts}  _status={status}")
    if status != "OK":
        print(f"FAIL — mission did not complete cleanly "
              f"(exc={d.get('_exception')})")
        _print_journal_tail()
        _print_mission_tail()
        _print_transcript_tail()
        return 1

    fails: list[str] = []

    # ── L4: trace ring shape + counters ──
    actions_ok       = servo_st.get("actions_ok", -1)
    faults_oob       = servo_st.get("faults_oob", -1)
    faults_no_bk     = servo_st.get("faults_no_backend", -1)
    faults_not_armed = servo_st.get("faults_not_armed", -1)
    last_action      = servo_st.get("last_action", -1)
    last_result      = servo_st.get("last_result", -99)
    backend          = servo_st.get("backend", -1)
    flight           = servo_st.get("flight", -1)
    armed            = servo_st.get("armed", -1)
    trace_count      = servo_st.get("trace_count", -1)
    print(f"          servo backend={backend} armed={armed} flight={flight}  "
          f"trace_count={trace_count}  last={last_action}/{last_result}")
    print(f"          actions_ok={actions_ok} faults oob={faults_oob} "
          f"no_bk={faults_no_bk} not_armed={faults_not_armed}")

    # actions_ok lower bound: INIT(1) + ARM(1) + TAKEOFF(1) + N_MARKERS*HOVER
    # + LAND(1) + DISARM(1) + at_least_one_MOVE_per_waypoint(N_MARKERS) = 5 + 2N.
    # No upper bound — closed-loop may legitimately issue many corrections.
    min_actions_ok = 5 + 2 * N_MARKERS
    if actions_ok < min_actions_ok:
        fails.append(f"actions_ok={actions_ok} < min {min_actions_ok}")
    if faults_oob != 0:
        fails.append(f"faults_oob={faults_oob} != 0")
    if faults_no_bk != 0:
        fails.append(f"faults_no_backend={faults_no_bk} != 0")
    if faults_not_armed != 0:
        fails.append(f"faults_not_armed={faults_not_armed} != 0")
    if last_action != ACT_DISARM:
        fails.append(f"last_action={last_action} != ACT_DISARM ({ACT_DISARM})")
    if last_result != 0:
        fails.append(f"last_result={last_result} != 0")
    if backend != BACKEND_SIM:
        fails.append(f"backend={backend} != SIM ({BACKEND_SIM})")
    if armed != 0:
        fails.append(f"final armed={armed} != 0 (should be disarmed)")
    if flight != FLIGHT_GROUND:
        fails.append(f"final flight={flight} != GROUND ({FLIGHT_GROUND})")

    # Structural trace check (closed-loop adds variable MOVE count, so
    # we validate invariants instead of an exact sequence):
    actual_seq = [e.get("action") for e in trace]
    n_hover = sum(1 for a in actual_seq if a == ACT_HOVER)
    n_move  = sum(1 for a in actual_seq if a == ACT_MOVE)
    n_takeoff = sum(1 for a in actual_seq if a == ACT_TAKEOFF)
    n_land    = sum(1 for a in actual_seq if a == ACT_LAND)
    print(f"          trace action counts: MOVE={n_move} HOVER={n_hover} "
          f"TAKEOFF={n_takeoff} LAND={n_land}")
    if n_hover != EXPECTED_HOVER_COUNT:
        # Note: with deep closed-loop iteration the ring may roll over
        # and lose old entries; the lifetime counter is more reliable
        # for n_hover than the snapshot but we'll catch outright misses.
        fails.append(f"trace HOVER count = {n_hover} (expected "
                     f"{EXPECTED_HOVER_COUNT} — one per waypoint)")
    if n_move < EXPECTED_HOVER_COUNT:
        fails.append(f"trace MOVE count = {n_move} < {EXPECTED_HOVER_COUNT} "
                     f"(need at least one move per waypoint)")
    if actual_seq and actual_seq[-1] != ACT_DISARM:
        fails.append(f"trace last action = {actual_seq[-1]} (expected DISARM)")

    # All MOVE entries should have result==0.
    bad_moves = [(i, e) for i, e in enumerate(trace)
                 if e.get("action") == ACT_MOVE and e.get("result") != 0]
    if bad_moves:
        fails.append(f"{len(bad_moves)} MOVE entries with non-zero result")

    # ── L2: storage survived flight ──
    print(f"          objects.list() returned {len(objects)} entries")
    if len(objects) != N_MARKERS:
        fails.append(f"objects.list() = {len(objects)} entries, expected {N_MARKERS}")

    # ── Cf2 ground truth: drone reached each waypoint ──
    print(f"          waypoint tour ({len(waypoints)} entries):")
    for w in waypoints:
        dist = w.get("dist_m", float("inf"))
        wp_ok = dist < WAYPOINT_NEAR_M
        marker = "OK " if wp_ok else "BAD"
        print(f"            [{marker}] {w.get('label')}  "
              f"target={w.get('target')}  cf2={w.get('cf2')}  dist={dist:.3f} m")
        if not wp_ok:
            fails.append(f"{w.get('label')} dist {dist:.3f} m >= "
                         f"{WAYPOINT_NEAR_M} m threshold")
        # ── Visual baseline: marker centered in image ──
        vis = w.get("visual")
        if vis is None:
            fails.append(f"{w.get('label')} missing 'visual' record — "
                         f"mission did not run the pixel-center check")
            continue
        if not vis.get("detected", False):
            fails.append(f"{w.get('label')} marker not detected during hover "
                         f"(hits=0) — drone not actually pointing at marker")
            continue
        px = vis.get("px_dist_to_center", float("inf"))
        vis_ok = px < MARKER_PIXEL_NEAR_PX
        vmarker = "OK " if vis_ok else "BAD"
        print(f"                  [{vmarker}] visual: id{w.get('aruco_id')}"
              f"  mean=({vis['cx_mean']:.0f},{vis['cy_mean']:.0f})"
              f"  img_center=({vis['img_cx']:.0f},{vis['img_cy']:.0f})"
              f"  px_dist={px:.1f}  hits={vis['n_hits']}")
        if not vis_ok:
            fails.append(f"{w.get('label')} pixel_dist {px:.1f} >= "
                         f"{MARKER_PIXEL_NEAR_PX} px (marker not centered)")
    if len(waypoints) != N_MARKERS:
        fails.append(f"only {len(waypoints)} waypoints flown, expected {N_MARKERS}")

    # ── Verdict ──
    if fails:
        print("\nFAIL — " + ";  ".join(fails))
        _print_journal_tail()
        _print_mission_tail()
        _print_transcript_tail()
        return 1
    print("\nPASS — L4.1Baseline: L2 + L4 + cf2/flow integrated")
    return 0


if __name__ == "__main__":
    sys.exit(main())

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

# Simplified s128 (2026-05-14): ONE marker tour.  Trace ring after
# clear_trace contains: ARM, TAKEOFF, MOVE, HOVER, LAND, DISARM = 6.
EXPECTED_SEQ = [
    ACT_ARM,
    ACT_TAKEOFF,
    ACT_MOVE, ACT_HOVER,
    ACT_LAND,
    ACT_DISARM,
]
# clear_trace() preserves lifetime counters, so actions_ok counts the
# pre-clear INIT plus everything after:
#   INIT, ARM, TAKEOFF, MOVE, HOVER, LAND, DISARM = 7 successful.
EXPECTED_ACTIONS_OK = 7
WAYPOINT_NEAR_M     = 0.30
N_MARKERS           = 1


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

    if actions_ok != EXPECTED_ACTIONS_OK:
        fails.append(f"actions_ok={actions_ok} != {EXPECTED_ACTIONS_OK}")
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

    actual_seq = [e.get("action") for e in trace]
    if actual_seq != EXPECTED_SEQ:
        fails.append(f"trace action sequence mismatch:\n"
                     f"          expected {EXPECTED_SEQ}\n"
                     f"          actual   {actual_seq}")

    # All MOVE entries should have result==0 (every move was within
    # the 5 m / pi/2 caps — markers placed at radius 0.5 m).
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
        ok = dist < WAYPOINT_NEAR_M
        marker = "OK " if ok else "BAD"
        print(f"            [{marker}] {w.get('label')}  "
              f"target={w.get('target')}  cf2={w.get('cf2')}  dist={dist:.3f} m")
        if not ok:
            fails.append(f"{w.get('label')} dist {dist:.3f} m >= "
                         f"{WAYPOINT_NEAR_M} m threshold")
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

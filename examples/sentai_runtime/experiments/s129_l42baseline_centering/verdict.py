#!/usr/bin/env python3
"""verdict.py — s129 L4.2Baseline (visual servoing).

PASS criteria:
  1. mission `_status` == "OK"
  2. IBVS converged (loop exited via tolerance, not timeout / max-iters)
  3. final visual pixel error < MARKER_PIXEL_TIGHT_PX
  4. trace_count ≥ minimum (init+arm+takeoff+ ≥1move + hover + land+disarm)
  5. all servo fault counters == 0, last_result == 0

On FAIL prints journal tail + mission tail + transcript tail.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

WORKDIR = Path("/tmp/s129_l42baseline")
SUMMARY = WORKDIR / "summary.json"
TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG = WORKDIR / "mission.log"
JOURNAL = WORKDIR / "journal.txt"
IBVS_LOG = WORKDIR / "ibvs_iter_log.json"

ACT_NONE, ACT_INIT, ACT_ARM, ACT_DISARM = 0, 1, 2, 3
ACT_TAKEOFF, ACT_MOVE, ACT_HOVER, ACT_LAND = 4, 5, 6, 7
BACKEND_SIM = 1
FLIGHT_GROUND = 0

MARKER_PIXEL_TIGHT_PX = 30
N_MARKERS_EXPECTED    = 4
# Floor: INIT + ARM + TAKEOFF + 4*(>=1 MOVE + 1 HOVER) + LAND + DISARM = 13.
MIN_ACTIONS_OK        = 13


def _print_tail(path: Path, n: int, label: str) -> None:
    if not path.is_file():
        print(f"[verdict] (no {label} at {path})")
        return
    lines = path.read_text(errors="replace").splitlines()
    print(f"\n── Last {min(n, len(lines))} lines of {path} ──")
    for ln in lines[-n:]:
        print(f"  | {ln}")
    print("──")


def main() -> int:
    if not SUMMARY.is_file():
        print(f"FAIL — no summary.json at {SUMMARY}")
        _print_tail(JOURNAL, 12, "journal")
        _print_tail(MISSION_LOG, 20, "mission.log")
        _print_tail(TRANSCRIPT, 30, "repl.transcript")
        return 1

    d = json.loads(SUMMARY.read_text())
    status = d.get("_status")
    print(f"[verdict] _last_run={d.get('_last_run')}  _status={status}")
    if status != "OK":
        print(f"FAIL — mission exception: {d.get('_exception')}")
        _print_tail(JOURNAL, 12, "journal")
        _print_tail(MISSION_LOG, 20, "mission.log")
        _print_tail(TRANSCRIPT, 30, "repl.transcript")
        return 1

    fails: list[str] = []

    # L4 FSM sanity
    s = d.get("servo_status", {})
    actions_ok       = s.get("actions_ok", -1)
    faults_oob       = s.get("faults_oob", -1)
    faults_no_bk     = s.get("faults_no_backend", -1)
    faults_not_armed = s.get("faults_not_armed", -1)
    last_action      = s.get("last_action", -1)
    last_result      = s.get("last_result", -99)
    armed            = s.get("armed", -1)
    flight           = s.get("flight", -1)
    print(f"          servo: backend={s.get('backend')} armed={armed} "
          f"flight={flight}  actions_ok={actions_ok}  last={last_action}/{last_result}")
    print(f"          faults oob={faults_oob} no_bk={faults_no_bk} "
          f"not_armed={faults_not_armed}")
    if actions_ok < MIN_ACTIONS_OK:
        fails.append(f"actions_ok={actions_ok} < min {MIN_ACTIONS_OK}")
    if faults_oob != 0 or faults_no_bk != 0 or faults_not_armed != 0:
        fails.append(f"non-zero fault counters")
    if last_action != ACT_DISARM or last_result != 0:
        fails.append(f"last action != DISARM/0  (got {last_action}/{last_result})")
    if armed != 0 or flight != FLIGHT_GROUND:
        fails.append(f"final armed/flight = {armed}/{flight} (expected 0/GROUND)")

    # IBVS per-marker results
    per_marker = d.get("per_marker", [])
    n_converged = d.get("n_markers_converged", 0)
    n_total     = d.get("n_markers_total", N_MARKERS_EXPECTED)
    print(f"          IBVS tour: {n_converged}/{n_total} markers converged")
    for m in per_marker:
        mid = m.get("id")
        ok = m.get("converged", False)
        n_it = m.get("n_iters", -1)
        perr = m.get("final_pixel_err", m.get("last_pixel_err", float("inf")))
        cfp = m.get("cf2_at_converge")
        pix = m.get("final_pixel")
        tag = "OK " if ok and perr < MARKER_PIXEL_TIGHT_PX else "BAD"
        cf2s = (f"  cf2=({cfp[0]:+.2f},{cfp[1]:+.2f},{cfp[2]:.2f})"
                if cfp else "")
        pxs  = (f"  pixel=({pix[0]:.0f},{pix[1]:.0f})" if pix else "")
        print(f"            [{tag}] id{mid}: converged={ok}  "
              f"n_iters={n_it}  px_err={perr:.1f}{pxs}{cf2s}")
        if not ok:
            fails.append(f"id{mid}: IBVS did not converge "
                         f"(n_iters={n_it}, last_pixel_err={perr:.1f})")
        elif perr >= MARKER_PIXEL_TIGHT_PX:
            fails.append(f"id{mid}: final px_err {perr:.1f} >= "
                         f"{MARKER_PIXEL_TIGHT_PX} px threshold")
    if n_converged != n_total:
        fails.append(f"only {n_converged}/{n_total} markers converged")

    print(f"          cf2 yaw post-takeoff = {d.get('cf2_yaw_deg','?')}°")

    if fails:
        print("\nFAIL — " + ";  ".join(fails))
        _print_tail(JOURNAL, 14, "journal")
        _print_tail(MISSION_LOG, 30, "mission.log")
        return 1
    print("\nPASS — L4.2Baseline: visual servoing centred on target")
    return 0


if __name__ == "__main__":
    sys.exit(main())

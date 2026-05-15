#!/usr/bin/env python3
"""verdict.py — s132 lifter Gazebo integration.

PASS criteria:
  L4 FSM sanity:
    1. _status == "OK"
    2. all servo fault counters == 0
    3. last_action == DISARM, last_result == 0
    4. final armed==0, flight==GROUND

  Lifter-specific:
    5. final snap.status == LIFTED (constant value 2)
    6. err_xy_m < 0.20
    7. err_z_m  < 0.60
    8. sigma_rho_ratio (final/init) < 0.5
    9. final snap.n_obs >= 4
   10. rejects (any flavor) <= 1

On FAIL: tail journal + mission log + transcript for forensics.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

WORKDIR     = Path("/tmp/s132_lifter_gazebo")
SUMMARY     = WORKDIR / "summary.json"
TRANSCRIPT  = WORKDIR / "repl.transcript"
MISSION_LOG = WORKDIR / "mission.log"
JOURNAL     = WORKDIR / "journal.txt"
LIFTER_TR   = WORKDIR / "lifter_trace.json"

ACT_NONE, ACT_INIT, ACT_ARM, ACT_DISARM = 0, 1, 2, 3
ACT_TAKEOFF, ACT_MOVE, ACT_HOVER, ACT_LAND = 4, 5, 6, 7
FLIGHT_GROUND = 0

# Lifter status constants (must match sentai_object_lifter.h enum)
LIFTER_FREE     = 0
LIFTER_TRACKING = 1
LIFTER_LIFTED   = 2
LIFTER_LOST     = 3

ERR_XY_MAX_M    = 0.20
ERR_Z_MAX_M     = 0.60
SIGMA_RHO_RATIO = 0.5
N_OBS_MIN       = 4
REJECTS_MAX     = 1


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
        _print_tail(MISSION_LOG, 30, "mission.log")
        _print_tail(TRANSCRIPT, 30, "repl.transcript")
        return 1

    d = json.loads(SUMMARY.read_text())
    status = d.get("_status")
    print(f"[verdict] _last_run={d.get('_last_run')}  _status={status}")
    if status != "OK":
        print(f"FAIL — mission exception: {d.get('_exception')}")
        _print_tail(JOURNAL, 12, "journal")
        _print_tail(MISSION_LOG, 30, "mission.log")
        _print_tail(TRANSCRIPT, 30, "repl.transcript")
        return 1

    fails: list[str] = []

    # ─── L4 FSM sanity ───
    s = d.get("servo_status", {})
    faults_oob       = s.get("faults_oob", -1)
    faults_no_bk     = s.get("faults_no_backend", -1)
    faults_not_armed = s.get("faults_not_armed", -1)
    last_action      = s.get("last_action", -1)
    last_result      = s.get("last_result", -99)
    armed            = s.get("armed", -1)
    flight           = s.get("flight", -1)
    print(f"          servo: backend={s.get('backend')} armed={armed} "
          f"flight={flight}  last={last_action}/{last_result}")
    print(f"          faults oob={faults_oob} no_bk={faults_no_bk} "
          f"not_armed={faults_not_armed}")
    if faults_oob != 0 or faults_no_bk != 0 or faults_not_armed != 0:
        fails.append("non-zero servo fault counters")
    if last_action != ACT_DISARM or last_result != 0:
        fails.append(f"last action != DISARM/0  (got {last_action}/{last_result})")
    if armed != 0 or flight != FLIGHT_GROUND:
        fails.append(f"final armed/flight = {armed}/{flight} (expected 0/GROUND)")

    # ─── Lifter-specific ───
    snap = d.get("final_snap", {}) or {}
    snap_status = snap.get("status", -1)
    n_obs       = snap.get("n_obs", -1)
    rho         = snap.get("rho", float("nan"))
    var_rho     = snap.get("var_rho", float("nan"))
    err_xy      = d.get("err_xy_m", float("nan"))
    err_z       = d.get("err_z_m",  float("nan"))
    sig_ratio   = d.get("sigma_rho_ratio", float("nan"))
    sig_init    = d.get("sigma_rho_init",  float("nan"))
    sig_final   = d.get("sigma_rho_final", float("nan"))
    wpos        = d.get("final_world_pos")
    gt          = d.get("target_world_xyz", [None, None, None])

    print(f"          lifter snap: status={snap_status} n_obs={n_obs} "
          f"rho={rho:.3f} var_rho={var_rho:.4f}")
    print(f"          world_pos_est={wpos}  GT={gt}")
    print(f"          err_xy={err_xy*100:.1f} cm (thr<{ERR_XY_MAX_M*100:.0f})  "
          f"err_z={err_z*100:.1f} cm (thr<{ERR_Z_MAX_M*100:.0f})")
    print(f"          sigma_rho: init={sig_init:.4f} → final={sig_final:.4f}  "
          f"ratio={sig_ratio:.3f} (thr<{SIGMA_RHO_RATIO})")

    if snap_status != LIFTER_LIFTED:
        fails.append(f"final status={snap_status} != LIFTED ({LIFTER_LIFTED})")
    if n_obs < N_OBS_MIN:
        fails.append(f"n_obs={n_obs} < {N_OBS_MIN}")
    if not (err_xy < ERR_XY_MAX_M):
        fails.append(f"err_xy={err_xy*100:.1f} cm >= {ERR_XY_MAX_M*100:.0f} cm")
    if not (err_z < ERR_Z_MAX_M):
        fails.append(f"err_z={err_z*100:.1f} cm >= {ERR_Z_MAX_M*100:.0f} cm")
    if not (sig_ratio < SIGMA_RHO_RATIO):
        fails.append(f"sigma_rho_ratio={sig_ratio:.3f} >= {SIGMA_RHO_RATIO}")

    stats = d.get("stats", {}) or {}
    rejects = (stats.get("rejects_invalid_input", 0)
               + stats.get("rejects_unknown_tracklet", 0)
               + stats.get("rejects_behind_camera", 0)
               + stats.get("rejects_var_invalid", 0)
               + stats.get("rejects_rho_clamp", 0))
    print(f"          lifter stats: inits={stats.get('inits', 0)} "
          f"updates={stats.get('updates', 0)} "
          f"rejects_total={rejects} (thr<={REJECTS_MAX})")
    if rejects > REJECTS_MAX:
        fails.append(f"rejects_total={rejects} > {REJECTS_MAX}")

    if fails:
        print("\nFAIL — " + ";  ".join(fails))
        _print_tail(JOURNAL, 14, "journal")
        _print_tail(MISSION_LOG, 30, "mission.log")
        return 1
    print("\nPASS — L5 lifter converges under Gazebo cadence")
    return 0


if __name__ == "__main__":
    sys.exit(main())

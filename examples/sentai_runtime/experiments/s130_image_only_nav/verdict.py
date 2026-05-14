#!/usr/bin/env python3
"""verdict.py — s130 4.5Baseline (image-only world-model navigation).

PASS criteria:
  L4-side FSM (same as s128/s129):
    1. mission `_status` == "OK"
    2. all servo fault counters == 0
    3. last action == DISARM, last_result == 0
    4. final armed==0, flight==GROUND

  s130-specific:
    5. 4/4 markers IBVS-converged with px_dist < MARKER_PIXEL_TIGHT_PX
    6. image-vs-cf2 position agreement: median XY error < 5 cm,
       max < 10 cm
    7. image-vs-cf2 yaw agreement: median |Δyaw| < 3°, max < 5°
    8. n_localize_failures == 0

On FAIL prints journal tail + mission tail + transcript tail.
"""
from __future__ import annotations
import json
import math
import sys
from pathlib import Path

WORKDIR     = Path("/tmp/s130_image_only_nav")
SUMMARY     = WORKDIR / "summary.json"
TRANSCRIPT  = WORKDIR / "repl.transcript"
MISSION_LOG = WORKDIR / "mission.log"
JOURNAL     = WORKDIR / "journal.txt"
IBVS_LOG    = WORKDIR / "ibvs_iter_log.json"
IMG_VS_CF2  = WORKDIR / "image_vs_cf2.json"

ACT_NONE, ACT_INIT, ACT_ARM, ACT_DISARM = 0, 1, 2, 3
ACT_TAKEOFF, ACT_MOVE, ACT_HOVER, ACT_LAND = 4, 5, 6, 7
BACKEND_SIM = 1
FLIGHT_GROUND = 0

MARKER_PIXEL_TIGHT_PX = 30
N_MARKERS_EXPECTED    = 4
IMG_POS_MEDIAN_THR_M  = 0.05
IMG_POS_MAX_THR_M     = 0.10
IMG_YAW_MEDIAN_THR_D  = 3.0
IMG_YAW_MAX_THR_D     = 5.0


def _print_tail(path: Path, n: int, label: str) -> None:
    if not path.is_file():
        print(f"[verdict] (no {label} at {path})")
        return
    lines = path.read_text(errors="replace").splitlines()
    print(f"\n── Last {min(n, len(lines))} lines of {path} ──")
    for ln in lines[-n:]:
        print(f"  | {ln}")
    print("──")


def _wrap_deg(d: float) -> float:
    """Wrap a degree value into [-180, 180]."""
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def _median(xs: list[float]) -> float:
    if not xs:
        return float("nan")
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _max(xs: list[float]) -> float:
    return max(xs) if xs else float("nan")


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
    if faults_oob != 0 or faults_no_bk != 0 or faults_not_armed != 0:
        fails.append("non-zero fault counters")
    if last_action != ACT_DISARM or last_result != 0:
        fails.append(f"last action != DISARM/0  (got {last_action}/{last_result})")
    if armed != 0 or flight != FLIGHT_GROUND:
        fails.append(f"final armed/flight = {armed}/{flight} (expected 0/GROUND)")

    # Per-marker IBVS results
    per_marker = d.get("per_marker", [])
    n_converged = d.get("n_markers_converged", 0)
    n_total     = d.get("n_markers_total", N_MARKERS_EXPECTED)
    print(f"          IBVS tour: {n_converged}/{n_total} markers converged")
    for m in per_marker:
        mid = m.get("id")
        ibvs = m.get("ibvs", {})
        outer = m.get("outer", {})
        ok = ibvs.get("converged", False)
        n_ibvs_it = ibvs.get("n_iters", -1)
        n_outer_it = outer.get("n_iters", -1)
        n_loc_fail = outer.get("n_localize_fail", 0)
        perr = m.get("final_pixel_err", ibvs.get("last_pixel_err", float("inf")))
        cfp = m.get("cf2_at_converge")
        pix = m.get("final_pixel")
        tag = "OK " if ok and perr < MARKER_PIXEL_TIGHT_PX else "BAD"
        cf2s = (f"  cf2=({cfp[0]:+.2f},{cfp[1]:+.2f},{cfp[2]:.2f})"
                if cfp else "")
        pxs  = (f"  pixel=({pix[0]:.0f},{pix[1]:.0f})" if pix else "")
        print(f"            [{tag}] id{mid}: outer_iters={n_outer_it} "
              f"loc_fail={n_loc_fail}  IBVS converged={ok} "
              f"iters={n_ibvs_it}  px_err={perr:.1f}{pxs}{cf2s}")
        if not ok:
            fails.append(f"id{mid}: IBVS did not converge "
                         f"(n_iters={n_ibvs_it}, last_pixel_err={perr:.1f})")
        elif perr >= MARKER_PIXEL_TIGHT_PX:
            fails.append(f"id{mid}: final px_err {perr:.1f} >= "
                         f"{MARKER_PIXEL_TIGHT_PX} px threshold")
    if n_converged != n_total:
        fails.append(f"only {n_converged}/{n_total} markers converged")

    # Localize failures (outer loop)
    n_loc_fail_total = d.get("n_localize_failures", -1)
    print(f"          n_localize_failures = {n_loc_fail_total}")
    if n_loc_fail_total != 0:
        fails.append(f"n_localize_failures={n_loc_fail_total} != 0")

    # Image vs cf2 agreement (the s130-specific metric)
    image_vs_cf2 = d.get("image_vs_cf2", [])
    if not image_vs_cf2:
        fails.append("no image_vs_cf2 samples (outer loop never ran?)")
    else:
        pos_errs: list[float] = []
        yaw_errs: list[float] = []
        for row in image_vs_cf2:
            ix, iy, _iz = row["image"][:3]
            cx, cy, _cz = row["cf2"]
            pos_errs.append(math.hypot(ix - cx, iy - cy))
            yaw_errs.append(abs(_wrap_deg(
                row["image_yaw_deg"] - row["cf2_yaw_deg"]
            )))
        pos_med = _median(pos_errs)
        pos_max = _max(pos_errs)
        yaw_med = _median(yaw_errs)
        yaw_max = _max(yaw_errs)
        print(f"          image vs cf2 over {len(image_vs_cf2)} samples:")
        print(f"            pos err: median={pos_med*100:.1f} cm  "
              f"max={pos_max*100:.1f} cm  "
              f"(thr median<{IMG_POS_MEDIAN_THR_M*100:.0f} max<{IMG_POS_MAX_THR_M*100:.0f})")
        print(f"            yaw err: median={yaw_med:.2f}°  max={yaw_max:.2f}°  "
              f"(thr median<{IMG_YAW_MEDIAN_THR_D} max<{IMG_YAW_MAX_THR_D})")
        if pos_med >= IMG_POS_MEDIAN_THR_M:
            fails.append(f"image-vs-cf2 pos median {pos_med*100:.1f} cm "
                         f">= {IMG_POS_MEDIAN_THR_M*100:.0f} cm")
        if pos_max >= IMG_POS_MAX_THR_M:
            fails.append(f"image-vs-cf2 pos max {pos_max*100:.1f} cm "
                         f">= {IMG_POS_MAX_THR_M*100:.0f} cm")
        if yaw_med >= IMG_YAW_MEDIAN_THR_D:
            fails.append(f"image-vs-cf2 yaw median {yaw_med:.2f}° "
                         f">= {IMG_YAW_MEDIAN_THR_D}°")
        if yaw_max >= IMG_YAW_MAX_THR_D:
            fails.append(f"image-vs-cf2 yaw max {yaw_max:.2f}° "
                         f">= {IMG_YAW_MAX_THR_D}°")

    print(f"          cf2 yaw post-takeoff = {d.get('cf2_yaw_deg_post_takeoff','?')}°")

    if fails:
        print("\nFAIL — " + ";  ".join(fails))
        _print_tail(JOURNAL, 14, "journal")
        _print_tail(MISSION_LOG, 30, "mission.log")
        return 1
    print("\nPASS — 4.5Baseline: image-only world-model navigation")
    return 0


if __name__ == "__main__":
    sys.exit(main())

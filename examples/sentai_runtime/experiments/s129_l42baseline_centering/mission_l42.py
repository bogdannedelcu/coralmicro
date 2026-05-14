"""s129 L4.2Baseline mission — visual servoing (PBVS via PnP tvec).

Drone navigates UNTIL the target ArUco marker is centred in the
downward camera image, using ONLY camera feedback (no world coords in
the inner loop).  See README.md for the control law derivation.

Reuses harness components from s128 L4.1Baseline:
    ReplDriver, StepLog, j_int helper, sentai.sim.journal API,
    cflib MotionCommander + flow_forwarder thread, telemetry capture.

Outputs (under /tmp/s129_l42baseline/):
    summary.json          — verdict reads this
    journal.txt           — REPL-side structured journal
    ibvs_iter_log.json    — per-iteration tvec / body_delta / pixel_err
    mission.log           — host-side step trace
    repl.transcript       — raw stdin/stdout dump
    cf2_telemetry.json    — stateEstimate samples
"""
from __future__ import annotations

import ast
import contextlib
import datetime as dt
import json
import math
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

S091_DIR = Path(__file__).resolve().parent.parent / "s091_aruco_lowalt"
S090_DIR = Path(__file__).resolve().parent.parent / "s090_hover_over_cat"
S128_DIR = Path(__file__).resolve().parent.parent / "s128_l41baseline_seeded"
sys.path.insert(0, str(S091_DIR))
sys.path.insert(0, str(S090_DIR))
sys.path.insert(0, str(S128_DIR))
import aruco_hover                                         # noqa: E402
from aruco_detector import (                               # noqa: E402
    detect_in_ppm, latest_ppm, CAM_CX, CAM_CY, CAM_FX, CAM_FY,
)
# Reuse the proven ReplDriver + StepLog from s128 (single source of truth).
from mission_l41 import (                                  # noqa: E402
    ReplDriver, StepLog,
)

WORKDIR = Path("/tmp/s129_l42baseline")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
TRACE_JSON      = WORKDIR / "servo_trace.json"
STATUS_JSON     = WORKDIR / "servo_status.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
IBVS_LOG_JSON   = WORKDIR / "ibvs_iter_log.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

JOURNAL_NAME      = "s129_journal.txt"
JOURNAL_HOST_PATH = lambda: SENTAI_FS_ROOT / JOURNAL_NAME    # noqa: E731

# ─── Targets ArUco ids visited in order ───────────────────────────────
# Each is centred via IBVS independently; between markers the IBVS loop
# sees the new target off-centre and drives toward it.  No world coords
# in the inner loop — the drone navigates purely on PnP tvec for the
# CURRENT target id.
#
# At z=1 m hover the camera FOV (1.11 × 0.83 m) easily contains the
# whole compact ±0.15×±0.10 marker pattern from any position over it,
# so the next target is always visible without exploration.
TARGET_ARUCO_IDS = [0, 1, 2, 3]   # NE → NW → SW → SE

# Approximate marker world XY (from sentai_crazysim.sdf).  Used ONLY
# for a coarse pre-IBVS hop — once cf2 is within ~10 cm, IBVS takes
# over and drives to pixel-tight centering.  Without this hop, the
# next marker often lands at the image edge where ArUco detection is
# unreliable (id3 not detected from id2's converged position in our
# 2026-05-14 first multi-marker test).
APPROACH_WORLD_XY = {
    0: (+0.15, +0.10),
    1: (-0.15, +0.10),
    2: (-0.15, -0.10),
    3: (+0.15, -0.10),
}
APPROACH_FRACTION = 0.7      # move 70% of the way; IBVS finishes the rest

# Flight params
HOVER_Z_M         = 1.00
TAKEOFF_Z_M       = 1.00
TAKEOFF_VEL_MPS   = 0.6
IBVS_VEL_MPS      = 0.20    # gentle move per IBVS iteration

# Inner-loop tolerances
NAV_TOL_M             = 0.03  # PnP-derived body residual; converge band
NAV_MIN_STEP_M        = 0.015 # ignore micro-corrections (PID dead-band)
NAV_SETTLE_S          = 0.7
NAV_MAX_ITERS         = 12   # damping (K=0.5) makes us need more iters
NAV_TIMEOUT_S         = 16.0
MARKER_PIXEL_TIGHT_PX = 30    # pass threshold on final centred frame
N_VISUAL_SAMPLES      = 5     # frames averaged for final pixel-error

# Camera-to-body rotation (from aruco_detector.py — empirically derived
# in s092).  cam_X = -body_Y, cam_Y = -body_X, cam_Z = -body_Z (down).
# So body_dx = +tvec[1], body_dy = +tvec[0] (initial guess; signs may
# need flipping per first-run observation).
SIGN_BODY_DX_FROM_TVEC_Y = +1.0
SIGN_BODY_DY_FROM_TVEC_X = +1.0
# Proportional gain on body_delta per iter — < 1 damps overshoot.
# First run with K=1.0 produced 4-iter convergence on single marker but
# oscillation on multi-marker tour (id3 went 29 → 28 → 33 → 53 → 22 px).
# K=0.5 takes ~2× more iters but no overshoot.
IBVS_GAIN_K              = 0.5

# ─────────────────────────────────────────────────────────────────
# cf2 telemetry capture (same as s128)
# ─────────────────────────────────────────────────────────────────
_tel_lock = threading.Lock()
_tel_last = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
_tel_samples: list[dict] = []


def _tel_cb(_ts, data, _lc):
    with _tel_lock:
        _tel_last["x"]       = data["stateEstimate.x"]
        _tel_last["y"]       = data["stateEstimate.y"]
        _tel_last["z"]       = data["stateEstimate.z"]
        _tel_last["yaw_deg"] = data["stateEstimate.yaw"]
        _tel_samples.append({
            "t": time.monotonic(),
            "x": _tel_last["x"], "y": _tel_last["y"], "z": _tel_last["z"],
            "yaw_deg": _tel_last["yaw_deg"],
        })


def tel_snapshot() -> dict:
    with _tel_lock:
        return dict(_tel_last)


# ─────────────────────────────────────────────────────────────────
# Read latest unique PPM with retry — frame_NNNNNN sequence
# ─────────────────────────────────────────────────────────────────
def wait_for_fresh_ppm(frames_dir: Path, last_seq: int,
                        deadline_s: float) -> tuple[Path, int] | None:
    deadline = time.monotonic() + deadline_s
    while time.monotonic() < deadline:
        ppm = latest_ppm(frames_dir)
        if ppm is None:
            time.sleep(0.05)
            continue
        try:
            seq = int(ppm.stem.replace("frame_", ""))
        except ValueError:
            time.sleep(0.05)
            continue
        if seq > last_seq:
            return ppm, seq
        time.sleep(0.05)
    return None


# ─────────────────────────────────────────────────────────────────
# IBVS / PBVS inner loop
# ─────────────────────────────────────────────────────────────────
def ibvs_center_on_marker(log: StepLog, repl: ReplDriver, mc,
                           j_int, frames_dir: Path,
                           target_id: int,
                           iter_log: list) -> dict:
    """Iterate: detect → compute body delta from tvec → move → settle.
    Returns dict with `converged`, `n_iters`, `last_pixel_err`."""
    last_seq = -1
    t_start = time.monotonic()
    n_iters = 0
    last_body_delta = (0.0, 0.0)
    last_pixel_err = float("inf")

    for it in range(NAV_MAX_ITERS):
        if (time.monotonic() - t_start) > NAV_TIMEOUT_S:
            log.info(f"        ibvs iter{it}: TIMEOUT "
                     f"({NAV_TIMEOUT_S}s exhausted)")
            break

        # 1) wait for a fresh frame
        got = wait_for_fresh_ppm(frames_dir, last_seq, deadline_s=3.0)
        if got is None:
            log.info(f"        ibvs iter{it}: no fresh PPM in 3 s — abort")
            break
        ppm, last_seq = got

        # 2) detect + estimate pose
        try:
            dets = detect_in_ppm(ppm, estimate_pose=True)
        except Exception as e:
            log.info(f"        ibvs iter{it}: detect failed {e}")
            continue
        if target_id not in dets:
            log.info(f"        ibvs iter{it}: id{target_id} NOT in "
                     f"frame  (saw {sorted(dets.keys())})")
            continue
        m = dets[target_id]
        tvec = m.tvec.tolist() if m.tvec is not None else None
        if tvec is None or not all(math.isfinite(v) for v in tvec):
            log.info(f"        ibvs iter{it}: invalid tvec {tvec}")
            continue

        # 3) compute body-frame delta + pixel error
        # Apply proportional gain K<1 to damp IBVS overshoot.  cf2 PID
        # already adds its own settling dynamic; commanding the full
        # delta each iter creates a double-amplitude oscillation.
        body_dx = SIGN_BODY_DX_FROM_TVEC_Y * tvec[1] * IBVS_GAIN_K
        body_dy = SIGN_BODY_DY_FROM_TVEC_X * tvec[0] * IBVS_GAIN_K
        pixel_err = math.hypot(m.cx - CAM_CX, m.cy - CAM_CY)
        last_body_delta = (body_dx, body_dy)
        last_pixel_err = pixel_err

        iter_log.append({
            "iter": it,
            "t_s": time.monotonic() - t_start,
            "ppm": ppm.name,
            "tvec": tvec,
            "pixel": [float(m.cx), float(m.cy)],
            "pixel_err": pixel_err,
            "body_delta": [body_dx, body_dy],
        })
        log.info(f"        ibvs iter{it}: tvec=({tvec[0]:+.3f},"
                 f"{tvec[1]:+.3f},{tvec[2]:+.3f})  "
                 f"px=({m.cx:.0f},{m.cy:.0f}) err={pixel_err:.1f} "
                 f"body=({body_dx:+.3f},{body_dy:+.3f})")

        # 4) convergence?
        if pixel_err < MARKER_PIXEL_TIGHT_PX and \
           max(abs(body_dx), abs(body_dy)) < NAV_TOL_M:
            log.info(f"        ibvs iter{it}: CONVERGED  px_err="
                     f"{pixel_err:.1f} < {MARKER_PIXEL_TIGHT_PX}")
            return {
                "converged": True,
                "n_iters": it + 1,
                "last_pixel_err": pixel_err,
                "last_body_delta": list(last_body_delta),
            }

        # 5) clamp tiny corrections + issue move
        sx = body_dx if abs(body_dx) >= NAV_MIN_STEP_M else 0.0
        sy = body_dy if abs(body_dy) >= NAV_MIN_STEP_M else 0.0
        if sx == 0.0 and sy == 0.0:
            log.info(f"        ibvs iter{it}: both deltas under "
                     f"NAV_MIN_STEP_M; staying put")
        else:
            rc = j_int(f"servo_move_ibvs_id{target_id}_it{it}",
                       f"sentai.servo.move({sx}, {sy}, 0)")
            if rc != 0:
                raise RuntimeError(f"servo.move ibvs it{it} rc={rc}")
            mc.move_distance(sx, sy, 0.0, velocity=IBVS_VEL_MPS)
        time.sleep(NAV_SETTLE_S)
        n_iters = it + 1

    return {
        "converged": False,
        "n_iters": n_iters,
        "last_pixel_err": last_pixel_err,
        "last_body_delta": list(last_body_delta),
    }


# ─────────────────────────────────────────────────────────────────
# Mission
# ─────────────────────────────────────────────────────────────────
def fly(log: StepLog, repl: ReplDriver) -> dict:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    summary: dict = {}

    with log.step("cf2 link + Kalman reset + damp params"):
        sync = SyncCrazyflie("udp://127.0.0.1:19850",
                             cf=Crazyflie(rw_cache=None))
        sync.open_link()
        cf = sync.cf
        cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(0.5)
        for k, v in {
            "posCtlPid.xyKd":     0.5,
            "velCtlPid.vxKd":     0.05,
            "velCtlPid.vyKd":     0.05,
            "posCtlPid.xVelMax":  2.5,
            "posCtlPid.yVelMax":  2.5,
            "posCtlPid.xKp":      3.0,
            "posCtlPid.yKp":      3.0,
        }.items():
            try:
                cf.param.set_value(k, v)
            except Exception:
                pass
        time.sleep(0.3)
        cf.param.set_value("kalman.resetEstimation", 1)
        time.sleep(0.5)
        cf.param.set_value("kalman.resetEstimation", 0)
        time.sleep(2.0)

    with log.step("telemetry log @ 20 ms"):
        lc = LogConfig(name="att", period_in_ms=20)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                  "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
            lc.add_variable(v, "float")
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(_tel_cb)
        lc.start()

    with log.step("flow forwarder thread"):
        stop_evt = threading.Event()
        flow_stats = {"n_sent": 0, "fatal": None, "last_err": None}
        flow_th = threading.Thread(target=aruco_hover.flow_forwarder,
                                    args=(stop_evt, cf, flow_stats),
                                    daemon=True)
        flow_th.start()
        time.sleep(2.0)
        if flow_stats["fatal"]:
            raise RuntimeError(f"flow forwarder fatal: {flow_stats['fatal']}")
        log.info(f"        flow forwarder running (n_sent={flow_stats['n_sent']})")

    # journal helper (mirrors s128)
    def j_int(label: str, cmd: str) -> int:
        rc = repl.exec_int(cmd)
        repl.exec_int(f"sentai.sim.journal_write('{label}', "
                       f"sentai.servo.status())")
        return rc

    with log.step("REPL: import + verbose(0) + journal_open"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        if repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')") != 0:
            raise RuntimeError("journal_open failed")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")

    with log.step("REPL: servo.init + arm + takeoff (intent)"):
        if j_int("servo_init", "sentai.servo.init('sim')") != 0:
            raise RuntimeError("servo.init failed")
        repl.exec_int("sentai.servo.clear_trace()")
        if j_int("servo_arm", "sentai.servo.arm()") != 0:
            raise RuntimeError("servo.arm failed")
        if j_int("servo_takeoff", f"sentai.servo.takeoff({TAKEOFF_Z_M})") != 0:
            raise RuntimeError("servo.takeoff failed")

    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(2.5)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        log.info(f"        post-takeoff cf2 @ ({tel['x']:+.2f},{tel['y']:+.2f},"
                 f"{tel['z']:.2f})  yaw={tel['yaw_deg']:+.1f}°  "
                 f"flow_n={flow_stats['n_sent']}")
        summary["cf2_post_takeoff"] = [tel["x"], tel["y"], tel["z"]]
        summary["cf2_yaw_deg"] = tel["yaw_deg"]

    # ─── Multi-marker IBVS loop ───
    # For each target in sequence, IBVS-centre on it.  Between markers,
    # the new target appears off-centre and the loop drives toward it.
    # No world coords used in the inner loop — pure visual chain.
    all_iter_logs: list = []
    per_marker: list = []
    with log.step(f"IBVS visit {len(TARGET_ARUCO_IDS)} markers"):
        for target_id in TARGET_ARUCO_IDS:
            # ─── Coarse pre-IBVS hop (world-coord, 70% of the way) ──
            # Ensures the target marker is well within FOV (not at edge)
            # before IBVS takes over.  Without this, after centering on
            # id2 (SW) the next marker id3 (SE) lands at cy≈10 (image
            # top) where ArUco detection fails.
            tel = tel_snapshot()
            wx, wy = APPROACH_WORLD_XY[target_id]
            dx_approach = (wx - tel["x"]) * APPROACH_FRACTION
            dy_approach = (wy - tel["y"]) * APPROACH_FRACTION
            if abs(dx_approach) >= 0.05 or abs(dy_approach) >= 0.05:
                with log.step(f"coarse hop toward id{target_id} "
                              f"({dx_approach:+.2f},{dy_approach:+.2f})"):
                    j_int(f"servo_move_approach_id{target_id}",
                           f"sentai.servo.move({dx_approach}, {dy_approach}, 0)")
                    mc.move_distance(dx_approach, dy_approach, 0.0,
                                      velocity=IBVS_VEL_MPS)
                    time.sleep(NAV_SETTLE_S)
            with log.step(f"IBVS centre on id{target_id} "
                          f"[max {NAV_MAX_ITERS} iters, "
                          f"{NAV_TIMEOUT_S}s timeout]"):
                iter_log: list = []
                result = ibvs_center_on_marker(log, repl, mc, j_int,
                                                repl.frames_dir,
                                                target_id, iter_log)
                # Tag iterations with their owning target for plotting.
                for ent in iter_log:
                    ent["target_id"] = target_id
                all_iter_logs.extend(iter_log)
                log.info(f"        id{target_id} result: {result}")
                # Record converged-iter pixel as the canonical "final"
                # for this marker (avoid post-settle drift, see 2026-05-14
                # trap in [[s129-l42baseline-shipped]]).
                m_record: dict = {"id": target_id, **result}
                if iter_log and result.get("converged"):
                    last = iter_log[-1]
                    tel = tel_snapshot()
                    m_record.update({
                        "final_pixel":     last["pixel"],
                        "final_pixel_err": last["pixel_err"],
                        "cf2_at_converge": [tel["x"], tel["y"], tel["z"]],
                    })
                per_marker.append(m_record)
                # Hover record between markers (intent-only; cf2 holds
                # via the implicit hover from the last move_distance).
                if j_int(f"servo_hover_id{target_id}",
                          "sentai.servo.hover()") != 0:
                    raise RuntimeError(
                        f"servo.hover after id{target_id} failed")

    summary["per_marker"] = per_marker
    # Aggregate stats for verdict.
    n_converged = sum(1 for m in per_marker if m.get("converged"))
    summary["n_markers_converged"] = n_converged
    summary["n_markers_total"]     = len(TARGET_ARUCO_IDS)
    log.info(f"        summary: {n_converged}/{len(TARGET_ARUCO_IDS)} markers converged")

    # ─── Land + tear down ───
    with log.step("REPL: servo.land + disarm"):
        if j_int("servo_land", "sentai.servo.land()") != 0:
            raise RuntimeError("servo.land failed")
        if j_int("servo_disarm", "sentai.servo.disarm()") != 0:
            raise RuntimeError("servo.disarm failed")

    with log.step("cf2 land"):
        mc.land(velocity=0.3)
        time.sleep(2.0)

    with log.step("teardown telemetry + cf2 link"):
        stop_evt.set()
        flow_th.join(timeout=1.5)
        lc.stop()
        sync.close_link()
        summary["flow_n_sent"] = flow_stats["n_sent"]

    with log.step("REPL: dump servo.status / trace via fs.write"):
        repl.exec_int("sentai.sim.journal_write('mission_dump_begin', None)")
        # exec_via_fs lives on the ReplDriver from s128.
        status   = repl.exec_via_fs("sentai.servo.status()", SENTAI_FS_ROOT)
        trace    = repl.exec_via_fs("sentai.servo.trace()",  SENTAI_FS_ROOT)
        STATUS_JSON.write_text(json.dumps(status, indent=2))
        TRACE_JSON.write_text(json.dumps(trace,  indent=2))
        summary["servo_status"] = status
        summary["servo_trace"]  = trace
        repl.exec_int("sentai.sim.journal_write('mission_end', None)")
        repl.exec_int("sentai.sim.journal_close()")

    # Surface journal + iter log
    journal = JOURNAL_HOST_PATH()
    if journal.is_file():
        (WORKDIR / "journal.txt").write_text(journal.read_text())
    IBVS_LOG_JSON.write_text(json.dumps(all_iter_logs, indent=2))

    return summary


def main() -> int:
    log = StepLog(MISSION_LOG)
    log.info(f"REPO_ROOT={REPO_ROOT}")
    if not SENTAI_SIM_BIN.is_file():
        log.info("FATAL — sentai_sim binary missing")
        return 1

    # Spawn sentai_sim with frames_dir under our WORKDIR.
    repl = ReplDriver(SENTAI_SIM_BIN, SENTAI_FS_ROOT, REPL_TRANSCRIPT)
    try:
        summary = fly(log, repl)
    except BaseException as e:
        log.info(f"MISSION FAILED: {type(e).__name__}: {e}")
        with contextlib.suppress(Exception):
            st = repl.exec_via_fs("sentai.servo.status()", SENTAI_FS_ROOT)
            STATUS_JSON.write_text(json.dumps(st, indent=2))
        with contextlib.suppress(Exception):
            tr = repl.exec_via_fs("sentai.servo.trace()", SENTAI_FS_ROOT)
            TRACE_JSON.write_text(json.dumps(tr, indent=2))
        with contextlib.suppress(Exception):
            repl.exec_int("sentai.sim.journal_write('mission_crash', None)")
            repl.exec_int("sentai.sim.journal_close()")
        journal = JOURNAL_HOST_PATH()
        if journal.is_file():
            (WORKDIR / "journal.txt").write_text(journal.read_text())
        SUMMARY_JSON.write_text(json.dumps({
            "_status": "FAIL",
            "_exception": f"{type(e).__name__}: {e}",
            "_last_run": dt.datetime.now().isoformat(),
        }, indent=2))
        repl.close()
        log.close()
        return 1

    with _tel_lock:
        TELEMETRY_JSON.write_text(json.dumps(_tel_samples))
    summary["_status"]   = "OK"
    summary["_last_run"] = dt.datetime.now().isoformat()
    SUMMARY_JSON.write_text(json.dumps(summary, indent=2))
    log.info("MISSION OK")
    repl.close()
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

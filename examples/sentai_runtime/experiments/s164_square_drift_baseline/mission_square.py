"""s164 — 1m square drift baseline (OP-S10-W11-T5.A).

Pure control-stack characterisation: fly a 1m × 1m horizontal square,
land at origin, measure drift.  SlamTask NOT started — this isolates
cf2 EKF + sentai.flow + ArUco-anchor PnP from any W11 perception
side-effects.

Trajectory (ENU world frame, takeoff at origin):
  corner_0 (0, 0)  ←  takeoff/return
  corner_1 (+1, 0)
  corner_2 (+1, +1)
  corner_3 (0, +1)
  corner_0 (0, 0)  ←  closure

Per-corner captures:
  - cf2 EKF telemetry pose (controller belief)
  - sentai.aruco.detect_from_camera() — number of markers visible
    and their world-frame x/y via PnP (independent of cf2 EKF)
  - dwell duration

Pass criteria (informational baseline, no hard gate):
  - mission completes without runtime error
  - corners 1-3 visit attempted (each goto_xy_abs returns within tol)
  - landing pose recorded
Hard gate:
  - landing pose ≤ 15 cm from PHYSICAL_ORIGIN (per [[sim-test-must-
    return-home]] universal rule)

Outputs under /tmp/s164_square_drift_baseline/:
  - summary.json     : per-corner metrics + closure
  - mission.log      : human-readable step log
  - repl.transcript  : raw REPL i/o
  - cf2_telemetry.json : raw 50 Hz pose stream
"""
from __future__ import annotations

import datetime as dt
import json
import math
import sys
import threading
import time
from pathlib import Path

S091_DIR = Path(__file__).resolve().parent.parent / "s091_aruco_lowalt"
S128_DIR = Path(__file__).resolve().parent.parent / "s128_l41baseline_seeded"
S132_DIR = Path(__file__).resolve().parent.parent / "s132_lifter_gazebo"
for p in (S091_DIR, S128_DIR, S132_DIR):
    sys.path.insert(0, str(p))

import aruco_hover                                       # noqa: E402
from mission_l41 import ReplDriver, StepLog              # noqa: E402
from mission_lifter import _tel_cb, tel_snapshot, _tel_samples  # noqa: E402

WORKDIR = Path("/tmp/s164_square_drift_baseline")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

TAKEOFF_Z_M      = 1.50
TAKEOFF_VEL_MPS  = 0.6
DWELL_S          = 2.0    # hover at each corner
GOTO_TIMEOUT_S   = 20.0
GOTO_TOL_M       = 0.08
LAND_DWELL_S     = 2.5

# Corners as ABSOLUTE world positions (origin = takeoff).
CORNERS = [
    ("c1_fwd",  +1.0,  0.0),    # 1 m forward
    ("c2_left", +1.0, +1.0),    # 1 m left
    ("c3_back",  0.0, +1.0),    # 1 m back
    ("c0_home",  0.0,  0.0),    # return
]


def goto_xy_abs(cf, tx, ty, tz, log, label):
    """Closed-loop goto.  Returns (arrived, last_pose, elapsed_s)."""
    t0 = time.monotonic()
    deadline = t0 + GOTO_TIMEOUT_S
    last = None
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(tx, ty, tz, 0.0)
        tel = tel_snapshot()
        last = (tel["x"], tel["y"], tel["z"])
        if math.hypot(last[0] - tx, last[1] - ty) <= GOTO_TOL_M:
            elapsed = time.monotonic() - t0
            log.info(f"        {label}: arrived ({last[0]:+.3f},{last[1]:+.3f},"
                     f"{last[2]:.3f}) in {elapsed:.2f}s")
            return True, last, elapsed
        time.sleep(0.05)
    return False, last, time.monotonic() - t0


def hold_position(cf, tx, ty, tz, dur_s):
    deadline = time.monotonic() + dur_s
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(tx, ty, tz, 0.0)
        time.sleep(0.05)


def aruco_snapshot(repl) -> dict:
    """Trigger one aruco.detect_from_camera() and return a dict.
    The binding returns a LIST of marker dicts (or None on no-frame).
    Always returns; on failure logs the exception in `error` field.
    """
    try:
        r = repl.exec_repr("sentai.aruco.detect_from_camera()")
    except Exception as e:
        return {"error": f"{type(e).__name__}: {e}", "n_markers": 0, "ids": []}
    if r is None:
        return {"error": "no_return_or_no_frame", "n_markers": 0, "ids": []}
    # The binding returns a top-level list of marker dicts.  Belt-and-
    # suspenders: also accept a {markers: [...]} dict shape in case the
    # API evolves later.
    if isinstance(r, dict):
        markers = r.get("markers") or []
    else:
        markers = list(r) if r else []
    return {
        "n_markers":   len(markers),
        "ids":         [int(m.get("marker_id", -1)) for m in markers],
        "detect_us":   int(markers[0]["detect_us"]) if markers else 0,
        "src_ts_ms":   int(markers[0]["src_ts_ms"]) if markers else 0,
        "reproj_px":   [float(m.get("reproj_err_px", 0.0)) for m in markers],
        "tvecs_cam":   [m.get("tvec_cam") for m in markers],
    }


def fly(log: StepLog, repl: ReplDriver) -> dict:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    summary: dict = {"corners": [list(c) for c in CORNERS]}
    t_start = time.monotonic()

    with log.step("cf2 link + params + Kalman reset"):
        sync = SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache=None))
        sync.open_link()
        cf = sync.cf
        cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(0.5)
        for k, v in {"posCtlPid.xVelMax": 3.0, "posCtlPid.yVelMax": 3.0,
                     "posCtlPid.xKp": 3.0, "posCtlPid.yKp": 3.0}.items():
            try: cf.param.set_value(k, v)
            except Exception: pass
        time.sleep(0.3)
        cf.param.set_value("kalman.resetEstimation", 1)
        time.sleep(0.5)
        cf.param.set_value("kalman.resetEstimation", 0)
        time.sleep(2.0)

    with log.step("telemetry + flow forwarder"):
        lc = LogConfig(name="att", period_in_ms=20)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                  "stateEstimate.roll", "stateEstimate.pitch",
                  "stateEstimate.yaw"):
            lc.add_variable(v, "float")
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(_tel_cb)
        lc.start()
        stop_evt = threading.Event()
        flow_stats = {"n_sent": 0, "fatal": None, "last_err": None}
        flow_th = threading.Thread(target=aruco_hover.flow_forwarder,
                                    args=(stop_evt, cf, flow_stats), daemon=True)
        flow_th.start()
        time.sleep(2.0)

    with log.step("REPL init (sentai.camera + sentai.aruco only — NO slam_task)"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        repl.exec_int("sentai.places.clear()")
        # Explicitly assert SLAM is NOT running — T5.A isolates the
        # control stack.  start_slam() would couple us to W11.
        is_running = repl.exec_int("sentai.places.slam_stats()['is_running']")
        log.info(f"        slam_stats.is_running = {is_running} (expect 0)")
        if is_running != 0:
            raise RuntimeError(
                f"T5.A invariant violated: SlamTask running at start "
                f"(is_running={is_running})")

    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        physical_origin = (tel["x"], tel["y"])
        summary["physical_origin"] = list(physical_origin)
        summary["takeoff_z_actual"] = tel["z"]
        log.info(f"        PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
                 f"{physical_origin[1]:+.3f}); z={tel['z']:.3f}")

    # Snapshot at origin BEFORE the square — baseline marker visibility.
    snap_origin = aruco_snapshot(repl)
    log.info(f"        origin aruco: n={snap_origin['n_markers']} "
             f"ids={snap_origin['ids']}")
    summary["snap_origin"] = snap_origin

    # ─── Square ────────────────────────────────────────────────────────
    corner_log = []
    for (label, dx, dy) in CORNERS:
        tx = physical_origin[0] + dx
        ty = physical_origin[1] + dy
        with log.step(f"goto {label} → ({tx:+.3f},{ty:+.3f},{TAKEOFF_Z_M:.3f})"):
            arrived, last_pose, elapsed = goto_xy_abs(
                cf, tx, ty, TAKEOFF_Z_M, log, label=label)
            hold_position(cf, tx, ty, TAKEOFF_Z_M, DWELL_S)
            tel_now = tel_snapshot()
            snap = aruco_snapshot(repl)
            err_xy = math.hypot(tel_now["x"] - tx, tel_now["y"] - ty)
            log.info(f"        post-dwell pose=({tel_now['x']:+.3f},"
                     f"{tel_now['y']:+.3f},{tel_now['z']:.3f}); "
                     f"err_xy={err_xy*100:.2f}cm; "
                     f"aruco n={snap['n_markers']} ids={snap['ids']}")
            corner_log.append({
                "label":       label,
                "cmd":         [tx, ty, TAKEOFF_Z_M],
                "arrived":     arrived,
                "goto_elapsed_s": elapsed,
                "post_dwell_pose": [tel_now["x"], tel_now["y"], tel_now["z"]],
                "cmd_err_xy_cm": err_xy * 100.0,
                "aruco":       snap,
            })
    summary["corners_log"] = corner_log

    # ─── Land at origin ─────────────────────────────────────────────────
    with log.step("controlled descent at PHYSICAL_ORIGIN"):
        t0 = time.monotonic()
        descent_dur = 3.0
        while True:
            te = time.monotonic() - t0
            frac = min(1.0, te / descent_dur)
            z = TAKEOFF_Z_M + frac * (0.05 - TAKEOFF_Z_M)
            cf.commander.send_position_setpoint(
                physical_origin[0], physical_origin[1], z, 0.0)
            tel = tel_snapshot()
            if frac >= 1.0 and tel["z"] < 0.10: break
            if te > descent_dur + 4: break
            time.sleep(0.05)
        tel = tel_snapshot()
        land_pose = [tel["x"], tel["y"], tel["z"]]
        summary["land_pose"] = land_pose
        cf.commander.send_stop_setpoint()
        time.sleep(LAND_DWELL_S)

    land_err = math.hypot(land_pose[0] - physical_origin[0],
                           land_pose[1] - physical_origin[1])
    summary["land_err_vs_origin_m"]  = land_err
    summary["land_err_vs_origin_cm"] = land_err * 100.0
    summary["mission_duration_s"]    = time.monotonic() - t_start
    log.info(f"        land_err vs origin = {land_err*100:.2f} cm")
    log.info(f"        mission_duration   = {summary['mission_duration_s']:.1f} s")

    stop_evt.set()
    flow_th.join(timeout=2.0)
    sync.close_link()
    return summary


def main() -> int:
    log = StepLog(MISSION_LOG)
    try:
        with log.step("spawn sentai_sim + REPL"):
            repl = ReplDriver(
                bin_path=SENTAI_SIM_BIN, fs_root=SENTAI_FS_ROOT,
                transcript=REPL_TRANSCRIPT, startup_timeout_s=15.0,
                frames_dir_base=WORKDIR,
            )
        try:
            summary = fly(log, repl)
            summary["_status"] = "OK"
        finally:
            repl.close()
    except BaseException as e:
        log.info(f"FATAL: {type(e).__name__}: {e}")
        SUMMARY_JSON.write_text(json.dumps({
            "_status": "FAIL", "_exception": f"{type(e).__name__}: {e}",
            "_last_run": dt.datetime.now().isoformat(),
        }, indent=2, default=str))
        log.close()
        return 1
    summary["_last_run"] = dt.datetime.now().isoformat()
    SUMMARY_JSON.write_text(json.dumps(summary, indent=2, default=str))
    TELEMETRY_JSON.write_text(json.dumps(_tel_samples, indent=2))
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

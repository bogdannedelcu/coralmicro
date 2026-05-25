"""s165 — 1m square drift, EKF-vs-GT comparison (OP-S10-W11-T5.A v2).

Same trajectory as s164 but the run.sh harness records Gazebo ground
truth in parallel via gt_recorder.py.  Verdict overlays cf2 EKF
belief vs GT and emits a comparison plot.

Anti-cheat: GT consumed HOST-SIDE only, never crosses into
sentai_sim / MicroPython.  Mission itself sees zero GT — same code
path as s164.

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

Outputs under /tmp/s165_square_drift_gt/:
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

WORKDIR = Path("/tmp/s165_square_drift_gt")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT  = WORKDIR / "repl.transcript"
MISSION_LOG      = WORKDIR / "mission.log"
TELEMETRY_JSON   = WORKDIR / "cf2_telemetry.json"
ATTITUDE_JSON    = WORKDIR / "cf2_attitude.json"   # roll/pitch/yaw stream
SUMMARY_JSON     = WORKDIR / "summary.json"
JOURNAL_NAME     = "s165_journal.txt"               # written by sentai.sim.journal_*

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

TAKEOFF_Z_M      = 1.50
TAKEOFF_VEL_MPS  = 0.3   # gentler takeoff — was 0.6
DWELL_S          = 2.0    # hover at each corner
GOTO_TIMEOUT_S   = 30.0   # longer because we lowered vel
GOTO_TOL_M       = 0.08
LAND_DWELL_S     = 2.5

# Reduced cf2 lateral velocity + Kp → smaller pitch excursions
# → camera stays closer to true 90° downward.  Operator-asked
# 2026-05-17 after observing "shakes f mult" in the s164 trial.
# Defaults (s142) were xVelMax=3.0, xKp=3.0 — too aggressive at this
# scale.  s127 FlowBaseline uses cf2 firmware defaults (xVelMax=1.0).
CF2_VEL_MAX_MPS  = 0.8
CF2_KP           = 1.5

# Corners as ABSOLUTE world positions (origin = takeoff).
CORNERS = [
    ("c1_fwd",  +1.0,  0.0),    # 1 m forward
    ("c2_left", +1.0, +1.0),    # 1 m left
    ("c3_back",  0.0, +1.0),    # 1 m back
    ("c0_home",  0.0,  0.0),    # return
]

# ─── Attitude stream (roll/pitch/yaw) — separate from the imported
# _tel_samples in mission_lifter, so we don't have to touch shared
# code.  Used to verify whether the camera lost its 90° downward
# orientation during the 1m legs.  Populated by _att_cb on the same
# LogConfig that produces stateEstimate.{x,y,z}.
import threading as _th
_att_lock = _th.Lock()
_att_samples: list = []
_att_last = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def _att_cb(_ts, data, _lc):
    with _att_lock:
        _att_last["roll"]  = data["stateEstimate.roll"]
        _att_last["pitch"] = data["stateEstimate.pitch"]
        _att_last["yaw"]   = data["stateEstimate.yaw"]
        _att_samples.append({
            "t":     time.monotonic(),
            "roll":  _att_last["roll"],
            "pitch": _att_last["pitch"],
            "yaw":   _att_last["yaw"],
        })


def att_snapshot() -> dict:
    with _att_lock:
        return dict(_att_last)


def att_summary_for_leg(t_lo: float, t_hi: float) -> dict:
    """Slice the attitude stream in [t_lo, t_hi] and return min/max/abs-max
    for roll + pitch (degrees).  Empty leg → all zeros."""
    with _att_lock:
        seg = [s for s in _att_samples if t_lo <= s["t"] <= t_hi]
    if not seg:
        return {"n": 0, "roll_abs_max_deg": 0.0, "pitch_abs_max_deg": 0.0,
                "roll_range_deg": 0.0, "pitch_range_deg": 0.0}
    rolls  = [s["roll"]  for s in seg]
    pitchs = [s["pitch"] for s in seg]
    return {
        "n":                  len(seg),
        "roll_abs_max_deg":   max(abs(r) for r in rolls),
        "pitch_abs_max_deg":  max(abs(p) for p in pitchs),
        "roll_range_deg":     max(rolls)  - min(rolls),
        "pitch_range_deg":    max(pitchs) - min(pitchs),
        "roll_min_deg":       min(rolls),
        "roll_max_deg":       max(rolls),
        "pitch_min_deg":      min(pitchs),
        "pitch_max_deg":      max(pitchs),
    }


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
        # Gentle position-PID — see CF2_VEL_MAX_MPS / CF2_KP rationale
        # at module head.  Big pitch excursions break flow + ArUco FOV.
        for k, v in {
            "posCtlPid.xVelMax": CF2_VEL_MAX_MPS,
            "posCtlPid.yVelMax": CF2_VEL_MAX_MPS,
            "posCtlPid.xKp":     CF2_KP,
            "posCtlPid.yKp":     CF2_KP,
        }.items():
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
        lc.data_received_cb.add_callback(_att_cb)
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
        # Structured post-mortem log (Sim.md §10x).  Each meaningful
        # step writes a labelled snapshot so a crash leaves the
        # last-known-good state on disk.
        repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")
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
    att_origin  = att_snapshot()
    log.info(f"        origin aruco: n={snap_origin['n_markers']} "
             f"ids={snap_origin['ids']}; "
             f"att(roll={att_origin['roll']:+.1f}°,"
             f"pitch={att_origin['pitch']:+.1f}°,"
             f"yaw={att_origin['yaw']:+.1f}°)")
    summary["snap_origin"] = snap_origin
    summary["att_origin"]  = att_origin
    repl.exec_int(f"sentai.sim.journal_write('takeoff_origin', "
                  f"{{'aruco_n': {snap_origin['n_markers']}, "
                  f"'roll': {att_origin['roll']:.2f}, "
                  f"'pitch': {att_origin['pitch']:.2f}}})")

    # ─── Square ────────────────────────────────────────────────────────
    corner_log = []
    for (label, dx, dy) in CORNERS:
        tx = physical_origin[0] + dx
        ty = physical_origin[1] + dy
        with log.step(f"goto {label} → ({tx:+.3f},{ty:+.3f},{TAKEOFF_Z_M:.3f})"):
            t_leg_start = time.monotonic()
            arrived, last_pose, elapsed = goto_xy_abs(
                cf, tx, ty, TAKEOFF_Z_M, log, label=label)
            t_arrived = time.monotonic()
            hold_position(cf, tx, ty, TAKEOFF_Z_M, DWELL_S)
            t_dwell_end = time.monotonic()
            tel_now = tel_snapshot()
            snap = aruco_snapshot(repl)
            err_xy = math.hypot(tel_now["x"] - tx, tel_now["y"] - ty)
            log.info(f"        post-dwell pose=({tel_now['x']:+.3f},"
                     f"{tel_now['y']:+.3f},{tel_now['z']:.3f}); "
                     f"err_xy={err_xy*100:.2f}cm; "
                     f"aruco n={snap['n_markers']} ids={snap['ids']}")
            att_leg  = att_summary_for_leg(t_leg_start, t_arrived)
            att_dwell = att_summary_for_leg(t_arrived, t_dwell_end)
            log.info(f"        att during leg : pitch_abs_max="
                     f"{att_leg['pitch_abs_max_deg']:.1f}° "
                     f"roll_abs_max={att_leg['roll_abs_max_deg']:.1f}° "
                     f"(n={att_leg['n']})")
            log.info(f"        att during hover: pitch_abs_max="
                     f"{att_dwell['pitch_abs_max_deg']:.1f}° "
                     f"roll_abs_max={att_dwell['roll_abs_max_deg']:.1f}° "
                     f"(n={att_dwell['n']})")
            corner_log.append({
                "label":       label,
                "cmd":         [tx, ty, TAKEOFF_Z_M],
                "arrived":     arrived,
                "goto_elapsed_s": elapsed,
                "t_leg_start":   t_leg_start,
                "t_arrived":     t_arrived,
                "t_dwell_end":   t_dwell_end,
                "post_dwell_pose": [tel_now["x"], tel_now["y"], tel_now["z"]],
                "cmd_err_xy_cm": err_xy * 100.0,
                "aruco":       snap,
                "att_leg":     att_leg,
                "att_dwell":   att_dwell,
            })
            repl.exec_int(
                f"sentai.sim.journal_write('{label}', "
                f"{{'aruco_n': {snap['n_markers']}, "
                f"'err_cm': {err_xy*100:.2f}, "
                f"'pitch_max': {att_leg['pitch_abs_max_deg']:.2f}, "
                f"'roll_max': {att_leg['roll_abs_max_deg']:.2f}}})")
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
    repl.exec_int(f"sentai.sim.journal_write('mission_end', "
                  f"{{'land_err_cm': {land_err*100:.2f}, "
                  f"'duration_s': {summary['mission_duration_s']:.1f}}})")
    repl.exec_int("sentai.sim.journal_close()")

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
    ATTITUDE_JSON.write_text(json.dumps(_att_samples, indent=2))
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

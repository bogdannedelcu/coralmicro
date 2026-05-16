"""s138 — L6 LOST state recovery test.

Mid-INSPECT, force LOST state.  Drone must ascend, then host signals
marker_seen → FSM resumes pre-LOST state, mission completes normally,
drone lands within tolerance of PHYSICAL_ORIGIN.

Outputs under /tmp/s138_lost_recovery/.
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

WORKDIR = Path("/tmp/s138_lost_recovery")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
EXPLORE_TRACE   = WORKDIR / "explore_trace.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

JOURNAL_NAME = "s138_journal.txt"

# ─── Single synthetic target ───
TARGET_ID = 10
TARGET_XY = (+1.00, 0.00)
TARGET_Z  = 1.40

# ─── Mission params ───
TAKEOFF_Z_M       = 1.50
TAKEOFF_VEL_MPS   = 0.6
STOP_DIST_M       = 0.10
APPROACH_VEL_MPS  = 0.4
LAND_VEL_MPS      = 0.40
LOST_ALT_BOOST_M  = 1.5
LOST_TIMEOUT_MS   = 15000

POSE_LOOP_HZ      = 5.0
INSPECT_DWELL_S   = 1.7
LAND_DWELL_S      = 3.2


def pose_tick(repl, x, y, z, yaw_deg, transitions_seen):
    repl.exec_int(
        f"sentai.explore.set_pose({x:.4f},{y:.4f},{z:.4f},"
        f"{math.radians(yaw_deg):.4f})")
    st = repl.exec_value("sentai.explore.state()")
    if not transitions_seen or transitions_seen[-1] != st:
        transitions_seen.append(st)
    return st


def goto_xy_abs(cf, tx, ty, tz, yaw, log, timeout_s=15.0, tol_m=0.08, label=""):
    deadline = time.monotonic() + timeout_s
    n = 0
    last = None
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(tx, ty, tz, yaw)
        n += 1
        tel = tel_snapshot()
        last = (tel['x'], tel['y'], tel['z'])
        d = math.hypot(tel['x'] - tx, tel['y'] - ty)
        if d <= tol_m:
            log.info(f"        {label}: arrived @ "
                     f"({tel['x']:+.3f},{tel['y']:+.3f},{tel['z']:.3f}) "
                     f"d={d*100:.1f} cm")
            return True, last
        time.sleep(0.05)
    df = math.hypot(last[0] - tx, last[1] - ty)
    log.info(f"        {label}: TIMEOUT @ {last} d={df*100:.1f} cm")
    return False, last


def wait_for_state(repl, target_states, transitions_seen, timeout_s, log):
    deadline = time.monotonic() + timeout_s
    last = ""
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        last = pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                          transitions_seen)
        if last in target_states:
            return last
        time.sleep(1.0 / POSE_LOOP_HZ)
    log.info(f"        TIMEOUT waiting for {target_states} (last='{last}')")
    return last


def dwell_with_hold(repl, cf, hx, hy, hz, hyaw, dur_s, transitions_seen, log):
    deadline = time.monotonic() + dur_s
    last = ""
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(hx, hy, hz, hyaw)
        tel = tel_snapshot()
        last = pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                          transitions_seen)
        time.sleep(0.05)
    return last


def tick_dwell(repl, dur_s, transitions_seen, log):
    deadline = time.monotonic() + dur_s
    last = ""
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        last = pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                          transitions_seen)
        time.sleep(1.0 / POSE_LOOP_HZ)
    return last


def fly(log: StepLog, repl: ReplDriver) -> dict:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    summary: dict = {"target": [TARGET_ID, TARGET_XY[0], TARGET_XY[1], TARGET_Z]}
    transitions_seen: list = []
    t_mission_start = time.monotonic()

    # ─── Phase 1-3 ───
    with log.step("cf2 link + Kalman reset + params"):
        sync = SyncCrazyflie("udp://127.0.0.1:19850",
                             cf=Crazyflie(rw_cache=None))
        sync.open_link()
        cf = sync.cf
        cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(0.5)
        for k, v in {
            "posCtlPid.xyKd":    0.5,
            "velCtlPid.vxKd":    0.05,
            "velCtlPid.vyKd":    0.05,
            "posCtlPid.xVelMax": 3.0,
            "posCtlPid.yVelMax": 3.0,
            "posCtlPid.zVelMax": 2.0,
            "posCtlPid.xKp":     3.0,
            "posCtlPid.yKp":     3.0,
            "posCtlPid.zKp":     3.0,
        }.items():
            try: cf.param.set_value(k, v)
            except Exception: pass
        time.sleep(0.3)
        cf.param.set_value("kalman.resetEstimation", 1)
        time.sleep(0.5)
        cf.param.set_value("kalman.resetEstimation", 0)
        time.sleep(2.0)

    with log.step("telemetry log @ 20 ms"):
        lc = LogConfig(name="att", period_in_ms=20)
        for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
                  "stateEstimate.roll", "stateEstimate.pitch",
                  "stateEstimate.yaw"):
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

    with log.step("REPL init + inject target"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")
        repl.exec_int("sentai.object_lifter.clear()")
        rc = repl.exec_int("sentai.explore.init('sim')")
        if rc != 0:
            raise RuntimeError(f"explore.init rc={rc}")
        repl.exec_int("sentai.explore.set_tunables(0.10, 1500, 3000)")
        repl.exec_int(
            f"sentai.explore.set_lost_tunables({LOST_ALT_BOOST_M},"
            f"{LOST_TIMEOUT_MS})")

    # ─── Phase 4: takeoff ───
    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        physical_origin = (tel['x'], tel['y'])
        physical_takeoff_z = tel['z']
        summary["physical_origin"] = list(physical_origin)
        summary["physical_takeoff_z"] = physical_takeoff_z
        log.info(f"        PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
                 f"{physical_origin[1]:+.3f}); takeoff_z = "
                 f"{physical_takeoff_z:.3f}")

    # ─── Phase 5: inject target ───
    with log.step(f"L5 inject id={TARGET_ID} @ {TARGET_XY}"):
        rc = repl.exec_int(
            f"sentai.object_lifter.inject({TARGET_ID}, 1, "
            f"{TARGET_XY[0]}, {TARGET_XY[1]}, {TARGET_Z})")
        if rc < 0:
            raise RuntimeError(f"inject rc={rc}")

    # ─── Phase 6-7: explore.start + takeoff (L6) ───
    with log.step("explore.start + explore.takeoff → HOVERING"):
        repl.exec_int("sentai.explore.start()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        repl.exec_int(f"sentai.explore.takeoff({TAKEOFF_Z_M})")
        wait_for_state(repl, {"HOVERING"}, transitions_seen, 5.0, log)

    # ─── Phase 8: goto target → INSPECT ───
    with log.step(f"explore.goto({TARGET_ID}, {STOP_DIST_M})"):
        rc = repl.exec_int(f"sentai.explore.goto({TARGET_ID}, {STOP_DIST_M})")
        if rc != 0:
            raise RuntimeError(f"goto rc={rc}")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        m = repl.exec_repr("sentai.explore.metrics()")
        tgt_xy = (m["target_x"], m["target_y"])
        log.info(f"        L6 target=({tgt_xy[0]:+.3f},{tgt_xy[1]:+.3f})")

    with log.step("cf2 → target via closed-loop"):
        goto_xy_abs(cf, tgt_xy[0], tgt_xy[1], TAKEOFF_Z_M, 0.0, log,
                    label="goto")

    with log.step("pose loop → INSPECT"):
        st = wait_for_state(repl, {"INSPECT"}, transitions_seen, 8.0, log)
        if st != "INSPECT":
            raise RuntimeError(f"never reached INSPECT (got '{st}')")

    # ─── Phase 9: FORCE LOST mid-INSPECT ───
    tel_pre_lost = tel_snapshot()
    summary["pre_lost_pose"] = [tel_pre_lost['x'], tel_pre_lost['y'],
                                  tel_pre_lost['z']]
    with log.step("*** force_lost() at INSPECT — drone should ascend ***"):
        rc = repl.exec_int("sentai.explore.force_lost()")
        if rc != 0:
            raise RuntimeError(f"force_lost rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")
        if st != "LOST":
            raise RuntimeError(f"expected LOST, got '{st}'")

    # Servo emitted move(0, 0, +1.5, 0).  Host now drives cf2 to ascend
    # to takeoff_z + LOST_ALT_BOOST = ~3.0 m.
    target_high_z = physical_takeoff_z + LOST_ALT_BOOST_M
    with log.step(f"cf2 ascend to z={target_high_z:.2f} m (alt boost)"):
        deadline = time.monotonic() + 12
        max_z = tel_pre_lost['z']
        while time.monotonic() < deadline:
            cf.commander.send_position_setpoint(
                tel_pre_lost['x'], tel_pre_lost['y'], target_high_z, 0.0)
            tel = tel_snapshot()
            max_z = max(max_z, tel['z'])
            if abs(tel['z'] - target_high_z) < 0.10:
                log.info(f"        cf2 at ascend target @ z={tel['z']:.3f}; "
                         f"max_z_seen={max_z:.3f}")
                break
            # Keep L6 ticking so it can detect timeout if we fail to recover
            pose_tick(repl, tel['x'], tel['y'], tel['z'], tel['yaw_deg'],
                       transitions_seen)
            time.sleep(0.05)
        tel = tel_snapshot()
        summary["max_z_during_lost"] = max_z
        summary["pose_at_recovery_decision"] = [tel['x'], tel['y'], tel['z']]
        log.info(f"        post-ascend cf2 @ ({tel['x']:+.3f},"
                 f"{tel['y']:+.3f},{tel['z']:.3f})")

    # ─── Phase 10: signal recovery ───
    with log.step("signal_marker_seen(target_xy) — recover FSM"):
        # Inform L6 that we "see" the target marker at its known world xy.
        # L6 will reset pose to (target_x, target_y, current z) and
        # transition LOST → pre_lost_state.
        rc = repl.exec_int(
            f"sentai.explore.signal_marker_seen({tgt_xy[0]:.4f},"
            f"{tgt_xy[1]:.4f})")
        if rc != 0:
            raise RuntimeError(f"signal_marker_seen rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        recovered to state='{st}'")
        summary["state_post_recovery"] = st
        if st == "LOST":
            raise RuntimeError("LOST not exited after marker_seen signal")

    # Drone is at high altitude — descend back to takeoff_z, then resume
    # mission.  Host drives cf2 to (target_xy, takeoff_z).
    with log.step("cf2 descend back to mission altitude"):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            cf.commander.send_position_setpoint(
                tgt_xy[0], tgt_xy[1], TAKEOFF_Z_M, 0.0)
            tel = tel_snapshot()
            if abs(tel['z'] - TAKEOFF_Z_M) < 0.10:
                log.info(f"        cf2 back @ z={tel['z']:.3f}")
                break
            pose_tick(repl, tel['x'], tel['y'], tel['z'], tel['yaw_deg'],
                       transitions_seen)
            time.sleep(0.05)

    # ─── Phase 11: continue INSPECT dwell + HOVERING ───
    # If state == INSPECT after recovery, let it dwell out.
    # If state == HOVERING already, fine.
    st = repl.exec_value("sentai.explore.state()")
    if st == "INSPECT":
        with log.step("INSPECT dwell completion (post-recovery)"):
            dwell_with_hold(repl, cf, tgt_xy[0], tgt_xy[1], TAKEOFF_Z_M,
                             0.0, INSPECT_DWELL_S, transitions_seen, log)
            st = repl.exec_value("sentai.explore.state()")
            if st != "HOVERING":
                raise RuntimeError(f"INSPECT did not → HOVERING; got '{st}'")

    # ─── Phase 12: return_home → land ───
    with log.step("explore.return_home"):
        repl.exec_int("sentai.explore.return_home()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))

    with log.step("cf2 → PHYSICAL_ORIGIN"):
        goto_xy_abs(cf, physical_origin[0], physical_origin[1], TAKEOFF_Z_M,
                     0.0, log, label="return_home")
        wait_for_state(repl, {"HOVERING"}, transitions_seen, 10.0, log)

    with log.step("explore.land + controlled descent"):
        repl.exec_int("sentai.explore.land()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        # Manual descent (anti-drift).
        t0 = time.monotonic()
        descent_dur = 3.0
        while True:
            te = time.monotonic() - t0
            frac = min(1.0, te / descent_dur)
            z = TAKEOFF_Z_M + frac * (0.05 - TAKEOFF_Z_M)
            cf.commander.send_position_setpoint(
                physical_origin[0], physical_origin[1], z, 0.0)
            tel = tel_snapshot()
            pose_tick(repl, tel['x'], tel['y'], tel['z'], tel['yaw_deg'],
                       transitions_seen)
            if frac >= 1.0 and tel['z'] < 0.10:
                break
            if te > descent_dur + 4:
                break
            time.sleep(0.05)
        tel = tel_snapshot()
        land_pose = [tel['x'], tel['y'], tel['z']]
        summary["land_pose"] = land_pose
        cf.commander.send_stop_setpoint()
        time.sleep(0.5)
        tick_dwell(repl, LAND_DWELL_S, transitions_seen, log)
        summary["state_final"] = repl.exec_value("sentai.explore.state()")

    # ─── Final metrics ───
    with log.step("final metrics + closure"):
        m = repl.exec_repr("sentai.explore.metrics()")
        trace = repl.exec_repr("sentai.explore.trace()")
        land_err = math.hypot(land_pose[0] - physical_origin[0],
                               land_pose[1] - physical_origin[1])
        summary["land_err_xy_vs_origin_m"] = land_err
        summary["transitions_seen"] = transitions_seen
        summary["explore_metrics_final"] = m
        summary["mission_duration_s"] = time.monotonic() - t_mission_start
        log.info(f"        land_err={land_err*100:.1f} cm")
        log.info(f"        max_z_during_lost={summary['max_z_during_lost']:.2f}")
        log.info(f"        transitions={transitions_seen}")
        log.info(f"        mission_duration={summary['mission_duration_s']:.1f} s")
        EXPLORE_TRACE.write_text(json.dumps(trace, indent=2, default=str))

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
            journal_path = SENTAI_FS_ROOT / JOURNAL_NAME
            if journal_path.is_file():
                (WORKDIR / "journal.txt").write_bytes(journal_path.read_bytes())
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

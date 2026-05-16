"""s137 — L6 LONG-DISTANCE exploration test via synthetic L5 injection.

Drone visibly traverses ~4 m total path:
  origin → goto target 1 at (+1.5, 0.0) → INSPECT
         → goto target 2 at (-0.5, +1.0) → INSPECT
         → return_home to origin → land

Uses sentai.object_lifter.inject(tid, cls, x, y, z) — TEST-ONLY API
documented in sentai_object_lifter.h — to place synthetic targets at
known world positions, bypassing L5 inverse-depth EKF.  Real L5
convergence (on real markers) was proved in s136.

Outputs under /tmp/s137_explore_long/.
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
S136_DIR = Path(__file__).resolve().parent.parent / "s136_explore_real"
for p in (S091_DIR, S128_DIR, S132_DIR, S136_DIR):
    sys.path.insert(0, str(p))

import aruco_hover                                       # noqa: E402
from mission_l41 import ReplDriver, StepLog              # noqa: E402
from mission_lifter import _tel_cb, tel_snapshot, _tel_samples  # noqa: E402

WORKDIR = Path("/tmp/s137_explore_long")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
EXPLORE_TRACE   = WORKDIR / "explore_trace.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

JOURNAL_NAME = "s137_journal.txt"

# ─── Synthetic targets (world positions) ───
TARGETS = [
    # (id, class, world_x, world_y, world_z)
    (10, 1, +1.50, 0.00, 1.40),   # 1.5 m forward
    (11, 1, -0.50, +1.00, 1.40),  # 1.0 m left + 0.5 m back (diagonal)
]

# ─── Mission params ───
TAKEOFF_Z_M       = 1.50
TAKEOFF_VEL_MPS   = 0.6
STOP_DIST_M       = 0.10
APPROACH_VEL_MPS  = 0.5    # higher — longer distances
LAND_VEL_MPS      = 0.40
GOTO_TIMEOUT_S    = 25.0   # 1.5 m at 0.5 m/s ≈ 3 s + PID settle
GOTO_TOL_M        = 0.08

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


def goto_xy_abs(cf, target_x, target_y, z, yaw_deg, log,
                 timeout_s=GOTO_TIMEOUT_S, tol_m=GOTO_TOL_M, label=""):
    deadline = time.monotonic() + timeout_s
    n_spam = 0
    last_pose = None
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(target_x, target_y, z, yaw_deg)
        n_spam += 1
        tel = tel_snapshot()
        last_pose = (tel['x'], tel['y'], tel['z'])
        dist = math.hypot(tel['x'] - target_x, tel['y'] - target_y)
        if dist <= tol_m:
            log.info(f"        {label}: arrived @ ({tel['x']:+.3f},"
                     f"{tel['y']:+.3f},{tel['z']:.3f}) "
                     f"dist={dist*100:.1f} cm spam={n_spam}")
            return True, last_pose
        time.sleep(0.05)
    dist_final = math.hypot(last_pose[0] - target_x, last_pose[1] - target_y)
    log.info(f"        {label}: TIMEOUT @ ({last_pose[0]:+.3f},"
             f"{last_pose[1]:+.3f},{last_pose[2]:.3f}) "
             f"dist={dist_final*100:.1f} cm")
    return False, last_pose


def wait_for_state(repl, target_states, transitions_seen, timeout_s, log):
    deadline = time.monotonic() + timeout_s
    last_state = ""
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        st = pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                       transitions_seen)
        last_state = st
        if st in target_states:
            return st
        time.sleep(1.0 / POSE_LOOP_HZ)
    log.info(f"        TIMEOUT waiting for {target_states} (last='{last_state}')")
    return last_state


def dwell_with_hold(repl, cf, hold_x, hold_y, hold_z, hold_yaw,
                     dur_s, transitions_seen, log):
    deadline = time.monotonic() + dur_s
    last_state = ""
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(hold_x, hold_y, hold_z, hold_yaw)
        tel = tel_snapshot()
        last_state = pose_tick(repl, tel["x"], tel["y"], tel["z"],
                               tel["yaw_deg"], transitions_seen)
        time.sleep(0.05)
    log.info(f"        dwell-with-hold {dur_s:.1f}s done; state='{last_state}'")
    return last_state


def tick_dwell(repl, dur_s, transitions_seen, log):
    deadline = time.monotonic() + dur_s
    last_state = ""
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        last_state = pose_tick(repl, tel["x"], tel["y"], tel["z"],
                               tel["yaw_deg"], transitions_seen)
        time.sleep(1.0 / POSE_LOOP_HZ)
    return last_state


def goto_phase(cf, repl, log, target_id, target_xy, transitions_seen,
                summary, label):
    """Execute one explore.goto cycle.  Returns displacement_xy."""
    tel_before = tel_snapshot()
    pose_before = (tel_before['x'], tel_before['y'])

    with log.step(f"explore.goto({target_id}, {STOP_DIST_M}) [{label}]"):
        rc = repl.exec_int(f"sentai.explore.goto({target_id}, {STOP_DIST_M})")
        if rc != 0:
            raise RuntimeError(f"explore.goto({target_id}) rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        m = repl.exec_repr("sentai.explore.metrics()")
        tgt_xy = (m["target_x"], m["target_y"])
        log.info(f"        state='{st}'; L6 target=({tgt_xy[0]:+.3f},"
                 f"{tgt_xy[1]:+.3f}); GT target=({target_xy[0]:+.3f},"
                 f"{target_xy[1]:+.3f})")

    with log.step(f"cf2 → target xy via closed-loop setpoint [{label}]"):
        arrived, _ = goto_xy_abs(cf, tgt_xy[0], tgt_xy[1], TAKEOFF_Z_M, 0.0,
                                   log, label=label)
        if not arrived:
            log.info("        WARN: did not reach goto tolerance")

    with log.step(f"pose loop → INSPECT/HOVERING [{label}]"):
        st = wait_for_state(repl, {"INSPECT", "HOVERING"}, transitions_seen,
                            timeout_s=10.0, log=log)
        if st not in ("INSPECT", "HOVERING"):
            raise RuntimeError(f"never reached INSPECT/HOVERING (got '{st}')")
        if st == "INSPECT":
            with log.step(f"INSPECT dwell w/ hold ~{INSPECT_DWELL_S:.1f}s [{label}]"):
                dwell_with_hold(repl, cf, tgt_xy[0], tgt_xy[1], TAKEOFF_Z_M,
                                 0.0, INSPECT_DWELL_S, transitions_seen, log)
                st2 = repl.exec_value("sentai.explore.state()")
                if st2 != "HOVERING":
                    raise RuntimeError(
                        f"INSPECT did not transition to HOVERING (got '{st2}')")

    tel_after = tel_snapshot()
    pose_after = (tel_after['x'], tel_after['y'])
    disp = math.hypot(pose_after[0] - pose_before[0],
                      pose_after[1] - pose_before[1])
    log.info(f"        [{label}] displacement = {disp*100:.1f} cm")
    return disp, pose_after


def fly(log: StepLog, repl: ReplDriver) -> dict:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    summary: dict = {"targets": TARGETS}
    transitions_seen: list = []
    t_mission_start = time.monotonic()

    # ─── Phase 1-3: cf2 setup ───
    with log.step("cf2 link + Kalman reset + damp params"):
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
            "posCtlPid.xKp":     3.0,
            "posCtlPid.yKp":     3.0,
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
                  "stateEstimate.roll", "stateEstimate.pitch",
                  "stateEstimate.yaw"):
            lc.add_variable(v, "float")
        cf.log.add_config(lc)
        lc.data_received_cb.add_callback(_tel_cb)
        lc.start()

    with log.step("flow forwarder thread (2s grace)"):
        stop_evt = threading.Event()
        flow_stats = {"n_sent": 0, "fatal": None, "last_err": None}
        flow_th = threading.Thread(target=aruco_hover.flow_forwarder,
                                    args=(stop_evt, cf, flow_stats),
                                    daemon=True)
        flow_th.start()
        time.sleep(2.0)
        if flow_stats["fatal"]:
            raise RuntimeError(f"flow forwarder fatal: {flow_stats['fatal']}")

    with log.step("REPL init + lifter.clear + explore.init"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")
        repl.exec_int("sentai.object_lifter.clear()")
        rc = repl.exec_int("sentai.explore.init('sim')")
        if rc != 0:
            raise RuntimeError(f"explore.init rc={rc}")
        # Long mission, looser HOME_RADIUS won't matter (targets are 1m+ away)
        # but shrink anyway so the geometric check is unambiguous.
        repl.exec_int("sentai.explore.set_tunables(0.15, 1500, 3000)")

    # ─── Phase 4: takeoff + CAPTURE PHYSICAL_ORIGIN ───
    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        physical_origin = (tel['x'], tel['y'])
        summary["physical_origin"] = list(physical_origin)
        log.info(f"        PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
                 f"{physical_origin[1]:+.3f})")

    # ─── Phase 5: INJECT synthetic targets ───
    with log.step(f"L5 inject {len(TARGETS)} synthetic targets"):
        for (tid, cls, wx, wy, wz) in TARGETS:
            rc = repl.exec_int(
                f"sentai.object_lifter.inject({tid}, {cls}, {wx}, {wy}, {wz})")
            if rc < 0:
                raise RuntimeError(f"inject({tid}) rc={rc}")
            wpos = repl.exec_repr(f"sentai.object_lifter.world_pos({tid})")
            log.info(f"        injected id={tid} @ ({wx:+.2f},{wy:+.2f},"
                     f"{wz:+.2f}); world_pos={wpos}")

    # ─── Phase 6-7: explore.start + takeoff (L6) ───
    with log.step("explore.start + explore.takeoff → HOVERING"):
        rc = repl.exec_int("sentai.explore.start()")
        if rc != 0:
            raise RuntimeError(f"explore.start rc={rc}")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        rc = repl.exec_int(f"sentai.explore.takeoff({TAKEOFF_Z_M})")
        if rc != 0:
            raise RuntimeError(f"explore.takeoff rc={rc}")
        st = wait_for_state(repl, {"HOVERING"}, transitions_seen,
                            timeout_s=5.0, log=log)
        m = repl.exec_repr("sentai.explore.metrics()")
        summary["l6_home"] = [m["home_x"], m["home_y"]]
        log.info(f"        L6 home = ({m['home_x']:+.3f},{m['home_y']:+.3f})")

    # ─── Phase 8-9: goto(10) ───
    disp1, _ = goto_phase(cf, repl, log, 10, (TARGETS[0][2], TARGETS[0][3]),
                           transitions_seen, summary, "goto1")
    summary["displacement_goto1_m"] = disp1

    # ─── Phase 10: goto(11) ───
    disp2, _ = goto_phase(cf, repl, log, 11, (TARGETS[1][2], TARGETS[1][3]),
                           transitions_seen, summary, "goto2")
    summary["displacement_goto2_m"] = disp2

    # ─── Phase 11: return_home ───
    pose_pre_return = tel_snapshot()
    with log.step("explore.return_home → RETURNING"):
        rc = repl.exec_int("sentai.explore.return_home()")
        if rc != 0:
            raise RuntimeError(f"return_home rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step("cf2 → PHYSICAL_ORIGIN via closed-loop"):
        goto_xy_abs(cf, physical_origin[0], physical_origin[1],
                     TAKEOFF_Z_M, 0.0, log, label="return_home")
        st = wait_for_state(repl, {"HOVERING"}, transitions_seen,
                            timeout_s=15.0, log=log)
        if st != "HOVERING":
            raise RuntimeError(f"never returned to HOVERING (got '{st}')")

    pose_post_return = tel_snapshot()
    disp_return = math.hypot(pose_post_return['x'] - pose_pre_return['x'],
                              pose_post_return['y'] - pose_pre_return['y'])
    summary["displacement_return_m"] = disp_return
    log.info(f"        return displacement = {disp_return*100:.1f} cm")

    # ─── Phase 12: land ───
    with log.step("explore.land → LANDING + controlled descent"):
        rc = repl.exec_int("sentai.explore.land()")
        if rc != 0:
            raise RuntimeError(f"explore.land rc={rc}")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))

        # Controlled descent (anti-drift)
        descent_dur_s = 3.0
        z_top = TAKEOFF_Z_M
        z_bottom = 0.05
        t0 = time.monotonic()
        while True:
            t_e = time.monotonic() - t0
            frac = min(1.0, t_e / descent_dur_s)
            z = z_top + frac * (z_bottom - z_top)
            cf.commander.send_position_setpoint(physical_origin[0],
                                                  physical_origin[1],
                                                  z, 0.0)
            tel = tel_snapshot()
            pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                       transitions_seen)
            if frac >= 1.0 and tel['z'] < 0.10:
                break
            if t_e > descent_dur_s + 4:
                break
            time.sleep(0.05)
        tel = tel_snapshot()
        land_pose = (tel['x'], tel['y'], tel['z'])
        summary["land_pose"] = list(land_pose)
        log.info(f"        post-descent cf2 @ ({tel['x']:+.3f},"
                 f"{tel['y']:+.3f},{tel['z']:.3f})  ← LAND_POSE")
        cf.commander.send_stop_setpoint()
        time.sleep(0.5)
        tick_dwell(repl, LAND_DWELL_S, transitions_seen, log)
        st = repl.exec_value("sentai.explore.state()")
        summary["state_final"] = st
        if st != "DONE":
            raise RuntimeError(f"never reached DONE (got '{st}')")

    # ─── Phase 13: final metrics ───
    with log.step("final metrics + closure check"):
        m = repl.exec_repr("sentai.explore.metrics()")
        trace = repl.exec_repr("sentai.explore.trace()")
        servo_st = repl.exec_repr("sentai.servo.status()")

        land_err = math.hypot(land_pose[0] - physical_origin[0],
                               land_pose[1] - physical_origin[1])
        summary["land_err_xy_vs_origin_m"] = land_err
        summary["transitions_seen"] = transitions_seen
        summary["explore_metrics_final"] = m
        summary["explore_trace_count"] = len(trace) if trace else 0
        summary["servo_status_final"] = servo_st
        summary["mission_duration_s"] = time.monotonic() - t_mission_start

        log.info(f"        PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
                 f"{physical_origin[1]:+.3f})")
        log.info(f"        LAND_POSE       = ({land_pose[0]:+.3f},"
                 f"{land_pose[1]:+.3f},{land_pose[2]:.3f})")
        log.info(f"        land_err = {land_err*100:.1f} cm")
        log.info(f"        disp goto1 = {disp1*100:.1f} cm; "
                 f"goto2 = {disp2*100:.1f} cm; "
                 f"return = {disp_return*100:.1f} cm")
        log.info(f"        transitions = {transitions_seen}")
        log.info(f"        gotos_completed = {m['gotos_completed']}; "
                 f"aborts = {m['aborts']}; "
                 f"transitions_counter = {m['transitions']}")
        log.info(f"        mission_duration = {summary['mission_duration_s']:.1f} s")
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
            with log.step("dump journal.txt copy"):
                if journal_path.is_file():
                    (WORKDIR / "journal.txt").write_bytes(
                        journal_path.read_bytes())
            repl.close()
    except BaseException as e:
        log.info(f"FATAL: {type(e).__name__}: {e}")
        SUMMARY_JSON.write_text(json.dumps({
            "_status":    "FAIL",
            "_exception": f"{type(e).__name__}: {e}",
            "_last_run":  dt.datetime.now().isoformat(),
        }, indent=2, default=str))
        log.close()
        return 1
    summary["_last_run"] = dt.datetime.now().isoformat()
    SUMMARY_JSON.write_text(json.dumps(summary, indent=2, default=str))
    TELEMETRY_JSON.write_text(json.dumps(_tel_samples, indent=2))
    log.info(f"summary written to {SUMMARY_JSON}")
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""s134 — drive sentai.explore (L6 mission FSM) end-to-end through cf2
Gazebo SITL.  First Gazebo integration after s133 SIM 100/100.

Reuses the s128/s132 harness (ReplDriver, StepLog, flow_forwarder,
telemetry callback) and the s130/s132 aruco_detector for marker
bbox extraction.

Outputs (under /tmp/s134_explore_gazebo/):
    summary.json         — verdict reads this
    explore_trace.json   — trace ring dump + state-transition log
    mission.log          — host-side step trace
    repl.transcript      — raw stdin/stdout dump
    journal.txt          — REPL-side structured journal
    cf2_telemetry.json   — stateEstimate samples
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
sys.path.insert(0, str(S091_DIR))
sys.path.insert(0, str(S128_DIR))
sys.path.insert(0, str(S132_DIR))

import aruco_hover                                       # noqa: E402
from aruco_detector import MARKER_SIZE_M                 # noqa: E402
from mission_l41 import ReplDriver, StepLog              # noqa: E402
from mission_lifter import (                             # noqa: E402
    bbox_from_marker, sample_marker_bbox, _tel_cb, tel_snapshot,
    _tel_samples,
)

WORKDIR = Path("/tmp/s134_explore_gazebo")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
EXPLORE_TRACE   = WORKDIR / "explore_trace.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

JOURNAL_NAME = "s134_journal.txt"

# ─── Target marker (same as s132) ───
TARGET_ARUCO_ID  = 0
TARGET_WORLD_XYZ = (+0.15, +0.10, 0.20)

# ─── Mission params ───
TAKEOFF_Z_M       = 1.50
TAKEOFF_VEL_MPS   = 0.6
AWAY_OFFSET_M     = (-1.0, 0.0)   # move cf2 out before goto (richer APPROACH)
APPROACH_VEL_MPS  = 0.30
STOP_DIST_M       = 0.30
LAND_VEL_MPS      = 0.40
HOME_RADIUS_M     = 0.20
LANDING_ERR_MAX_M = 0.30
MISSION_TIMEOUT_S = 90.0

POSE_LOOP_HZ      = 5.0
INSPECT_DWELL_S   = 1.7     # > L6 INSPECT_DUR_MS (1500)
LAND_DWELL_S      = 3.2     # > L6 LAND_DUR_MS (3000)


def pose_tick(repl, x: float, y: float, z: float, yaw_deg: float,
              transitions_seen: list) -> str:
    """Push set_pose into L6, then read state.  Logs transitions."""
    repl.exec_int(
        f"sentai.explore.set_pose({x:.4f},{y:.4f},{z:.4f},"
        f"{math.radians(yaw_deg):.4f})")
    st = repl.exec_value("sentai.explore.state()")
    if not transitions_seen or transitions_seen[-1] != st:
        transitions_seen.append(st)
    return st


def wait_for_state(repl, target_states: set[str], transitions_seen: list,
                   timeout_s: float, log: StepLog) -> str:
    """Pose-loop until L6 state hits one of target_states.  Returns last state."""
    deadline = time.monotonic() + timeout_s
    last_state = ""
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        st = pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                       transitions_seen)
        last_state = st
        if st in target_states:
            log.info(f"        reached state '{st}'")
            return st
        time.sleep(1.0 / POSE_LOOP_HZ)
    log.info(f"        TIMEOUT after {timeout_s:.1f}s waiting for "
             f"{target_states} (last='{last_state}')")
    return last_state


def tick_dwell(repl, dur_s: float, transitions_seen: list,
               log: StepLog) -> str:
    """Pose-tick at low rate during dwell phases (INSPECT/LANDING)."""
    deadline = time.monotonic() + dur_s
    last_state = ""
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        last_state = pose_tick(repl, tel["x"], tel["y"], tel["z"],
                               tel["yaw_deg"], transitions_seen)
        time.sleep(1.0 / POSE_LOOP_HZ)
    log.info(f"        dwell {dur_s:.1f}s done; state='{last_state}'")
    return last_state


def fly(log: StepLog, repl: ReplDriver) -> dict:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    summary: dict = {
        "target_aruco_id":  TARGET_ARUCO_ID,
        "target_world_xyz": list(TARGET_WORLD_XYZ),
        "phases": [],
        "transitions": [],
    }
    transitions_seen: list = []
    t_mission_start = time.monotonic()

    # ─── Phase 1: cf2 setup ───
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
            "posCtlPid.xVelMax": 2.5,
            "posCtlPid.yVelMax": 2.5,
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
        log.info(f"        flow forwarder running "
                 f"(n_sent={flow_stats['n_sent']})")

    # ─── Phase 2: REPL imports + explore.init ───
    with log.step("REPL: import sentai + journal_open + lifter.clear"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        rc = repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        if rc != 0:
            raise RuntimeError(f"journal_open returned {rc}")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")
        repl.exec_int("sentai.object_lifter.clear()")

    with log.step("REPL: explore.init('sim')"):
        rc = repl.exec_int("sentai.explore.init('sim')")
        if rc != 0:
            raise RuntimeError(f"explore.init rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        log.info(f"        post-init state='{st}'")
        if st != "IDLE":
            raise RuntimeError(f"expected IDLE post-init, got '{st}'")

    # ─── Phase 3: cf2 takeoff to 1.5m at origin ───
    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m @ {TAKEOFF_VEL_MPS} m/s"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        log.info(f"        post-takeoff cf2 @ ({tel['x']:+.2f},"
                 f"{tel['y']:+.2f},{tel['z']:.2f}) yaw={tel['yaw_deg']:+.1f}")

    # ─── Phase 4: seed lifter for marker id=0 ───
    with log.step("lifter init_from_bbox for id=0 (force LIFTED via near-marker prior)"):
        bbox = sample_marker_bbox(repl.frames_dir, TARGET_ARUCO_ID,
                                   -1, log)
        if bbox is None:
            raise RuntimeError(
                f"id{TARGET_ARUCO_ID} not visible at takeoff — abort")
        tel = tel_snapshot()
        drone_W = (tel["x"], tel["y"], tel["z"])
        yaw = math.radians(tel["yaw_deg"])
        # NEAR-MARKER prior: bbox=45 px on 0.0625 m physical → LIFTED at first
        # call per the L5 init math (σ_ρ₀ < ε·ρ²).  We use a fabricated bbox
        # size for the prior even though the real frame may show a different
        # apparent size — this is intentional, since we want the L6 FSM test
        # to start from a LIFTED tracklet without needing a parallax pass.
        # The world_pos returned reflects this prior, which is good enough
        # for goto target xy (drone navigates roughly toward the marker zone).
        rc = repl.exec_int(
            f"sentai.object_lifter.init_from_bbox(0, 0, "
            f"{bbox['u']:.2f}, {bbox['v']:.2f}, 45.0, {MARKER_SIZE_M}, "
            f"({drone_W[0]:.4f},{drone_W[1]:.4f},{drone_W[2]:.4f}), {yaw:.4f})")
        if rc < 0:
            raise RuntimeError(f"lifter init_from_bbox rc={rc}")
        snap = repl.exec_repr("sentai.object_lifter.get(0)")
        log.info(f"        post-init: status={snap['status']} rho={snap['rho']:.3f}"
                 f"  var_rho={snap['var_rho']:.4f}")
        wpos = repl.exec_repr("sentai.object_lifter.world_pos(0)")
        log.info(f"        world_pos(0)={wpos}  GT={TARGET_WORLD_XYZ}")
        summary["lifter_seed_snap"] = snap
        summary["lifter_seed_world_pos"] = list(wpos) if wpos else None
        if snap["status"] != 2:    # LIFTER_LIFTED
            raise RuntimeError(
                f"lifter did not reach LIFTED (status={snap['status']})")

    # ─── Phase 5: move cf2 to AWAY_OFFSET for richer APPROACH ───
    with log.step(f"cf2 mc.move_distance{AWAY_OFFSET_M} → AWAY pose"):
        mc.move_distance(AWAY_OFFSET_M[0], AWAY_OFFSET_M[1], 0.0,
                         velocity=APPROACH_VEL_MPS)
        time.sleep(0.5)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        log.info(f"        cf2 @ ({tel['x']:+.2f},{tel['y']:+.2f},"
                 f"{tel['z']:.2f}) yaw={tel['yaw_deg']:+.1f}")
        summary["away_pose"] = [tel['x'], tel['y'], tel['z']]

    # ─── Phase 6: explore.start + explore.takeoff + pose loop → HOVERING ───
    with log.step("explore.start → ARMING"):
        rc = repl.exec_int("sentai.explore.start()")
        if rc != 0:
            raise RuntimeError(f"explore.start rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step(f"explore.takeoff({TAKEOFF_Z_M}) → TAKEOFF"):
        rc = repl.exec_int(f"sentai.explore.takeoff({TAKEOFF_Z_M})")
        if rc != 0:
            raise RuntimeError(f"explore.takeoff rc={rc}")
        # pose loop should transition TAKEOFF → HOVERING almost immediately
        # since cf2 is already at z ≈ 1.5 m
        st = wait_for_state(repl, {"HOVERING"}, transitions_seen,
                            timeout_s=5.0, log=log)
        if st != "HOVERING":
            raise RuntimeError(f"never reached HOVERING (got '{st}')")
        m = repl.exec_repr("sentai.explore.metrics()")
        summary["home_xy"] = [m["home_x"], m["home_y"], m["home_z"]]
        log.info(f"        home set @ ({m['home_x']:+.2f},"
                 f"{m['home_y']:+.2f},{m['home_z']:.2f})")

    # ─── Phase 7: explore.goto(0) → APPROACH → INSPECT → HOVERING ───
    with log.step(f"explore.goto(0, {STOP_DIST_M}) → APPROACH"):
        rc = repl.exec_int(f"sentai.explore.goto(0, {STOP_DIST_M})")
        if rc != 0:
            raise RuntimeError(f"explore.goto rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")
        m = repl.exec_repr("sentai.explore.metrics()")
        tgt_xy = (m["target_x"], m["target_y"])
        log.info(f"        L6 target xy=({tgt_xy[0]:+.3f},{tgt_xy[1]:+.3f})")
        summary["goto_target_xy"] = list(tgt_xy)

    with log.step(f"cf2 mc.move_distance toward target_xy (stop {STOP_DIST_M} m short)"):
        tel = tel_snapshot()
        dx = tgt_xy[0] - tel['x']
        dy = tgt_xy[1] - tel['y']
        dist = math.hypot(dx, dy)
        if dist > STOP_DIST_M:
            step_len = dist - STOP_DIST_M
            step_x = dx * step_len / dist
            step_y = dy * step_len / dist
            log.info(f"        moving cf2 dx={step_x:+.2f} dy={step_y:+.2f}")
            mc.move_distance(step_x, step_y, 0.0, velocity=APPROACH_VEL_MPS)
        else:
            log.info(f"        already within stop_dist (d={dist:.2f})")
        time.sleep(0.5)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(0.5)

    with log.step("pose loop → INSPECT"):
        st = wait_for_state(repl, {"INSPECT", "HOVERING"}, transitions_seen,
                            timeout_s=10.0, log=log)
        if st not in ("INSPECT", "HOVERING"):
            raise RuntimeError(f"never reached INSPECT/HOVERING (got '{st}')")
        if st == "INSPECT":
            with log.step(f"INSPECT dwell ~{INSPECT_DWELL_S:.1f}s → HOVERING"):
                tick_dwell(repl, INSPECT_DWELL_S, transitions_seen, log)
                st2 = repl.exec_value("sentai.explore.state()")
                if st2 != "HOVERING":
                    raise RuntimeError(
                        f"INSPECT did not transition to HOVERING (got '{st2}')")
        m = repl.exec_repr("sentai.explore.metrics()")
        summary["gotos_completed"] = m.get("gotos_completed", 0)

    # ─── Phase 8: explore.return_home → RETURNING → HOVERING ───
    with log.step("explore.return_home() → RETURNING"):
        rc = repl.exec_int("sentai.explore.return_home()")
        if rc != 0:
            raise RuntimeError(f"return_home rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step("cf2 mc.move_distance back toward home_xy"):
        tel = tel_snapshot()
        m = repl.exec_repr("sentai.explore.metrics()")
        hx, hy = m["home_x"], m["home_y"]
        dx, dy = hx - tel['x'], hy - tel['y']
        log.info(f"        moving cf2 dx={dx:+.2f} dy={dy:+.2f}")
        mc.move_distance(dx, dy, 0.0, velocity=APPROACH_VEL_MPS)
        time.sleep(0.5)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(0.5)

    with log.step("pose loop → HOVERING (home reached)"):
        st = wait_for_state(repl, {"HOVERING"}, transitions_seen,
                            timeout_s=10.0, log=log)
        if st != "HOVERING":
            raise RuntimeError(f"never returned to HOVERING (got '{st}')")

    # ─── Phase 9: explore.land → LANDING → DONE ───
    with log.step("explore.land() → LANDING"):
        rc = repl.exec_int("sentai.explore.land()")
        if rc != 0:
            raise RuntimeError(f"explore.land rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step(f"cf2 mc.land + LANDING dwell ~{LAND_DWELL_S:.1f}s → DONE"):
        mc.land(velocity=LAND_VEL_MPS)
        time.sleep(2.5)
        tick_dwell(repl, LAND_DWELL_S, transitions_seen, log)
        st = repl.exec_value("sentai.explore.state()")
        log.info(f"        final state='{st}'")
        summary["state_final"] = st
        if st != "DONE":
            raise RuntimeError(f"never reached DONE (got '{st}')")

    # ─── Phase 10: final metrics ───
    with log.step("final metrics + trace dump"):
        m = repl.exec_repr("sentai.explore.metrics()")
        trace = repl.exec_repr("sentai.explore.trace()")
        servo_st = repl.exec_repr("sentai.servo.status()")

        tel = tel_snapshot()
        land_err_xy = math.hypot(tel['x'] - summary["home_xy"][0],
                                  tel['y'] - summary["home_xy"][1])
        summary["land_err_xy_m"] = land_err_xy
        summary["land_pose"] = [tel['x'], tel['y'], tel['z']]
        summary["transitions_seen"] = transitions_seen
        summary["explore_metrics_final"] = m
        summary["explore_trace_count"] = len(trace) if trace else 0
        summary["servo_status_final"] = servo_st
        summary["mission_duration_s"] = time.monotonic() - t_mission_start

        log.info(f"        state final = {st}")
        log.info(f"        transitions_seen = {transitions_seen}")
        log.info(f"        aborts = {m['aborts']}")
        log.info(f"        gotos_completed = {m['gotos_completed']}")
        log.info(f"        transitions counter = {m['transitions']}")
        log.info(f"        land_err_xy = {land_err_xy:.3f} m")
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

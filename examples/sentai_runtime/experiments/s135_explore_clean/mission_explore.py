"""s135 — clean origin-locked L6 mission FSM test.

Per [[sim-test-must-return-home]]: drone MUST return to its physical
takeoff origin.  No mid-flight displacement between cf2.take_off and
explore.start (would contaminate L6's `home` capture).

Outputs (under /tmp/s135_explore_clean/):
    summary.json         — verdict reads this
    explore_trace.json   — L6 trace ring dump
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
    sample_marker_bbox, _tel_cb, tel_snapshot, _tel_samples,
)

WORKDIR = Path("/tmp/s135_explore_clean")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
EXPLORE_TRACE   = WORKDIR / "explore_trace.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

JOURNAL_NAME = "s135_journal.txt"

# ─── Target marker (same world as s132) ───
TARGET_ARUCO_ID  = 0
TARGET_WORLD_XYZ = (+0.15, +0.10, 0.20)

# ─── Mission params ───
TAKEOFF_Z_M       = 1.50
TAKEOFF_VEL_MPS   = 0.6
STOP_DIST_M       = 0.05      # small — keep APPROACH visible despite tight field
APPROACH_VEL_MPS  = 0.20
LAND_VEL_MPS      = 0.40
MISSION_TIMEOUT_S = 60.0

POSE_LOOP_HZ      = 5.0
INSPECT_DWELL_S   = 1.7       # > L6 INSPECT_DUR_MS (1500)
LAND_DWELL_S      = 3.2       # > L6 LAND_DUR_MS (3000)


def pose_tick(repl, x: float, y: float, z: float, yaw_deg: float,
              transitions_seen: list) -> str:
    repl.exec_int(
        f"sentai.explore.set_pose({x:.4f},{y:.4f},{z:.4f},"
        f"{math.radians(yaw_deg):.4f})")
    st = repl.exec_value("sentai.explore.state()")
    if not transitions_seen or transitions_seen[-1] != st:
        transitions_seen.append(st)
    return st


def wait_for_state(repl, target_states: set[str], transitions_seen: list,
                   timeout_s: float, log: StepLog) -> str:
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
        "stop_dist_m":      STOP_DIST_M,
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

    # ─── Phase 3: cf2 takeoff at origin + CAPTURE PHYSICAL_ORIGIN ───
    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m @ {TAKEOFF_VEL_MPS} m/s"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        # PHYSICAL_ORIGIN — captured ONCE, used as the only truth for
        # closure verdict per [[sim-test-must-return-home]].
        physical_origin = (tel['x'], tel['y'])
        summary["physical_origin"] = list(physical_origin)
        summary["physical_origin_z"] = tel['z']
        log.info(f"        post-takeoff cf2 @ ({tel['x']:+.3f},"
                 f"{tel['y']:+.3f},{tel['z']:.3f}) yaw={tel['yaw_deg']:+.1f}")
        log.info(f"        PHYSICAL_ORIGIN captured: "
                 f"({physical_origin[0]:+.3f},{physical_origin[1]:+.3f})")

    # ─── Phase 4: seed lifter for marker id=0 (drone still at origin) ───
    with log.step("lifter init_from_bbox for id=0 (LIFTED via near-marker prior)"):
        bbox = sample_marker_bbox(repl.frames_dir, TARGET_ARUCO_ID,
                                   -1, log)
        if bbox is None:
            raise RuntimeError(
                f"id{TARGET_ARUCO_ID} not visible at takeoff — abort")
        tel = tel_snapshot()
        drone_W = (tel['x'], tel['y'], tel['z'])
        yaw = math.radians(tel['yaw_deg'])
        rc = repl.exec_int(
            f"sentai.object_lifter.init_from_bbox(0, 0, "
            f"{bbox['u']:.2f}, {bbox['v']:.2f}, 45.0, {MARKER_SIZE_M}, "
            f"({drone_W[0]:.4f},{drone_W[1]:.4f},{drone_W[2]:.4f}), {yaw:.4f})")
        if rc < 0:
            raise RuntimeError(f"lifter init_from_bbox rc={rc}")
        snap = repl.exec_repr("sentai.object_lifter.get(0)")
        wpos = repl.exec_repr("sentai.object_lifter.world_pos(0)")
        log.info(f"        post-init: status={snap['status']} rho={snap['rho']:.3f}")
        log.info(f"        world_pos(0)={wpos}  GT={TARGET_WORLD_XYZ}")
        summary["lifter_world_pos"] = list(wpos) if wpos else None
        if snap["status"] != 2:
            raise RuntimeError(
                f"lifter did not reach LIFTED (status={snap['status']})")

    # ─── Phase 5: explore.start AT ORIGIN (NO displacement!) ───
    # CRITICAL: no mc.move_distance between cf2.take_off and this point.
    # L6.home will be captured at the current pose ≈ PHYSICAL_ORIGIN.
    with log.step("explore.start → ARMING (home will = PHYSICAL_ORIGIN)"):
        rc = repl.exec_int("sentai.explore.start()")
        if rc != 0:
            raise RuntimeError(f"explore.start rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step(f"explore.takeoff({TAKEOFF_Z_M}) → TAKEOFF → HOVERING"):
        rc = repl.exec_int(f"sentai.explore.takeoff({TAKEOFF_Z_M})")
        if rc != 0:
            raise RuntimeError(f"explore.takeoff rc={rc}")
        st = wait_for_state(repl, {"HOVERING"}, transitions_seen,
                            timeout_s=5.0, log=log)
        if st != "HOVERING":
            raise RuntimeError(f"never reached HOVERING (got '{st}')")
        m = repl.exec_repr("sentai.explore.metrics()")
        l6_home = (m["home_x"], m["home_y"])
        summary["l6_home"] = list(l6_home)
        l6_vs_origin = math.hypot(l6_home[0] - physical_origin[0],
                                   l6_home[1] - physical_origin[1])
        summary["l6_home_vs_origin_m"] = l6_vs_origin
        log.info(f"        L6 home @ ({l6_home[0]:+.3f},{l6_home[1]:+.3f}); "
                 f"vs PHYSICAL_ORIGIN diff={l6_vs_origin*100:.1f} cm")
        if l6_vs_origin > 0.10:
            log.info(f"        WARN: L6 home off origin by {l6_vs_origin*100:.1f} cm")

    # ─── Phase 6: explore.goto(0) → APPROACH → INSPECT → HOVERING ───
    with log.step(f"explore.goto(0, {STOP_DIST_M}) → APPROACH"):
        rc = repl.exec_int(f"sentai.explore.goto(0, {STOP_DIST_M})")
        if rc != 0:
            raise RuntimeError(f"explore.goto rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        m = repl.exec_repr("sentai.explore.metrics()")
        tgt_xy = (m["target_x"], m["target_y"])
        log.info(f"        state='{st}'; target=({tgt_xy[0]:+.3f},"
                 f"{tgt_xy[1]:+.3f})")
        summary["goto_target_xy"] = list(tgt_xy)

    with log.step(f"cf2 mc.move_distance toward target_xy"):
        tel = tel_snapshot()
        dx = tgt_xy[0] - tel['x']
        dy = tgt_xy[1] - tel['y']
        dist = math.hypot(dx, dy)
        if dist > STOP_DIST_M:
            step_len = dist - STOP_DIST_M
            step_x = dx * step_len / dist
            step_y = dy * step_len / dist
            log.info(f"        moving cf2 dx={step_x:+.3f} dy={step_y:+.3f}")
            mc.move_distance(step_x, step_y, 0.0, velocity=APPROACH_VEL_MPS)
        else:
            log.info(f"        already within stop_dist (d={dist:.3f})")
        time.sleep(0.5)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(0.5)

    with log.step("pose loop → INSPECT/HOVERING"):
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

    # ─── Phase 7: explore.return_home → RETURNING → HOVERING ───
    with log.step("explore.return_home() → RETURNING"):
        rc = repl.exec_int("sentai.explore.return_home()")
        if rc != 0:
            raise RuntimeError(f"return_home rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step("cf2 mc.move_distance back toward PHYSICAL_ORIGIN"):
        tel = tel_snapshot()
        # Drive cf2 back to PHYSICAL_ORIGIN (not L6 home — they should
        # coincide here, but PHYSICAL_ORIGIN is the truth).
        dx = physical_origin[0] - tel['x']
        dy = physical_origin[1] - tel['y']
        log.info(f"        moving cf2 dx={dx:+.3f} dy={dy:+.3f}")
        mc.move_distance(dx, dy, 0.0, velocity=APPROACH_VEL_MPS)
        time.sleep(0.5)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(0.5)

    with log.step("pose loop → HOVERING (origin reached)"):
        st = wait_for_state(repl, {"HOVERING"}, transitions_seen,
                            timeout_s=10.0, log=log)
        if st != "HOVERING":
            raise RuntimeError(f"never returned to HOVERING (got '{st}')")

    # ─── Phase 8: explore.land → LANDING → DONE ───
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

    # ─── Phase 9: closure check vs PHYSICAL_ORIGIN ───
    with log.step("final metrics + closure check vs PHYSICAL_ORIGIN"):
        m = repl.exec_repr("sentai.explore.metrics()")
        trace = repl.exec_repr("sentai.explore.trace()")
        servo_st = repl.exec_repr("sentai.servo.status()")

        tel = tel_snapshot()
        # Land error vs PHYSICAL_ORIGIN — the only truth per
        # [[sim-test-must-return-home]].
        land_err_xy_vs_origin = math.hypot(
            tel['x'] - physical_origin[0],
            tel['y'] - physical_origin[1])
        summary["land_err_xy_vs_origin_m"] = land_err_xy_vs_origin
        summary["land_pose"] = [tel['x'], tel['y'], tel['z']]
        summary["transitions_seen"] = transitions_seen
        summary["explore_metrics_final"] = m
        summary["explore_trace_count"] = len(trace) if trace else 0
        summary["servo_status_final"] = servo_st
        summary["mission_duration_s"] = time.monotonic() - t_mission_start

        log.info(f"        PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
                 f"{physical_origin[1]:+.3f})")
        log.info(f"        LAND_POSE       = ({tel['x']:+.3f},{tel['y']:+.3f},"
                 f"{tel['z']:.3f})")
        log.info(f"        land_err_xy_vs_origin = {land_err_xy_vs_origin*100:.1f} cm")
        log.info(f"        transitions = {transitions_seen}")
        log.info(f"        aborts={m['aborts']}  gotos={m['gotos_completed']}"
                 f"  transitions_counter={m['transitions']}")
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

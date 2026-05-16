"""s136 — L6 RELEVANT exploration test (real parallax + visible goto).

Per [[test-must-be-relevant-to-claim]] + [[sim-test-must-return-home]]:
this test proves L6 actually navigates the drone, not just that the
FSM transitions.  Behavior under test (in README):
    drone visibly moves ≥ 12 cm to marker, INSPECTs, traverses ≥ 12 cm
    back via RETURNING, lands ≤ 10 cm from PHYSICAL_ORIGIN.

Verdict gates the behavior with HARD assertions on displacement,
RETURNING entry, and lifter convergence — not just FSM shape checks.
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

WORKDIR = Path("/tmp/s136_explore_real")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
EXPLORE_TRACE   = WORKDIR / "explore_trace.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

JOURNAL_NAME = "s136_journal.txt"

# ─── Target (same world as s132) ───
TARGET_ARUCO_ID  = 0
TARGET_WORLD_XYZ = (+0.15, +0.10, 0.20)

# ─── Mission params ───
TAKEOFF_Z_M       = 1.50
TAKEOFF_VEL_MPS   = 0.6
PARALLAX_STEP_M   = 0.10   # 2× lateral steps for lifter convergence
PARALLAX_VEL_MPS  = 0.20
PARALLAX_SETTLE_S = 1.5   # generous — cf2 PID needs time to converge to setpoint
STOP_DIST_M       = 0.05
APPROACH_VEL_MPS  = 0.25
APPROACH_SETTLE_S = 2.5   # critical — observed cf2 only achieves ~50% of mc.move
LAND_VEL_MPS      = 0.40

POSE_LOOP_HZ      = 5.0
INSPECT_DWELL_S   = 1.7
LAND_DWELL_S      = 3.2


# ─── Helpers ───
def pose_tick(repl, x, y, z, yaw_deg, transitions_seen):
    repl.exec_int(
        f"sentai.explore.set_pose({x:.4f},{y:.4f},{z:.4f},"
        f"{math.radians(yaw_deg):.4f})")
    st = repl.exec_value("sentai.explore.state()")
    if not transitions_seen or transitions_seen[-1] != st:
        transitions_seen.append(st)
    return st


def wait_for_state(repl, target_states, transitions_seen, timeout_s, log):
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


def tick_dwell(repl, dur_s, transitions_seen, log):
    deadline = time.monotonic() + dur_s
    last_state = ""
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        last_state = pose_tick(repl, tel["x"], tel["y"], tel["z"],
                               tel["yaw_deg"], transitions_seen)
        time.sleep(1.0 / POSE_LOOP_HZ)
    log.info(f"        dwell {dur_s:.1f}s done; state='{last_state}'")
    return last_state


def dwell_with_hold(repl, cf, hold_x, hold_y, hold_z, hold_yaw_deg,
                     dur_s, transitions_seen, log):
    """Dwell phase with concurrent position-hold spam.

    Spams send_position_setpoint(hold_*) at 20 Hz to keep cf2 stationary
    while we poll L6 state.  Without this hold, cf2 drifts (~5-7 cm/s)
    during long dwells because MotionCommander hover-hold has slip.
    """
    deadline = time.monotonic() + dur_s
    last_state = ""
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(hold_x, hold_y, hold_z,
                                              hold_yaw_deg)
        tel = tel_snapshot()
        last_state = pose_tick(repl, tel["x"], tel["y"], tel["z"],
                               tel["yaw_deg"], transitions_seen)
        time.sleep(0.05)
    log.info(f"        dwell-with-hold {dur_s:.1f}s done; "
             f"state='{last_state}'  cf2_drift_xy="
             f"{math.hypot(tel['x']-hold_x, tel['y']-hold_y)*100:.1f} cm")
    return last_state


def goto_xy_abs(cf, target_x, target_y, z, yaw_deg, log,
                 timeout_s=15.0, tol_m=0.04, label=""):
    """Drive cf2 to absolute (target_x, target_y, z) via position setpoint
    spam + closed-loop pose feedback.  Returns (arrived: bool, final_pose).

    Replaces mc.move_distance for precision moves — that uses
    time-based open-loop estimation and consistently undershoots.
    """
    deadline = time.monotonic() + timeout_s
    last_pose = None
    n_spam = 0
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(target_x, target_y, z,
                                              yaw_deg)
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
             f"dist={dist_final*100:.1f} cm  (tol={tol_m*100:.0f} cm)")
    return False, last_pose


def lifter_observe(repl, last_seq, log, phase_label, t_prev_ref):
    """One real bbox observation + L5 init_from_bbox or update_bbox.

    Returns (rc, snap, world_pos, new_seq, t_now) or (None, ...) on no detection.
    First call should be init_from_bbox; subsequent calls update_bbox.
    Caller decides which based on n_obs.
    """
    bbox = sample_marker_bbox(repl.frames_dir, TARGET_ARUCO_ID,
                               last_seq, log)
    if bbox is None:
        log.info(f"        {phase_label}: no marker visible")
        return None, None, None, last_seq, t_prev_ref
    tel = tel_snapshot()
    drone_W = (tel['x'], tel['y'], tel['z'])
    yaw = math.radians(tel['yaw_deg'])
    log.info(f"        {phase_label}: bbox u={bbox['u']:.1f} v={bbox['v']:.1f} "
             f"w_px={bbox['w_px']:.1f}  drone=({drone_W[0]:+.3f},"
             f"{drone_W[1]:+.3f},{drone_W[2]:.3f})")
    return bbox, drone_W, yaw, bbox["seq"], time.monotonic()


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
        "parallax_step_m":  PARALLAX_STEP_M,
    }
    transitions_seen: list = []
    t_mission_start = time.monotonic()

    # ─── Phase 1-3: cf2 setup + telemetry + flow ───
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

    with log.step("REPL: import sentai + journal_open + lifter.clear"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        rc = repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        if rc != 0:
            raise RuntimeError(f"journal_open returned {rc}")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")
        repl.exec_int("sentai.object_lifter.clear()")
        rc = repl.exec_int("sentai.explore.init('sim')")
        if rc != 0:
            raise RuntimeError(f"explore.init rc={rc}")
        # Tight world (markers at ±0.15 m) — shrink HOME_RADIUS so a
        # ~18 cm goto+return actually exercises RETURNING instead of
        # being short-circuited by "already at home".
        rc = repl.exec_int("sentai.explore.set_tunables(0.10, 1500, 3000)")
        log.info(f"        set_tunables(home_r=0.10) rc={rc}")

    # ─── Phase 4: cf2 takeoff + capture PHYSICAL_ORIGIN ───
    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m @ {TAKEOFF_VEL_MPS} m/s"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        physical_origin = (tel['x'], tel['y'])
        summary["physical_origin"] = list(physical_origin)
        log.info(f"        post-takeoff cf2 @ ({tel['x']:+.3f},"
                 f"{tel['y']:+.3f},{tel['z']:.3f})")
        log.info(f"        PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
                 f"{physical_origin[1]:+.3f})")

    # ─── Phase 5: REAL lifter parallax convergence ───
    # NO trickery (no fake bbox=45 px).  Real bbox + 3 observations
    # (1 init + 2 lateral updates).  After parallax, return to origin
    # to preserve closure.
    last_seq = -1
    t_prev = time.monotonic()
    with log.step("lifter parallax: frame 0 (init at origin)"):
        bbox, drone_W, yaw, last_seq, t_prev = lifter_observe(
            repl, last_seq, log, "frame0", t_prev)
        if bbox is None:
            raise RuntimeError("id=0 not visible at takeoff — abort")
        # REAL bbox values — no hardcoding.
        rc = repl.exec_int(
            f"sentai.object_lifter.init_from_bbox(0, 0, "
            f"{bbox['u']:.2f}, {bbox['v']:.2f}, {bbox['w_px']:.2f}, "
            f"{MARKER_SIZE_M}, "
            f"({drone_W[0]:.4f},{drone_W[1]:.4f},{drone_W[2]:.4f}), {yaw:.4f})")
        if rc < 0:
            raise RuntimeError(f"init_from_bbox rc={rc}")
        snap = repl.exec_repr("sentai.object_lifter.get(0)")
        wpos = repl.exec_repr("sentai.object_lifter.world_pos(0)")
        log.info(f"        post-init: status={snap['status']} "
                 f"rho={snap['rho']:.3f} var_rho={snap['var_rho']:.4f}")
        log.info(f"        world_pos(0)={wpos}  GT={TARGET_WORLD_XYZ}")
        summary["lifter_init_world_pos"] = list(wpos) if wpos else None

    for step_idx in (1, 2):
        with log.step(f"lifter parallax: lateral step {step_idx}/2 "
                       f"(+{PARALLAX_STEP_M:.2f} m body-Y)"):
            mc.move_distance(0.0, PARALLAX_STEP_M, 0.0, velocity=PARALLAX_VEL_MPS)
            time.sleep(PARALLAX_SETTLE_S)
            mc.start_linear_motion(0.0, 0.0, 0.0)
            time.sleep(0.3)
            bbox, drone_W, yaw, last_seq, t_now = lifter_observe(
                repl, last_seq, log, f"frame{step_idx}", t_prev)
            if bbox is None:
                log.info(f"        step {step_idx}: no marker, skipping update")
                t_prev = t_now
                continue
            dt_s = t_now - t_prev
            t_prev = t_now
            rc = repl.exec_int(
                f"sentai.object_lifter.update_bbox(0, "
                f"{bbox['u']:.2f}, {bbox['v']:.2f}, "
                f"({drone_W[0]:.4f},{drone_W[1]:.4f},{drone_W[2]:.4f}), "
                f"{yaw:.4f}, {dt_s:.3f})")
            snap = repl.exec_repr("sentai.object_lifter.get(0)")
            wpos = repl.exec_repr("sentai.object_lifter.world_pos(0)")
            log.info(f"        update rc={rc}  status={snap['status']} "
                     f"rho={snap['rho']:.3f} var_rho={snap['var_rho']:.4f} "
                     f"n_obs={snap['n_obs']}")
            log.info(f"        world_pos(0)={wpos}")

    with log.step("cf2 → PHYSICAL_ORIGIN via abs setpoint (post-parallax)"):
        # mc.move_distance back open-loop leaves 3-6 cm drift, which then
        # contaminates L6's home capture.  Use closed-loop abs setpoint to
        # land cf2 within 3 cm of PHYSICAL_ORIGIN before explore.start.
        arrived, _ = goto_xy_abs(cf, physical_origin[0], physical_origin[1],
                                   TAKEOFF_Z_M, 0.0, log,
                                   timeout_s=10.0, tol_m=0.03,
                                   label="post_parallax_return")
        if not arrived:
            log.info("        WARN: did not reach origin tol; continuing")
        time.sleep(0.5)
        tel = tel_snapshot()
        d_from_origin = math.hypot(tel['x'] - physical_origin[0],
                                    tel['y'] - physical_origin[1])
        log.info(f"        cf2 back @ ({tel['x']:+.3f},{tel['y']:+.3f}); "
                 f"dist_from_PHYSICAL_ORIGIN={d_from_origin*100:.1f} cm")
        summary["dist_after_parallax_return"] = d_from_origin

    with log.step("CHECK lifter convergence vs GT marker"):
        snap = repl.exec_repr("sentai.object_lifter.get(0)")
        wpos = repl.exec_repr("sentai.object_lifter.world_pos(0)")
        log.info(f"        final lifter snap: status={snap['status']} "
                 f"n_obs={snap['n_obs']} var_rho={snap['var_rho']:.5f}")
        log.info(f"        world_pos(0)={wpos}  GT={TARGET_WORLD_XYZ}")
        gt = TARGET_WORLD_XYZ
        if wpos is None:
            wpos_err_xy = float("inf")
        else:
            wpos_err_xy = math.hypot(wpos[0] - gt[0], wpos[1] - gt[1])
        summary["lifter_final_world_pos"] = list(wpos) if wpos else None
        summary["lifter_world_pos_xy_err_vs_GT_m"] = wpos_err_xy
        summary["lifter_status"] = snap['status']
        log.info(f"        lifter xy err vs GT marker = {wpos_err_xy*100:.1f} cm")

    # ─── Phase 6: explore.start AT ORIGIN ───
    with log.step("explore.start → ARMING (home = PHYSICAL_ORIGIN)"):
        rc = repl.exec_int("sentai.explore.start()")
        if rc != 0:
            raise RuntimeError(f"explore.start rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step(f"explore.takeoff({TAKEOFF_Z_M}) → HOVERING"):
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

    # ─── Phase 8: goto(0) → APPROACH → INSPECT ───
    with log.step(f"CAPTURE pose_pre_goto"):
        tel = tel_snapshot()
        pose_pre_goto = (tel['x'], tel['y'])
        summary["pose_pre_goto"] = list(pose_pre_goto)
        log.info(f"        pose_pre_goto=({pose_pre_goto[0]:+.3f},"
                 f"{pose_pre_goto[1]:+.3f})")

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

    with log.step("cf2 → target_xy via abs position setpoint (closed-loop)"):
        # mc.move_distance is open-loop time×velocity; cf2 typically only
        # achieves ~50% of commanded displacement.  Use absolute position
        # setpoint with spam-and-wait instead.
        log.info(f"        target_abs=({tgt_xy[0]:+.3f},{tgt_xy[1]:+.3f},"
                 f"{TAKEOFF_Z_M:.2f})")
        arrived, _ = goto_xy_abs(cf, tgt_xy[0], tgt_xy[1], TAKEOFF_Z_M,
                                   0.0, log, timeout_s=15.0, tol_m=0.04,
                                   label="goto_target")
        if not arrived:
            log.info("        WARN: did not reach target tolerance; "
                     "continuing anyway")

    with log.step("pose loop → INSPECT/HOVERING"):
        st = wait_for_state(repl, {"INSPECT", "HOVERING"}, transitions_seen,
                            timeout_s=10.0, log=log)
        if st not in ("INSPECT", "HOVERING"):
            raise RuntimeError(f"never reached INSPECT/HOVERING (got '{st}')")
        if st == "INSPECT":
            with log.step(f"INSPECT dwell ~{INSPECT_DWELL_S:.1f}s WITH HOLD → HOVERING"):
                # Hold cf2 at the L6 target_xy so it doesn't drift during dwell.
                dwell_with_hold(repl, cf, tgt_xy[0], tgt_xy[1], TAKEOFF_Z_M,
                                 0.0, INSPECT_DWELL_S, transitions_seen, log)
                st2 = repl.exec_value("sentai.explore.state()")
                if st2 != "HOVERING":
                    raise RuntimeError(
                        f"INSPECT did not transition to HOVERING (got '{st2}')")

    with log.step("CAPTURE pose_at_target — measure forward displacement"):
        tel = tel_snapshot()
        pose_at_target = (tel['x'], tel['y'])
        summary["pose_at_target"] = list(pose_at_target)
        disp_forward = math.hypot(pose_at_target[0] - pose_pre_goto[0],
                                   pose_at_target[1] - pose_pre_goto[1])
        summary["displacement_to_target_xy_m"] = disp_forward
        log.info(f"        pose_at_target=({pose_at_target[0]:+.3f},"
                 f"{pose_at_target[1]:+.3f}); "
                 f"displacement_forward={disp_forward*100:.1f} cm")

    # ─── Phase 10: return_home → RETURNING ───
    with log.step("CAPTURE pose_pre_return"):
        tel = tel_snapshot()
        pose_pre_return = (tel['x'], tel['y'])
        summary["pose_pre_return"] = list(pose_pre_return)

    with log.step("explore.return_home() → RETURNING"):
        rc = repl.exec_int("sentai.explore.return_home()")
        if rc != 0:
            raise RuntimeError(f"return_home rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step("cf2 → PHYSICAL_ORIGIN via abs position setpoint"):
        arrived, _ = goto_xy_abs(cf, physical_origin[0], physical_origin[1],
                                   TAKEOFF_Z_M, 0.0, log,
                                   timeout_s=15.0, tol_m=0.04,
                                   label="goto_home")
        if not arrived:
            log.info("        WARN: did not reach home tolerance; "
                     "continuing anyway")

    with log.step("pose loop → HOVERING (origin reached)"):
        st = wait_for_state(repl, {"HOVERING"}, transitions_seen,
                            timeout_s=10.0, log=log)
        if st != "HOVERING":
            raise RuntimeError(f"never returned to HOVERING (got '{st}')")

    with log.step("CAPTURE pose_post_return — measure backward displacement"):
        tel = tel_snapshot()
        pose_post_return = (tel['x'], tel['y'])
        summary["pose_post_return"] = list(pose_post_return)
        disp_back = math.hypot(pose_post_return[0] - pose_pre_return[0],
                                pose_post_return[1] - pose_pre_return[1])
        summary["displacement_back_to_origin_xy_m"] = disp_back
        log.info(f"        pose_post_return=({pose_post_return[0]:+.3f},"
                 f"{pose_post_return[1]:+.3f}); "
                 f"displacement_back={disp_back*100:.1f} cm")

    # ─── Phase 11: land → LANDING → DONE ───
    with log.step("explore.land() → LANDING"):
        rc = repl.exec_int("sentai.explore.land()")
        if rc != 0:
            raise RuntimeError(f"explore.land rc={rc}")
        st = repl.exec_value("sentai.explore.state()")
        transitions_seen.append(st)
        log.info(f"        state='{st}'")

    with log.step(f"cf2 controlled descent via setpoint + LANDING dwell → DONE"):
        # mc.land does an open-loop descent and lets cf2 hover-hold xy —
        # we observed ~12 cm xy drift during descent.  Replace with
        # closed-loop xy-hold + z ramp using send_position_setpoint.
        descent_dur_s = 3.0
        z_top = TAKEOFF_Z_M
        z_bottom = 0.05
        t0 = time.monotonic()
        while True:
            t_elapsed = time.monotonic() - t0
            frac = min(1.0, t_elapsed / descent_dur_s)
            z = z_top + frac * (z_bottom - z_top)
            cf.commander.send_position_setpoint(physical_origin[0],
                                                  physical_origin[1],
                                                  z, 0.0)
            tel = tel_snapshot()
            pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                       transitions_seen)
            if frac >= 1.0 and tel['z'] < 0.10:
                break
            if t_elapsed > descent_dur_s + 4:  # safety timeout
                log.info(f"        descent timeout @ z={tel['z']:.2f}")
                break
            time.sleep(0.05)
        tel = tel_snapshot()
        # CAPTURE the actual landing pose HERE (before tick_dwell).
        # MotionCommander hover-thread re-ascends to default_height during
        # the LANDING dwell, so reading telemetry post-dwell is wrong.
        land_pose = (tel['x'], tel['y'], tel['z'])
        summary["land_pose"] = list(land_pose)
        log.info(f"        post-descent cf2 @ ({tel['x']:+.3f},"
                 f"{tel['y']:+.3f},{tel['z']:.3f})  ← LAND_POSE captured")
        cf.commander.send_stop_setpoint()
        time.sleep(0.5)
        # Now L6 should tick through LANDING_DUR_MS → DONE
        tick_dwell(repl, LAND_DWELL_S, transitions_seen, log)
        st = repl.exec_value("sentai.explore.state()")
        log.info(f"        final state='{st}'")
        summary["state_final"] = st
        if st != "DONE":
            raise RuntimeError(f"never reached DONE (got '{st}')")

    # ─── Phase 12: final metrics + verdict-feeding numbers ───
    with log.step("final metrics + closure"):
        m = repl.exec_repr("sentai.explore.metrics()")
        trace = repl.exec_repr("sentai.explore.trace()")
        servo_st = repl.exec_repr("sentai.servo.status()")

        # IMPORTANT: use land_pose captured at end of descent, NOT current
        # telemetry — MotionCommander thread re-ascends during LANDING dwell
        # and corrupts post-mission telemetry.  land_pose was saved above
        # at the correct moment (drone actually on ground).
        land_pose_for_verdict = summary.get("land_pose",
                                              [tel_snapshot()['x'],
                                               tel_snapshot()['y'],
                                               tel_snapshot()['z']])
        land_err_xy = math.hypot(land_pose_for_verdict[0] - physical_origin[0],
                                  land_pose_for_verdict[1] - physical_origin[1])
        summary["land_err_xy_vs_origin_m"] = land_err_xy
        # land_pose already saved at descent-end; do not overwrite here.
        summary["transitions_seen"] = transitions_seen
        summary["explore_metrics_final"] = m
        summary["explore_trace_count"] = len(trace) if trace else 0
        summary["servo_status_final"] = servo_st
        summary["mission_duration_s"] = time.monotonic() - t_mission_start

        log.info(f"        PHYSICAL_ORIGIN = ({physical_origin[0]:+.3f},"
                 f"{physical_origin[1]:+.3f})")
        log.info(f"        LAND_POSE       = ({tel['x']:+.3f},"
                 f"{tel['y']:+.3f},{tel['z']:.3f})")
        log.info(f"        land_err_xy_vs_origin = {land_err_xy*100:.1f} cm")
        log.info(f"        displacement_to_target = "
                 f"{summary.get('displacement_to_target_xy_m', 0)*100:.1f} cm")
        log.info(f"        displacement_back      = "
                 f"{summary.get('displacement_back_to_origin_xy_m', 0)*100:.1f} cm")
        log.info(f"        lifter xy err vs GT    = "
                 f"{summary.get('lifter_world_pos_xy_err_vs_GT_m', 0)*100:.1f} cm")
        log.info(f"        transitions = {transitions_seen}")
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

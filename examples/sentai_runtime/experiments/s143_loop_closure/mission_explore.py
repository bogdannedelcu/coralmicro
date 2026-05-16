"""s143 — Loop closure: drone stores descriptors in lap-1, then
revisits target1 in lap-2 and queries L3 gallery, verifying match.

First mission where drone CONSUMES the memory it built.

Outputs under /tmp/s143_loop_closure/.
"""
from __future__ import annotations

import datetime as dt
import json
import math
import shutil
import sys
import threading
import time
from pathlib import Path

S091_DIR = Path(__file__).resolve().parent.parent / "s091_aruco_lowalt"
S128_DIR = Path(__file__).resolve().parent.parent / "s128_l41baseline_seeded"
S132_DIR = Path(__file__).resolve().parent.parent / "s132_lifter_gazebo"
S142_DIR = Path(__file__).resolve().parent.parent / "s142_hex_descriptor_patrol"
for p in (S091_DIR, S128_DIR, S132_DIR, S142_DIR):
    sys.path.insert(0, str(p))

import aruco_hover                                       # noqa: E402
from mission_l41 import ReplDriver, StepLog              # noqa: E402
from mission_lifter import _tel_cb, tel_snapshot, _tel_samples  # noqa: E402

WORKDIR = Path("/tmp/s143_loop_closure")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"
HEX_HELPERS_SRC = (REPO_ROOT / "examples" / "sentai_runtime" /
                   "experiments" / "s142_hex_descriptor_patrol" /
                   "hex_helpers.py")

JOURNAL_NAME = "s143_journal.txt"

TARGETS = [
    (10, 1, +1.50, 0.00, 1.40, 1),   # target 1
    (11, 1, -0.50, +1.00, 1.40, 2),  # target 2
]

TAKEOFF_Z_M       = 1.50
TAKEOFF_VEL_MPS   = 0.6
STOP_DIST_M       = 0.10
APPROACH_VEL_MPS  = 0.5
LAND_VEL_MPS      = 0.40

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


def goto_xy_abs(cf, tx, ty, tz, yaw, log, timeout_s=20.0, tol_m=0.08, label=""):
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(tx, ty, tz, yaw)
        tel = tel_snapshot()
        last = (tel['x'], tel['y'], tel['z'])
        if math.hypot(tel['x'] - tx, tel['y'] - ty) <= tol_m:
            log.info(f"        {label}: arrived @ "
                     f"({last[0]:+.3f},{last[1]:+.3f},{last[2]:.3f})")
            return True, last
        time.sleep(0.05)
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
    log.info(f"        TIMEOUT (last='{last}')")
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


def tick_dwell(repl, dur_s, transitions_seen):
    deadline = time.monotonic() + dur_s
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                   transitions_seen)
        time.sleep(1.0 / POSE_LOOP_HZ)


def store_at(repl, log, seed, x, y, z, label):
    rc = repl.exec_int(
        f"hex_helpers.capture_and_store({seed}, {x:.4f}, {y:.4f}, {z:.4f})")
    log.info(f"        store {label} seed={seed} @ "
             f"({x:+.3f},{y:+.3f}) → place_id={rc}")
    if rc <= 0:
        raise RuntimeError(f"store {label} returned {rc}")
    return rc


def query_at(repl, log, seed, x, y, z, label):
    """Compute descriptor at this pose, query L3 (NO store).
    Returns dict {id, score_pct, l1_dist} or None.
    """
    # Build query inline: image → phog → gist → quantize → cell → query.
    cmd = (
        f"_q_seed={seed}; _q_x={x:.4f}; _q_y={y:.4f};\n"
        f"_q_img = hex_helpers.hex_image(_q_seed);\n"
        f"_q_phog = sentai.places.compute_phog(_q_img, hex_helpers.W, hex_helpers.H);\n"
        f"_q_gist = sentai.places.compute_gist(_q_img, hex_helpers.W, hex_helpers.H);\n"
        f"_q_desc = hex_helpers.quantize(_q_phog, _q_gist);\n"
        f"_q_cell = hex_helpers.xy_to_h3(_q_x, _q_y, 15);\n"
        f"_q_res = sentai.places.query(_q_desc, _q_cell, 1, 0)\n"
    )
    # Push as multiline via fs.write + exec — simpler: use compound on one line
    one_line = (
        f"sentai.places.query("
        f"hex_helpers.quantize("
        f"sentai.places.compute_phog(hex_helpers.hex_image({seed}), hex_helpers.W, hex_helpers.H),"
        f"sentai.places.compute_gist(hex_helpers.hex_image({seed}), hex_helpers.W, hex_helpers.H)),"
        f"hex_helpers.xy_to_h3({x:.4f}, {y:.4f}, 15), 1, 0)"
    )
    res = repl.exec_repr(one_line)
    log.info(f"        query {label} seed={seed} @ ({x:+.3f},{y:+.3f}) → {res}")
    return res


def fly(log: StepLog, repl: ReplDriver) -> dict:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    cflib.crtp.init_drivers()
    summary: dict = {"targets": [list(t) for t in TARGETS]}
    transitions_seen: list = []
    t_start = time.monotonic()

    # ─── cf2 setup ───
    with log.step("cf2 link + params + Kalman"):
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

    with log.step("telemetry + flow"):
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

    with log.step("REPL init + hex_helpers"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")
        repl.exec_int("sentai.places.clear()")
        repl.exec_int("sentai.object_lifter.clear()")
        rc = repl.exec_int("sentai.explore.init('sim')")
        if rc != 0: raise RuntimeError(f"explore.init rc={rc}")
        repl.exec_int("sentai.explore.set_tunables(0.15, 1500, 3000)")
        repl.exec("import hex_helpers")

    with log.step(f"cf2 take_off"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        origin = (tel['x'], tel['y'])
        summary["physical_origin"] = list(origin)
        log.info(f"        PHYSICAL_ORIGIN = ({origin[0]:+.3f},{origin[1]:+.3f})")

    # ─── LAP 1 (build memory) ───
    log.info("=== LAP 1 (build memory) ===")
    pid_home = store_at(repl, log, 0, origin[0], origin[1], tel['z'], "HOME")
    summary["pid_home"] = pid_home

    with log.step("L5 inject 2 targets"):
        for (tid, cls, x, y, z, _seed) in TARGETS:
            repl.exec_int(
                f"sentai.object_lifter.inject({tid}, {cls}, {x}, {y}, {z})")

    with log.step("explore.start + takeoff → HOVERING"):
        repl.exec_int("sentai.explore.start()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        repl.exec_int(f"sentai.explore.takeoff({TAKEOFF_Z_M})")
        wait_for_state(repl, {"HOVERING"}, transitions_seen, 5.0, log)

    pids = []
    for (tid, cls, tx, ty, tz, seed) in TARGETS:
        with log.step(f"LAP1: goto({tid}) seed={seed}"):
            repl.exec_int(f"sentai.explore.goto({tid}, {STOP_DIST_M})")
            transitions_seen.append(repl.exec_value("sentai.explore.state()"))
            m = repl.exec_repr("sentai.explore.metrics()")
            tgt = (m["target_x"], m["target_y"])
            goto_xy_abs(cf, tgt[0], tgt[1], TAKEOFF_Z_M, 0.0, log,
                         label=f"goto_{tid}_lap1")
            st = wait_for_state(repl, {"INSPECT", "HOVERING"},
                                 transitions_seen, 10.0, log)
            if st == "INSPECT":
                dwell_with_hold(repl, cf, tgt[0], tgt[1], TAKEOFF_Z_M, 0.0,
                                 INSPECT_DWELL_S, transitions_seen, log)
            tel_now = tel_snapshot()
            pid = store_at(repl, log, seed, tel_now['x'], tel_now['y'],
                           tel_now['z'], f"T{tid}")
            pids.append((tid, pid, seed))
    summary["pids_lap1"] = pids

    # ─── BACK TO HOME between laps ───
    with log.step("LAP1 done → return to origin"):
        repl.exec_int("sentai.explore.return_home()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        goto_xy_abs(cf, origin[0], origin[1], TAKEOFF_Z_M, 0.0, log,
                     label="return_after_lap1")
        wait_for_state(repl, {"HOVERING"}, transitions_seen, 12.0, log)

    summary["gallery_count_after_lap1"] = repl.exec_int("sentai.places.count()")
    log.info(f"        gallery count after lap1 = {summary['gallery_count_after_lap1']}")

    # ─── LAP 2 (consume memory — revisit target 1, query) ───
    log.info("=== LAP 2 (consume memory) ===")
    target1 = TARGETS[0]
    t1_seed = target1[5]
    t1_pid = pids[0][1]    # pid_t1 from lap-1

    # We need to navigate again — but L6 already used goto(10).
    # Reset target lock or just drive cf2 directly to t1 xy.
    with log.step(f"LAP2: cf2 → target1 ({target1[2]:+.2f},{target1[3]:+.2f}) directly"):
        goto_xy_abs(cf, target1[2], target1[3], TAKEOFF_Z_M, 0.0, log,
                     label="lap2_goto_t1")
        # Hold position briefly for stable pose snapshot
        for _ in range(20):
            cf.commander.send_position_setpoint(target1[2], target1[3],
                                                  TAKEOFF_Z_M, 0.0)
            time.sleep(0.05)
        tel_now = tel_snapshot()

    with log.step("LAP2: QUERY L3 (no store) — does drone recognize the place?"):
        # Re-use seed=t1_seed (deterministic synthetic image).  In real
        # mission, this would be a fresh camera frame.
        res = query_at(repl, log, t1_seed, tel_now['x'], tel_now['y'],
                        tel_now['z'], "lap2_t1")
        match_id = res.get('id', 0) if res else 0
        match_score = res.get('score_pct', 0) if res else 0
        match_l1 = res.get('l1_dist', 0) if res else 0
        summary["match_lap2_id"] = match_id
        summary["match_lap2_score"] = match_score
        summary["match_lap2_l1"] = match_l1
        summary["expected_match_id"] = t1_pid
        log.info(f"        MATCH: id={match_id} (expected {t1_pid}), "
                 f"score={match_score}%, l1_dist={match_l1}")

    summary["gallery_count_after_lap2"] = repl.exec_int("sentai.places.count()")
    log.info(f"        gallery count after lap2 = {summary['gallery_count_after_lap2']}")

    # ─── RETURN HOME + LAND ───
    with log.step("LAP2 done → return + land"):
        goto_xy_abs(cf, origin[0], origin[1], TAKEOFF_Z_M, 0.0, log,
                     label="final_return")
        repl.exec_int("sentai.explore.land()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        t0 = time.monotonic()
        descent_dur = 3.0
        while True:
            te = time.monotonic() - t0
            frac = min(1.0, te / descent_dur)
            z = TAKEOFF_Z_M + frac * (0.05 - TAKEOFF_Z_M)
            cf.commander.send_position_setpoint(origin[0], origin[1], z, 0.0)
            tel = tel_snapshot()
            pose_tick(repl, tel['x'], tel['y'], tel['z'], tel['yaw_deg'],
                       transitions_seen)
            if frac >= 1.0 and tel['z'] < 0.10: break
            if te > descent_dur + 4: break
            time.sleep(0.05)
        tel = tel_snapshot()
        land_pose = [tel['x'], tel['y'], tel['z']]
        summary["land_pose"] = land_pose
        cf.commander.send_stop_setpoint()
        time.sleep(0.5)
        tick_dwell(repl, LAND_DWELL_S, transitions_seen)
        summary["state_final"] = repl.exec_value("sentai.explore.state()")

    land_err = math.hypot(land_pose[0] - origin[0], land_pose[1] - origin[1])
    summary["land_err_xy_vs_origin_m"] = land_err
    summary["transitions_seen"] = transitions_seen
    summary["mission_duration_s"] = time.monotonic() - t_start
    log.info(f"        land_err = {land_err*100:.1f} cm")
    log.info(f"        mission_duration = {summary['mission_duration_s']:.1f}s")

    stop_evt.set()
    flow_th.join(timeout=2.0)
    sync.close_link()
    return summary


def main() -> int:
    if HEX_HELPERS_SRC.is_file():
        shutil.copy(HEX_HELPERS_SRC, SENTAI_FS_ROOT / "hex_helpers.py")
    log = StepLog(MISSION_LOG)
    try:
        with log.step("spawn sentai_sim + REPL"):
            repl = ReplDriver(bin_path=SENTAI_SIM_BIN, fs_root=SENTAI_FS_ROOT,
                               transcript=REPL_TRANSCRIPT,
                               startup_timeout_s=15.0, frames_dir_base=WORKDIR)
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

"""s142 — HexPatrol: descriptors stored per H3 cell during cf2 mission.

Mission flow:
  1. cf2 takeoff at origin → CAPTURE PHYSICAL_ORIGIN
  2. Store HOME descriptor at h3(origin) via hex_helpers.capture_and_store(seed=0)
  3. Inject 2 synthetic L5 targets at long distances (s137-style)
  4. explore.start + takeoff → HOVERING
  5. explore.goto(t1) → INSPECT → store seed=1 descriptor at h3(t1)
  6. explore.goto(t2) → INSPECT → store seed=2 descriptor at h3(t2)
  7. explore.return_home → land at origin
  8. Verify gallery: ≥3 cells, unique, self-match works, closure < 15 cm

Outputs under /tmp/s142_hex_descriptor_patrol/.
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
for p in (S091_DIR, S128_DIR, S132_DIR):
    sys.path.insert(0, str(p))

import aruco_hover                                       # noqa: E402
from mission_l41 import ReplDriver, StepLog              # noqa: E402
from mission_lifter import _tel_cb, tel_snapshot, _tel_samples  # noqa: E402

WORKDIR = Path("/tmp/s142_hex_descriptor_patrol")
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

JOURNAL_NAME = "s142_journal.txt"

TARGETS = [
    # (id, class, x, y, z, seed)
    (10, 1, +1.50, 0.00, 1.40, 1),
    (11, 1, -0.50, +1.00, 1.40, 2),
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


def tick_dwell(repl, dur_s, transitions_seen):
    deadline = time.monotonic() + dur_s
    while time.monotonic() < deadline:
        tel = tel_snapshot()
        pose_tick(repl, tel["x"], tel["y"], tel["z"], tel["yaw_deg"],
                   transitions_seen)
        time.sleep(1.0 / POSE_LOOP_HZ)


def store_at_pose(repl, log, seed, x, y, z, label):
    """REPL call to hex_helpers.capture_and_store, returns place_id."""
    rc = repl.exec_int(
        f"hex_helpers.capture_and_store({seed}, {x:.4f}, {y:.4f}, {z:.4f})")
    log.info(f"        store {label} seed={seed} @ ({x:+.3f},{y:+.3f}) → "
             f"place_id={rc}")
    if rc <= 0:
        raise RuntimeError(f"capture_and_store({label}) returned {rc}")
    return rc


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

    with log.step("REPL init + import hex_helpers + L3/L5/L6 setup"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")
        # places: clean gallery for this mission
        n_cleared = repl.exec_int("sentai.places.clear()")
        log.info(f"        places cleared (n={n_cleared})")
        # lifter
        repl.exec_int("sentai.object_lifter.clear()")
        # explore
        rc = repl.exec_int("sentai.explore.init('sim')")
        if rc != 0: raise RuntimeError(f"explore.init rc={rc}")
        repl.exec_int("sentai.explore.set_tunables(0.15, 1500, 3000)")
        # import hex_helpers
        repl.exec("import hex_helpers")
        log.info("        hex_helpers imported")

    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        physical_origin = (tel['x'], tel['y'])
        summary["physical_origin"] = list(physical_origin)
        log.info(f"        PHYSICAL_ORIGIN = "
                 f"({physical_origin[0]:+.3f},{physical_origin[1]:+.3f})")

    # ─── Store HOME descriptor ───
    pid_home = store_at_pose(repl, log, seed=0,
                              x=physical_origin[0], y=physical_origin[1],
                              z=tel['z'], label="HOME")
    summary["pid_home"] = pid_home

    # ─── Inject 2 synthetic targets ───
    with log.step(f"L5 inject {len(TARGETS)} synthetic targets"):
        for (tid, cls, x, y, z, _seed) in TARGETS:
            rc = repl.exec_int(
                f"sentai.object_lifter.inject({tid}, {cls}, {x}, {y}, {z})")
            if rc < 0: raise RuntimeError(f"inject({tid}) rc={rc}")
            log.info(f"        injected id={tid} @ ({x:+.2f},{y:+.2f})")

    # ─── explore.start + takeoff ───
    with log.step("explore.start + takeoff → HOVERING"):
        repl.exec_int("sentai.explore.start()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        repl.exec_int(f"sentai.explore.takeoff({TAKEOFF_Z_M})")
        wait_for_state(repl, {"HOVERING"}, transitions_seen, 5.0, log)

    # ─── Visit each target → store descriptor at its H3 cell ───
    pids = []
    for (tid, cls, tx, ty, tz, seed) in TARGETS:
        with log.step(f"explore.goto({tid}) seed={seed}"):
            repl.exec_int(f"sentai.explore.goto({tid}, {STOP_DIST_M})")
            transitions_seen.append(repl.exec_value("sentai.explore.state()"))
            m = repl.exec_repr("sentai.explore.metrics()")
            tgt = (m["target_x"], m["target_y"])
            log.info(f"        L6 target=({tgt[0]:+.3f},{tgt[1]:+.3f})")

        with log.step(f"cf2 → target_{tid} via closed-loop"):
            goto_xy_abs(cf, tgt[0], tgt[1], TAKEOFF_Z_M, 0.0, log,
                         label=f"goto_{tid}")
            st = wait_for_state(repl, {"INSPECT", "HOVERING"},
                                 transitions_seen, 10.0, log)
            if st == "INSPECT":
                dwell_with_hold(repl, cf, tgt[0], tgt[1], TAKEOFF_Z_M, 0.0,
                                 INSPECT_DWELL_S, transitions_seen, log)

        # Store descriptor for this target's location
        tel_now = tel_snapshot()
        pid = store_at_pose(repl, log, seed=seed,
                             x=tel_now['x'], y=tel_now['y'], z=tel_now['z'],
                             label=f"T{tid}")
        pids.append(pid)
    summary["pids_targets"] = pids

    # ─── return_home + land ───
    with log.step("explore.return_home"):
        repl.exec_int("sentai.explore.return_home()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
    with log.step("cf2 → PHYSICAL_ORIGIN"):
        goto_xy_abs(cf, physical_origin[0], physical_origin[1],
                     TAKEOFF_Z_M, 0.0, log, label="return_home")
        wait_for_state(repl, {"HOVERING"}, transitions_seen, 15.0, log)

    with log.step("explore.land + controlled descent"):
        repl.exec_int("sentai.explore.land()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        t0 = time.monotonic()
        descent_dur = 3.0
        while True:
            te = time.monotonic() - t0
            frac = min(1.0, te / descent_dur)
            z = TAKEOFF_Z_M + frac * (0.05 - TAKEOFF_Z_M)
            cf.commander.send_position_setpoint(physical_origin[0],
                                                  physical_origin[1], z, 0.0)
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

    # ─── Verify gallery ───
    with log.step("verify L3 places gallery"):
        gallery_count = repl.exec_int("sentai.places.count()")
        summary["gallery_count"] = gallery_count
        log.info(f"        gallery count = {gallery_count}")

        # Collect H3 cells + desc_set for home + targets
        cells = []
        all_descs_set = True
        for pid in [pid_home] + pids:
            p = repl.exec_repr(f"sentai.places.get({pid})")
            if p is None:
                all_descs_set = False
                continue
            cells.append(p["h3_cell"])
            if not p["desc_set"]:
                all_descs_set = False
        summary["h3_cells"] = cells
        summary["all_descs_set"] = all_descs_set
        distinct_cells = len(set(cells))
        summary["distinct_h3_cells"] = distinct_cells
        log.info(f"        H3 cells = {cells}")
        log.info(f"        distinct = {distinct_cells}; all_desc_set = {all_descs_set}")

        # Self-match: each stored desc should be findable
        match_results = []
        for pid in [pid_home] + pids:
            mid = repl.exec_int(f"hex_helpers.self_query({pid})")
            match_results.append((pid, mid))
            log.info(f"        self_query({pid}) → {mid}")
        summary["match_results"] = match_results

        # Closure
        land_err = math.hypot(land_pose[0] - physical_origin[0],
                               land_pose[1] - physical_origin[1])
        summary["land_err_xy_vs_origin_m"] = land_err
        summary["transitions_seen"] = transitions_seen
        summary["mission_duration_s"] = time.monotonic() - t_start
        log.info(f"        land_err={land_err*100:.1f} cm")
        log.info(f"        transitions={transitions_seen}")
        log.info(f"        mission_duration={summary['mission_duration_s']:.1f}s")

    stop_evt.set()
    flow_th.join(timeout=2.0)
    sync.close_link()
    return summary


def main() -> int:
    # Ensure hex_helpers is in SIM fs root
    if HEX_HELPERS_SRC.is_file():
        dst = SENTAI_FS_ROOT / "hex_helpers.py"
        shutil.copy(HEX_HELPERS_SRC, dst)
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

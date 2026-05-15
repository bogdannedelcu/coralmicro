"""s132 mission — drive sentai.object_lifter through a real Gazebo
lateral pass and verify it converges to the true marker 3D position.

Reuses harness from s128/s130:
    ReplDriver, StepLog, j_int, sentai.sim.journal_* API,
    cflib MotionCommander + flow_forwarder thread, telemetry capture.

Per-frame: detect ArUco marker id 0, compute bbox center + mean edge
length, push to sentai.object_lifter via REPL exec_repr. Query state
back, log to lifter_trace.

Outputs (under /tmp/s132_lifter_gazebo/):
    summary.json        — verdict reads this
    lifter_trace.json   — per-frame state
    journal.txt         — REPL-side structured journal
    mission.log         — host-side step trace
    repl.transcript     — raw stdin/stdout dump
    cf2_telemetry.json  — stateEstimate samples
"""
from __future__ import annotations

import datetime as dt
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

S091_DIR = Path(__file__).resolve().parent.parent / "s091_aruco_lowalt"
S090_DIR = Path(__file__).resolve().parent.parent / "s090_hover_over_cat"
S128_DIR = Path(__file__).resolve().parent.parent / "s128_l41baseline_seeded"
S129_DIR = Path(__file__).resolve().parent.parent / "s129_l42baseline_centering"
sys.path.insert(0, str(S091_DIR))
sys.path.insert(0, str(S090_DIR))
sys.path.insert(0, str(S128_DIR))
sys.path.insert(0, str(S129_DIR))
import aruco_hover                                       # noqa: E402
from aruco_detector import (                             # noqa: E402
    detect_in_ppm, latest_ppm,
    CAM_FX, CAM_FY, CAM_CX, CAM_CY,
    MARKER_SIZE_M,
)
from mission_l41 import ReplDriver, StepLog              # noqa: E402

WORKDIR = Path("/tmp/s132_lifter_gazebo")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
LIFTER_TRACE    = WORKDIR / "lifter_trace.json"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"

JOURNAL_NAME      = "s132_journal.txt"

# ─── Target marker ───
# id 0 (NE corner of s130 arena). Same world pose, same convention.
TARGET_ARUCO_ID    = 0
TARGET_WORLD_XYZ   = (+0.15, +0.10, 0.20)

# Flight params
TAKEOFF_Z_M        = 1.50      # higher than s130's 1.0 m for more vertical room
TAKEOFF_VEL_MPS    = 0.6
LATERAL_STEP_M     = 0.10
N_LATERAL_STEPS    = 4         # total +0.40 m parallax baseline
SETTLE_PER_STEP_S  = 0.6       # let CSI ring produce fresh frame
HOVER_INIT_S       = 1.0       # initial settle before first bbox sample
LAND_VEL_MPS       = 0.4

# Lifter pass criteria thresholds (verdict checks against)
ERR_XY_MAX_M       = 0.20
ERR_Z_MAX_M        = 0.60
SIGMA_RHO_RATIO    = 0.5
N_OBS_MIN          = 4

# ─────────────────────────────────────────────────────────────────
# Telemetry capture (mirrors mission_l41 pattern)
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
# Helpers — bbox from corners, fresh PPM
# ─────────────────────────────────────────────────────────────────
def bbox_from_marker(m) -> tuple[float, float, float]:
    """Return (u_c, v_c, mean_edge_px) from a Marker.

    mean_edge_px is rotation-tolerant: average of all 4 corner-to-corner
    edge lengths. Equivalent to the "apparent marker width" in pixels
    regardless of yaw/orientation.
    """
    c = np.asarray(m.corners, dtype=np.float64).reshape(4, 2)
    center = c.mean(axis=0)
    edges = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
    return float(center[0]), float(center[1]), float(sum(edges) / 4.0)


def wait_for_fresh_ppm(frames_dir: Path, last_seq: int,
                       timeout_s: float = 3.0) -> tuple[Path, int]:
    """Block until a PPM with seq > last_seq appears. Returns (path, seq)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ppm = latest_ppm(frames_dir)
        if ppm is not None:
            try:
                seq = int(ppm.stem.replace("frame_", ""))
            except ValueError:
                time.sleep(0.05)
                continue
            if seq > last_seq:
                return ppm, seq
        time.sleep(0.05)
    raise TimeoutError(f"no fresh PPM > seq{last_seq} in {timeout_s}s")


def sample_marker_bbox(frames_dir: Path, target_id: int,
                       last_seq: int, log: StepLog,
                       max_attempts: int = 5) -> dict | None:
    """Get one fresh detection of target_id. Returns dict or None."""
    seq = last_seq
    for attempt in range(max_attempts):
        try:
            ppm, seq = wait_for_fresh_ppm(frames_dir, seq, timeout_s=2.5)
        except TimeoutError as e:
            log.info(f"        bbox sample: {e}")
            return None
        try:
            dets = detect_in_ppm(ppm, estimate_pose=True)
        except Exception as e:
            log.info(f"        bbox sample: detect failed on {ppm.name}: {e}")
            continue
        if target_id not in dets:
            log.info(f"        bbox sample: id{target_id} not in {ppm.name} "
                     f"(saw {sorted(dets.keys())})")
            continue
        m = dets[target_id]
        u, v, w_px = bbox_from_marker(m)
        # Capture tvec for verification (NOT fed to lifter — lifter only
        # gets bbox + drone pose).
        tvec = (m.tvec.tolist() if m.tvec is not None
                else [float("nan")] * 3)
        return {
            "ppm": ppm.name, "seq": seq,
            "u": u, "v": v, "w_px": w_px,
            "tvec_cam": tvec,
        }
    return None


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
    summary: dict = {
        "target_aruco_id":  TARGET_ARUCO_ID,
        "target_world_xyz": list(TARGET_WORLD_XYZ),
        "lateral_step_m":   LATERAL_STEP_M,
        "n_lateral_steps":  N_LATERAL_STEPS,
        "trace":            [],
    }
    trace = summary["trace"]

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

    with log.step("flow forwarder thread (give it 2s to connect bridge)"):
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

    def j_int(label: str, cmd: str) -> int:
        rc = repl.exec_int(cmd)
        repl.exec_int(f"sentai.sim.journal_write('{label}', "
                       f"sentai.servo.status())")
        return rc

    with log.step("REPL: import + verbose(0) + journal_open"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        rc = repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        if rc != 0:
            raise RuntimeError(f"journal_open returned {rc}")
        repl.exec_int("sentai.sim.journal_write('mission_begin', None)")

    with log.step("REPL: lifter.clear + servo.init"):
        n = repl.exec_int("sentai.object_lifter.clear()")
        log.info(f"        cleared {n} prior lifter entries")
        # Lifter intrinsics already default to (577, 579, 320, 240) +
        # R_B_C identity-with-swap + cam_offset_B. Match aruco_detector.
        rc = j_int("servo_init", "sentai.servo.init('sim')")
        if rc != 0:
            raise RuntimeError(f"servo.init rc={rc}")
        cleared_trace = repl.exec_int("sentai.servo.clear_trace()")
        log.info(f"        cleared {cleared_trace} prior trace entries")

    # Sanity: verify lifter camera intrinsics match aruco_detector.
    cam_stats = repl.exec_repr("sentai.object_lifter.stats()")
    log.info(f"        lifter stats pre-flight: {cam_stats}")

    with log.step(f"REPL: servo.arm + servo.takeoff({TAKEOFF_Z_M})"):
        rc = j_int("servo_arm", "sentai.servo.arm()")
        if rc != 0:
            raise RuntimeError(f"servo.arm rc={rc}")
        rc = j_int("servo_takeoff", f"sentai.servo.takeoff({TAKEOFF_Z_M})")
        if rc != 0:
            raise RuntimeError(f"servo.takeoff rc={rc}")

    with log.step(f"cf2 take_off → {TAKEOFF_Z_M} m @ {TAKEOFF_VEL_MPS} m/s"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(HOVER_INIT_S)
        tel = tel_snapshot()
        log.info(f"        post-takeoff cf2 @ ({tel['x']:+.2f},"
                 f"{tel['y']:+.2f},{tel['z']:.2f})  "
                 f"yaw={tel['yaw_deg']:+.1f}°  "
                 f"flow_n={flow_stats['n_sent']}")

    # ─── Lifter init from first visible frame ───
    last_seq = -1
    with log.step("lifter init_from_bbox: wait for first marker sighting"):
        bbox = sample_marker_bbox(repl.frames_dir, TARGET_ARUCO_ID,
                                   last_seq, log)
        if bbox is None:
            raise RuntimeError(
                f"id{TARGET_ARUCO_ID} not visible at takeoff — abort")
        last_seq = bbox["seq"]
        tel = tel_snapshot()
        drone_W = (tel["x"], tel["y"], tel["z"])
        yaw = math.radians(tel["yaw_deg"])
        log.info(f"        bbox: u={bbox['u']:.1f} v={bbox['v']:.1f} "
                 f"w_px={bbox['w_px']:.1f}  drone_W=({drone_W[0]:+.3f},"
                 f"{drone_W[1]:+.3f},{drone_W[2]:.3f})  yaw={tel['yaw_deg']:+.1f}°")
        tid = 0
        cls = 0
        rc = repl.exec_int(
            f"sentai.object_lifter.init_from_bbox({tid}, {cls}, "
            f"{bbox['u']}, {bbox['v']}, {bbox['w_px']}, {MARKER_SIZE_M}, "
            f"({drone_W[0]}, {drone_W[1]}, {drone_W[2]}), {yaw})")
        if rc < 0:
            raise RuntimeError(f"init_from_bbox rc={rc} — see transcript")
        log.info(f"        init OK slot={rc}")
        snap = repl.exec_repr(f"sentai.object_lifter.get({tid})")
        log.info(f"        post-init: status={snap['status']} "
                 f"rho={snap['rho']:.3f}  var_rho={snap['var_rho']:.4f}  "
                 f"n_obs={snap['n_obs']}")
        trace.append({
            "phase":   "init",
            "step":    0,
            "ppm":     bbox["ppm"],
            "u":       bbox["u"], "v": bbox["v"], "w_px": bbox["w_px"],
            "tvec_cam": bbox["tvec_cam"],
            "drone_W": list(drone_W),
            "yaw":     yaw,
            "snap":    snap,
        })
        sigma_rho_init = math.sqrt(snap["var_rho"])

    # ─── Lateral pass with per-step update ───
    t_prev = time.monotonic()
    for step_idx in range(1, N_LATERAL_STEPS + 1):
        with log.step(f"lateral step {step_idx}/{N_LATERAL_STEPS}: "
                      f"body_dy +{LATERAL_STEP_M:.2f} m"):
            rc = j_int(f"servo_move_step{step_idx}",
                       f"sentai.servo.move(0.0, {LATERAL_STEP_M}, 0.0)")
            if rc != 0:
                raise RuntimeError(
                    f"servo.move step{step_idx} rc={rc}")
            mc.move_distance(0.0, LATERAL_STEP_M, 0.0, velocity=0.20)
            time.sleep(SETTLE_PER_STEP_S)
            mc.start_linear_motion(0.0, 0.0, 0.0)
            time.sleep(0.2)

            bbox = sample_marker_bbox(repl.frames_dir, TARGET_ARUCO_ID,
                                       last_seq, log)
            if bbox is None:
                log.info(f"        step{step_idx}: no marker visible, "
                         "skipping lifter update")
                trace.append({
                    "phase": "update", "step": step_idx,
                    "ppm":   None, "snap": None,
                    "reason": "no marker visible",
                })
                continue
            last_seq = bbox["seq"]
            tel = tel_snapshot()
            drone_W = (tel["x"], tel["y"], tel["z"])
            yaw = math.radians(tel["yaw_deg"])
            t_now = time.monotonic()
            dt_s = t_now - t_prev
            t_prev = t_now
            log.info(f"        bbox: u={bbox['u']:.1f} v={bbox['v']:.1f} "
                     f"w_px={bbox['w_px']:.1f}  drone_W=({drone_W[0]:+.3f},"
                     f"{drone_W[1]:+.3f},{drone_W[2]:.3f})  "
                     f"yaw={tel['yaw_deg']:+.1f}°  dt={dt_s:.2f}s")
            rc = repl.exec_int(
                f"sentai.object_lifter.update_bbox(0, "
                f"{bbox['u']}, {bbox['v']}, "
                f"({drone_W[0]}, {drone_W[1]}, {drone_W[2]}), {yaw}, {dt_s})")
            snap = repl.exec_repr("sentai.object_lifter.get(0)")
            wpos = repl.exec_repr("sentai.object_lifter.world_pos(0)")
            log.info(f"        update rc={rc}  status={snap['status']} "
                     f"rho={snap['rho']:.3f}  var_rho={snap['var_rho']:.4f}  "
                     f"n_obs={snap['n_obs']}  world_pos={wpos}")
            trace.append({
                "phase":   "update", "step": step_idx,
                "ppm":     bbox["ppm"],
                "u":       bbox["u"], "v": bbox["v"], "w_px": bbox["w_px"],
                "tvec_cam": bbox["tvec_cam"],
                "drone_W": list(drone_W),
                "yaw":     yaw,
                "dt_s":    dt_s,
                "rc":      rc,
                "snap":    snap,
                "world_pos_est": list(wpos) if wpos is not None else None,
            })

    # ─── Final verification ───
    with log.step("final lifter state + world_pos vs GT"):
        snap = repl.exec_repr("sentai.object_lifter.get(0)")
        wpos = repl.exec_repr("sentai.object_lifter.world_pos(0)")
        stats = repl.exec_repr("sentai.object_lifter.stats()")
        gt = TARGET_WORLD_XYZ
        if wpos is None:
            err_xy = err_z = float("nan")
        else:
            err_xy = math.hypot(wpos[0] - gt[0], wpos[1] - gt[1])
            err_z  = abs(wpos[2] - gt[2])
        sigma_rho_final = math.sqrt(snap["var_rho"]) if snap else float("nan")
        log.info(f"        final snap: {snap}")
        log.info(f"        final world_pos_est: {wpos}  GT={gt}")
        log.info(f"        err_xy={err_xy*100:.1f} cm  err_z={err_z*100:.1f} cm")
        log.info(f"        sigma_rho: init={sigma_rho_init:.4f} → "
                 f"final={sigma_rho_final:.4f}  "
                 f"ratio={sigma_rho_final/sigma_rho_init:.3f}")
        log.info(f"        stats: {stats}")
        summary["final_snap"]        = snap
        summary["final_world_pos"]   = list(wpos) if wpos else None
        summary["err_xy_m"]          = err_xy
        summary["err_z_m"]           = err_z
        summary["sigma_rho_init"]    = sigma_rho_init
        summary["sigma_rho_final"]   = sigma_rho_final
        summary["sigma_rho_ratio"]   = (sigma_rho_final / sigma_rho_init
                                         if sigma_rho_init > 0 else float("nan"))
        summary["stats"]             = stats

    # ─── Land + disarm ───
    with log.step("cf2 land + servo.land + servo.disarm"):
        rc = j_int("servo_land", "sentai.servo.land()")
        if rc != 0:
            log.info(f"        WARN: servo.land rc={rc}")
        mc.land(velocity=LAND_VEL_MPS)
        time.sleep(2.0)
        rc = j_int("servo_disarm", "sentai.servo.disarm()")
        if rc != 0:
            log.info(f"        WARN: servo.disarm rc={rc}")

    with log.step("final servo status snapshot"):
        st = repl.exec_repr("sentai.servo.status()")
        log.info(f"        servo status: {st}")
        summary["servo_status"] = st

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
            # Capture journal lines BEFORE closing the REPL.
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
        }, indent=2))
        log.close()
        return 1
    summary["_last_run"] = dt.datetime.now().isoformat()
    SUMMARY_JSON.write_text(json.dumps(summary, indent=2, default=str))
    LIFTER_TRACE.write_text(json.dumps(summary["trace"], indent=2,
                                        default=str))
    TELEMETRY_JSON.write_text(json.dumps(_tel_samples, indent=2))
    log.info(f"summary written to {SUMMARY_JSON}")
    log.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

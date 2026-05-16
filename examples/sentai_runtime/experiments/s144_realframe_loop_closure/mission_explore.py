"""s144 — Real Gazebo frame loop closure + telemetry observability.

Same 2-lap structure as s143 but:
  - Replaces synthetic seed-based images with REAL Gazebo camera frames
    via sentai.camera.grab_gray().
  - Adds telemetry callback gap tracking + journal-based event capture
    so any stale-tel issue leaves a parseable post-mortem trail.

Designed per embeded.md §2 (Power of Ten): bounded loops with explicit
timeouts, every status checked, fault-localised handlers, journal
breadcrumb on every state change.
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

WORKDIR = Path("/tmp/s144_realframe_loop_closure")
WORKDIR.mkdir(parents=True, exist_ok=True)

REPL_TRANSCRIPT = WORKDIR / "repl.transcript"
MISSION_LOG     = WORKDIR / "mission.log"
TELEMETRY_JSON  = WORKDIR / "cf2_telemetry.json"
SUMMARY_JSON    = WORKDIR / "summary.json"
JOURNAL_COPY    = WORKDIR / "journal.txt"

REPO_ROOT      = Path(__file__).resolve().parents[4]
SENTAI_SIM_BIN = REPO_ROOT / "build-sim" / "sim" / "sentai_sim"
SENTAI_FS_ROOT = REPO_ROOT / "build-sim" / "sentai_fs_root"
HEX_HELPERS_SRC = (REPO_ROOT / "examples" / "sentai_runtime" /
                   "experiments" / "s142_hex_descriptor_patrol" /
                   "hex_helpers.py")

JOURNAL_NAME = "s144_journal.txt"

TARGETS = [
    # (id, class, x, y, z)
    (10, 1, +1.50, 0.00, 1.40),
    (11, 1, -0.50, +1.00, 1.40),
]

TAKEOFF_Z_M       = 1.50
TAKEOFF_VEL_MPS   = 0.6
STOP_DIST_M       = 0.10
APPROACH_VEL_MPS  = 0.5
LAND_VEL_MPS      = 0.40

FRAME_W           = 80      # grab_gray target — small but discriminative
FRAME_H           = 60

POSE_LOOP_HZ      = 5.0
INSPECT_DWELL_S   = 1.7
LAND_DWELL_S      = 3.2
CAPTURE_SETTLE_S  = 0.5     # give camera bridge a frame after arrival
TEL_STALE_MAX_MS  = 5000    # hard gate for telemetry observability

# ─── Telemetry state (with gap tracking — s144 new) ─────────────
_tel_state = {
    "x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0,
    "last_cb_ms": 0,
    "n_callbacks": 0,
    "max_gap_ms": 0,
}
_tel_lock = threading.Lock()
_tel_samples: list = []


def _tel_cb(_ts, data, _lc):
    now_ms = int(time.monotonic() * 1000)
    with _tel_lock:
        if _tel_state["last_cb_ms"] > 0:
            gap = now_ms - _tel_state["last_cb_ms"]
            if gap > _tel_state["max_gap_ms"]:
                _tel_state["max_gap_ms"] = gap
        _tel_state["last_cb_ms"] = now_ms
        _tel_state["n_callbacks"] += 1
        _tel_state["x"]       = data["stateEstimate.x"]
        _tel_state["y"]       = data["stateEstimate.y"]
        _tel_state["z"]       = data["stateEstimate.z"]
        _tel_state["yaw_deg"] = data["stateEstimate.yaw"]
        _tel_samples.append({
            "t": time.monotonic(),
            "x": _tel_state["x"], "y": _tel_state["y"],
            "z": _tel_state["z"], "yaw_deg": _tel_state["yaw_deg"],
        })


def tel_snapshot():
    with _tel_lock:
        return dict(_tel_state)


def tel_age_ms():
    now_ms = int(time.monotonic() * 1000)
    with _tel_lock:
        if _tel_state["last_cb_ms"] == 0:
            return 999999
        return now_ms - _tel_state["last_cb_ms"]


# ─── Journal helper: structured event capture with tel snapshot ─────
def journal_event(repl, label, payload=None):
    """Append event to SIM journal.  Payload is a dict; this function
    augments it with current tel state for post-mortem debug.
    """
    with _tel_lock:
        env = {
            "tel_x": _tel_state["x"], "tel_y": _tel_state["y"],
            "tel_z": _tel_state["z"],
            "tel_age_ms": tel_age_ms(),
            "tel_n_cb": _tel_state["n_callbacks"],
            "tel_max_gap_ms": _tel_state["max_gap_ms"],
        }
    if isinstance(payload, dict):
        env.update(payload)
    elif payload is not None:
        env["v"] = payload
    # Use plain repr — MP-side parses with ast.literal_eval.
    try:
        repl.exec_int(
            "sentai.sim.journal_write('%s', %s)" % (label, repr(env)))
    except Exception:
        pass


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
    n_spam = 0
    while time.monotonic() < deadline:
        cf.commander.send_position_setpoint(tx, ty, tz, yaw)
        n_spam += 1
        tel = tel_snapshot()
        last = (tel['x'], tel['y'], tel['z'])
        if math.hypot(tel['x'] - tx, tel['y'] - ty) <= tol_m:
            log.info(f"        {label}: arrived @ "
                     f"({last[0]:+.3f},{last[1]:+.3f},{last[2]:.3f}) "
                     f"spam={n_spam} tel_age={tel_age_ms()}ms")
            return True, last
        time.sleep(0.05)
    log.info(f"        {label}: TIMEOUT @ "
             f"({last[0]:+.3f},{last[1]:+.3f},{last[2]:.3f}) "
             f"spam={n_spam} tel_age={tel_age_ms()}ms")
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


def real_store_at(repl, log, x, y, z, label):
    """Call hex_helpers.real_capture_and_store, return pid (or raise)."""
    # Allow a couple of attempts in case grab_gray returns None for a frame
    # (camera bridge can have a transient between Gazebo frames).
    for attempt in range(3):
        rc = repl.exec_int(
            f"hex_helpers.real_capture_and_store({x:.4f}, {y:.4f}, {z:.4f}, "
            f"{FRAME_W}, {FRAME_H})")
        if rc >= 1:
            log.info(f"        store {label} (REAL FRAME) @ "
                     f"({x:+.3f},{y:+.3f}) → pid={rc} (attempt {attempt+1})")
            return rc
        log.info(f"        store {label} attempt {attempt+1}: rc={rc}")
        time.sleep(0.3)
    raise RuntimeError(f"real_capture_and_store({label}) failed after retries")


def real_query_at(repl, log, x, y, label):
    """Call hex_helpers.real_query_at, return match dict (or None)."""
    for attempt in range(3):
        res = repl.exec_repr(
            f"hex_helpers.real_query_at({x:.4f}, {y:.4f}, "
            f"{FRAME_W}, {FRAME_H})")
        if res is not None:
            log.info(f"        query {label} (REAL FRAME) @ "
                     f"({x:+.3f},{y:+.3f}) → {res} (attempt {attempt+1})")
            return res
        log.info(f"        query {label} attempt {attempt+1}: None")
        time.sleep(0.3)
    log.info(f"        query {label}: failed after retries")
    return None


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

    with log.step("telemetry log @ 20 ms + flow forwarder"):
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
        log.info(f"        tel_n_cb={_tel_state['n_callbacks']}, "
                 f"max_gap={_tel_state['max_gap_ms']}ms")

    with log.step("REPL init + hex_helpers + journal"):
        repl.exec("import sentai")
        repl.exec("sentai.verbose(0)")
        rc = repl.exec_int(f"sentai.sim.journal_open('{JOURNAL_NAME}')")
        if rc != 0: raise RuntimeError(f"journal_open rc={rc}")
        repl.exec_int("sentai.places.clear()")
        repl.exec_int("sentai.object_lifter.clear()")
        rc = repl.exec_int("sentai.explore.init('sim')")
        if rc != 0: raise RuntimeError(f"explore.init rc={rc}")
        repl.exec_int("sentai.explore.set_tunables(0.15, 1500, 3000)")
        # Use exec_int (sentinel-based) for import: pure `exec` blocks
        # on prompt detection in the presence of camera_bridge chatter.
        # The expression evaluates to 0 after a successful import.
        rc = repl.exec_int("(__import__('hex_helpers'), 0)[1]", timeout_s=10.0)
        if rc != 0:
            raise RuntimeError(f"hex_helpers import rc={rc}")
        log.info("        hex_helpers imported")
        journal_event(repl, "mission_begin", {"phase": "init_done"})

    with log.step(f"cf2 take_off → {TAKEOFF_Z_M}m"):
        mc = MotionCommander(sync, default_height=TAKEOFF_Z_M)
        mc.take_off(height=TAKEOFF_Z_M, velocity=TAKEOFF_VEL_MPS)
        time.sleep(3.0)
        mc.start_linear_motion(0.0, 0.0, 0.0)
        time.sleep(1.0)
        tel = tel_snapshot()
        origin = (tel['x'], tel['y'])
        summary["physical_origin"] = list(origin)
        log.info(f"        PHYSICAL_ORIGIN=({origin[0]:+.3f},{origin[1]:+.3f}) "
                 f"tel_age={tel_age_ms()}ms n_cb={_tel_state['n_callbacks']}")
        journal_event(repl, "phys_origin",
                      {"x": origin[0], "y": origin[1], "z": tel['z']})

    # ─── LAP 1 ───
    log.info("=== LAP 1 (build memory with REAL frames) ===")
    journal_event(repl, "lap1_begin")

    # HOME: wait for camera frame, then capture+store
    time.sleep(CAPTURE_SETTLE_S)  # let camera bridge produce a fresh frame
    pid_home = real_store_at(repl, log, origin[0], origin[1], tel['z'], "HOME")
    p_home = repl.exec_repr(f"sentai.places.get({pid_home})")
    summary["pid_home"] = pid_home
    summary["h3_home"] = p_home["h3_cell"]
    journal_event(repl, "store_home", {"pid": pid_home,
                                          "h3": p_home["h3_cell"]})

    with log.step("L5 inject 2 targets"):
        for (tid, cls, x, y, z) in TARGETS:
            repl.exec_int(
                f"sentai.object_lifter.inject({tid}, {cls}, {x}, {y}, {z})")

    with log.step("explore.start + takeoff → HOVERING"):
        repl.exec_int("sentai.explore.start()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        repl.exec_int(f"sentai.explore.takeoff({TAKEOFF_Z_M})")
        wait_for_state(repl, {"HOVERING"}, transitions_seen, 5.0, log)
    journal_event(repl, "lap1_takeoff_done")

    pids_t = []
    for (tid, cls, tx, ty, tz) in TARGETS:
        label = f"T{tid}"
        with log.step(f"LAP1: goto({tid})"):
            repl.exec_int(f"sentai.explore.goto({tid}, {STOP_DIST_M})")
            transitions_seen.append(repl.exec_value("sentai.explore.state()"))
            m = repl.exec_repr("sentai.explore.metrics()")
            tgt = (m["target_x"], m["target_y"])
            journal_event(repl, f"lap1_pre_goto_{tid}",
                          {"target_x": tgt[0], "target_y": tgt[1]})
            goto_xy_abs(cf, tgt[0], tgt[1], TAKEOFF_Z_M, 0.0, log,
                         label=f"goto_{tid}_lap1")
            st = wait_for_state(repl, {"INSPECT", "HOVERING"},
                                 transitions_seen, 10.0, log)
            if st == "INSPECT":
                dwell_with_hold(repl, cf, tgt[0], tgt[1], TAKEOFF_Z_M, 0.0,
                                 INSPECT_DWELL_S, transitions_seen, log)
        # Capture REAL frame at this place
        time.sleep(CAPTURE_SETTLE_S)
        tel_now = tel_snapshot()
        pid = real_store_at(repl, log, tel_now['x'], tel_now['y'],
                             tel_now['z'], label)
        p = repl.exec_repr(f"sentai.places.get({pid})")
        pids_t.append((tid, pid, p["h3_cell"]))
        journal_event(repl, f"lap1_store_{tid}",
                      {"pid": pid, "h3": p["h3_cell"]})
    summary["pids_lap1"] = pids_t

    with log.step("LAP1 done → return to origin"):
        repl.exec_int("sentai.explore.return_home()")
        transitions_seen.append(repl.exec_value("sentai.explore.state()"))
        goto_xy_abs(cf, origin[0], origin[1], TAKEOFF_Z_M, 0.0, log,
                     label="return_after_lap1")
        wait_for_state(repl, {"HOVERING"}, transitions_seen, 12.0, log)

    summary["gallery_count_after_lap1"] = repl.exec_int("sentai.places.count()")
    journal_event(repl, "lap1_end",
                  {"gallery_count": summary["gallery_count_after_lap1"]})
    log.info(f"        lap1 gallery count = {summary['gallery_count_after_lap1']}")

    # ─── LAP 2 ───
    log.info("=== LAP 2 (consume memory with FRESH REAL frame) ===")
    journal_event(repl, "lap2_begin")
    target1 = TARGETS[0]
    t1_pid = pids_t[0][1]

    with log.step(f"LAP2: cf2 → target1 ({target1[2]:+.2f},{target1[3]:+.2f})"):
        ok, _ = goto_xy_abs(cf, target1[2], target1[3], TAKEOFF_Z_M, 0.0, log,
                              label="lap2_goto_t1")
        # Hold position for a stable frame
        for _ in range(int(CAPTURE_SETTLE_S / 0.05)):
            cf.commander.send_position_setpoint(target1[2], target1[3],
                                                  TAKEOFF_Z_M, 0.0)
            time.sleep(0.05)
        tel_now = tel_snapshot()
        journal_event(repl, "lap2_at_target",
                      {"goto_ok": ok, "pose_x": tel_now['x'],
                       "pose_y": tel_now['y']})

    with log.step("LAP2: REAL FRAME query — does drone recognize target1?"):
        res = real_query_at(repl, log, tel_now['x'], tel_now['y'], "lap2_t1")
        if res is None:
            raise RuntimeError("real_query_at returned None")
        match_id    = int(res['id'])
        match_score = int(res['score_pct'])
        match_l1    = int(res['l1_dist'])
        frame_seq   = int(res.get('frame_seq', 0))
        summary["match_lap2"] = {
            "id": match_id, "score_pct": match_score, "l1_dist": match_l1,
            "frame_seq_lap2": frame_seq,
        }
        summary["expected_match_id"] = t1_pid
        journal_event(repl, "match_result",
                      {"id": match_id, "score": match_score,
                       "l1": match_l1, "frame_seq": frame_seq,
                       "expected_id": t1_pid})
        log.info(f"        MATCH: id={match_id} (expected {t1_pid}), "
                 f"score={match_score}%, l1={match_l1}, frame_seq={frame_seq}")

    summary["gallery_count_after_lap2"] = repl.exec_int("sentai.places.count()")

    # ─── Return + land ───
    with log.step("return + land"):
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
    journal_event(repl, "land_done", {"land_pose": land_pose})

    # ─── Telemetry summary ───
    tel_summary = tel_snapshot()
    summary["tel_summary"] = {
        "n_callbacks": tel_summary["n_callbacks"],
        "max_gap_ms":  tel_summary["max_gap_ms"],
    }
    journal_event(repl, "mission_end", summary["tel_summary"])

    land_err = math.hypot(land_pose[0] - origin[0], land_pose[1] - origin[1])
    summary["land_err_xy_vs_origin_m"] = land_err
    summary["transitions_seen"] = transitions_seen
    summary["mission_duration_s"] = time.monotonic() - t_start
    log.info(f"        land_err = {land_err*100:.1f} cm")
    log.info(f"        tel max_gap = {tel_summary['max_gap_ms']} ms "
             f"(threshold {TEL_STALE_MAX_MS} ms)")
    log.info(f"        mission_duration = {summary['mission_duration_s']:.1f}s")

    repl.exec_int("sentai.sim.journal_close()")
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
            journal_path = SENTAI_FS_ROOT / JOURNAL_NAME
            if journal_path.is_file():
                JOURNAL_COPY.write_bytes(journal_path.read_bytes())
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

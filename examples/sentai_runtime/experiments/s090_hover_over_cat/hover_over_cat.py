#!/usr/bin/env python3
"""
hover_over_cat.py — Phase 5 capstone: drone homes itself over the cat
picture using SSD MobileNet V2 (class 16) + SentAI-SORT.

Architecture (single Python process to keep it simple):
  - This script spawns sentai_sim as a subprocess and drives its REPL
    via stdin/stdout (Python lists are eval()-able on the way back).
  - Loop @ RATE_HZ:
      * pipeline.step() in sentai_sim
      * read pipeline.tracker_tracks() output
      * pick the highest-confidence confirmed cat track
      * compute pixel offset of bbox centroid from image centre
      * convert to body-frame velocity, send via cflib
  - Concurrent: a cflib-side takeoff/landing sequence in a thread.

Prereqs (start before this script):
  - TPU helper:        venv-coral/bin/python3 sim_tpu_helper.py &
  - Garden + cf2:      via test_closed_loop.sh OR test_detect_e2e.sh stack
  - gz_to_uds_bridge:  via dbox_run_bridge.sh
"""
from __future__ import annotations

import ast
import math
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

# ── attitude (roll/pitch) compensation on bbox centroid ────────────
# Drone tilts to command lateral velocity → camera FOV rotates → static
# ground content appears to shift in image WITHOUT drone translating.
# Without correction the loop over-reacts to tilt-induced bbox shift.
# Real PMW3901 deck firmware filters this via gyro-rate de-rotation
# (mm_flow.c).  For our offboard bbox path we instead pre-compensate
# the centroid using cf2's stateEstimate.{roll,pitch} log block.
SSD_INPUT_W = 300
SSD_INPUT_H = 300
CAM_FOV_H_RAD = math.radians(58.0)
CAM_FOV_V_RAD = math.radians(45.0)
# Focal length in pixels — image_w / (2 * tan(FOV/2))
CAM_FX_PX = SSD_INPUT_W / (2.0 * math.tan(CAM_FOV_H_RAD / 2.0))   # ~270 px
CAM_FY_PX = SSD_INPUT_H / (2.0 * math.tan(CAM_FOV_V_RAD / 2.0))   # ~362 px

# Live drone attitude (deg) + position (m), EKF-fused, updated by cflib
# log callback at 100 Hz.  Position drift accumulates because we have no
# absolute position observation — values are useful for visualisation, not
# absolute reference.
_att_lock = threading.Lock()
_drone_roll_deg = 0.0
_drone_pitch_deg = 0.0
_drone_yaw_deg = 0.0
_drone_x = 0.0
_drone_y = 0.0
_drone_z = 0.0


def attitude_compensate(cx: int, cy: int) -> tuple[int, int]:
    """Subtract apparent-bbox shift due to drone tilt.
    cam0+vflip=1 convention:
      - image LEFT = body FORWARD: forward pitch makes image center
        track body+X, so cat at body+X appears at smaller cx → we ADD
        f_px*pitch back to recover true bbox-vs-body position.
      - image TOP = body LEFT (empirical): right roll makes image
        center track body-Y, so cat at body-Y appears at smaller cy →
        we ADD f_py*roll back to recover true position.
    Sign conventions verified empirically with cf2's stateEstimate
    (roll right > 0, pitch fwd > 0).
    """
    with _att_lock:
        pitch_rad = math.radians(_drone_pitch_deg)
        roll_rad = math.radians(_drone_roll_deg)
    cx_corr = int(round(cx + CAM_FX_PX * pitch_rad))
    cy_corr = int(round(cy + CAM_FY_PX * roll_rad))
    return cx_corr, cy_corr


# ── flow → cf2 conversion (mirrors examples/sentai_runtime/diag/_t_flow_to_drone.py) ─
# Same math used on HW board; same body convention (cam0 + vflip=1).
FLOW_FOV_H_DEG    = 58.0
FLOW_FOV_V_DEG    = 45.0
FLOW_GRID_W       = 80
FLOW_GRID_H       = 60
DRONE_NPIX        = 35.0      # PMW3901 px count
DRONE_THETAPIX    = 0.71674   # PMW3901 42° FOV
DRONE_FLOW_RES    = 0.10      # FLOW_RESOLUTION in mm_flow.c
# Body-frame xform for cam0 + vflip=1 (verified 2026-05-07).  body_fw=-dx, body_left=+dy.
BODY_XFORM = (-1.0, 0.0, 0.0, +1.0)
_FLOW_SCALE_X = (math.radians(FLOW_FOV_H_DEG) * DRONE_NPIX) / (FLOW_GRID_W * DRONE_FLOW_RES * DRONE_THETAPIX)
_FLOW_SCALE_Y = (math.radians(FLOW_FOV_V_DEG) * DRONE_NPIX) / (FLOW_GRID_H * DRONE_FLOW_RES * DRONE_THETAPIX)
# CrazySim sensors_sitl.c protocol — CRTP_PORT_SETPOINT_SIM = 0x09,
# packet body = [type=6=SENSOR_FLOW_SIM, dpx_f32 LE, dpy_f32 LE, dt_f32 LE].
CRTP_PORT_SETPOINT_SIM = 0x09
SENSOR_FLOW_SIM        = 6


def flow_to_dpixel(dx_q1000: int, dy_q1000: int) -> tuple[float, float]:
    """Convert sentai.flow (milli-grid-px/frame) to PMW3901-equivalent dpixel."""
    dx_grid = dx_q1000 / 1000.0
    dy_grid = dy_q1000 / 1000.0
    fw_dx, fw_dy, lf_dx, lf_dy = BODY_XFORM
    fw_grid   = fw_dx * dx_grid + fw_dy * dy_grid
    left_grid = lf_dx * dx_grid + lf_dy * dy_grid
    return (fw_grid * _FLOW_SCALE_X, left_grid * _FLOW_SCALE_Y)


# ── PID params persistence + in-flight refinement ──────────────────
# Drone loads gains from a JSON file at startup, uses them, and saves
# them back at end of flight after any in-flight refinements.  Next
# flight loads the refined version — iterative learning across runs.
# If the file is missing (first flight ever), uses sensible defaults.
import json

PID_PARAMS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "pid_params.json")
PID_DEFAULTS = {
    # Working tuning from 2026-05-11 manual run (hit 11.6cm final dist).
    # Auto-calibration runs as ILC across flights: if THIS flight had
    # bad outcome (large overshoot or far from target at end), gains
    # adjust for NEXT flight.  If outcome was good, gains FREEZE.  The
    # "bad behaviour" detection at end of takeoff is the same logic.
    "KP_M": 0.5,
    "KD_M": 0.6,
    "V_MAX_M": 0.20,
    # ILC bounds
    "OVERSHOOT_TARGET_M": 0.20,    # acceptable peak err during flight
    "OVERSHOOT_HIGH_M":   0.40,    # >40cm overshoot → recalibrate (KP--)
    "FINAL_DIST_TIGHT_M": 0.15,    # within 15cm of target at end = converged
    "FINAL_DIST_LOOSE_M": 0.30,    # >30cm at end + no overshoot → too slow → KP++
    "KP_ADJUST_RATE": 0.10,
    "KD_ADJUST_RATE": 0.10,
    "KP_MIN": 0.20,
    "KP_MAX": 0.90,
    "KD_MIN": 0.30,
    "KD_MAX": 1.20,
    "_last_overshoot_x_m": 0.0,
    "_last_overshoot_y_m": 0.0,
    "_last_final_dist_m": 0.0,
}


def load_pid_params() -> dict:
    """Load gains from disk, falling back to DEFAULTS for missing keys."""
    p = dict(PID_DEFAULTS)
    try:
        with open(PID_PARAMS_PATH) as f:
            saved = json.load(f)
        p.update({k: v for k, v in saved.items() if k in PID_DEFAULTS})
        print(f"[pid] loaded params from {PID_PARAMS_PATH}: "
              f"KP_M={p['KP_M']:.3f} KD_M={p['KD_M']:.3f}",
              file=sys.stderr)
    except FileNotFoundError:
        print(f"[pid] no saved params at {PID_PARAMS_PATH} — using defaults "
              f"KP_M={p['KP_M']:.3f} KD_M={p['KD_M']:.3f}",
              file=sys.stderr)
    except Exception as e:
        print(f"[pid] WARN load failed: {e}; using defaults", file=sys.stderr)
    return p


def save_pid_params(p: dict, stable: bool):
    """Write refined params to disk.  Adds metadata for audit:
    last_run timestamp + whether converged."""
    out = dict(p)
    out["_last_run"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    out["_stable_at_end"] = bool(stable)
    try:
        with open(PID_PARAMS_PATH, "w") as f:
            json.dump(out, f, indent=2)
        print(f"[pid] saved refined params to {PID_PARAMS_PATH} "
              f"(stable={stable})", file=sys.stderr)
    except Exception as e:
        print(f"[pid] WARN save failed: {e}", file=sys.stderr)


def flow_conf_to_std(conf: int) -> float:
    """Conf-to-stdDev mapping mirrors _t_flow_to_drone.py DEFAULTS.
    Higher std = less EKF trust = drone moves more freely under noisy flow.
    sentai_sim phase-corr conf is uint8 (0..255).
    """
    if conf >= 200: return 1.0
    if conf >= 128: return 2.0
    if conf >= 64:  return 4.0
    return 8.0

# ── mission tuning ─────────────────────────────────────────────────────
TARGET_CLASS   = 16
IMG_W, IMG_H   = 300, 300
TARGET_Z       = float(os.environ.get("HOVER_TARGET_Z", "2.5"))
# Env-configurable so the altitude-normalization claim (gains
# learned at z=2.5 work at any z) can be validated experimentally.
GATE_Z         = 0.30
GAIN_M_PER_PX  = 0.004
V_MAX          = 0.20
HOVER_S        = 30.0
RATE_HZ        = 5
LOST_FRAMES    = 8
MIN_CONF_PERMIL = 300
TRACK_CONFIRMED = 1     # from sentai_tracker.h enum
SENTAI_SIM     = "/home/bogdan/work/coralmicro/build-sim/sim/sentai_sim"
MODEL          = "/home/bogdan/work/coralmicro/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class SentaiRepl:
    """Spawn sentai_sim, write Python commands, read line-buffered output."""

    def __init__(self):
        self.proc = subprocess.Popen(
            [SENTAI_SIM],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            bufsize=1, text=True,
        )
        self._lock = threading.Lock()
        time.sleep(1.0)   # let banner print

    def cmd(self, line: str, expect_value: bool = False, timeout_s: float = 2.0):
        """Send a single-line MP command, optionally read back one >>> value."""
        with self._lock:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
            if not expect_value:
                return None
            # Read until we see the next prompt-suffix line (heuristic).
            deadline = time.monotonic() + timeout_s
            out_lines = []
            while time.monotonic() < deadline:
                line = self.proc.stdout.readline()
                if not line:
                    break
                out_lines.append(line)
                if line.rstrip().endswith(">>> ") or "Traceback" in line:
                    break
            return out_lines

    def stop(self):
        try:
            self.proc.stdin.write("exit\n")
            self.proc.stdin.flush()
        except Exception:
            pass
        try:
            self.proc.wait(timeout=2.0)
        except Exception:
            self.proc.kill()


# Background reader keeps sentai_sim's stdout drained + parses STATE lines
# into a queue.  Earlier impl overwrote a single _latest_state slot which
# caused the host loop to miss most tid>0 transitions.
import queue as _queue
_state_queue: _queue.Queue = _queue.Queue(maxsize=4096)
_stop = threading.Event()
# state.tsv path — set once we know the per-exp dir (post env-var resolution).
_state_tsv_path = None
_state_tsv_f = None
_state_tsv_lock = threading.Lock()
# Fires when hover_logic prints HOVER_CENTERED — bbox within CENTER_THRESH
# for CENTERED_HOLD consecutive frames.  Host waits 3s post-centered then
# lands.
_centered_event = threading.Event()


def reader_thread(proc: subprocess.Popen):
    """Read sentai_sim stdout continuously; push STATE= tuples into queue
    AND log each STATE row as TSV to state_tsv_f for disk replay.  All
    experiment artifacts land on disk (per Sim.md best practice §10i)."""
    sim_log = open("/tmp/hover_sim.log", "w")
    while not _stop.is_set():
        line = proc.stdout.readline()
        if not line:
            break
        sim_log.write(line)
        idx = line.find("STATE=")
        if idx >= 0:
            payload = line[idx + len("STATE="):].strip()
            try:
                tup = ast.literal_eval(payload)
                try:
                    _state_queue.put_nowait(tup)
                except _queue.Full:
                    pass
                # ALSO write to state.tsv for disk-only replay.
                with _state_tsv_lock:
                    if _state_tsv_f is not None:
                        # Pad to 17 fields with -1 (so columns match).
                        padded = list(tup) + [-1] * (17 - len(tup))
                        _state_tsv_f.write("\t".join(str(v) for v in padded[:17]) + "\n")
            except Exception as e:
                print(f"[reader] parse err: {e}: {payload[:80]}", file=sys.stderr)
        elif "Traceback" in line or "ERROR" in line.upper():
            print(f"[reader] {line.rstrip()}", file=sys.stderr)
        elif "HOVER_LOGIC_" in line or "HOVER_CENTERED" in line or "HOVER_COAST_END" in line:
            print(f"[reader] {line.rstrip()[:200]}", file=sys.stderr)
            if "HOVER_CENTERED" in line:
                _centered_event.set()
    sim_log.close()


def main() -> int:
    print("[hover] spawn sentai_sim", file=sys.stderr)
    # Open state.tsv in per-experiment dir.  ALL artifacts (PPMs, flight.tsv,
    # state.tsv, hover.log, hover_sim.log) live in the same folder so each
    # experiment is fully self-contained on disk.  Sim.md §10i.
    global _state_tsv_path, _state_tsv_f
    exp_dir = os.environ.get("SENTAI_DUMP_FRAMES_DIR", "/tmp/sentai_frames")
    os.makedirs(exp_dir, exist_ok=True)
    _state_tsv_path = os.path.join(exp_dir, "state.tsv")
    _state_tsv_f = open(_state_tsv_path, "w")
    _state_tsv_f.write("iter\ttid\tcls\tconf\tcx\tcy\terr_x\terr_y\t"
                       "vx\tvy\tfvx\tfvy\tx1\ty1\tx2\ty2\tfseq\n")
    print(f"[hover] state.tsv → {_state_tsv_path}", file=sys.stderr)
    repl = SentaiRepl()

    # Drain banner
    t_reader = threading.Thread(target=reader_thread, args=(repl.proc,), daemon=True)
    t_reader.start()

    # Init: load model, enable tracker, start continuous pipeline.
    # `hover_logic.py` lives in the SIM virtual FS at <sim_fs_root>/.  The
    # SIM build wires `mp_lexer_new_from_file` + `mp_import_stat` through
    # `sim_fs_resolve()`, so a plain `import hover_logic` streams the file
    # via the MP lexer (no source-string heap copy that `exec(read_str())`
    # would force).  This is the same path firmware uses through FileX.
    setup = [
        'import sentai',
        f'sentai.tpu.load("{MODEL}")',
        'sentai.pipeline.tracker_camera(0, 58.0, 45.0)',
        'sentai.pipeline.tracker_pose(100, -1)',
        'sentai.pipeline.tracker_enable(True)',
        f'sentai.pipeline.start({RATE_HZ})',
        # Top-level loop in hover_logic.py emits "STATE= (...)" lines.
        # Import (not exec) so the lexer streams the file from the FS
        # instead of allocating the source string on the heap.
        'import hover_logic',
    ]
    for cmd in setup:
        repl.cmd(cmd)
    time.sleep(0.5)
    print("[hover] sentai_sim configured, tracker emitting", file=sys.stderr)

    # Concurrent: cflib drone control via MotionCommander (designed
    # specifically for flow-deck stacks per Bitcraze guidance — runs a
    # background thread streaming velocity setpoints at 10 Hz, handles
    # takeoff/landing, refreshes the cf2 watchdog automatically).
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    from cflib.crtp.crtpstack import CRTPPacket
    cflib.crtp.init_drivers()
    sync = SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache=None))
    sync.open_link()
    cf = sync.cf
    cf.param.set_value("stabilizer.estimator", 2)
    time.sleep(0.5)
    # Reset Kalman filter AFTER setting estimator + before flow injection.
    # Per Bitcraze: "external position can make EKF diverge to NaN ...
    # reset the EKF after starting to send position update".  Our reset
    # is pre-injection so EKF starts clean.
    cf.param.set_value("kalman.resetEstimation", 1)
    time.sleep(0.5)
    cf.param.set_value("kalman.resetEstimation", 0)
    time.sleep(2.0)   # let EKF settle

    # Subscribe to drone attitude + position (EKF state) at 100 Hz.  Used
    # for: (a) bbox tilt compensation, (b) flight.tsv log for analysis,
    # (c) overlay metadata in the final MP4.
    flight_log_path = os.path.join(
        os.environ.get("SENTAI_DUMP_FRAMES_DIR", "/tmp/sentai_frames"),
        "flight.tsv")
    flight_log_f = open(flight_log_path, "w")
    flight_log_f.write("ts\troll\tpitch\tyaw\tx\ty\tz\n")
    def _att_cb(timestamp, data, logconf):
        global _drone_roll_deg, _drone_pitch_deg, _drone_yaw_deg
        global _drone_x, _drone_y, _drone_z
        with _att_lock:
            _drone_roll_deg = data['stateEstimate.roll']
            _drone_pitch_deg = data['stateEstimate.pitch']
            _drone_yaw_deg = data['stateEstimate.yaw']
            _drone_x = data['stateEstimate.x']
            _drone_y = data['stateEstimate.y']
            _drone_z = data['stateEstimate.z']
        flight_log_f.write(f"{timestamp}\t{_drone_roll_deg:.3f}\t{_drone_pitch_deg:.3f}\t"
                           f"{_drone_yaw_deg:.3f}\t{_drone_x:.4f}\t{_drone_y:.4f}\t{_drone_z:.4f}\n")
    log_att = LogConfig(name="att", period_in_ms=20)  # 50 Hz (8 vars × 4 B = 32 B < 26 B limit so split)
    log_att.add_variable("stateEstimate.roll", "float")
    log_att.add_variable("stateEstimate.pitch", "float")
    log_att.add_variable("stateEstimate.yaw", "float")
    log_att.add_variable("stateEstimate.x", "float")
    log_att.add_variable("stateEstimate.y", "float")
    log_att.add_variable("stateEstimate.z", "float")
    try:
        cf.log.add_config(log_att)
        log_att.data_received_cb.add_callback(_att_cb)
        log_att.start()
        print(f"[hover] attitude+pos log subscribed @ 50 Hz → {flight_log_path}",
              file=sys.stderr)
    except Exception as e:
        print(f"[hover] WARN attitude log failed: {e}", file=sys.stderr)

    # MotionCommander takeoff to TARGET_Z (2.5m).  MC auto-streams hover
    # setpoints at 10Hz on a background thread → satisfies cf2 watchdog.
    HOLD_Z = TARGET_Z
    PLAUSIBLE_CAT_CLASSES = {15, 16, 21}
    print(f"[hover] MotionCommander takeoff → {HOLD_Z:.1f}m", file=sys.stderr)
    mc = MotionCommander(sync, default_height=HOLD_Z)
    mc.take_off(height=HOLD_Z, velocity=0.5)   # 0.5 m/s climb

    # ─── ALTITUDE SWEEP CALIBRATION (CHIRP-style multi-DOF excitation) ───
    # Quad-M principle: rich-enough input (Manoeuvres) reveals all plant
    # modes.  We sweep z by ±0.25m around HOLD_Z for one full cycle while
    # simultaneously tracking the cat (XY centering active).  Bbox metrics
    # are binned by altitude — if performance is altitude-invariant the
    # current gains are universal; if performance degrades at low z (or
    # high z), gain scheduling KP(z) / KD(z) is justified.
    # Enabled via env var SENTAI_ALT_CAL=1 (default off — adds ~20s overhead).
    if os.environ.get("SENTAI_ALT_CAL", "0") == "1":
        print("[hover] altitude-sweep calibration: ±0.25m around HOLD_Z, "
              "1 cycle / 20s with XY centering active", file=sys.stderr)
        cal_buckets = {"low": [], "mid_low": [], "mid_high": [], "high": []}
        cal_t0 = time.monotonic()
        CAL_DURATION_S = 20.0
        CAL_AMP_M = 0.25
        while time.monotonic() - cal_t0 < CAL_DURATION_S:
            t = time.monotonic() - cal_t0
            z_cmd = HOLD_Z + CAL_AMP_M * math.sin(2 * math.pi * t / CAL_DURATION_S)
            # Get current state for bucketing
            with _att_lock:
                z_now = _drone_z
            # Bucket by current z relative to HOLD_Z
            dz = z_now - HOLD_Z
            if dz < -0.15:
                bucket = "low"
            elif dz < 0:
                bucket = "mid_low"
            elif dz < 0.15:
                bucket = "mid_high"
            else:
                bucket = "high"
            # Drain any STATE rows since last poll, log bbox err per bucket
            try:
                while True:
                    s = _state_queue.get_nowait()
                    if len(s) >= 16 and s[1] > 0:
                        cal_buckets[bucket].append((abs(s[6]), abs(s[7])))  # (|err_x|, |err_y|)
            except _queue.Empty:
                pass
            # Send vertical sweep cmd + zero lateral velocity (calibration: pure z motion)
            try:
                mc.start_linear_motion(0, 0, (z_cmd - z_now) * 0.5)
            except Exception:
                pass
            time.sleep(0.1)
        # Report per-bucket overshoot
        print("[cal] per-altitude bbox tracking precision:", file=sys.stderr)
        for name, errs in cal_buckets.items():
            if errs:
                mean_ex = sum(e[0] for e in errs) / len(errs)
                mean_ey = sum(e[1] for e in errs) / len(errs)
                print(f"[cal]   {name:9s}: {len(errs):4d} samples, "
                      f"mean |err_x|={mean_ex:5.1f}px, |err_y|={mean_ey:5.1f}px",
                      file=sys.stderr)
            else:
                print(f"[cal]   {name:9s}: 0 samples", file=sys.stderr)
        # Return to HOLD_Z before main hover
        try:
            mc.start_linear_motion(0, 0, 0)
        except Exception:
            pass
        time.sleep(0.5)
    # Confirm airborne via gz pose query before starting hover-over.
    try:
        pose = subprocess.run(
            ["distrobox", "enter", "crazysim-garden", "--",
             "gz", "model", "-m", "crazyflie_0", "-p"],
            capture_output=True, text=True, timeout=3.0,
        ).stdout
        for ln in pose.split("\n"):
            if "[" in ln and "]" in ln and ln.strip().startswith("["):
                print(f"[hover] takeoff pose: {ln.strip()}", file=sys.stderr)
                break
    except Exception as e:
        print(f"[hover] pose query failed: {e}", file=sys.stderr)

    print(f"[hover] hover-over for {HOVER_S}s targeting class={TARGET_CLASS}",
          file=sys.stderr)
    # The MP-side `hover_logic.py` script (loaded via
    # exec(sentai.fs.read_str("hover_logic.py"))) emits STATE= tuples
    # every 200 ms with the chosen (vx, vy) in mm/s.  We just forward
    # them to cflib.  If no STATE in a few ticks, hold steady.
    # === Load PID gains from disk (or DEFAULTS) for this flight ===
    pid = load_pid_params()

    miss = 0
    vx_body = vy_body = 0.0
    n_lock = 0
    n_flow_sent = 0
    last_flow_send_t = time.monotonic()
    centered_at = None        # monotonic timestamp when hover_logic first reported HOVER_CENTERED
    HOLD_AFTER_CENTER_S = 3.0
    HOVER_TIMEOUT_S = 60.0    # safety cap if drone never centers
    # Adaptive-PID tracking state — refined gains as flight progresses.
    pid_err_hist = []          # last N (err_x_m, err_y_m) for cycle analysis
    pid_overshoot_events = 0    # tick count where |err| past last sign-flip+threshold
    pid_stable_count = 0        # consecutive ticks within stable bounds
    pid_converged = False
    pid_last_sign_x = 0
    pid_last_sign_y = 0
    pid_max_err_since_flip_x = 0
    pid_max_err_since_flip_y = 0
    t0 = time.monotonic()
    # Loop until HOVER_CENTERED fires + 3s elapsed, OR safety timeout.
    while True:
        elapsed = time.monotonic() - t0
        if centered_at is not None and (time.monotonic() - centered_at) >= HOLD_AFTER_CENTER_S:
            print(f"[hover] HOLD_AFTER_CENTER_S ({HOLD_AFTER_CENTER_S}s) elapsed — landing",
                  file=sys.stderr)
            break
        if elapsed >= HOVER_TIMEOUT_S:
            print(f"[hover] HOVER_TIMEOUT_S ({HOVER_TIMEOUT_S}s) — landing without center",
                  file=sys.stderr)
            break
        if _centered_event.is_set() and centered_at is None:
            centered_at = time.monotonic()
            print(f"[hover] CENTERED detected — holding {HOLD_AFTER_CENTER_S}s then land",
                  file=sys.stderr)
            _centered_event.clear()
        # Drain queue — process every STATE emitted by hover_logic since
        # last host poll, not just the latest.  Last lock state wins for
        # the actual setpoint.
        states_this_tick = []
        try:
            while True:
                states_this_tick.append(_state_queue.get_nowait())
        except _queue.Empty:
            pass
        for state in states_this_tick:
            # STATE: ..., vx_cmd_raw, vy_cmd_raw, flow_dx_q, flow_dy_q,
            # x1, y1, x2, y2.  We RECOMPUTE vx/vy host-side after applying
            # attitude compensation on the bbox centroid (drone tilt
            # rotates camera FOV → bbox appears to shift in image without
            # drone translating; correct by subtracting f_px*tilt_rad).
            if len(state) >= 16:
                # 17-field has fseq at end; we don't need it host-side.
                it, tid, cls, conf, cx, cy, ex, ey, _, _, fvx, fvy, x1, y1, x2, y2 = state[:16]
                # Tilt-compensated centroid.
                cx_c, cy_c = attitude_compensate(cx, cy)
                err_x_c = cx_c - SSD_INPUT_W // 2
                err_y_c = cy_c - SSD_INPUT_H // 2
                # Altitude-aware PD on tilt-compensated centroid.  Gains
                # come from disk (pid_params.json) and refine in-flight.
                KP_M = pid["KP_M"]
                KD_M = pid["KD_M"]
                V_MAX_M = pid["V_MAX_M"]
                # Need z to convert.  Read from latest attitude log
                # (updated by _att_cb at 50 Hz).
                with _att_lock:
                    z_now = max(0.1, _drone_z)
                m_per_px_x = 2 * z_now * math.tan(CAM_FOV_H_RAD/2) / SSD_INPUT_W
                m_per_px_y = 2 * z_now * math.tan(CAM_FOV_V_RAD/2) / SSD_INPUT_H
                err_x_m = err_x_c * m_per_px_x
                err_y_m = err_y_c * m_per_px_y
                if 'prev_err_x_m' not in dir():
                    prev_err_x_m = err_x_m
                    prev_err_y_m = err_y_m
                # dt is the host poll interval ~0.2s.  Derivative computed
                # per-tick (close enough); proper d/dt would need actual dt.
                d_err_x_m = err_x_m - prev_err_x_m
                d_err_y_m = err_y_m - prev_err_y_m
                vx_m = max(-V_MAX_M, min(V_MAX_M, KP_M * err_y_m + KD_M * d_err_y_m))
                vy_m = max(-V_MAX_M, min(V_MAX_M, KP_M * err_x_m + KD_M * d_err_x_m))
                vx_mm = int(vx_m * 1000)
                vy_mm = int(vy_m * 1000)
                prev_err_x_m = err_x_m
                prev_err_y_m = err_y_m

                # ILC tracking — just record max overshoot during flight.
                # End-of-flight code adjusts gains for NEXT flight.
                if abs(err_x_m) > abs(pid_max_err_since_flip_x):
                    pid_max_err_since_flip_x = err_x_m
                if abs(err_y_m) > abs(pid_max_err_since_flip_y):
                    pid_max_err_since_flip_y = err_y_m
            elif len(state) >= 12:
                it, tid, cls, conf, cx, cy, ex, ey, vx_mm, vy_mm, fvx, fvy = state[:12]
                cx_c, cy_c, err_x_c, err_y_c = cx, cy, ex, ey
            else:
                it, tid, cls, conf, cx, cy, ex, ey, vx_mm, vy_mm = state[:10]
                fvx = fvy = 0
                cx_c, cy_c, err_x_c, err_y_c = cx, cy, ex, ey
            # Forward fresh flow to cf2 (PMW3901-style conf→std mapping).
            if fvx or fvy:
                now = time.monotonic()
                dt = max(0.001, min(0.2, now - last_flow_send_t))
                last_flow_send_t = now
                dpx, dpy = flow_to_dpixel(fvx, fvy)
                std = 4.0
                pk = CRTPPacket()
                pk.port = CRTP_PORT_SETPOINT_SIM
                pk.channel = 0
                pk.data = struct.pack("<Bffff", SENSOR_FLOW_SIM, dpx, dpy, dt, std)
                try:
                    cf.send_packet(pk)
                    n_flow_sent += 1
                except Exception as e:
                    print(f"[flow] send err: {e}", file=sys.stderr)
            if tid > 0 and conf >= MIN_CONF_PERMIL:
                vx_body = vx_mm / 1000.0
                vy_body = vy_mm / 1000.0
                miss = 0
                n_lock += 1
                with _att_lock:
                    rd, pd = _drone_roll_deg, _drone_pitch_deg
                print(f"[hover] LOCK iter={it} id={tid} cls={cls} conf={conf} "
                      f"raw=({cx},{cy}) corr=({cx_c},{cy_c}) "
                      f"err_c=({err_x_c:+4d},{err_y_c:+4d}) "
                      f"att=(r{rd:+4.1f},p{pd:+4.1f}) "
                      f"cmd=({vx_body:+.3f},{vy_body:+.3f})", file=sys.stderr)
            else:
                miss += 1
                if miss == LOST_FRAMES:
                    print(f"[hover] target LOST (iter={it})", file=sys.stderr)
                    vx_body = vy_body = 0.0
        # MotionCommander streams setpoints on its OWN background thread
        # @ 10 Hz — we just call start_linear_motion each time we have a
        # new command.  zero-velocity is implicit between calls until
        # next update arrives.
        try:
            mc.start_linear_motion(vx_body, vy_body, 0.0)
        except Exception as e:
            print(f"[hover] MC update err: {e}", file=sys.stderr)
        time.sleep(1.0 / RATE_HZ)
    print(f"[hover] total LOCK events: {n_lock}  flow packets sent to cf2: {n_flow_sent}  HOLD_Z={HOLD_Z:.2f}m",
          file=sys.stderr)
    # === ILC: adjust gains for NEXT flight based on observed overshoot ===
    overshoot_x = abs(pid_max_err_since_flip_x)
    overshoot_y = abs(pid_max_err_since_flip_y)
    overshoot_max = max(overshoot_x, overshoot_y)
    # Final dist to target estimated by err at end of hover phase
    final_err_m = (err_x_c * math.tan(CAM_FOV_H_RAD/2) / (SSD_INPUT_W/2)
                   if 'err_x_c' in dir() else 0) * _drone_z
    final_dist_m = math.hypot(err_x_m, err_y_m) if 'err_x_m' in dir() else 0.5
    print(f"[pid] flight summary: max overshoot x={overshoot_x:.3f}m "
          f"y={overshoot_y:.3f}m, final err {final_dist_m:.3f}m",
          file=sys.stderr)
    # Rules — bounded by KP_MIN/MAX, KD_MIN/MAX:
    if overshoot_max > pid["OVERSHOOT_HIGH_M"]:
        # Too much overshoot → reduce KP, boost KD
        pid["KP_M"] = max(pid["KP_M"] * (1 - pid["KP_ADJUST_RATE"]), pid["KP_MIN"])
        pid["KD_M"] = min(pid["KD_M"] * (1 + pid["KD_ADJUST_RATE"]), pid["KD_MAX"])
        print(f"[pid] ILC: overshoot {overshoot_max:.2f}m > "
              f"{pid['OVERSHOOT_HIGH_M']:.2f}m → REDUCE KP "
              f"({pid['KP_M']:.3f}), BOOST KD ({pid['KD_M']:.3f})",
              file=sys.stderr)
    elif overshoot_max < pid["OVERSHOOT_TARGET_M"] and final_dist_m > pid["FINAL_DIST_LOOSE_M"]:
        # Drone too cautious — increase KP slightly
        pid["KP_M"] = min(pid["KP_M"] * (1 + pid["KP_ADJUST_RATE"]), pid["KP_MAX"])
        print(f"[pid] ILC: drone too slow (final {final_dist_m:.2f}m off + "
              f"low overshoot) → BOOST KP ({pid['KP_M']:.3f})",
              file=sys.stderr)
    else:
        print(f"[pid] ILC: behaviour within bounds → gains FROZEN",
              file=sys.stderr)
    pid["_last_overshoot_x_m"] = round(overshoot_x, 4)
    pid["_last_overshoot_y_m"] = round(overshoot_y, 4)
    pid["_last_final_dist_m"] = round(final_dist_m, 4)
    stable_for_save = (overshoot_max <= pid["OVERSHOOT_TARGET_M"] and
                       final_dist_m <= pid["FINAL_DIST_TIGHT_M"])
    save_pid_params(pid, stable=stable_for_save)

    print("[hover] MotionCommander landing", file=sys.stderr)
    try:
        mc.land(velocity=0.4)
    except Exception as e:
        print(f"[hover] mc.land err: {e}", file=sys.stderr)
    # mc.land handles the descent profile + send_stop_setpoint internally.
    try:
        log_att.stop()
    except Exception:
        pass
    sync.close_link()

    _stop.set()
    repl.stop()
    # Flush + close per-experiment disk artifacts.
    try:
        flight_log_f.close()
    except Exception:
        pass
    with _state_tsv_lock:
        if _state_tsv_f is not None:
            _state_tsv_f.close()
    print(f"[hover] artifacts on disk: {exp_dir}/", file=sys.stderr)
    print(f"[hover]   frame_*.ppm  ({len(list(__import__('glob').glob(exp_dir + '/frame_*.ppm')))} files)",
          file=sys.stderr)
    print(f"[hover]   flight.tsv   (drone XYZ + RPY @ 50 Hz)", file=sys.stderr)
    print(f"[hover]   state.tsv    (tracker + flow + cmd @ 5 Hz)", file=sys.stderr)
    print(f"[hover] done", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

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
TARGET_Z       = 2.5   # higher altitude → larger FOV → easier to see cat picture
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
# Fires when hover_logic prints HOVER_CENTERED — bbox within CENTER_THRESH
# for CENTERED_HOLD consecutive frames.  Host waits 3s post-centered then
# lands.
_centered_event = threading.Event()


def reader_thread(proc: subprocess.Popen):
    """Read sentai_sim stdout continuously; push STATE= tuples into queue
    so the host hover loop sees every transition, not just the latest."""
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

    # Concurrent: cflib drone control
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

    from cflib.crtp.crtpstack import CRTPPacket
    cflib.crtp.init_drivers()
    sync = SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache=None))
    sync.open_link()
    cf = sync.cf
    cf.param.set_value("stabilizer.estimator", 2)
    # Longer settle: EKF needs ~1-2s to converge from cold-start after
    # the param flip; without flow injection it relies solely on baro+IMU
    # and can otherwise refuse takeoff (SUP lock on tumble).
    time.sleep(3.0)

    # Staged takeoff — drone camera is BELOW ground for z<1m on this drone
    # model (user-confirmed via Gazebo GUI PIP) so any "lock" at <1m is a
    # spurious hallucination on underground checkerboard.  Filter by:
    #   (a) altitude >= MIN_LOCK_Z
    #   (b) tracker class in PLAUSIBLE_CAT_CLASSES (MobileNet COCO17
    #       conflates cat with dog and bear on this photo).
    print("[hover] staged takeoff: climbing until cat detected (need z>=1m, class∈{15,16,21})",
          file=sys.stderr)
    target_z = 0.0
    cat_locked_at_z = None
    MAX_CLIMB_Z = 3.0
    MIN_LOCK_Z = 2.0   # raised so 1×1m cat picture fits in FOV with margin
    PLAUSIBLE_CAT_CLASSES = {15, 16, 21}
    while target_z < MAX_CLIMB_Z:
        target_z = min(MAX_CLIMB_Z, target_z + 0.5 * 0.1)  # 0.5 m/s = +0.05m/tick at 10Hz → MAX 3m in 6s
        cf.commander.send_hover_setpoint(0, 0, 0, target_z)
        time.sleep(0.1)
        peek_lock = False
        try:
            while True:
                s = _state_queue.get_nowait()
                if (len(s) >= 12 and s[1] > 0 and s[3] >= MIN_CONF_PERMIL
                        and s[2] in PLAUSIBLE_CAT_CLASSES):
                    peek_lock = True
                    break
        except _queue.Empty:
            pass
        if peek_lock and target_z >= MIN_LOCK_Z:
            cat_locked_at_z = target_z
            print(f"[hover] CAT LOCKED at altitude {target_z:.2f} m — "
                  f"start centering", file=sys.stderr)
            break
    if cat_locked_at_z is None:
        print(f"[hover] climbed to {target_z:.2f} m without cat lock — proceeding anyway", file=sys.stderr)
        cat_locked_at_z = target_z
    # HOLD at lock altitude — do NOT keep climbing.  User feedback
    # 2026-05-11: "cand se detecteaza pisica sa nu se mai ridice, sa se
    # miste spre centrul boxului".  Hover loop below sends (vx, vy, 0,
    # HOLD_Z) so lateral motion happens but altitude stays put.
    HOLD_Z = cat_locked_at_z
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
    miss = 0
    vx_body = vy_body = 0.0
    n_lock = 0
    n_flow_sent = 0
    last_flow_send_t = time.monotonic()
    centered_at = None        # monotonic timestamp when hover_logic first reported HOVER_CENTERED
    HOLD_AFTER_CENTER_S = 3.0
    HOVER_TIMEOUT_S = 60.0    # safety cap if drone never centers
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
            # STATE now has 16 fields: ..., vx_cmd, vy_cmd, flow_dx_q,
            # flow_dy_q, x1, y1, x2, y2.  Older runs had 12 / 10 fields.
            if len(state) >= 12:
                it, tid, cls, conf, cx, cy, ex, ey, vx_mm, vy_mm, fvx, fvy = state[:12]
            else:
                it, tid, cls, conf, cx, cy, ex, ey, vx_mm, vy_mm = state[:10]
                fvx = fvy = 0
            # Forward each fresh flow sample to cf2 EKF as a
            # SENSOR_FLOW_SIM CRTP packet (mirrors what the PMW3901 flow
            # deck would push — same conf→std mapping the HW deck uses
            # via _t_flow_to_drone.py, so noisy phase-corr samples get
            # high std and the EKF weighs them less).
            if fvx or fvy:
                now = time.monotonic()
                dt = max(0.001, min(0.2, now - last_flow_send_t))
                last_flow_send_t = now
                dpx, dpy = flow_to_dpixel(fvx, fvy)
                # State has fconf at index 12? Not yet — fconf not in STATE.
                # Use fvx+fvy magnitude as a crude conf proxy until we plumb
                # conf through STATE.  Better: just default mid std for now.
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
                print(f"[hover] LOCK iter={it} id={tid} cls={cls} conf={conf} "
                      f"cxy=({cx},{cy}) err=({ex:+4d},{ey:+4d}) "
                      f"flow=({fvx:+4d},{fvy:+4d}mm/s) "
                      f"cmd=({vx_body:+.3f},{vy_body:+.3f})", file=sys.stderr)
            else:
                miss += 1
                if miss == LOST_FRAMES:
                    print(f"[hover] target LOST (iter={it})", file=sys.stderr)
                    vx_body = vy_body = 0.0
        cf.commander.send_hover_setpoint(vx_body, vy_body, 0, HOLD_Z)
        time.sleep(1.0 / RATE_HZ)
    print(f"[hover] total LOCK events: {n_lock}  flow packets sent to cf2: {n_flow_sent}  HOLD_Z={HOLD_Z:.2f}m",
          file=sys.stderr)

    print("[hover] landing", file=sys.stderr)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        z = max(0.05, TARGET_Z * (1.0 - (time.monotonic() - t0) / 3.0))
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.05)
    cf.commander.send_stop_setpoint()
    sync.close_link()

    _stop.set()
    repl.stop()
    print("[hover] done", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

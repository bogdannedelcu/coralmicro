#!/usr/bin/env python3
"""
cflib_takeoff_with_flow.py — flow-corrected hover.

Identical takeoff/hover/land profile to cflib_takeoff_no_flow.py, but a
background thread reads flow snapshots from /tmp/sentai_flow_out.sock
(published by the C++ gz_to_uds_bridge running inside the crazysim-garden
distrobox) and pushes them into the cf2 EKF via cflib LOCALIZATION ch=1.

Switch the EKF to the Extended Kalman variant (= 2) which actually
consumes flow; the Complementary one (= 1) silently drops it.
"""
import csv
import math
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

URI         = "udp://127.0.0.1:19850"
GATE_Z      = 0.30   # m — start flow injection here (per session plan)
TARGET_Z    = 1.0    # m — final hover altitude
GATE_HOLD_S = 2.0    # hold at GATE_Z this long before climbing to TARGET_Z
HOVER_S     = 25.0   # hover at TARGET_Z (with flow stabilising X/Y)
LAND_S      = 3.0
LOG_PERIOD_MS = 50
FLOW_SOCK   = "/tmp/sentai_flow_out.sock"

# Mirror of DEFAULTS from sim/scripts/gz_to_camera_bridge.py — same FOV,
# body_xform, EKF geometry constants.
CFG = dict(
    fov_h_deg            = 58.0,
    fov_v_deg            = 45.0,
    grid_w               = 80,
    grid_h               = 60,
    drone_npix           = 35.0,
    drone_thetapix_rad   = 0.71674,
    drone_flow_resolution= 0.10,
    body_xform           = (-1.0, 0.0, 0.0, +1.0),
    min_dt_s             = 1e-3,
    max_dt_s             = 0.20,
    conf_thresh_high     = 200,
    conf_thresh_mid      = 128,
    conf_thresh_low      = 64,
    std_floor            = 1.0,
    std_low              = 2.0,
    std_mid              = 4.0,
    std_ceil             = 8.0,
)
SCALE_X = (math.radians(CFG["fov_h_deg"]) * CFG["drone_npix"]) / (
    CFG["grid_w"] * CFG["drone_flow_resolution"] * CFG["drone_thetapix_rad"])
SCALE_Y = (math.radians(CFG["fov_v_deg"]) * CFG["drone_npix"]) / (
    CFG["grid_h"] * CFG["drone_flow_resolution"] * CFG["drone_thetapix_rad"])

REPLY_FMT   = "<IIiiIQ"
REPLY_LEN   = struct.calcsize(REPLY_FMT)
REPLY_MAGIC = 0x46524C31

OUT = Path(__file__).parent / "csv" / "with_flow.csv"
OUT.parent.mkdir(parents=True, exist_ok=True)


def _conf_to_std(c):
    if c >= CFG["conf_thresh_high"]: return CFG["std_floor"]
    if c >= CFG["conf_thresh_mid"]:  return CFG["std_low"]
    if c >= CFG["conf_thresh_low"]:  return CFG["std_mid"]
    return CFG["std_ceil"]


def _altitude_std_scale(z):
    """Scale flow standard deviation by altitude.

    cf2 mm_flow.c partial-derivative wrt KC_STATE_Z is proportional to
    1/z^2 (R[2][2] * dx_g / -z^2).  Low altitude => any flow update
    bleeds strongly into the Z estimate, causing the Z swings we
    observed (1.24 m up / -0.19 m down at z_target=1.0 m).  Compensate
    by inflating std at low altitude so the EKF down-weights flow there.

    Returns a multiplier in [1.0, 8.0]:
      z >= 1.0 m  -> 1.0  (full trust)
      z == 0.5 m  -> 4.0  (mid trust)
      z <= 0.30 m -> 8.0  (essentially noise)
    """
    if z <= 0.30: return 8.0
    if z >= 1.0:  return 1.0
    # Linear ramp in between.
    return 1.0 + 7.0 * (1.0 - z) / 0.7


def _to_body(dx_q, dy_q):
    fw_dx, fw_dy, lf_dx, lf_dy = CFG["body_xform"]
    dx_g = dx_q / 1000.0
    dy_g = dy_q / 1000.0
    fw   = fw_dx * dx_g + fw_dy * dy_g
    left = lf_dx * dx_g + lf_dy * dy_g
    return (fw * SCALE_X, left * SCALE_Y)


class FlowReceiver(threading.Thread):
    """Background thread: read flow_out UDS, push into cf via send_packet.

    Per embeded.md "bounded behavior" + "explicit failure semantics":
    - All sock I/O has timeouts (no infinite blocks).
    - Flow packets are GATED on `enable.is_set()` — main thread releases
      the gate only after the drone is airborne (z > MIN_INJECT_Z).  This
      avoids feeding the cf2 EKF noisy/wrong ground-state observations
      that would lock the supervisor before takeoff completes.
    - Disconnect / EOF detected and logged; thread exits cleanly so the
      main thread's join() returns.
    """
    MIN_INJECT_Z   = 0.30        # metres — gate threshold per embeded.md
    MAX_TILT_RAD   = 0.35        # ~20° — skip flow above this attitude
                                  # (mm_flow.c R[2][2] term degrades fast at
                                  # large tilt, leaks into KC_STATE_Z)

    def __init__(self, cf, stop_evt, enable_evt, altitude_ref):
        super().__init__(daemon=True)
        self.cf = cf
        self.stop_evt = stop_evt
        self.enable_evt = enable_evt   # main thread sets when z > MIN_INJECT_Z
        self.altitude_ref = altitude_ref  # dict {"z","roll","pitch"} updated by main
        self.n_sent = 0
        self.n_drop = 0
        self.n_gated = 0           # flow snapshots dropped because not airborne
        self.connected = False     # main thread checks this after start()

    def run(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        for _ in range(20):
            try:
                sock.connect(FLOW_SOCK)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(0.5)
        else:
            print(f"[flow] FATAL: could not connect {FLOW_SOCK} — "
                  "main thread will see no flow!", file=sys.stderr)
            self.connected = False
            return
        # CRITICAL fix (code review HIGH): bounded recv so stop_evt is
        # observed within 0.5s and the join() actually completes.
        sock.settimeout(0.5)
        self.connected = True
        print(f"[flow] connected {FLOW_SOCK}", file=sys.stderr)

        t_last = time.monotonic()
        buf = b""
        while not self.stop_evt.is_set():
            try:
                chunk = sock.recv(REPLY_LEN * 4)
            except socket.timeout:
                continue        # observe stop_evt and re-poll
            except OSError:
                break
            if not chunk:
                print("[flow] sock EOF — bridge died?", file=sys.stderr)
                break
            buf += chunk
            while len(buf) >= REPLY_LEN:
                rec, buf = buf[:REPLY_LEN], buf[REPLY_LEN:]
                rmagic, seq, dx, dy, conf, lat = struct.unpack(REPLY_FMT, rec)
                if rmagic != REPLY_MAGIC: continue
                now = time.monotonic()
                dt  = max(CFG["min_dt_s"], min(CFG["max_dt_s"], now - t_last))
                t_last = now
                # Gate per embeded.md: do not feed flow into EKF until
                # drone is high enough that the camera actually sees the
                # ground (and not the underside of the ground plane).
                if not self.enable_evt.is_set():
                    self.n_gated += 1
                    continue
                # Bridge marks duplicate frames + saturation as conf=0.
                # Skip them - feeding them with high std would still nudge
                # the EKF; just drop entirely is safer.
                if conf == 0:
                    self.n_gated += 1
                    continue
                dpx, dpy = _to_body(dx, dy)
                std = _conf_to_std(conf) * _altitude_std_scale(self.altitude_ref["z"])
                pk = CRTPPacket()
                pk.port = CRTPPort.LOCALIZATION
                pk.channel = 1
                pk.data = struct.pack("<fhhfHH",
                                       float(dt),
                                       int(round(dpx)),
                                       int(round(dpy)),
                                       float(std), 0, 0)
                try:
                    self.cf.send_packet(pk)
                    self.n_sent += 1
                except Exception as e:
                    self.n_drop += 1
                    if self.n_drop % 50 == 1:
                        print(f"[flow] send err: {e}", file=sys.stderr)
        try: sock.close()
        except Exception: pass
        print(f"[flow] thread exit: sent={self.n_sent} drop={self.n_drop}",
              file=sys.stderr)


def main():
    cflib.crtp.init_drivers()
    print(f"[with_flow] linking {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache=None)) as scf:
        cf = scf.cf
        print("[with_flow] linked")

        # EKF variant 2 = Extended Kalman; consumes flow + extpos.
        cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(1.0)
        print("[with_flow] estimator switched to EKF (2)")

        stop_evt    = threading.Event()
        enable_evt  = threading.Event()   # set when z > MIN_INJECT_Z
        altitude    = {"z": 0.0}          # mutated by log callback below
        flow = FlowReceiver(cf, stop_evt, enable_evt, altitude)
        flow.start()
        # CRITICAL fix (code review HIGH): operator MUST know if flow is
        # actually being injected.  Bail loudly within 5s if not connected.
        time.sleep(2.0)
        if not flow.connected:
            print("[with_flow] FATAL: FlowReceiver did not connect to "
                  f"{FLOW_SOCK} — gz_to_uds_bridge must be running!",
                  file=sys.stderr)
            stop_evt.set()
            return 2

        log_cfg = LogConfig(name="pose", period_in_ms=LOG_PERIOD_MS)
        log_cfg.add_variable("stateEstimate.x", "float")
        log_cfg.add_variable("stateEstimate.y", "float")
        log_cfg.add_variable("stateEstimate.z", "float")

        rows = []
        def cb(ts, data, _):
            z = data["stateEstimate.z"]
            altitude["z"] = z
            if z > FlowReceiver.MIN_INJECT_Z and not enable_evt.is_set():
                enable_evt.set()
                print(f"[gate OPEN] z={z:.2f} > {FlowReceiver.MIN_INJECT_Z}, "
                      "flow injection enabled", flush=True)
            rows.append((ts, data["stateEstimate.x"],
                         data["stateEstimate.y"], z))

        cf.log.add_config(log_cfg)
        log_cfg.data_received_cb.add_callback(cb)
        log_cfg.start()

        # New takeoff profile (per session plan 2026-05-10):
        #   1. ramp 0 -> GATE_Z (0.30 m) over ~1 s — no flow yet (gate closed)
        #   2. hold at GATE_Z for GATE_HOLD_S so the EKF settles its R matrix
        #      before flow starts pushing observations through hx[KC_STATE_Z]
        #   3. flow gate opens automatically when stateEstimate.z > GATE_Z
        #   4. ramp GATE_Z -> TARGET_Z over 2 s; flow std drops to 1.0
        #      smoothly via _altitude_std_scale()
        #   5. hover HOVER_S at TARGET_Z with flow stabilising X/Y
        #   6. land
        print(f"[with_flow] ramp 0 -> {GATE_Z} m (1s)", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0,
                                              GATE_Z * (time.monotonic() - t0))
            time.sleep(0.05)

        print(f"[with_flow] hold {GATE_HOLD_S}s at {GATE_Z} m "
              "(flow gate opens here)", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < GATE_HOLD_S:
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, GATE_Z)
            time.sleep(0.05)

        print(f"[with_flow] climb {GATE_Z} -> {TARGET_Z} m (2s) — "
              "flow std smoothly drops via altitude scale", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            frac = (time.monotonic() - t0) / 2.0
            z = GATE_Z + (TARGET_Z - GATE_Z) * frac
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)
            time.sleep(0.05)

        print(f"[with_flow] hover {HOVER_S}s at z={TARGET_Z} m "
              "with flow injection", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < HOVER_S:
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, TARGET_Z)
            time.sleep(0.05)

        print("[with_flow] landing ...", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < LAND_S:
            t = 1.0 - (time.monotonic() - t0) / LAND_S
            z = max(0.05, TARGET_Z * t)
            cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)
            time.sleep(0.1)

        cf.commander.send_stop_setpoint()
        log_cfg.stop()
        stop_evt.set()
        flow.join(timeout=2.0)

    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "x", "y", "z"])
        w.writerows(rows)
    print(f"[with_flow] wrote {len(rows)} rows -> {OUT}")


if __name__ == "__main__":
    sys.exit(main() or 0)

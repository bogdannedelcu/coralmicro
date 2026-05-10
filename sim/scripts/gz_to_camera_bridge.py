#!/usr/bin/env python3
"""
gz_to_camera_bridge.py — full Phase 4 bridge:

    Gazebo /downward_cam/image
        -> UDS to sentai_sim (frames in)
        -> UDS reply with flow snapshot
        -> cflib cf.send_flow() to cf2 SITL on UDP 19850

Pure I/O on the Python side: no per-pixel work, no resize, no FFT.  Resize
+ phase-correlation all happen inside sentai_sim in C (sentai_pxp_shim_sim.c
+ sentai_fft_shim_sim.c + flow_phase_corr.cc).  This script just pipes
bytes around.

Wire protocol on the camera UDS (request + reply, half-duplex per frame):

  request (Python -> sentai_sim):
    uint32_t magic   = 0x53434D31  ('SCM1')
    uint32_t seq
    uint32_t width
    uint32_t height
    uint32_t pix_fmt = 0  (RGB888)
    uint32_t payload_bytes
    uint8_t  payload[payload_bytes]

  reply   (sentai_sim -> Python):
    uint32_t reply_magic = 0x46524C31  ('FRL1')
    uint32_t seq
    int32_t  dx_q1000
    int32_t  dy_q1000
    uint32_t conf
    uint64_t latency_us

Usage (full closed-loop):
    # 1. start sentai_sim                  (creates /tmp/sentai_cam.sock)
    # 2. start CrazySim cf2 + Gazebo with sentai_crazysim world
    # 3. start this bridge:
    python3 sim/scripts/gz_to_camera_bridge.py \\
            --topic /downward_cam/image --sock /tmp/sentai_cam.sock \\
            --send-flow --uri udp://127.0.0.1:19850

Usage (sentai_sim plumbing only, no cflib):
    python3 sim/scripts/gz_to_camera_bridge.py \\
            --topic /downward_cam/image --sock /tmp/sentai_cam.sock

Requirements (Gazebo Garden 7.9 — Harmonic is banned, see Sim.md):
  - gz.transport12 + gz.msgs9 Python bindings (Garden ABI).  On Ubuntu
    22.04 the apt packages are `python3-gz-transport12` /
    `python3-gz-msgs9`; on 24.04 install via pip or run this script
    inside the `crazysim-garden` distrobox where Garden is already
    available.
  - cflib >= 0.1.32  (`pip install --upgrade cflib` — earlier versions
    use the old UDP handshake that CrazySim doesn't speak).
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import threading
import time

MAGIC       = 0x53434D31  # 'SCM1'
REPLY_MAGIC = 0x46524C31  # 'FRL1'
HEADER_FMT  = "<IIIIII"
HEADER_LEN  = struct.calcsize(HEADER_FMT)
REPLY_FMT   = "<IIiiIQ"
REPLY_LEN   = struct.calcsize(REPLY_FMT)

# Drone EKF geometry constants — same numbers as
# examples/sentai_runtime/diag/_t_flow_to_drone.py DEFAULTS.
# Keeping them here lets the bridge stand alone (no MicroPython).
DEFAULTS = dict(
    fov_h_deg            = 58.0,
    fov_v_deg            = 45.0,
    grid_w               = 80,
    grid_h               = 60,
    drone_npix           = 35.0,
    drone_thetapix_rad   = 0.71674,
    drone_flow_resolution= 0.10,
    # cam0 + vflip=1 baseline (see flow_body_frame.md + Sim.md §10b).
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
    send_every_n         = 1,
)


def _scale_to_drone_units(cfg) -> tuple[float, float]:
    Np = cfg["drone_npix"]
    th = cfg["drone_thetapix_rad"]
    fr = cfg["drone_flow_resolution"]
    sx = (math.radians(cfg["fov_h_deg"]) * Np) / (cfg["grid_w"] * fr * th)
    sy = (math.radians(cfg["fov_v_deg"]) * Np) / (cfg["grid_h"] * fr * th)
    return (sx, sy)


def _conf_to_std(conf: int, cfg) -> float:
    if conf >= cfg["conf_thresh_high"]: return cfg["std_floor"]
    if conf >= cfg["conf_thresh_mid"]:  return cfg["std_low"]
    if conf >= cfg["conf_thresh_low"]:  return cfg["std_mid"]
    return cfg["std_ceil"]


def _to_body_dpx(dx_q: int, dy_q: int, cfg, scale_x: float, scale_y: float
                 ) -> tuple[float, float]:
    """Image-frame milli-grid-px -> drone-EKF dpixel units (PMW3901 eq.).
    Mirrors examples/sentai_runtime/diag/_t_flow_to_drone.py:_body_xy.
    """
    fw_dx, fw_dy, lf_dx, lf_dy = cfg["body_xform"]
    dx_grid = dx_q / 1000.0
    dy_grid = dy_q / 1000.0
    fw_grid   = fw_dx * dx_grid + fw_dy * dy_grid
    left_grid = lf_dx * dx_grid + lf_dy * dy_grid
    return (fw_grid * scale_x, left_grid * scale_y)


def open_uds(path: str, retries: int = 30, delay_s: float = 0.5) -> socket.socket:
    """Connect with retries — sentai_sim may still be booting."""
    last_err = None
    for _ in range(retries):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.connect(path)
            return s
        except (FileNotFoundError, ConnectionRefusedError) as e:
            last_err = e
            time.sleep(delay_s)
    raise SystemExit(f"could not connect to {path}: {last_err}")


def send_frame(sock: socket.socket, seq: int, w: int, h: int, rgb_bytes: bytes) -> bool:
    n = len(rgb_bytes)
    expected = w * h * 3
    if n != expected:
        # Drop and warn — sender side mismatch, never propagate corrupt frames.
        print(f"[gz_bridge] WARN dropping frame seq={seq}: "
              f"got {n} B, expected {expected} B", file=sys.stderr)
        return False
    hdr = struct.pack(HEADER_FMT, MAGIC, seq, w, h, 0, n)
    sock.sendall(hdr + rgb_bytes)
    return True


def _read_exact(sock: socket.socket, n: int) -> bytes | None:
    """Receive exactly n bytes; return None on EOF."""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def recv_flow_reply(sock: socket.socket) -> dict | None:
    raw = _read_exact(sock, REPLY_LEN)
    if raw is None: return None
    rmagic, seq, dx, dy, conf, lat = struct.unpack(REPLY_FMT, raw)
    if rmagic != REPLY_MAGIC:
        print(f"[gz_bridge] bad reply magic 0x{rmagic:08x}", file=sys.stderr)
        return None
    return dict(seq=seq, dx_q1000=dx, dy_q1000=dy, conf=conf, latency_us=lat)


class CFLibSink:
    """Optional cflib send-flow sink; lazily imported so the no-flow mode
    works even without cflib installed."""

    def __init__(self, uri: str):
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        cflib.crtp.init_drivers()
        self._sync = SyncCrazyflie(uri, cf=Crazyflie(rw_cache=None))
        self._sync.open_link()
        self._cf = self._sync.cf
        print(f"[cflib] linked to {uri}", file=sys.stderr)

    def send_flow(self, dpx: float, dpy: float, dt: float, std: float) -> None:
        # cf.extpos.send_extflow doesn't exist in stock cflib; the canonical
        # API to push optical flow into the EKF is via the Localization
        # pluggable packets (CRTP_PORT_LOCALIZATION).  cflib exposes:
        #   self._cf.loc.send_external_position(...) (extpos)
        # but for OPTICAL_FLOW the cf2 firmware accepts our SentAI deck
        # CRTP_FLOW packet (port=15 ch=1) directly.  We send raw bytes.
        try:
            from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
        except ImportError:
            return
        # 16-byte deck flow_pkt_t mirroring the firmware-side bridge.
        # Layout (little-endian): float dt, int16 dpx, int16 dpy,
        # float quality_std, uint16 conf, uint16 reserved.
        # See firmware src/deck/drivers/src/sentai_bridge.c.
        # Conf is informational; the EKF uses std for weighting.
        pk = CRTPPacket()
        pk.port = CRTPPort.LOCALIZATION
        pk.channel = 1
        pk.data = struct.pack("<fhhfHH",
                              float(dt),
                              int(round(dpx)),
                              int(round(dpy)),
                              float(std),
                              0,  # conf placeholder
                              0)  # reserved
        self._cf.send_packet(pk)

    def close(self):
        try:
            self._sync.close_link()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/downward_cam/image",
                    help="gz transport image topic")
    ap.add_argument("--sock", default="/tmp/sentai_cam.sock",
                    help="UDS path exposed by sentai_sim camera_bridge_recv")
    ap.add_argument("--fps-log", type=float, default=2.0,
                    help="how often (Hz) to print rolling FPS to stderr")
    ap.add_argument("--send-flow", action="store_true",
                    help="forward each flow snapshot to a cf2 over cflib")
    ap.add_argument("--uri", default="udp://127.0.0.1:19850",
                    help="cflib link URI for cf2 SITL")
    args = ap.parse_args()

    cfg = dict(DEFAULTS)
    scale_x, scale_y = _scale_to_drone_units(cfg)
    print(f"[gz_bridge] drone-scale: x={scale_x:.3f}  y={scale_y:.3f}  "
          f"(dpixel per grid-px)", file=sys.stderr)

    sink: CFLibSink | None = None
    if args.send_flow:
        try:
            sink = CFLibSink(args.uri)
        except Exception as e:
            print(f"[cflib] FAILED to link {args.uri}: {e}", file=sys.stderr)
            sink = None

    # Lazy import — we want a clean error message if the pkg isn't present.
    # Gazebo Garden 7.9 = transport12 / msgs9.  Harmonic (transport13/msgs10)
    # is BANNED for SentAI Phase 4 — see Sim.md "3 known-issue".
    try:
        from gz.transport12 import Node
        from gz.msgs9.image_pb2 import Image
    except ImportError as e:
        print(f"ERROR: missing Gazebo Garden python bindings: {e}", file=sys.stderr)
        print("  Inside the crazysim-garden distrobox:", file=sys.stderr)
        print("    sudo apt install python3-gz-transport12 python3-gz-msgs9",
              file=sys.stderr)
        print("  (Do NOT use Harmonic transport13/msgs10 — see Sim.md.)",
              file=sys.stderr)
        return 2

    sock = open_uds(args.sock)
    print(f"[gz_bridge] connected to {args.sock}", file=sys.stderr)

    # cb fires on the gz-transport callback thread; UDS access is serialized
    # by `lock` so we never interleave a frame send with a reply read.
    lock = threading.Lock()
    state = {
        "seq": 0,
        "frames_since_log": 0,
        "t_last_log": time.monotonic(),
        "t_last_flow": time.monotonic(),
        "log_interval_s": 1.0 / max(0.1, args.fps_log),
        "n_send": 0,        # successful flow injections
        "n_drop": 0,        # rejected by std/dt
    }

    def cb(msg: Image) -> None:
        # msg.data is bytes already in PIXEL_FORMAT_RGB_INT8 layout for our
        # world (R8G8B8 in the SDF).  We pass it straight through — zero copy
        # if the sendall buffer is small enough.
        with lock:
            state["seq"] += 1
            seq = state["seq"]
            ok = send_frame(sock, seq, msg.width, msg.height, msg.data)
            if not ok: return

            reply = recv_flow_reply(sock)
            if reply is None:
                print("[gz_bridge] sentai_sim closed connection", file=sys.stderr)
                return

            now = time.monotonic()
            dt  = now - state["t_last_flow"]
            state["t_last_flow"] = now
            dt = max(cfg["min_dt_s"], min(cfg["max_dt_s"], dt))

            if sink is not None and (seq % cfg["send_every_n"]) == 0:
                dpx, dpy = _to_body_dpx(reply["dx_q1000"], reply["dy_q1000"],
                                         cfg, scale_x, scale_y)
                std = _conf_to_std(reply["conf"], cfg)
                try:
                    sink.send_flow(dpx, dpy, dt, std)
                    state["n_send"] += 1
                except Exception as e:
                    state["n_drop"] += 1
                    if state["n_drop"] % 50 == 1:
                        print(f"[cflib] send_flow err: {e}", file=sys.stderr)

            state["frames_since_log"] += 1
            if now - state["t_last_log"] >= state["log_interval_s"]:
                fps = state["frames_since_log"] / (now - state["t_last_log"])
                print(f"[gz_bridge] {fps:5.1f} fps  seq={seq}  "
                      f"dx={reply['dx_q1000']:+5d} dy={reply['dy_q1000']:+5d} "
                      f"conf={reply['conf']:3d}  cflib_sent={state['n_send']}",
                      file=sys.stderr, flush=True)
                state["frames_since_log"] = 0
                state["t_last_log"] = now

    node = Node()
    if not node.subscribe(Image, args.topic, cb):
        print(f"[gz_bridge] failed to subscribe to {args.topic}", file=sys.stderr)
        return 3

    print(f"[gz_bridge] subscribed to {args.topic}; press Ctrl+C to stop",
          file=sys.stderr)
    try:
        # Spin forever; gz transport runs callbacks on its own thread.
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if sink is not None:
            sink.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

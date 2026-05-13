#!/usr/bin/env python3
"""flow_to_cf2.py — minimal flow forwarder.

Reads (dx, dy, conf, …) snapshots from /tmp/sentai_flow_out.sock
(produced by sim/gazebo/gz_to_uds_bridge.cc) and forwards them as
CRTP_LOCALIZATION ch=1 packets to a cf2 SITL via cflib.

This is the host-side half of the Phase 4 pipeline (Sim.md §10w.7):

   Gazebo /downward_cam/image                            (distrobox)
       ↓  gz_to_uds_bridge (C++, uses Garden gz-transport12)
   /tmp/sentai_cam.sock     →  sentai_sim camera_bridge   (host)
       ↓  6-pipe phase-corr (flow_phase_corr.cc)
   /tmp/sentai_flow_out.sock                              (host)
       ↓  THIS SCRIPT
   cflib cf.send_packet(CRTP_LOCALIZATION ch=1)
       ↓  UDP 19850
   cf2 firmware Kalman EKF observes optical flow → stable hover

Same packet semantics as s091/aruco_hover.py's flow_forwarder thread —
extracted into a standalone script so the orchestrator stays clean.

Usage:
   python3 sim/scripts/flow_to_cf2.py [--uri udp://127.0.0.1:19850] \
                                       [--sock /tmp/sentai_flow_out.sock]
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

# Wire format produced by gz_to_uds_bridge.cc → sentai_sim → bridge reply.
# Same struct layout s091 uses (124 bytes, FRL1 magic).
REPLY_FMT  = "<IIiiIQiIiiIiiIiiIIiiIIiiIIiiIB3x"
REPLY_SZ   = struct.calcsize(REPLY_FMT)
REPLY_MAGIC = 0x46524C31  # 'FRL1'

# Drone EKF geometry (matches DEFAULTS in gz_to_camera_bridge.py).
DRONE_NPIX            = 35.0
DRONE_THETAPIX_RAD    = 0.71674
DRONE_FLOW_RESOLUTION = 0.10
FOV_H_DEG             = 58.0
FOV_V_DEG             = 45.0
GRID_W                = 80
GRID_H                = 60


def _scale_to_drone_units() -> tuple[float, float]:
    import math
    # dpixel per grid-px conversion (drone EKF expects pixel delta over
    # the standard PMW3901 flow deck "NPIX" sensor).
    grid_per_rad_x = GRID_W / math.radians(FOV_H_DEG)
    grid_per_rad_y = GRID_H / math.radians(FOV_V_DEG)
    return (DRONE_NPIX * DRONE_THETAPIX_RAD / grid_per_rad_x,
            DRONE_NPIX * DRONE_THETAPIX_RAD / grid_per_rad_y)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", default="/tmp/sentai_flow_out.sock")
    ap.add_argument("--uri",  default="udp://127.0.0.1:19850")
    args = ap.parse_args()

    # Connect cflib
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
    cflib.crtp.init_drivers()
    sync = SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=None))
    sync.open_link()
    cf = sync.cf
    print(f"[flow] cflib linked {args.uri}", file=sys.stderr)

    # Connect UDS to bridge — retry up to 30s while bridge wires up.
    sock: socket.socket | None = None
    for i in range(60):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect(args.sock)
            sock = s
            break
        except Exception:
            time.sleep(0.5)
    if sock is None:
        print(f"[flow] FAIL connect {args.sock}", file=sys.stderr)
        return 2
    print(f"[flow] connected {args.sock}", file=sys.stderr)

    scale_x, scale_y = _scale_to_drone_units()
    last_send = time.monotonic()
    buf = b""
    n_sent = 0
    last_log = time.monotonic()

    while True:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                time.sleep(0.01)
                continue
            buf += chunk
            while len(buf) >= REPLY_SZ:
                rec, buf = buf[:REPLY_SZ], buf[REPLY_SZ:]
                fields = struct.unpack(REPLY_FMT, rec)
                magic = fields[0]
                if magic != REPLY_MAGIC:
                    # Re-sync on next FRL1 marker
                    idx = buf.find(struct.pack("<I", REPLY_MAGIC))
                    buf = buf[idx:] if idx >= 0 else b""
                    continue
                # Indices match REPLY_FMT order from s091:
                # 0:magic 1:seq 2:dx 3:dy 4:conf 5:latency 6:dz ...
                dx_q1000 = fields[2]
                dy_q1000 = fields[3]
                conf     = fields[4]
                # Convert q1000 → grid-pixel delta then to drone-pixel via scale.
                dx_grid = dx_q1000 / 1000.0
                dy_grid = dy_q1000 / 1000.0
                dpx = dx_grid * scale_x
                dpy = dy_grid * scale_y

                now = time.monotonic()
                dt  = max(0.001, min(0.2, now - last_send))
                last_send = now
                # std from confidence: higher conf → lower std
                std = max(1.0, 8.0 - conf / 32.0)

                pk = CRTPPacket()
                pk.port    = CRTPPort.LOCALIZATION
                pk.channel = 1
                pk.data    = struct.pack("<fhhfHH",
                                          float(dt),
                                          int(round(dpx)),
                                          int(round(dpy)),
                                          float(std),
                                          int(min(conf, 0xFFFF)),
                                          0)
                cf.send_packet(pk)
                n_sent += 1

                if now - last_log > 2.0:
                    print(f"[flow] sent={n_sent}  last conf={conf} dpx={dpx:+.1f} dpy={dpy:+.1f}",
                          file=sys.stderr)
                    last_log = now
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[flow] err {e}", file=sys.stderr)
            time.sleep(0.1)

    try:
        sync.close_link()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())

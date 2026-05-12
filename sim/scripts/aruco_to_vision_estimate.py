#!/usr/bin/env python3
"""aruco_to_vision_estimate.py — REAL ArUco PnP → MAVLink VISION_POSITION_ESTIMATE.

Acts as the receiver on /tmp/sentai_cam.sock (= same protocol as
sentai_sim's camera_bridge_recv).  For each frame from
gz_to_uds_bridge it:
  1. Detects 4×4_50 ArUco markers via cv2.aruco
  2. Runs solvePnP per marker
  3. Computes drone world pose via estimate_drone_world_pose
  4. Sends MAVLink VISION_POSITION_ESTIMATE (msg 102) to PX4
  5. Returns a 0-flow reply to the bridge (so it stays connected)

Closes s103 / B-path with REAL CV instead of gz-pose mock.  Provides
EKF an absolute position anchor → no flow-sign-tuning needed.

Run inside /home/bogdan/work/coralmicro/venv (has cv2 + pymavlink).

Coordinate frame:
  estimate_drone_world_pose returns (x, y, z) in WORLD ENU (gz frame).
  VISION_POSITION_ESTIMATE expects NED.  Convert: x_ned=y_enu,
  y_ned=x_enu, z_ned=-z_enu.
"""
from __future__ import annotations
import argparse
import math
import socket
import struct
import sys
import time

import numpy as np
import cv2
from pymavlink import mavutil

sys.path.insert(0, '/home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/s090_hover_over_cat')
from aruco_detector import (detect_markers, estimate_drone_world_pose,
                              KNOWN_POSITIONS_M)

# New marker positions (doubled 2026-05-12 for x500 hover bench, see
# Sim.md §10n update).  Override the import-time defaults.
KNOWN_POSITIONS_M_X500 = {
    0: (+0.30, +0.20, 0.20),
    1: (-0.30, +0.20, 0.20),
    2: (-0.30, -0.20, 0.20),
    3: (+0.30, -0.20, 0.20),
}

MAGIC       = 0x53434D31  # 'SCM1' — request from gz_to_uds_bridge
REPLY_MAGIC = 0x46524C31  # 'FRL1' — reply we send back
HEADER_FMT  = "<IIIIII"
HEADER_LEN  = struct.calcsize(HEADER_FMT)
# Full Reply struct per gz_to_uds_bridge.cc (124 bytes, packed):
#   reply_magic, seq, dx, dy, conf, latency_us,
#   dz, dz_conf,
#   dx_center, dy_center, conf_center,
#   dx_fine, dy_fine, conf_fine,
#   dx_anchor, dy_anchor, conf_anchor, frames_since_anchor,
#   dx_anchor_L2, dy_anchor_L2, conf_anchor_L2, frames_since_anchor_L2,
#   dx_anchor_L1, dy_anchor_L1, conf_anchor_L1, frames_since_anchor_L1,
#   dx_best, dy_best, conf_best,
#   best_source (u8) + pad[3]
REPLY_FMT   = "<IIiiIQiI iiI iiI iiII iiII iiII iiI B3s"
REPLY_LEN   = struct.calcsize(REPLY_FMT)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sock", default="/tmp/sentai_cam.sock")
    ap.add_argument("--mav", default="udpout:127.0.0.1:14580",
                    help="PX4 Onboard listen port")
    ap.add_argument("--rate-log", type=float, default=1.0,
                    help="Hz of stderr status logging")
    args = ap.parse_args()

    # Open mavlink.
    print(f"[aruco-vpe] mavlink {args.mav}", file=sys.stderr)
    m = mavutil.mavlink_connection(args.mav, source_system=210,
                                    source_component=200)
    m.target_system = 1
    m.target_component = 1

    # Open UDS in server mode (gz_to_uds_bridge will connect to us).
    try:
        import os
        if os.path.exists(args.sock):
            os.unlink(args.sock)
    except OSError:
        pass
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(args.sock)
    srv.listen(1)
    print(f"[aruco-vpe] listening on {args.sock}", file=sys.stderr)

    conn, _ = srv.accept()
    print("[aruco-vpe] bridge connected", file=sys.stderr)

    n_frames = 0
    n_detect = 0
    n_pose = 0
    n_vpe = 0
    last_log = time.monotonic()
    t0 = time.monotonic()

    def recv_exact(sock, n):
        buf = b""
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    while True:
        hdr = recv_exact(conn, HEADER_LEN)
        if hdr is None:
            print("[aruco-vpe] bridge disconnected", file=sys.stderr)
            break
        magic, seq, w, h, _, payload_bytes = struct.unpack(HEADER_FMT, hdr)
        if magic != MAGIC:
            print(f"[aruco-vpe] bad magic 0x{magic:x}", file=sys.stderr)
            break
        rgb_bytes = recv_exact(conn, payload_bytes)
        if rgb_bytes is None:
            break
        n_frames += 1

        rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape(h, w, 3)
        dets = detect_markers(rgb, estimate_pose=True)
        if dets:
            n_detect += 1
            pose = estimate_drone_world_pose(
                dets, known_positions=KNOWN_POSITIONS_M_X500,
                drone_yaw=0.0)            # TODO: read from MAVLink ATTITUDE
            if pose is not None:
                n_pose += 1
                x_enu, y_enu, z_enu = pose
                # ENU → NED
                x_ned = y_enu
                y_ned = x_enu
                z_ned = -z_enu
                usec = int((time.monotonic() - t0) * 1e6)
                cov = [float('nan')] + [0.0] * 20
                # NaN orientation → PX4 uses VPE for position only,
                # keeps yaw from mag (avoids yaw-conflict crashes).
                nan = float('nan')
                m.mav.vision_position_estimate_send(
                    usec, x_ned, y_ned, z_ned, nan, nan, nan, cov)
                n_vpe += 1

        # Always reply with zero-flow (bridge protocol).  Full 124-byte
        # reply struct — all fields zero except magic+seq+best_source.
        rep = struct.pack(REPLY_FMT,
            REPLY_MAGIC, seq,
            0, 0, 0, 0,             # dx, dy, conf, latency_us
            0, 0,                   # dz, dz_conf
            0, 0, 0,                # L1 center
            0, 0, 0,                # L2 fine
            0, 0, 0, 0,             # L0 anchor
            0, 0, 0, 0,             # L2 anchor
            0, 0, 0, 0,             # L1 anchor
            0, 0, 0,                # best
            0, b'\x00\x00\x00')     # best_source + pad
        conn.sendall(rep)

        now = time.monotonic()
        if now - last_log >= 1.0 / args.rate_log:
            print(f"[aruco-vpe] frames={n_frames} detect={n_detect} "
                  f"pose={n_pose} vpe={n_vpe}", file=sys.stderr, flush=True)
            last_log = now

    conn.close(); srv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

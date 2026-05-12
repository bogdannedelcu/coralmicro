#!/usr/bin/env python3
"""gz_pose_to_vision_estimate.py — read x500_sentai_0 ground-truth pose
from `gz topic -e -t /world/.../dynamic_pose/info` (stdin) and forward
to PX4 as MAVLink VISION_POSITION_ESTIMATE (msg id 102) at ~30 Hz.

Used by s102 to validate that PX4 EKF2 with EKF2_EV_CTRL accepts
external vision pose, so the drone can arm + AUTO.TAKEOFF without GPS.
A perfect "ArUco PnP" mock — real ArUco is s103.

Coordinate frame:
  gz dynamic_pose/info: world ENU (X-east, Y-north, Z-up)
  PX4 VISION_POSITION_ESTIMATE: local NED (X-north, Y-east, Z-down)
  ENU→NED transform: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu

Usage:
  gz topic -e -t /world/sentai_crazysim/dynamic_pose/info 2>/dev/null \\
    | python3 gz_pose_to_vision_estimate.py \\
        --model x500_sentai_0 \\
        --mav udpout:127.0.0.1:18570
"""
from __future__ import annotations
import argparse
import math
import re
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pip install pymavlink", file=sys.stderr); sys.exit(2)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--mav", default="udpout:127.0.0.1:18570",
                    help="pymavlink connection string for PX4")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="max VISION_POSITION_ESTIMATE Hz")
    ap.add_argument("--log", type=str, default=None,
                    help="optional CSV log path")
    args = ap.parse_args()

    print(f"[vision] connecting to {args.mav} ...", file=sys.stderr)
    m = mavutil.mavlink_connection(args.mav, source_system=255)
    m.wait_heartbeat(timeout=10)
    print(f"[vision] PX4 sys={m.target_system} comp={m.target_component}",
          file=sys.stderr)

    log_f = None
    if args.log:
        log_f = open(args.log, "w")
        log_f.write("t_s,x_enu,y_enu,z_enu,qx,qy,qz,qw,yaw_rad\n")

    pat_name = re.compile(r'^\s*name:\s*"([^"]+)"')
    pat_x = re.compile(r'^\s*x:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')
    pat_y = re.compile(r'^\s*y:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')
    pat_z = re.compile(r'^\s*z:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')
    pat_w = re.compile(r'^\s*w:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')

    cur_name: str | None = None
    in_pose = False
    in_pos = False
    in_orient = False
    px = py = pz = None
    qx = qy = qz = qw = None

    t0 = time.monotonic()
    last_tx = 0.0
    min_dt = 1.0 / max(args.rate, 1.0)
    n_tx = 0

    for line in sys.stdin:
        s = line.rstrip("\n")
        if s.startswith("pose {"):
            in_pose = True; in_pos = False; in_orient = False
            cur_name = None
            px = py = pz = None
            qx = qy = qz = qw = None
            continue
        if not in_pose:
            continue
        if (mn := pat_name.match(s)) and cur_name is None:
            cur_name = mn.group(1)
            continue
        ls = s.lstrip()
        if ls.startswith("position {"):
            in_pos = True; in_orient = False; continue
        if ls.startswith("orientation {"):
            in_pos = False; in_orient = True; continue

        if in_pos:
            if (mx := pat_x.match(s)): px = float(mx.group(1))
            elif (my := pat_y.match(s)): py = float(my.group(1))
            elif (mz := pat_z.match(s)): pz = float(mz.group(1))
        elif in_orient:
            if (mx := pat_x.match(s)): qx = float(mx.group(1))
            elif (my := pat_y.match(s)): qy = float(my.group(1))
            elif (mz := pat_z.match(s)): qz = float(mz.group(1))
            elif (mw := pat_w.match(s)): qw = float(mw.group(1))

        if s == "}" or s.startswith("}"):
            if cur_name == args.model and None not in (px, py, pz, qx, qy, qz, qw):
                now = time.monotonic()
                if now - last_tx >= min_dt:
                    last_tx = now
                    # ENU (gz) → NED (PX4): swap x↔y, negate z.
                    x_ned = py
                    y_ned = px
                    z_ned = -pz
                    # Yaw from quaternion in ENU.  PX4 wants NED yaw.
                    # yaw_enu = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
                    yaw_enu = math.atan2(2.0*(qw*qz + qx*qy),
                                          1.0 - 2.0*(qy*qy + qz*qz))
                    # ENU→NED yaw: yaw_ned = pi/2 - yaw_enu (rotated frame).
                    yaw_ned = (math.pi / 2.0) - yaw_enu

                    usec = int((time.monotonic() - t0) * 1e6)
                    # Per MAVLink spec, covariance[0]=NaN means "unknown".
                    # If we send all zeros, PX4 EKF interprets as
                    # zero-variance ("infinitely confident") → unstable.
                    cov = [float('nan')] + [0.0] * 20
                    m.mav.vision_position_estimate_send(
                        usec, x_ned, y_ned, z_ned, 0.0, 0.0, yaw_ned, cov)
                    n_tx += 1
                    if n_tx % 30 == 1:
                        print(f"[vision] tx={n_tx}  ned=({x_ned:+.3f}, "
                              f"{y_ned:+.3f}, {z_ned:+.3f}) yaw={yaw_ned:+.2f}",
                              file=sys.stderr, flush=True)
                    if log_f:
                        log_f.write(f"{usec/1e6:.4f},{px:.5f},{py:.5f},{pz:.5f},"
                                    f"{qx:.5f},{qy:.5f},{qz:.5f},{qw:.5f},{yaw_ned:.5f}\n")
                        log_f.flush()
            in_pose = False; in_pos = False; in_orient = False

    if log_f:
        log_f.close()
    print(f"[vision] stop: {n_tx} VISION_POSITION_ESTIMATE sent", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

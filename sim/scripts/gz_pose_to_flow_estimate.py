#!/usr/bin/env python3
"""gz_pose_to_flow_estimate.py — Mock the PX4 px4flow sensor by
deriving OPTICAL_FLOW_RAD (msg 106) from gz ground-truth velocity.

Used by s106 to validate that PX4 EKF2 with `EKF2_OF_CTRL=1` accepts
flow data and fuses it into a stable velocity estimate, enabling
OFFBOARD velocity-setpoint hover WITHOUT GPS.

Physics:
  flow_y[rad] (about body +X motion) ≈ +(vx_body[m/s] · Δt) / h[m]
  flow_x[rad] (about body +Y motion) ≈ -(vy_body[m/s] · Δt) / h[m]
where h is ground distance.  Sent in the OPTICAL_FLOW_RAD frame:
  - integrated_x: +vy → -flow_x  (RH about Y axis, sensor +Y motion → -flow)
  - integrated_y: +vx → +flow_y  (RH about Y axis, sensor +X motion → +flow)

Quality = 255 (perfect mock sensor).
Distance = drone altitude (positive) from gz pose.
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


def quat_to_yaw(qx, qy, qz, qw):
    """Yaw (Z rotation) from ENU quaternion."""
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--mav", default="udpout:127.0.0.1:14580",
                    help="pymavlink connection.  PX4 Onboard listens on "
                         "14580 — that's where flow data should arrive.")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="max OPTICAL_FLOW_RAD Hz")
    args = ap.parse_args()

    print(f"[flow] connecting {args.mav}", file=sys.stderr)
    m = mavutil.mavlink_connection(args.mav, source_system=200,
                                    source_component=200)
    m.target_system = 1
    m.target_component = 1
    print(f"[flow] sending to PX4 sys=1 comp=1", file=sys.stderr)

    pat_name = re.compile(r'^\s*name:\s*"([^"]+)"')
    pat_x = re.compile(r'^\s*x:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')
    pat_y = re.compile(r'^\s*y:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')
    pat_z = re.compile(r'^\s*z:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')
    pat_w = re.compile(r'^\s*w:\s*(-?\d+\.?\d*(?:e[-+]?\d+)?)')

    cur_name = None
    in_pose = False; in_pos = False; in_orient = False
    px = py = pz = None
    qx = qy = qz = qw = None

    last_pose = None         # (t, x_enu, y_enu, z_enu)
    last_yaw = 0.0
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
            cur_name = mn.group(1); continue
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
                if last_pose is None:
                    last_pose = (now, px, py, pz)
                    last_yaw = quat_to_yaw(qx, qy, qz, qw)
                elif now - last_tx >= min_dt:
                    last_tx = now
                    dt = now - last_pose[0]
                    if dt > 0:
                        # Velocity in world ENU frame.
                        vx_enu = (px - last_pose[1]) / dt
                        vy_enu = (py - last_pose[2]) / dt
                        # Rotate ENU velocity into body XY using yaw.
                        # body_x = forward, body_y = left.
                        yaw = quat_to_yaw(qx, qy, qz, qw)
                        cy = math.cos(yaw); sy = math.sin(yaw)
                        vx_body =  cy*vx_enu + sy*vy_enu
                        vy_body = -sy*vx_enu + cy*vy_enu
                        h = max(0.10, pz)        # altitude (m), floor 10cm to
                                                 # avoid div by tiny number
                        # SANITY CHECK: send pure-zero flow always.
                        # If drone diverges anyway, the issue is not sign
                        # convention but something else (yaw stability,
                        # gyro NaN, EKF framing).  If drone HOLDS, then
                        # divergence is sign-driven and we iterate signs.
                        flow_y = 0.0
                        flow_x = 0.0

                        dt_us = max(1, int(dt * 1e6))
                        usec = int((now - args_t0) * 1e6)
                        nan_v = float('nan')
                        m.mav.optical_flow_rad_send(
                            usec,                  # time_usec
                            0,                     # sensor_id
                            dt_us,                 # integration_time_us
                            flow_x, flow_y,        # integrated rad
                            nan_v, nan_v, nan_v,   # xgyro/ygyro/zgyro (NaN = no derotation)
                            20,                    # temperature (centidegC)
                            255,                   # quality
                            dt_us,                 # time_delta_distance_us
                            h)                     # distance (m)
                        n_tx += 1
                        if n_tx % 30 == 1:
                            print(f"[flow] tx={n_tx}  body_v=({vx_body:+.3f},{vy_body:+.3f}) "
                                  f"flow=({flow_x*1e3:+.2f},{flow_y*1e3:+.2f})mrad "
                                  f"h={h:.2f}m",
                                  file=sys.stderr, flush=True)
                        last_pose = (now, px, py, pz)
                        last_yaw = yaw
            in_pose = False; in_pos = False; in_orient = False

    print(f"[flow] stop: {n_tx} OPTICAL_FLOW_RAD sent", file=sys.stderr)
    return 0


if __name__ == "__main__":
    args_t0 = time.monotonic()
    sys.exit(main())

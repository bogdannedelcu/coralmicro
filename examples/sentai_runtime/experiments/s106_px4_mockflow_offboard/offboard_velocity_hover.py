#!/usr/bin/env python3
"""s106 — PX4 OFFBOARD hover via VELOCITY setpoint (vx=vy=0, vz from
altitude controller).  Pairs with the mock OPTICAL_FLOW_RAD bridge
(`gz_pose_to_flow_estimate.py`).  No GPS — flow + baro only.

Velocity setpoint is the right choice for flow-only: flow gives EKF
*velocity* not position; commanding (vx=0, vy=0) tells PX4 "hold
zero velocity", and EKF velocity-from-flow closes the loop.  Position
will still drift slowly (no anchor) but velocity is stable — same
as cf2+PMW3901 indoor hover.
"""
from __future__ import annotations
import argparse
import threading
import time
from pymavlink import mavutil


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

# SET_POSITION_TARGET_LOCAL_NED type_mask.
# bit set = field IGNORED.  Standard PX4 OFFBOARD velocity setup uses
# velocity (vx, vy, vz) + yaw_rate (kept active at 0 = hold heading).
# Pure velocity-only (with yaw_rate also masked) confuses PX4 attitude
# controller → minimum thrust → no takeoff.
TYPE_MASK_VELOCITY_YAWRATE = (
    (1 << 0) | (1 << 1) | (1 << 2) |     # px, py, pz ignored
    (1 << 6) | (1 << 7) | (1 << 8) |     # ax, ay, az ignored
    (1 << 9) | (1 << 10)                 # force, yaw ignored (NOT yaw_rate)
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav", default="udp:127.0.0.1:14550",
                    help="PX4 Normal stream sends to 14550 by convention. "
                         "Use a different stream than flow bridge (14580) "
                         "so partnership doesn't conflict.")
    ap.add_argument("--takeoff-vz", type=float, default=-0.5,
                    help="climb velocity m/s NED (negative = up)")
    ap.add_argument("--takeoff-s", type=float, default=3.0)
    ap.add_argument("--hover-s", type=float, default=15.0)
    ap.add_argument("--land-vz", type=float, default=+0.3,
                    help="descend velocity m/s NED")
    ap.add_argument("--land-s", type=float, default=8.0)
    ap.add_argument("--rate", type=float, default=20.0)
    args = ap.parse_args()

    print(f"[offboard-v] connecting {args.mav}", flush=True)
    m = mavutil.mavlink_connection(args.mav, source_system=2,
                                    source_component=190)
    m.target_system = 1
    m.target_component = 1
    hb = m.wait_heartbeat(timeout=5)
    if hb is not None:
        print(f"[offboard-v] HB sys={m.target_system}", flush=True)
    else:
        print("[offboard-v] WARN no HB, defaulting sys=1", flush=True)

    # NED velocity setpoint (held in dict so streamer + mission thread share).
    setpoint = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
    stop = threading.Event()
    n_sp = [0]
    t0 = time.monotonic()

    def streamer():
        period = 1.0 / args.rate
        while not stop.is_set():
            s = setpoint
            m.mav.set_position_target_local_ned_send(
                int((time.monotonic() - t0) * 1e3),
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                TYPE_MASK_VELOCITY_YAWRATE,
                0, 0, 0,
                s["vx"], s["vy"], s["vz"],
                0, 0, 0,
                0, 0)             # yaw (ignored), yaw_rate=0 (hold heading)
            n_sp[0] += 1
            time.sleep(period)

    th = threading.Thread(target=streamer, daemon=True)
    th.start()
    print(f"[offboard-v] streaming velocity setpoints @ {args.rate:.0f} Hz",
          flush=True)
    time.sleep(2.0)

    # Switch to OFFBOARD.
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, 0, 0, 0, 0)
    for _ in range(20):
        msg = m.recv_match(type='COMMAND_ACK', blocking=False, timeout=0.1)
        if msg and msg.command == 176:
            r = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            print(f"[offboard-v] DO_SET_MODE ACK: {r}", flush=True); break
    time.sleep(0.5)

    # Arm.
    print("[offboard-v] arming...", flush=True)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 21196, 0, 0, 0, 0, 0)
    for _ in range(20):
        msg = m.recv_match(type='COMMAND_ACK', blocking=False, timeout=0.1)
        if msg and msg.command == 400:
            r = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            print(f"[offboard-v] ARM ACK: {r}", flush=True); break

    # Climb.
    setpoint["vz"] = args.takeoff_vz
    print(f"[offboard-v] climb vz={args.takeoff_vz} for {args.takeoff_s}s",
          flush=True)
    t_climb0 = time.monotonic()
    while time.monotonic() - t_climb0 < args.takeoff_s:
        msg = m.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.3)
        if msg:
            print(f"  [{time.monotonic()-t_climb0:5.1f}s] z={msg.z:+.2f} vz={msg.vz:+.2f}",
                  flush=True)

    # Hover (vx=vy=vz=0).
    setpoint["vz"] = 0.0
    print(f"[offboard-v] hover {args.hover_s:.1f}s at zero velocity", flush=True)
    t_h0 = time.monotonic()
    while time.monotonic() - t_h0 < args.hover_s:
        msg = m.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.5)
        if msg:
            print(f"  [{time.monotonic()-t_h0:5.1f}s] x={msg.x:+.2f} y={msg.y:+.2f} "
                  f"z={msg.z:+.2f}  v=({msg.vx:+.2f},{msg.vy:+.2f},{msg.vz:+.2f})",
                  flush=True)

    # Descend.
    setpoint["vz"] = args.land_vz
    print(f"[offboard-v] descend vz={args.land_vz} for {args.land_s}s", flush=True)
    time.sleep(args.land_s)

    # Disarm.
    setpoint["vz"] = 0.0
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 21196, 0, 0, 0, 0, 0)
    time.sleep(0.5)

    stop.set(); th.join(timeout=1)
    print(f"[offboard-v] done. setpoints={n_sp[0]}", flush=True)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())

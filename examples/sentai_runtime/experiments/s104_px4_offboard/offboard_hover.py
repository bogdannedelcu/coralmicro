#!/usr/bin/env python3
"""s104 — PX4 OFFBOARD hover via SET_POSITION_TARGET_LOCAL_NED.

Path A of Phase 6d: bypass PX4 AUTO.TAKEOFF arm-gate by using OFFBOARD
mode + continuous position setpoints, same way cf2 cflib does it with
send_position_setpoint().

Runs in distrobox (where pymavlink lives).  Talks to PX4 on UDP
14550 (Normal GCS port) — does NOT conflict with sentai_sim on 14540
or PX4 onboard 14580.

Flow:
  1. Wait heartbeat from PX4
  2. Stream SET_POSITION_TARGET_LOCAL_NED at 20 Hz (background thread)
  3. After ~2 s of stream, MAV_CMD_DO_SET_MODE → PX4_CUSTOM_MAIN_MODE_OFFBOARD
  4. Arm via MAV_CMD_COMPONENT_ARM_DISARM
  5. Hold (0, 0, -TARGET_Z) for HOVER_S seconds
  6. Descend setpoint (0, 0, -0.1) for LAND_S seconds (let drone touch)
  7. Disarm

Output:
  - stdout: log lines
  - --log CSV: t_s, setpoint x/y/z, current x/y/z, ack hits
"""
from __future__ import annotations
import argparse
import math
import threading
import time
from pymavlink import mavutil


# PX4 custom main_mode values.
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
# MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED type_mask bits.
# bit set = field IGNORED.  We want position-only → ignore velocity,
# accel, yaw, yaw_rate.
TYPE_MASK_POSITION_ONLY = (
    (1 << 3) | (1 << 4) | (1 << 5) |        # vx, vy, vz ignored
    (1 << 6) | (1 << 7) | (1 << 8) |        # ax, ay, az ignored
    (1 << 9) | (1 << 10) | (1 << 11) |      # force(unused), yaw, yaw_rate
    0
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav", default="udpin:127.0.0.1:14540",
                    help="pymavlink connection.  PX4 SITL Onboard "
                         "stream sends to 14540 by convention; bind there "
                         "with udpin so PX4's outbound HB lands on us "
                         "and pymavlink learns its source addr for "
                         "command replies.  14550 is the GCS port "
                         "(QGroundControl) — avoid; vision_bridge on "
                         "18570 may already claim partnership.")
    ap.add_argument("--target-z", type=float, default=1.5,
                    help="hover altitude (m, AGL positive up)")
    ap.add_argument("--hover-s", type=float, default=15.0)
    ap.add_argument("--land-s", type=float, default=8.0)
    ap.add_argument("--rate", type=float, default=20.0,
                    help="setpoint stream Hz")
    args = ap.parse_args()

    print(f"[offboard] connecting {args.mav}", flush=True)
    m = mavutil.mavlink_connection(args.mav, source_system=2,
                                    source_component=190)
    # wait_heartbeat returns immediately on first HB.  Force a non-zero
    # initial sys=1 in case HB doesn't arrive — PX4 default sysid is 1
    # so commands target the right vehicle even if our HB-listen times
    # out (e.g. when GCS port 14550 is busy with another listener).
    m.target_system = 1
    m.target_component = 1
    hb = m.wait_heartbeat(timeout=5)
    if hb is not None:
        print(f"[offboard] HB sys={m.target_system} comp={m.target_component} "
              f"(type={hb.type} autopilot={hb.autopilot})", flush=True)
    else:
        print("[offboard] WARN: no HB received in 5s, defaulting "
              "target_sys=1 (PX4 SITL default)", flush=True)

    # Subscribe to position telemetry so we can log/observe.
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1)
    m.mav.request_data_stream_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1)

    # ---- Setpoint streamer thread (must run BEFORE mode switch and
    #      throughout flight, else PX4 falls out of OFFBOARD).
    setpoint = {"x": 0.0, "y": 0.0, "z": -args.target_z}   # NED: z negative = up
    stop = threading.Event()
    n_sp = [0]

    def streamer():
        period = 1.0 / args.rate
        while not stop.is_set():
            t = setpoint
            m.mav.set_position_target_local_ned_send(
                int((time.monotonic() - t0) * 1e3),   # time_boot_ms
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                TYPE_MASK_POSITION_ONLY,
                t["x"], t["y"], t["z"],     # position NED
                0, 0, 0,                    # velocity (ignored)
                0, 0, 0,                    # accel (ignored)
                0, 0)                       # yaw, yaw_rate (ignored)
            n_sp[0] += 1
            time.sleep(period)

    t0 = time.monotonic()
    th = threading.Thread(target=streamer, daemon=True)
    th.start()
    print(f"[offboard] streaming setpoints @ {args.rate:.0f} Hz to "
          f"({setpoint['x']:.1f}, {setpoint['y']:.1f}, {setpoint['z']:.1f}) NED",
          flush=True)

    # Wait so PX4 has setpoints to feed OFFBOARD when we switch mode.
    time.sleep(2.0)
    print(f"[offboard] {n_sp[0]} setpoints sent, switching mode...",
          flush=True)

    # Switch to OFFBOARD via DO_SET_MODE.
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0,
        0, 0, 0, 0)

    # Read mode-switch ACK
    for _ in range(20):
        msg = m.recv_match(type='COMMAND_ACK', blocking=False, timeout=0.1)
        if msg and msg.command == 176:
            r = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            print(f"[offboard] DO_SET_MODE ACK: {r}", flush=True)
            break
    time.sleep(0.5)

    # Arm.
    print("[offboard] arming...", flush=True)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 21196, 0, 0, 0, 0, 0)
    for _ in range(20):
        msg = m.recv_match(type='COMMAND_ACK', blocking=False, timeout=0.1)
        if msg and msg.command == 400:
            r = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            print(f"[offboard] ARM ACK: {r}", flush=True)
            break

    # Hover: just sit at the setpoint for HOVER_S.
    print(f"[offboard] hovering {args.hover_s:.1f}s at z={-setpoint['z']:.2f}m...",
          flush=True)
    t_hover0 = time.monotonic()
    while time.monotonic() - t_hover0 < args.hover_s:
        msg = m.recv_match(type='LOCAL_POSITION_NED', blocking=False,
                           timeout=0.5)
        if msg:
            elapsed = time.monotonic() - t_hover0
            print(f"  [{elapsed:5.1f}s] x={msg.x:+.2f} y={msg.y:+.2f} "
                  f"z={msg.z:+.2f}  vz={msg.vz:+.2f}",
                  flush=True)

    # Descend.
    print("[offboard] descending to z=0.2m (landing)...", flush=True)
    setpoint["z"] = -0.2
    time.sleep(args.land_s)

    # Disarm.
    print("[offboard] disarming...", flush=True)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 21196, 0, 0, 0, 0, 0)
    time.sleep(0.5)

    stop.set()
    th.join(timeout=1)
    print(f"[offboard] done.  setpoints sent: {n_sp[0]}", flush=True)
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())

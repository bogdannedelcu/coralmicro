#!/usr/bin/env python3
"""takeoff_with_mocap.py — STABLE cf2 SITL takeoff using GT pose feedback.

The cf2 SITL flight is inherently unstable in 5+ second hovers because
the Kalman EKF has no horizontal position observations — drift
accumulates, attitude exceeds limits, drone flips.

This script provides position observations from Gazebo's ground truth
(`/world/<world>/pose/info`) and forwards them to cf2 via cflib's
`cf.extpos.send_extpos(x, y, z)` — same mechanism used with motion
capture systems (Vicon, OptiTrack).  cf2 EKF locks to extpos → no drift.

Same architecture sentai.flow ultimately provides (flow gives
delta-position observations, extpos gives absolute) — extpos is just
the simpler one for a basic stability test, no algorithm needed.

Usage:
   python3 takeoff_with_mocap.py [--world s125_simple] [--target-z 2.0]

Run only after SITL is up.  Uses two threads:
  - mocap thread: subscribes to gz pose topic via a subprocess pipe,
    parses each "name: crazyflie_0" → position {x,y,z} entry, and
    immediately forwards to cf2 via cf.extpos.send_extpos()
  - main thread: runs MotionCommander takeoff/hover/land

Note: the mocap thread runs `gz topic -e -t /world/<world>/pose/info`
inside distrobox (Garden gz CLI) and parses text-protobuf.
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander


URI = "udp://127.0.0.1:19850"


def mocap_thread(stop_evt: threading.Event, world: str, cf):
    """Stream cf2 GT pose from gz topic + push into cf2 EKF via extpos."""
    cmd = [
        "distrobox", "enter", "crazysim-garden", "--",
        "gz", "topic", "-e", "-t", f"/world/{world}/pose/info",
    ]
    print("[mocap] starting gz topic subscriber", flush=True)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True, bufsize=1)
    try:
        # State machine: look for `name: "crazyflie_0"`, then capture the
        # next `position { x:N y:N z:N }` block.
        in_cf  = False
        in_pos = False
        x = y = z = None
        sent = 0
        last_log = time.monotonic()
        rgx_name = re.compile(r'name:\s*"crazyflie_0"\s*$')
        rgx_pos  = re.compile(r'^\s*position\s*{')
        rgx_end  = re.compile(r'^\s*}\s*$')
        rgx_field = re.compile(r'^\s*([xyz]):\s*([-+0-9.eE]+)')
        for line in p.stdout:
            if stop_evt.is_set():
                break
            if rgx_name.search(line):
                in_cf = True
                x = y = z = None
                continue
            if not in_cf:
                continue
            if rgx_pos.match(line):
                in_pos = True
                continue
            if in_pos:
                if rgx_end.match(line):
                    in_pos = False
                    in_cf = False
                    if x is not None and y is not None and z is not None:
                        try:
                            cf.extpos.send_extpos(x, y, z)
                            sent += 1
                            now = time.monotonic()
                            if now - last_log > 1.0:
                                print(f"[mocap] extpos sent={sent}  last=({x:+.2f},{y:+.2f},{z:+.2f})",
                                      flush=True)
                                last_log = now
                        except Exception as e:
                            print(f"[mocap] send_extpos err: {e}", flush=True)
                    continue
                m = rgx_field.match(line)
                if m:
                    axis, val = m.group(1), float(m.group(2))
                    if axis == "x": x = val
                    elif axis == "y": y = val
                    elif axis == "z": z = val
    finally:
        try: p.kill()
        except Exception: pass
        print(f"[mocap] stopped ({sent if 'sent' in dir() else 0} packets sent)", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world",  default="s125_simple")
    ap.add_argument("--target-z", type=float, default=2.0)
    ap.add_argument("--hover-s",  type=float, default=8.0)
    args = ap.parse_args()

    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache=None)
    print(f"[t] connecting → {URI}")
    stop_evt = threading.Event()

    with SyncCrazyflie(URI, cf=cf) as scf:
        # Kalman estimator + reset cycle — same as proven s091 setup.
        scf.cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(0.3)
        scf.cf.param.set_value("kalman.resetEstimation", 1)
        time.sleep(0.5)
        scf.cf.param.set_value("kalman.resetEstimation", 0)
        time.sleep(0.5)

        # Start mocap BEFORE takeoff so EKF has position observations
        # ready when motors spin.
        th = threading.Thread(target=mocap_thread,
                              args=(stop_evt, args.world, scf.cf),
                              daemon=True)
        th.start()
        time.sleep(1.0)  # let mocap establish + EKF lock

        print(f"[t] takeoff → {args.target_z} m via MotionCommander")
        with MotionCommander(scf, default_height=args.target_z) as mc:
            print(f"[t] hover {args.hover_s}s — operator can verify "
                  f"ArUco markers visible in PIP")
            time.sleep(args.hover_s)
            print("[t] landing")
        print("[t] done")
    stop_evt.set()
    th.join(timeout=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())

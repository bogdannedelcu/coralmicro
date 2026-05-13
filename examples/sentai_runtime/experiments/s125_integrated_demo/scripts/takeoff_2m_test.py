#!/usr/bin/env python3
"""takeoff_2m_test.py — MINIMAL takeoff isolation test.

Connects to cf2 SITL via cflib + uses MotionCommander to:
  1. takeoff to 2.0 m
  2. hold 5 s
  3. land

No orchestrator, no sentai_sim, no SFLVP, no saver, no pose logger.
Just cflib + cf2 SITL + MotionCommander.

If the drone flies calmly here but chaotically in run_demo.sh, the
delta is the aggressor.
"""
import time
import sys
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander

URI = "udp://127.0.0.1:19850"
TARGET_Z = 2.0
HOVER_S  = 5.0


def main():
    cflib.crtp.init_drivers()
    print(f"[t] connecting cflib → {URI}")
    cf = Crazyflie(rw_cache=None)
    with SyncCrazyflie(URI, cf=cf) as scf:
        # Same setup as the proven s091_aruco_hover.py / s125 e049e09d
        # configuration: Kalman estimator + reset cycle.
        scf.cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(0.3)
        scf.cf.param.set_value("kalman.resetEstimation", 1)
        time.sleep(0.5)
        scf.cf.param.set_value("kalman.resetEstimation", 0)
        time.sleep(1.5)

        # Pose logger for diagnostic output (cheap — single config @ 5 Hz).
        lg = LogConfig(name="pose", period_in_ms=200)
        lg.add_variable("stateEstimate.x", "float")
        lg.add_variable("stateEstimate.y", "float")
        lg.add_variable("stateEstimate.z", "float")

        print(f"[t] takeoff → {TARGET_Z} m via MotionCommander")
        t0 = time.time()
        with MotionCommander(scf, default_height=TARGET_Z) as mc:
            # The MotionCommander already issued take_off on __enter__.
            # Sample altitude continuously to verify takeoff + hold.
            with SyncLogger(scf, lg) as logger:
                end = time.time() + 3.0 + HOVER_S + 0.5
                last_t = 0.0
                for entry in logger:
                    now = time.time() - t0
                    _ts, data, _logconf = entry
                    x = data["stateEstimate.x"]
                    y = data["stateEstimate.y"]
                    z = data["stateEstimate.z"]
                    if now - last_t > 0.5:
                        print(f"[t] t={now:5.1f}s  pos=({x:+.2f},{y:+.2f},{z:+.2f})  |xy|={(x*x+y*y)**0.5:.3f}")
                        last_t = now
                    if time.time() >= end:
                        break
            print(f"[t] {HOVER_S}s hover done, landing via MotionCommander.exit")
        print("[t] landed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

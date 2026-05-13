#!/usr/bin/env python3
"""minimal_takeoff.py — TRULY minimal cf2 SITL takeoff.

NO parameter tweaks, NO estimator reset, NO orchestrator.  Just:
  1. cflib connect to udp://127.0.0.1:19850
  2. MotionCommander.__enter__   → auto-takeoff to default_height=1.0
  3. sleep HOVER_S
  4. MotionCommander.__exit__    → auto-land

Used to isolate whether our orchestrator's setup is the aggressor —
if drone flies calmly here, the chaos came from something we added.
"""
import sys
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = "udp://127.0.0.1:19850"
TARGET_Z = 2.0
HOVER_S  = 5.0

def main():
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache=None)
    print(f"[min] connecting → {URI}")
    with SyncCrazyflie(URI, cf=cf) as scf:
        # NO param.set_value calls — let cf2 firmware use whatever
        # default estimator/controller it was compiled with.
        print(f"[min] takeoff → {TARGET_Z} m (auto, MotionCommander default)")
        t0 = time.time()
        with MotionCommander(scf, default_height=TARGET_Z) as mc:
            print(f"[min] hover {HOVER_S}s")
            time.sleep(HOVER_S)
            print(f"[min] landing (MotionCommander exit) at t={time.time()-t0:.1f}s")
        print("[min] done")
    return 0

if __name__ == "__main__":
    sys.exit(main())

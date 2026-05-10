#!/usr/bin/env python3
"""
cflib_takeoff_no_flow.py — baseline drift run.

Connect to cf2 SITL via cflib, takeoff to 1 m, hover for 30 s with NO
optical-flow correction, log estimated pose to csv/no_flow.csv.

The cf2 EKF will integrate IMU + baro only -> X/Y drifts noticeably.
This is the control case; cflib_takeoff_with_flow.py is the experiment.
"""
import csv
import os
import sys
import time
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI         = "udp://127.0.0.1:19850"
GATE_Z      = 0.30   # match with_flow profile (apples-to-apples)
GATE_HOLD_S = 2.0
TARGET_Z    = 1.0
HOVER_S     = 25.0
LAND_S      = 3.0
LOG_PERIOD_MS = 50

OUT = Path(__file__).parent / "csv" / "no_flow.csv"
OUT.parent.mkdir(parents=True, exist_ok=True)


def main():
    cflib.crtp.init_drivers()
    print(f"[no_flow] linking {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache=None)) as scf:
        cf = scf.cf
        print("[no_flow] linked")

        # Use the Extended Kalman estimator (= 2) for BOTH runs so the
        # comparison is apples-to-apples.  The only difference between
        # baseline and flow-corrected is whether flow packets get fed in.
        # Kalman uses baro for Z (so altitude stays roughly stable) and
        # integrates IMU for X/Y — without flow it drifts laterally,
        # which is exactly what we want to measure.
        cf.param.set_value("stabilizer.estimator", 2)
        time.sleep(1.0)
        print("[no_flow] estimator = EKF (2), no flow source")

        log_cfg = LogConfig(name="pose", period_in_ms=LOG_PERIOD_MS)
        log_cfg.add_variable("stateEstimate.x", "float")
        log_cfg.add_variable("stateEstimate.y", "float")
        log_cfg.add_variable("stateEstimate.z", "float")

        rows = []
        def cb(ts, data, _):
            rows.append((ts, data["stateEstimate.x"],
                         data["stateEstimate.y"], data["stateEstimate.z"]))

        cf.log.add_config(log_cfg)
        log_cfg.data_received_cb.add_callback(cb)
        log_cfg.start()

        # Same takeoff profile as cflib_takeoff_with_flow.py for fair
        # baseline comparison: ramp-hold-climb-hover-land.  Without flow
        # injection, the EKF integrates IMU noise -> X/Y drifts freely
        # under wind disturbance.
        print(f"[no_flow] ramp 0 -> {GATE_Z} m (1s)", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            cf.commander.send_hover_setpoint(0,0,0, GATE_Z * (time.monotonic()-t0))
            time.sleep(0.05)

        print(f"[no_flow] hold {GATE_HOLD_S}s at {GATE_Z} m", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < GATE_HOLD_S:
            cf.commander.send_hover_setpoint(0,0,0, GATE_Z); time.sleep(0.05)

        print(f"[no_flow] climb {GATE_Z} -> {TARGET_Z} m (2s)", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            frac = (time.monotonic()-t0) / 2.0
            cf.commander.send_hover_setpoint(0,0,0, GATE_Z + (TARGET_Z-GATE_Z)*frac)
            time.sleep(0.05)

        print(f"[no_flow] hover {HOVER_S}s at z={TARGET_Z} m (X/Y free)...", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < HOVER_S:
            cf.commander.send_hover_setpoint(0,0,0, TARGET_Z); time.sleep(0.05)

        print("[no_flow] landing ...", flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < LAND_S:
            t = 1.0 - (time.monotonic() - t0) / LAND_S
            z = max(0.05, TARGET_Z * t)
            cf.commander.send_hover_setpoint(0,0,0, z); time.sleep(0.1)

        cf.commander.send_stop_setpoint()
        log_cfg.stop()

    with open(OUT, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "x", "y", "z"])
        w.writerows(rows)
    print(f"[no_flow] wrote {len(rows)} rows -> {OUT}")


if __name__ == "__main__":
    sys.exit(main() or 0)

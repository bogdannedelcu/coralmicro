"""Low-altitude ArUco visibility smoke test (z=0.5m).

User spec: drone stabilizes at z=0.5m with the COMPACT marker layout
(markers at ±0.15, ±0.10 from origin, 8cm square).  Validates that
the new geometry actually fits all 4 markers in 640×480 FOV at z=0.5m
before we commit the rest of the PnP-altitude work.

What it checks per tick:
  - how many ArUco markers detected
  - per-marker PnP tvec (camera→marker translation)
  - drone EKF z vs PnP-derived z = mean(tvec[2]) across visible markers
  - drift between PnP-z and EKF-z over time

Pass criteria:
  - all 4 markers detected in ≥80% of frames during 5s hover at 0.5m
  - |pnp_z - ekf_z| < 5cm RMS  (cflib EKF at 0.5m is typically tight)

Usage:
    bash /tmp/run_lowalt_smoke.sh
    # which sets SENTAI_DUMP_FRAMES_DIR and starts sentai_sim + gz bridge
"""
from __future__ import annotations

import json
import math
import os
import random
import statistics
import sys
import threading
import time
from pathlib import Path

# Import the detector + known-positions table
sys.path.insert(0, str(Path(__file__).parent.parent / "s090_hover_over_cat"))
from aruco_detector import (
    detect_in_ppm, latest_ppm, KNOWN_POSITIONS_M, MARKER_SIZE_M,
    CAM_W, CAM_H,
)

TARGET_Z_M     = 0.5
HOVER_S        = 12.0
SAMPLE_HZ      = 5

# Wind disturbance: Gauss noise on velocity setpoints, refreshed at WIND_HZ.
# σ chosen so peak velocity ~3σ ≈ 15 cm/s lateral, 9 cm/s vertical (~realistic
# gusts at low altitude indoors).  Setpoint is body-frame velocity, so wind
# is "drone's controller fighting random disturbance" — the cf2 position
# controller will pull back toward target_z whenever vz reference drifts.
WIND_HZ        = 10
WIND_SIGMA_XY  = 0.05    # m/s lateral noise
WIND_SIGMA_Z   = 0.03    # m/s vertical noise
LOG_PATH       = Path(__file__).parent / "lowalt_log.json"


# ────────────────────────────────────────────────────────────
# cflib state
# ────────────────────────────────────────────────────────────
_lock = threading.Lock()
_ekf_x = _ekf_y = _ekf_z = 0.0
_roll = _pitch = _yaw = 0.0


def _att_cb(timestamp, data, _lc):
    global _ekf_x, _ekf_y, _ekf_z, _roll, _pitch, _yaw
    with _lock:
        _ekf_x = data["stateEstimate.x"]
        _ekf_y = data["stateEstimate.y"]
        _ekf_z = data["stateEstimate.z"]
        _roll = data["stateEstimate.roll"]
        _pitch = data["stateEstimate.pitch"]
        _yaw = data["stateEstimate.yaw"]


def get_ekf():
    with _lock:
        return _ekf_x, _ekf_y, _ekf_z, _roll, _pitch, _yaw


def pnp_altitude(dets) -> float | None:
    """Mean PnP tvec[2] across all detected markers — camera→marker
    distance.  For a down-cam pointed at the ground (markers at z=0),
    this equals the drone's altitude above the ground (modulo cam
    mount offset, which is sub-cm on cf2).  Returns None if no PnP."""
    zs = [m.tvec[2] for m in dets.values() if m.tvec is not None]
    if not zs:
        return None
    return statistics.mean(zs)


def main() -> int:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    frames_dir = Path(os.environ["SENTAI_DUMP_FRAMES_DIR"])
    assert frames_dir.is_dir(), (
        f"set SENTAI_DUMP_FRAMES_DIR + run sentai_sim first ({frames_dir})")

    cflib.crtp.init_drivers()
    sync = SyncCrazyflie("udp://127.0.0.1:19850", cf=Crazyflie(rw_cache=None))
    sync.open_link()
    cf = sync.cf

    cf.param.set_value("stabilizer.estimator", 2)
    time.sleep(0.5)
    cf.param.set_value("kalman.resetEstimation", 1)
    time.sleep(0.5)
    cf.param.set_value("kalman.resetEstimation", 0)
    time.sleep(2.0)

    lc = LogConfig(name="att", period_in_ms=20)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
        lc.add_variable(v, "float")
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(_att_cb)
    lc.start()

    mc = MotionCommander(sync, default_height=TARGET_Z_M)
    print(f"[smoke] take off to {TARGET_Z_M}m", file=sys.stderr)
    mc.take_off(height=TARGET_Z_M, velocity=0.3)
    time.sleep(2.0)

    # No Python-side noise injection.  Disturbances come from Gazebo IMU
    # noise plugin (see sentai_crazysim.sdf / cf2 model).
    print(f"[smoke] hovering at {TARGET_Z_M}m for {HOVER_S}s "
          f"(Gz IMU noise active)", file=sys.stderr)
    t0 = time.monotonic()
    last_seq = 0
    samples = []
    while time.monotonic() - t0 < HOVER_S:
        time.sleep(1.0 / SAMPLE_HZ)
        ppm = latest_ppm(frames_dir)
        if ppm is None:
            continue
        try:
            fseq = int(ppm.stem.replace("frame_", ""))
        except Exception:
            continue
        if fseq <= last_seq:
            continue
        last_seq = fseq

        try:
            dets = detect_in_ppm(ppm, estimate_pose=True)
        except Exception as e:
            print(f"[smoke] detect err: {e}", file=sys.stderr)
            continue

        ekf_x, ekf_y, ekf_z, _, _, _ = get_ekf()
        pnp_z = pnp_altitude(dets)
        n_det = len(dets)
        ids = sorted(dets.keys())
        per_marker = {}
        for mid, m in dets.items():
            if m.tvec is not None:
                per_marker[str(mid)] = {
                    "cx_px": round(m.cx, 1),
                    "cy_px": round(m.cy, 1),
                    "tvec_x": round(float(m.tvec[0]), 4),
                    "tvec_y": round(float(m.tvec[1]), 4),
                    "tvec_z": round(float(m.tvec[2]), 4),
                }
        samples.append({
            "fseq": fseq,
            "n_detected": n_det,
            "ids": ids,
            "ekf_x": round(ekf_x, 4),
            "ekf_y": round(ekf_y, 4),
            "ekf_z": round(ekf_z, 4),
            "pnp_z": round(pnp_z, 4) if pnp_z is not None else None,
            "z_err_cm": round((pnp_z - ekf_z) * 100, 2) if pnp_z is not None else None,
            "per_marker": per_marker,
        })
        print(f"[smoke] fseq={fseq:>4}  n={n_det}  ids={ids}  "
              f"ekf=({ekf_x:+.2f},{ekf_y:+.2f},{ekf_z:.3f})  "
              f"pnp_z={pnp_z if pnp_z is None else f'{pnp_z:.3f}'}  "
              f"err_cm={'n/a' if pnp_z is None else f'{(pnp_z-ekf_z)*100:+.1f}'}",
              file=sys.stderr)

    mc.land(velocity=0.3)
    time.sleep(2.0)
    lc.stop()
    sync.close_link()

    # Summary
    if not samples:
        print("[smoke] NO SAMPLES — fix infrastructure", file=sys.stderr)
        return 1

    full_detect = [s for s in samples if s["n_detected"] == 4]
    detect_4_rate = len(full_detect) / len(samples)
    with_pnp = [s for s in samples if s["pnp_z"] is not None]
    if with_pnp:
        z_errs_cm = [s["z_err_cm"] for s in with_pnp]
        rms_err_cm = math.sqrt(sum(e * e for e in z_errs_cm) / len(z_errs_cm))
        mean_err_cm = statistics.mean(z_errs_cm)
    else:
        rms_err_cm = mean_err_cm = float("nan")

    print(f"\n=== SMOKE SUMMARY (z={TARGET_Z_M}m, {HOVER_S}s hover) ===",
          file=sys.stderr)
    print(f"samples           : {len(samples)}", file=sys.stderr)
    print(f"all-4-detect rate : {detect_4_rate*100:5.1f}%  "
          f"({len(full_detect)}/{len(samples)})", file=sys.stderr)
    print(f"PnP samples       : {len(with_pnp)}/{len(samples)}", file=sys.stderr)
    print(f"|pnp_z - ekf_z|   : mean={mean_err_cm:+.2f} cm  "
          f"RMS={rms_err_cm:.2f} cm", file=sys.stderr)

    pass_4det = detect_4_rate >= 0.80
    pass_z = (not math.isnan(rms_err_cm)) and rms_err_cm < 5.0
    print(f"PASS all4_det>=80%: {'YES' if pass_4det else 'no'}", file=sys.stderr)
    print(f"PASS |z|_rms<5cm  : {'YES' if pass_z else 'no'}", file=sys.stderr)

    LOG_PATH.write_text(json.dumps({
        "target_z_m": TARGET_Z_M,
        "hover_s": HOVER_S,
        "n_samples": len(samples),
        "all4_detect_rate": detect_4_rate,
        "z_err_mean_cm": mean_err_cm if not math.isnan(mean_err_cm) else None,
        "z_err_rms_cm": rms_err_cm if not math.isnan(rms_err_cm) else None,
        "pass_4det": pass_4det,
        "pass_z": pass_z,
        "_last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "samples": samples,
    }, indent=2))
    print(f"[smoke] log → {LOG_PATH}", file=sys.stderr)
    return 0 if (pass_4det and pass_z) else 1


if __name__ == "__main__":
    sys.exit(main())

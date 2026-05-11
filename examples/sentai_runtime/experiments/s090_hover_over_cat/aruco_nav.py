"""ArUco multi-marker navigation — TPU-independent FOV-safety validation.

User spec (2026-05-11):
    "centrezi pe unul si incerci sa navighezi spre altul pe care il ai
     in campul vizual"

Translation: anchor on one currently-visible marker, navigate toward
another, keep both in FOV throughout.  This is the validation test
for the velocity envelope produced by aruco_calibration.py — if the
envelope is correctly identified, visiting all 4 corner markers in
sequence should NEVER lose visual lock on the "anchor" (most central)
marker.

Algorithm per leg:
    1. Pick destination marker (next in tour sequence).
    2. PD in world frame using cflib EKF position vs known marker
       world position (KNOWN_POSITIONS_M from aruco_detector).
    3. Body-frame velocity = world-frame error × KP, clamped per axis
       by velocity_envelope.json (the FOV-safety guarantee).
    4. Track per-tick: which markers visible, which is "anchor"
       (closest to image center), max pitch/roll, distance to dest.
    5. Leg complete when |dest - drone| < ARRIVAL_THRESH_M.

Outputs nav_log.json with per-leg statistics + per-tick samples,
prints summary table to stderr.

Usage:
    SENTAI_DUMP_FRAMES_DIR=/tmp/sentai_frames_arucotour_$(date +%s) \
        python3 aruco_nav.py
    # Or via the bash wrapper which sets up sentai_sim + gz bridge:
    # bash run_aruco_nav.sh
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from aruco_detector import (
    detect_in_ppm, latest_ppm, KNOWN_POSITIONS_M,
    CAM_W, CAM_H,
)

# ────────────────────────────────────────────────────────────
# Constants
# ────────────────────────────────────────────────────────────
NAV_TARGET_Z_M       = 2.5          # cruise altitude (matches calibration z)
TOUR_SEQUENCE        = [1, 2, 3, 0]  # NW → SW → SE → NE (CCW around square)
ARRIVAL_THRESH_M     = 0.20          # |xy_world - target_world| considered "arrived"
LEG_TIMEOUT_S        = 25.0
KP_WORLD             = 0.6           # world-frame PD proportional gain
SAMPLE_RATE_HZ       = 10
ENVELOPE_PATH        = Path(__file__).parent / "velocity_envelope.json"
LOG_PATH             = Path(__file__).parent / "nav_log.json"

V_MAX_FALLBACK_X_M   = 0.10
V_MAX_FALLBACK_Y_M   = 0.20


# ────────────────────────────────────────────────────────────
# Shared state (50 Hz attitude+position log)
# ────────────────────────────────────────────────────────────
_att_lock = threading.Lock()
_roll = _pitch = _yaw = _x = _y = _z = 0.0


def _att_cb(timestamp, data, _logconf):
    global _roll, _pitch, _yaw, _x, _y, _z
    with _att_lock:
        _roll = data["stateEstimate.roll"]
        _pitch = data["stateEstimate.pitch"]
        _yaw = data["stateEstimate.yaw"]
        _x = data["stateEstimate.x"]
        _y = data["stateEstimate.y"]
        _z = data["stateEstimate.z"]


def get_pose():
    with _att_lock:
        return (_roll, _pitch, _yaw, _x, _y, _z)


# ────────────────────────────────────────────────────────────
# Envelope loading
# ────────────────────────────────────────────────────────────
def load_envelope() -> tuple[float, float]:
    try:
        env = json.loads(ENVELOPE_PATH.read_text())
        return float(env["v_max_x_mps"]), float(env["v_max_y_mps"])
    except FileNotFoundError:
        print(f"[nav] no envelope, falling back to "
              f"({V_MAX_FALLBACK_X_M}, {V_MAX_FALLBACK_Y_M})", file=sys.stderr)
        return V_MAX_FALLBACK_X_M, V_MAX_FALLBACK_Y_M


# ────────────────────────────────────────────────────────────
# Sampling helpers
# ────────────────────────────────────────────────────────────
def sample_markers(frames_dir: Path, prev_seq: int):
    """Newest PPM with fseq > prev_seq → (fseq, {id: Marker} dict)."""
    ppm = latest_ppm(frames_dir)
    if ppm is None:
        return prev_seq, {}
    try:
        fseq = int(ppm.stem.replace("frame_", ""))
    except Exception:
        return prev_seq, {}
    if fseq <= prev_seq:
        return prev_seq, {}
    try:
        return fseq, detect_in_ppm(ppm, estimate_pose=False)
    except Exception:
        return fseq, {}


def closest_to_center(dets: dict):
    """Pick the detected marker whose centroid is closest to image
    centre.  This is the "anchor" — the one we want to NOT lose."""
    if not dets:
        return None
    cx0, cy0 = CAM_W / 2, CAM_H / 2
    return min(dets.values(),
               key=lambda m: (m.cx - cx0) ** 2 + (m.cy - cy0) ** 2)


# ────────────────────────────────────────────────────────────
# One leg: navigate to one waypoint with envelope clamp
# ────────────────────────────────────────────────────────────
def fly_leg(mc, target_id: int, v_cap_x: float, v_cap_y: float,
            frames_dir: Path) -> dict:
    """Navigate to target_id's known world position.  Returns leg
    statistics: arrival success, time, samples, anchor lost count."""
    target_xy = KNOWN_POSITIONS_M[target_id][:2]
    print(f"[nav] leg → marker id={target_id} world={target_xy}", file=sys.stderr)

    t0 = time.monotonic()
    n_total = 0
    n_anchor_lost = 0
    n_target_seen = 0
    n_markers_sum = 0
    last_seq = 0
    samples = []

    while True:
        now = time.monotonic()
        if now - t0 > LEG_TIMEOUT_S:
            print(f"[nav]   leg timeout after {LEG_TIMEOUT_S}s", file=sys.stderr)
            arrived = False
            break

        with _att_lock:
            ex = target_xy[0] - _x
            ey = target_xy[1] - _y
            cur_pitch, cur_roll = _pitch, _roll
        dist_m = math.hypot(ex, ey)

        if dist_m < ARRIVAL_THRESH_M:
            mc.start_linear_motion(0, 0, 0)
            print(f"[nav]   arrived (dist={dist_m:.3f}m) in {now-t0:.1f}s",
                  file=sys.stderr)
            arrived = True
            break

        # World-frame PD → body-frame velocity.  cf2 with yaw=0 has body
        # X axis aligned to world X axis (drone faces +X by default).
        # If yaw drifts the conversion is roll×cos/sin — for the SIM
        # default-orientation case (yaw~0), body ≈ world.
        vx_world = KP_WORLD * ex
        vy_world = KP_WORLD * ey

        # Per-axis clamp from velocity_envelope.json — the FOV-safe
        # envelope. Direction preserved.
        vx_body = max(-v_cap_x, min(v_cap_x, vx_world))
        vy_body = max(-v_cap_y, min(v_cap_y, vy_world))

        mc.start_linear_motion(vx_body, vy_body, 0)

        # Sample markers
        fseq, dets = sample_markers(frames_dir, last_seq)
        if fseq > last_seq:
            last_seq = fseq
            n_total += 1
            n_markers_sum += len(dets)
            if target_id in dets:
                n_target_seen += 1
            anchor = closest_to_center(dets)
            if anchor is None:
                n_anchor_lost += 1
            samples.append({
                "t": round(now - t0, 3),
                "fseq": fseq,
                "dist_m": round(dist_m, 3),
                "vx_cmd": round(vx_body, 3),
                "vy_cmd": round(vy_body, 3),
                "pitch": round(cur_pitch, 3),
                "roll": round(cur_roll, 3),
                "n_markers": len(dets),
                "target_seen": target_id in dets,
                "anchor_id": int(anchor.id) if anchor is not None else -1,
            })

        time.sleep(1.0 / SAMPLE_RATE_HZ)

    return {
        "target_id": target_id,
        "target_world": list(target_xy),
        "arrived": arrived,
        "duration_s": round(time.monotonic() - t0, 2),
        "n_samples": n_total,
        "n_anchor_lost": n_anchor_lost,
        "n_target_seen": n_target_seen,
        "target_seen_rate": (n_target_seen / n_total) if n_total else 0.0,
        "anchor_lost_rate": (n_anchor_lost / n_total) if n_total else 0.0,
        "avg_markers_visible": (n_markers_sum / n_total) if n_total else 0.0,
        "samples": samples,
    }


# ────────────────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────────────────
def main() -> int:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    frames_dir = Path(os.environ["SENTAI_DUMP_FRAMES_DIR"])
    assert frames_dir.is_dir(), (
        f"set SENTAI_DUMP_FRAMES_DIR + run sentai_sim first ({frames_dir})")

    v_cap_x, v_cap_y = load_envelope()
    print(f"[nav] envelope: v_cap_x={v_cap_x:.2f} m/s  "
          f"v_cap_y={v_cap_y:.2f} m/s", file=sys.stderr)

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
    for v in ("stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw",
              "stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
        lc.add_variable(v, "float")
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(_att_cb)
    lc.start()

    mc = MotionCommander(sync, default_height=NAV_TARGET_Z_M)
    mc.take_off(height=NAV_TARGET_Z_M, velocity=0.4)
    time.sleep(2.0)

    legs = []
    for target_id in TOUR_SEQUENCE:
        leg = fly_leg(mc, target_id, v_cap_x, v_cap_y, frames_dir)
        legs.append(leg)
        time.sleep(1.0)

    mc.land(velocity=0.4)
    time.sleep(2.0)
    lc.stop()
    sync.close_link()

    # Summary
    print("\n=== TOUR SUMMARY ===", file=sys.stderr)
    print(f"{'leg':>3}  {'target':>6}  {'arrived':>7}  {'t_s':>5}  "
          f"{'samples':>7}  {'avg_mk':>6}  {'tgt%':>5}  {'anch_lost%':>9}",
          file=sys.stderr)
    for i, leg in enumerate(legs):
        print(f"{i+1:>3}  id={leg['target_id']:>3}  "
              f"{'YES' if leg['arrived'] else 'no':>7}  "
              f"{leg['duration_s']:>5.1f}  "
              f"{leg['n_samples']:>7}  "
              f"{leg['avg_markers_visible']:>6.2f}  "
              f"{leg['target_seen_rate']*100:>4.1f}  "
              f"{leg['anchor_lost_rate']*100:>8.1f}", file=sys.stderr)

    LOG_PATH.write_text(json.dumps({
        "envelope": {"v_max_x_mps": v_cap_x, "v_max_y_mps": v_cap_y},
        "target_z_m": NAV_TARGET_Z_M,
        "tour_sequence": TOUR_SEQUENCE,
        "legs": legs,
        "_last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }, indent=2))
    print(f"\n[nav] log → {LOG_PATH}", file=sys.stderr)
    return 0 if all(leg["arrived"] for leg in legs) else 1


if __name__ == "__main__":
    sys.exit(main())

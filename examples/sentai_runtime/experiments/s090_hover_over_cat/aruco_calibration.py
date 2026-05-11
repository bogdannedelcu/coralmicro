"""ArUco-based velocity envelope calibration — TPU-independent.

Identifies the maximum lateral velocity the drone can sustain WITHOUT
losing visual lock on a ground target.  The constraint isn't actuator
saturation (drone can do 0.5+ m/s easily); it's that the drone must
PITCH/ROLL to accelerate, and that pitch tilts the camera, which can
push the target out of FOV.  Literature calls this the "FOV
constraint" / "visibility constraint" in IBVS — see Sim.md §10j ArUco
calibration section for refs.

Algorithm (revised 2026-05-11 per user feedback "centrezi pe unul si
incerci sa navighezi spre altul"):

  1. Takeoff + fly to world (0, 0, 2.5) — all 4 corner markers (id 0..3)
     visible simultaneously at hover.
  2. For axis x ∈ {+X, +Y}:
       For v ∈ [0.05, 0.10, 0.15, 0.20, 0.25, 0.30] m/s:
         a. mc.start_linear_motion(v, 0, 0)  (axis-aware)
         b. hold STEP_HOLD_S, sample which markers stay detected +
            in-FOV-bounds; record max |pitch|/|roll|.
         c. mc.start_linear_motion(-v, 0, 0) — return-leg same duration
            (drone ends ~back at origin, not drifted off).
         d. Stop.  Wait SETTLE_S.
       v_max_safe = highest v at which detection_rate stayed ≥ DETECTION_RATE_THRESH
                    on ≥ N_REQUIRED_MARKERS of the corner markers.
  3. Save velocity_envelope.json:
       {"v_max_x_mps": ..., "v_max_y_mps": ...,
        "max_pitch_at_vmax_deg": ..., "max_roll_at_vmax_deg": ...,
        "calibration_z_m": ..., "marker_id_used": 0,
        "_last_run": "..."}

The runtime hover controller in hover_over_cat.py can read this file
and clamp `V_MAX_M` to `v_max_x_mps * 0.8` (20% safety margin) so the
SSD-based hover-over phase never commands a velocity that would tilt
the camera out of FOV.

Usage:
    python3 aruco_calibration.py
    # produces velocity_envelope.json + console summary
"""
from __future__ import annotations

import json
import math
import os
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from aruco_detector import (
    detect_in_ppm, latest_ppm, KNOWN_POSITIONS_M,
    CAM_W, CAM_H, CAM_FOV_H_RAD, CAM_FOV_V_RAD,
)

# ────────────────────────────────────────────────────────────
# Tuning constants
# ────────────────────────────────────────────────────────────
# Multi-altitude sweep: one flight, full schedule.  At each z the drone
# climbs, cruises to origin, runs X then Y sweep.  Result is a schedule
# of (z, v_max_x, v_max_y) rows that hover runtime can interpolate.
#
# Why these z values: at z<1.3m the corner anchor at (+0.7,+0.5) is OUT
# of FOV at pose neutral (geometry: 320 - fx*0.7/z < 30px), so 1.5m is
# practical floor.  At z>3.5m the SIM gz environment starts to lose
# marker resolution (each tag spans <30 px on a side).
Z_SCHEDULE_M         = [1.5, 2.0, 2.5, 3.0]
ANCHOR_MARKER_ID     = 0           # informational only — primary metric is avg_markers
N_REQUIRED_MARKERS   = 2           # PRIMARY threshold: avg markers visible per trial
V_STEP_M             = 0.05        # increment between trial velocities
V_MAX_TRIAL_M        = 0.40        # 40 cm/s upper bound (envelope grows with z)
STEP_HOLD_S          = 1.5         # how long to hold each velocity step
SETTLE_S             = 2.0         # let drone return + settle between steps
DETECTION_RATE_THRESH = 0.80       # min fraction of frames detecting anchor
EDGE_MARGIN_PX       = 5           # marker corners > 5px from image edge (just "not clipped")
SAMPLE_RATE_HZ       = 10          # poll rate during step
ENVELOPE_PATH        = Path(__file__).parent / "velocity_envelope.json"


# ────────────────────────────────────────────────────────────
# Shared state for the attitude logger
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


def get_att():
    with _att_lock:
        return _roll, _pitch, _yaw, _x, _y, _z


# ────────────────────────────────────────────────────────────
# Marker observation utilities
# ────────────────────────────────────────────────────────────
def marker_inside_frame(marker, margin: int = EDGE_MARGIN_PX) -> bool:
    """All 4 corners > margin px from any image edge."""
    cs = marker.corners
    return bool(
        cs[:, 0].min() > margin
        and cs[:, 0].max() < (CAM_W - margin)
        and cs[:, 1].min() > margin
        and cs[:, 1].max() < (CAM_H - margin)
    )


def sample_marker(frames_dir: Path, prev_seq: int) -> tuple[bool, int, dict | None]:
    """Find the newest PPM (with fseq > prev_seq), detect markers, return
    (has_detection, fseq, marker_dict)."""
    ppm = latest_ppm(frames_dir)
    if ppm is None:
        return False, prev_seq, None
    try:
        fseq = int(ppm.stem.replace("frame_", ""))
    except Exception:
        return False, prev_seq, None
    if fseq <= prev_seq:
        return False, prev_seq, None
    try:
        dets = detect_in_ppm(ppm, estimate_pose=False)
    except Exception:
        return False, fseq, None
    if CAL_MARKER_ID not in dets:
        return False, fseq, None
    return True, fseq, dets[CAL_MARKER_ID]


# ────────────────────────────────────────────────────────────
# Step trial — measure detection rate + max attitude at given v
# ────────────────────────────────────────────────────────────
def sample_all_markers(frames_dir: Path, prev_seq: int):
    """Find newest PPM, detect all markers. Returns (fseq, dict_of_markers)."""
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


def run_step_trial(mc, axis: str, v: float, frames_dir: Path) -> dict:
    """Step + return-step trial: command +v for STEP_HOLD_S, then -v
    same duration so drone ends near where it started.  Sample which
    markers are detected + in-bounds.  Returns metrics."""
    n_total = 0
    n_anchor = 0
    n_anchor_in_bounds = 0
    n_markers_avg = 0   # sum of markers-visible-count over all samples
    max_pitch = 0.0
    max_roll = 0.0
    last_seq = 0
    samples = []

    def do_step(vx, vy, label):
        nonlocal n_total, n_anchor, n_anchor_in_bounds, n_markers_avg
        nonlocal max_pitch, max_roll, last_seq
        mc.start_linear_motion(vx, vy, 0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < STEP_HOLD_S:
            time.sleep(1.0 / SAMPLE_RATE_HZ)
            with _att_lock:
                p, r = _pitch, _roll
            max_pitch = max(max_pitch, abs(p))
            max_roll = max(max_roll, abs(r))
            fseq, dets = sample_all_markers(frames_dir, last_seq)
            if fseq > last_seq:
                last_seq = fseq
                n_total += 1
                n_markers_avg += len(dets)
                m = dets.get(ANCHOR_MARKER_ID)
                if m is not None:
                    n_anchor += 1
                    if marker_inside_frame(m):
                        n_anchor_in_bounds += 1
                samples.append({
                    "label": label,
                    "fseq": fseq,
                    "n_markers": len(dets),
                    "anchor_seen": m is not None,
                    "anchor_in_bounds": (m is not None and marker_inside_frame(m)),
                    "pitch": p, "roll": r,
                })

    if axis == "x":
        do_step(+v, 0, "fwd")
        do_step(-v, 0, "ret")
    else:
        do_step(0, +v, "left")
        do_step(0, -v, "ret")

    mc.start_linear_motion(0, 0, 0)
    return {
        "n_total": n_total,
        "n_anchor_seen": n_anchor,
        "n_anchor_in_bounds": n_anchor_in_bounds,
        "anchor_detection_rate": (n_anchor / n_total) if n_total else 0.0,
        "anchor_in_bounds_rate": (n_anchor_in_bounds / n_total) if n_total else 0.0,
        "avg_markers_visible": (n_markers_avg / n_total) if n_total else 0.0,
        "max_pitch_deg": max_pitch,
        "max_roll_deg": max_roll,
    }


# ────────────────────────────────────────────────────────────
# Cruise to world (0, 0, z) — drone needs all 4 markers in FOV
# ────────────────────────────────────────────────────────────
def climb_to(mc, target_z: float, timeout_s: float = 12.0) -> bool:
    """Open-loop climb/descend to target_z.  Sends vertical velocity
    commands until |z - target| < 10cm or timeout.  Uses cflib EKF z."""
    KP_Z = 0.6
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        with _att_lock:
            dz = target_z - _z
        if abs(dz) < 0.10:
            mc.start_linear_motion(0, 0, 0)
            return True
        vz = max(-0.4, min(0.4, dz * KP_Z))
        mc.start_linear_motion(0, 0, vz)
        time.sleep(0.1)
    mc.start_linear_motion(0, 0, 0)
    return False


def cruise_to_origin(mc, timeout_s: float = 10.0) -> bool:
    """Open-loop cruise to world (0, 0) using cflib's EKF position
    estimate.  Good enough to position the drone where all 4 corner
    markers are simultaneously visible."""
    KP = 0.5
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        with _att_lock:
            dx, dy = -_x, -_y
        if abs(dx) < 0.10 and abs(dy) < 0.10:
            mc.start_linear_motion(0, 0, 0)
            return True
        vx = max(-0.2, min(0.2, dx * KP))
        vy = max(-0.2, min(0.2, dy * KP))
        mc.start_linear_motion(vx, vy, 0)
        time.sleep(0.1)
    mc.start_linear_motion(0, 0, 0)
    return False


# ────────────────────────────────────────────────────────────
# Velocity-envelope sweep on one axis
# ────────────────────────────────────────────────────────────
def sweep_axis(mc, axis: str, frames_dir: Path) -> dict:
    print(f"\n[cal] sweeping body-{axis.upper()} velocity envelope", file=sys.stderr)
    results = []
    v_max_safe = 0.0
    v = V_STEP_M
    while v <= V_MAX_TRIAL_M + 1e-6:
        # Return to origin (where all 4 markers visible) between trials.
        cruise_to_origin(mc, timeout_s=5.0)
        time.sleep(SETTLE_S)
        r = run_step_trial(mc, axis, v, frames_dir)
        results.append({"v": v, **r})
        print(f"[cal]   v={v:.2f} m/s: "
              f"anchor_det={r['anchor_detection_rate']*100:5.1f}%  "
              f"anchor_in_bounds={r['anchor_in_bounds_rate']*100:5.1f}%  "
              f"avg_markers={r['avg_markers_visible']:.1f}  "
              f"max|pitch|={r['max_pitch_deg']:5.2f}°", file=sys.stderr)
        # PRIMARY safety metric: avg markers visible >= N_REQUIRED across the trial.
        # The anchor in-bounds check is secondary (only enforced when anchor is
        # geometrically reachable at this z — at low z the corner anchor may be
        # out-of-FOV even at pose neutral, in which case the avg-markers metric
        # is the only reliable signal).
        if r["avg_markers_visible"] >= N_REQUIRED_MARKERS:
            v_max_safe = v
        else:
            print(f"[cal]   v={v:.2f} m/s: visibility constraint hit "
                  f"(avg_mk={r['avg_markers_visible']:.1f}<{N_REQUIRED_MARKERS}) — "
                  f"stopping sweep", file=sys.stderr)
            break
        v += V_STEP_M
    return {
        "axis": axis,
        "v_max_safe_mps": v_max_safe,
        "trials": results,
    }


# ────────────────────────────────────────────────────────────
# Main
# ────────────────────────────────────────────────────────────
def main() -> int:
    # The wrapper that drives sentai_sim, manages MotionCommander, flight
    # log, frame dump dir, etc.  We reuse the existing infrastructure but
    # SKIP the SSD+cat tracking phase entirely.
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.positioning.motion_commander import MotionCommander
    from cflib.crazyflie.log import LogConfig

    frames_dir = Path(os.environ["SENTAI_DUMP_FRAMES_DIR"])
    assert frames_dir.is_dir(), f"set SENTAI_DUMP_FRAMES_DIR + run sentai_sim first ({frames_dir})"

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

    # Attitude + position log @ 50 Hz
    lc = LogConfig(name="att", period_in_ms=20)
    for v in ("stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw",
              "stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
        lc.add_variable(v, "float")
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(_att_cb)
    lc.start()

    # Take off to the first scheduled altitude
    z0 = Z_SCHEDULE_M[0]
    mc = MotionCommander(sync, default_height=z0)
    mc.take_off(height=z0, velocity=0.4)
    time.sleep(2.0)

    # In-flight altitude sweep — one decolare, calibration per z, then land.
    schedule = []
    for z in Z_SCHEDULE_M:
        print(f"\n[cal] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━",
              file=sys.stderr)
        print(f"[cal] altitude tier z={z:.2f} m", file=sys.stderr)
        print(f"[cal] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━",
              file=sys.stderr)

        climb_to(mc, z, timeout_s=10.0)
        time.sleep(1.5)
        cruise_to_origin(mc, timeout_s=8.0)
        time.sleep(SETTLE_S)

        # Snapshot the actual achieved z (climb may undershoot by ~10cm)
        with _att_lock:
            z_achieved = _z

        x_result = sweep_axis(mc, "x", frames_dir)
        time.sleep(2.0)
        cruise_to_origin(mc, timeout_s=5.0)
        time.sleep(SETTLE_S)
        y_result = sweep_axis(mc, "y", frames_dir)

        row = {
            "z_target_m": z,
            "z_achieved_m": round(z_achieved, 3),
            "v_max_x_mps": x_result["v_max_safe_mps"],
            "v_max_y_mps": y_result["v_max_safe_mps"],
            "_x_trials": x_result["trials"],
            "_y_trials": y_result["trials"],
        }
        schedule.append(row)
        print(f"\n[cal] z={z:.2f}m  →  v_max_x={row['v_max_x_mps']:.2f}  "
              f"v_max_y={row['v_max_y_mps']:.2f} m/s", file=sys.stderr)

    # Land
    try:
        mc.land(velocity=0.4)
    except Exception as e:
        print(f"[cal] WARN land: {e}", file=sys.stderr)
    lc.stop()
    sync.close_link()

    # Persist envelope as a SCHEDULE.  Format keeps backward-compat scalar
    # fields (v_max_x_mps / v_max_y_mps) populated from the middle tier
    # so legacy hover code without interpolation support still works.
    if not schedule:
        print("[cal] no calibrations completed", file=sys.stderr)
        return 1
    mid = schedule[len(schedule) // 2]
    out = {
        "schedule": schedule,
        # Legacy single-z fields (back-compat for hover_over_cat.py before
        # the interp wiring lands): use the middle tier.
        "v_max_x_mps": mid["v_max_x_mps"],
        "v_max_y_mps": mid["v_max_y_mps"],
        "calibration_z_m": mid["z_achieved_m"],
        "marker_id_used": ANCHOR_MARKER_ID,
        "_last_run": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    with open(ENVELOPE_PATH, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n[cal] velocity envelope schedule saved → {ENVELOPE_PATH}",
          file=sys.stderr)
    print(f"[cal]   {'z':>6}  {'v_max_x':>8}  {'v_max_y':>8}", file=sys.stderr)
    for row in schedule:
        print(f"[cal]   {row['z_achieved_m']:>6.2f}  "
              f"{row['v_max_x_mps']:>8.2f}  {row['v_max_y_mps']:>8.2f}",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

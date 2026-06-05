---
name: s130-45baseline-shipped
description: s130 Stage 4.5Baseline (image-only world-model navigation) — 2026-05-14 first PASS. Drone visits 4 ArUco markers using ONLY multi-marker PnP for pose+yaw; cf2.stateEstimate never read by control loop. Also exposes a stale R_cam_to_body in aruco_detector.py.
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**State 2026-05-14, commit pending on `integration/from-180bbb5f`**.

Drone tours 4 markers (±0.15×±0.10 m) using:
- `image_localize(dets, MARKER_WORLD_MAP)` → `(pose_xyz, yaw)` from
  multi-marker PnP + 2D Procrustes (closed-form). No IMU, no EKF in
  the inner loop.
- `sentai.objects.get(objid)` for target world lookup (validates the
  L2 round-trip; world coords seeded at mission start).
- `sentai.servo.move()` for intent recording (L4).
- Reuses `ibvs_center_on_marker` from s129 for final pixel centring.

**Result (4/4 markers PASS):**
- pos error image-vs-cf2: median 1.2 cm, max 6.6 cm (thr 5/10 cm)
- yaw error image-vs-cf2: median 0.60°, max 3.12° (thr 3/5°)
- IBVS final px_err: 8.9, 8.9, 29.7, 26.9 (all < 30 threshold)
- 0 localization failures

## The yaw-from-baselines primitive (image-only, no IMU)

For ≥2 visible markers with known world XY:
1. Body-frame positions: `bᵢ = R_cam_to_body @ tvecᵢ`
2. Center both sets: `b̄ᵢ = bᵢ - mean(b)`, `w̄ᵢ = wᵢ - mean(w)`
3. Closed-form 2D Procrustes:
   ```
   yaw = atan2(Σᵢ b̄ᵢ.x·w̄ᵢ.y - b̄ᵢ.y·w̄ᵢ.x,
               Σᵢ b̄ᵢ.x·w̄ᵢ.x + b̄ᵢ.y·w̄ᵢ.y)
   ```

Then pose = `marker_world - R_cam_to_world(yaw) @ tvec` averaged over
visible markers, minus `R_body_to_world(yaw) @ CAM_OFFSET_BODY` for
drone CoM.

ARM portability: ~40 lines float-only, no SVD, no OpenCV. Trivial port
to CMSIS-DSP `arm_mat_*` for Stage 7+ (~10 µs theoretical on M7).

## Stale R_cam_to_body in aruco_detector.py — DISCOVERED + worked around

`aruco_detector._R_CAM_TO_BODY = [[0,-1,0],[-1,0,0],[0,0,-1]]` was
calibrated against an older camera SDF. Current sentai_crazysim camera
yields tvecs that need `R = [[0,+1,0],[+1,0,0],[0,0,-1]]` (X/Y rows
sign-flipped, equivalent to extra 180° rotation around Z).

**Symptom**: `estimate_drone_world_pose(dets, drone_yaw=0)` returns
pose reflected through origin + yaw mistakenly reported as 180°. s128
and s129 work because their IBVS empirical signs `body_dx=+tvec[1]`,
`body_dy=+tvec[0]` HAPPEN to match the new R (`R_new @ (+tvec[0..1], 0)
= (tvec[1], tvec[0], 0)`).

**Affected**: any consumer of `estimate_drone_world_pose`:
- `s091_aruco_lowalt/aruco_hover.py` (line 364)
- `sim/scripts/aruco_to_vision_estimate.py` (line 131) — PX4 VPE bridge
- These haven't been re-validated end-to-end since aruco_detector last
  worked.

**Workaround in s130**: local `_R_CAM_TO_BODY_LOCAL` + local
re-implementation of pose recovery. Avoids touching aruco_detector.

**TODO**: fix `_R_CAM_TO_BODY` in aruco_detector.py + re-validate the
PX4 VPE path. Probably ≤ 50 lines (matrix + sign flips) but needs an
end-to-end PX4 hover run.

## Pass criteria (codified in s130/verdict.py)

```
servo trace structural OK (faults==0, last=DISARM/0, armed/flight=0/GROUND)
4/4 markers IBVS-converged with px_err < 30
n_localize_failures == 0
image-vs-cf2 pos: median < 5 cm, max < 10 cm
image-vs-cf2 yaw: median < 3°, max < 5°
```

## Files

- `examples/sentai_runtime/experiments/s130_image_only_nav/`:
  - `README.md`, `mission_l45.py`, `verdict.py`, `run.sh`
- `examples/sentai_runtime/experiments/s128_l41baseline_seeded/mission_l41.py`:
  added optional `frames_dir_base` kwarg to ReplDriver (backward
  compatible — defaults to module WORKDIR).

## How to reproduce

```bash
bash examples/sentai_runtime/experiments/s130_image_only_nav/run.sh
# Single-target smoke:
S130_SINGLE_TARGET=0 bash examples/sentai_runtime/experiments/s130_image_only_nav/run.sh
# Verbose REPL (sign-convention debug):
S130_VERBOSE=1 bash examples/sentai_runtime/experiments/s130_image_only_nav/run.sh
```

## Architecture significance

Validates the pipeline that becomes permanent at:
- **Takeoff calibration** — set origin from visible markers before
  flow takes over (current cf2 EKF starts at 0 anyway, but on hardware
  + drift this matters).
- **Stage 6 loop closure on yaw** — re-acquire pose via PnP when
  markers come back into FOV.

Independent of `sentai.places.query` (Stage 11.D, descriptor-based
topological) and `sentai_object_lifter` (Stage 5, lift tracklets to
3D landmarks) which become the long-term world model.

Related: [[servo-l4-shipped]], [[s128-l41baseline-shipped]],
[[s129-l42baseline-shipped]], [[objects-l2-shipped]],
[[places-l3-shipped]], [[objectsplan]], [[objectsplan-l5-handoff]],
[[flowbaseline-canonical-config]],
[[experiments-start-from-origin]], [[sentai-sim-journal]].

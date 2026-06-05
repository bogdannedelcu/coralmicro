---
name: camera-mount-calibration
description: "Real-world hardware concern (operator-flagged 2026-05-14): per-unit OV5640 mount tolerance ±2-5° → R_cam_to_body unknown without calibration. Plan adds Kabsch 3D Procrustes auto-calibration at takeoff via ArUco landing pad. objects_plan.md §21."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Concern flagged by operator 2026-05-14, mid-session post-s130.**

In SIM, `R_cam_to_body` is exact (vine din SDF). On real hardware
(production Crazyflie + OV5640 module):

- Glue/screw mount has ±2-5° mechanical tolerance per unit
- Vibrations + minor crashes shift mount over time
- Two physically identical drones run identical firmware will have
  DIFFERENT R_cam_to_body → per-unit calibration mandatory

**Why:** 3° error in R_cam_to_body @ z=1.5 m → bias bearing ≈ 8 cm
horizontal. Unacceptable for landing precision or loop closure.
This is a load-bearing assumption in Stage 4.5 image-only nav,
Stage 5 lifter inverse-depth EKF, Stage 6 yaw loop closure.

**How to apply (long-term plan, see objects_plan.md §21):**
- Auto-calibration at takeoff via ArUco landing pad
- Kabsch 3D Procrustes (closed-form SVD) → R_cam_to_body
- Persist in `/system/cam_calib.json` (FileX, schema-versioned)
- Re-validate every takeoff; trigger re-calib if delta > 3°
- Degraded mode if marker not visible (use persisted R)
- Self-healing: corrupt cam_calib.json never bricks boot

**Implementation order:**
1. Pas 2 immediate: host-side shared module
   `examples/sentai_runtime/_shared/camera_calibration.py`
   (Kabsch implementation + JSON schema).
2. Stage 6 follow-up: on-board `sentai.calib` MP binding
   - `sentai.calib.cam_to_body_from_aruco(samples)` → R
   - `sentai.calib.save_cam_to_body(R)` / `.load_cam_to_body()`
3. Stage 4.5 + Stage 5 consume `sentai.calib.load_cam_to_body()`
   instead of hardcoded matrix in `_R_CAM_TO_BODY_LOCAL`.

**Cost:** ~15 ms one-shot at takeoff (30 samples × 0.5 ms + 1× SVD
3×3 ~50 µs). Negligible vs. compute budget M7 (CMSIS-DSP `arm_mat_*`).
Runtime: 1× JSON load at boot.

**Related to:** [[s130-45baseline-shipped]] (current `_R_CAM_TO_BODY_LOCAL`
is the hardcoded SIM value), [[objectsplan-l5-handoff]] (Stage 5
lifter depends on calibrated R), [[no-broken-branch-test-reuse]]
(don't auto-port stale aruco_detector R — calibrate instead).

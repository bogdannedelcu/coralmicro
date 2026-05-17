<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 4139,4248. -->
<!-- Translated to English 2026-05-17 per English-only docs rule. -->

# Chapter 11_camera_calib — Camera-to-body extrinsic auto-calibration (§21)

WBS anchors: `OP-S6-W1` — `sentai.calib` MP binding.

## 21. Camera-to-body orientation calibration at takeoff (HARDWARE REALITY 2026-05-14)

### 21.1 The problem (operator-flagged 2026-05-14)

**Concern**: in SIM the camera orientation on the drone body is exact
(it comes from the SDF).  On real hardware:

- Soldering / mounting the OV5640 module on the Crazyflie body has
  mechanical tolerance (~±2–5° pitch/roll/yaw between the body plane
  and the sensor plane).
- Vibrations + minor in-flight impacts can shift the mount (wear).
- Two physical drones running the same firmware will NOT share the
  same `R_cam_to_body` — each unit needs per-unit calibration.

All the proposed algorithms (Stage 4.5 image-only nav, Stage 5
inverse-depth EKF lifter, Stage 6 yaw loop closure) **assume**
`R_cam_to_body` is known exactly.  An orientation error propagates
directly into:
- the lifter's bearing direction → marker world-position bias
- yaw recovery from Procrustes → drone-heading bias
- IBVS pixel servoing → biasing the saturation point

**A 3° error in `R_cam_to_body` at z = 1.5 m → bearing bias ≈ 8 cm in
the horizontal plane.** Unacceptable for precise landing or loop
closure.

### 21.2 Proposed solution — auto-calibration at takeoff using an ArUco

**Concept**: during takeoff (or immediately after) the drone runs a
short calibration procedure where:

1. The drone hovers at `z = 0.5–1.5 m` above a known ArUco marker
   (typically the landing-pad marker).
2. The drone executes a **known controlled motion** (e.g. slow 360°
   yaw or ±0.1 m lateral translation).
3. At each frame:
   - The ArUco detector publishes `tvec_cam, rvec_cam` (PnP).
   - cf2 / PX4 EKF publishes drone state `(x_W, y_W, z_W, yaw_W)`.
4. For each frame we have:
   - `marker_world_known` = (0, 0, 0) (landing-pad origin)
   - `cam_world = marker_world − R_cam_to_world @ tvec_cam`
   - But `R_cam_to_world = R_body_to_world(yaw_W) @ R_cam_to_body`
   - Unknown: `R_cam_to_body` (3×3 matrix, parametrized as a 4-DOF
     quaternion).
5. **Kabsch 3D Procrustes** on N ≥ 4 frames → closed-form solution
   for `R_cam_to_body`:
   ```
   H = Σ (tvec_cam_i) ⊗ (R_W_B(yaw_i)^T @ (marker_W − drone_W_i))^T
   U Σ V^T = SVD(H)
   R_cam_to_body = V @ diag(1, 1, det(V @ U^T)) @ U^T
   ```

### 21.3 Persistence + verification at every takeoff

- The calibration result (`R_cam_to_body`) is persisted to
  `/system/cam_calib.json` (FileX user partition, schema-versioned).
- At every takeoff:
  - Read `cam_calib.json`.
  - Run a quick "calibration sanity check" (1–2 s with an ArUco
    marker in FOV) — if the new estimate differs from the persisted
    one by more than 3°, re-calibrate and overwrite.
  - If the landing-pad marker is NOT visible → log a warning and
    continue with the persisted value (degraded mode).

### 21.4 Planned implementation

This is the immediate "Pas 2" after s131:
- Create `examples/sentai_runtime/_shared/camera_calibration.py`
  (host module, Python — shared by all experiments).
- Create the `sentai.calib` MicroPython binding (Stage 6 follow-up,
  WBS `OP-S6-W1`) for on-board calibration at takeoff:
  - `sentai.calib.cam_to_body_from_aruco(samples)` — Kabsch 3D
    Procrustes over samples collected from controlled flight.
  - `sentai.calib.save_cam_to_body(R)` → `/system/cam_calib.json`.
  - `sentai.calib.load_cam_to_body()` → R matrix, or identity if
    missing.
- Stage 5 lifter + Stage 4.5 `image_localize` go through
  `sentai.calib` for R instead of relying on hard-coded constants.

### 21.5 Fault modes (per `embeded.md` discipline)

| Failure | Detection | Reaction |
|---|---|---|
| Landing-pad marker not visible at takeoff | bbox count == 0 after 3 s | use persisted value; emit `CAL_NO_MARKER` event |
| Calibration produces R with det(R) < 0.99 | SVD post-check | reject + use persisted; emit `CAL_DET_FAIL` event |
| New calibration differs > 10° from persisted | comparison post-Kabsch | reject + log; assume critical marker drift / mechanical shift |
| `cam_calib.json` corrupt or missing | JSON parse fail | fall back to identity + emit `CAL_SCHEMA_FAIL`; mission continues in degraded mode |
| Calibration drift > 3° in flight | runtime monitor (periodic check at hover) | trigger re-calibration at next hover; emit `CAL_DRIFT` event |

The system remains **self-healing**: calibration corruption does NOT
brick the boot; it merely degrades operation until the next takeoff
with a visible marker.

### 21.6 Compute cost (M7 budget)

Per takeoff (one-shot, ~1–2 s):
- 30 samples × (tvec computation 0.5 ms + matrix push) → 15 ms total
  accumulation.
- 1× SVD 3×3 closed-form (LAPACK / CMSIS-DSP) → ~50 µs.
- Total ~15 ms one-shot, completely outside the hot path.

Runtime (load `cam_calib.json` at boot): 1 `fs.read` + JSON parse
~5 ms, once in `main_freertos.cc` init.

**Verdict**: compute negligible, FileX overhead negligible.  The
problem is implementation discipline (frame conventions + SVD
numerical stability), not compute.

---

*Operator-flagged 2026-05-14: real-world camera-mount tolerance ≠
SIM SDF exact.  Calibration at takeoff is mandatory before hardware
deployment, optional in SIM (we already have identity).*

---
name: sentai-calib-is-production-bringup
description: "sentai.calib este metoda de fabrică (production bringup) pentru fiecare drone, nu un experiment de test. Trebuie validată pe Gazebo ca digital twin."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: d6a1253b-689f-48a9-ae07-ff4bbdfffe61
---

`sentai.calib` is the on-board **production bringup method** — runs on
every shipped drone at first-boot / service to auto-discover the
drone-specific parameters that vary unit-to-unit:

- Camera intrinsics (fx, fy, cx, cy) — lens-to-lens variance
- Camera extrinsics (R_cam_to_body, cam_offset_B) — mount tolerance ±2-5°
- (future) PID / motor response — payload, battery, motor balance
- (future) Sensor noise characterisation
- (future, 2026-05-22) Manual-takeoff thrust ramp rate
  (`thrust_base`, `thrust_max`, `ramp_seconds`) — drone-to-drone mass
  varies (battery age, payload, frame trim).  Same ramp lifts heavier
  drones too slowly (marker-acquisition timeout) or lighter drones too
  fast (overshoot, FOV loss).  W21-T7 (s190) hard-codes a reference
  ramp; auto-learn deferred until iter data is repeatable.

Output persisted to `/system/cam_calib.json` and consumed by all
downstream consumers (`sentai.markers.set_cam_extrinsics`, picker,
mission code).

**Why:** Operator-stated 2026-05-21. The thesis claim and the
deployment story require that we ship ONE firmware that auto-tunes
per drone, not that we hand-tune each drone in the lab. Hundreds-to-
thousands of units must self-calibrate without engineer touch.

**How to apply:**

- ANY change that touches the calib data flow (markers backend swap,
  PnP solver replace, frame format change, intrinsics convention) MUST
  be validated by re-running calib **end-to-end in Gazebo**, NOT by a
  synthetic-samples smoke alone.  Synth tests only validate the inner
  numerics (Kabsch); they cannot catch a regression in sample
  collection, PnP bias, frame rate effects, EKF coupling, or perception
  noise — exactly the failure modes that matter at deployment.

- Gazebo is the **digital twin** for the first drone of the production
  line. The calib mission run in SIM must mirror exactly what we'd run
  on the bench in front of a real unit (hover sequence, sample budget,
  acceptance gate). When we cut the production firmware, the same
  `sentai.calib.run_bringup(...)` is what the bench technician triggers
  on the real drone.

- Synth smokes (e.g. s157 ArUco, s186 WhyCon) stay as **numerical
  regression gates** — they ARE useful, just NOT a stand-in for the
  Gazebo end-to-end test.

- Separately: `sentai.autotune` (s174-style PID/Kp tuning) is its own
  bringup subsystem, also lives ON the drone, also runs per-unit.
  Don't conflate with calib — but the disciplines are parallel:
  on-board bringup, validated in Gazebo, deployed at scale.

Reference: [[op-s10-w14-autotune-converged]] for the parallel
autotune bringup. [[op-s6-w1-calib-shipped]] for the original calib
numeric port. `s187_calib_gazebo_whycon` will be the first Gazebo
end-to-end calib bringup test on the WhyCon pad.

---
name: no-flow-deck-camera-imu-only-2026-05-19
description: "HARD ARCHITECTURAL FACT (operator confirmed 2026-05-19): the SentAI drone stack has NO Flow Deck.  Sensors are ONLY (1) downward Coral OV5640 camera, (2) cf2 internal IMU (BMI088 accelerometer + gyro).  No VL53L0X TOF rangefinder, no PMW3901 optical-flow chip.  Therefore altitude Z and XY velocity are NOT independently available from a dedicated sensor — they come exclusively from camera+IMU fusion: ArUco PnP for absolute pose anchor + sentai.flow (downward camera phase-correlation) for high-rate translational velocity + IMU for high-rate inertial."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

Operator-instituted as a hard fact 2026-05-19 during the M7
budget analysis: *"nu avem Flow Deck! Tine minte asta. Ne
bazam doar pe camera si pe IMU"*.

## Sensor inventory

| Source | What | Rate | Where it lives |
|---|---|---:|---|
| **Coral OV5640 camera (downward)** | 320×240 grayscale frames | 30 fps | physical OV5640 sensor, CSI → SDRAM on M7 |
| **cf2 internal IMU (BMI088)** | 3-axis accel + 3-axis gyro | 500 Hz | inside Crazyflie 2.1 (not Coral side) |

That's it.  No expansion deck.  No rangefinder.  No PMW3901
optical-flow IC.

## What this means for state estimation

Everything except IMU comes through one camera frame:

- **ArUco PnP** (`sentai_aruco_detect` → `sentai_calib_task` VPE
  forwarder): absolute (x, y, z, yaw) pose vs known marker pad,
  ~30 Hz live.  THE primary exteroceptive anchor.  Without ArUco
  visibility (off pad / lost) the EKF has only IMU → drifts.
- **sentai.flow** (downward camera phase correlation in
  `flow_task.cc`): image-plane optical flow `(vx, vy)` body-
  frame.  Needs altitude (from ArUco) to scale to metric.
  Provides smooth velocity between ArUco fixes.  Does **NOT**
  give Z directly — phase correlation is 2D image registration,
  no scale info.
- **IMU** (cf2 firmware EKF input): high-rate accel + gyro,
  drifts on integration; needs corrections from above to bound
  XY drift over time.

## Architectural implication for ArUco-rate planning

Reducing SafetyTask period from 33 ms (30 Hz) → 100 ms (10 Hz)
to free M7 budget **directly degrades altitude hold**: ArUco
is the sole Z source.  10 Hz VPE means cf2 EKF integrates IMU
accelerometer for 100 ms between corrections.  Acceptable for
hover with stable IMU, marginal for tight altitude tracking,
fails on aggressive Z maneuvers.

There is no zero-cost knob to reclaim M7 budget — the M4
investigation under `OP-S10-W16` is the necessary release valve.

## What this RULES OUT

- Any analysis assuming Flow Deck VL53L0X TOF range exists.
- Any "altitude from rangefinder" reasoning.
- Any "if camera fails we have backup velocity" reasoning.
- Any altitude logic that doesn't run through ArUco PnP +
  KNOWN_POS_M.

## What this RULES IN

- ArUco visibility ≡ position-hold viability.
- Mission planning must keep markers in FOV almost continuously
  (≥4 markers is the safety abort gate per
  `[[flowbaseline2-4markers-abort]]`).
- Indoor pad design must guarantee the 4-marker view over the
  entire operating volume.
- Outdoor PX4 missions (`OP-S10-W9`) use GPS for the same role
  ArUco plays indoor — different sensor, same architecture.

## Cross-refs

- `[[op-s10-w14-t18-simd-threshold-2026-05-19]]` — the 22 ms /
  30 Hz / 66 % M7 result that prompted this clarification.
- `[[op-s10-w16-multicore-2026-05-19]]` — the multi-core
  release-valve WP this constraint now makes urgent.
- `[[flowbaseline2-4markers-abort]]` — abort if <4 markers for
  30 frames (no fall-back altitude source).
- `examples/sentai_runtime/sentai_calib_task.cc:270-339` — VPE
  forwarder, 30 Hz hardcoded.
- `examples/sentai_runtime/flow_task.cc:14` — phase-correlation
  flow on M7, NOT on a Flow Deck.

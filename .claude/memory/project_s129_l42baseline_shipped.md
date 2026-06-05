---
name: s129-l42baseline-shipped
description: s129 L4.2Baseline (IBVS visual servoing) — 2026-05-14 first PASS. Drone centers on ArUco marker using PnP-tvec → R_cam_to_body → body-frame delta. Converges in 4-7 iters to px_dist < 30 px. Fixes the camera-offset bias that L4.1 EKF closed-loop cannot.
metadata: 
  node_type: memory
  type: project
  originSessionId: 1da91302-81c4-4cab-9a4b-2660dccdfd85
---

**State 2026-05-14**: single-marker IBVS centering converges from any
takeoff drift to pixel-tight in <8 iterations within 12 s timeout.
Final px_dist 15.1 px (= ~2.6 cm physical at z=1 m), comfortable margin
under 30 px tight threshold.

Path: `examples/sentai_runtime/experiments/s129_l42baseline_centering/`.

## Why L4.2 exists (vs L4.1)

L4.1 closed-loop on EKF residual converges cf2's position estimate to
within 3-7 cm of world-frame target, but visual centering then varies
18-82 px across markers because:
- Camera is 4 cm BACK of drone CoM along body +X (`<pose>-0.04 0 -0.02 ...`).
- cf2 yaw is nominally 0 but drifts ±1-2°.
- These contribute a SYSTEMATIC ~20-30 px bias that EKF cannot see.

L4.2 closes the loop on **image-derived metric**: PnP-tvec gives the
marker's position in CAMERA frame, which is the actual reference for
"centered in image".  No world-frame coords used in the inner loop.

## Control law (PBVS via PnP tvec)

```
tvec = marker position in camera frame from solvePnP
# To put camera over marker, camera must move by (-tvec[0], -tvec[1])
body_delta = R_CAM_TO_BODY @ (-tvec[0], -tvec[1], 0)
# With R_CAM_TO_BODY from aruco_detector.py:
#   body_dx = +tvec[1]
#   body_dy = +tvec[0]
```

Loop: detect → tvec → body_delta → `mc.move_distance` + `servo.move`
(intent recording) → `time.sleep(NAV_SETTLE_S=0.7)` → repeat until
`pixel_err < MARKER_PIXEL_TIGHT_PX=30` AND `|body_delta| < NAV_TOL_M=0.03`.

## Load-bearing tuning parameters

- `NAV_MAX_ITERS = 8` — enough for any reasonable initial offset
- `NAV_TIMEOUT_S = 12.0` — hard cap (closed-loop can't run forever)
- `NAV_SETTLE_S = 0.7` — cf2 PID needs time to act on each setpoint
- `NAV_MIN_STEP_M = 0.015` — ignore micro-corrections (PID dead-band)
- `IBVS_VEL_MPS = 0.20` — gentle move (no overshoot)

## Trap noted (don't repeat)

First implementation took a post-IBVS pixel reading AFTER 1.5 s
"settle" — drone drifted ~14 cm between converged-iter and the
sample, blowing up px_dist from 20 to 61.  Fix: take the final
pixel reading FROM THE CONVERGED IBVS ITER itself, not after.

## Frozen result vs L4.1

| Metric             | L4.1 EKF closed-loop | L4.2 IBVS  |
|--------------------|---------------------:|-----------:|
| Final pixel error  | 18–82 px (variable)  | 15.1 px    |
| Iter cap           | 5 (per waypoint)     | 8 (total)  |
| Camera-offset bias | NOT corrected        | corrected  |
| Inner loop source  | cf2.stateEstimate    | PnP tvec   |

## What L4.2 does NOT cover yet

- Single marker only (id0_NE).  Multi-marker IBVS centering would loop
  the IBVS primitive per marker (similar to s128's tour but with IBVS).
- No yaw control (just XY centering).  Heading alignment is L5+ work.
- No object discovery — target id is hardcoded.  True "scan-and-find"
  would be the next experiment (s130+).
- Stage 4.A still pending: `sentai.servo.move()` records intent only;
  cflib does the actual transport.  When Stage 4.A ships, the cflib
  calls collapse into `servo.move()`'s firmware-side backend.

Related: [[servo-l4-shipped]], [[s128-l41baseline-shipped]],
[[sentai-sim-journal]], [[experiments-start-from-origin]],
[[flowbaseline-canonical-config]].

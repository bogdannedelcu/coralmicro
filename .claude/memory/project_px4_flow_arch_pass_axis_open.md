---
name: PX4 + Garden + flow-only — architecture VALIDATED, axis tuning OPEN
description: s107 sanity (flow=0, wind=0) proved PX4 SITL can hover stable in OFFBOARD-POSITION + no-GPS + OPTICAL_FLOW_RAD. Drift < 2cm in 28s. Real-flow path open on axis sign convention — s092 cf2 mapping doesn't directly translate due to different firmware-side conventions.
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
s107 (2026-05-12) breakthrough finally proved PX4 flow-only is viable
on x86 SIM.  Architecture pieces validated:

- Airframe 4041 (no GPS, EKF2_OF_CTRL=1, COM_ARM_WO_GPS=1)
- SET_GPS_GLOBAL_ORIGIN pre-arm for "home set"
- Auto-heartbeat task in C (sentai.link.init starts it)
- OFFBOARD-POSITION setpoint via SET_POSITION_TARGET_LOCAL_NED
- Mock OPTICAL_FLOW_RAD bridge from gz pose
- pymavlink port `udp:127.0.0.1:14550` (Normal stream)

**Sanity result (flow=0 mock, wind=0):**
  - EKF: hovers at (0, 0, -1.49m up) target 1.5
  - gz ground truth drift < 2 cm in 28s
  - No Attitude failure, no arm-gate issues

**Open issue — flow sign convention for OPTICAL_FLOW_RAD:**
Tried 4 sign combinations of `flow_y = ±vx*dt/h, flow_x = ±vy*dt/h`
plus swap variants.  Each caused divergent drift in a DIFFERENT
direction (positive feedback loop, not just sign typo).

s092 cf2 SIM finding (`BODY_XFORM=(0,-1,-1,0)`, body +X → -L0_dy,
body +Y → -L0_dx) does NOT directly apply to PX4 OPTICAL_FLOW_RAD
because cf2 firmware consumes flow differently than PX4 EKF2.  PX4
EKF source (`EKF2.cpp:2191`) negates `pixel_flow` internally.

**Lesson (per Sim.md §10l):** *"Empirical control test > rotation-
matrix arithmetic."*  Replicating s092 protocol on PX4 (controlled
motion, +X then -X then +Y then -Y, observe which sign EKF velocity
estimate moves) is the proper way to nail this down.  Estimate
8-12h focused session for full axis calibration.

**IMU + motor noise stress factor (gz built-in):**
Added to x500_sentai SDF (outside coralmicro repo):
- IMU gaussian noise: gyro stddev 0.01 rad/s + bias 0.001;
  accel 0.1 m/s² + bias 0.01
- Motor asymmetry: motorConstant perturbed ±2% per rotor
  (default 8.54858e-06 → 8.61, 8.40, 8.71, 8.41 × e-6)

With noise + flow=0, gz drift over 10s = ~70 cm in both x AND y
simultaneously (motor asymmetry causes coupled drift, not axis-
aligned).  EKF holds (0,0) but real drone drifts → demonstrates
flow correction NEED for wind/noise rejection.

**Paths forward (priority for next session):**
1. **Empirical PX4 axis calibration** — s092 protocol on PX4:
   command +body_X velocity setpoint, observe EKF velocity sign;
   fix flow bridge to produce flow with matching sign.  Iterate.
2. **Use s104 GPS baseline** (proven 9.8cm drift) for wind test
   instead — apples-to-apples cf2 wind test still has flow on cf2.
   PX4 SIM "flow-only" remains a research goal but architecture
   already demonstrated.

**Files touched (outside coralmicro repo, NOT tracked):**
- `/home/bogdan/work/px4/PX4-Autopilot/Tools/simulation/gz/models/x500_sentai/model.sdf`
  IMU noise blocks + motor constant perturbations
- `/home/bogdan/work/crazyflie/CrazySim/.../sentai_crazysim.sdf`
  wind reset to (0 0 0); walls lowered to 0.5m tall; ground 25×25m;
  ArUco markers spaced 2× (±0.30 ±0.20)

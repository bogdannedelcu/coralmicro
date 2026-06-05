---
name: PX4 flow-only nav BLOCKED — flow is velocity, not position
description: PX4 EKF2 with EKF2_OF_CTRL=1 + flow-only refuses to arm (pos_horiz_ratio=NaN) because optical flow doesn't anchor absolute position. SET_GPS_GLOBAL_ORIGIN doesn't help. Use OFFBOARD or VISION_POSITION_ESTIMATE.
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
s101b (2026-05-12) hit a fundamental architecture limitation: PX4 v1.14
EKF2 will NOT produce a valid horizontal position estimate from
optical flow alone, even with all the right params and a SET_GPS_GLOBAL_ORIGIN.

**Why:** flow is a *velocity* sensor.  Without a separate position
anchor (GPS, VIO pose, ArUco PnP), position drifts unboundedly.
PX4 refuses to arm or takeoff when `pos_horiz_ratio = NaN`.

**Symptom:** `MAV_RESULT_TEMPORARILY_REJECTED` for ARM, no error
message in the PX4 log beyond `notify negative` tone.  ESTIMATOR_STATUS
shows `pos_horiz_ratio: NaN`, `vel_ratio: NaN`.

**Things that did NOT help (already tried, don't redo):**
- `EKF2_OF_CTRL=1` + `EKF2_GPS_CTRL=0` + `EKF2_BARO_CTRL=1` (correct
  v1.14 param names; old names AID_MASK/HGT_MODE/RNG_AID are
  deprecated)
- `EKF2_OF_QMIN=1` with quality-floor 50 in forwarder (so EKF
  accepts flow data even when drone is static + dx=dy=0)
- `EKF2_RNG_CTRL=1` to use distance field from OPTICAL_FLOW_RAD msg
- `COM_ARM_WO_GPS=1`
- `SET_GPS_GLOBAL_ORIGIN` + `SET_HOME_POSITION` via pymavlink before
  arm (origin accepted: `INFO [ekf2] 0 - New NED origin (LLA): ...`
  but `pos_horiz_ratio` stays NaN)

**Architectural difference vs cf2:** Crazyflie firmware integrates
flow velocity in its OWN position controller (no PX4-style "must have
absolute pos estimate to arm" gate).  cf2 + s091 worked because cf2
is happy with velocity hold.

**Paths forward** (s102+):
1. **OFFBOARD mode + SET_POSITION_TARGET_LOCAL_NED** — stream
   setpoints, PX4 follows.  Bypasses the EKF arm-gate via "you tell
   us where to be, we don't care about absolute pos."
2. **VISION_POSITION_ESTIMATE from ArUco PnP** — host-side ArUco
   detector → absolute drone pose → MAVLink msg → PX4 EKF2 gets
   true position anchor.  Reuses s091 ArUco code, matches PX4's
   `x500_vision` airframe pattern.
3. **PX4 source patch** to skip the EKF pos-horiz gate when only
   flow is available (force position estimate to (0,0,0) at arm
   time, integrate from there).  Heavy hammer, last resort.

s101 (GPS baseline) proves the rest of the chain works: airframe
4040, drone spawn, EKF, takeoff/land via NaN-lat/lon, hover drift
14 cm.  Plumbing parity with cf2 SITL achieved; what changes for
flow-only is just the navigator anchor.

Refs:
- https://discuss.px4.io/t/how-do-you-set-the-home-position-to-fly-with-optical-flow-navigation/26454
- https://github.com/PX4/PX4-Autopilot/issues/22250
- https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html

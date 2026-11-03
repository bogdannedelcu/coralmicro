# PX4 control interface for the SentAI follow-me vehicle (memo)

Scope: companion (SentAI, i.MX RT1176) on TELEM2, MAVLink v2, vision-only person estimate; path A = FOLLOW_TARGET + PX4 Follow Me, path B = Offboard velocity/acceleration setpoints. Date 2026-11-03. "main" = docs.px4.io/main (PX4 v1.17/v2.0 dev); v1.14/v1.15/v1.16 cited where they differ. Statements tagged **[src]** come from the PX4 source tree, not from the docs; "not in docs" is used where neither says anything.

Reference tags (each used inline below):

| Tag | URL |
|---|---|
| FM | https://docs.px4.io/main/en/flight_modes_mc/follow_me.html (same text in v1.15, v1.16; v1.14: https://docs.px4.io/v1.14/en/flight_modes_mc/follow_me.html) |
| FM13 | https://docs.px4.io/v1.13/en/flight_modes/follow_me.html |
| OB | https://docs.px4.io/main/en/flight_modes/offboard.html (v1.16: https://docs.px4.io/v1.16/en/flight_modes/offboard.html) |
| PR | https://docs.px4.io/main/en/advanced_config/parameter_reference.html#<PARAM> (v1.16: /v1.16/..., v1.14: /v1.14/...) |
| SF | https://docs.px4.io/main/en/config/safety.html |
| GF | https://docs.px4.io/main/en/flying/geofence.html |
| RTL | https://docs.px4.io/main/en/flight_modes/return.html and https://docs.px4.io/main/en/flight_modes_mc/return.html |
| CP | https://docs.px4.io/main/en/computer_vision/collision_prevention.html |
| JT | https://docs.px4.io/main/en/config_mc/mc_jerk_limited_type_trajectory.html |
| POS | https://docs.px4.io/main/en/flight_modes_mc/position.html |
| ARM | https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm.html |
| TKO | https://docs.px4.io/main/en/flight_modes_mc/takeoff.html |
| MP | https://docs.px4.io/main/en/peripherals/mavlink_peripherals.html |
| CC | https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html |
| DEV | https://docs.px4.io/main/en/concept/flight_modes.html |
| MAV | https://mavlink.io/en/messages/common.html#<MSG_OR_ENUM> |
| src | https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/... (file named inline) |

---

## 1. Follow Me flight mode

**Entry.** Selected from the GCS or an RC mode switch; it is an automatic mode with auto-takeoff, but the docs recommend taking off manually to 2-3 m first. On entry the copter climbs to a 1 m minimum safety altitude, then until it is within 3 m of `FLW_TGT_HT`, and only then moves horizontally, always yawing to face the target [FM]. Via MAVLink the mode is `MAV_CMD_DO_SET_MODE` with `base_mode=1 (CUSTOM_MODE_ENABLED)`, `custom_mode=4 (PX4_CUSTOM_MAIN_MODE_AUTO)`, `custom_sub_mode=8 (PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET)` **[src commander/px4_custom_mode.h, Commander.cpp]**; the docs only say "switch into follow-me mode" [FM]. `MAV_CMD_DO_FOLLOW` (32) is defined by MAVLink [MAV] but is not mentioned anywhere in the PX4 docs (not in docs).

**Vehicle requirements (main/1.15/1.16).** `mode_req_angular_velocity`, `mode_req_attitude`, `mode_req_local_alt`, `mode_req_local_position`, `mode_req_prevent_arming`, `mode_req_wind_and_flight_time_compliance` — "requires at least a valid local position estimate (does not require a global position)"; the vehicle **cannot be armed while this mode is selected** and will failsafe if the position estimate is lost [FM]. Gotcha **[src AutoFollowTarget/follow_target_estimator/TargetEstimator.cpp]**: the estimator projects `lat/lon` of the target into the local frame only when `vehicle_local_position.xy_global && z_global` and `ref_lat/lon/alt` are finite, i.e. the EKF needs a global origin (GNSS) even though the mode requirement says "local position". v1.13 says plainly "this mode requires GPS" [FM13]. The flight task also refuses to activate without a valid home altitude **[src FlightTaskAutoFollowTarget.cpp activate()]**.

**FOLLOW_TARGET (id 144) fields** [MAV]: `timestamp` (ms), `est_capabilities` (bitmask FOLLOW_TARGET_CAP_FLAGS: 1 POS, 2 VEL, 4 ACCEL, 8 ATT_RATES), `lat`/`lon` (degE7, WGS84), `alt` (m, **MSL**), `vel[3]` (m/s, LOCAL_NED, (0,0,0) = unknown), `acc[3]`, `attitude_q[4]`, `rates[3]`, `position_cov[3]`, `custom_state`.
What PX4 actually consumes (not in docs) **[src mavlink/mavlink_receiver.cpp handle_message_follow_target]**: only `lat`, `lon`, `alt`, `vel[0..2]`; `est_capabilities`, `acc`, `attitude_q`, `rates`, `position_cov`, `timestamp` are ignored (PX4 stamps the message with its own receive time). Mandatory in practice: lat/lon/alt; vel optional but **zeros are fused as a real 0 m/s measurement** (the receiver does not check the VEL capability bit), so if SentAI cannot estimate the person's velocity it should expect the estimate to be dragged toward zero. The docs stress that `alt` must be AMSL; QGC-Android sends ellipsoid height (up to 200 m off) — hence "do not use 3D tracking" [FM].

**Rate and timeout.** Not in docs. **[src TargetEstimator.hpp/.cpp]**: a position fix is fused at most every 500 ms and a velocity at most every 100 ms (duplicate/unchanged measurements are skipped unless 2x that age); the target is `stale` after **3000 ms** without a message (velocity/acceleration estimate zeroed, last position kept); the flight task resets its own 2nd-order filter when the estimator times out for **1.5 s** **[src FlightTaskAutoFollowTarget.hpp TARGET_ESTIMATOR_TIMEOUT_US]**. Sending at 2-5 Hz is therefore sufficient; faster is pointless for position (500 ms throttle). The v1.14/1.15 page warns that MAVSDK's occasional duplicate message "can confuse the target position and velocity estimator" [FM v1.14].

**Target-stream stop.** Not in docs. **[src FlightTaskAutoFollowTarget.cpp]**: while the estimate is `valid` but stale the drone keeps tracking the frozen position; if the estimate is invalid (never received / non-finite) the task commands position NaN + zero velocity, i.e. it **hovers in place** — there is no automatic Hold/RTL/Land on target loss. Emergency ascent at 0.5 m/s if distance-to-ground < 1 m.

**Filter.** `FLW_TGT_RS` sets the responsiveness of an alpha-beta-gamma filter on position/velocity/acceleration (0 = very responsive/noisy, 1 = very stable/slow) [FM]; the task then runs a 2nd-order filter (1 Hz, damping 0.707) and generates an orbit trajectory limited by jerk 4 m/s3, acceleration 2 m/s2 and tangential velocity `FLW_TGT_MAX_VEL` **[src FlightTaskAutoFollowTarget.hpp]**. The resulting setpoint is position + velocity + acceleration feed-forward into the MC position controller, so section 3 limits apply.

| Parameter [PR] | Default (main = 1.16 = 1.14) | Range | Meaning |
|---|---|---|---|
| FLW_TGT_HT | 8.0 m | >= 8.0 | Follow height **relative to home/arming position**, not the target [FM] |
| FLW_TGT_DST | 8.0 m | >= 1.0 | Horizontal separation; docs recommend > 12 m because of 3-5 m GPS bias between target and drone [FM] |
| FLW_TGT_FA | 180.0 deg | -180..180 | Follow angle vs target course, clockwise positive; 180 = behind; wrapped if out of range |
| FLW_TGT_ALT_M | 0 | 0/1/2 | 0 = 2D (fixed alt vs home), 1 = 2D + terrain (needs distance sensor), 2 = 3D track target GPS alt |
| FLW_TGT_MAX_VEL | 5.0 m/s (page text says "10 m/s sweet spot") | 0..20 | Max tangential velocity of the orbit around the target |
| FLW_TGT_RS | 0.1 | 0..1 | Estimator responsiveness (clamped to >= 0.1 in code **[src]**) |
| MC limits | see section 3 | | MPC_XY_VEL_MAX etc. still cap the result |

RC sticks adjust height/distance/angle live (2 m/s, 1.5 m/s, 1.5 rad/s rates **[src]**), values discarded on exit [FM]. No obstacle avoidance in this mode [FM].

**Pre-1.14 vs 1.14+.** v1.13 implements Follow Me in the navigator with parameters `NAV_FT_DST` (8 m), `NAV_MIN_FT_HT` (8 m), `NAV_FT_FS` (0 front-right, 1 behind, 2 front, 3 front-left; default 1), ignores target altitude, "pauses to assess the radio link" before following, and requires GPS [FM13]. v1.14 moved it to a flight task + target estimator with the `FLW_TGT_*` set, continuous follow angle, three altitude modes, RC adjustment and the stated local-position-only requirement [FM v1.14]. `NAV_FT_*` no longer exist in the 1.14+ parameter reference [PR]. 1.15 and 1.16 pages are textually identical to main apart from the MAVSDK-duplicate warning being dropped in 1.16.

---

## 2. Offboard mode

**Entry** [OB]: a continuous **2 Hz "proof of life"** stream of any supported setpoint message must already be running before arming in Offboard or switching to it in flight; mode requirements (MC) are `mode_req_angular_velocity`, `mode_req_attitude`, `mode_req_offboard_signal`; manual control is disabled except for mode changes; `COM_RC_IN_MODE=4` allows flight without any RC. Position/velocity/acceleration setpoints additionally require a valid local position / local velocity / attitude estimate respectively ("Offboard requires local position/velocity") **[src commander/HealthAndArmingChecks/checks/offboardCheck.cpp]**. Switch with `MAV_CMD_DO_SET_MODE` `base_mode=1`, `custom_mode=6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)`, `sub_mode=0` **[src]**; the heartbeat the vehicle reports back carries `custom_mode = (sub_mode<<24)|(main_mode<<16)` **[src px4_custom_mode.h union]**. All incoming `SET_POSITION_TARGET_*` are dropped unless `MAV_FWDEXTSP=1` (default Enabled, [PR]); when not in Offboard the receiver still publishes the proof-of-life (`offboard_control_mode`) but discards the trajectory setpoint **[src mavlink_receiver.cpp]**, so the stream can (and must) start before the switch.

**SET_POSITION_TARGET_LOCAL_NED (84)** [MAV; OB]: `type_mask` bits = 1 x, 2 y, 4 z, 8 vx, 16 vy, 32 vz, 64 ax, 128 ay, 256 az, 512 FORCE_SET, 1024 yaw, 2048 yaw_rate (set bit = ignore). Supported MC combinations: position only; velocity only; acceleration only; position + velocity (velocity = feed-forward added to the position-controller output); position + velocity + acceleration (acceleration = feed-forward added to the velocity-controller output, used to build the thrust vector) [OB]. **Velocity + acceleration without position is also accepted** (acceleration is feed-forward to the velocity loop; this is the ROS 2 TrajectorySetpoint rule "velocity setpoint: non-NaN acceleration is feed-forward" [OB]). FORCE_SET is rejected ("force not supported") **[src]**. Frames: only `MAV_FRAME_LOCAL_NED` (1) and `MAV_FRAME_BODY_NED` (8); in BODY_NED only velocity/acceleration are used, rotated by the vehicle yaw (heading only, not full attitude), position is forced to NaN [OB] **[src]**; `LOCAL_OFFSET_NED` (7) and `BODY_OFFSET_NED` (9) produce "coordinate frame unsupported" **[src]**. Yaw: `yaw` (rad, NED heading) and `yaw_rate` (rad/s) are passed through when their ignore bits are clear; the docs do not say what happens when both are ignored (not in docs; expected: hold last yaw — verify, see open questions). Recommended path-B mask for velocity + acceleration + yaw: `type_mask = 1|2|4|2048 = 0x807` (ignore position and yaw_rate).

| Parameter [PR] | main | 1.16 | 1.14 | Meaning |
|---|---|---|---|---|
| COM_OF_LOSS_T | 1.0 s (0..60) | 1.0 | 1.0 | Timeout without setpoint/OffboardControlMode before the offboard-loss action [OB, SF] |
| COM_OBL_RC_ACT | 0 | 0 | 0 | Action: 0 Position, 1 Altitude, 2 Stabilized (1.14: "Manual"), 3 Return, 4 Land, 5 Hold, 6 Terminate, 7 Disarm. Falls back to Hold instead of a manual mode if manual control is also lost [SF] |
| COM_RCL_EXCEPT | 0 (bitmask 0..31) | 0 | 0 | main: bit0 Mission, bit1 **Auto modes** (Hold, Takeoff, Land, Return, Descend, **Follow Target**, Precland, Orbit), bit2 Offboard, bit3 External, bit4 Altitude Cruise. 1.14/1.16: bit0 Mission, bit1 Hold, bit2 Offboard |
| COM_DLL_EXCEPT | 0 (main only) | missing | missing | Same bit layout for GCS-link loss |
| COM_RC_IN_MODE | 3 | 3 | 3 | 4 = disable manual control (no RC needed) [OB] |
| MAV_FWDEXTSP | 1 | — | — | Must be 1 or external setpoints are ignored **[src]** |
| COM_ARM_WO_GPS | 1 | 1 | 1 | main: 0 deny / 1 allow with warning / 2 allow silently; 1.14: 0 require GPS / 1 allow [PR, SF] |
| MAN_OVERRIDE_SPD | 1 (v1.18) | — | — | Stick motion speed that kicks the vehicle out of Offboard into Position mode; -1 disables [OB] |

**Offboard-loss failsafe** [OB, SF]: after `COM_OF_LOSS_T` PX4 leaves Offboard and performs `COM_OBL_RC_ACT`; with RC lost as well, a manual fallback becomes Hold. Note `COM_FAIL_ACT_T` (5 s hold before Return/Land) applies to "most failsafes" [SF]; whether it applies to offboard loss is not in docs.

**Mode numbers for DO_SET_MODE** (`param1=1`, `param2=main`, `param3=sub`) **[src px4_custom_mode.h, Commander.cpp]** — not in docs: Position 3/0, Offboard 6/0, Hold (Loiter) 4/3, Mission 4/4, Return 4/5, Land 4/6, Takeoff 4/2, Follow Target 4/8, Precland 4/9. Land is "always allowed" (forced) so it can serve as an emergency mode **[src]**. PX4 >= 1.15 also implements the MAVLink Standard Modes protocol (AVAILABLE_MODES/CURRENT_MODE) for discovery [DEV].

---

## 3. Multicopter position-controller limits

| Parameter [PR] | main | 1.16 | 1.14 | Applies to |
|---|---|---|---|---|
| MPC_TILTMAX_AIR | 45 deg (20..89) | 45 | 45 | "Absolute maximum for all velocity or acceleration controlled modes" — Offboard, Follow Me, Position, Auto. Direct pitch-transient cap. 1.14 text: "AUTO and POSCTRL" |
| MPC_XY_VEL_MAX | 12 m/s (0..20) | 12 | 12 | "Absolute maximum for all velocity controlled modes, any higher value truncated" — all modes |
| MPC_XY_CRUISE | 5 m/s (3..20) | 5 | 5 | Default speed in autonomous modes (Mission, Return, Goto) — not Offboard velocity |
| MPC_ACC_HOR | 3 m/s2 (2..15) | 3 | 3 | Acceleration for autonomous modes and manual acceleration-based mode. Orbit/trajectory generation in Auto; whether the MC position controller clamps Offboard velocity setpoints with it is not in docs |
| MPC_ACC_HOR_MAX | 5 m/s2 (2..15) | 5 | 5 | main: "With MPC_POS_MODE=0 it limits deceleration; with 4 unused". Manual only; used by the geofence margin formula v2/2a [SF] |
| MPC_JERK_AUTO | 4 m/s3 (1..80) | 4 | 4 | Autonomous-mode jerk limit (trajectory generator, Mission/Return) [JT] |
| MPC_JERK_MAX | 8 m/s3 (0.5..500) | 8 | 8 | Position/Altitude manual mode only, acceleration-based variant [JT, PR] |
| MPC_YAWRAUTO_MAX | 60 deg/s (5..360) | 45 | 45 | Max yaw-setpoint rate in autonomous modes (Follow Me yaw-to-target goes through it **[src]**) |
| MPC_YAWRAUTO_ACC | 20 deg/s2 | 60 | missing | Yaw-setpoint acceleration limit, autonomous modes |
| MPC_POS_MODE | 4 (0 direct velocity / 4 acceleration based) | 4 | 4 (0/3/4) | Manual Position mode only [POS]; collision prevention needs it [CP] |
| MPC_XY_VEL_P_ACC / I_ACC / D_ACC | 1.8 / 0.4 / 0.2 | | | Velocity-loop gains — the actual pitch response to a velocity step in Offboard |
| MPC_XY_P | 0.95 | | | Position-loop gain |
| MPC_Z_VEL_MAX_UP / DN | 3.0 / 1.5 m/s | 3.0/1.5 | 3.0/1.0 (POS page says 1) | "For manually controlled modes and offboard" (per MPC_Z_V_AUTO_* text) |
| MPC_Z_V_AUTO_UP / DN | 3.0 / 1.5 m/s | | | Autonomous modes |
| MPC_ACC_UP_MAX / DOWN_MAX | 4 / 3 m/s2 | | | Climb-rate controlled modes |
| SYS_VEHICLE_RESP, MPC_XY_VEL_ALL | -0.4, -10 | | | Master knobs; > 0 overrides the individual limits above |

Which apply where [JT, PR]: Follow Me is an autonomous flight task, so `MPC_XY_VEL_MAX`, `MPC_TILTMAX_AIR`, `MPC_YAWRAUTO_MAX/ACC` apply and the orbit generator uses its own 4 m/s3 / 2 m/s2 constants plus `FLW_TGT_MAX_VEL` **[src]**; `MPC_XY_CRUISE`/`MPC_JERK_AUTO`/`MPC_ACC_HOR` are documented for Mission/Return trajectory generation [JT], their effect on Follow Me and Offboard is not in docs. Offboard velocity mode is only documented to be capped by `MPC_XY_VEL_MAX` and `MPC_TILTMAX_AIR` ("all velocity or acceleration controlled modes"); no jerk/acceleration smoothing is documented for raw Offboard velocity setpoints — SentAI must shape its own velocity/acceleration profile (path B) or rely on the orbit generator (path A). Practical pitch tamer in both paths: lower `MPC_TILTMAX_AIR` (e.g. 25-30 deg) and `MPC_XY_VEL_MAX`.

---

## 4. Safety

**Failsafe actions** (increasing severity): None, Warning, Hold, Return, Land, Disarm, Flight termination; "if multiple failsafes are triggered, the more severe action is taken"; most failsafes first hold for `COM_FAIL_ACT_T` (5.0 s, 0..25) during which sticks cannot override [SF].

| Parameter [PR] | main default | 1.16 | 1.14 | Notes |
|---|---|---|---|---|
| COM_RC_LOSS_T | 0.5 s | 0.5 | 0.5 | Manual-control loss timeout |
| NAV_RCL_ACT | 2 (Return) | 2 | 2 | 1 Hold, 2 Return, 3 Land, 5 Terminate, 6 Disarm, 7 Hold (no failsafe, main only). Not triggered when COM_RC_IN_MODE disables RC checks (1.14/1.16 text) |
| COM_DL_LOSS_T | 10 s (5..300) | 10 | 10 | GCS connection loss timeout |
| NAV_DLL_ACT | 0 (Disabled) | 0 | 0 | 1 Hold, 2 Return, 3 Land, 5 Terminate, 6 Disarm |
| COM_RCL_EXCEPT / COM_DLL_EXCEPT | 0 / 0 | 0 / — | 0 / — | See section 2 |
| GF_ACTION | 2 (Hold) | | | 0 None, 1 Warning, 2 Hold, 3 Return, 4 Terminate (needs CBRK_FLIGHTTERM=0), 5 Land |
| GF_MAX_HOR_DIST / GF_MAX_VER_DIST | 0 / 0 m (disabled) | 0/0 | 0/0 | Cylinder around **home**; fence triggers only after breach — add margin v2/2a [SF] |
| GF_SOURCE | 0 (GPOS) / 1 (GPS) | | | |
| GF_PREDICT | missing in main | 0 | 0 | Experimental pre-emptive breach |
| GF_COUNT / GF_ALTMODE | missing | missing | -1 / 0 | 1.14 only |
| RTL_TYPE | 0 (home/rally direct) | 0 | 0 | 1 mission landing/rally, 2 mission path, 3 closest safe [RTL] |
| RTL_RETURN_ALT | 60 m | 60 | 60 | Min altitude above destination; climbs unless already higher |
| RTL_DESCEND_ALT | 30 m | 30 | 30 | Loiter/descend altitude before landing |
| RTL_LAND_DELAY | 0.0 s in PR main (RTL page text says 0.5 s) | | | -1 = loiter instead of landing |
| RTL_MIN_DIST / RTL_CONE_ANG | 10 m / 45 deg | | | Cone: within RTL_MIN_DIST the return altitude is lower; 0 = no cone, 90 = at least RTL_DESCEND_ALT |
| COM_LOW_BAT_ACT | 0 (Warning) | 0 | 0 | 2 Land, 3 Return then Land (recommended [SF]), 4 Return then Terminate (main) |
| COM_POS_FS_EPH / EKF2_NOAID_TOUT | 5 m / 5 s | 5 / 5 | 5 / 5 | Position-loss failsafe: autonomous modes try Return, then Land, then Descend [SF] |
| COM_FLT_TIME_MAX / COM_WIND_MAX | -1 / -1 | | | Flight-time and wind limits feed `mode_req_wind_and_flight_time_compliance`, which Follow Me requires [FM] |
| CBRK_FLIGHTTERM | 121212 (termination disabled) | | | |
| CP_DIST | -1 m (disabled) | | | Collision prevention: **"Only used in Position mode"** (acceleration-based manual Position mode / VTOL in MC) [CP]; it does not act in Offboard, Follow Me or Mission. CP_DELAY 0.4 s, CP_GUIDE_ANG 30 deg, CP_GO_NO_DATA 0; 5 s without range data switches to Hold [CP] |

**What "datalink" means for a companion on TELEM2.** The data-link-loss failsafe "is triggered if the connection to the last MAVLink ground station like QGroundControl is lost" [SF]; the docs do not define "ground station" further (not in docs). **[src mavlink_receiver.cpp handle_message_heartbeat, Commander.cpp]**: a HEARTBEAT with `type == MAV_TYPE_GCS (6)` from **any** sysid refreshes the GCS timer and arms the `COM_DL_LOSS_T`/`NAV_DLL_ACT` logic; a HEARTBEAT with `type == MAV_TYPE_ONBOARD_CONTROLLER (18)` counts **only if its sysid equals the vehicle's `MAV_SYS_ID` (default 1)**, is tracked separately, and its loss after a fixed 5 s produces only the critical message "Connection to mission computer lost" with **no failsafe action**. Consequently: if SentAI sends `type=18`, sysid 1, compid 191 (MAV_COMP_ID_ONBOARD_COMPUTER), it is not the GCS, `NAV_DLL_ACT` never fires on its behalf and the offboard-loss timeout `COM_OF_LOSS_T` is the only link watchdog in path B; in path A there is no watchdog at all beyond the 3 s stale flag (hover). If SentAI sends `type=6` it becomes the GCS and `COM_DL_LOSS_T`/`NAV_DLL_ACT` apply to it. TELEM2 defaults: `MAV_1_CONFIG=102 (TELEM 2)`, `MAV_1_MODE=2 (Onboard)`, `MAV_1_RATE=0` (half the theoretical link rate), `MAV_1_FORWARD=0`, `SER_TEL2_BAUD=921600` [MP, CC]; the main parameter reference shows `MAV_1_CONFIG` default 0 (board-specific defaults set it) [PR]. The Onboard stream set includes ATTITUDE 100 Hz, LOCAL_POSITION_NED 30 Hz, GLOBAL_POSITION_INT 50 Hz, ALTITUDE 10 Hz, DISTANCE_SENSOR 10 Hz, TIMESYNC 10 Hz, HOME_POSITION 0.5 Hz, EXTENDED_SYS_STATE 5 Hz, ATTITUDE_QUATERNION 50 Hz **[src mavlink_main.cpp streams_mode_onboard]**; rates can be changed with `MAV_CMD_SET_MESSAGE_INTERVAL` (511) [MP].

**DO_SET_MODE vs NAV_* commands.** `MAV_CMD_DO_SET_MODE` (176) selects any mode by number (section 2). `MAV_CMD_NAV_TAKEOFF` (22), `MAV_CMD_NAV_LAND` (21) and `MAV_CMD_NAV_RETURN_TO_LAUNCH` (20) are accepted as **direct mode changes** (Takeoff, Land, Return) when sent as commands to the commander **[src Commander.cpp]**; the docs only mention takeoff "commanded from a ground station without coordinates" [TKO]. `MAV_CMD_DO_REPOSITION` (192) is a guided go-to (not covered here).

---

## 5. Arming and takeoff from the companion

- `MAV_CMD_COMPONENT_ARM_DISARM` (400): `param1` 1 arm / 0 disarm, `param2 = 21196` forces (skips checks) [MAV]; PX4 rejects any other param1 and answers TEMPORARILY_REJECTED when the transition is denied **[src Commander.cpp]**. The docs say GCS commands can pre-arm/arm/disarm and that safety can be toggled with `MAV_CMD_DO_SET_SAFETY_SWITCH_STATE` [ARM].
- `MAV_CMD_NAV_TAKEOFF` (22): `param7` = target altitude (AMSL in COMMAND_LONG; use COMMAND_INT with a relative frame); `param5/6` lat/lon optional (0/NaN = current position); `param4` yaw is fixed-wing only (MC yaw is left to the flight task) **[src navigator_main.cpp]**. Without a valid altitude the navigator uses `MIS_TAKEOFF_ALT` (2.5 m) [TKO, PR]; climb rate `MPC_TKO_SPEED` 1.5 m/s, ramp `MPC_TKO_RAMP_T` 3 s, tilt during ramp `MPC_TILTMAX_LND` 12 deg [PR]. After reaching altitude the vehicle holds; `COM_TAKEOFF_ACT` (0 Hold / 1 Mission, 1.14/1.16, missing in main) [PR]. Takeoff mode needs `mode_req_local_alt` + `mode_req_local_position`; a disarmed vehicle can select it without a position estimate but cannot arm [TKO].
- Follow Me cannot be armed into (`mode_req_prevent_arming`) [FM]; Offboard can — the stream must be running first [OB]. Recommended sequence: Hold or Position selected → stream setpoints → ARM → NAV_TAKEOFF (or arm in Offboard and climb with a velocity setpoint) → DO_SET_MODE Offboard / Follow Target.
- Relevant pre-arm parameters [ARM, SF, PR]: `COM_ARMABLE` 1, `COM_ARM_WO_GPS` 1 (allow w/ warning), `COM_ARM_MIS_REQ` 0, `COM_ARM_AUTH_REQ` 0, `COM_ARM_BAT_MIN` -1 (uses BAT_CRIT_THR), `COM_ARM_MAG_ANG` 60 deg, `COM_ARM_MAG_STR` 2 (warn), `COM_ARM_IMU_ACC` 0.7 m/s2, `COM_ARM_IMU_GYR` 0.25 rad/s, `COM_ARM_SDCARD` 1 (warn; missing in main PR), `COM_ARM_HFLT_CHK` 1, `COM_CPU_MAX` 95 %, `COM_ARM_EKF_POS/VEL/HGT/YAW` 0.5/0.5/1.0/0.5 (1.14 only), `COM_DISARM_PRFLT` 10 s (auto-disarm if no takeoff), `COM_DISARM_LAND` 2 s, `COM_SPOOLUP_TIME` 2.0 s main / 1.0 s 1.14-1.16, `CBRK_USB_CHK` 197848 (default already allows flying with USB connected), `COM_PREARM_MODE` 0, `EKF2_REQ_GPS_H` 10 s, `EKF2_REQ_EPH/EPV/SACC/NSATS/PDOP` 3 m / 5 m / 0.5 m/s / 6 / 2.5 (1.14/1.16; moved out of main PR). `COM_ARM_CHK_ESCS` 0.

---

## Recommended PX4 parameter set for the follow-me vehicle

| Parameter | Value | Why |
|---|---|---|
| MAV_1_CONFIG / MAV_1_MODE / SER_TEL2_BAUD | 102 / 2 (Onboard) / 921600 (or the highest the RT1176 UART sustains) | Companion stream on TELEM2 [MP, CC] |
| MAV_PROTO_VER | 2 | MAVLink v2 (main default 2; 1.14/1.16 default 0 = auto) [PR] |
| MAV_FWDEXTSP | 1 | Required for Offboard setpoints **[src]** |
| COM_RC_IN_MODE | 3 (RC + joystick) — keep an RC for safety; 4 only for RC-less tests | [OB] |
| COM_OF_LOSS_T | 0.5-1.0 s | Fast reaction to companion stall [OB] |
| COM_OBL_RC_ACT | 5 (Hold) for first flights, 3 (Return) outdoors with GNSS | Deterministic action, not a manual mode [SF] |
| COM_RCL_EXCEPT | 0 | Keep RC-loss failsafe active even in Offboard/Follow Me |
| NAV_RCL_ACT / COM_RC_LOSS_T | 2 (Return) / 0.5 s | Pilot RC is the last line of defence [SF] |
| NAV_DLL_ACT / COM_DL_LOSS_T | 1 (Hold) / 10 s if a real GCS (QGC) is on TELEM1; 0 if none | A companion with type 18 does not count as GCS **[src]** |
| GF_MAX_HOR_DIST / GF_MAX_VER_DIST / GF_ACTION | test-field radius (e.g. 100 m) / 30 m / 2 (Hold) | Breach-after-the-fact; add v2/2a margin [SF] |
| RTL_RETURN_ALT / RTL_DESCEND_ALT / RTL_CONE_ANG / RTL_LAND_DELAY | 20 m / 10 m / 45 / 0 | Low follow heights make the 60/30 m defaults excessive [RTL] |
| COM_LOW_BAT_ACT | 3 (Return then Land) | docs recommendation [SF] |
| FLW_TGT_HT / FLW_TGT_DST / FLW_TGT_FA / FLW_TGT_ALT_M | 8 m (minimum) / 8-10 m (vision has no 3-5 m GPS bias, but keep > min) / 180 / 0 | [FM] |
| FLW_TGT_MAX_VEL / FLW_TGT_RS | 3-5 m/s / 0.1-0.3 | Walking pilot; lower orbit speed = gentler pitch |
| MPC_TILTMAX_AIR | 25-30 deg | Direct pitch-transient cap for Offboard and Follow Me [PR] |
| MPC_XY_VEL_MAX / MPC_XY_CRUISE | 4 m/s / 3 m/s | Walking-speed envelope |
| MPC_ACC_HOR / MPC_JERK_AUTO | 2 m/s2 / 2 m/s3 | Smoother autonomous trajectories [JT] |
| MPC_YAWRAUTO_MAX / MPC_YAWRAUTO_ACC | 45 deg/s / 20 deg/s2 | Keep the person in the camera FOV without yaw jerks |
| MPC_Z_VEL_MAX_UP / DN, MPC_TKO_SPEED | 1.5 / 1.0 / 1.0 m/s | Gentle vertical envelope |
| MIS_TAKEOFF_ALT | 8 m (= FLW_TGT_HT) | Follow Me climbs to FLW_TGT_HT anyway |
| COM_DISARM_PRFLT / COM_DISARM_LAND | 30 s / 2 s | Allow a slow companion takeoff handshake |
| COM_ARM_WO_GPS | 0 (deny) outdoors | Follow Me/Return need a global origin **[src]** |
| CP_DIST | -1 | Collision prevention cannot help in Offboard/Follow Me anyway [CP] |

---

## Open questions to verify on the vehicle

1. Firmware version actually flashed (1.14 / 1.15 / 1.16 / main): `COM_RCL_EXCEPT` bit meaning, `MPC_YAWRAUTO_MAX` default (45 vs 60) and `MAV_PROTO_VER` default differ between versions.
2. Path A: confirm with a logged `follow_target_estimator`/`follow_target_status` that lat/lon/alt + vel are the only consumed fields and that `vel = (0,0,0)` is fused as zero; measure the behaviour when the stream stops (expect hover at last estimate after 3 s, no mode change).
3. Path A: does Follow Me accept the mode switch with a GNSS fix but `COM_ARM_WO_GPS=1` and a drifting origin; verify the "requires home altitude" refusal path and the 1 m emergency-ascent trigger with/without a distance sensor.
4. Path B: measured behaviour when `yaw` and `yaw_rate` are both ignored in SET_POSITION_TARGET_LOCAL_NED (yaw hold vs. drift); whether `MPC_ACC_HOR`/`MPC_JERK_AUTO` have any effect on raw velocity setpoints (docs silent) — step a 1 m/s velocity setpoint and log pitch.
5. Path B: effective proof-of-life margin at 57600 vs 921600 baud with the full Onboard stream running; confirm `COM_OF_LOSS_T` triggers only on the setpoint stream and not on heartbeat loss.
6. Does `COM_FAIL_ACT_T` (5 s hold) apply to the offboard-loss action (docs: "most failsafes").
7. Heartbeat identity: confirm that SentAI with `type=18 ONBOARD_CONTROLLER`, sysid = `MAV_SYS_ID`, compid 191 does not arm the GCS-loss failsafe, and decide whether a separate GCS (QGC) must be present for `NAV_DLL_ACT` to protect the flight.
8. `MAV_CMD_NAV_TAKEOFF` via COMMAND_LONG: verify the altitude frame actually used (AMSL per navigator source) versus COMMAND_INT with `MAV_FRAME_GLOBAL_RELATIVE_ALT`.
9. Whether the RTL cone/`RTL_MIN_DIST` interplay at 8 m follow height yields an acceptable return altitude for the chosen test field.
10. Collision prevention: confirm it is inactive in Offboard/Follow Me on the installed version (docs: Position mode only), so SentAI's own stand-off distance logic is the only obstacle guard.

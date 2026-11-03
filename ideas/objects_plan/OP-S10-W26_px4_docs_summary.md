# OP-S10-W26 — What we can use from PX4 (documentation summary)

**WBS**: OP-S10-W26-T2
**Date**: 2026-11-03
**Sources**: two detailed memos with per-statement URLs, kept next to this file: `OP-S10-W26_px4_control_interface_memo.md` (Follow Me, Offboard, MPC limits, safety, arming) and `OP-S10-W26_px4_telemetry_time_memo.md` (TELEM2 setup, stream rates, TIMESYNC, AGL, EKF2, logging, heartbeat). Facts the docs do not state were taken from the PX4 source and are marked **[src]**. Versions: docs `main` (v1.17-dev) with v1.14 / v1.15 / v1.16 differences noted. Design context: chapter 23 (vision only, SentAI is the brain, PX4 provides attitude + position + GPS).

## 1. The two control paths and what PX4 imposes on each

| | Path A — `FOLLOW_TARGET` → PX4 Follow Me | Path B — Offboard velocity/acceleration setpoints |
|---|---|---|
| What SentAI sends | lat/lon/alt(AMSL) + velocity of the pilot, 2–5 Hz | `SET_POSITION_TARGET_LOCAL_NED` at 20–50 Hz (≥ 2 Hz proof of life before the mode switch) |
| What PX4 does | target estimator (alpha-beta-gamma, `FLW_TGT_RS`) → 2nd-order filter → orbit trajectory (jerk 4 m/s³, accel 2 m/s², `FLW_TGT_MAX_VEL`) → position controller; yaw always toward the target | velocity loop + optional acceleration feed-forward → thrust vector; no trajectory smoothing documented for raw velocity setpoints |
| Fields consumed **[src]** | only `lat, lon, alt, vel[3]`; `est_capabilities`, `acc`, `attitude`, `timestamp`, `position_cov` ignored; **`vel = 0` is fused as a real zero**, so SentAI must send its estimated pilot velocity | `type_mask` bits: 1 x, 2 y, 4 z, 8 vx, 16 vy, 32 vz, 64 ax, 128 ay, 256 az, 1024 yaw, 2048 yaw_rate (set = ignore). Velocity + acceleration + yaw = `0x807` |
| Frames | WGS84, alt AMSL (QGC-Android's ellipsoid height is the known trap) | **only `LOCAL_NED` (1) and `BODY_NED` (8)**; `LOCAL_OFFSET_NED` (7) and `BODY_OFFSET_NED` are rejected **[src]** → the SIM bridge's `cmd_move` (frame 7) must be rewritten as LOCAL_NED with the current pose added (s104 used LOCAL_NED and worked) |
| Rates / timeouts **[src]** | position fused at most every 500 ms, velocity every 100 ms; stale after 3 s (velocity zeroed, position kept); flight task filter reset after 1.5 s | `COM_OF_LOSS_T` 1.0 s without a setpoint → `COM_OBL_RC_ACT` |
| Height | **`FLW_TGT_HT` minimum 8 m above home** (parameter range); climbs to 1 m, then to within 3 m of `FLW_TGT_HT` before moving horizontally | free (we choose 3–5 m) |
| Distance | `FLW_TGT_DST` ≥ 1 m (default 8); `FLW_TGT_FA` follow angle (180 = behind the pilot's course) | our standoff logic |
| Loss of target | hovers at the last estimate; **no Hold/RTL on target loss** | leaving Offboard after `COM_OF_LOSS_T` |
| Needs | global origin (GNSS) even though the docs say "local position" **[src]**; cannot be armed into the mode | local position + attitude valid; can arm in Offboard; `MAV_FWDEXTSP = 1` |
| Mode numbers `DO_SET_MODE` **[src]** | base 1, main 4, sub 8 | base 1, main 6, sub 0 (Hold 4/3, Return 4/5, Land 4/6, Takeoff 4/2, Position 3/0) |

**Consequences for chapter 23.** Path A is the quickest flight but it is a slow, high follower: 8 m minimum height puts the pilot at ≈ 11 m slant range at 8 m standoff (≈ 75 px tall at VGA, ≈ 38 px at 320×240, with the camera looking 45° down), the target position is refreshed at most every 0.5 s, and it is a mode switch away from "hover forever" when SentAI stops sending. Path B lets us fly at 3–5 m and react at 50 Hz, with `COM_OF_LOSS_T` as a real watchdog. Keep T0 (path A) as the baseline, plan the deliverable on path B.

## 2. Telemetry SentAI consumes

TELEM2 defaults: `MAV_1_CONFIG = 102`, `MAV_1_MODE = 2 (Onboard)`, `MAV_1_RATE = 0` (= baud/20 = 46 080 B/s at 921 600), `MAV_1_FORWARD = 0`, `SER_TEL2_BAUD = 921600`; identical in 1.14 / 1.15 / main. The parameter reference lists `MAV_1_CONFIG` default 0 ("disabled to save RAM") while board defaults usually set 102: verify with `mavlink status`.

Onboard-mode default stream rates **[src, identical 1.14/1.16/main]**: TIMESYNC 10 Hz, HIGHRES_IMU 50, ATTITUDE 100, ATTITUDE_QUATERNION 50, LOCAL_POSITION_NED 30, ODOMETRY 30, GLOBAL_POSITION_INT 50, ALTITUDE 10, DISTANCE_SENSOR 10, ESTIMATOR_STATUS 1, HOME_POSITION 0.5, GPS_GLOBAL_ORIGIN 1, SYSTEM_TIME 1, debug streams 10.

| Message | Use | Timestamp | Rate we request |
|---|---|---|---|
| **ODOMETRY (331)** | attitude quaternion + angular velocity + position + velocity, LOCAL_NED / BODY_FRD, covariances, `reset_counter` | **`time_usec` = EKF `timestamp_sample` (µs)** — the only attitude message with a µs sample time | 100 Hz via `SET_MESSAGE_INTERVAL(331, 10000)`; 244 B on the wire → 24 kB/s, inside the 46 kB/s cap |
| ATTITUDE (30) | fallback attitude + rates | `time_boot_ms` = publication time / 1000 (1 ms quantisation, not the sample time) | default 100 Hz |
| LOCAL_POSITION_NED (32) | position, velocity (no AGL, no attitude) | `time_boot_ms` | 30–50 Hz |
| GLOBAL_POSITION_INT (33) | lat/lon/alt, `relative_alt` above home, `hdg` | `time_boot_ms` | 10 Hz |
| ALTITUDE (141) | `bottom_clearance` = `dist_bottom` **only when a rangefinder makes it valid, else NaN**; `altitude_relative` = height above take-off | `time_usec` = now | 10 Hz |
| HOME_POSITION (242), GPS_GLOBAL_ORIGIN (49) | local-frame origin for local ↔ lat/lon (azimuthal equidistant projection) | | default |
| ESTIMATOR_STATUS (230) | quality gate: flags ATTITUDE 1, POS_HORIZ_REL 8, POS_VERT_ABS 32, POS_VERT_AGL 64; innovation ratios should be < 0.5 | `time_usec` | 5 Hz |
| HIGHRES_IMU (105) | bias-corrected gyro at 50 Hz if a raw rate is ever needed | `time_usec` sample | optional |
| TIMESYNC (111), SYSTEM_TIME (2) | clock sync (§3) | | PX4 requests at 10 Hz |

`SET_MESSAGE_INTERVAL`: param2 = interval in µs, −1 removes a stream, 0 restores the mode default; PX4 rejects it for HEARTBEAT and whenever param3/4/7 are non-zero **[src]**; a stream cannot exceed its uORB publisher rate and the whole instance is scaled by `rate mult` (0.05…1) if the byte budget is exceeded (`mavlink status` shows it).

## 3. Time synchronisation (replaces STROBE)

- Protocol: requester sends `tc1 = 0, ts1 = own clock (ns)`; responder mirrors `ts1` and sets `tc1 = own clock (ns)`, targeted to the requester (main; v1.14 broadcasts). Both fields are **nanoseconds**.
- PX4 streams TIMESYNC requests at 10 Hz in Onboard mode. SentAI must answer them (so PX4 can stamp anything we ever send in ODOMETRY/VISION/LANDING_TARGET form) **and** run its own requester for its clock: `offset = (t1 + t4)/2 − tc1`, keep min-RTT samples, drop RTT > 10 ms (PX4's own rule), double-exponential smoothing. PX4 logs `timesync_status` (observed/estimated offset, RTT) at 1 Hz.
- Accuracy over the UART is not in the docs. A 25-byte frame at 921 600 takes ≈ 0.27 ms each way; PX4 stamps receive and reply with one `now`, so its scheduling latency is the asymmetric error. Measure the RTT distribution on the vehicle (T14, s271).
- Camera side: `g_cam_buf_ts_us` (GPT1 µs at frame start) + offset → PX4 time → interpolate ODOMETRY attitude at that instant from the pose ring.
- `time_boot_ms` wraps after 49.7 days; `hrt_absolute_time()` is µs since boot.

## 4. Height above ground for the ground-plane projection

| Vehicle has | AGL source | Notes |
|---|---|---|
| downward rangefinder | `ALTITUDE.bottom_clearance` (= `vehicle_local_position.dist_bottom`), terrain estimator (`EKF2_RNG_*`, `EKF2_TERR_*`) | valid only when `dist_bottom_valid`; `EKF2_RNG_CTRL` 1 = conditional (below 1 m/s, 5 m), 2 = always |
| none | `ALTITUDE.altitude_relative` or `−z` of LOCAL_POSITION_NED relative to take-off | flat-ground assumption; the pilot stands on the take-off ground, so this is the natural choice |

`EKF2_HGT_REF = 2` (range) makes the local origin follow the ground; keep the default (GNSS) so the local frame stays fixed.

## 5. Commands SentAI sends

| Command / message | Fields | Notes |
|---|---|---|
| HEARTBEAT 1 Hz | sysid = `MAV_SYS_ID` (1), compid 191 `ONBOARD_COMPUTER`, type 18 `ONBOARD_CONTROLLER` | PX4 registers it only with the vehicle's sysid (or type GCS); **never type 6 GCS** |
| `MAV_CMD_SET_MESSAGE_INTERVAL` (511) | id, interval µs | §2 |
| `MAV_CMD_COMPONENT_ARM_DISARM` (400) | param1 1/0, param2 21196 forces | answers `COMMAND_ACK` (TEMPORARILY_REJECTED when denied) |
| `MAV_CMD_NAV_TAKEOFF` (22) | param7 altitude — **AMSL in COMMAND_LONG [src]**; use `COMMAND_INT` with a relative frame, or rely on `MIS_TAKEOFF_ALT` (2.5 m) | climb `MPC_TKO_SPEED` 1.5 m/s |
| `MAV_CMD_DO_SET_MODE` (176) | param1 1, param2 main, param3 sub (§1) | Land (4/6) is always allowed → emergency mode; `NAV_LAND` / `NAV_RETURN_TO_LAUNCH` also act as direct mode changes |
| `SET_POSITION_TARGET_LOCAL_NED` (84) | §1, frame 1 or 8 | stream before switching to Offboard; `MAV_FWDEXTSP = 1` |
| `FOLLOW_TARGET` (144) | lat/lon degE7, alt m AMSL, vel m/s NED | 2–5 Hz is enough (500 ms position throttle) |
| `NAMED_VALUE_FLOAT` (251), `DEBUG_VECT` (250), `DEBUG_FLOAT_ARRAY` (350) | name ≤ 9 chars, values | **PX4 overwrites the timestamp with receive time** → put our µs stamp inside the payload; `SDLOG_PROFILE = 33` logs them; read with `pyulog` (`ulog2csv -m debug_vect,debug_key_value,timesync_status`) |

## 6. Safety model with a companion on TELEM2

- **Companion heartbeat loss is not a failsafe.** Type-18 heartbeat loss after 5 s only logs "Connection to mission computer lost". The data-link-loss failsafe (`COM_DL_LOSS_T` 10 s, `NAV_DLL_ACT` default Disabled) keys on a GCS heartbeat only.
- **Path B watchdog** = `COM_OF_LOSS_T` (set 0.5–1 s) → `COM_OBL_RC_ACT` (5 Hold for first flights, 3 Return outdoors). `COM_FAIL_ACT_T` (5 s hold before Return/Land) applies to "most failsafes"; whether to Offboard loss is not in docs.
- **Path A has no watchdog**: if SentAI dies, PX4 hovers at the last estimate indefinitely. SentAI's own `sentai.safety` must therefore switch the mode to Hold/Return via `DO_SET_MODE` on any internal fault, and the RC pilot remains the last line (`NAV_RCL_ACT` 2 Return, `COM_RCL_EXCEPT` 0).
- Geofence: cylinder around home (`GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`, default disabled), `GF_ACTION` 2 Hold; breach is detected after the fact, add the v²/2a margin.
- RTL defaults (60 m return / 30 m descend) are excessive for 3–8 m follow heights: 20 / 10 m.
- Collision prevention (`CP_DIST`) acts **only in manual Position mode**: the minimum-distance barrier is ours in both paths.
- Position-estimate loss failsafe (`COM_POS_FS_EPH` 5 m, `EKF2_NOAID_TOUT` 5 s) covers EKF degradation; Follow Me also requires `mode_req_wind_and_flight_time_compliance` (`COM_WIND_MAX`, `COM_FLT_TIME_MAX`).

## 7. Position-controller limits that tame the pitch transients (chapter 23 §3)

| Parameter | Default | Applies to | Suggested |
|---|---|---|---|
| `MPC_TILTMAX_AIR` | 45° | "all velocity or acceleration controlled modes" = Offboard, Follow Me | 25–30° (≈ 10° would cap speed too hard; tune with the FOV budget) |
| `MPC_XY_VEL_MAX` | 12 m/s | all velocity-controlled modes | 4 m/s |
| `MPC_XY_CRUISE`, `MPC_ACC_HOR`, `MPC_JERK_AUTO` | 5, 3, 4 | documented for Mission/Return; effect on Follow Me / Offboard velocity **not in docs** | 3, 2, 2 |
| `MPC_JERK_MAX`, `MPC_ACC_HOR_MAX` | 8, 5 | manual Position mode only | — |
| `MPC_YAWRAUTO_MAX` / `_ACC` | 60 (main) / 45 (1.14, 1.16) °/s | autonomous modes incl. Follow Me yaw-to-target | 45 / 20 |
| `MPC_Z_VEL_MAX_UP/DN`, `MPC_TKO_SPEED` | 3 / 1.5 / 1.5 | manual + Offboard | 1.5 / 1.0 / 1.0 |

Raw Offboard velocity setpoints get **no documented jerk/acceleration shaping** → in path B SentAI shapes its own velocity/acceleration profile (the LQR + CBF output is already smooth; add a jerk limit on the command).

## 8. Recommended parameter set (merged)

| Parameter | Value |
|---|---|
| `MAV_1_CONFIG` / `MAV_1_MODE` / `MAV_1_RATE` / `MAV_1_FORWARD` / `MAV_1_FLOW_CTRL` | 102 / 2 Onboard / 0 / 0 / 0 (no CTS/RTS wired) |
| `SER_TEL2_BAUD` / `MAV_PROTO_VER` / `MAV_FWDEXTSP` | 921600 / 2 / 1 |
| `COM_RC_IN_MODE` | 3 (keep the RC) |
| `COM_OF_LOSS_T` / `COM_OBL_RC_ACT` | 0.5–1.0 s / 5 Hold (then 3 Return outdoors) |
| `NAV_RCL_ACT` / `COM_RC_LOSS_T` / `COM_RCL_EXCEPT` | 2 Return / 0.5 s / 0 |
| `NAV_DLL_ACT` / `COM_DL_LOSS_T` | 1 Hold / 10 s only if a real GCS is on TELEM1; else 0 |
| `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` / `GF_ACTION` | field radius (e.g. 100 m) / 30 m / 2 |
| `RTL_RETURN_ALT` / `RTL_DESCEND_ALT` / `RTL_CONE_ANG` / `RTL_LAND_DELAY` | 20 / 10 / 45 / 0 |
| `COM_LOW_BAT_ACT` | 3 Return then Land |
| `FLW_TGT_HT` / `FLW_TGT_DST` / `FLW_TGT_FA` / `FLW_TGT_ALT_M` / `FLW_TGT_MAX_VEL` / `FLW_TGT_RS` | 8 / 8 / 180 / 0 / 3 / 0.1–0.3 (path A only) |
| `MPC_TILTMAX_AIR` / `MPC_XY_VEL_MAX` / `MPC_XY_CRUISE` / `MPC_ACC_HOR` / `MPC_JERK_AUTO` | 25–30° / 4 / 3 / 2 / 2 |
| `MPC_YAWRAUTO_MAX` / `MPC_YAWRAUTO_ACC` | 45 / 20 |
| `MPC_Z_VEL_MAX_UP` / `MPC_Z_VEL_MAX_DN` / `MPC_TKO_SPEED` / `MIS_TAKEOFF_ALT` | 1.5 / 1.0 / 1.0 / per path (8 m for A, 3–5 m for B) |
| `COM_DISARM_PRFLT` / `COM_DISARM_LAND` / `COM_ARM_WO_GPS` | 30 s / 2 s / 0 outdoors |
| `SDLOG_PROFILE` / `SDLOG_MODE` | 33 / 1 |
| `CP_DIST` | −1 |

## 9. Impact on the W26 task list

- **T0**: path A needs 8 m height, 2–5 Hz `FOLLOW_TARGET` with velocity, AMSL altitude from `GLOBAL_POSITION_INT`/origin, and SentAI-side mode fallback (no PX4 watchdog).
- **T13**: subscribe ODOMETRY at 100 Hz as the primary attitude source (µs sample time); `SET_MESSAGE_INTERVAL` at init; heartbeat sysid 1 / compid 191 / type 18; `COMMAND_ACK` tracking; ESTIMATOR_STATUS gate; ODOMETRY at 100 Hz raises the RX load to ~30 kB/s, which confirms the 921 600 baud and the ring/priority fix of the link review §3.3.
- **T14**: answer PX4's 10 Hz TIMESYNC requests and run our own; RTT filter as PX4 does; log `timesync_status` + our stamp in the `DEBUG_FLOAT_ARRAY` payload.
- **T15**: `SET_POSITION_TARGET_LOCAL_NED` in **LOCAL_NED only** (the SIM bridge's frame-7 `cmd_move` is wrong for current PX4), type mask `0x807` for velocity + acceleration + yaw; takeoff via `COMMAND_INT` relative altitude; mode switch numbers from §1; own jerk limiting.
- **T16**: `COM_OF_LOSS_T` is the only PX4-side watchdog and only in path B; in path A SentAI must command Hold itself.
- **New hardware check for T12**: whether the FC has a downward rangefinder (AGL source), the firmware version (`ulog_info` `ver_sw`), and whether `MAV_1_CONFIG` is really 102.

## 10. Open questions to verify on the vehicle (merged, deduplicated)

1. Firmware version (1.14 / 1.15 / 1.16 / main): `COM_RCL_EXCEPT` bit meaning, `MPC_YAWRAUTO_MAX` default, TIMESYNC targeting (1.14 broadcasts), `MAV_PROTO_VER` default.
2. `MAV_1_CONFIG` actually 102 on the delivered board; `SER_TEL2_BAUD` 1 500 000 accepted (NuttX `B1500000`)?
3. Path A: confirm the consumed `FOLLOW_TARGET` fields and the zero-velocity fusion in the logged `follow_target_estimator`; behaviour when the stream stops (expect hover after 3 s); whether `FLW_TGT_HT` < 8 m is accepted from the shell (`param set` does not enforce ranges; the flight task reads the value directly **[src]**, to verify).
4. Path B: yaw behaviour when both yaw and yaw_rate are ignored; whether `MPC_ACC_HOR` / `MPC_JERK_AUTO` touch raw velocity setpoints (step 1 m/s and log pitch); `COM_FAIL_ACT_T` on Offboard loss.
5. TIMESYNC RTT distribution and `timesync_status.estimated_offset` jitter on the real UART.
6. ODOMETRY real maximum rate when 100 Hz is requested (EKF2 output rate bound).
7. Downward rangefinder present → `dist_bottom_valid`; otherwise AGL = `altitude_relative`.
8. Companion heartbeat visible as `heartbeat_type_onboard_controller` in the logged `telemetry_status`.
9. `NAV_TAKEOFF` altitude frame actually used with COMMAND_LONG vs COMMAND_INT.
10. RTL cone / `RTL_MIN_DIST` behaviour at 3–8 m follow heights on the chosen field.

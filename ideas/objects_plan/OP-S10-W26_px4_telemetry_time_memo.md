# PX4 telemetry, time and logging for the SentAI companion link

Scope: SentAI (i.MX RT1176, MAVLink v2 over UART on PX4 `TELEM2`) needs attitude + position interpolated at camera exposure time, height above ground, and a way to put its own scalars into the PX4 ULog. Sources are the PX4 user guide (`main`, with `v1.14` / `v1.15` checked where behaviour differs), the MAVLink common dialect, and -- where the guide is silent -- the PX4 source on `main` / `release/1.14` / `release/1.16`. Items the docs do not state are marked **not in docs** and backed by the source file that does state them.

Abbreviations: UG = `https://docs.px4.io/main/en/`, MAV = `https://mavlink.io/en/messages/common.html`, SRC = `https://github.com/PX4/PX4-Autopilot/blob/main/`.

---

## 1. Companion serial setup (TELEM2)

- PX4 "expects companion computers to connect via `TELEM2` for offboard control. The port is configured by default to interface using MAVLink" (UG `companion_computer/pixhawk_companion.html`). Ports are 3.3 V, 5 V tolerant; many companions are 1.8 V -- use a level shifter (same page). TELEM2 pin 4 = CTS (in), pin 5 = RTS (out) (same page).
- The port is bound through a *MAVLink instance* (`MAV_X_CONFIG`), then tuned with `MAV_X_MODE`, `MAV_X_RATE`, `MAV_X_FORWARD` (UG `peripherals/mavlink_peripherals.html`).

| Parameter | Default (`main`) | Values / notes | Source |
|---|---|---|---|
| `MAV_1_CONFIG` | `0` Disabled in the parameter reference; the peripherals page says TELEM2 is "usually configured by default" as `TELEM 2` (102) | 0 Disabled, 101 TELEM 1, 102 TELEM 2, 103 TELEM 3, 104 TELEM/SERIAL 4, 201-203 GPS 1-3, 1000 Ethernet | UG `advanced_config/parameter_reference.html#MAV_1_CONFIG`; `module.yaml` comment: "Telem2 Port (Companion Link). Disabled by default to reduce RAM usage" (SRC `src/modules/mavlink/module.yaml`) -- **verify on the vehicle** |
| `MAV_1_MODE` | `2` Onboard | 0 Normal, 1 Custom, 2 Onboard, 3 OSD, 4 Magic, 5 Config, 7 Minimal, 8 External Vision, 10 Gimbal, 11 Onboard Low Bandwidth, 12 uAvionix, 13 Low Bandwidth (6 Iridium, 9 ExtVisionMin, 14 distance_sensor hidden) | UG `#MAV_1_MODE`; `mavlink_main.cpp` |
| `MAV_1_RATE` | `0` | B/s cap for the instance; `0` = baudrate/20 ("half of the theoretical maximum bandwidth"; USB/UDP: 100000); hard max 10 000 000 B/s (`MAX_DATA_RATE`) | UG `#MAV_1_RATE`; `mavlink_main.cpp` |
| `MAV_1_FORWARD` | `false` | forward packets not addressed to the autopilot to other instances | UG `#MAV_1_FORWARD` |
| `MAV_1_FLOW_CTRL` | `2` Auto-detected | 0 Force off, 1 Force on, 2 Auto (auto disables HW flow control if the TX buffer stays full) | UG `#MAV_1_FLOW_CTRL`; `mavlink_main.cpp` `FLOW_CONTROL_AUTO` |
| `MAV_1_RADIO_CTL` | `1` | throttle on `RADIO_STATUS.txbuf`; harmless with no radio | UG `#MAV_1_RADIO_CTL` |
| `SER_TEL2_BAUD` | `921600` | 0 Auto, 9600 ... 115200, 230400, 460800, 500000, **921600, 1000000, 1500000, 2000000, 3000000** (8N1). Same list in v1.14 (`Tools/serial/serial_params.c.jinja`, release/1.14). 1.5/2/3 Mbaud are compiled only `#ifdef B1500000` etc. in `mavlink_open_uart()` -- NuttX board dependent, **not in docs** | UG `#SER_TEL2_BAUD`; SRC `src/modules/mavlink/mavlink_main.cpp` |
| `MAV_PROTO_VER` | `2` | 1 = v1 with auto-upgrade, 2 = v2 | UG `#MAV_PROTO_VER` |
| `MAV_SYS_ID` / `MAV_COMP_ID` | `1` / `1` | 1..250 | UG `#MAV_SYS_ID`, `#MAV_COMP_ID` |

Defaults documented for the TELEM2 instance: `MAV_1_MODE=Onboard`, `MAV_1_RATE=0` (half maximum), `MAV_1_FORWARD=Disabled`, `SER_TEL2_BAUD=921600` -- identical in `main`, `v1.14`, `v1.15` (UG `peripherals/mavlink_peripherals.html#telem2`, each version).

**What each mode streams.** The guide only names the modes ("search for `MAVLINK_MODE_` in mavlink_main.cpp", UG `peripherals/mavlink_peripherals.html`). From `mavlink_main.cpp` (`main`; rates in Hz, `-1` = every topic update):

| Stream | Normal (GCS) | Onboard | ExtVisionMin | Config (USB) |
|---|---|---|---|---|
| TIMESYNC | -- | 10 | 10 | 10 |
| HIGHRES_IMU | -- | 50 | -- | 50 |
| ATTITUDE | 15 | **100** | 20 | 50 |
| ATTITUDE_QUATERNION | 10 | **50** | -- | 50 |
| ATTITUDE_TARGET | 2 | 10 | 2 | 8 |
| LOCAL_POSITION_NED | 1 | **30** | 30 | 30 |
| GLOBAL_POSITION_INT | 5 | 50 | 5 | 10 |
| ODOMETRY | -- | **30** | 30 | 30 |
| ALTITUDE | 1 | 10 | 10 | 10 |
| DISTANCE_SENSOR | 0.5 | 10 | 10 | 10 |
| ESTIMATOR_STATUS | 0.5 | 1 | 1 | 5 |
| HOME_POSITION / GPS_GLOBAL_ORIGIN | 0.5 / 1 | 0.5 / 1 | 0.5 / 1 | 0.5 / 1 |
| SYSTEM_TIME | -- | 1 | -- | 1 |
| VFR_HUD | 4 | 10 | 4 | 20 |
| WIND_COV | 0.5 | 10 | 1 | 10 |
| DEBUG / DEBUG_VECT / NAMED_VALUE_FLOAT / DEBUG_FLOAT_ARRAY | 1 | 10 | 1 | 50 |

Version diff: the Onboard core rates above are identical in release/1.14, release/1.16 and `main`; 1.14 additionally streams COLLISION, TRAJECTORY_REPRESENTATION_WAYPOINTS, UTM_GLOBAL_POSITION; `main` adds AVAILABLE_MODES, CURRENT_MODE, OPEN_DRONE_ID_ARM_STATUS, FIGURE_EIGHT_EXECUTION_STATUS, FUEL_STATUS and caps ADSB_VEHICLE at 5 Hz (was unlimited).

**Confirming the link.** `mavlink status` prints every instance (mode, `tx rate mult`, `tx rate max`, flow control, rx/tx rates and error counters); `mavlink status streams` lists enabled streams and rates; `rate mult < 1` means PX4 is throttling because the configured streams exceed `MAV_1_RATE` (UG `modules/modules_communication.html#mavlink`). The equivalent uORB topic `telemetry_status` is logged at 1 Hz (`logged_topics.cpp`). Run-time change: `mavlink stream -d /dev/ttyS2 -s ATTITUDE -r 50` (`-r 0` off, `-r -1` default) (same page).

Gotchas: all `MAV_1_*` and `SER_TEL2_BAUD` need a reboot (UG parameter reference, "Reboot" column). With `MAV_1_RATE=0` at 921600 the cap is 46 080 B/s; the Onboard default set at full rate is well under that, so attitude at 100 Hz arrives unthrottled.

---

## 2. Message rate control

- `MAV_CMD_SET_MESSAGE_INTERVAL` (511): param1 = message id, param2 = interval in **µs**, `-1` disable, `0` "request default rate (which may be zero)", param7 response target (MAV `#MAV_CMD_SET_MESSAGE_INTERVAL`). PX4 maps interval `<0` -> stream removed, `>0` -> rate = 1e6/interval, `0` -> mode default; it **rejects** the command for HEARTBEAT and whenever param3, param4 or param7 is non-zero, and rejects unknown ids (**not in docs**; SRC `src/modules/mavlink/mavlink_receiver.cpp` `set_message_interval()`).
- `MAV_CMD_GET_MESSAGE_INTERVAL` (510) and `MAV_CMD_REQUEST_MESSAGE` (512, one-shot; with param1=244, param2=id it returns the interval) are supported; the reply is `MESSAGE_INTERVAL` (#244) with `interval_us` (`-1` = disabled, `0` = not available) (MAV `#MESSAGE_INTERVAL`; `mavlink_receiver.cpp`).
- Maximum rate: no explicit cap. A stream only emits when its uORB topic has a new sample (`_att_sub.update()`), so the ceiling is the publisher rate (attitude = EKF2 output rate), and everything is scaled by `rate mult` (clamped 0.05..1) when the byte budget is exceeded (**not in docs**; `mavlink_main.cpp` `update_rate_mult()`, `streams/ATTITUDE.hpp`).

| Msg (id) | Timestamp field and PX4 source | Content, units, frame | Angular rates |
|---|---|---|---|
| ATTITUDE (30) | `time_boot_ms` = `vehicle_attitude.timestamp/1000` | roll/pitch/yaw rad, "aeronautical frame (right-handed, Z-down, Y-right, X-front, ZYX, intrinsic)" | yes, `vehicle_angular_velocity` rad/s |
| ATTITUDE_QUATERNION (31) | `time_boot_ms` = `vehicle_attitude.timestamp/1000` | q = (w,x,y,z), `repr_offset_q` for display | yes |
| LOCAL_POSITION_NED (32) | `time_boot_ms` = `vehicle_local_position.timestamp/1000` | x,y,z m NED, vx,vy,vz m/s; **no AGL, no attitude** | no |
| GLOBAL_POSITION_INT (33) | `time_boot_ms` = `vehicle_global_position.timestamp/1000` | lat/lon degE7, alt mm MSL, `relative_alt` mm above home (or above 0 if no home), v cm/s, `hdg` cdeg from `lpos.heading` | no |
| HIGHRES_IMU (105) | `time_usec` = `vehicle_imu.timestamp_sample` (µs) | accel m/s², gyro rad/s, mag gauss, body FRD; EKF2 bias estimates subtracted when device ids match (`streams/HIGHRES_IMU.hpp`) | raw gyro |
| ATTITUDE_TARGET (83) | `time_boot_ms` = setpoint timestamp | commanded q + body rate setpoints, thrust 0..1 | setpoints only |
| ODOMETRY (331) | `time_usec` = `vehicle_odometry.timestamp_sample` (µs, EKF sample time) | `frame_id` LOCAL_NED, `child_frame_id` BODY_FRD; x,y,z, q, v, pose/velocity covariance diagonals, `reset_counter`, `quality` | yes |
| ESTIMATOR_STATUS (230) | `time_usec` = `estimator_status.timestamp` | `flags` = `solution_status_flags`, innovation test ratios, `pos_horiz/vert_accuracy` 1σ m | no |
| VFR_HUD (74) | none | heading deg 0..360, alt MSL m, climb m/s | no |

Sources: field definitions MAV `#ATTITUDE` ... `#ESTIMATOR_STATUS`; timestamp origin per stream in SRC `src/modules/mavlink/streams/*.hpp` (**not in docs**).

Gotchas for exposure-time interpolation: the ms messages truncate a µs clock (1 ms quantisation at 100 Hz) and carry the *publication* time, not the IMU sample time; ODOMETRY is the only attitude message with a µs `timestamp_sample`, so for SentAI raise ODOMETRY to the needed rate with SET_MESSAGE_INTERVAL and use ATTITUDE only as a fallback. `time_boot_ms` wraps after 49.7 days (uint32).

---

## 3. Time synchronisation

**Protocol** (`https://mavlink.io/en/services/timesync.html`, MAV `#TIMESYNC`): requester sends `tc1 = 0`, `ts1 = own timestamp (ns)`; responder mirrors `ts1`, sets `tc1 = own timestamp (ns)` and (v2) `target_system/target_component` of the requester. Repeat and filter; both fields are `int64` **nanoseconds**.

**PX4 implementation** (**not in docs** beyond "implementation: `mavlink_timesync.cpp`"; SRC `src/modules/mavlink/mavlink_timesync.cpp`, `src/lib/timesync/Timesync.cpp`):

- Inbound `tc1 == 0`: PX4 answers `tc1 = hrt_absolute_time()*1000`, `ts1` mirrored, `target_system = msg.sysid`, `target_component = msg.compid` (`main`; release/1.14 answers by broadcast without targets).
- Inbound `tc1 > 0`: treated as a reply to PX4's own request: `offset_us = ((ts1/1000 + now) - 2*tc1/1000) / 2`, `rtt_us = now - ts1/1000`. Samples with `rtt > 10 ms` are dropped; after convergence samples deviating `> 100 ms` are dropped and 10 in a row reset the filter. Double-exponential smoothing with alpha/beta 0.05 -> 0.003 over `CONVERGENCE_WINDOW = 500` samples (50 s at the 10 Hz TIMESYNC stream). `sync_stamp(usec) = usec + offset` once converged, **else `hrt_absolute_time()` -- the remote timestamp is ignored until then**.
- `sync_stamp()` is applied only to inbound ODOMETRY, VISION_POSITION_ESTIMATE, ATT_POS_MOCAP, LANDING_TARGET, TARGET_ABSOLUTE/RELATIVE, RANGING_BEACON. Debug/NAMED_VALUE messages are stamped with receive time (see §6).
- PX4 streams TIMESYNC *requests* at 10 Hz in Onboard/ExtVision/Config modes (not Normal), `ts1 = hrt_absolute_time()*1000` (`streams/TIMESYNC.hpp`). It publishes `timesync_status` (`observed_offset`, `estimated_offset`, `round_trip_time`, µs), logged every 1000 ms in the default profile (`msg/TimesyncStatus.msg`, `logged_topics.cpp`).
- No `MAV_x_*` parameter influences timesync (**not in docs**; none in `module.yaml`).

**SYSTEM_TIME (#2)**: `time_unix_usec` (µs, UNIX epoch) + `time_boot_ms` (MAV `#SYSTEM_TIME`). PX4 streams it at 1 Hz in Onboard mode only when its RTC is past 2001-01-01 (`streams/SYSTEM_TIME.hpp`). Inbound on `main`: sets the RTC only if `SYS_TIME_SRC` has the MAVLink bit and the remote clock is > 60 s ahead (`SYS_TIME_SRC` **not in the parameter reference**); release/1.14: sets it when the onboard clock is before 2009-02-13 and the remote one valid.

**Outgoing timestamps**: `hrt_absolute_time()`, µs since boot; every uORB `timestamp` is "time since system start (microseconds)" (UG `dev_log/ulog_file_format.html`; `msg/*.msg`).

**Companion offset**: SentAI must (a) answer PX4's requests (mirror `ts1` exactly, `tc1` = local ns) so PX4 can stamp anything SentAI sends, and (b) run its own requester for its clock: `offset = ((t1 + t4)/2) - tc1` with t1 = `ts1`, t4 = local receive time, keeping min-RTT samples. Accuracy over UART: **not in docs**. Wire time of a 25-byte v2 frame at 921600 8N1 is ~0.27 ms each way; PX4 uses a single `now` for both receive and reply (no separate TX stamp), so its own scheduling + TX-queue delay is the asymmetric error term. PX4's own acceptance threshold (10 ms RTT) is the only documented expectation; measure the real RTT distribution on the vehicle.

---

## 4. Height above ground

| Item | Fact | Source |
|---|---|---|
| `DISTANCE_SENSOR` (132) **in** | `time_boot_ms` ignored ("don't trust sender"), stamped with receive time; cm -> m, `covariance` cm² -> m², `signal_quality` 0 -> -1 (unknown); device = MAVLINK bus; orientation must be `ROTATION_PITCH_270` for EKF2 to treat it as downward | SRC `mavlink_receiver.cpp` `handle_message_distance_sensor()` (**not in docs**); MAV `#DISTANCE_SENSOR` |
| `DISTANCE_SENSOR` **out** | 10 Hz in Onboard, all instances, `time_boot_ms = timestamp/1000`; only if a rangefinder is installed | UG `sensor/rangefinders.html`; `streams/DISTANCE_SENSOR.hpp` |
| `vehicle_local_position` | `dist_bottom` m, `dist_bottom_valid`, `dist_bottom_sensor_bitfield` (RANGE=1, FLOW=2), `hagl_min/max` | SRC `msg/VehicleLocalPosition.msg` |
| `ALTITUDE` (141) | `bottom_clearance = dist_bottom` and `altitude_terrain = -z - dist_bottom` **only if `dist_bottom_valid`, else NaN**; `altitude_local = -z`; `altitude_relative = -(z - home.z)`; `altitude_amsl = -z + ref_alt`; `time_usec = hrt now` | MAV `#ALTITUDE`; `streams/ALTITUDE.hpp` (**not in docs**) |
| LOCAL_POSITION_NED / ODOMETRY | no AGL field | MAV `#LOCAL_POSITION_NED`, `#ODOMETRY` |
| ESTIMATOR_STATUS | flag `ESTIMATOR_POS_VERT_AGL` (64) + `hagl_ratio` | MAV `#ESTIMATOR_STATUS_FLAGS` |
| Terrain estimator | rangefinder "is used by a single state filter to estimate the vertical position of the terrain relative to the height datum" | UG `advanced_config/tuning_the_ecl_ekf.html#range-finder` |
| `HOME_POSITION` (242) | lat/lon degE7, alt mm MSL, home x,y,z in local NED, `time_usec = home.timestamp`; 0.5 Hz | MAV `#HOME_POSITION`; `streams/HOME_POSITION.hpp` |
| `GPS_GLOBAL_ORIGIN` (49) | lat/lon/alt of local (0,0,0); sent when `ref_timestamp` changes (1 Hz slot); `time_usec = ref_timestamp` | MAV `#GPS_GLOBAL_ORIGIN`; `streams/GPS_GLOBAL_ORIGIN.hpp` |
| Local <-> lat/lon | local NED is an "azimuthal_equidistant_projection centred on an arbitrary origin"; origin set when global position is fused, or manually (`SET_GPS_GLOBAL_ORIGIN`) | UG `tuning_the_ecl_ekf.html#precisions-about-the-position-output` |

EKF2 range parameters (SRC `src/modules/ekf2/params_range_finder.yaml`, `params_terrain.yaml`; descriptions on UG `#range-finder`):

| Param | Default | Meaning |
|---|---|---|
| `EKF2_RNG_CTRL` | 1 | 0 off, 1 conditional (only below `EKF2_RNG_A_VMAX`/`A_HMAX`), 2 always |
| `EKF2_RNG_A_VMAX` / `A_HMAX` | 1.0 m/s / 5.0 m | conditional-aid limits |
| `EKF2_RNG_A_IGATE` | docs mention it; **not in `main` parameter reference** (removed) | |
| `EKF2_RNG_QLTY_T` | 1.0 s | min non-zero signal_quality duration |
| `EKF2_RNG_DELAY` / `NOISE` / `SFE` / `GATE` / `K_GATE` | 5 ms / 0.1 m / 0.05 m/m / 5 SD / 1 SD | |
| `EKF2_RNG_PITCH`, `EKF2_RNG_POS_X/Y/Z` | 0 rad, 0 m | mounting |
| `EKF2_RNG_FOG` | 3.0 m | fog/rain rejection (0 disables) |
| `EKF2_TERR_NOISE` / `TERR_GRAD` / `MIN_RNG` | 5 m/s / 0.5 m/m / 0.01 m | terrain filter |
| `EKF2_HGT_REF` | 1 GPS | 0 baro, 1 GNSS, 2 range, 3 vision |
| `EKF2_TERR_MASK` | **not in `main` parameter reference** | |

Gotcha: with no rangefinder (and no flow) `dist_bottom_valid` is false, `bottom_clearance` is NaN and the only AGL proxy is `altitude_relative` / `-z` relative to the take-off point (flat-ground assumption). The local origin may move with `EKF2_HGT_REF = 2` ("the local NED origin will move up and down with ground level", UG `#EKF2_HGT_REF`).

---

## 5. EKF2 outputs and accuracy

- Attitude accuracy numbers: **not in docs**. The guide gives qualitative diagnostics only ("Bad yaw alignment causes a velocity test ratio that increases rapidly when the vehicle starts moving", UG `tuning_the_ecl_ekf.html#determination-of-poor-yaw-accuracy`) and the state/covariance index map (`states[0..3]` quaternion, `[22..23]` wind NE, UG `#states`).
- Quality gates in ESTIMATOR_STATUS (MAV `#ESTIMATOR_STATUS`, enum `ESTIMATOR_STATUS_FLAGS`): `ATTITUDE`=1, `VELOCITY_HORIZ`=2, `VELOCITY_VERT`=4, `POS_HORIZ_REL`=8, `POS_HORIZ_ABS`=16, `POS_VERT_ABS`=32, `POS_VERT_AGL`=64, `CONST_POS_MODE`=128, `PRED_POS_HORIZ_REL`=256, `PRED_POS_HORIZ_ABS`=512, `GPS_GLITCH`=1024, `ACCEL_ERROR`=2048. Ratios: "Under normal operation the innovation test ratios should be below 0.5" (MAV description); PX4 maps `mag_ratio` <- `hdg_test_ratio` (low-pass filtered), `pos_horiz_ratio` <- `pos_test_ratio`, `pos_vert_ratio` <- `hgt_test_ratio` (`streams/ESTIMATOR_STATUS.hpp`). `pos_horiz/vert_accuracy` are 1σ m relative to the EKF origin. Richer `control_mode_flags` (CS_YAW_ALIGN, CS_MAG_HDG, CS_MAG_3D, CS_GPS_YAW, CS_WIND, CS_RNG_HGT, CS_GND_EFFECT ...) exist only in the logged `estimator_status` topic (`msg/EstimatorStatus.msg`).
- Yaw sources (UG `#magnetometer`, `#yaw-measurements`, `#yaw-from-gps-velocity`; SRC `params_magnetometer.yaml`, `params_gnss.yaml`):

| Param | Default | Values |
|---|---|---|
| `EKF2_MAG_TYPE` | 0 Automatic | 0 Automatic (heading before arming, full attitude after), 1 Magnetic heading, 5 None, 6 Init only (2-4 deprecated); reboot |
| `EKF2_GPS_CTRL` | 7 | bit0 lon/lat, bit1 altitude, bit2 3D velocity, bit3 (+8) dual-antenna heading |
| `EKF2_MAG_CHECK` | 1 | bit0 strength, bit1 inclination, bit2 wait for WMM |
| GSF yaw from GPS velocity | always on | `yaw_estimator_status.yaw_composite`, `yaw_variance`; needs horizontal acceleration ("The true heading is observable when the vehicle is accelerating ... while absolute position or velocity measurements are fused") |

- Multirotor wind/drag (UG `#multicopter-wind-estimation-using-drag-specific-forces`; SRC `params_drag.yaml`): `EKF2_DRAG_CTRL` bool, default 0; `EKF2_BCOEF_X`/`_Y` 100 kg/m² (0..200, 0 disables the axis); `EKF2_MCOEF` 0.15 1/s (0..1); `EKF2_DRAG_NOISE` 2.5 (m/s²)² (0.5..10). Tune from a position-mode back-and-forth flight with `mc_wind_estimator_tuning.py`. Output: WIND_COV (10 Hz Onboard) / `wind` topic.
- Position and velocity outputs are shifted from IMU to body origin using `EKF2_IMU_POS_X/Y/Z` (UG `#precisions-about-the-position-output`); the camera lever arm is the companion's job.

---

## 6. Logging SentAI scalars into the ULog

Mapping (UG `debug/debug_values.html`; SRC `mavlink_receiver.cpp`, `msg/Debug*.msg`):

| MAVLink in | uORB topic | Payload | Timestamp written |
|---|---|---|---|
| NAMED_VALUE_FLOAT (251) | `debug_key_value` | `key[10]`, float `value` | **receive time** (`hrt_absolute_time()`); `time_boot_ms` discarded |
| NAMED_VALUE_INT (252) | `debug_key_value` (value cast to float) | **not in docs**, handled in source | receive time |
| DEBUG (254) | `debug_value` | `ind`, `value` | receive time |
| DEBUG_VECT (250) | `debug_vect` | `name[10]`, x,y,z | receive time; `time_usec` discarded |
| DEBUG_FLOAT_ARRAY (350) | `debug_array` | `id`, `name[10]`, `data[58]` | **not in docs**, handled in source; receive time |

Handlers are compiled out on `CONSTRAINED_FLASH` boards (`mavlink_receiver.cpp`). Names are `char[10]` with forced null at `[9]` -> 9 usable characters (**not in docs**).

- Logging: `SDLOG_PROFILE` bitmask, default `1`; bit 5 (value 32) "Debug" adds `debug_array`, `debug_key_value`, `debug_value`, `debug_vect` at full rate (UG `dev_log/logging.html`; SRC `src/modules/logger/params.c`, `logged_topics.cpp` `add_debug_topics()`). Set `SDLOG_PROFILE = 33` (reboot). Alternatively list the topics in `etc/logging/logger_topics.txt` on the SD card (replaces the default set) (UG `dev_log/logging.html#sd-card-configuration`). `SDLOG_MODE` 0 = armed-to-disarm (default), 1 = boot-to-disarm; `SDLOG_BACKEND` bit 1 enables MAVLink log streaming (needs ~50 KB/s, MAVLink 2, one client) (same page).
- Rate limits: no numeric limit in docs; "Multiple debug messages must have enough time between their respective publishings for Mavlink to process them" (UG `debug/debug_values.html`). Logger default bandwidth ~50 KB/s (UG `dev_log/logging.html`); a `debug_array` sample is ~260 B, so 20 Hz costs ~5 KB/s.
- Reading: `pip install pyulog`; `ulog2csv -m debug_vect,debug_key_value,timesync_status log.ulg` (pyulog README). Python API (package source `pyulog/core.py`, not in README): `ULog(path, ['debug_vect','timesync_status']).get_dataset('debug_vect').data['timestamp']` -- all timestamps are PX4 µs since boot, comparable with `timesync_status.estimated_offset` to map back to SentAI time.

Gotcha (load-bearing): because PX4 overwrites the timestamp with receive time, SentAI must carry its own frame timestamp *inside* the payload (e.g. `DEBUG_FLOAT_ARRAY.data[0..1]` = local µs split hi/lo, or `DEBUG_VECT.z`) and log `timesync_status` to recover the offset post-mortem.

---

## 7. Heartbeat and component identity

- Identity enums (MAV `#MAV_TYPE`, `#MAV_COMPONENT`): `MAV_TYPE_ONBOARD_CONTROLLER = 18` ("Onboard companion controller"); `MAV_COMP_ID_ONBOARD_COMPUTER = 191` (192-194 for more), `MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197`, `MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196`, `MAV_COMP_ID_AUTOPILOT1 = 1`.
- System id: "A MAVLink system consists of one or more MAVLink components that all share the same system ID ... Any components in the system will share that same system ID" (`https://mavlink.io/en/services/mavlink_id_assignment.html`). Heartbeat cadence "typically 1 Hz"; disconnected after "four or five missed heartbeats" on RF links (`https://mavlink.io/en/services/heartbeat.html`).
- **PX4 only registers a heartbeat if `msg.sysid == MAV_SYS_ID` or `type == MAV_TYPE_GCS`** (**not in docs**; `mavlink_receiver.cpp` `handle_message_heartbeat()`). A companion on a different sysid is invisible to PX4's link bookkeeping. With the same sysid and type 18 it sets `telemetry_status.heartbeat_type_onboard_controller`, timeout `HEARTBEAT_TIMEOUT_US = 2.5 s` ("tolerate missing 1 + jitter", `msg/TelemetryStatus.msg`).
- Effect in Commander (**not in docs**; SRC `src/modules/commander/Commander.cpp` `dataLinkCheck()`): onboard-controller heartbeat loss after 5 s -> critical message "Connection to mission computer lost" + status change, **no failsafe action**. The data-link-loss failsafe (`COM_DL_LOSS_T`, default 10 s, 5..300; `NAV_DLL_ACT`, default 0 Disabled; `COM_DLL_EXCEPT` bit 2 = Offboard) is keyed **only** on `heartbeat_type_gcs` (UG `config/safety.html#data-link-loss-failsafe`; parameter reference). So never advertise SentAI as `MAV_TYPE_GCS`.
- Offboard loss is independent of HEARTBEAT: `offboard_control_signal_lost` is true when `offboard_control_mode.timestamp` (refreshed by setpoint messages) is older than `COM_OF_LOSS_T` (default 1.0 s, 0..60) -> action `COM_OBL_RC_ACT` (default 0 Position mode; 1 Altitude, 2 Stabilized, 3 Return, 4 Land, 5 Hold, 6 Terminate, 7 Disarm) (UG `config/safety.html#offboard-loss-failsafe`, `flight_modes/offboard.html` "continuous 2Hz proof of life"; SRC `HealthAndArmingChecks/checks/offboardCheck.cpp`).
- PX4's own HEARTBEAT interval cannot be changed by SET_MESSAGE_INTERVAL; `MAV_HB_FORW_EN` (default 1) controls forwarding of heartbeats across instances only when `MAV_1_FORWARD` is on (UG `#MAV_HB_FORW_EN`).

---

## Recommended companion-link configuration

| Item | Value | Why |
|---|---|---|
| `MAV_1_CONFIG` | 102 (TELEM 2) | §1; verify it is not 0 on the vehicle |
| `MAV_1_MODE` | 2 Onboard | TIMESYNC 10 Hz, ATTITUDE 100 Hz, ODOMETRY 30 Hz, ALTITUDE 10 Hz, SYSTEM_TIME 1 Hz, debug streams 10 Hz |
| `SER_TEL2_BAUD` | 921600 (default); 1500000/2000000 only after an on-vehicle test | enum has them; NuttX `B1500000` support is board dependent |
| `MAV_1_RATE` | 0 (= 46 080 B/s at 921600) | ample for the Onboard set |
| `MAV_1_FLOW_CTRL` | 1 Force on if CTS/RTS wired, else 0 | auto mode silently drops HW flow control when the buffer fills |
| `MAV_1_FORWARD` | 0 | keep GCS traffic off the link |
| `MAV_PROTO_VER` | 2 | required for >255 ids (ODOMETRY 331, DEBUG_FLOAT_ARRAY 350) |
| Companion heartbeat | sysid = `MAV_SYS_ID` (1), compid 191, type 18, 1 Hz | registered as onboard controller, never as GCS |
| Attitude source | ODOMETRY via SET_MESSAGE_INTERVAL(331, 10 000 µs = 100 Hz); keep ATTITUDE 100 Hz as fallback | µs `timestamp_sample` |
| Position / AGL | LOCAL_POSITION_NED 30 Hz, ALTITUDE 10 Hz (`bottom_clearance`), HOME_POSITION, GPS_GLOBAL_ORIGIN | §4 |
| Quality gate | ESTIMATOR_STATUS raised to 5 Hz; require flags & (1|8|32), ratios < 1 | §5 |
| Time | answer PX4 TIMESYNC requests; run own requester at 10 Hz, min-RTT filter, drop RTT > 10 ms | §3 |
| Logging | `SDLOG_PROFILE = 33`, `SDLOG_MODE = 1`; DEBUG_FLOAT_ARRAY with local µs in `data[0..1]`, name <= 9 chars | §6 |

## Open questions to verify on the vehicle

1. Is `MAV_1_CONFIG` actually 102 on the delivered firmware/board (parameter reference says default 0; board `rc.board_defaults` may differ)? Check `mavlink status`.
2. Does the flight-controller NuttX build define `B1500000`/`B2000000`? Try `SER_TEL2_BAUD=1500000` and confirm `mavlink status` shows the port open at that rate.
3. Measured TIMESYNC RTT distribution and the resulting `timesync_status.estimated_offset` jitter on the real UART (docs give no accuracy figure).
4. Does the companion heartbeat appear as `heartbeat_type_onboard_controller` in the logged `telemetry_status` (confirms sysid/type handling)?
5. Whether the vehicle has a downward rangefinder (`dist_bottom_valid`), otherwise `bottom_clearance` is NaN and AGL must be derived from `altitude_relative`.
6. Actual ODOMETRY maximum rate when requested at 100 Hz (bounded by EKF2 output rate; not documented).
7. Confirm the firmware version in `ulog_info` (`ver_sw`) so the 1.14 vs main differences in §1/§3 (TIMESYNC targeting, SYSTEM_TIME handling, stream list) can be pinned.

// ============== sentai.crazy — CrazyFlie autopilot bridge ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Python API:
//   sentai.crazy.init(baudrate=576000)        -> int (0=ok)
//   sentai.crazy.stop()                       -> int
//   sentai.crazy.debug(level)                 -> None
//   sentai.crazy.takeoff(h=0.5, dur=2.0, ...)  -> int
//   sentai.crazy.land(h=0.0, dur=2.0, ...)     -> int
//   sentai.crazy.stop_motors(group=0)          -> int
//   sentai.crazy.hover(vx=0, vy=0, yr=0, z=0.5) -> int  (call at ~10-20 Hz!)
//   sentai.crazy.go_to(x, y, z, ...)           -> int
//   sentai.crazy.send_crtp(port, ch, data)     -> int  (raw CRTP packet)

// sentai.crazy.init(baudrate=576000) -> int
static mp_obj_t mod_sentai_crazy_init(size_t n_args, const mp_obj_t *args) {
    uint32_t baudrate = (n_args > 0) ? (uint32_t)mp_obj_get_int(args[0]) : 576000;
    if (sentai_console_get_target() != 0) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("REPL must be on USB: call sentai.console('usb')"));
    }
    if (sentai_mesh_is_running()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("mesh is running: call sentai.mesh.stop() first"));
    }
    if (sentai_link_is_running()) {
        mp_raise_msg(&mp_type_OSError,
            MP_ERROR_TEXT("link is running: call sentai.link.stop() first"));
    }
    return mp_obj_new_int(sentai_crazy_init(baudrate));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_init_obj, 0, 1, mod_sentai_crazy_init);

// sentai.crazy.stop() -> int
static mp_obj_t mod_sentai_crazy_stop(void) {
    return mp_obj_new_int(sentai_crazy_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_stop_obj, mod_sentai_crazy_stop);

// sentai.crazy.debug(level) -> None
// 0=off, 1=TX/RX summary, 2=+hex dump
static mp_obj_t mod_sentai_crazy_debug(mp_obj_t level_obj) {
    sentai_crazy_set_debug(mp_obj_get_int(level_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_crazy_debug_obj, mod_sentai_crazy_debug);

// sentai.crazy.arm() -> int
// Arm the CrazyFlie (required before motors will spin).
// CF 2023+ firmware requires explicit arming. Returns 0=ok.
static mp_obj_t mod_sentai_crazy_arm(void) {
    return mp_obj_new_int(sentai_crazy_arm());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_arm_obj, mod_sentai_crazy_arm);

// sentai.crazy.disarm() -> int
// Disarm the CrazyFlie (motors won't spin until re-armed).
static mp_obj_t mod_sentai_crazy_disarm(void) {
    return mp_obj_new_int(sentai_crazy_disarm());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_disarm_obj, mod_sentai_crazy_disarm);

// sentai.crazy.takeoff(height=0.5, duration=2.0, yaw=0, use_current_yaw=1, group=0) -> int
static mp_obj_t mod_sentai_crazy_takeoff(size_t n_args, const mp_obj_t *args) {
    float height   = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.5f;
    float duration = (n_args > 1) ? mp_obj_get_float(args[1]) : 2.0f;
    float yaw      = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    int use_yaw    = (n_args > 3) ? mp_obj_get_int(args[3])   : 1;
    uint8_t group  = (n_args > 4) ? (uint8_t)mp_obj_get_int(args[4]) : 0;
    return mp_obj_new_int(sentai_crazy_takeoff(height, duration, yaw, use_yaw, group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_takeoff_obj, 0, 5, mod_sentai_crazy_takeoff);

// sentai.crazy.land(height=0.0, duration=2.0, yaw=0, use_current_yaw=1, group=0) -> int
static mp_obj_t mod_sentai_crazy_land(size_t n_args, const mp_obj_t *args) {
    float height   = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.0f;
    float duration = (n_args > 1) ? mp_obj_get_float(args[1]) : 2.0f;
    float yaw      = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    int use_yaw    = (n_args > 3) ? mp_obj_get_int(args[3])   : 1;
    uint8_t group  = (n_args > 4) ? (uint8_t)mp_obj_get_int(args[4]) : 0;
    return mp_obj_new_int(sentai_crazy_land(height, duration, yaw, use_yaw, group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_land_obj, 0, 5, mod_sentai_crazy_land);

// sentai.crazy.stop_motors(group=0) -> int
// Emergency: kills all motors immediately.
static mp_obj_t mod_sentai_crazy_stop_motors(size_t n_args, const mp_obj_t *args) {
    uint8_t group = (n_args > 0) ? (uint8_t)mp_obj_get_int(args[0]) : 0;
    return mp_obj_new_int(sentai_crazy_stop_motors(group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_stop_motors_obj, 0, 1, mod_sentai_crazy_stop_motors);

// sentai.crazy.hl_stop(group=0) -> int
// Release HL Commander (set to IDLE).  After takeoff, call this so
// Generic Setpoints (hover, attitude) actually drive the drone
// instead of being overridden by HL position-hold.  Does NOT kill
// motors; caller must already be sending Generic Setpoints at 10+ Hz.
static mp_obj_t mod_sentai_crazy_hl_stop(size_t n_args, const mp_obj_t *args) {
    uint8_t group = (n_args > 0) ? (uint8_t)mp_obj_get_int(args[0]) : 0;
    return mp_obj_new_int(sentai_crazy_hl_stop(group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_hl_stop_obj, 0, 1, mod_sentai_crazy_hl_stop);

// sentai.crazy.hover(vx=0, vy=0, yaw_rate=0, z=0.5) -> int
// MUST be called continuously at ~10-20 Hz to maintain hover!
// vx, vy: m/s (body frame), yaw_rate: deg/s, z: metres (absolute)
static mp_obj_t mod_sentai_crazy_hover(size_t n_args, const mp_obj_t *args) {
    float vx       = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.0f;
    float vy       = (n_args > 1) ? mp_obj_get_float(args[1]) : 0.0f;
    float yaw_rate = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    float z        = (n_args > 3) ? mp_obj_get_float(args[3]) : 0.5f;
    return mp_obj_new_int(sentai_crazy_hover(vx, vy, yaw_rate, z));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_hover_obj, 0, 4, mod_sentai_crazy_hover);

// sentai.crazy.go_to(x, y, z, yaw=0, duration=3.0, relative=1, linear=0, group=0) -> int
// x, y, z: metres;  yaw: radians;  duration: seconds
// relative=1: relative to current position;  linear=1: straight line
static mp_obj_t mod_sentai_crazy_go_to(size_t n_args, const mp_obj_t *args) {
    float x        = mp_obj_get_float(args[0]);
    float y        = mp_obj_get_float(args[1]);
    float z        = mp_obj_get_float(args[2]);
    float yaw      = (n_args > 3) ? mp_obj_get_float(args[3]) : 0.0f;
    float duration = (n_args > 4) ? mp_obj_get_float(args[4]) : 3.0f;
    int relative   = (n_args > 5) ? mp_obj_get_int(args[5])   : 1;
    int linear     = (n_args > 6) ? mp_obj_get_int(args[6])   : 0;
    uint8_t group  = (n_args > 7) ? (uint8_t)mp_obj_get_int(args[7]) : 0;
    return mp_obj_new_int(sentai_crazy_go_to(x, y, z, yaw, duration,
                                             relative, linear, group));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_go_to_obj, 3, 8, mod_sentai_crazy_go_to);

// sentai.crazy.send_crtp(port, channel, data) -> int
// Raw CRTP packet. data: bytes object (max 30 bytes).
static mp_obj_t mod_sentai_crazy_send_crtp(mp_obj_t port_obj,
                                           mp_obj_t ch_obj,
                                           mp_obj_t data_obj) {
    uint8_t port = (uint8_t)mp_obj_get_int(port_obj);
    uint8_t ch   = (uint8_t)mp_obj_get_int(ch_obj);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data_obj, &bufinfo, MP_BUFFER_READ);
    return mp_obj_new_int(sentai_crazy_send_crtp(
        port, ch, (const uint8_t*)bufinfo.buf, bufinfo.len));
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_sentai_crazy_send_crtp_obj, mod_sentai_crazy_send_crtp);

// sentai.crazy.ping(timeout_ms=1000) -> int
// Send CRTP echo to CrazyFlie and wait for response.
// Returns round-trip time in ms (>0) on success, or negative on error:
//   -1: bridge not running
//   -2: send failed
//   -3: timeout (drone not responding)
static mp_obj_t mod_sentai_crazy_ping(size_t n_args, const mp_obj_t *args) {
    int timeout_ms = (n_args > 0) ? mp_obj_get_int(args[0]) : 1000;
    return mp_obj_new_int(sentai_crazy_ping(timeout_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_ping_obj, 0, 1, mod_sentai_crazy_ping);

// sentai.crazy.test_fly(power=6553, duration_ms=2000) -> int
// Motor test via motorPowerSet params (same as cflib).
// power: 0-65535 (10% = 6553), duration_ms: blocking time.
// First call scans param TOC (~1-2s), then instant.
static mp_obj_t mod_sentai_crazy_test_fly(size_t n_args, const mp_obj_t *args) {
    uint16_t power      = (n_args > 0) ? (uint16_t)mp_obj_get_int(args[0]) : 6553;
    int      duration_ms = (n_args > 1) ? mp_obj_get_int(args[1])           : 2000;
    return mp_obj_new_int(sentai_crazy_test_fly(power, duration_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_test_fly_obj, 0, 2, mod_sentai_crazy_test_fly);

// sentai.crazy.fly(height=0.5, hold_ms=2000, takeoff_ms=3000, land_ms=3000) -> int
// Blocking HL Commander flight: arm → takeoff → hold → land → disarm.
// Uses Kalman estimator + barometer for altitude hold.
// height: altitude in metres (0.5 = 50cm above takeoff point).
static mp_obj_t mod_sentai_crazy_fly(size_t n_args, const mp_obj_t *args) {
    float height_m   = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.5f;
    int   hold_ms    = (n_args > 1) ? mp_obj_get_int(args[1])   : 2000;
    int   takeoff_ms = (n_args > 2) ? mp_obj_get_int(args[2])   : 3000;
    int   land_ms    = (n_args > 3) ? mp_obj_get_int(args[3])   : 3000;
    return mp_obj_new_int(sentai_crazy_fly(height_m, hold_ms, takeoff_ms, land_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_fly_obj, 0, 4, mod_sentai_crazy_fly);

// sentai.crazy.attitude(roll=0, pitch=0, yaw_rate=0, thrust=0) -> int
// Non-blocking: set Commander setpoint. CMD task sends at 50 Hz.
// Auto-arms on first call. Use fly_stop() to land.
static mp_obj_t mod_sentai_crazy_attitude(size_t n_args, const mp_obj_t *args) {
    float    roll    = (n_args > 0) ? mp_obj_get_float(args[0]) : 0.0f;
    float    pitch   = (n_args > 1) ? mp_obj_get_float(args[1]) : 0.0f;
    float    yawrate = (n_args > 2) ? mp_obj_get_float(args[2]) : 0.0f;
    uint16_t thrust  = (n_args > 3) ? (uint16_t)mp_obj_get_int(args[3]) : 0;
    return mp_obj_new_int(sentai_crazy_attitude(roll, pitch, yawrate, thrust));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_attitude_obj, 0, 4, mod_sentai_crazy_attitude);

// sentai.crazy.fly_stop() -> int
// Stop flying and disarm. Non-blocking, safe from any state.
static mp_obj_t mod_sentai_crazy_fly_stop(void) {
    return mp_obj_new_int(sentai_crazy_fly_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_fly_stop_obj, mod_sentai_crazy_fly_stop);

// =====================================================================
// CH_TELEM (channel 2) — drone telemetry queries via the deck-driver
// path. Each call sends a 1-byte cmd to the drone, drone reads the
// matching log var and replies with `[cmd][float32]`. Round-trip is
// typically a few ms (UART latency dominated).
//
// Returns the float (NaN on unknown cmd, default on transport error).
//
// On error returns the default-on-fail value (-999.0 by convention,
// matching the legacy `sentai.crazy.altitude()` API). Callers wanting
// to distinguish should use `sentai.crazy.telem(cmd, timeout)` which
// raises OSError on transport failure.
// =====================================================================
extern int sentai_crazy_query_telemetry(uint8_t cmd, float* out, int timeout_ms);

#define TELEM_BARO_ASL     0x01
#define TELEM_STATE_Z      0x02
#define TELEM_BATTERY_V    0x03
#define TELEM_BATTERY_PCT  0x04
#define TELEM_TEMP_C       0x05
#define TELEM_PRESSURE     0x06
/* Attitude (degrees, fused EKF) */
#define TELEM_ROLL         0x10
#define TELEM_PITCH        0x11
#define TELEM_YAW          0x12
/* Velocity (m/s, world frame, fused EKF) */
#define TELEM_VX           0x20
#define TELEM_VY           0x21
#define TELEM_VZ           0x22
/* Supervisor flags (bool — true if value != 0.0) */
#define TELEM_CANFLY       0x30
#define TELEM_IS_FLYING    0x31
#define TELEM_IS_TUMBLED   0x32
/* EKF cross-check (kalman_pred log group from mm_flow.c) */
#define TELEM_PRED_NX      0x40
#define TELEM_PRED_NY      0x41
#define TELEM_MEAS_NX      0x42
#define TELEM_MEAS_NY      0x43
/* PARAM SET — flowdeck.flowdeckPos_{x,y,z} (lever-arm), m */
#define TELEM_SET_POS_X    0x80
#define TELEM_SET_POS_Y    0x81
#define TELEM_SET_POS_Z    0x82

static float telem_or_default(uint8_t cmd, int timeout_ms, float fallback) {
    float v = fallback;
    if (sentai_crazy_query_telemetry(cmd, &v, timeout_ms) != 0) return fallback;
    return v;
}

// sentai.crazy.baro(timeout_ms=200) -> float
//   Barometric altitude above sea level (m). Raw baro reading, NOT
//   fused with IMU. Use altitude() for the EKF-fused estimate.
static mp_obj_t mod_sentai_crazy_baro(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_BARO_ASL, t, -999.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_baro_obj, 0, 1, mod_sentai_crazy_baro);

// sentai.crazy.altitude(timeout_ms=200) -> float
//   Fused altitude estimate (m) from CF Kalman estimator. Combines
//   barometer + IMU + flow if a flow deck is attached. Replaces the
//   former CPX-based altitude() which is now defunct (CPX disabled).
static mp_obj_t mod_sentai_crazy_altitude(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_STATE_Z, t, -999.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_altitude_obj, 0, 1, mod_sentai_crazy_altitude);

// sentai.crazy.battery(timeout_ms=200) -> float
//   Battery voltage (V).
static mp_obj_t mod_sentai_crazy_battery(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_BATTERY_V, t, -1.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_battery_obj, 0, 1, mod_sentai_crazy_battery);

// sentai.crazy.battery_pct(timeout_ms=200) -> float
//   Battery level (%).
static mp_obj_t mod_sentai_crazy_battery_pct(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_BATTERY_PCT, t, -1.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_battery_pct_obj, 0, 1, mod_sentai_crazy_battery_pct);

// sentai.crazy.temp(timeout_ms=200) -> float
//   Barometer temperature (°C).
static mp_obj_t mod_sentai_crazy_temp(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_TEMP_C, t, -999.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_temp_obj, 0, 1, mod_sentai_crazy_temp);

// sentai.crazy.pressure(timeout_ms=200) -> float
//   Atmospheric pressure (mbar).
static mp_obj_t mod_sentai_crazy_pressure(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_float(telem_or_default(TELEM_PRESSURE, t, -1.0f));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_pressure_obj, 0, 1, mod_sentai_crazy_pressure);

// sentai.crazy.telem(cmd:int, timeout_ms=200) -> float
//   Generic telemetry query — drops directly through to channel 2.
//   Useful for forward-compat: drone-side firmware can add cmd codes
//   without needing a new MP binding. Raises OSError on transport
//   failure so callers can distinguish "drone offline" from "value =
//   -999.0".
static mp_obj_t mod_sentai_crazy_telem(size_t n_args, const mp_obj_t* args) {
    uint8_t cmd = (uint8_t)mp_obj_get_int(args[0]);
    int t = (n_args > 1) ? mp_obj_get_int(args[1]) : 200;
    float v = 0.0f;
    int rc = sentai_crazy_query_telemetry(cmd, &v, t);
    if (rc != 0) {
        mp_raise_OSError(rc);
    }
    return mp_obj_new_float(v);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_telem_obj, 1, 2, mod_sentai_crazy_telem);

// sentai.crazy.flow_pred(timeout_ms=200) -> (predNX, predNY, measNX, measNY)
//   EKF cross-check: read the drone's predicted vs measured flow pixel
//   motion (PMW3901 units) for the last update.  When our scale_x/y
//   and body_xform are correct, predicted and measured should track
//   each other tightly during controlled motion.  Diverging values
//   point at a calibration error before flight.  Four sequential CH=2
//   queries (~5 ms wall).  Returns (-999.0, ...) tuple on transport
//   failure for any of the four.
static mp_obj_t mod_sentai_crazy_flow_pred(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    float pnx = -999.0f, pny = -999.0f, mnx = -999.0f, mny = -999.0f;
    sentai_crazy_query_telemetry(TELEM_PRED_NX, &pnx, t);
    sentai_crazy_query_telemetry(TELEM_PRED_NY, &pny, t);
    sentai_crazy_query_telemetry(TELEM_MEAS_NX, &mnx, t);
    sentai_crazy_query_telemetry(TELEM_MEAS_NY, &mny, t);
    mp_obj_t tuple[4] = { mp_obj_new_float(pnx), mp_obj_new_float(pny),
                          mp_obj_new_float(mnx), mp_obj_new_float(mny) };
    return mp_obj_new_tuple(4, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_flow_pred_obj, 0, 1, mod_sentai_crazy_flow_pred);

// sentai.crazy.flowdeck_pos(x, y, z, timeout_ms=400) -> (rb_x, rb_y, rb_z)
//   PARAM SET: write camera lever-arm offsets to the drone EKF
//   (flowdeck.flowdeckPos_x/y/z).  Body-frame metres, +x forward / +y
//   left / +z up.  Returns the drone's readback values (NaN on failure).
//   Three sequential CH=2 SETs (~6 ms wall) with longer default timeout
//   to absorb the paramSetFloat path.  Persistence: the param is marked
//   PARAM_PERSISTENT in the drone, so the value survives a drone reboot
//   if you also call eepromCommit on the drone side -- TODO add an
//   opcode for that, today the value goes to RAM only.
extern int sentai_crazy_set_telem(uint8_t cmd, float value,
                                   float* out_readback, int timeout_ms);
static mp_obj_t mod_sentai_crazy_flowdeck_pos(size_t n_args, const mp_obj_t* args) {
    float x = mp_obj_get_float(args[0]);
    float y = mp_obj_get_float(args[1]);
    float z = mp_obj_get_float(args[2]);
    int t = (n_args > 3) ? mp_obj_get_int(args[3]) : 400;
    float rbx = 0.0f, rby = 0.0f, rbz = 0.0f;
    sentai_crazy_set_telem(TELEM_SET_POS_X, x, &rbx, t);
    sentai_crazy_set_telem(TELEM_SET_POS_Y, y, &rby, t);
    sentai_crazy_set_telem(TELEM_SET_POS_Z, z, &rbz, t);
    mp_obj_t tuple[3] = { mp_obj_new_float(rbx), mp_obj_new_float(rby), mp_obj_new_float(rbz) };
    return mp_obj_new_tuple(3, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_flowdeck_pos_obj, 3, 4, mod_sentai_crazy_flowdeck_pos);

// sentai.crazy.attitude(timeout_ms=200) -> (roll, pitch, yaw)
//   Fused EKF Euler angles in degrees. Three sequential CH=2 queries
//   on the wire (~1.5 ms each), so worst-case latency is ~5 ms with
//   a 200 ms timeout per query. Returns NaN-tuple if transport fails.
static mp_obj_t mod_sentai_crazy_attitude_get(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    float r = -999.0f, p = -999.0f, y = -999.0f;
    sentai_crazy_query_telemetry(TELEM_ROLL,  &r, t);
    sentai_crazy_query_telemetry(TELEM_PITCH, &p, t);
    sentai_crazy_query_telemetry(TELEM_YAW,   &y, t);
    mp_obj_t tuple[3] = { mp_obj_new_float(r), mp_obj_new_float(p), mp_obj_new_float(y) };
    return mp_obj_new_tuple(3, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_attitude_get_obj, 0, 1, mod_sentai_crazy_attitude_get);

// sentai.crazy.velocity(timeout_ms=200) -> (vx, vy, vz)
//   Fused EKF velocity in world frame (m/s). Three sequential CH=2
//   queries — same wire-time pattern as attitude().
static mp_obj_t mod_sentai_crazy_velocity(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    float vx = -999.0f, vy = -999.0f, vz = -999.0f;
    sentai_crazy_query_telemetry(TELEM_VX, &vx, t);
    sentai_crazy_query_telemetry(TELEM_VY, &vy, t);
    sentai_crazy_query_telemetry(TELEM_VZ, &vz, t);
    mp_obj_t tuple[3] = { mp_obj_new_float(vx), mp_obj_new_float(vy), mp_obj_new_float(vz) };
    return mp_obj_new_tuple(3, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_velocity_obj, 0, 1, mod_sentai_crazy_velocity);

// Boolean flag accessors — drone log var is uint8_t, widened to float
// by logGetFloat. We threshold at != 0.0; transport failure returns
// False (conservative — better to say "not flying" than to claim it).
static bool telem_bool(uint8_t cmd, int timeout_ms) {
    float v = 0.0f;
    if (sentai_crazy_query_telemetry(cmd, &v, timeout_ms) != 0) return false;
    return v != 0.0f;
}

// sentai.crazy.canfly(timeout_ms=200) -> bool
static mp_obj_t mod_sentai_crazy_canfly(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_bool(telem_bool(TELEM_CANFLY, t));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_canfly_obj, 0, 1, mod_sentai_crazy_canfly);

// sentai.crazy.is_flying(timeout_ms=200) -> bool
static mp_obj_t mod_sentai_crazy_is_flying(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_bool(telem_bool(TELEM_IS_FLYING, t));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_is_flying_obj, 0, 1, mod_sentai_crazy_is_flying);

// sentai.crazy.is_tumbled(timeout_ms=200) -> bool
static mp_obj_t mod_sentai_crazy_is_tumbled(size_t n_args, const mp_obj_t* args) {
    int t = (n_args > 0) ? mp_obj_get_int(args[0]) : 200;
    return mp_obj_new_bool(telem_bool(TELEM_IS_TUMBLED, t));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_is_tumbled_obj, 0, 1, mod_sentai_crazy_is_tumbled);

// =====================================================================
// sentai.crazy.on_message(callback|None) — async dispatch
// =====================================================================
//
// Register a single Python callable to receive every inbound 0xAA frame
// from the drone-side bridge except those handled by the built-in
// `$`-prefix REPL exec path. Setting None drops further frames silently.
//
// Callback signature:    fn(channel: int, data: bytes) -> None
//
// Invocation runs in the MicroPython VM context (via mp_sched_schedule)
// — fully heap-safe, can call link_send to reply, can raise exceptions
// (caught + reported back over the radio as `ERR ...`).
//
// Inbound CH_REPL fragments are reassembled in C (MF byte stripped); the
// callback receives the *full* message. Other channels deliver each
// frame's raw body untouched.
//
// Wire-side semantics live in sentai_crazy.cc. This file owns the MP
// interface only.
//
// (runtime.h / obj.h / lexer.h / parse.h / compile.h are already pulled
//  in by modsentai.c — modsentai_crazy.c is #include'd from there.)

extern int sentai_crazy_dispatch_pop(uint8_t* kind, uint8_t* channel,
                                     uint8_t* buf, int max, int* out_len);
extern int sentai_crazy_link_send(int channel, const uint8_t* data, int len);

// GC-rooted handler slot. NULL = no handler (drop frames).
MP_REGISTER_ROOT_POINTER(mp_obj_t crazy_msg_handler);

#define CRAZY_KIND_EXEC   1u
#define CRAZY_KIND_USER   2u
#define CRAZY_DRAIN_BUF   256

/* Note: an earlier "g_crazy_drain_pending" dedup flag was removed
 * after we found that a single lost trampoline call could leave it
 * stuck at 1 forever (no consumer to clear it), silently blocking
 * all further schedules. The cost of always-schedule is at most a
 * few "drain empty" trampoline invocations per burst — each exits
 * immediately on FIFO empty. Worth it for the safety. */

int sentai_crazy_handler_is_set(void) {
    /* Read of an aligned pointer is atomic on 32-bit; the rx task only
     * needs a binary "set or not" answer to decide whether to enqueue
     * USER-kind frames at all. Race against on_message(None) at worst
     * delivers one extra frame to the trampoline, which checks again. */
    return MP_STATE_VM(crazy_msg_handler) != MP_OBJ_NULL ? 1 : 0;
}

/* ---------- $-prefix built-in REPL exec ---------- */

/* One screenful is the bound on every reply path. Keeps stack vstrs tiny
 * and prevents runaway repr() output from monopolizing the radio link. */
#define CRAZY_REPLY_BUF   200

/* Silent-truncation counter for reply payloads. Visible via
 * sentai.crazy.diag() (TODO) — callers can detect when their REPL
 * output is being clipped instead of suspecting a network bug. */
static volatile uint32_t g_crazy_reply_truncated = 0;

static void crazy_send_reply(const uint8_t* data, int len) {
    if (len < 0) len = 0;
    if (len > CRAZY_REPLY_BUF) {
        g_crazy_reply_truncated++;
        len = CRAZY_REPLY_BUF;
    }
    sentai_crazy_link_send(0, data, len);
}

static void crazy_send_reply_str(const char* s) {
    crazy_send_reply((const uint8_t*)s, (int)strlen(s));
}

static void crazy_run_exec(const uint8_t* src, int src_len) {
    if (src_len <= 0) {
        crazy_send_reply_str("ERR empty");
        return;
    }

    /* mp_lexer_new_from_str_len with free_len=0 doesn't take ownership
     * of the bytes — it reads exactly `src_len` chars. The dispatch
     * trampoline keeps the source buffer alive on its stack until this
     * function returns, so no copy / heap alloc is needed here.
     *
     * We lex twice (EVAL first, FILE on SyntaxError) because mp_parse
     * consumes its lexer; pre-tokenizing once would require a custom
     * reader. Two lex passes are fine — the source is at most 200 B. */
    qstr src_name = qstr_from_str("<radio>");
    mp_obj_t val = mp_const_none;
    bool used_eval = true;
    bool ok = false;

    nlr_buf_t outer;
    if (nlr_push(&outer) == 0) {
        nlr_buf_t inner;
        mp_parse_tree_t pt;
        if (nlr_push(&inner) == 0) {
            mp_lexer_t* lex = mp_lexer_new_from_str_len(src_name,
                (const char*)src, (size_t)src_len, 0);
            pt = mp_parse(lex, MP_PARSE_EVAL_INPUT);
            nlr_pop();
        } else {
            used_eval = false;
            mp_lexer_t* lex2 = mp_lexer_new_from_str_len(src_name,
                (const char*)src, (size_t)src_len, 0);
            pt = mp_parse(lex2, MP_PARSE_FILE_INPUT);
        }
        mp_obj_t mod = mp_compile(&pt, src_name, false);
        val = mp_call_function_0(mod);
        ok = true;
        nlr_pop();
    }

    /* Stack-only vstr — no GC heap on the reply path. Wrap the print in
     * an inner NLR: vstr_ensure_extra on a fixed-buf vstr raises
     * RuntimeError when full, and we want to ship whatever fit instead
     * of silently propagating the overflow up the dispatch trampoline. */
    VSTR_FIXED(v, CRAZY_REPLY_BUF);
    mp_print_t pr = { &v, (mp_print_strn_t)vstr_add_strn };

    if (ok) {
        if (used_eval) {
            vstr_add_str(&v, "OK ");
            nlr_buf_t pn;
            if (nlr_push(&pn) == 0) {
                mp_obj_print_helper(&pr, val, PRINT_REPR);
                nlr_pop();
            }
            /* On overflow we just send the truncated prefix as-is. */
        } else {
            vstr_add_str(&v, "OK");
        }
    } else {
        vstr_add_str(&v, "ERR ");
        nlr_buf_t pn;
        if (nlr_push(&pn) == 0) {
            mp_obj_print_helper(&pr, MP_OBJ_FROM_PTR(outer.ret_val), PRINT_EXC);
            nlr_pop();
        }
    }
    /* Always ship the reply, even when truncated or empty. */
    crazy_send_reply((const uint8_t*)vstr_str(&v), (int)vstr_len(&v));
}

/* ---------- user on_message(channel, data) ---------- */

static void crazy_run_user(uint8_t channel, const uint8_t* data, int len) {
    mp_obj_t handler = MP_STATE_VM(crazy_msg_handler);
    if (handler == MP_OBJ_NULL) return;

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t args[2] = {
            MP_OBJ_NEW_SMALL_INT((mp_int_t)channel),
            mp_obj_new_bytes(data, (size_t)len),
        };
        mp_call_function_n_kw(handler, 2, 0, args);
        nlr_pop();
    } else {
        /* User handler raised — send a short error back, no traceback.
         * Inner NLR catches vstr fixed-buf overflow (RuntimeError) so we
         * always ship whatever fit. */
        VSTR_FIXED(v, CRAZY_REPLY_BUF);
        mp_print_t pr = { &v, (mp_print_strn_t)vstr_add_strn };
        vstr_add_str(&v, "ERR ");
        nlr_buf_t pn;
        if (nlr_push(&pn) == 0) {
            mp_obj_print_helper(&pr, MP_OBJ_FROM_PTR(nlr.ret_val), PRINT_EXC);
            nlr_pop();
        }
        crazy_send_reply((const uint8_t*)vstr_str(&v), (int)vstr_len(&v));
    }
}

/* ---------- drain trampoline (one mp_sched slot, drains entire FIFO) ---------- */

static mp_obj_t crazy_dispatch_drain(mp_obj_t arg) {
    (void)arg;
    for (;;) {
        uint8_t kind = 0, channel = 0;
        uint8_t buf[CRAZY_DRAIN_BUF];
        int len = 0;
        if (sentai_crazy_dispatch_pop(&kind, &channel, buf, sizeof(buf), &len) == 0)
            break;
        if (kind == CRAZY_KIND_EXEC) {
            crazy_run_exec(buf, len);
        } else if (kind == CRAZY_KIND_USER) {
            crazy_run_user(channel, buf, len);
        }
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(crazy_dispatch_drain_obj, crazy_dispatch_drain);

void sentai_crazy_request_drain(void) {
    /* Always attempt to schedule (no dedup) — see comment near the
     * removed g_crazy_drain_pending. If the MP scheduler queue is
     * full, mp_sched_schedule returns false and the next push retries
     * naturally. Discarding the return value is safe because:
     *   - the FIFO already has the message stashed
     *   - any subsequent push will re-attempt the schedule
     *   - if the trampoline is already in-flight it will drain us anyway */
    (void)mp_sched_schedule(MP_OBJ_FROM_PTR(&crazy_dispatch_drain_obj),
                            mp_const_none);
}

// sentai.crazy.on_message(callback) -> None
//   callback(channel: int, data: bytes) is invoked for every inbound
//   frame except `$`-prefix exec messages on CH_REPL.
//   Pass None to detach (frames are dropped silently).
static mp_obj_t mod_sentai_crazy_on_message(mp_obj_t cb_obj) {
    if (cb_obj == mp_const_none) {
        MP_STATE_VM(crazy_msg_handler) = MP_OBJ_NULL;
    } else {
        if (!mp_obj_is_callable(cb_obj)) {
            mp_raise_TypeError(MP_ERROR_TEXT("on_message: callable or None required"));
        }
        MP_STATE_VM(crazy_msg_handler) = cb_obj;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_crazy_on_message_obj, mod_sentai_crazy_on_message);

// sentai.crazy.send_flow(dpx, dpy, dt, std) -> int
//
// Pack and ship one optical-flow measurement to the drone EKF on
// CH=1. Body-frame conventions (per paper/flow_body_frame.md):
//   dpx — accumulated body-X (forward) pixel motion since last sample
//   dpy — accumulated body-Y (left)    pixel motion since last sample
//   dt  — elapsed time in seconds (drone rejects dt <= 0 or > 1.0)
//   std — measurement standard deviation in pixels (>0, ≤ 100)
//
// On the drone, our deck driver parses the packed 16-byte flow_pkt_t,
// builds a flowMeasurement_t (Bitcraze convention) and calls
// estimatorEnqueueFlow(&fm). Sanity-rejected on bad floats / range —
// counter visible as deck.sentaiFlowDrp; accepted as deck.sentaiFlow.
//
// Caller is responsible for the camera→body transform AND the
// mgp→pixel conversion. With our 80×60 grid (step-8 from VGA),
// 1 grid-px = 8 raw-px, so:
//   raw_px = sentai.flow.read()['dx'] / 1000.0 * 8.0
// then apply the cam0/cam1 sign flip per body_frame.md.
//
// Returns 0 on success, -1=not running, -3=UART tx fail.
static mp_obj_t mod_sentai_crazy_send_flow(size_t n_args, const mp_obj_t* args) {
    /* Pack into the same layout the drone deck parses (16 B). Wire
     * is LE float32; Cortex-M is LE so direct memcpy works (the
     * static_assert in sentai_crazy.cc enforces this). */
    float dpx = mp_obj_get_float(args[0]);
    float dpy = mp_obj_get_float(args[1]);
    float dt  = mp_obj_get_float(args[2]);
    float std = mp_obj_get_float(args[3]);

    uint8_t pkt[16];
    memcpy(&pkt[0],  &dpx, 4);
    memcpy(&pkt[4],  &dpy, 4);
    memcpy(&pkt[8],  &dt,  4);
    memcpy(&pkt[12], &std, 4);

    return mp_obj_new_int(sentai_crazy_link_send(1, pkt, 16));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_send_flow_obj, 4, 4, mod_sentai_crazy_send_flow);

// sentai.crazy.send_extpos(x, y, z) -> int
// CRTP LOCALIZATION port 6 channel 0 — position-only VPE update.
// Packs the 12-byte payload inline and reuses the existing
// sentai_crazy_send_crtp transport (ARM + SIM, no extra wrapper).
static mp_obj_t mod_sentai_crazy_send_extpos(size_t n_args, const mp_obj_t* args) {
    float x = mp_obj_get_float(args[0]);
    float y = mp_obj_get_float(args[1]);
    float z = mp_obj_get_float(args[2]);
    uint8_t p[12];
    memcpy(p +  0, &x, 4);
    memcpy(p +  4, &y, 4);
    memcpy(p +  8, &z, 4);
    return mp_obj_new_int(sentai_crazy_send_crtp(/*port*/6, /*ch*/0, p, 12));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_send_extpos_obj, 3, 3, mod_sentai_crazy_send_extpos);

// sentai.crazy.send_extpose(x, y, z, qx, qy, qz, qw) -> int
// CRTP LOCALIZATION port 6 channel 1 — full ExtPose with quaternion
// (cf2 firmware expects type_id=8 prefix + 7 floats = 29 bytes).
static mp_obj_t mod_sentai_crazy_send_extpose(size_t n_args, const mp_obj_t* args) {
    float x  = mp_obj_get_float(args[0]);
    float y  = mp_obj_get_float(args[1]);
    float z  = mp_obj_get_float(args[2]);
    float qx = mp_obj_get_float(args[3]);
    float qy = mp_obj_get_float(args[4]);
    float qz = mp_obj_get_float(args[5]);
    float qw = mp_obj_get_float(args[6]);
    uint8_t p[29];
    p[0] = 8; /* EXT_POSE type id */
    memcpy(p +  1, &x,  4);
    memcpy(p +  5, &y,  4);
    memcpy(p +  9, &z,  4);
    memcpy(p + 13, &qx, 4);
    memcpy(p + 17, &qy, 4);
    memcpy(p + 21, &qz, 4);
    memcpy(p + 25, &qw, 4);
    return mp_obj_new_int(sentai_crazy_send_crtp(/*port*/6, /*ch*/1, p, 29));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_send_extpose_obj, 7, 7, mod_sentai_crazy_send_extpose);

// sentai.crazy.link_send(channel, data) -> int
// Send a payload to the drone-side "sentai" deck driver over UART2,
// using channel multiplexing on the 0xAA wire format:
//   channel=0  REPL / text reply    (drone forwards over radio CRTP port 0x0E)
//   channel=1  optical-flow data    (drone injects into EKF via estimatorEnqueueFlow)
//   channel=2  reserved
//   channel=3  reserved
// Returns 0=ok, -1=crazy not initialized, -2=invalid args, -3=UART tx fail.
extern int sentai_crazy_link_send(int channel, const uint8_t* data, int len);
static mp_obj_t mod_sentai_crazy_link_send(mp_obj_t channel_obj, mp_obj_t data_obj) {
    int channel = mp_obj_get_int(channel_obj);
    mp_buffer_info_t bi;
    mp_get_buffer_raise(data_obj, &bi, MP_BUFFER_READ);
    return mp_obj_new_int(sentai_crazy_link_send(
        channel, (const uint8_t*)bi.buf, (int)bi.len));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_crazy_link_send_obj, mod_sentai_crazy_link_send);

// ========== sentai.crazy.pose_* — Task #44: CRTP LOG in C =============
// Implementations live in examples/sentai_runtime/sentai_crazy_log.cc
// (shared ARM+SIM TU; ARM transport relies on a weak recv_pop stub
// until proper LOG routing lands — see sentai_crazy_log.cc comment).

#include "sentai_crazy_log.h"

// sentai.crazy.pose_subscribe(period_ms=100) -> int
// Returns 0 ok, -1 transport not initialised, -2 TOC scan timeout,
// -3 missing var, -4 CREATE_BLOCK NAK, -5 START_LOGGING NAK.
static mp_obj_t mod_sentai_crazy_pose_subscribe(size_t n_args, const mp_obj_t *args) {
    int period_ms = (n_args > 0) ? mp_obj_get_int(args[0]) : 100;
    return mp_obj_new_int(sentai_crazy_pose_subscribe(period_ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_pose_subscribe_obj, 0, 1, mod_sentai_crazy_pose_subscribe);

// sentai.crazy.pose() -> (x, y, z, yaw) tuple, or None if not ready.
static mp_obj_t mod_sentai_crazy_pose(void) {
    float x = 0, y = 0, z = 0, yaw = 0;
    int rc = sentai_crazy_pose(&x, &y, &z, &yaw);
    if (rc != 0) return mp_const_none;
    mp_obj_t tup[4] = {
        mp_obj_new_float(x), mp_obj_new_float(y),
        mp_obj_new_float(z), mp_obj_new_float(yaw),
    };
    return mp_obj_new_tuple(4, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_pose_obj, mod_sentai_crazy_pose);

// sentai.crazy.pose_stop() -> int (always 0).
static mp_obj_t mod_sentai_crazy_pose_stop(void) {
    return mp_obj_new_int(sentai_crazy_pose_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_pose_stop_obj, mod_sentai_crazy_pose_stop);

// sentai.crazy.pose_ready() -> 1 if subscribed AND data flowing.
static mp_obj_t mod_sentai_crazy_pose_ready(void) {
    return mp_obj_new_int(sentai_crazy_pose_ready());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_pose_ready_obj, mod_sentai_crazy_pose_ready);

// sentai.crazy.log_stats() -> (subscribed, toc_n, frames, block_id)
static mp_obj_t mod_sentai_crazy_log_stats(void) {
    uint32_t s[4];
    sentai_crazy_log_stats(s);
    mp_obj_t tup[4] = {
        mp_obj_new_int_from_uint(s[0]), mp_obj_new_int_from_uint(s[1]),
        mp_obj_new_int_from_uint(s[2]), mp_obj_new_int_from_uint(s[3]),
    };
    return mp_obj_new_tuple(4, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_log_stats_obj, mod_sentai_crazy_log_stats);

// ---- module table ----
static const mp_rom_map_elem_t sentai_crazy_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR_crazy) },
    { MP_ROM_QSTR(MP_QSTR_init),          MP_ROM_PTR(&mod_sentai_crazy_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),          MP_ROM_PTR(&mod_sentai_crazy_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug),         MP_ROM_PTR(&mod_sentai_crazy_debug_obj) },
    { MP_ROM_QSTR(MP_QSTR_arm),            MP_ROM_PTR(&mod_sentai_crazy_arm_obj) },
    { MP_ROM_QSTR(MP_QSTR_disarm),         MP_ROM_PTR(&mod_sentai_crazy_disarm_obj) },
    { MP_ROM_QSTR(MP_QSTR_takeoff),       MP_ROM_PTR(&mod_sentai_crazy_takeoff_obj) },
    { MP_ROM_QSTR(MP_QSTR_land),          MP_ROM_PTR(&mod_sentai_crazy_land_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_motors),   MP_ROM_PTR(&mod_sentai_crazy_stop_motors_obj) },
    { MP_ROM_QSTR(MP_QSTR_hl_stop),       MP_ROM_PTR(&mod_sentai_crazy_hl_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_hover),         MP_ROM_PTR(&mod_sentai_crazy_hover_obj) },
    { MP_ROM_QSTR(MP_QSTR_go_to),         MP_ROM_PTR(&mod_sentai_crazy_go_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_crtp),     MP_ROM_PTR(&mod_sentai_crazy_send_crtp_obj) },
    { MP_ROM_QSTR(MP_QSTR_ping),          MP_ROM_PTR(&mod_sentai_crazy_ping_obj) },
    { MP_ROM_QSTR(MP_QSTR_test_fly),      MP_ROM_PTR(&mod_sentai_crazy_test_fly_obj) },
    { MP_ROM_QSTR(MP_QSTR_fly),           MP_ROM_PTR(&mod_sentai_crazy_fly_obj) },
    { MP_ROM_QSTR(MP_QSTR_attitude),      MP_ROM_PTR(&mod_sentai_crazy_attitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_fly_stop),      MP_ROM_PTR(&mod_sentai_crazy_fly_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_altitude),       MP_ROM_PTR(&mod_sentai_crazy_altitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_baro),           MP_ROM_PTR(&mod_sentai_crazy_baro_obj) },
    { MP_ROM_QSTR(MP_QSTR_battery),        MP_ROM_PTR(&mod_sentai_crazy_battery_obj) },
    { MP_ROM_QSTR(MP_QSTR_battery_pct),    MP_ROM_PTR(&mod_sentai_crazy_battery_pct_obj) },
    { MP_ROM_QSTR(MP_QSTR_temp),           MP_ROM_PTR(&mod_sentai_crazy_temp_obj) },
    { MP_ROM_QSTR(MP_QSTR_pressure),       MP_ROM_PTR(&mod_sentai_crazy_pressure_obj) },
    { MP_ROM_QSTR(MP_QSTR_telem),          MP_ROM_PTR(&mod_sentai_crazy_telem_obj) },
    { MP_ROM_QSTR(MP_QSTR_attitude_get),   MP_ROM_PTR(&mod_sentai_crazy_attitude_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_velocity),       MP_ROM_PTR(&mod_sentai_crazy_velocity_obj) },
    { MP_ROM_QSTR(MP_QSTR_flow_pred),      MP_ROM_PTR(&mod_sentai_crazy_flow_pred_obj) },
    { MP_ROM_QSTR(MP_QSTR_flowdeck_pos),   MP_ROM_PTR(&mod_sentai_crazy_flowdeck_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_canfly),         MP_ROM_PTR(&mod_sentai_crazy_canfly_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_flying),      MP_ROM_PTR(&mod_sentai_crazy_is_flying_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_tumbled),     MP_ROM_PTR(&mod_sentai_crazy_is_tumbled_obj) },
    { MP_ROM_QSTR(MP_QSTR_on_message),     MP_ROM_PTR(&mod_sentai_crazy_on_message_obj) },
    { MP_ROM_QSTR(MP_QSTR_link_send),      MP_ROM_PTR(&mod_sentai_crazy_link_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_flow),      MP_ROM_PTR(&mod_sentai_crazy_send_flow_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_extpos),    MP_ROM_PTR(&mod_sentai_crazy_send_extpos_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_extpose),   MP_ROM_PTR(&mod_sentai_crazy_send_extpose_obj) },
    // Task #44 — pose_* (CRTP LOG in C)
    { MP_ROM_QSTR(MP_QSTR_pose_subscribe), MP_ROM_PTR(&mod_sentai_crazy_pose_subscribe_obj) },
    { MP_ROM_QSTR(MP_QSTR_pose),           MP_ROM_PTR(&mod_sentai_crazy_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_pose_stop),      MP_ROM_PTR(&mod_sentai_crazy_pose_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_pose_ready),     MP_ROM_PTR(&mod_sentai_crazy_pose_ready_obj) },
    { MP_ROM_QSTR(MP_QSTR_log_stats),      MP_ROM_PTR(&mod_sentai_crazy_log_stats_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_crazy_globals, sentai_crazy_globals_table);
static const mp_obj_module_t sentai_crazy_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_crazy_globals,
};

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

// sentai.crazy.altitude() -> float
// Read altitude from CF Kalman estimator (stateEstimate.z).
// First call scans log TOC + starts streaming (~2-5s), then instant.
// Returns altitude in metres, or -999.0 if not available.
static mp_obj_t mod_sentai_crazy_altitude(void) {
    return mp_obj_new_float(sentai_crazy_get_altitude());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_crazy_altitude_obj, mod_sentai_crazy_altitude);

// sentai.crazy.poll_event(timeout_ms=100) -> bytes or None
// Drain one CPX APP-layer message from the radio bridge queue.
// timeout_ms < 0 = wait forever (Ctrl-C to break).
extern int sentai_crazy_app_poll(int timeout_ms, uint8_t* out_buf,
                                 int out_max, int* out_len);
static mp_obj_t mod_sentai_crazy_poll_event(size_t n_args, const mp_obj_t* args) {
    int timeout_ms = (n_args >= 1) ? mp_obj_get_int(args[0]) : 100;
    uint8_t buf[96];
    int got = 0;
    int rc = sentai_crazy_app_poll(timeout_ms, buf, sizeof(buf), &got);
    if (rc <= 0) return mp_const_none;
    return mp_obj_new_bytes(buf, got);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_crazy_poll_event_obj, 0, 1, mod_sentai_crazy_poll_event);

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
    { MP_ROM_QSTR(MP_QSTR_hover),         MP_ROM_PTR(&mod_sentai_crazy_hover_obj) },
    { MP_ROM_QSTR(MP_QSTR_go_to),         MP_ROM_PTR(&mod_sentai_crazy_go_to_obj) },
    { MP_ROM_QSTR(MP_QSTR_send_crtp),     MP_ROM_PTR(&mod_sentai_crazy_send_crtp_obj) },
    { MP_ROM_QSTR(MP_QSTR_ping),          MP_ROM_PTR(&mod_sentai_crazy_ping_obj) },
    { MP_ROM_QSTR(MP_QSTR_test_fly),      MP_ROM_PTR(&mod_sentai_crazy_test_fly_obj) },
    { MP_ROM_QSTR(MP_QSTR_fly),           MP_ROM_PTR(&mod_sentai_crazy_fly_obj) },
    { MP_ROM_QSTR(MP_QSTR_attitude),      MP_ROM_PTR(&mod_sentai_crazy_attitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_fly_stop),      MP_ROM_PTR(&mod_sentai_crazy_fly_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_altitude),       MP_ROM_PTR(&mod_sentai_crazy_altitude_obj) },
    { MP_ROM_QSTR(MP_QSTR_poll_event),     MP_ROM_PTR(&mod_sentai_crazy_poll_event_obj) },
    { MP_ROM_QSTR(MP_QSTR_link_send),      MP_ROM_PTR(&mod_sentai_crazy_link_send_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_crazy_globals, sentai_crazy_globals_table);
static const mp_obj_module_t sentai_crazy_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_crazy_globals,
};

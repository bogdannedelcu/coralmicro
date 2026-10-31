// ============== sentai.imu — LIS2DU12 accelerometer ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// sentai.imu.init() -> int
// Returns 0 on success, -1 on failure
static mp_obj_t mod_sentai_imu_init(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.imu is board-only; not available in emulator"));
#else
    return mp_obj_new_int(sentai_imu_init());
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_init_obj, mod_sentai_imu_init);

// sentai.imu.read() -> dict or None
// Returns {x, y, z, temp} — x/y/z in milli-g (float), temp in °C (float)
static mp_obj_t mod_sentai_imu_read(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.imu is board-only; not available in emulator"));
#else
    float x, y, z, temp;
    int ret = sentai_imu_read_accel(&x, &y, &z, &temp);
    if (ret != 0) return mp_const_none;
    
    mp_obj_dict_t *d = mp_obj_new_dict(4);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_x), mp_obj_new_float(x));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_y), mp_obj_new_float(y));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_z), mp_obj_new_float(z));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_temp), mp_obj_new_float(temp));
    return MP_OBJ_FROM_PTR(d);
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_read_obj, mod_sentai_imu_read);

// sentai.imu.degrees() -> dict or None
// Returns {pitch, roll, temp} — pitch/roll in degrees (float), temp in °C
// pitch = atan2(x, sqrt(y²+z²)) — tilt forward/back, range ±90°
// roll  = atan2(y, sqrt(x²+z²)) — tilt left/right, range ±90°
static mp_obj_t mod_sentai_imu_degrees(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.imu is board-only; not available in emulator"));
#else
    float x, y, z, temp;
    int ret = sentai_imu_read_accel(&x, &y, &z, &temp);
    if (ret != 0) return mp_const_none;
    float pitch = atan2f(x, sqrtf(y * y + z * z)) * (180.0f / (float)M_PI);
    float roll  = atan2f(y, sqrtf(x * x + z * z)) * (180.0f / (float)M_PI);
    mp_obj_dict_t *d = mp_obj_new_dict(3);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_pitch), mp_obj_new_float(pitch));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_roll), mp_obj_new_float(roll));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_temp), mp_obj_new_float(temp));
    return MP_OBJ_FROM_PTR(d);
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_degrees_obj, mod_sentai_imu_degrees);

// sentai.imu.radians() -> dict or None
// Returns {pitch, roll, temp} — pitch/roll in radians (float), temp in °C
static mp_obj_t mod_sentai_imu_radians(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.imu is board-only; not available in emulator"));
#else
    float x, y, z, temp;
    int ret = sentai_imu_read_accel(&x, &y, &z, &temp);
    if (ret != 0) return mp_const_none;
    float pitch = atan2f(x, sqrtf(y * y + z * z));
    float roll  = atan2f(y, sqrtf(x * x + z * z));
    mp_obj_dict_t *d = mp_obj_new_dict(3);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_pitch), mp_obj_new_float(pitch));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_roll), mp_obj_new_float(roll));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_temp), mp_obj_new_float(temp));
    return MP_OBJ_FROM_PTR(d);
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_radians_obj, mod_sentai_imu_radians);

// sentai.imu.tap_start() -> int
// Configures LIS2DU12 hardware double-tap on INT2 and starts the background
// FreeRTOS task that polls TAP_SRC at 50 Hz and pushes events into a queue.
// Returns 0=ok or already running, <0=error.
static mp_obj_t mod_sentai_imu_tap_start(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.imu is board-only; not available in emulator"));
#else
    return mp_obj_new_int(sentai_imu_tap_start());
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_tap_start_obj, mod_sentai_imu_tap_start);

// sentai.imu.tap_stop() -> int
// Stops the tap-event task. Idempotent (safe to call when not running).
static mp_obj_t mod_sentai_imu_tap_stop(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.imu is board-only; not available in emulator"));
#else
    return mp_obj_new_int(sentai_imu_tap_stop());
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_tap_stop_obj, mod_sentai_imu_tap_stop);

// sentai.imu.poll_event(timeout_ms=100) -> str or None
// Block up to timeout_ms for an IMU event. Returns:
//   'double_tap' on a double-tap event
//   None on timeout
// timeout_ms < 0 means wait forever (use Ctrl-C to interrupt).
static mp_obj_t mod_sentai_imu_poll_event(size_t n_args, const mp_obj_t* args) {
#ifdef SENTAI_ARM_EMU
    (void)n_args;
    (void)args;
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.imu is board-only; not available in emulator"));
#else
    int timeout_ms = (n_args >= 1) ? mp_obj_get_int(args[0]) : 100;
    uint32_t ev = 0;
    int rc = sentai_imu_tap_poll(timeout_ms, &ev);
    if (rc <= 0) return mp_const_none;
    if (ev == 1u) return MP_OBJ_NEW_QSTR(MP_QSTR_double_tap);
    return mp_const_none;
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_imu_poll_event_obj, 0, 1, mod_sentai_imu_poll_event);

// ---- bus diagnostics ----
// sentai.imu.scan() -> list of 7-bit addresses that ACK on I2C5 (0x08..0x77)
// sentai.imu.reg_read(reg) -> int (0..255) or negative LPI2C status
// sentai.imu.reg_write(reg, val) -> 0 or negative LPI2C status
extern int sentai_imu_i2c_probe(int addr);
extern int sentai_imu_reg_read(int reg);
extern int sentai_imu_reg_write(int reg, int val);
static mp_obj_t mod_sentai_imu_scan(void) {
    mp_obj_t lst = mp_obj_new_list(0, NULL);
    for (int a = 0x08; a <= 0x77; ++a) {
        if (sentai_imu_i2c_probe(a) == 0) mp_obj_list_append(lst, mp_obj_new_int(a));
    }
    return lst;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_scan_obj, mod_sentai_imu_scan);
static mp_obj_t mod_sentai_imu_reg_read(mp_obj_t reg_o) {
    return mp_obj_new_int(sentai_imu_reg_read(mp_obj_get_int(reg_o)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_imu_reg_read_obj, mod_sentai_imu_reg_read);
static mp_obj_t mod_sentai_imu_reg_write(mp_obj_t reg_o, mp_obj_t val_o) {
    return mp_obj_new_int(sentai_imu_reg_write(mp_obj_get_int(reg_o), mp_obj_get_int(val_o)));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_imu_reg_write_obj, mod_sentai_imu_reg_write);

// ---- module table ----
static const mp_rom_map_elem_t sentai_imu_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR_scan),       MP_ROM_PTR(&mod_sentai_imu_scan_obj) },
    { MP_ROM_QSTR(MP_QSTR_reg_read),   MP_ROM_PTR(&mod_sentai_imu_reg_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_reg_write),  MP_ROM_PTR(&mod_sentai_imu_reg_write_obj) },
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_imu) },
    { MP_ROM_QSTR(MP_QSTR_init),       MP_ROM_PTR(&mod_sentai_imu_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),       MP_ROM_PTR(&mod_sentai_imu_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_degrees),    MP_ROM_PTR(&mod_sentai_imu_degrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_radians),    MP_ROM_PTR(&mod_sentai_imu_radians_obj) },
    { MP_ROM_QSTR(MP_QSTR_tap_start),  MP_ROM_PTR(&mod_sentai_imu_tap_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_tap_stop),   MP_ROM_PTR(&mod_sentai_imu_tap_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_poll_event), MP_ROM_PTR(&mod_sentai_imu_poll_event_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_imu_globals, sentai_imu_globals_table);
static const mp_obj_module_t sentai_imu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_imu_globals,
};

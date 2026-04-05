// ============== sentai.imu — LIS2DU12 accelerometer ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.imu.init() -> int
// Returns 0 on success, -1 on failure
static mp_obj_t mod_sentai_imu_init(void) {
    return mp_obj_new_int(sentai_imu_init());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_init_obj, mod_sentai_imu_init);

// sentai.imu.read() -> dict or None
// Returns {x, y, z, temp} — x/y/z in milli-g (float), temp in °C (float)
static mp_obj_t mod_sentai_imu_read(void) {
    float x, y, z, temp;
    int ret = sentai_imu_read_accel(&x, &y, &z, &temp);
    if (ret != 0) return mp_const_none;
    
    mp_obj_dict_t *d = mp_obj_new_dict(4);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_x), mp_obj_new_float(x));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_y), mp_obj_new_float(y));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_z), mp_obj_new_float(z));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_temp), mp_obj_new_float(temp));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_read_obj, mod_sentai_imu_read);

// sentai.imu.degrees() -> dict or None
// Returns {pitch, roll, temp} — pitch/roll in degrees (float), temp in °C
// pitch = atan2(x, sqrt(y²+z²)) — tilt forward/back, range ±90°
// roll  = atan2(y, sqrt(x²+z²)) — tilt left/right, range ±90°
static mp_obj_t mod_sentai_imu_degrees(void) {
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
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_degrees_obj, mod_sentai_imu_degrees);

// sentai.imu.radians() -> dict or None
// Returns {pitch, roll, temp} — pitch/roll in radians (float), temp in °C
static mp_obj_t mod_sentai_imu_radians(void) {
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
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_imu_radians_obj, mod_sentai_imu_radians);

// ---- module table ----
static const mp_rom_map_elem_t sentai_imu_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_imu) },
    { MP_ROM_QSTR(MP_QSTR_init),       MP_ROM_PTR(&mod_sentai_imu_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),       MP_ROM_PTR(&mod_sentai_imu_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_degrees),    MP_ROM_PTR(&mod_sentai_imu_degrees_obj) },
    { MP_ROM_QSTR(MP_QSTR_radians),    MP_ROM_PTR(&mod_sentai_imu_radians_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_imu_globals, sentai_imu_globals_table);
static const mp_obj_module_t sentai_imu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_imu_globals,
};

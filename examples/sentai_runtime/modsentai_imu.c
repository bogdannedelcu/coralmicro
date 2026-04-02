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

// ---- module table ----
static const mp_rom_map_elem_t sentai_imu_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_imu) },
    { MP_ROM_QSTR(MP_QSTR_init),       MP_ROM_PTR(&mod_sentai_imu_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),       MP_ROM_PTR(&mod_sentai_imu_read_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_imu_globals, sentai_imu_globals_table);
static const mp_obj_module_t sentai_imu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_imu_globals,
};

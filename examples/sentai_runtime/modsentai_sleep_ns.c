// ============== sentai.sleep — Light sleep / idle ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.sleep.idle(threshold_db=70, timeout_ms=0, enable_tap=1) -> int
// Light sleep waiting for mic sound, double-tap, or timeout.
// threshold_db: 60, 65, 70, 75, 80, 85, 90, 95 dB (lower = more sensitive)
// timeout_ms: max wait time in ms (0 = wait forever)
// enable_tap: 1 = enable double-tap wakeup (default), 0 = mic only
// Returns: 0 = timeout, 1 = mic wakeup, 2 = double-tap wakeup, -1 = error
extern int sentai_sleep_idle(int threshold_db, int timeout_ms, int enable_tap);
static mp_obj_t mod_sentai_sleep_idle(size_t n_args, const mp_obj_t *args) {
    int threshold_db = (n_args > 0) ? mp_obj_get_int(args[0]) : 70;
    int timeout_ms = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
    int enable_tap = (n_args > 2) ? mp_obj_get_int(args[2]) : 1;  // enabled by default
    int ret = sentai_sleep_idle(threshold_db, timeout_ms, enable_tap);
    if (ret < 0) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("Failed to enter idle mode"));
    }
    return mp_obj_new_int(ret);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_sleep_idle_obj, 0, 3, mod_sentai_sleep_idle);

// ---- module table ----
static const mp_rom_map_elem_t sentai_sleep_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sleep) },
    { MP_ROM_QSTR(MP_QSTR_idle),     MP_ROM_PTR(&mod_sentai_sleep_idle_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_sleep_globals, sentai_sleep_globals_table);
static const mp_obj_module_t sentai_sleep_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_sleep_globals,
};

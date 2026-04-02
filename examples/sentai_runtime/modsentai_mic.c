// ============== sentai.mic — Microphone recording ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.mic.start(seconds=5) -> int
static mp_obj_t mod_sentai_mic_start(size_t n_args, const mp_obj_t *args) {
    int seconds = (n_args > 0) ? mp_obj_get_int(args[0]) : 5;
    return mp_obj_new_int(sentai_mic_start(seconds));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mic_start_obj, 0, 1, mod_sentai_mic_start);

// sentai.mic.stop() -> int
static mp_obj_t mod_sentai_mic_stop(void) {
    return mp_obj_new_int(sentai_mic_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_stop_obj, mod_sentai_mic_stop);

// sentai.mic.busy() -> bool
static mp_obj_t mod_sentai_mic_busy(void) {
    return mp_obj_new_bool(sentai_mic_busy() == 1);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_busy_obj, mod_sentai_mic_busy);

// sentai.mic.samples() -> int
static mp_obj_t mod_sentai_mic_samples(void) {
    return mp_obj_new_int(sentai_mic_samples());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_samples_obj, mod_sentai_mic_samples);

// sentai.mic.save_l3() -> str or None (MP3 encoding)
static mp_obj_t mod_sentai_mic_save_l3(void) {
    char name[48];
    int ret = sentai_mic_save_l3(name, sizeof(name));
    if (ret < 0) return mp_const_none;
    return mp_obj_new_str(name, strlen(name));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_save_l3_obj, mod_sentai_mic_save_l3);

// sentai.mic.level() -> int
static mp_obj_t mod_sentai_mic_level(void) {
    return mp_obj_new_int(sentai_mic_level());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_level_obj, mod_sentai_mic_level);

// ---- module table ----
static const mp_rom_map_elem_t sentai_mic_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_mic) },
    { MP_ROM_QSTR(MP_QSTR_start),      MP_ROM_PTR(&mod_sentai_mic_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),       MP_ROM_PTR(&mod_sentai_mic_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_busy),       MP_ROM_PTR(&mod_sentai_mic_busy_obj) },
    { MP_ROM_QSTR(MP_QSTR_samples),    MP_ROM_PTR(&mod_sentai_mic_samples_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_l3),    MP_ROM_PTR(&mod_sentai_mic_save_l3_obj) },
    { MP_ROM_QSTR(MP_QSTR_level),      MP_ROM_PTR(&mod_sentai_mic_level_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_mic_globals, sentai_mic_globals_table);
static const mp_obj_module_t sentai_mic_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_mic_globals,
};

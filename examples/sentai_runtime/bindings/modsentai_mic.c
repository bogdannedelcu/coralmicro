// ============== sentai.mic — Microphone recording ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.mic.start(seconds=5) -> int
static mp_obj_t mod_sentai_mic_start(size_t n_args, const mp_obj_t *args) {
#ifdef SENTAI_ARM_EMU
    (void)n_args;
    (void)args;
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.mic is board-only; not available in emulator"));
#else
    int seconds = (n_args > 0) ? mp_obj_get_int(args[0]) : 5;
    return mp_obj_new_int(sentai_mic_start(seconds));
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_mic_start_obj, 0, 1, mod_sentai_mic_start);

// sentai.mic.stop() -> int
static mp_obj_t mod_sentai_mic_stop(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.mic is board-only; not available in emulator"));
#else
    return mp_obj_new_int(sentai_mic_stop());
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_stop_obj, mod_sentai_mic_stop);

// sentai.mic.recording() -> bool
static mp_obj_t mod_sentai_mic_recording(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.mic is board-only; not available in emulator"));
#else
    return mp_obj_new_bool(sentai_mic_busy() == 1);
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_recording_obj, mod_sentai_mic_recording);

// sentai.mic.samples() -> int
static mp_obj_t mod_sentai_mic_samples(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.mic is board-only; not available in emulator"));
#else
    return mp_obj_new_int(sentai_mic_samples());
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_samples_obj, mod_sentai_mic_samples);

// sentai.mic.save_mp3() -> str or None (MP3 encoding)
static mp_obj_t mod_sentai_mic_save_mp3(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.mic is board-only; not available in emulator"));
#else
    char name[48];
    int ret = sentai_mic_save_l3(name, sizeof(name));
    if (ret < 0) return mp_const_none;
    return mp_obj_new_str(name, strlen(name));
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_save_mp3_obj, mod_sentai_mic_save_mp3);

// sentai.mic.level() -> int
static mp_obj_t mod_sentai_mic_level(void) {
#ifdef SENTAI_ARM_EMU
    mp_raise_NotImplementedError(
        MP_ERROR_TEXT("sentai.mic is board-only; not available in emulator"));
#else
    return mp_obj_new_int(sentai_mic_level());
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_mic_level_obj, mod_sentai_mic_level);

// ---- module table ----
static const mp_rom_map_elem_t sentai_mic_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_mic) },
    { MP_ROM_QSTR(MP_QSTR_start),      MP_ROM_PTR(&mod_sentai_mic_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),       MP_ROM_PTR(&mod_sentai_mic_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_recording),  MP_ROM_PTR(&mod_sentai_mic_recording_obj) },
    { MP_ROM_QSTR(MP_QSTR_samples),    MP_ROM_PTR(&mod_sentai_mic_samples_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_mp3),   MP_ROM_PTR(&mod_sentai_mic_save_mp3_obj) },
    { MP_ROM_QSTR(MP_QSTR_level),      MP_ROM_PTR(&mod_sentai_mic_level_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_mic_globals, sentai_mic_globals_table);
static const mp_obj_module_t sentai_mic_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_mic_globals,
};

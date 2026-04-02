// ============== sentai.io — LED / GPIO ==============
// This file is #include'd from modsentai.c — do NOT compile separately.

// sentai.io.led_on()
static mp_obj_t mod_sentai_led_on(void) {
    sentai_led_set(1);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_led_on_obj, mod_sentai_led_on);

// sentai.io.led_off()
static mp_obj_t mod_sentai_led_off(void) {
    sentai_led_set(0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_led_off_obj, mod_sentai_led_off);

// ---- module table ----
static const mp_rom_map_elem_t sentai_io_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_io) },
    { MP_ROM_QSTR(MP_QSTR_led_on),   MP_ROM_PTR(&mod_sentai_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_led_off),  MP_ROM_PTR(&mod_sentai_led_off_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_io_globals, sentai_io_globals_table);
static const mp_obj_module_t sentai_io_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_io_globals,
};

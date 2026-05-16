/* modsentai_sim_io.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */

/* ===== sentai.io ===== */
static mp_obj_t sentai_io_led_on(void) {
    if (s_verbose) printf("[LED] ON\n");
    fflush(stdout);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_io_led_on_obj, sentai_io_led_on);

static mp_obj_t sentai_io_led_off(void) {
    if (s_verbose) printf("[LED] OFF\n");
    fflush(stdout);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_io_led_off_obj, sentai_io_led_off);

static const mp_rom_map_elem_t sentai_io_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_io) },
    { MP_ROM_QSTR(MP_QSTR_led_on),   MP_ROM_PTR(&sentai_io_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_led_off),  MP_ROM_PTR(&sentai_io_led_off_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_io_globals, sentai_io_globals_table);
static const mp_obj_module_t sentai_io_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_io_globals,
};


/* modsentai_sim_sys.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */

/* ===== sentai.sys ===== */
static mp_obj_t sentai_sys_reset(void) {
    printf("[sim] sentai.sys.reset() called — exiting\n");
    fflush(stdout);
    exit(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_sys_reset_obj, sentai_sys_reset);

static const mp_rom_map_elem_t sentai_sys_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sys) },
    { MP_ROM_QSTR(MP_QSTR_reset),    MP_ROM_PTR(&sentai_sys_reset_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_sys_globals, sentai_sys_globals_table);
static const mp_obj_module_t sentai_sys_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_sys_globals,
};


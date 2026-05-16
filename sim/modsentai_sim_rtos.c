/* modsentai_sim_rtos.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */

/* ===== sentai.rtos ===== */
static mp_obj_t sentai_rtos_sleep_ms(mp_obj_t ms_obj) {
    mp_int_t ms = mp_obj_get_int(ms_obj);
    if (ms < 0) ms = 0;
    /* vTaskDelay is the real FreeRTOS API — same as on board.  Schedules
     * other tasks for the duration.  No EINTR concern: vTaskDelay is
     * implemented inside the kernel's signal mask. */
    vTaskDelay(pdMS_TO_TICKS((TickType_t) ms));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_rtos_sleep_ms_obj,
                                  sentai_rtos_sleep_ms);

static const mp_rom_map_elem_t sentai_rtos_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_rtos) },
    { MP_ROM_QSTR(MP_QSTR_sleep_ms), MP_ROM_PTR(&sentai_rtos_sleep_ms_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_rtos_globals, sentai_rtos_globals_table);
static const mp_obj_module_t sentai_rtos_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_rtos_globals,
};

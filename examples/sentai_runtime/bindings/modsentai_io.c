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

// sentai.io.pin_probe() -> tuple of 11 ints
//
// EXP-s245 iter2 (TD-S10-A10) — is Gpio::kCameraTrigger (GPIO8_IO27,
// pad GPIO_EMC_B2_17) physically routed to anything?  Runs a weak-pull
// flip test on it plus on the adjacent known-routed kEdgeTpuPgood as a
// methodology control, and checks SDRAM integrity across the probe.
// Full method + limitations documented at the C implementation in
// sentai_runtime.cc and in the experiment README.
//
// Layout — 5 entries per pin, pin0 = kCameraTrigger (under test),
// pin1 = kEdgeTpuPgood (control):
//   [0] no-pull read   [1] pull-up read   [2] pull-down read
//   [3] cycles to charge to 1 through the pull-up (-1 = n/a)
//   [4] 1 = followed both pulls (floating or high-Z load)
// [10] = SDRAM canary mismatches, 0 = memory intact.
// park_input=1 leaves kCameraTrigger as a high-impedance input instead
// of restoring it to the boot state (output driving 0).  Required for
// EXP-s245 iter3: while any OV5640 output pad is enabled, an MCU output
// on the same net would be a short through both drivers.
extern int sentai_pin_probe_cam_trigger(int32_t* out, int n, int park_input);
static mp_obj_t mod_sentai_pin_probe(size_t n_args, const mp_obj_t *args) {
    int park = (n_args >= 1) ? (mp_obj_is_true(args[0]) ? 1 : 0) : 0;
    int32_t r[11];
    int got = sentai_pin_probe_cam_trigger(r, 11, park);
    if (got < 11) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("pin_probe failed"));
    }
    mp_obj_t items[11];
    for (int i = 0; i < 11; ++i) items[i] = mp_obj_new_int(r[i]);
    return mp_obj_new_tuple(11, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pin_probe_obj, 0, 1,
                                           mod_sentai_pin_probe);

// ---- module table ----
static const mp_rom_map_elem_t sentai_io_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_io) },
    { MP_ROM_QSTR(MP_QSTR_led_on),   MP_ROM_PTR(&mod_sentai_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_led_off),  MP_ROM_PTR(&mod_sentai_led_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_pin_probe),MP_ROM_PTR(&mod_sentai_pin_probe_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_io_globals, sentai_io_globals_table);
static const mp_obj_module_t sentai_io_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_io_globals,
};

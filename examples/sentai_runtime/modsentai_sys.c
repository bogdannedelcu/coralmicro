// ============== sentai.sys — System control & anti-brick status ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Provides:
//   sentai.sys.reset()          — flush crash log then NVIC_SystemReset
//   sentai.sys.recovery_mode()  — True if boot loop detected, running minimal REPL
//   sentai.sys.boot_attempts()  — Current SRC_GPR boot attempt counter value

// C API implemented in sentai_runtime.cc (extern "C" linkage)
extern bool    sentai_is_recovery_mode(void);
extern unsigned int sentai_get_boot_attempts(void);
extern void    sentai_sys_do_reset(void);

// sentai.sys.reset() → None
// Flushes crash log and triggers a clean software reset.
static mp_obj_t mod_sentai_sys_reset(void) {
    sentai_sys_do_reset();  // logs + NVIC_SystemReset — never returns
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sys_reset_obj, mod_sentai_sys_reset);

// sentai.sys.recovery_mode() → bool
// Returns True when the board entered recovery mode due to 3+ consecutive boot crashes.
// In recovery mode: REPL works, USB visible, can reflash; no bridges/HTTP/detection.
static mp_obj_t mod_sentai_sys_recovery_mode(void) {
    return mp_obj_new_bool(sentai_is_recovery_mode());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sys_recovery_mode_obj, mod_sentai_sys_recovery_mode);

// sentai.sys.boot_attempts() → int
// Returns current boot attempt counter from SRC_GPR (survives warm reset).
// Cleared to 0 by a healthy boot that reaches boot_complete().
// >= 3 triggers automatic recovery mode on next reboot.
static mp_obj_t mod_sentai_sys_boot_attempts(void) {
    return mp_obj_new_int((mp_int_t)sentai_get_boot_attempts());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sys_boot_attempts_obj, mod_sentai_sys_boot_attempts);

// ---- module table ----
static const mp_rom_map_elem_t sentai_sys_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),       MP_ROM_QSTR(MP_QSTR_sys) },
    { MP_ROM_QSTR(MP_QSTR_reset),          MP_ROM_PTR(&mod_sentai_sys_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_recovery_mode),  MP_ROM_PTR(&mod_sentai_sys_recovery_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_boot_attempts),  MP_ROM_PTR(&mod_sentai_sys_boot_attempts_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_sys_globals, sentai_sys_globals_table);
static const mp_obj_module_t sentai_sys_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_sys_globals,
};

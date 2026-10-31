// ============== sentai.sys — System control & anti-brick status ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Provides:
//   sentai.sys.reset()          — flush crash log then NVIC_SystemReset
//   sentai.sys.recovery_mode()  — True if boot loop detected, running minimal REPL
//   sentai.sys.boot_attempts()  — Current SRC_GPR boot attempt counter value
//   sentai.sys.peek32(addr)     — Read one 32-bit word (diagnostics; ARM only)

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

// sentai.sys.peek32(addr) → int
// Volatile 32-bit read of an arbitrary address — peripheral registers
// (SRC_SBMR*, CCM, IOMUXC …) or memory.  Read-only diagnostic; the address
// must be word-aligned.  EXP-s248 added it to read the boot-mode straps
// latched in SRC_SBMR1/2 when neither J-Link nor the flashloader could.
// SIM has no MMIO: raises OSError.
// sentai.sys.uid() -> str
// 64-bit chip UNIQUE_ID from OCOTP (fuse words 0x900/0x910) as 16 lower-case
// hex digits; identical to the USB iSerial descriptor, so
// /dev/serial/by-id/*_<uid>-if00 names the same board.  SIM: "sim".
#ifndef SENTAI_PLATFORM_SIM
extern const char *sentai_board_serial(void);
#endif
static mp_obj_t mod_sentai_sys_uid(void) {
#ifdef SENTAI_PLATFORM_SIM
    return mp_obj_new_str("sim", 3);
#else
    const char *s = sentai_board_serial();
    return mp_obj_new_str(s, strlen(s));
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sys_uid_obj, mod_sentai_sys_uid);

static mp_obj_t mod_sentai_sys_peek32(mp_obj_t addr_obj) {
#ifdef SENTAI_PLATFORM_SIM
    (void)addr_obj;
    mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("peek32 not available on SIM"));
#else
    uint32_t addr = (uint32_t)mp_obj_get_int_truncated(addr_obj);
    if (addr & 3u) {
        mp_raise_ValueError(MP_ERROR_TEXT("peek32: address must be 4-byte aligned"));
    }
    return mp_obj_new_int_from_uint(*(volatile uint32_t *)addr);
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_sys_peek32_obj, mod_sentai_sys_peek32);

// sentai.sys.health() -> dict: per-peripheral presence (True/False/None =
// not probed yet), the missing-code list the STAT LED blinks (1 SD, 2 cam0,
// 3 cam1, 4 IMU, 5 TPU) and the remaining indication window in ms.
extern void     sentai_health_periph(int id, int present);
extern uint32_t sentai_health_periph_missing_mask(void);
extern uint32_t sentai_health_periph_known_mask(void);
extern uint32_t sentai_health_led_remaining_ms(void);
static mp_obj_t mod_sentai_sys_health(void) {
    uint32_t miss = sentai_health_periph_missing_mask(), known = sentai_health_periph_known_mask();
    static const char* names[6] = {"", "sd", "cam0", "cam1", "imu", "tpu"};
    mp_obj_t d = mp_obj_new_dict(8);
    mp_obj_t codes = mp_obj_new_list(0, NULL);
    for (int id = 1; id <= 5; ++id) {
        mp_obj_t v = !(known & (1u << id)) ? mp_const_none : ((miss & (1u << id)) ? mp_const_false : mp_const_true);
        mp_obj_dict_store(d, mp_obj_new_str(names[id], strlen(names[id])), v);
        if (miss & (1u << id)) mp_obj_list_append(codes, mp_obj_new_int(id));
    }
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_led_codes), codes);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_led_ms_left), mp_obj_new_int_from_uint(sentai_health_led_remaining_ms()));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_sys_health_obj, mod_sentai_sys_health);

// System LittleFS (NAND boot partition) diagnostics -- see sentai_runtime.cc.
// sentai.sys.sysfs_size(path) -> int (bytes, or negative error)
// sentai.sys.sysfs_read(path, off, n) -> bytes (n <= 512)
// sentai.sys.sysfs_find(path, needle, start=0) -> int offset or -1
extern int sentai_sysfs_size(const char* path);
extern int sentai_sysfs_read(const char* path, uint32_t off, uint8_t* buf, uint32_t n);
extern int sentai_sysfs_find(const char* path, const uint8_t* needle, uint32_t nlen, uint32_t start);
static mp_obj_t mod_sentai_sys_sysfs_size(mp_obj_t path_o) {
    return mp_obj_new_int(sentai_sysfs_size(mp_obj_str_get_str(path_o)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_sys_sysfs_size_obj, mod_sentai_sys_sysfs_size);
static mp_obj_t mod_sentai_sys_sysfs_read(mp_obj_t path_o, mp_obj_t off_o, mp_obj_t n_o) {
    extern uint8_t g_sysfs_read_buf[512];   // SDRAM, sentai_runtime.cc
    uint8_t* buf = g_sysfs_read_buf;
    mp_int_t n = mp_obj_get_int(n_o);
    if (n < 0 || n > 512) mp_raise_ValueError(MP_ERROR_TEXT("n must be 0..512"));
    int r = sentai_sysfs_read(mp_obj_str_get_str(path_o), (uint32_t)mp_obj_get_int(off_o), buf, (uint32_t)n);
    if (r < 0) mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sysfs read failed"));
    return mp_obj_new_bytes(buf, (size_t)r);
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_sentai_sys_sysfs_read_obj, mod_sentai_sys_sysfs_read);
static mp_obj_t mod_sentai_sys_sysfs_find(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t nb;
    mp_get_buffer_raise(args[1], &nb, MP_BUFFER_READ);
    uint32_t start = (n_args >= 3) ? (uint32_t)mp_obj_get_int(args[2]) : 0u;
    return mp_obj_new_int(sentai_sysfs_find(mp_obj_str_get_str(args[0]), (const uint8_t*)nb.buf, (uint32_t)nb.len, start));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_sys_sysfs_find_obj, 2, 3, mod_sentai_sys_sysfs_find);

// sentai.sys.poke32(addr, value) → None
// Volatile 32-bit write.  BRING-UP DIAGNOSTIC ONLY (board rev 2026-10 pad /
// GPIO polarity checks); it can crash or brick the running image if misused.
// Not available on SIM.
static mp_obj_t mod_sentai_sys_poke32(mp_obj_t addr_obj, mp_obj_t val_obj) {
#ifdef SENTAI_PLATFORM_SIM
    (void)addr_obj; (void)val_obj;
    mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("poke32 not available on SIM"));
#else
    uint32_t addr = (uint32_t)mp_obj_get_int_truncated(addr_obj);
    if (addr & 3u) {
        mp_raise_ValueError(MP_ERROR_TEXT("poke32: address must be 4-byte aligned"));
    }
    *(volatile uint32_t *)addr = (uint32_t)mp_obj_get_int_truncated(val_obj);
    return mp_const_none;
#endif
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_sentai_sys_poke32_obj, mod_sentai_sys_poke32);

// ---- module table ----
static const mp_rom_map_elem_t sentai_sys_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),       MP_ROM_QSTR(MP_QSTR_sys) },
    { MP_ROM_QSTR(MP_QSTR_reset),          MP_ROM_PTR(&mod_sentai_sys_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_recovery_mode),  MP_ROM_PTR(&mod_sentai_sys_recovery_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_boot_attempts),  MP_ROM_PTR(&mod_sentai_sys_boot_attempts_obj) },
    { MP_ROM_QSTR(MP_QSTR_uid),            MP_ROM_PTR(&mod_sentai_sys_uid_obj) },
    { MP_ROM_QSTR(MP_QSTR_peek32),         MP_ROM_PTR(&mod_sentai_sys_peek32_obj) },
    { MP_ROM_QSTR(MP_QSTR_health),         MP_ROM_PTR(&mod_sentai_sys_health_obj) },
    { MP_ROM_QSTR(MP_QSTR_sysfs_size),     MP_ROM_PTR(&mod_sentai_sys_sysfs_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_sysfs_read),     MP_ROM_PTR(&mod_sentai_sys_sysfs_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_sysfs_find),     MP_ROM_PTR(&mod_sentai_sys_sysfs_find_obj) },
    { MP_ROM_QSTR(MP_QSTR_poke32),         MP_ROM_PTR(&mod_sentai_sys_poke32_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_sys_globals, sentai_sys_globals_table);
static const mp_obj_module_t sentai_sys_module = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_sys_globals,
};

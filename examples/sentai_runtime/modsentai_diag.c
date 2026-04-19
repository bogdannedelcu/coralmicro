// ============== sentai.diag — Diagnostics & Health ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Provides:
//   sentai.diag.health()     — subsystem health summary string
//   sentai.diag.sys_mode()   — overall system mode string

#include "sentai_health.h"

// ===================== Health summary =====================

// sentai.diag.health() -> multi-line health summary string (compact)
static mp_obj_t mod_sentai_diag_health(void) {
    char buf[384];
    int len = sentai_health_summary(buf, sizeof(buf));
    return mp_obj_new_str(buf, len);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_health_obj, mod_sentai_diag_health);

// ===================== System mode =====================

// sentai.diag.sys_mode() -> system mode string
static mp_obj_t mod_sentai_diag_sys_mode(void) {
    static const char* mode_names[] = {"BOOTING", "NORMAL", "DEGRADED", "SAFE", "RECOVERY"};
    SystemMode_t mode = sentai_health_system_mode();
    if (mode > SYS_MODE_RECOVERY) mode = SYS_MODE_RECOVERY;
    const char* name = mode_names[(int)mode];
    return mp_obj_new_str(name, strlen(name));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_sys_mode_obj, mod_sentai_diag_sys_mode);

// ===================== Crash log reader =====================

// sentai.diag.crash_log() -> str
// Returns the contents of the most recent crash log file as a string.
// Returns "" if no crash log exists.
// Capped at 8 KB to avoid large allocations.
static mp_obj_t mod_sentai_diag_crash_log(void) {
    extern void sentai_get_last_crash_log_path(char* out, size_t len);
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int max_size);

    char path[40];
    sentai_get_last_crash_log_path(path, sizeof(path));
    if (path[0] == '\0') return mp_obj_new_str("", 0);

    int size = sentai_fs_size(path);
    if (size <= 0) return mp_obj_new_str("", 0);
    // Cap to prevent large allocation
    if (size > 8192) size = 8192;

    uint8_t* buf = m_new(uint8_t, size);
    int n = sentai_fs_read(path, buf, size);
    mp_obj_t result = mp_obj_new_str((const char*)buf, n > 0 ? n : 0);
    m_del(uint8_t, buf, size);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_crash_log_obj, mod_sentai_diag_crash_log);

// ===================== Boot log reader =====================

// sentai.diag.boot_log() -> str
// Returns the contents of /log/boot.log (most recent boot).
// Falls back to /log/boot_old.log if current boot log is empty.
// Capped at 16 KB.
static mp_obj_t mod_sentai_diag_boot_log(void) {
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int max_size);

    static const char* const paths[] = {"/log/boot.log", "/log/boot_old.log"};

    for (int pi = 0; pi < 2; pi++) {
        int size = sentai_fs_size(paths[pi]);
        if (size <= 0) continue;
        if (size > 16384) size = 16384;
        uint8_t* buf = m_new(uint8_t, size);
        int n = sentai_fs_read(paths[pi], buf, size);
        mp_obj_t result = mp_obj_new_str((const char*)buf, n > 0 ? n : 0);
        m_del(uint8_t, buf, size);
        return result;
    }
    return mp_obj_new_str("", 0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_boot_log_obj, mod_sentai_diag_boot_log);

// ===================== In-RAM dmesg reader =====================
// sentai.diag.dmesg([max_bytes]) -> str
// Returns the in-RAM ring buffer contents (oldest-first). The ring is
// populated via sentai_dmesg() calls throughout the firmware and survives
// REPL resets, USB reconnects and command failures — cleared only by reboot.
// Default cap 16 KB; pass a smaller value to tail just the recent entries.
#include "sentai_dmesg.h"
static mp_obj_t mod_sentai_diag_dmesg(size_t n_args, const mp_obj_t *args) {
    int max_bytes = (n_args >= 1) ? mp_obj_get_int(args[0]) : 16384;
    if (max_bytes < 64)    max_bytes = 64;
    if (max_bytes > 65536) max_bytes = 65536;

    char* buf = m_new(char, max_bytes);
    size_t n = sentai_dmesg_read(buf, (size_t)max_bytes);
    mp_obj_t result = mp_obj_new_str(buf, n);
    m_del(char, buf, max_bytes);
    return result;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_dmesg_obj,
                                            0, 1, mod_sentai_diag_dmesg);

// sentai.diag.dmesg_clear() -> None
static mp_obj_t mod_sentai_diag_dmesg_clear(void) {
    sentai_dmesg_clear();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_dmesg_clear_obj,
                                  mod_sentai_diag_dmesg_clear);

// sentai.diag.dmesg_stats() -> (bytes_used, bytes_dropped)
static mp_obj_t mod_sentai_diag_dmesg_stats(void) {
    mp_obj_t items[2] = {
        mp_obj_new_int_from_uint((uint32_t)sentai_dmesg_used()),
        mp_obj_new_int_from_uint(sentai_dmesg_dropped()),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_dmesg_stats_obj,
                                  mod_sentai_diag_dmesg_stats);

// ---- module table ----
static const mp_rom_map_elem_t sentai_diag_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),   MP_ROM_QSTR(MP_QSTR_diag) },
    { MP_ROM_QSTR(MP_QSTR_health),     MP_ROM_PTR(&mod_sentai_diag_health_obj) },
    { MP_ROM_QSTR(MP_QSTR_sys_mode),   MP_ROM_PTR(&mod_sentai_diag_sys_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_crash_log),  MP_ROM_PTR(&mod_sentai_diag_crash_log_obj) },
    { MP_ROM_QSTR(MP_QSTR_boot_log),   MP_ROM_PTR(&mod_sentai_diag_boot_log_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmesg),      MP_ROM_PTR(&mod_sentai_diag_dmesg_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmesg_clear), MP_ROM_PTR(&mod_sentai_diag_dmesg_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmesg_stats), MP_ROM_PTR(&mod_sentai_diag_dmesg_stats_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_diag_globals, sentai_diag_globals_table);
static const mp_obj_module_t sentai_diag_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_diag_globals,
};

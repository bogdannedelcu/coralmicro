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

// sentai.diag.cam_stats() -> dict of camera-switch fault counters.
//
// Persistent breadcrumbs accumulated by sentai_cam_switch and
// sentai_cam_get_raw_with_recovery since boot.  An operator can poll
// this to see whether a degraded path is being exercised silently
// (e.g. fallback count > 0 means EOF ISR occasionally does not
// consume the arm; drain_timeout > 0 means post-switch wait hit the
// 300 ms ceiling).  All counters cleared on reboot; no runtime clear
// API on purpose — field-diagnostic semantics require monotonic
// counters so a late connection still sees the cumulative history.
//
// See sentai_error.h, section "Camera Errors (0x0Axx)" for the error
// codes emitted via SERR_LOG at each event.
extern void sentai_cam_stats_get(uint32_t* ok_eof, uint32_t* fallback,
                                 uint32_t* drain_timeout,
                                 uint32_t* grab_retry, uint32_t* grab_fatal);
static mp_obj_t mod_sentai_diag_cam_stats(void) {
    uint32_t ok_eof = 0, fallback = 0, drain_timeout = 0,
             grab_retry = 0, grab_fatal = 0;
    sentai_cam_stats_get(&ok_eof, &fallback, &drain_timeout,
                         &grab_retry, &grab_fatal);
    mp_obj_t d = mp_obj_new_dict(5);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_switch_ok_eof),
                      mp_obj_new_int_from_uint(ok_eof));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_switch_fallback),
                      mp_obj_new_int_from_uint(fallback));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_drain_timeout),
                      mp_obj_new_int_from_uint(drain_timeout));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_grab_retry),
                      mp_obj_new_int_from_uint(grab_retry));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_grab_fatal),
                      mp_obj_new_int_from_uint(grab_fatal));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_cam_stats_obj,
                                  mod_sentai_diag_cam_stats);

// sentai.diag.lfs_stats() — HTTP ↔ lfs_task state-machine counters.
// Every `lfs_busy` response increments exactly one `busy_*` field;
// check them after a failing GET to learn which branch fired and why.
extern void sentai_lfs_stats_get(uint32_t* out, int max_fields);
static mp_obj_t mod_sentai_diag_lfs_stats(void) {
    uint32_t s[11] = {0};
    sentai_lfs_stats_get(s, 11);
    mp_obj_t d = mp_obj_new_dict(11);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_served_ready_cached),
                      mp_obj_new_int_from_uint(s[0]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_served_fast_raw),
                      mp_obj_new_int_from_uint(s[1]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_served_after_wait),
                      mp_obj_new_int_from_uint(s[2]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_busy_serving_prev),
                      mp_obj_new_int_from_uint(s[3]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_busy_fast_raw_mutex),
                      mp_obj_new_int_from_uint(s[4]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_busy_slow_timeout),
                      mp_obj_new_int_from_uint(s[5]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_busy_slow_state_drift),
                      mp_obj_new_int_from_uint(s[6]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_busy_path_mismatch),
                      mp_obj_new_int_from_uint(s[7]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_busy_not_inited),
                      mp_obj_new_int_from_uint(s[8]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_enqueue_ok),
                      mp_obj_new_int_from_uint(s[9]));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_enqueue_fail),
                      mp_obj_new_int_from_uint(s[10]));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_lfs_stats_obj,
                                  mod_sentai_diag_lfs_stats);

// sentai.diag.tpu_perf([reset]) — per-Invoke split-cycle counters
// accumulated inside EdgeTpuExecutable::Invoke.  Reads the five
// DWT-cycle accumulators (params, ins, input, output, event), their
// call counts, and bytes-moved totals.  Call with reset=True to zero
// the accumulators so the next measurement starts clean.
//
// DWT @ 800 MHz → 800 cycles / µs → 800000 cycles / ms.
extern volatile uint32_t g_sentai_tpu_cyc_params;
extern volatile uint32_t g_sentai_tpu_cyc_ins;
extern volatile uint32_t g_sentai_tpu_cyc_input;
extern volatile uint32_t g_sentai_tpu_cyc_output;
extern volatile uint32_t g_sentai_tpu_cyc_event;
extern volatile uint32_t g_sentai_tpu_n_params;
extern volatile uint32_t g_sentai_tpu_n_ins;
extern volatile uint32_t g_sentai_tpu_n_input;
extern volatile uint32_t g_sentai_tpu_n_output;
extern volatile uint32_t g_sentai_tpu_n_event;
extern volatile uint32_t g_sentai_tpu_by_params;
extern volatile uint32_t g_sentai_tpu_by_ins;
extern volatile uint32_t g_sentai_tpu_by_input;
extern volatile uint32_t g_sentai_tpu_by_output;
extern void sentai_tpu_perf_reset(void);

static mp_obj_t mod_sentai_diag_tpu_perf(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 1 && mp_obj_is_true(args[0])) {
        sentai_tpu_perf_reset();
    }
    mp_obj_t d = mp_obj_new_dict(15);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cyc_params),
                      mp_obj_new_int_from_uint(g_sentai_tpu_cyc_params));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cyc_ins),
                      mp_obj_new_int_from_uint(g_sentai_tpu_cyc_ins));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cyc_input),
                      mp_obj_new_int_from_uint(g_sentai_tpu_cyc_input));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cyc_output),
                      mp_obj_new_int_from_uint(g_sentai_tpu_cyc_output));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cyc_event),
                      mp_obj_new_int_from_uint(g_sentai_tpu_cyc_event));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n_params),
                      mp_obj_new_int_from_uint(g_sentai_tpu_n_params));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n_ins),
                      mp_obj_new_int_from_uint(g_sentai_tpu_n_ins));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n_input),
                      mp_obj_new_int_from_uint(g_sentai_tpu_n_input));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n_output),
                      mp_obj_new_int_from_uint(g_sentai_tpu_n_output));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n_event),
                      mp_obj_new_int_from_uint(g_sentai_tpu_n_event));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_by_params),
                      mp_obj_new_int_from_uint(g_sentai_tpu_by_params));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_by_ins),
                      mp_obj_new_int_from_uint(g_sentai_tpu_by_ins));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_by_input),
                      mp_obj_new_int_from_uint(g_sentai_tpu_by_input));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_by_output),
                      mp_obj_new_int_from_uint(g_sentai_tpu_by_output));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_perf_obj,
                                            0, 1, mod_sentai_diag_tpu_perf);

// Per-transfer async submit/callback counters — diagnose the async path
// health.  Mirrors g_edgetpu_async_* in libs/tpu/usb_host_edgetpu.c.
extern volatile uint32_t g_edgetpu_async_submit_ok;
extern volatile uint32_t g_edgetpu_async_submit_fail;
extern volatile uint32_t g_edgetpu_async_cb_fired;
extern volatile uint32_t g_edgetpu_async_cb_ok;
extern volatile uint32_t g_edgetpu_async_cb_fail;
extern volatile uint32_t g_edgetpu_bo_fail_pipe_idx;
extern volatile uint32_t g_edgetpu_bo_fail_not_bulk;
extern volatile uint32_t g_edgetpu_bo_fail_malloc;
extern volatile uint32_t g_edgetpu_bo_fail_send;
extern volatile uint32_t g_edgetpu_bo_ok;
extern volatile uint32_t g_edgetpu_legacy_cb_entered;
extern volatile uint32_t g_edgetpu_legacy_cb_found_pipe;
extern volatile uint32_t g_edgetpu_legacy_cb_called_user;
extern volatile uint32_t g_edgetpu_legacy_cb_no_pipe;
extern volatile uint32_t g_sentai_tpu_lambda_entered;
extern volatile uint32_t g_sentai_tpu_lambda_gave;
extern volatile uint32_t g_sentai_tpu_lambda_null_sema;
extern volatile uint32_t g_sentai_tpu_take_failed;
extern volatile uint32_t g_sentai_tpu_take_succeeded;
extern volatile uint32_t g_sentai_tpu_urb_cancelled;
extern volatile uint32_t g_sentai_tpu_urb_cancel_no_cb;

static mp_obj_t mod_sentai_diag_async_stats(void) {
    mp_obj_t d = mp_obj_new_dict(14);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_submit_ok),
                      mp_obj_new_int_from_uint(g_edgetpu_async_submit_ok));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_submit_fail),
                      mp_obj_new_int_from_uint(g_edgetpu_async_submit_fail));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cb_fired),
                      mp_obj_new_int_from_uint(g_edgetpu_async_cb_fired));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cb_ok),
                      mp_obj_new_int_from_uint(g_edgetpu_async_cb_ok));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cb_fail),
                      mp_obj_new_int_from_uint(g_edgetpu_async_cb_fail));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_bo_ok),
                      mp_obj_new_int_from_uint(g_edgetpu_bo_ok));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_bo_pipe),
                      mp_obj_new_int_from_uint(g_edgetpu_bo_fail_pipe_idx));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_bo_bulk),
                      mp_obj_new_int_from_uint(g_edgetpu_bo_fail_not_bulk));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_bo_malloc),
                      mp_obj_new_int_from_uint(g_edgetpu_bo_fail_malloc));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_bo_send),
                      mp_obj_new_int_from_uint(g_edgetpu_bo_fail_send));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cb_entered),
                      mp_obj_new_int_from_uint(g_edgetpu_legacy_cb_entered));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cb_found),
                      mp_obj_new_int_from_uint(g_edgetpu_legacy_cb_found_pipe));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cb_user),
                      mp_obj_new_int_from_uint(g_edgetpu_legacy_cb_called_user));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cb_nopipe),
                      mp_obj_new_int_from_uint(g_edgetpu_legacy_cb_no_pipe));
    // Dynamic-key entries (no QSTR regen needed for new counters)
    mp_obj_dict_store(d, mp_obj_new_str("lambda_entered", 14),
                      mp_obj_new_int_from_uint(g_sentai_tpu_lambda_entered));
    mp_obj_dict_store(d, mp_obj_new_str("lambda_gave", 11),
                      mp_obj_new_int_from_uint(g_sentai_tpu_lambda_gave));
    mp_obj_dict_store(d, mp_obj_new_str("lambda_null_sema", 16),
                      mp_obj_new_int_from_uint(g_sentai_tpu_lambda_null_sema));
    mp_obj_dict_store(d, mp_obj_new_str("take_ok", 7),
                      mp_obj_new_int_from_uint(g_sentai_tpu_take_succeeded));
    mp_obj_dict_store(d, mp_obj_new_str("take_timeout", 12),
                      mp_obj_new_int_from_uint(g_sentai_tpu_take_failed));
    mp_obj_dict_store(d, mp_obj_new_str("urb_cancelled", 13),
                      mp_obj_new_int_from_uint(g_sentai_tpu_urb_cancelled));
    mp_obj_dict_store(d, mp_obj_new_str("urb_cancel_no_cb", 16),
                      mp_obj_new_int_from_uint(g_sentai_tpu_urb_cancel_no_cb));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_async_stats_obj,
                                  mod_sentai_diag_async_stats);

// sentai.diag.tpu_async_input([enable]) — route SendInputs through the
// pipelined BulkOutTransferPipelined path (2 URBs in flight) instead
// of the serial legacy BulkOutTransfer.  Only affects the input-
// activations phase; params/instructions stay on the legacy path.
extern int  sentai_tpu_async_input_get(void);
extern void sentai_tpu_async_input_set(int v);
static mp_obj_t mod_sentai_diag_tpu_async_input(size_t n_args,
                                                const mp_obj_t *args) {
    if (n_args >= 1) {
        sentai_tpu_async_input_set(mp_obj_is_true(args[0]) ? 1 : 0);
    }
    return mp_obj_new_int(sentai_tpu_async_input_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_async_input_obj,
                                            0, 1, mod_sentai_diag_tpu_async_input);

// sentai.diag.tpu_desc_cache([enable]) — skip re-uploading parameters
// AND instructions on subsequent invokes when the (executable, token)
// pair matches the previous invoke.  Experimental — the TPU
// instruction FIFO is expected to retain the last-uploaded stream
// between invokes for the same model, but this relies on TPU-side
// behaviour not fully documented.  Start with verbose(1) and a
// single invoke to validate output correctness before looping.
extern volatile int g_sentai_tpu_desc_cache_enabled;
extern volatile uint32_t g_sentai_tpu_desc_cache_sent_params;
extern volatile uint32_t g_sentai_tpu_desc_cache_sent_ins;
extern volatile uint32_t g_sentai_tpu_desc_cache_skip_params;
extern volatile uint32_t g_sentai_tpu_desc_cache_skip_ins;
extern void sentai_tpu_desc_cache_invalidate(void);

static mp_obj_t mod_sentai_diag_tpu_desc_cache(size_t n_args,
                                                const mp_obj_t *args) {
    if (n_args >= 1) {
        int want = mp_obj_is_true(args[0]) ? 1 : 0;
        // Invalidate old key when flipping state — otherwise the first
        // invoke after re-enable would hit a stale cache.
        sentai_tpu_desc_cache_invalidate();
        g_sentai_tpu_desc_cache_enabled = want;
    }
    mp_obj_t d = mp_obj_new_dict(5);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_enabled),
                      mp_obj_new_bool(g_sentai_tpu_desc_cache_enabled != 0));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sent_params),
                      mp_obj_new_int_from_uint(g_sentai_tpu_desc_cache_sent_params));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sent_ins),
                      mp_obj_new_int_from_uint(g_sentai_tpu_desc_cache_sent_ins));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_skip_params),
                      mp_obj_new_int_from_uint(g_sentai_tpu_desc_cache_skip_params));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_skip_ins),
                      mp_obj_new_int_from_uint(g_sentai_tpu_desc_cache_skip_ins));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_desc_cache_obj,
                                            0, 1, mod_sentai_diag_tpu_desc_cache);

// sentai.diag.tpu_chunk_size([n]) — runtime-tunable bulk chunk size.
// Clamped to [4096, 160*1024].  Default 64 KB; empirical sweet spot
// on our EHCI host + YOLO 512 workload.  Bigger chunks amortize URB
// submit overhead but cost DCACHE_CleanByRange time; run a sweep via
// sentai.diag.tpu_chunk_size(n) in MicroPython without reflashing.
extern uint32_t sentai_tpu_chunk_size_get(void);
extern void     sentai_tpu_chunk_size_set(uint32_t n);
static mp_obj_t mod_sentai_diag_tpu_chunk_size(size_t n_args,
                                                const mp_obj_t *args) {
    if (n_args >= 1) {
        sentai_tpu_chunk_size_set((uint32_t)mp_obj_get_int(args[0]));
    }
    return mp_obj_new_int_from_uint(sentai_tpu_chunk_size_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_chunk_size_obj,
                                            0, 1, mod_sentai_diag_tpu_chunk_size);


// sentai.diag.tpu_zero_copy([enable]) — toggle zero-copy for INPUT
// phase of TPU invoke.  Default ON (fastest, standalone-safe).
// Turn OFF when running the camera pipeline (direct_tensor mode)
// if you observe `E:0420:2` TPU invoke failures — staged DTCM
// path sidesteps cache races with concurrent PXP/quant writers.
extern int  sentai_tpu_zero_copy_input_get(void);
extern void sentai_tpu_zero_copy_input_set(int v);
static mp_obj_t mod_sentai_diag_tpu_zero_copy(size_t n_args,
                                               const mp_obj_t *args) {
    if (n_args >= 1) {
        sentai_tpu_zero_copy_input_set(mp_obj_is_true(args[0]) ? 1 : 0);
    }
    return mp_obj_new_int(sentai_tpu_zero_copy_input_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_zero_copy_obj,
                                            0, 1, mod_sentai_diag_tpu_zero_copy);

// sentai.diag.tpu_urb_timeout([ms]) — per-URB wait ceiling.  Default
// 50 ms.  Below 5 ms is rejected to avoid spurious timeouts under
// normal load; above 5000 ms capped to preserve fault-tolerance.
// When a URB exceeds this cap, the driver cancels it + returns
// -1 so InferTask can skip the frame and continue.
extern uint32_t sentai_tpu_urb_timeout_ms_get(void);
extern void     sentai_tpu_urb_timeout_ms_set(uint32_t n);
static mp_obj_t mod_sentai_diag_tpu_urb_timeout(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 1) sentai_tpu_urb_timeout_ms_set((uint32_t)mp_obj_get_int(args[0]));
    return mp_obj_new_int_from_uint(sentai_tpu_urb_timeout_ms_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_urb_timeout_obj,
                                            0, 1, mod_sentai_diag_tpu_urb_timeout);

// sentai.diag.tpu_multi_ep([enable]) — arm per-tag EP routing.  Only
// meaningful when the firmware was built with SENTAI_TPU_MULTI_EP=ON
// (see libs/tpu/CMakeLists.txt:36) AND the TPU was DFU'd with
// apex_latest_multi_ep_bin.  When enabled, the TPU driver routes
// parameters/instructions/input-activations to EP3/EP1/EP2 instead of
// multiplexing them on EP1, and writes the multi_bo_ep CSR on first
// use.  Concurrent URBs on DIFFERENT pipes are safe by construction
// because the NXP per-pipe callback state collision doesn't happen
// across pipes.
extern int  sentai_tpu_multi_ep_routing_get(void);
extern void sentai_tpu_multi_ep_routing_set(int v);
static mp_obj_t mod_sentai_diag_tpu_multi_ep(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 1) {
        sentai_tpu_multi_ep_routing_set(mp_obj_is_true(args[0]) ? 1 : 0);
    }
    return mp_obj_new_int(sentai_tpu_multi_ep_routing_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_multi_ep_obj,
                                            0, 1, mod_sentai_diag_tpu_multi_ep);

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
    { MP_ROM_QSTR(MP_QSTR_cam_stats),  MP_ROM_PTR(&mod_sentai_diag_cam_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_lfs_stats),  MP_ROM_PTR(&mod_sentai_diag_lfs_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_perf),   MP_ROM_PTR(&mod_sentai_diag_tpu_perf_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_multi_ep), MP_ROM_PTR(&mod_sentai_diag_tpu_multi_ep_obj) },
    { MP_ROM_QSTR(MP_QSTR_async_stats), MP_ROM_PTR(&mod_sentai_diag_async_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_async_input), MP_ROM_PTR(&mod_sentai_diag_tpu_async_input_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_desc_cache),  MP_ROM_PTR(&mod_sentai_diag_tpu_desc_cache_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_chunk_size),  MP_ROM_PTR(&mod_sentai_diag_tpu_chunk_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_zero_copy),   MP_ROM_PTR(&mod_sentai_diag_tpu_zero_copy_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_urb_timeout), MP_ROM_PTR(&mod_sentai_diag_tpu_urb_timeout_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_diag_globals, sentai_diag_globals_table);
static const mp_obj_module_t sentai_diag_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_diag_globals,
};

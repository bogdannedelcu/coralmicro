// ============== sentai.diag — Diagnostics & Health ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Provides:
//   sentai.diag.health()     — subsystem health summary string
//   sentai.diag.sys_mode()   — overall system mode string

#include "sentai_health.h"

/* Forward declarations for the FileX/LevelX user-FS diagnostics.
 * Mirrors libs/base/fx_user_fs.h; kept local so the QSTR-regen
 * mini-gcc (which runs with a narrower include path) can preprocess
 * this TU.  The destructive smoke runner from Phase 1 is gone — the
 * FS is now persistent, so we only expose non-destructive bench +
 * stats here. */
typedef struct {
    int      ok;
    uint32_t entries;
    uint32_t time_list_ms;
    uint32_t time_size_ms;
} fx_bench_result_t;
typedef struct {
    uint32_t mounted;
    uint32_t free_clusters;
    uint32_t total_clusters;
    uint32_t bytes_per_sector;
    uint32_t sectors_per_cluster;
    uint32_t mount_failures;
    uint32_t format_count;
} fx_user_stats_t;
extern int  FxUserBenchRoot(fx_bench_result_t* out);
extern void FxUserGetStats(fx_user_stats_t* out);
extern int  FxUserIsMounted(void);
extern int  FxUserInit(int force_format);
extern void fx_nand_driver_get_stats(uint32_t* reads, uint32_t* writes,
                                      uint32_t* erases,
                                      uint32_t* read_errors,
                                      uint32_t* write_errors,
                                      uint32_t* erase_errors,
                                      uint32_t* bad_blocks);

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
extern void sentai_fs_stats_get(uint32_t* out, int max_fields);
static mp_obj_t mod_sentai_diag_lfs_stats(void) {
    uint32_t s[11] = {0};
    sentai_fs_stats_get(s, 11);
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

// sentai.diag.tpu_call_stats([reset]) — raw call counters for
// SendParameters/SendInstructions/SendInputs.  Each Send* method
// increments these once per call (regardless of cache/skip
// behaviour).  Lets us diff "how many calls did config X make per
// invoke" between A/B/C runs without speculation.
extern void sentai_tpu_call_stats(uint32_t* p_calls, uint32_t* p_bytes,
                                   uint32_t* i_calls, uint32_t* i_bytes,
                                   uint32_t* in_calls, uint32_t* in_bytes,
                                   uint32_t* in_done);
extern void sentai_tpu_call_reset(void);
static mp_obj_t mod_sentai_diag_tpu_call_stats(size_t n_args,
                                                const mp_obj_t *args) {
    if (n_args >= 1 && mp_obj_is_true(args[0])) sentai_tpu_call_reset();
    uint32_t pc=0, pb=0, ic=0, ib=0, inc=0, inb=0, ind=0;
    sentai_tpu_call_stats(&pc, &pb, &ic, &ib, &inc, &inb, &ind);
    mp_obj_t d = mp_obj_new_dict(7);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p_calls),  mp_obj_new_int_from_uint(pc));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_p_bytes),  mp_obj_new_int_from_uint(pb));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_i_calls),  mp_obj_new_int_from_uint(ic));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_i_bytes),  mp_obj_new_int_from_uint(ib));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_in_calls), mp_obj_new_int_from_uint(inc));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_in_bytes), mp_obj_new_int_from_uint(inb));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_in_done),  mp_obj_new_int_from_uint(ind));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_call_stats_obj,
                                            0, 1, mod_sentai_diag_tpu_call_stats);

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

// sentai.diag.tpu_trace([0|1]) — toggle per-stage USB trace prints
// inside EdgeTpuExecutable::Invoke + BulkInTransferInternal.  Used
// when adding a new model to inspect the dma_hints order, per-chunk
// USB-IN sizes, and URB completion status.  Costs ~1 printf per
// hint + per chunk when enabled.
extern volatile uint8_t g_sentai_tpu_trace;
static mp_obj_t mod_sentai_diag_tpu_trace(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 1) g_sentai_tpu_trace = (uint8_t)(mp_obj_get_int(args[0]) ? 1 : 0);
    return mp_obj_new_int_from_uint(g_sentai_tpu_trace);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_diag_tpu_trace_obj,
                                            0, 1, mod_sentai_diag_tpu_trace);

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

// sentai.diag.repl_kick() -> None
//
// Bumps the REPL-activity timestamp explicitly.  The combined watchdog
// task counts >120 s of REPL silence as "dead" and stops kicking
// WDOG1 -- which then resets the board ~30 s later.  A long-running
// driver that legitimately holds the REPL for several minutes must
// call this from inside its outer loop to extend the deadline.
//
// (The REPL task already auto-bumps the timestamp every 5 s while a
// Python script is running -- see micropython_task.c.  This binding
// is a defense-in-depth: if a driver disables the auto-heartbeat
// path or runs in a context where it doesn't fire, an explicit
// keep-alive every <60 s prevents the dead-threshold from tripping.)
extern void sentai_repl_activity(void);
static mp_obj_t mod_sentai_diag_repl_kick(void) {
    sentai_repl_activity();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_repl_kick_obj,
                                  mod_sentai_diag_repl_kick);

// ===================== FlexRAM partition probe =====================
//
// Returns the LIVE FlexRAM bank configuration as a dict.  The RT1176
// FlexRAM is 16 banks × 32 KB = 512 KB; each bank can be ITCM, DTCM,
// or OCRAM (encoded 2 bits per bank in GPR17/GPR18, source selected
// by GPR16.FLEXRAM_BANK_CFG_SEL: 0=eFuse, 1=GPR17/18).
//
// Per RT1170RM Table 38-2 (FLEXRAM_BANK_CFG):
//   00 = bank not used (reserved)
//   01 = OCRAM
//   10 = DTCM
//   11 = ITCM
//
// We expose:
//   src       : 'efuse' (GPR16.SEL=0) or 'gpr17_18' (SEL=1)
//   itcm_kb   : count of ITCM banks × 32 KB
//   dtcm_kb   : count of DTCM banks × 32 KB
//   ocram_kb  : count of OCRAM banks × 32 KB (FlexRAM share only;
//               OCRAM1+OCRAM2 dedicated 1 MB are NOT counted here)
//   gpr17     : raw 32-bit value at IOMUXC_GPR.GPR17 (low 16 banks)
//   gpr18     : raw 32-bit value at IOMUXC_GPR.GPR18 (high 16 banks)
//   gpr16_sel : 1 if GPR17/18 active, 0 if eFuse active
/* Direct MMIO access — IOMUXC_GPR base @ 0x400E_4000 (RT1170RM
 * Table 11-7).  Avoids pulling fsl_iomuxc.h into the QSTR-scanner
 * preprocess path (the scanner doesn't have the SDK include dirs).
 *   GPR16 @ +0x40
 *   GPR17 @ +0x44
 *   GPR18 @ +0x48
 *   GPR16.FLEXRAM_BANK_CFG_SEL = bit 2 (0x4). */
static mp_obj_t mod_sentai_diag_flexram_info(void) {
    volatile uint32_t* gpr16_p = (volatile uint32_t*)0x400E4040u;
    volatile uint32_t* gpr17_p = (volatile uint32_t*)0x400E4044u;
    volatile uint32_t* gpr18_p = (volatile uint32_t*)0x400E4048u;
    const uint32_t gpr16 = *gpr16_p;
    const uint32_t gpr17 = *gpr17_p;
    const uint32_t gpr18 = *gpr18_p;
    const uint32_t sel   = (gpr16 & 0x4u) ? 1u : 0u;

    /* Decode the bank assignment.  When SEL=0, GPR17/18 are not
     * authoritative — the eFuse-driven config governs the actual
     * FlexRAM partitioning.  We still report the GPR17/18 raw bits
     * so the caller sees what they would BECOME if SEL flips. */
    const uint32_t cfg_low  = gpr17 & 0xFFFFu;  /* banks  0-7 */
    const uint32_t cfg_high = gpr18 & 0xFFFFu;  /* banks  8-15 */

    /* RT1170 fsl_flexram_allocate.h encoding (Table 38-2):
     *   00 = NotUsed    01 = OCRAM    10 = DTCM    11 = ITCM */
    uint32_t itcm = 0, dtcm = 0, ocram = 0, unused = 0;
    for (int b = 0; b < 16; ++b) {
        const uint32_t code = (b < 8)
            ? ((cfg_low  >> (b * 2)) & 0x3u)
            : ((cfg_high >> ((b - 8) * 2)) & 0x3u);
        if      (code == 0x3u) itcm++;
        else if (code == 0x2u) dtcm++;
        else if (code == 0x1u) ocram++;
        else                   unused++;
    }

    /* Address-fault probe: confirm the linker-stated regions are
     * actually live by reading the first word of each region.  A
     * fault here would reset the board (BusFault -> WDOG), so only
     * probe addresses we KNOW are mapped per the linker script.
     * If the eFuse delivered a different partition than the linker
     * expects, the first probe to .text or .bss would have already
     * crashed at startup — by the time we reach this MP function,
     * ITCM and DTCM are guaranteed live. */
    volatile uint32_t* itcm_probe  = (volatile uint32_t*)0x00000c00u;
    volatile uint32_t* dtcm_probe  = (volatile uint32_t*)0x20000000u;
    volatile uint32_t* ocram_probe = (volatile uint32_t*)0x20240000u;
    /* Reads are throwaway; the act of reading proves the region is
     * mapped (mis-config would BusFault at the load instruction). */
    (void)*itcm_probe;
    (void)*dtcm_probe;
    (void)*ocram_probe;

    mp_obj_t d = mp_obj_new_dict(0);
    /* When SEL=0, GPR17/18 are NOT authoritative — the eFuse default
     * (typically 256 KB ITCM + 256 KB DTCM + 0 OCRAM for RT1176) is
     * the live config.  The decoded itcm_kb/dtcm_kb/ocram_kb_flexram
     * fields in this dict reflect ONLY the GPR17/18 contents, so when
     * SEL=0 they all read 0.  Use the linker-stated regions plus the
     * effective_* fields below for the truth. */
    const uint32_t eff_itcm = sel ? (itcm * 32u)  : 256u;
    const uint32_t eff_dtcm = sel ? (dtcm * 32u)  : 256u;
    const uint32_t eff_flx_ocram = sel ? (ocram * 32u) : 0u;
    const char* src_name = sel ? "gpr17_18" : "efuse";
    mp_obj_dict_store(d, mp_obj_new_str("src", 3),
                      mp_obj_new_str(src_name, strlen(src_name)));
    mp_obj_dict_store(d, mp_obj_new_str("gpr16_sel", 9),
                      mp_obj_new_int_from_uint(sel));
    mp_obj_dict_store(d, mp_obj_new_str("gpr17", 5),
                      mp_obj_new_int_from_uint(gpr17));
    mp_obj_dict_store(d, mp_obj_new_str("gpr18", 5),
                      mp_obj_new_int_from_uint(gpr18));
    mp_obj_dict_store(d, mp_obj_new_str("itcm_kb", 7),
                      mp_obj_new_int_from_uint(itcm * 32u));
    mp_obj_dict_store(d, mp_obj_new_str("dtcm_kb", 7),
                      mp_obj_new_int_from_uint(dtcm * 32u));
    mp_obj_dict_store(d, mp_obj_new_str("ocram_kb_flexram", 16),
                      mp_obj_new_int_from_uint(ocram * 32u));
    mp_obj_dict_store(d, mp_obj_new_str("unused_banks", 12),
                      mp_obj_new_int_from_uint(unused));
    /* Effective active config — TRUSTED: respects SEL and falls back
     * to the silicon eFuse default (256/256/0) when SEL=0. */
    mp_obj_dict_store(d, mp_obj_new_str("effective_itcm_kb", 17),
                      mp_obj_new_int_from_uint(eff_itcm));
    mp_obj_dict_store(d, mp_obj_new_str("effective_dtcm_kb", 17),
                      mp_obj_new_int_from_uint(eff_dtcm));
    mp_obj_dict_store(d, mp_obj_new_str("effective_flx_ocram_kb", 22),
                      mp_obj_new_int_from_uint(eff_flx_ocram));
    /* Total OCRAM = FlexRAM share + dedicated OCRAM1+OCRAM2 (1 MB).
     * Doc-stated 1.25 MB requires effective_flx_ocram_kb == 256. */
    mp_obj_dict_store(d, mp_obj_new_str("effective_ocram_kb_total", 24),
                      mp_obj_new_int_from_uint(eff_flx_ocram + 1024u));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_flexram_info_obj,
                                  mod_sentai_diag_flexram_info);

// =====================================================================
// sentai.diag.fx_bench()  — non-destructive root listing latency probe.
//   Returns dict with entries-count + time_list_ms.
//   Usage: r = sentai.diag.fx_bench(); print(r["entries"], r["time_list_ms"])
//
// sentai.diag.fx_stats() — combined media + NAND BD adapter counters.
//
// sentai.diag.fx_format(magic) — DESTRUCTIVE force-reformat of the user
//   partition.  Caller MUST pass 0xDEADBEEF; otherwise no-op.
// =====================================================================
static mp_obj_t mod_sentai_diag_fx_bench(void) {
    fx_bench_result_t r;
    int rc = FxUserBenchRoot(&r);
    (void)rc;
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_ok), mp_obj_new_bool(r.ok));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_entries),
                      mp_obj_new_int_from_uint(r.entries));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_time_list_ms),
                      mp_obj_new_int_from_uint(r.time_list_ms));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_fx_bench_obj,
                                  mod_sentai_diag_fx_bench);

static mp_obj_t mod_sentai_diag_fx_stats(void) {
    uint32_t reads, writes, erases;
    uint32_t read_errors, write_errors, erase_errors, bad_blocks;
    fx_nand_driver_get_stats(&reads, &writes, &erases,
                             &read_errors, &write_errors,
                             &erase_errors, &bad_blocks);
    fx_user_stats_t st = {0};
    FxUserGetStats(&st);
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_mounted),
                      mp_obj_new_bool(st.mounted));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_free_clusters),
                      mp_obj_new_int_from_uint(st.free_clusters));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_total_clusters),
                      mp_obj_new_int_from_uint(st.total_clusters));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_bytes_per_sector),
                      mp_obj_new_int_from_uint(st.bytes_per_sector));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sectors_per_cluster),
                      mp_obj_new_int_from_uint(st.sectors_per_cluster));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_mount_failures),
                      mp_obj_new_int_from_uint(st.mount_failures));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_format_count),
                      mp_obj_new_int_from_uint(st.format_count));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_reads),
                      mp_obj_new_int_from_uint(reads));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_writes),
                      mp_obj_new_int_from_uint(writes));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_erases),
                      mp_obj_new_int_from_uint(erases));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_read_errors),
                      mp_obj_new_int_from_uint(read_errors));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_write_errors),
                      mp_obj_new_int_from_uint(write_errors));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_erase_errors),
                      mp_obj_new_int_from_uint(erase_errors));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_bad_blocks),
                      mp_obj_new_int_from_uint(bad_blocks));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_fx_stats_obj,
                                  mod_sentai_diag_fx_stats);

/* sentai.diag.storage_log() — read the /log/storage_debug.log content
 * captured during the previous storage-mode session (when REPL was
 * disconnected because USB MSC took over).  Capped at 16 KB. */
extern int sentai_fs_size_path(const char* path);  /* see modsentai_hal */
extern int sentai_fs_read_path(const char* path, uint8_t* buf, int max_size);
static mp_obj_t mod_sentai_diag_storage_log(void) {
    /* Use the existing modsentai_hal helpers via sentai.fs.* C API. */
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int max_size);
    const char* path = "/log/storage_debug.log";
    int size = sentai_fs_size(path);
    if (size <= 0) return mp_obj_new_str("", 0);
    if (size > 16384) size = 16384;
    vstr_t vstr;
    vstr_init_len(&vstr, (size_t)size);
    int n = sentai_fs_read(path, (uint8_t*)vstr.buf, size);
    if (n <= 0) {
        vstr_clear(&vstr);
        return mp_obj_new_str("", 0);
    }
    vstr.len = (size_t)n;
    return mp_obj_new_str_from_vstr(&vstr);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_storage_log_obj,
                                  mod_sentai_diag_storage_log);

/* ===== Generic write-cache for the user FS (sentai_fs_cache.cc) =====
 * 256 KB SDRAM buffer, ONE active session at a time.  Lets diag
 * drivers append arbitrary bytes (CSV rows, raw frames, dumps) at
 * high rate without paying per-call FileX open/write/close cost.
 * Driver flushes ('save') to disk at the end of the run -- or
 * mid-run when cache_len() grows past a threshold the driver
 * defines.  See agent.md "Write-cache pattern" section for usage
 * rules and sentai_fs_cache.cc for the rationale.
 */
extern int sentai_fs_cache_open(const char* path);
extern int sentai_fs_cache_write(const uint8_t* data, int size);
extern int sentai_fs_cache_save(void);
extern int sentai_fs_cache_len(void);
extern int sentai_fs_cache_dropped(void);
extern int sentai_fs_cache_close(void);

static mp_obj_t mod_sentai_diag_cache_open(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(sentai_fs_cache_open(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_diag_cache_open_obj,
                                  mod_sentai_diag_cache_open);

/* Accepts str OR bytes / bytearray / memoryview -- mp_get_buffer
 * gives us a uniform read-only view either way.  No newline magic;
 * the caller adds '\n' to its rows if it wants line-oriented CSV. */
static mp_obj_t mod_sentai_diag_cache_write(mp_obj_t obj) {
    mp_buffer_info_t bufinfo;
    if (!mp_get_buffer(obj, &bufinfo, MP_BUFFER_READ)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "cache_write: expected str/bytes/bytearray"));
    }
    return mp_obj_new_int(sentai_fs_cache_write(
        (const uint8_t*)bufinfo.buf, (int)bufinfo.len));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_diag_cache_write_obj,
                                  mod_sentai_diag_cache_write);

static mp_obj_t mod_sentai_diag_cache_save(void) {
    return mp_obj_new_int(sentai_fs_cache_save());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_cache_save_obj,
                                  mod_sentai_diag_cache_save);

static mp_obj_t mod_sentai_diag_cache_len(void) {
    return mp_obj_new_int(sentai_fs_cache_len());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_cache_len_obj,
                                  mod_sentai_diag_cache_len);

static mp_obj_t mod_sentai_diag_cache_dropped(void) {
    return mp_obj_new_int(sentai_fs_cache_dropped());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_cache_dropped_obj,
                                  mod_sentai_diag_cache_dropped);

static mp_obj_t mod_sentai_diag_cache_close(void) {
    return mp_obj_new_int(sentai_fs_cache_close());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_cache_close_obj,
                                  mod_sentai_diag_cache_close);

/* See FX_DESTRUCTIVE_CONFIRM_MAGIC in libs/base/fx_user_fs.h — single source. */
static mp_obj_t mod_sentai_diag_fx_format(mp_obj_t magic_obj) {
    uint32_t magic = (uint32_t)mp_obj_get_int_truncated(magic_obj);
    if (magic != 0xDEADBEEFu) {  /* must match FX_DESTRUCTIVE_CONFIRM_MAGIC */
        return mp_obj_new_int(-22);  /* -EINVAL */
    }
    int ok = FxUserInit(/*force_format=*/1);
    return mp_obj_new_int(ok ? 0 : -5);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_diag_fx_format_obj,
                                  mod_sentai_diag_fx_format);

// ---- ArUco feasibility benchmark ----
// Runs adaptive threshold + edge filter on a synthetic 320×240 image
// and returns DWT cycle counts.  Used to validate if hand-rolled
// ArUco detection on M7 is feasible (vs companion-computer fallback).
typedef struct {
    uint32_t w, h;
    uint32_t scan_cyc, thresh_cyc, edge_cyc;
    uint32_t thresh_bradley_cyc;
    uint32_t thresh_separable_cyc;
    uint32_t thresh_pxp_cyc;
    uint32_t pxp_stat_before_start, pxp_ctrl_before_start;
    uint32_t pxp_stat_after_wait,   pxp_ctrl_after_wait;
    uint32_t pxp_wait_iters;
} aruco_bench_result_t;
extern void aruco_bench_run(aruco_bench_result_t* out);

static mp_obj_t mod_sentai_diag_aruco_bench(void) {
    aruco_bench_result_t r = {0};
    aruco_bench_run(&r);
    // M7 @ 800 MHz: cycles / 800 = microseconds.
    mp_obj_t dict = mp_obj_new_dict(0);
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_w), mp_obj_new_int(r.w));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_h), mp_obj_new_int(r.h));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_scan_us),   mp_obj_new_int(r.scan_cyc / 800));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_thresh_us), mp_obj_new_int(r.thresh_cyc / 800));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_thresh_bradley_us), mp_obj_new_int(r.thresh_bradley_cyc / 800));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_thresh_separable_us), mp_obj_new_int(r.thresh_separable_cyc / 800));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_thresh_pxp_us), mp_obj_new_int(r.thresh_pxp_cyc / 800));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_pxp_stat_before), mp_obj_new_int_from_uint(r.pxp_stat_before_start));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_pxp_ctrl_before), mp_obj_new_int_from_uint(r.pxp_ctrl_before_start));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_pxp_stat_after), mp_obj_new_int_from_uint(r.pxp_stat_after_wait));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_pxp_ctrl_after), mp_obj_new_int_from_uint(r.pxp_ctrl_after_wait));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_pxp_iters), mp_obj_new_int_from_uint(r.pxp_wait_iters));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_edge_us),   mp_obj_new_int(r.edge_cyc / 800));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_scan_cyc),   mp_obj_new_int_from_uint(r.scan_cyc));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_thresh_cyc), mp_obj_new_int_from_uint(r.thresh_cyc));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_thresh_bradley_cyc), mp_obj_new_int_from_uint(r.thresh_bradley_cyc));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_edge_cyc),   mp_obj_new_int_from_uint(r.edge_cyc));
    return dict;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_aruco_bench_obj,
                                  mod_sentai_diag_aruco_bench);

// OP-S10-W16-T3: M4 ArUco-threshold bench.
//   sentai.diag.m4_aruco_bench(block) -> dict
// Returns:
//   ok          : 1 if M4 produced a result, 0 if timeout / M4 dead
//   alive       : 1 if M4 magic visible
//   cycles      : DWT cycle delta (M4 @ 400 MHz, divide by 400 for us)
//   us          : cycles / 400 (M4 clock)
//   frame_w/h   : 160 / 120 (M4 bench fixed frame)
extern int sentai_m4_bench_start(uint32_t timeout_ms);
extern int sentai_m4_bench_run(int block, uint32_t* out_cyc,
                                uint32_t timeout_ms);
extern int sentai_m4_bench_is_alive(void);

static mp_obj_t mod_sentai_diag_m4_aruco_bench(mp_obj_t block_obj) {
    const int block = mp_obj_get_int(block_obj);
    // Ensure M4 is up.  Cold-start budget generous: M4 needs to
    // boot, init clocks, init MPU, run pre_app_main + my app_main,
    // create worker task, then trigger the RemoteApplicationEvent
    // that resumes M7's tx_task.  500 ms accommodates a slow path.
    const int started = sentai_m4_bench_start(500u);
    uint32_t cyc = 0;
    int ok = 0;
    if (started) {
        ok = sentai_m4_bench_run(block, &cyc, 500u);
    }
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_ok),
                       mp_obj_new_int(ok));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_alive),
                       mp_obj_new_int(sentai_m4_bench_is_alive()));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cycles),
                       mp_obj_new_int_from_uint(cyc));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_us),
                       mp_obj_new_int(cyc / 400u));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_w),
                       mp_obj_new_int(80));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_h),
                       mp_obj_new_int(60));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_sentai_diag_m4_aruco_bench_obj,
                                  mod_sentai_diag_m4_aruco_bench);

// Diagnostic peek into the M7-side IPC handler — useful when the
// bench returns ok=0 to tell whether M7 ever saw a reply at all.
extern uint32_t sentai_m4_bench_handler_calls(void);
extern uint32_t sentai_m4_bench_handler_done(void);
extern uint32_t sentai_m4_bench_handler_other(void);
extern uint32_t sentai_m4_bench_last_type(void);
extern uint32_t sentai_m4_bench_last_block(void);
static mp_obj_t mod_sentai_diag_m4_bench_diag(void) {
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_handler_calls),
                       mp_obj_new_int_from_uint(sentai_m4_bench_handler_calls()));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_handler_done),
                       mp_obj_new_int_from_uint(sentai_m4_bench_handler_done()));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_handler_other),
                       mp_obj_new_int_from_uint(sentai_m4_bench_handler_other()));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_type),
                       mp_obj_new_int_from_uint(sentai_m4_bench_last_type()));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_block),
                       mp_obj_new_int_from_uint(sentai_m4_bench_last_block()));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_diag_m4_bench_diag_obj,
                                  mod_sentai_diag_m4_bench_diag);

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
    { MP_ROM_QSTR(MP_QSTR_tpu_call_stats),  MP_ROM_PTR(&mod_sentai_diag_tpu_call_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_chunk_size),  MP_ROM_PTR(&mod_sentai_diag_tpu_chunk_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_zero_copy),   MP_ROM_PTR(&mod_sentai_diag_tpu_zero_copy_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_urb_timeout), MP_ROM_PTR(&mod_sentai_diag_tpu_urb_timeout_obj) },
    { MP_ROM_QSTR(MP_QSTR_tpu_trace),       MP_ROM_PTR(&mod_sentai_diag_tpu_trace_obj) },
    { MP_ROM_QSTR(MP_QSTR_repl_kick),       MP_ROM_PTR(&mod_sentai_diag_repl_kick_obj) },
    { MP_ROM_QSTR(MP_QSTR_flexram_info),    MP_ROM_PTR(&mod_sentai_diag_flexram_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_fx_bench),        MP_ROM_PTR(&mod_sentai_diag_fx_bench_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_aruco_bench),  MP_ROM_PTR(&mod_sentai_diag_m4_aruco_bench_obj) },
    { MP_ROM_QSTR(MP_QSTR_m4_bench_diag),   MP_ROM_PTR(&mod_sentai_diag_m4_bench_diag_obj) },
    { MP_ROM_QSTR(MP_QSTR_fx_stats),        MP_ROM_PTR(&mod_sentai_diag_fx_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_fx_format),       MP_ROM_PTR(&mod_sentai_diag_fx_format_obj) },
    { MP_ROM_QSTR(MP_QSTR_storage_log),     MP_ROM_PTR(&mod_sentai_diag_storage_log_obj) },
    { MP_ROM_QSTR(MP_QSTR_cache_open),      MP_ROM_PTR(&mod_sentai_diag_cache_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_cache_write),     MP_ROM_PTR(&mod_sentai_diag_cache_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_cache_save),      MP_ROM_PTR(&mod_sentai_diag_cache_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_cache_len),       MP_ROM_PTR(&mod_sentai_diag_cache_len_obj) },
    { MP_ROM_QSTR(MP_QSTR_cache_dropped),   MP_ROM_PTR(&mod_sentai_diag_cache_dropped_obj) },
    { MP_ROM_QSTR(MP_QSTR_cache_close),     MP_ROM_PTR(&mod_sentai_diag_cache_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_aruco_bench),     MP_ROM_PTR(&mod_sentai_diag_aruco_bench_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_diag_globals, sentai_diag_globals_table);
static const mp_obj_module_t sentai_diag_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_diag_globals,
};

// sentai_sim_link_shims.c — SIM (x86 Linux) stand-ins for ARM-only symbols
// that shared sources reference (OP-S10-W23-T8, 2026-10-31).
//
// Between July and October 2026 the ARM tree gained a handful of entry
// points (TPU clock mode + wedge recovery, YUV420 dual-input invoke, PXP
// YUV conversion, the camera-trigger pin probe, the MicroPython exec
// guard, the FreeRTOS run-time counter) that the SIM never got, so
// `cmake --build build-sim --target sentai_sim` failed at link time.  The
// SIM has no EdgeTPU clock, no PXP, no CSI pin and no run-time stats, so
// each shim returns the documented "not available" value and nothing else.
//
// Rule: a shim here MUST match the ARM signature exactly (grep the ARM
// definition before editing) and MUST NOT fake success for hardware that
// does not exist in SIM.

#include <stdint.h>
#include <stddef.h>
#include <time.h>

// --- TPU (ARM: sentai_slow_bridge.cc / modsentai_tpu.c) -------------------

// Runtime Edge TPU clock mode (0 low / 1 high).  No TPU clock in SIM.
int sentai_tpu_set_mode(int mode) {
    (void)mode;
    return -1;
}

// Power-cycle + re-upload after a USB wedge.  Nothing to recover in SIM.
int sentai_tpu_recover(void) {
    return -1;
}

// Dual-input (UV + Y planes) invoke for YUV420 models.  The SIM TPU
// backend (sentai_tpu_posix_backend.cc) exposes single-input invoke only.
int sentai_tpu_invoke_with_inputs2(uint8_t* uv_buf, uint8_t* y_buf) {
    (void)uv_buf;
    (void)y_buf;
    return -1;
}

// Set to 1 by the ARM model loader when the model has two inputs.
volatile int g_sentai_yuv_dual = 0;

// --- PXP (ARM: sentai_runtime.cc) ----------------------------------------

// Hardware XYUV -> planar YUV420 conversion.  No PXP in SIM.
int sentai_pxp_xyuv_to_yuv420(const uint8_t* src, int src_w, int src_h,
                              uint8_t* y_dst, uint8_t* uv_dst) {
    (void)src; (void)src_w; (void)src_h; (void)y_dst; (void)uv_dst;
    return -1;
}

// --- Camera trigger pin probe (ARM: sentai_runtime.cc, EXP-s245) ---------

int sentai_pin_probe_cam_trigger(int32_t* out, int n, int park_input) {
    (void)out; (void)n; (void)park_input;
    return -1;
}

// --- MicroPython exec guard (ARM: micropython_task.c) --------------------
// On ARM these arm/disarm the ctrl-c monitor deadline around a long
// native call.  SIM has no monitor task; no-ops keep the call sites
// identical.

void sentai_mp_exec_guard_begin(int timeout_ms) {
    (void)timeout_ms;
}

void sentai_mp_exec_guard_end(void) {
}

// --- FreeRTOS run-time counter (ARM: GPT1 microseconds) ------------------
// sim/FreeRTOSConfig.h has configGENERATE_RUN_TIME_STATS 0, so the kernel
// does not provide this symbol; detection_task.cc uses it as a free-running
// microsecond clock for throttling.  Monotonic host clock, wraps at 2^32 us.

uint32_t vPortGetRunTimeCounterValue(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000000ull +
                      (uint64_t)ts.tv_nsec / 1000ull);
}

// --- HTTP file-API slot FSM counters (ARM: sentai_fs_task.cc) -----------
// The shared bindings/modsentai_fs.c exposes sentai.fs.stats(); the SIM
// serves files through POSIX, so there is no slot FSM and every counter
// reads 0.

void sentai_fs_stats_get(uint32_t* out, int max_fields) {
    if (!out) return;
    for (int i = 0; i < max_fields; i++) out[i] = 0;
}

// --- HTTP / network health counters (ARM: sentai_runtime.cc watchdog) -----
// sentai.rtos.http_stats() is registered since build 1644; the SIM has no
// CDC-NCM path, so it reports no requests, no hangs, healthy.

uint32_t sentai_get_http_requests(void) { return 0; }
uint32_t sentai_get_http_hangs(void) { return 0; }
int sentai_get_network_healthy(void) { return 1; }

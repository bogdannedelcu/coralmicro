// sentai_safety_task.cc — OP-S10-W12-T3 implementation.
//
// Worker that drives the ArUco check of sentai.safety.  See
// sentai_safety_task.h for the SYSTEM MODEL (fault / execution /
// recovery / safe state).
//
// REUSE NOTE (operator spec 2026-05-18 — "sa nu dublam calculele, sa
// cache-uim calculele facute pe un frame de la camera"):
//   - This task calls the EXISTING sentai_aruco_detect() with the
//     camera's frame_seq.  If another consumer (mission MP, sentai.calib)
//     also calls detect() on the same frame_seq, that consumer's call
//     is a duplicate compute on the same input.
//   - To eliminate that duplication TRANSPARENTLY for all callers, we
//     should extend sentai_aruco with frame_seq memoisation: when
//     detect() is asked for a frame_seq matching its cache, return
//     the cached markers without re-running detection.  Tracked as
//     OP-S10-W12-T11 (cache extension in sentai_aruco).
//   - For T3 ship, this task is the sole continuous consumer; mission
//     reads safety via sentai.safety.snapshot() (no extra aruco call).
//     Duplication risk = nil in the s167 use case.

#include "sentai_safety_task.h"
#include "sentai_safety.h"
#include "sentai_aruco.h"
#include "sentai_fr.h"        // OP-S10-W13: Flight Recorder push API

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// ── Platform abstraction ───────────────────────────────────────────
// Both ARM firmware and the POSIX SIM build use FreeRTOS (the SIM
// links libfreertos_posix).  Raw POSIX pthreads on SIM would bypass
// the FreeRTOS scheduler and starve under vTaskDelay-heavy MP REPL
// loads — use xTaskCreate everywhere.
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#define SAFETY_HAVE_FREERTOS 1

// ── External: existing camera zero-copy hook (provided by ARM CSI
// pipeline or SIM camera_bridge_recv).  Weak symbol — if absent in a
// minimal build, the task gracefully degrades (logged + DEGRADED). ────
extern "C" int sentai_camera_grab_gray_zerocopy(const uint8_t** out_buf,
                                                  int* out_w, int* out_h,
                                                  uint32_t* out_seq,
                                                  uint32_t* out_ts_ms)
    __attribute__((weak));

namespace {

// ── Time helpers ───────────────────────────────────────────────────────
inline uint32_t now_ms() {
#if SAFETY_HAVE_FREERTOS
    return (uint32_t)(xTaskGetTickCount() * (1000U / configTICK_RATE_HZ));
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
#endif
}

inline uint32_t now_us() {
#if SAFETY_HAVE_FREERTOS
    // FreeRTOS native us-resolution helper would be the SDK's
    // xTaskGetTickCount * (1000000 / configTICK_RATE_HZ) — adequate
    // for deadline-miss counting at ms granularity.
    return now_ms() * 1000U;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
#endif
}

inline void bounded_sleep_us(uint32_t us) {
    if (us == 0) return;
    // Bound the upper sleep to one period (defence in depth: never
    // sleep so long that stop_evt response > FTTI).
    if (us > SENTAI_SAFETY_TASK_PERIOD_MS * 1000U) {
        us = SENTAI_SAFETY_TASK_PERIOD_MS * 1000U;
    }
#if SAFETY_HAVE_FREERTOS
    TickType_t ticks = (us / 1000U) / (1000U / configTICK_RATE_HZ);
    if (ticks == 0) ticks = 1;
    vTaskDelay(ticks);
#else
    struct timespec ts;
    ts.tv_sec  = us / 1000000U;
    ts.tv_nsec = (long)(us % 1000000U) * 1000L;
    nanosleep(&ts, nullptr);
#endif
}

// ── Static state (no heap, embeded.md §2 rule 3) ──────────────────────
struct State {
    sentai_safety_task_stats_t  stats = {};
    uint32_t                    last_seq_processed = 0;
    uint8_t                     camera_consecutive_fails = 0;
    bool                        started = false;
};
static State s;

// Markers buffer: capped at SENTAI_ARUCO_MAX_MARKERS (16).  Static.
static sentai_aruco_marker_t s_markers[SENTAI_ARUCO_MAX_MARKERS];

// Stop event + task handle.  Stack must be >= configMINIMAL_STACK_SIZE.
// SIM (POSIX) requires 1024 words minimum (per FreeRTOSConfig.h).
// We use xTaskCreate (dynamic) like other SIM tasks (pipeline_task,
// crazy_rx, etc.) for consistency.  Stack = MINIMAL × 4 (POSIX
// pthread frames are larger than ARM, plus sentai_aruco_detect's
// internal contour finder + Jacobi PnP).
static TaskHandle_t       s_task_handle = nullptr;
static EventGroupHandle_t s_stop_evt    = nullptr;
#define STOP_BIT  0x01
inline bool stop_requested() {
    if (!s_stop_evt) return true;
    return (xEventGroupGetBits(s_stop_evt) & STOP_BIT) != 0;
}

// ── Health update helper ──────────────────────────────────────────────
inline void set_health(sentai_safety_task_health_t h) {
    s.stats.health = (uint8_t)h;
}

// ── Frame recording delegated to sentai.fr (OP-S10-W13) ──────────────
// Old in-place PGM writer removed 2026-05-18: belongs in the dedicated
// Flight Recorder subsystem.  SafetyTask now just calls
// `sentai_fr_push_frame()` after the safety state push.  If the FR
// frames channel is not open, the push is a silent no-op (zero cost) —
// gating happens inside sentai_fr, not here.  This keeps SafetyTask
// pure per agent/embeded.md §3.1 strict layer separation.

// ── ONE LOOP ITERATION ────────────────────────────────────────────────
// Returns true on a healthy iteration that pushed to safety, false on
// any sub-failure (caller still advances period boundary).
bool tick_one_iter() {
    const uint32_t t0_us = now_us();
    s.stats.loops_total++;

    // ── §1.1 F1: camera grab ──────────────────────────────────────
    const uint8_t* gray = nullptr;
    int w = 0, h = 0;
    uint32_t cam_seq = 0, cam_ts_ms = 0;
    int cam_rc = -1;
    if (sentai_camera_grab_gray_zerocopy) {
        cam_rc = sentai_camera_grab_gray_zerocopy(
                    &gray, &w, &h, &cam_seq, &cam_ts_ms);
    }
    if (cam_rc != 0 || gray == nullptr || w <= 0 || h <= 0) {
        s.stats.camera_fails++;
        if (++s.camera_consecutive_fails >= SENTAI_SAFETY_TASK_CAM_FAIL_DEGRADE
            && s.stats.health == SENTAI_SAFETY_TASK_HEALTHY) {
            set_health(SENTAI_SAFETY_TASK_DEGRADED);
        }
        // Diagnostic (rate-limited): print first failure + every 100th.
        if (s.stats.camera_fails == 1 ||
            (s.stats.camera_fails % 100) == 0) {
            fprintf(stderr,
                    "[safety_task] cam grab FAIL #%u  rc=%d gray=%p w=%d h=%d\n",
                    (unsigned)s.stats.camera_fails, cam_rc, (void*)gray, w, h);
        }
        // DO NOT push to safety: stale watchdog (~2 s) will latch abort
        // if this persists, which is the correct "feeder silent" reaction.
        return false;
    }
    if (s.camera_consecutive_fails != 0) {
        fprintf(stderr,
                "[safety_task] cam grab RECOVERED after %u fails\n",
                (unsigned)s.camera_consecutive_fails);
    }
    s.camera_consecutive_fails = 0;

    // ── §1.1 F3: duplicate frame seq → skip silently ──────────────
    // (Avoids spuriously growing/resetting the streak when camera
    // publishes the same frame twice.)
    if (cam_seq != 0 && cam_seq == s.last_seq_processed) {
        s.stats.frames_duplicate++;
        return false;
    }
    s.last_seq_processed = cam_seq;
    s.stats.last_seq_processed = cam_seq;

    // ── §1.1 F2: aruco detect ─────────────────────────────────────
    // REUSE: this is the SINGLE detection per frame in the s167 use
    // case.  See file header REUSE NOTE for the OP-S10-W12-T11
    // memoisation TODO that would also dedupe across mission calls.
    int n = sentai_aruco_detect(gray, w, h, cam_seq, cam_ts_ms,
                                  s_markers, SENTAI_ARUCO_MAX_MARKERS);
    if (n < 0) {
        s.stats.aruco_fails++;
        // Recovery escalation §1.3 step 1: treat as worst-case (0
        // markers) so the streak grows; mission sees abort via the
        // SAME path as legitimate marker loss, no special code.
        n = 0;
    }

    // ── Push to safety state machine (single writer) ──────────────
    sentai_safety_on_aruco_result(n, cam_seq, cam_ts_ms);
    s.stats.frames_consumed++;

    // Diagnostic: log first frame + every 30th (~1s @ camera rate).
    if (s.stats.frames_consumed == 1 ||
        (s.stats.frames_consumed % 30) == 0) {
        fprintf(stderr,
                "[safety_task] f%u  w=%d h=%d  n_dets=%d  consumed=%u\n",
                (unsigned)cam_seq, w, h, n,
                (unsigned)s.stats.frames_consumed);
    }

    // ── Frame recording → sentai.fr (operator opens the channel at
    //    mission start; otherwise this is a silent no-op).
    sentai_fr_push_frame(gray, w, h, n, cam_seq, cam_ts_ms);

    // Health recover: was DEGRADED, now a successful loop → HEALTHY.
    if (s.stats.health == SENTAI_SAFETY_TASK_DEGRADED) {
        set_health(SENTAI_SAFETY_TASK_HEALTHY);
    }

    // ── Deadline supervision (§3.2 alive/deadline) ────────────────
    const uint32_t elapsed_us = now_us() - t0_us;
    s.stats.last_loop_us = elapsed_us;
    if (elapsed_us > s.stats.worst_loop_us) s.stats.worst_loop_us = elapsed_us;
    const uint32_t period_us = SENTAI_SAFETY_TASK_PERIOD_MS * 1000U;
    if (elapsed_us > period_us) s.stats.deadline_misses++;
    return true;
}

// ── Worker loop body (platform-agnostic) ──────────────────────────────
void worker_main_loop() {
    set_health(SENTAI_SAFETY_TASK_HEALTHY);
    while (!stop_requested()) {
        const uint32_t t0_us = now_us();
        tick_one_iter();
        // Always drive the stale-feed watchdog (catches our OWN silence).
        sentai_safety_tick(now_ms());
        // Sleep to next period boundary (bounded).
        const uint32_t elapsed = now_us() - t0_us;
        const uint32_t period_us = SENTAI_SAFETY_TASK_PERIOD_MS * 1000U;
        bounded_sleep_us(elapsed < period_us ? (period_us - elapsed) : 0);
    }
    set_health(SENTAI_SAFETY_TASK_STOPPING);
}

void worker_task_entry(void* /*arg*/) {
    worker_main_loop();
    set_health(SENTAI_SAFETY_TASK_UNAVAILABLE);
    vTaskDelete(nullptr);
}

}  // namespace

// =======================================================================
// Public API
// =======================================================================

extern "C" int sentai_safety_task_start(void) {
    if (s.started) return 0;          // §1.1 F6 idempotent
    // Zero stats but keep "started" as transition marker.
    memset(&s.stats, 0, sizeof(s.stats));
    s.last_seq_processed       = 0;
    s.camera_consecutive_fails = 0;
    set_health(SENTAI_SAFETY_TASK_HEALTHY);   // optimistic; tick will flip if bad
    // Ensure ArUco detector is initialised — SafetyTask is the only
    // continuous caller in MP missions that don't otherwise call
    // sentai.aruco.init().  Idempotent; uses built-in defaults
    // (fx=fy=240, cx=160, cy=120 for 320×240 — SIM downward_cam matches).
    sentai_aruco_init();
    // Frame journal moved to sentai.fr (OP-S10-W13): mission MP calls
    // `sentai.fr.open("frames", "/tmp/.../frames")` + `task_start()`.

    if (s_stop_evt == nullptr) {
        s_stop_evt = xEventGroupCreate();
        if (s_stop_evt == nullptr) {
            set_health(SENTAI_SAFETY_TASK_FAULTED);
            return -1;
        }
    }
    xEventGroupClearBits(s_stop_evt, STOP_BIT);
    BaseType_t ok = xTaskCreate(
        worker_task_entry,
        "sentai_safety",
        configMINIMAL_STACK_SIZE * 4,    // POSIX needs more headroom
        nullptr,
        tskIDLE_PRIORITY + 2,            // same as other SIM tasks
        &s_task_handle);
    if (ok != pdPASS || s_task_handle == nullptr) {
        set_health(SENTAI_SAFETY_TASK_FAULTED);
        return -2;
    }
    s.started = true;
    return 0;
}

extern "C" int sentai_safety_task_stop(void) {
    if (!s.started) return 0;          // §1.1 F7 idempotent
    if (s_stop_evt) xEventGroupSetBits(s_stop_evt, STOP_BIT);
    // Spin-wait up to a few periods for clean exit.  Worker calls
    // vTaskDelete(nullptr) on return.  Bounded.
    for (int i = 0; i < 20; ++i) {
        if (s.stats.health == SENTAI_SAFETY_TASK_UNAVAILABLE) break;
        vTaskDelay(pdMS_TO_TICKS(SENTAI_SAFETY_TASK_PERIOD_MS));
    }
    s_task_handle = nullptr;
    s.started = false;
    return 0;
}

extern "C" int sentai_safety_task_get_stats(sentai_safety_task_stats_t* out) {
    if (!out) return SENTAI_SAFETY_ERR_PARAMS;
    // Lock-free read — stats updated by worker only; readers see
    // self-consistent fields (single word writes; counter races are
    // benign — we never compare counters across each other atomically).
    *out = s.stats;
    return 0;
}

// sentai_anchor_forward.cc — continuous C++ auto-forward loop for
// sentai.flow.mode("anchor") pose snapshots.
//
// ────────────────────────────────────────────────────────────────────
// NASA/JPL discipline notes (per agent/embeded.md):
//
// FAULT MODEL — credible faults this task handles:
//   F1  Detector publishes NaN / Inf (cv2/PnP numerical blow-up).
//       Action: drop sample, increment `dropped_nonfinite`.
//   F2  Detector publishes pose with out-of-world-bounds coords
//       (e.g. solvePnP outlier).  Action: drop, increment
//       `dropped_oob`.  Bounds: ±50 m XY, [-2, +20] m Z.
//   F3  Stale pose snapshot — publisher silent for > STALE_MS ms.
//       Action: drop, increment `dropped_stale`.  Keeps FCU EKF
//       from integrating a frozen pose during a publisher hang.
//   F4  Transport down at send time (link / crazy not running).
//       Action: skip the corresponding leg; counter only.
//   F5  Transport send_*() returns error (UART / UDP / CRTP fail).
//       Action: increment `send_failed`.  No retry — next tick
//       re-evaluates; FCU EKF handles VPE gaps gracefully.
//   F6  Task starvation (FreeRTOS scheduler bug, priority too low).
//       Action: start() does a 500 ms liveness check via `s_iters`
//       and fails with -3 rather than reporting a phantom running.
//
// EXECUTION MODEL:
//   - Single FreeRTOS task at tskIDLE_PRIORITY+2 (round-robin with
//     REPL + camera_bridge_recv).
//   - Period = 1000/rate_hz ms (1 ≤ rate ≤ 200), gated by vTaskDelay.
//   - No ISR work, no peripheral access — only reads volatile globals
//     + calls into supervised transports.
//
// RECOVERY MODEL:
//   - "running" is the only state, but each tick decides per-sample:
//     send / skip / drop.  Soft-fault counters expose health to
//     REPL via anchor_forward_stats().
//   - On send_failed >> threshold (caller-monitored), user pauses
//     via anchor_forward(0).  No auto-degraded-mode here — would
//     require visibility into FCU EKF rejection rate.
//
// RESOURCE BUDGET:
//   - Stack: configMINIMAL_STACK_SIZE * 2 (16 KB on POSIX, 2 KB on
//     M7).  High-water-mark exposed via stats (`stack_hwm`).
//   - Heap: zero — no dynamic alloc.
//   - Time per tick: <1 ms (one struct copy + two short MAVLink/
//     CRTP encodes + UART/UDP write).
// ────────────────────────────────────────────────────────────────────
//
// REPL toggle pattern (mirrors flow.start / pipeline.start):
//
//   >>> sentai.flow.mode("anchor")
//   >>> sentai.flow.anchor_forward(10, "auto")    # 10 Hz, auto-route
//   >>> # ... mission runs ...
//   >>> sentai.flow.anchor_forward_stats()
//   {'sent_px4': 142, 'sent_cf2': 0, 'skipped': 18, 'last_seq': 412, ...}
//   >>> sentai.flow.anchor_forward(0)             # stop

#include <stdint.h>
#include <string.h>
// math.h NOT included on purpose — see pose_finite() below; we use
// bit-pattern IEEE-754 checks to avoid pulling libgcc helpers into
// m_text (ITCM is tight; this file lives in .sdram_text but external
// references to isnan/isinf still resolve into the main libgcc copy).

#include "FreeRTOS.h"
#include "task.h"

#include "sentai_aruco_shim.h"

extern "C" {
// Forward decls — these live in sentai_link / sentai_crazy on ARM,
// and have SIM stubs / SIM-side implementations on x86.
int  sentai_link_send_vpe(float x_enu, float y_enu, float z_enu, float yaw_rad);
int  sentai_link_is_running(void);
int  sentai_crazy_send_ext_position(float x_m, float y_m, float z_m);
int  sentai_crazy_is_running(void);
}

// Shared with MicroPython flow.mode() — see modsentai_flow.c.
extern "C" volatile uint32_t g_flow_anchor_mode;

namespace {

enum anchor_target_t : uint8_t {
    TGT_OFF  = 0,
    TGT_AUTO = 1,    // route to every transport that's running
    TGT_PX4  = 2,    // sentai_link_send_vpe only
    TGT_CF2  = 3,    // sentai_crazy_send_ext_position only
    TGT_BOTH = 4,    // both regardless of running state
};

static volatile uint32_t s_running       = 0;
static volatile uint32_t s_rate_hz       = 10;
static volatile uint8_t  s_target        = TGT_OFF;
static volatile uint32_t s_sent_px4      = 0;
static volatile uint32_t s_sent_cf2      = 0;
static volatile uint32_t s_skipped       = 0;
static volatile uint32_t s_last_seq      = 0;
static volatile uint32_t s_send_failed   = 0;
static volatile uint32_t s_iters             = 0;   // body executions
static volatile uint32_t s_dropped_nonfinite = 0;   // F1
static volatile uint32_t s_dropped_oob       = 0;   // F2
static volatile uint32_t s_dropped_stale     = 0;   // F3
static TaskHandle_t      s_task_handle       = NULL;

// Fault-model thresholds (per embeded.md — explicit + bounded).
//   - WORLD frame is ENU; expected lab/outdoor mission volume.
//   - STALE_MS caps acceptable publisher silence (detector + UDS round
//     trip should land <100 ms; 500 ms is a generous safety margin).
static constexpr float    POSE_MAX_XY_M = 50.0f;
static constexpr float    POSE_MIN_Z_M  = -2.0f;
static constexpr float    POSE_MAX_Z_M  = 20.0f;
static constexpr float    POSE_MAX_YAW  = 3.2f;     // > pi
static constexpr uint32_t POSE_STALE_MS = 500;

__attribute__((section(".sdram_text"), noinline))
static bool pose_finite(const sentai_aruco_pose_t* p) {
    // Avoid math.h isnan/isinf (pulls libgcc helpers into m_text).
    // IEEE-754 float: exponent bits [30:23].  NaN or Inf iff exponent
    // is all-ones, regardless of sign/mantissa.  So "finite" is
    // `(bits & 0x7F800000) != 0x7F800000`.
    uint32_t bx, by, bz, byaw;
    memcpy(&bx,   &p->x_m,    4);
    memcpy(&by,   &p->y_m,    4);
    memcpy(&bz,   &p->z_m,    4);
    memcpy(&byaw, &p->yaw_rad,4);
    const uint32_t M = 0x7F800000u;
    return (bx   & M) != M
        && (by   & M) != M
        && (bz   & M) != M
        && (byaw & M) != M;
}

__attribute__((section(".sdram_text"), noinline))
static bool pose_in_bounds(const sentai_aruco_pose_t* p) {
    return p->x_m >= -POSE_MAX_XY_M && p->x_m <= POSE_MAX_XY_M
        && p->y_m >= -POSE_MAX_XY_M && p->y_m <= POSE_MAX_XY_M
        && p->z_m >= POSE_MIN_Z_M   && p->z_m <= POSE_MAX_Z_M
        && p->yaw_rad >= -POSE_MAX_YAW && p->yaw_rad <= POSE_MAX_YAW;
}

static inline bool send_to_px4(uint8_t tgt) {
    if (tgt == TGT_PX4 || tgt == TGT_BOTH) return true;
    if (tgt == TGT_AUTO) return sentai_link_is_running() != 0;
    return false;
}

static inline bool send_to_cf2(uint8_t tgt) {
    if (tgt == TGT_CF2 || tgt == TGT_BOTH) return true;
    if (tgt == TGT_AUTO) return sentai_crazy_is_running() != 0;
    return false;
}

__attribute__((section(".sdram_text"), noinline))
static void anchor_forward_task(void* /*arg*/) {
    sentai_aruco_pose_t pose;
    memset(&pose, 0, sizeof(pose));
    s_iters = 1;     // mark "I ran" BEFORE the first vTaskDelay so
                     // the liveness check in start() succeeds even if
                     // rate_hz is misconfigured.

    while (s_running) {
        uint32_t hz = s_rate_hz;
        if (hz == 0) hz = 10;
        if (hz > 200) hz = 200;     // cap; FCU VPE rate-limited anyway
        TickType_t delay = pdMS_TO_TICKS(1000u / hz);
        if (delay == 0) delay = 1;
        vTaskDelay(delay);
        s_iters++;     // proves task body is actually scheduled

        // ❶ gate on anchor mode
        if (g_flow_anchor_mode == 0) { s_skipped++; continue; }

        // ❷ pull latest pose snapshot
        sentai_aruco_get_latest(&pose);
        if (!pose.detected) { s_skipped++; continue; }

        // ❸ deduplicate — only forward on a fresh frame_seq
        if (pose.frame_seq == s_last_seq) { s_skipped++; continue; }

        // ❹ fault-model gates per embeded.md F1-F3
        if (!pose_finite(&pose))    { s_dropped_nonfinite++; continue; }
        if (!pose_in_bounds(&pose)) { s_dropped_oob++;       continue; }
        // F3 — stale snapshot detection.  `src_ts_ms` is the publisher's
        // monotonic ms at frame time; compare to our local tick (also ms).
        // ARM tick = configTICK_RATE_HZ (1000 here) so xTaskGetTickCount
        // is already ms.  Wrap is fine — diff wraps too.
        uint32_t now_ms = (uint32_t)xTaskGetTickCount();
        uint32_t age_ms = now_ms - pose.src_ts_ms;
        if (pose.src_ts_ms != 0 && age_ms > POSE_STALE_MS) {
            s_dropped_stale++;
            continue;
        }

        s_last_seq = pose.frame_seq;

        const uint8_t tgt = s_target;

        // ❹ dispatch
        if (send_to_px4(tgt)) {
            int rc = sentai_link_send_vpe(pose.x_m, pose.y_m, pose.z_m,
                                           pose.yaw_rad);
            if (rc == 0) s_sent_px4++;
            else         s_send_failed++;
        }
        if (send_to_cf2(tgt)) {
            int rc = sentai_crazy_send_ext_position(pose.x_m, pose.y_m,
                                                     pose.z_m);
            if (rc == 0) s_sent_cf2++;
            else         s_send_failed++;
        }
    }
    s_task_handle = NULL;
    vTaskDelete(NULL);
}

static uint8_t parse_target(const char* name, size_t len) {
    if (!name) return TGT_AUTO;
    if (len == 4 && strncmp(name, "auto", 4) == 0) return TGT_AUTO;
    if (len == 3 && strncmp(name, "px4",  3) == 0) return TGT_PX4;
    if (len == 3 && strncmp(name, "cf2",  3) == 0) return TGT_CF2;
    if (len == 4 && strncmp(name, "both", 4) == 0) return TGT_BOTH;
    if (len == 3 && strncmp(name, "off",  3) == 0) return TGT_OFF;
    return 0xFF;     // unknown — caller raises ValueError
}

}  // namespace


extern "C" __attribute__((section(".sdram_text"), noinline))
int sentai_anchor_forward_start(uint32_t rate_hz,
                                 const char* target,
                                 int target_len) {
    uint8_t tgt = parse_target(target, (size_t)target_len);
    if (tgt == 0xFF) return -2;       // bad target name

    s_rate_hz = rate_hz;
    s_target  = tgt;

    if (rate_hz == 0 || tgt == TGT_OFF) {
        // Treat as a stop request.
        s_running = 0;
        return 0;
    }
    if (s_running) {
        // Already running — just live-update rate + target above.
        return 0;
    }
    s_sent_px4 = 0; s_sent_cf2 = 0;
    s_skipped  = 0; s_send_failed = 0;
    s_dropped_nonfinite = 0;
    s_dropped_oob       = 0;
    s_dropped_stale     = 0;
    s_iters    = 0;
    s_last_seq = 0;
    s_running  = 1;
    // Stack: ARM Cortex-M7 needs ~2 KB; POSIX port `configMINIMAL_STACK_SIZE`
    // is 1024 words (8 KB) so we use configMINIMAL_STACK_SIZE * 2 for
    // portability — generous on both, can't be smaller than the minimum on
    // POSIX without xTaskCreate silently failing to schedule the task.
    //
    // Priority: tskIDLE_PRIORITY + 2 matches the REPL + camera bridge tasks
    // on SIM (per sim/main_sim.c).  With "+1" the SIM POSIX port never
    // scheduled this task (REPL pre-empted it and IDLE got the slack);
    // round-robin at the same priority works.
    BaseType_t rc = xTaskCreate(anchor_forward_task,
                                 "anchor_fwd",
                                 configMINIMAL_STACK_SIZE * 2,
                                 NULL,
                                 tskIDLE_PRIORITY + 2,
                                 &s_task_handle);
    if (rc != pdPASS) {
        s_running = 0;
        return -1;
    }
    // Liveness check — wait up to 500 ms for the task to enter its
    // body at least once.  If it doesn't, fail loud rather than
    // silently lying about a "running" forwarder.
    for (int i = 0; i < 50; ++i) {
        if (s_iters > 0) return 0;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Task never woke up — most likely a scheduler / priority bug.
    // No printf here (string literals would land in m_text/ITCM which
    // is full).  Caller checks rc == -3 to discover the regression;
    // sentai_anchor_forward_health() exposes the in-task iteration
    // counter so the failure is still post-mortem visible.
    s_running = 0;
    s_task_handle = NULL;        // we leak the handle; can't safely vTaskDelete from here
    return -3;
}

extern "C" __attribute__((section(".sdram_text"), noinline))
int sentai_anchor_forward_stop(void) {
    s_running = 0;
    return 0;
}

extern "C" __attribute__((section(".sdram_text"), noinline))
void sentai_anchor_forward_stats(uint32_t* sent_px4,
                                             uint32_t* sent_cf2,
                                             uint32_t* skipped,
                                             uint32_t* last_seq,
                                             uint32_t* send_failed,
                                             uint32_t* running,
                                             uint32_t* rate_hz,
                                             uint32_t* target) {
    if (sent_px4)    *sent_px4    = s_sent_px4;
    if (sent_cf2)    *sent_cf2    = s_sent_cf2;
    if (skipped)     *skipped     = s_skipped;
    if (last_seq)    *last_seq    = s_last_seq;
    if (send_failed) *send_failed = s_send_failed;
    if (running)     *running     = s_running;
    if (rate_hz)     *rate_hz     = s_rate_hz;
    if (target)      *target      = s_target;
}

// Iteration counter — separate so binding can fetch it without
// reshuffling the existing stats signature.
extern "C" __attribute__((section(".sdram_text"), noinline))
uint32_t sentai_anchor_forward_iters(void) {
    return s_iters;
}

// Per embeded.md "D. Memory and resource rules" — expose the
// task stack high-water-mark + the fault-drop counters so callers
// (and CI) can monitor for budget creep + spurious drops.
extern "C" __attribute__((section(".sdram_text"), noinline))
void sentai_anchor_forward_health(uint32_t* dropped_nonfinite,
                                   uint32_t* dropped_oob,
                                   uint32_t* dropped_stale,
                                   uint32_t* stack_hwm_words) {
    if (dropped_nonfinite) *dropped_nonfinite = s_dropped_nonfinite;
    if (dropped_oob)       *dropped_oob       = s_dropped_oob;
    if (dropped_stale)     *dropped_stale     = s_dropped_stale;
    if (stack_hwm_words) {
        // uxTaskGetStackHighWaterMark returns "min unused stack words
        // since task creation" — a leading indicator of stack pressure.
        // Returns 0 if task not running.
        *stack_hwm_words = s_task_handle
            ? (uint32_t)uxTaskGetStackHighWaterMark(s_task_handle)
            : 0;
    }
}

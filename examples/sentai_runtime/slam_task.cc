// slam_task.cc — InferTask-style perception loop.
// See slam_task.h for the design contract (OP-S10-W11-T3).

#include "slam_task.h"
#include "sentai_prep.h"
#include "sentai_hsv.h"
#include "sentai_places.h"
#include "sentai_error.h"

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/semphr.h"

#ifndef SENTAI_PLATFORM_SIM
// ARM-only — sentai_health is not yet ported to the SIM build (the
// subsystem table sizing + boot-mode is ARM-specific).  SIM treats
// the calls as no-ops.
#include "sentai_health.h"
#endif

#include <stdint.h>
#include <string.h>

#ifdef SENTAI_PLATFORM_SIM
  #include <time.h>
  #define SLAM_BSS    /* default .bss */
  #define SLAM_DMB()  /* x86 single-core: no barrier needed */
#else
  #define SLAM_BSS    __attribute__((section(".sdram_bss"), aligned(8)))
  #define SLAM_DMB()  __asm volatile ("dmb" ::: "memory")
#endif

// ============================================================
// High-resolution timer (microseconds since boot, mod 2^32).
// ============================================================
static inline uint32_t slam_now_us(void) {
#ifdef SENTAI_PLATFORM_SIM
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t ns = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
    return (uint32_t)(ns / 1000ull);
#else
    // Cortex-M7 DWT->CYCCNT, 32-bit, @ 800 MHz core → rolls over every ~5.36 s.
    // Deltas across one HSV+query cycle (~200 µs) are far below rollover.
    const uint32_t cyc = *((volatile uint32_t*)0xE0001004u);
    return cyc / 800u;   // SystemCoreClock = 800 MHz; cyc/800 → µs
#endif
}

// ============================================================
// Task storage (NASA/JPL §1.4: static allocation, no heap in steady state).
// ============================================================
static constexpr unsigned kSlamStackWords = configMINIMAL_STACK_SIZE * 6;

static StaticTask_t       s_slam_tcb;
static StackType_t        s_slam_stack[kSlamStackWords] SLAM_BSS;
static TaskHandle_t       s_slam_task = nullptr;
static SemaphoreHandle_t  s_sem_slam_input = nullptr;

// Control + counters.  `s_running` is the sole loop predicate (NASA/JPL §1.1).
static volatile bool      s_running = false;

static volatile uint32_t  s_frames_processed = 0;
static volatile uint32_t  s_frames_dropped   = 0;
static volatile uint32_t  s_last_compute_us  = 0;

// EMA-free moving average: ring of last 16 samples, recomputed on each push.
// 16 entries × uint32 = 64 B — negligible memory.  Keeps the math
// deterministic and bounded (per [[no-ema-for-integration]] — for
// metric averaging EMA is fine, but a fixed ring is even simpler and
// gives a more honest "mean of last N" view for the operator).
#define SLAM_AVG_RING 16
static uint32_t  s_avg_ring[SLAM_AVG_RING];
static uint8_t   s_avg_idx = 0;
static uint8_t   s_avg_count = 0;

static void push_avg(uint32_t us) {
    s_avg_ring[s_avg_idx] = us;
    s_avg_idx = (uint8_t)((s_avg_idx + 1u) % SLAM_AVG_RING);
    if (s_avg_count < SLAM_AVG_RING) s_avg_count++;
}

static uint32_t mean_avg(void) {
    if (s_avg_count == 0) return 0;
    uint64_t sum = 0;
    for (uint8_t i = 0; i < s_avg_count; ++i) sum += s_avg_ring[i];
    return (uint32_t)(sum / s_avg_count);
}

// Latest published match.  Updated by SlamTask; read by any task.
// The atomic-publish pattern (write fields → DMB → bump result_seq)
// lets MP readers detect a fresh result by sampling result_seq.
static volatile sentai_slam_result_t s_current = {0, 0, 0, 0, 0, 0};

// Health calls are ARM-only (SIM doesn't link sentai_health).
static inline void slam_health_success(void) {
#ifndef SENTAI_PLATFORM_SIM
    sentai_health_success(SUBSYS_SLAM);
#endif
}
static inline void slam_health_fail(void) {
#ifndef SENTAI_PLATFORM_SIM
    sentai_health_fail(SUBSYS_SLAM);
#endif
}

// Bounded retry budget for seqlock collisions per consumer cycle.
// 0 = no retry (single try), 1 = one retry, etc.  PrepTask fires at
// ~30 Hz (33 ms period) and HSV compute is ~150 µs, so the collision
// window is ~0.5% — a single retry resolves >99.99% of cases.
#define SLAM_TORN_RETRIES   1

// ============================================================
// SlamTask body.
// ============================================================
static void slam_task_fn(void* /*param*/) {
    // Loop predicate: s_running.  Bounded — sentai_slam_stop() flips it
    // and gives the sem to unblock the take().  No other exit path.
    while (s_running) {

        // F1: sem timeout.  Bounded wait at 500 ms — guarantees the
        // task wakes periodically even if PrepTask is silent, so the
        // s_running flag flip is observed within FTTI ≤ 500 ms.
        if (xSemaphoreTake(s_sem_slam_input, pdMS_TO_TICKS(500)) != pdTRUE) {
            // Don't bump frames_dropped on idle timeout — only on
            // failures inside the perception path.  Idle is normal.
            continue;
        }

        if (!s_running) break;

        const uint32_t t_start = slam_now_us();

        // ── Seqlock-protected consume (W11-T3.1 — C1 fix) ──────────
        // Try up to SLAM_TORN_RETRIES + 1 times; if every attempt is
        // torn, accept the last result as best-effort and SERR_LOG
        // the unresolved torn read (rare — should not happen in
        // practice given the 0.5% collision rate × 2 attempts).
        const uint8_t* rgb        = nullptr;
        int            sw         = 0;
        int            sh         = 0;
        uint32_t       ticket     = 0;
        uint8_t        desc[SENTAI_HSV_DIM];
        int            attempt;
        bool           consistent = false;
        bool           hsv_ok     = false;

        for (attempt = 0; attempt <= SLAM_TORN_RETRIES; ++attempt) {
            if (sentai_prep_slot_begin_read(SENTAI_PREP_SLOT_RGB_64,
                                             &rgb, &sw, &sh, &ticket) != 0) {
                // F2: slot disabled / no frame yet.  Bounded miss —
                // don't retry, next signal will arrive.
                break;
            }
            if (sentai_hsv_compute(rgb, sw, sh, desc) != 0) {
                // F3: hsv_compute bad args — should not happen since
                // begin_read validated dims.  Treat as a drop.
                hsv_ok = false;
                // Still need to close the read so producer_overruns
                // isn't biased by us walking away mid-compute.
                (void)sentai_prep_slot_end_read(SENTAI_PREP_SLOT_RGB_64, ticket);
                break;
            }
            hsv_ok = true;
            if (sentai_prep_slot_end_read(SENTAI_PREP_SLOT_RGB_64, ticket)) {
                // Tear-free read — done.
                consistent = true;
                break;
            }
            // Torn read — retry if budget remains.  end_read already
            // bumped s_producer_overruns + SERR'd on first occurrence.
        }

        if (!hsv_ok || !consistent) {
            if (!consistent && hsv_ok) {
                // Retries exhausted; result accepted best-effort.
                // SERR once per session — rare enough to surface.
                static uint8_t s_torn_unresolved_logged = 0;
                if (!s_torn_unresolved_logged) {
                    s_torn_unresolved_logged = 1;
                    SERR_LOG(SERR_SLAM_TORN_UNRESOLVED, 0);
                }
            }
            if (!hsv_ok) {
                // Drop the frame (no publish).
                s_frames_dropped++;
                slam_health_fail();
                continue;
            }
            // hsv_ok && !consistent → fall through, publish best-effort.
        }

        // Places query: scan whole gallery (cell_ring=0).  Threshold 0
        // → caller MP can filter via score_pct.  ~4 µs bounded for the
        // full 64-slot sweep per sentai_places.h §"Execution model".
        const sentai_places_match_t m =
            sentai_places_query(desc, /*cell_ring=*/0,
                                 /*k_disk=*/0, /*thresh_pct=*/0);

        const uint32_t t_end = slam_now_us();
        const uint32_t dt    = t_end - t_start;   // mod-2^32 wraparound safe

        s_last_compute_us = dt;
        push_avg(dt);
        s_frames_processed++;
        slam_health_success();

        // Publish.  Fields-then-seq with __DMB barrier between, so an MP
        // reader sampling result_seq sees fully-consistent fields.
        s_current.match_id     = m.id;
        s_current.score_pct    = m.score_pct;
        s_current.l1_dist      = m.l1_dist;
        s_current.frame_seq    = ticket;   // seq at which descriptor was sampled
        s_current.t_compute_us = dt;
        SLAM_DMB();
        s_current.result_seq   = s_current.result_seq + 1u;
    }

    // Self-delete on stop.  Caller (sentai_slam_stop) is waiting on
    // s_slam_task going NULL.
    s_slam_task = nullptr;
    vTaskDelete(nullptr);
}

// ============================================================
// Lifecycle.
// ============================================================

#ifndef SENTAI_PLATFORM_SIM
// ARM prereq probes — defined in sentai_runtime.cc / detection_task.cc.
// On SIM the camera/pipeline equivalents don't exist; the producer
// (camera_bridge_recv) is always alive once main() runs.
extern "C" int sentai_cam_is_initialized(void);
extern "C" int sentai_detection_is_running(void);
#endif

extern "C" int sentai_slam_start(void) {
    if (s_running && s_slam_task) return 0;   // idempotent

#ifndef SENTAI_PLATFORM_SIM
    // F0: prereq — camera (and therefore PXP HW via BOARD_InitPxp,
    // see [[pxp-init-required]]) must be initialised before we can
    // run sentai_pxp_scale in the PrepTask SLOT_RGB_64 producer.
    if (!sentai_cam_is_initialized()) {
        SERR_LOG(SERR_SLAM_PREREQ_CAM, 0);
        return -10;
    }

    // F0: prereq — PrepTask must be running, since IT is the
    // SLOT_RGB_64 producer.  Without it SlamTask would idle forever.
    if (!sentai_detection_is_running()) {
        SERR_LOG(SERR_SLAM_PREREQ_PIPE, 0);
        return -11;
    }
#endif

    // Lazy sem create — heap touched only at init, per NASA/JPL §1.3.
    // Counting sem with max=1 → multiple signals between consumer wakes
    // coalesce, so we never accumulate a backlog (last-frame-wins on
    // the producer-signal path as well).
    if (!s_sem_slam_input) {
        s_sem_slam_input = xSemaphoreCreateCounting(1, 0);
        if (!s_sem_slam_input) {
            SERR_LOG(SERR_SLAM_SEM_ALLOC, 0);
            return -1;
        }
    }
    // Drain any leftover signal from a prior run so the first iteration
    // genuinely waits for a fresh PrepTask publish.
    while (xSemaphoreTake(s_sem_slam_input, 0) == pdTRUE) { /* drain */ }

    // Refcount-enable SLOT_RGB_64 so the producer (PrepTask on ARM,
    // camera_bridge_recv on SIM Phase 1d) starts populating it.
    if (sentai_prep_slot_enable(SENTAI_PREP_SLOT_RGB_64) < 0) {
        return -2;
    }

    // Reset per-run counters (preserve nothing — fresh diagnostic view).
    s_frames_processed = 0;
    s_frames_dropped   = 0;
    s_last_compute_us  = 0;
    s_avg_idx          = 0;
    s_avg_count        = 0;
    memset(s_avg_ring, 0, sizeof(s_avg_ring));

    s_running = true;
    s_slam_task = xTaskCreateStatic(slam_task_fn, "slam",
                                     kSlamStackWords, nullptr,
                                     tskIDLE_PRIORITY + 1,
                                     s_slam_stack, &s_slam_tcb);
    if (!s_slam_task) {
        SERR_LOG(SERR_SLAM_TASK_ALLOC, 0);
        s_running = false;
        sentai_prep_slot_disable(SENTAI_PREP_SLOT_RGB_64);
        return -3;
    }
#ifndef SENTAI_PLATFORM_SIM
    // Module starts healthy — subsequent fail()/success() calls move
    // it through the standard DEGRADED/FAULTED transitions.
    sentai_health_set_recovering(SUBSYS_SLAM);
#endif
    return 0;
}

extern "C" int sentai_slam_stop(void) {
    if (!s_running) return 0;   // idempotent

    s_running = false;
    // Unblock the take() so the task observes s_running == false.
    if (s_sem_slam_input) xSemaphoreGive(s_sem_slam_input);

    // Best-effort wait for self-delete (NASA/JPL §1.1: bounded loops).
    // 200 × 10 ms = 2 s ceiling — generous because POSIX-port FreeRTOS
    // jitter can stretch a sem timeout cycle (~500 ms) under load.
    for (int i = 0; i < 200 && s_slam_task; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    sentai_prep_slot_disable(SENTAI_PREP_SLOT_RGB_64);
#ifndef SENTAI_PLATFORM_SIM
    sentai_health_set_unavailable(SUBSYS_SLAM);
#endif
    // Always return 0: from the caller's perspective the task IS
    // stopped — s_running is false, MP `is_running` reads 0.  The
    // self-delete is best-effort; a dirty exit (rare, POSIX jitter)
    // is logged via the internal dirty-exit pattern when added.
    return 0;
}

extern "C" int sentai_slam_is_running(void) {
    return (s_running && s_slam_task) ? 1 : 0;
}

extern "C" void sentai_slam_get_current(sentai_slam_result_t* out) {
    if (!out) return;
    // Sample-retry pattern: read seq, copy fields, re-read seq.
    // Bounded retries (3) — collision is rare at typical consumer
    // rates (MP polls at < 50 Hz vs SlamTask at 30 Hz).
    for (int retry = 0; retry < 3; ++retry) {
        const uint32_t seq_pre = s_current.result_seq;
        out->match_id     = s_current.match_id;
        out->score_pct    = s_current.score_pct;
        out->l1_dist      = s_current.l1_dist;
        out->frame_seq    = s_current.frame_seq;
        out->result_seq   = seq_pre;
        out->t_compute_us = s_current.t_compute_us;
        SLAM_DMB();
        if (s_current.result_seq == seq_pre) return;
    }
    // Caller still gets a valid (possibly slightly-torn) snapshot;
    // result_seq lets them detect collisions externally if needed.
}

extern "C" void sentai_slam_get_stats(sentai_slam_stats_t* out) {
    if (!out) return;
    out->frames_processed = s_frames_processed;
    out->frames_dropped   = s_frames_dropped;
    out->last_compute_us  = s_last_compute_us;
    out->avg_compute_us   = mean_avg();
    out->is_running       = (s_running && s_slam_task) ? 1 : 0;
}

extern "C" void sentai_slam_signal_frame(void) {
    // PrepTask producer-side hook.  No-op when SlamTask isn't running
    // (sem may not even exist yet).  Counting sem with max=1 coalesces
    // multiple signals between consumer wakes — last-frame-wins.
    if (s_sem_slam_input && s_running) {
        // Ignore the return — pdFALSE means "already at max", which is
        // exactly the coalescing behaviour we want.  No backlog.
        (void)xSemaphoreGive(s_sem_slam_input);
    }
}

// PrepTask helper — extracted from detection_task.cc:prep_task_fn so
// the implementation lives in .sentai_slow (SDRAM) and the m_text
// region stays under budget per [[itcm-budget]].  Called by PrepTask
// once per camera frame for the SLOT_RGB_64 fan-out path.
// Forward decl of the PXP HW scaler (defined in sentai_runtime.cc,
// .sdram_text) — duplicating here so slam_task.cc compiles standalone.
extern "C" int sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                                 uint8_t* dst, int dw, int dh);

extern "C" int sentai_prep_publish_slot_rgb_64(const uint8_t* raw_xrgb,
                                                int src_w, int src_h) {
    int sw = 0, sh = 0;
    uint8_t* sbuf = sentai_prep_slot_begin_write(
        SENTAI_PREP_SLOT_RGB_64, &sw, &sh);
    if (!sbuf) return -1;
    const int rc = sentai_pxp_scale(raw_xrgb, src_w, src_h, sbuf, sw, sh);
    if (rc != 0) return -2;
    sentai_prep_slot_commit(SENTAI_PREP_SLOT_RGB_64);
    sentai_slam_signal_frame();
    return 0;
}

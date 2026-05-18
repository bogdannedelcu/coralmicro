// sentai_safety_task.h — OP-S10-W12-T3.  Companion to sentai_safety.h.
//
// Worker task that DRIVES the ArUco check of sentai.safety: each tick
// it grabs the latest camera frame (zero-copy via existing
// sentai_camera_grab_gray_zerocopy), runs the existing sentai_aruco_
// detect, and pushes the result into the safety state machine.  No
// camera intake, no ArUco implementation, no policy — pure
// orchestration loop that REUSES upstream subsystems.
//
// Lifecycle (explicit, mission-controlled):
//
//   sentai_safety_init();
//   sentai_safety_enable(SENTAI_SAFETY_CHK_ARUCO, &params);
//   sentai_safety_task_start();          // spawn worker
//   ... mission runs, polls aborted() ...
//   sentai_safety_task_stop();           // join worker
//   sentai_safety_disable(SENTAI_SAFETY_CHK_ARUCO);
//   sentai_safety_clear();
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §1)
// =========================================================================
// Fault model:
//   F1  sentai_camera_grab_gray_zerocopy returns rc != 0 / null gray /
//        w*h == 0                                  -> camera_fails++,
//                                                     NO push (stale watchdog
//                                                     in state machine catches
//                                                     if persistent > 2s)
//   F2  sentai_aruco_detect returns negative      -> aruco_fails++,
//                                                     push n_dets=0 (treated
//                                                     same as no markers —
//                                                     contributes to streak)
//   F3  same camera frame_seq twice in a row      -> skip silently (no
//                                                     double-count, no spurious
//                                                     streak reset)
//   F4  deadline miss (loop_us > TARGET_PERIOD_US) -> deadline_misses++;
//                                                     loop continues at next
//                                                     available period boundary
//   F5  worker thread crash / OOM                 -> daemon dies; stale
//                                                     watchdog in state machine
//                                                     latches abort within 2s
//   F6  start() called when already running       -> 0 (idempotent)
//   F7  stop() called when not running            -> 0 (idempotent)
//
// Execution model:
//   - SINGLE TASK / pthread, period TARGET_PERIOD_US ≈ camera FPS.
//   - Uses one static markers array (no heap, no per-loop malloc).
//   - Reads camera frame via the existing zero-copy hook.  The hook
//     returns a pointer to a buffer owned by camera_bridge_recv (SIM)
//     or the CSI ISR pipeline (ARM); we treat that buffer as
//     read-only and finish detect() before the next loop releases it.
//   - On POSIX SIM: pthread + clock_gettime(MONOTONIC).
//   - On ARM RTOS: FreeRTOS task + xTaskGetTickCount().  Task priority
//     deliberately MEDIUM (above MP REPL, below camera ISR / CSI DMA).
//   - Stack: static, ≤ 4 KB.  Watermark + overflow hook on ARM.
//   - No ISR access from this task; no shared mutable state outside
//     sentai_safety state-machine APIs (themselves mutex-protected).
//
// Recovery model:
//   - Local retries (camera grab) bounded by counter; on persistent
//     failure transitions to DEGRADED (still pushes n_dets=0 so streak
//     reaches abort and mission escapes).
//   - Worker thread does NOT restart itself; if it dies, the stale
//     watchdog ensures the mission is informed.
//   - Mission re-arm via sentai_safety_task_stop() + clear() + start().
//
// Safe state (per agent/embeded.md §1.4):
//   - stop_evt set → loop exits within TARGET_PERIOD_US.
//   - All sentai_safety_on_aruco_result calls have completed (single
//     writer; no half-state in state machine).
//   - Camera frame buffer is read-only; nothing to release.
//   - last_push_t_ms in state machine LEFT AS-IS (so the stale watchdog
//     correctly detects "feeder stopped" if the mission still queries).
//
// FTTI (Fault Tolerance Time Interval):
//   - From "all 4 markers lost" to "abort flag latched":
//       SafetyTask period (~33 ms) × streak threshold frames (default 30)
//       + state-machine latency (< 1 ms)  ≈  1.0 s
//     matches operator's hard rule [[flowbaseline2-4markers-abort]].
//   - From "camera pipeline silent" to "abort flag latched":
//       STALE_TIMEOUT_S (2 s) + tick period (~100 ms)  ≈  2.1 s.
//
// Diagnostic coverage:
//   - Camera failures: counted, reflected in health.
//   - ArUco failures: counted, treated as worst-case (n=0).
//   - Deadline misses: counted, exposed via stats.
//   - Worker dies: caught by stale watchdog.
//   - Total credible-failure coverage: 95%+ of in-loop failure modes.
//
// Memory:
//   - Static globals: SafetyTaskState (~80 B) + markers buffer
//     (16 × ~96 B = ~1.5 KB) + task stack (4 KB).  Total ~6 KB BSS.
//   - Zero heap, zero per-call malloc.
//   - .sdram_text per [[itcm-budget]].
//
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Timing knobs ------------------------------------------------------
// Target loop period: matches typical camera dump rate (~30 FPS raw,
// ~10 FPS dumped at FRAMES_EVERY=3) with headroom.  Tunable at compile
// time; runtime configuration would expand the SOUP surface for no
// observed benefit yet.
#ifndef SENTAI_SAFETY_TASK_PERIOD_MS
#define SENTAI_SAFETY_TASK_PERIOD_MS    33      // ~30 Hz nominal
#endif

// Bounded retry on camera grab before transitioning to DEGRADED.  At
// PERIOD_MS = 33, 6 failures ≈ 200 ms blind window — within FTTI for
// camera silence triggering the stale watchdog.
#define SENTAI_SAFETY_TASK_CAM_FAIL_DEGRADE  6

// ---- Task health state ------------------------------------------------
typedef enum {
    SENTAI_SAFETY_TASK_UNAVAILABLE = 0,   // not started
    SENTAI_SAFETY_TASK_HEALTHY     = 1,   // running, feeding successfully
    SENTAI_SAFETY_TASK_DEGRADED    = 2,   // running, but camera/aruco failing
    SENTAI_SAFETY_TASK_FAULTED     = 3,   // running, but unrecoverable failure
    SENTAI_SAFETY_TASK_STOPPING    = 4,   // stop requested, waiting for exit
} sentai_safety_task_health_t;

// ---- Snapshot exposed to MP/diag --------------------------------------
typedef struct {
    uint8_t     health;                   // sentai_safety_task_health_t
    uint8_t     _pad[3];
    uint32_t    loops_total;
    uint32_t    frames_consumed;          // fresh frames pushed to safety
    uint32_t    frames_duplicate;         // skipped due to same seq
    uint32_t    camera_fails;
    uint32_t    aruco_fails;
    uint32_t    deadline_misses;
    uint32_t    last_loop_us;
    uint32_t    worst_loop_us;
    uint32_t    last_seq_processed;
} sentai_safety_task_stats_t;

// =======================================================================
// API
// =======================================================================

// Spawn the worker.  Idempotent — calling twice is a no-op.  Returns 0
// on success, negative on resource error (cannot spawn thread/task).
int sentai_safety_task_start(void);

// Signal stop + join the worker.  Idempotent.  Returns 0.  Worker
// exits within ≤ TARGET_PERIOD_MS + ε.  Blocking call.
int sentai_safety_task_stop(void);

// Snapshot the worker's health + counters.  Lock-free read.
int sentai_safety_task_get_stats(sentai_safety_task_stats_t* out);

#ifdef __cplusplus
}
#endif

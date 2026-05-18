// sentai_fr_task.cc — OP-S10-W13 worker side of the Flight Recorder.
//
// Owns the FreeRTOS task that drives `sentai_fr_drain_round()` at
// a fixed cadence (20 ms by default).  Lifecycle is start/stop only;
// the worker is a single thread, single state (running/stopped).  All
// channel state lives in sentai_fr.cc; this file knows nothing about
// pool sizes, slot formats, or sinks.
//
// Split rationale (mirrors sentai_safety vs sentai_safety_task):
//   - sentai_fr.cc           = "what does FR record + how is it queued"
//   - sentai_fr_task.cc (here) = "who turns the crank, on what cadence"
//
// =========================================================================
// SYSTEM MODEL  (per agent/embeded.md §A — fault/execution/recovery/safe)
// =========================================================================
// Fault model:
//   F1  task_start when already running       -> 0 idempotent
//   F2  xTaskCreate fails                     -> -1, stderr diagnostic
//   F3  task_stop when not running            -> 0 idempotent
//   F4  worker entry crashes / exits early    -> s_worker_alive flips,
//                                                 task_stop's bounded
//                                                 join returns within
//                                                 ≤ 1.5 s anyway
//
// Execution model:
//   - Single FreeRTOS task at tskIDLE_PRIORITY + 2 (same priority
//     class as crazy_rx + sentai_safety_task — proven to schedule).
//   - Periodic 20 ms vTaskDelay between drain rounds (no condition
//     variable / counting semaphore; producers do not wake the
//     worker — they just enqueue and return).
//   - Stack: configMINIMAL_STACK_SIZE * 4 (POSIX pthread frames are
//     larger than ARM; fopen/fwrite stacks can be deep).
//
// Recovery:
//   - All errors are LOCAL to a drain attempt and bumped into FR
//     channel stats (`writes_fail`).  Worker never aborts; the loop
//     just continues.  s_stop_flag is the ONLY exit path.
//
// Safe state:
//   - On task_stop: signal flag, do a final bounded drain pass so
//     queued items don't leak, then vTaskDelete(nullptr).

#include "sentai_fr.h"
#include "sentai_fr_task.h"

#include <stdio.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

// Internal helper from sentai_fr.cc — ensures the binary semaphore
// used by drain_round() exists before the worker starts pulling.
extern "C" void sentai_fr_internal_mu_init(void);

namespace {

static constexpr uint32_t FR_POLL_MS    = 20;
static constexpr uint32_t FR_STOP_BUDGET = 30;   // × 50 ms = 1.5 s
static constexpr uint32_t FR_FINAL_DRAINS = 16;  // bounded best-effort

static TaskHandle_t  s_task_handle  = nullptr;
static volatile bool s_started      = false;
static volatile bool s_stop_flag    = false;
static volatile bool s_worker_alive = false;   // set on entry, cleared
                                                // on exit; lets stop()
                                                // join return early

void worker_main_loop_() {
    // One-shot start / end markers so operators can confirm in stderr
    // that the recorder task was actually scheduled.  Per-tick prints
    // were used during s170 bring-up but are noise once stable.
    fprintf(stderr, "[sentai_fr] worker START\n");
    uint32_t round         = 0;
    uint32_t total_drained = 0;
    while (!s_stop_flag) {
        vTaskDelay(pdMS_TO_TICKS(FR_POLL_MS));
        total_drained += sentai_fr_drain_round();
        ++round;
    }
    // Final drain on stop — bounded so we never lose items just
    // because task_stop was called fast, but we don't loop forever if
    // a sink is wedged.
    for (uint32_t i = 0; i < FR_FINAL_DRAINS; ++i) {
        if (sentai_fr_drain_round() == 0) break;
    }
    fprintf(stderr,
            "[sentai_fr] worker STOP rounds=%u drained=%u\n",
            (unsigned)round, (unsigned)total_drained);
}

void worker_task_entry(void*) {
    s_worker_alive = true;
    worker_main_loop_();
    s_worker_alive = false;
    vTaskDelete(nullptr);
}

}  // namespace

extern "C" int sentai_fr_task_start(void) {
    if (s_started) return 0;                          // F1
    sentai_fr_internal_mu_init();                     // arm the lock
    s_stop_flag    = false;
    s_worker_alive = false;
    BaseType_t ok = xTaskCreate(
        worker_task_entry,
        "sentai_fr",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s_task_handle);
    if (ok != pdPASS || !s_task_handle) {             // F2
        fprintf(stderr, "[sentai_fr] xTaskCreate FAIL ok=%ld\n", (long)ok);
        return -1;
    }
    s_started = true;
    return 0;
}

extern "C" int sentai_fr_task_stop(void) {
    if (!s_started) return 0;                         // F3
    s_stop_flag = true;
    // Bounded join: break as soon as worker confirms exit by
    // clearing s_worker_alive (catches F4).
    for (uint32_t i = 0; i < FR_STOP_BUDGET; ++i) {
        if (!s_worker_alive) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    s_task_handle = nullptr;
    s_started     = false;
    return 0;
}

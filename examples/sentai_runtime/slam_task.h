// slam_task.h — InferTask-style perception loop for place recognition.
//
// OP-S10-W11-T3 per project_op_s10_w11_prep_pipeline.md.  Embedded
// perception task: wakes on PrepTask signal AFTER SLOT_RGB_64 has
// been written, computes HSV descriptor on the 64×64 RGB888 slot,
// queries the sentai.places L3 gallery, publishes the current_match.
//
// Why a task (not on-demand from MP):
//   - [[no-heavy-data-through-mp]] HARD RULE — never expose camera
//     frames through the MP heap.  Slot pointer stays in C; only the
//     small match struct {id, score, l1_dist, seq} crosses to MP.
//   - [[arm-hw-primitives-first]] — SLOT_RGB_64 is produced by PrepTask
//     via sentai_pxp_scale (PXP HW, XRGB→RGB888 + scale to 64×64).
//   - Mirror of detection_task.cc:infer_task_fn — same static-task,
//     same counting-sem coalescing, same atomic publish.
//
// System model (per agent/embeded.md §1.1–§1.4):
//   Fault model   F1 sem timeout   → frames_dropped++, retry next signal
//                 F2 slot_get fail → frames_dropped++, retry next signal
//                 F3 hsv_compute<0 → frames_dropped++, retry next signal
//   Execution     bounded loop (`while (s_running)` is the only loop);
//                 take(sem, 500 ms) — never blocks longer than 500 ms;
//                 no malloc, no recursion, no shared mutable state
//                 except `s_running` (volatile) + the publish struct.
//   Recovery      no escalation — caller invokes sentai_slam_stop() to
//                 quiesce; safe state = task self-deletes, slot
//                 refcount released, last published result preserved.
//   Safe state    `is_running=false` + slot refcount=0 + sem deleted.
//
// Concurrency contract:
//   - Producer (PrepTask) calls sentai_slam_signal_frame() AFTER
//     sentai_prep_slot_commit(SLOT_RGB_64).  Signal is a counting-sem
//     give() with max=1, so multiple signals between two consumer
//     wakes coalesce — no backlog.
//   - Single consumer (SlamTask).  No reentrancy.
//   - Publish: result_seq incremented after __DMB so MP readers can
//     detect a fresh result by sampling seq.
//
// SIM build: same task body, same lifecycle.  Real frames only land
// once Phase 1d wires camera_bridge_recv as a SLOT_RGB_64 producer.
// Until then, SlamTask on SIM waits on the sem (timeout every 500 ms)
// and reports `frames_dropped` ticking — that's the expected idle
// state on SIM pre-Phase 1d.
//
// Lifetime:
//   sentai_slam_start() → creates sem (idempotent), enables
//   SLOT_RGB_64 refcount, creates task → 0 on success.
//   sentai_slam_stop()  → s_running=false, gives sem once to unblock
//   wait, waits up to 1 s for task self-delete, disables refcount.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Latest published descriptor-match snapshot.  All fields read by MP.
// Fits in 16 bytes — safe to copy across MP boundary as a small struct.
typedef struct {
    uint8_t  match_id;          // sentai_places match id (0 = no match)
    uint8_t  score_pct;         // 0..100
    uint16_t l1_dist;           // raw L1 byte distance
    uint32_t frame_seq;         // SLOT_RGB_64 seq from which this match came
    uint32_t result_seq;        // monotonic; bumps every publish
    uint32_t t_compute_us;      // last hsv+query duration (microseconds)
} sentai_slam_result_t;

typedef struct {
    uint32_t frames_processed;  // successful hsv+query cycles
    uint32_t frames_dropped;    // sem timeout OR slot_get fail OR hsv fail
    uint32_t avg_compute_us;    // EMA-free moving average over last 16
    uint32_t last_compute_us;
    uint8_t  is_running;        // 1 if task is alive
} sentai_slam_stats_t;

// Lifecycle.  Idempotent — calling start() while running is a no-op
// (returns 0).  Calling stop() while not running is also a no-op.
//
// ARM prereqs (enforced — start() fails if missing):
//   1. sentai.camera.init(streaming=1, ...)  → enables CSI + PXP HW
//      (BOARD_InitPxp runs inside HandleEnableRequest, only triggered
//      by sentai.camera.init).  See [[pxp-init-required]].
//   2. sentai.pipeline.start()               → starts PrepTask, which
//      is the SLOT_RGB_64 producer.  Without this, SlamTask wakes on
//      its 500-ms timeout cycle and reports idle (frames_processed=0).
//
// SIM: no equivalent gates.  The frame producer is
// `camera_bridge_recv` which is always live once `sentai_sim` is up;
// Phase 1d will wire SLOT_RGB_64 production into it.  Until then,
// start_slam() returns 0 but the task idles.
//
// Return codes:
//    0  success
//   -1  sem allocation failed
//   -2  slot enable failed
//   -3  task allocation failed
//  -10  ARM-only: sentai.camera.init not called
//  -11  ARM-only: sentai.pipeline.start not called
int  sentai_slam_start(void);
int  sentai_slam_stop(void);
int  sentai_slam_is_running(void);

// Snapshot accessors — copy the latest values out under brief DMB
// fencing.  Safe to call from any task context (MP or C).
void sentai_slam_get_current(sentai_slam_result_t* out);
void sentai_slam_get_stats(sentai_slam_stats_t* out);

// Producer-side hook called by PrepTask AFTER sentai_prep_slot_commit
// for SLOT_RGB_64.  No-op when SlamTask is not running.  Non-blocking;
// ISR-safe (counting-sem give from ISR if ever needed).
void sentai_slam_signal_frame(void);

// Helper: PXP-scale XRGB8888 → RGB888P 64×64 into SLOT_RGB_64 and
// signal SlamTask.  Pulled out of detection_task.cc:prep_task_fn so
// the code lives in .sentai_slow (slam_task.cc.obj) and doesn't
// inflate m_text.  No-op when slot is disabled.  Returns 0 on
// success, negative on PXP / slot failure.
int sentai_prep_publish_slot_rgb_64(const uint8_t* raw_xrgb,
                                     int src_w, int src_h);

#ifdef __cplusplus
}
#endif

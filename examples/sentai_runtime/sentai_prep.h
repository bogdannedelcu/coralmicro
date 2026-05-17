// sentai_prep.h — pre-processed camera-frame slot infrastructure.
//
// Producer (PrepTask on ARM, camera_bridge_recv on SIM) writes one
// or more "prep slots" per camera frame, each at a known resolution
// and format.  Consumer tasks (InferTask, SlamTask, AnchorTask, …)
// read the latest finalised slot via a zerocopy C accessor — NEVER
// through the MicroPython binding boundary (per
// [[no-heavy-data-through-mp]]).
//
// Producer model — non-blocking, continuous, ISR-friendly:
//   - Each frame: producer writes every ENABLED slot's buffer, emits
//     a __DMB() memory barrier, then increments slot.seq.  No
//     semaphore is taken on the producer side for aux slots.
//   - The TPU staging_buf path stays sem-gated for InferTask (lossless,
//     existing behaviour preserved in detection_task.cc).
//   - Designed so the aux-slot loop body is eventually callable from
//     the CSI EOF ISR directly (no FreeRTOS blocking primitives in
//     this loop; only the staging_buf path needs task context).
//
// Consumer model — last-frame-wins, lossy OK:
//   - sentai_prep_slot_get(SLOT_ID, &buf, &w, &h, &seq) returns the
//     pointer to the live buffer + dims + current seq.  Cold-path
//     consumers (HSV @ ~1 Hz, ArUco place-recognition @ ~1 Hz)
//     accept tearing across the rare frame-boundary read.
//   - For tear-free read, caller may sample seq before+after the
//     read and retry on collision (rare at typical consumer rates).
//
// Enable refcount:
//   - sentai_prep_slot_enable(SLOT_ID) increments refcount.
//   - sentai_prep_slot_disable(SLOT_ID) decrements.
//   - Slot is populated iff refcount > 0.  Multiple consumers can
//     hold a slot open without coordination.
//
// Per-slot skip-frame (future, fields present but unused):
//   - slot.frame_div = N → producer runs the PXP for that slot every
//     N-th frame only.  N=1 = every frame (default), N=4 = quarter
//     rate, etc.  Wired in once we have an empirical reason; today
//     all enabled slots fire every frame.
//
// Cross-platform: same .cc compiles for ARM (PrepTask producer with
// PXP HW) and SIM (camera_bridge_recv producer with scalar resize).
// Storage attribute differs via SENTAI_PREP_BSS macro.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Slot identifiers ------------------------------------------------
// Append only; never renumber (consumers refer by ID).
typedef enum {
    SENTAI_PREP_SLOT_GRAY_NATIVE = 0,    // Y8,   320×240 (or native scaled)
    SENTAI_PREP_SLOT_RGB_64      = 1,    // RGB888 packed, 64×64
    SENTAI_PREP_SLOT_GRAY_64     = 2,    // Y8,   64×64
    SENTAI_PREP_SLOT_COUNT
} sentai_prep_slot_id_t;

// ---- Slot formats ----------------------------------------------------
typedef enum {
    SENTAI_PREP_FMT_Y8     = 0,     // 1 byte/pixel
    SENTAI_PREP_FMT_RGB888 = 1,     // 3 bytes/pixel (packed R,G,B)
} sentai_prep_fmt_t;

// ---- Public stats / monitoring ---------------------------------------
typedef struct {
    uint32_t frames_total;          // PrepTask iterations seen
    uint32_t frames_with_aux;       // frames where ≥1 aux slot fired
    uint32_t producer_overruns;     // consumer was reading when producer wrote
                                    // (best-effort, optional to track)
    uint8_t  slot_refcount[SENTAI_PREP_SLOT_COUNT];
    uint32_t slot_seq[SENTAI_PREP_SLOT_COUNT];
} sentai_prep_stats_t;

// =====================================================================
// API
// =====================================================================

// Initialise the slot table (zero counters, refcount=0).  Safe to call
// multiple times.  Must be called before any enable/get call.  Today
// this is invoked from sentai_runtime app_main; consumers do not need
// to call it directly.
void sentai_prep_init(void);

// Refcount-based enable/disable.  Slot is populated by the producer
// iff refcount > 0.  Multiple consumers can hold the same slot open.
// Returns the new refcount on success, -1 on invalid slot id.
int sentai_prep_slot_enable(sentai_prep_slot_id_t id);
int sentai_prep_slot_disable(sentai_prep_slot_id_t id);

// Read-only check.
int sentai_prep_slot_is_enabled(sentai_prep_slot_id_t id);

// Zerocopy read.  Returns 0 on success and fills buf/w/h/seq with the
// latest published snapshot for the slot.  Returns -1 if the slot id
// is invalid, the slot is disabled (refcount == 0), or no frame has
// been produced yet (seq == 0).  Caller MUST consume buf before the
// next frame interval — buffer is single-slot last-frame-wins.
int sentai_prep_slot_get(sentai_prep_slot_id_t id,
                          const uint8_t** out_buf,
                          int* out_w, int* out_h,
                          uint32_t* out_seq);

// Optional skip-frame divider (future hook — today the producer
// ignores it and runs every enabled slot per frame).  N=1 = every
// frame; N=2 = every other; ≤ 0 reverts to 1.  Returns 0 on success.
int sentai_prep_slot_set_div(sentai_prep_slot_id_t id, int n);

// Producer-side write API — used ONLY by PrepTask (ARM) /
// camera_bridge_recv (SIM).  Consumers must not call these.
//
// _begin returns a writable buffer pointer + dims for the slot, or
// NULL if disabled.  Caller writes pixels into the buffer, then calls
// _commit which emits __DMB() and increments slot.seq atomically.
uint8_t* sentai_prep_slot_begin_write(sentai_prep_slot_id_t id,
                                       int* out_w, int* out_h);
void     sentai_prep_slot_commit(sentai_prep_slot_id_t id);

// Producer-side: increment frames_total counter (called by PrepTask
// once per camera frame regardless of slot state).  Sets up the
// per-slot frame_counter for the skip-frame logic.  Returns the
// uint32_t mask of slots whose frame_counter aligned this frame
// (slot will be produced).
uint32_t sentai_prep_tick_frame(void);

// Stats dump for diagnostics / pipeline.prep_stats.
void sentai_prep_get_stats(sentai_prep_stats_t* out);
void sentai_prep_reset_stats(void);

// ---- Format / dimension helpers (compile-time look-up) --------------
sentai_prep_fmt_t sentai_prep_slot_fmt(sentai_prep_slot_id_t id);
int sentai_prep_slot_max_w(sentai_prep_slot_id_t id);
int sentai_prep_slot_max_h(sentai_prep_slot_id_t id);
int sentai_prep_slot_buf_size(sentai_prep_slot_id_t id);

#ifdef __cplusplus
}
#endif

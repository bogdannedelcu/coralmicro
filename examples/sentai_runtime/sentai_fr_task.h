// sentai_fr_task.h — OP-S10-W13-T2 worker side of the Flight Recorder.
//
// The FR module is split into a `state + push + drain primitives` half
// (sentai_fr.{h,cc}) and a `worker task` half (this header + .cc),
// mirroring the sentai.safety split (state machine in sentai_safety.cc,
// camera FPS worker in sentai_safety_task.cc).  This keeps the
// disk-backed I/O and the FreeRTOS task lifecycle isolated from the
// channel state / push API — same pattern, same testability story.
//
// The worker calls `sentai_fr_drain_round()` from sentai_fr.h on a
// 20 ms cadence; it knows nothing about channel internals.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Spawn the drain task.  Idempotent.  Uses xTaskCreate at
// tskIDLE_PRIORITY + 2 (same priority class as crazy_rx /
// sentai_safety_task — proven to schedule reliably under FreeRTOS
// POSIX SIM).  Returns 0 on success, negative on failure.
int sentai_fr_task_start(void);

// Signal stop + bounded join (≤1.5 s).  Idempotent.  Triggers a final
// drain pass so items already in the queues are flushed before exit.
int sentai_fr_task_stop(void);

#ifdef __cplusplus
}
#endif

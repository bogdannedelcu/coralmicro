// sentai_crazy_log.h — CRTP LOG subscription (C port of crtp_log.py).
//
// Task #44.  Replaces ~270 LoC of pure-MP crtp_log.py with a single C
// translation unit shared by ARM and SIM builds.  Mission code stops
// importing crtp_log.py; instead it calls sentai.crazy.pose_subscribe()
// once and sentai.crazy.pose() to read the latest (x,y,z,yaw) tuple.
//
// Rationale: the wire format is frozen (Bitcraze cflib v8) and the LOG
// state machine is identical across every mission.  Doing it in MP burns
// ~32 KB MP heap (TOC dict + struct.pack frames) and ~50 ms per
// scan_toc call.  Doing it in C keeps the MP VM available for actual
// mission logic.
//
// SYSTEM MODEL (NASA/JPL §A, see agent/embeded.md)
// =================================================
// Fault model:
//   F1 subscribe before sentai_crazy_init() ...... return -1
//   F2 TOC scan times out ........................ return -2
//   F3 required TOC entry not found .............. return -3
//   F4 CREATE_BLOCK / START_LOGGING NAK .......... return -4 / -5
//   F5 pose() before subscribe ................... return -1
//
// Execution model:
//   - subscribe()/pose()/stop() are MP-task context (REPL or mission).
//     No ISR access, no blocking in critical sections.
//   - subscribe() blocks bounded ≤ scan_timeout_ms + 500 ms (default
//     ~3.5 s — same envelope as crtp_log.py's scan_toc + create_block).
//   - pose() is non-blocking; it drains the RX FIFO ≤ 16 packets per
//     call and returns the latest cached snapshot (single-writer ←
//     drain callback inside pose(); single-reader ← MP caller).
//   - stop() sends STOP_LOGGING + DELETE_BLOCK and clears state.
//
// Concurrency:
//   - Internal state struct is single-writer (the calling MP task) and
//     does NOT need locking on either platform; the RX FIFO it drains
//     IS multi-writer/single-reader (the C transport task writes, MP
//     reads), but that contract is owned by sentai_crazy_recv_pop().
//
// Memory:
//   - One static state struct (~80 B) in .bss / .sdram_bss.
//   - No heap, no per-call malloc.

#ifndef SENTAI_CRAZY_LOG_H_
#define SENTAI_CRAZY_LOG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Reset all log blocks on cf2.  Idempotent.  Called automatically by
// sentai_crazy_pose_subscribe(); exposed so missions that share a cf2
// across multiple subscribe/unsubscribe cycles can force a clean slate.
//   Return: 0 on success, -1 if transport not initialised.
int sentai_crazy_log_reset(void);

// Subscribe to (stateEstimate.x, .y, .z, stabilizer.yaw) at period_ms.
// Internally: RESET → scan TOC (early-exit on the 4 targets) →
// CREATE_BLOCK_V2 → START_LOGGING.
//
// period_ms is rounded to nearest 10 ms (cf2 firmware quantises log
// rate to 10 ms units).  Recommended ≥ 50 ms for cf2 SITL stability.
//
//   Return: 0 ok, -1..-5 per fault model above.
int sentai_crazy_pose_subscribe(int period_ms);

// Read latest pose.  Drains the CRTP RX FIFO ≤ 16 packets to update
// the cache, then writes through the out pointers.  Returns 0 on
// success (cache valid since at least one LOGDATA frame arrived), -1
// if not subscribed, -2 if no frame has arrived yet.
//
// out_x/out_y/out_z/out_yaw may be NULL individually (caller can
// request a subset).
int sentai_crazy_pose(float* out_x, float* out_y, float* out_z, float* out_yaw);

// Stop + delete the log block.  Idempotent.  Always returns 0.
int sentai_crazy_pose_stop(void);

// True iff a pose subscription is active AND at least one LOGDATA frame
// has been consumed since subscribe().  Mirrors the "is pose ready"
// check that missions used to do via `latest_pose() is not None`.
int sentai_crazy_pose_ready(void);

// Diagnostics — copy current LOG state to caller.  Useful for verdict.
//   out[0] = subscribed (0/1)
//   out[1] = toc_n_items (last scan)
//   out[2] = data_frames_received
//   out[3] = block_id (0 if not subscribed)
void sentai_crazy_log_stats(uint32_t out[4]);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_CRAZY_LOG_H_

// sentai_fr.h — ObjectsPlan OP-S10-W13: Flight Recorder (FR) subsystem.
//
// A NASA/JPL-style independent recorder, separated from mission and
// safety logic per agent/embeded.md §3.1 (strict layer separation) +
// §7.2 (structured event log: "Persistent breadcrumbs for post-mortem
// ... Bounded size + atomic writes (ring buffer)").
//
// Aviation analogy: the Flight Data Recorder (FDR) + Cockpit Voice
// Recorder (CVR) are dedicated subsystems that NEVER live inside the
// flight control loop.  This module is the project's equivalent —
// producers (SafetyTask, mission, flow_task, etc.) push items into
// channel queues and continue; a single recorder task drains queues
// to disk asynchronously.  Producers NEVER block on I/O.
//
// First scope (SIM only, OP-S10-W12-T6 SafetyArucoBaseline debug):
//   - channel `frames` — per-frame gray dump as PGM with name
//       `t<ms>_n<n_dets>_f<seq>.pgm`
//   - channel `events` — text events, CSV append (PX4-style ULog spirit
//       simplified: one line per event, no JSON.  Format per line:
//       `<ts_ms>,<event_type>,<text>` — comma-separated, text-only.
//       Operator spec 2026-05-18: "din MP nu vom publica jsoane, e
//       prea complicat... event type si TEXT, poate coma separated").
//   - channel `scalars` — CSV append.  Format per line:
//       `<ts_ms>,<label>,<value>` — single scalar per push.
//   - channel `kernel`  — mirror of sentai_dmesg ring (stub T1)
//
// ARM port is OUT OF SCOPE for this WP — `__ARM_ARCH` paths exist for
// thread/mutex but the disk-backend sinks are SIM-only initially.
// On ARM, FxUser will replace the host stdio fopen path in a later T.
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A) — KEY DECISIONS
// =========================================================================
// Fault model (FR is best-effort by design — recorder failure NEVER
// affects mission/safety, only post-mortem visibility):
//   F1  open_channel(unknown ch enum)            -> -1 unknown
//   F2  open_channel(channel already open)       -> 0 (idempotent — path updated)
//   F3  open_channel(path NULL/empty)            -> -2 bad_params
//   F4  push_*(channel not open)                 -> 0 (silent no-op)
//   F5  push_*(channel queue full)               -> drops++, no signal (audit only)
//   F6  recorder task disk write fail            -> fails++, item dropped, continue
//   F7  task_start when running                  -> 0 idempotent
//   F8  task_stop with pending items in queue    -> drains best-effort then exits
//
// Execution model:
//   - Single recorder task / pthread.  Period: event-driven (waits on a
//     binary semaphore signalled by every push), or 10 ms fallback poll.
//   - Producers call sentai_fr_push_* from any task / ISR-free context.
//     Push acquires a brief channel-level mutex, copies item to next
//     ring slot, signals the semaphore, releases mutex.  O(1).
//   - Bounded queues per channel (slot pool sized at init from compile-
//     time constants — no malloc post-init).
//   - SIM disk I/O: fopen/fwrite/fclose per item, dir path opened at
//     init.  Writes are not fsync'd — recorder is best-effort.
//
// Recovery:
//   - All errors are LOCAL.  Producer's push always returns 0 (no
//     channel back-pressure into hot path).  Recovery is observable
//     via stats counters (writes/drops/fails).
//
// Safe state:
//   - task_stop drains queue best-effort within a bounded join (≤ 1 s),
//     then exits.  Channels stay "open" but no further drains until
//     task_start.
//
// Memory (SIM):
//   - frames channel: 16-slot pool × 76800 B (320×240 gray) = 1.2 MB,
//     static SDRAM.
//   - events channel: 256 slots × 96 B (timestamp + label + payload) = 24 KB.
//   - scalars channel: 1024 slots × 24 B = 24 KB.
//   - Total static BSS ≈ 1.3 MB.
//   - Zero heap, zero per-call malloc.
//
// Anti-cheat invariants:
//   - FR only records what producers feed it.  It does NOT read camera
//     buffers, sensors, or any source directly — keeps the boundary
//     clean.  Producers (e.g. SafetyTask) are responsible for what
//     they push.
//   - Recorder output is post-mortem-only consumed by host scripts;
//     never injected into mission or safety.
//
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Channel registry --------------------------------------------------
// Fixed enum (not dynamic strings) for embed-friendly dispatch.
typedef enum {
    SENTAI_FR_CH_NONE    = 0,
    SENTAI_FR_CH_FRAMES  = 1,   // gray PGM per push: t<ms>_n<n>_f<seq>.pgm
    SENTAI_FR_CH_EVENTS  = 2,   // JSONL append (stub T1)
    SENTAI_FR_CH_SCALARS = 3,   // CSV append (stub T1)
    SENTAI_FR_CH_KERNEL  = 4,   // sentai_dmesg mirror (stub T1)
    SENTAI_FR_CH__COUNT          // sentinel
} sentai_fr_channel_t;

// ---- Reject codes ------------------------------------------------------
typedef enum {
    SENTAI_FR_OK            =  0,
    SENTAI_FR_ERR_UNKNOWN   = -1,   // unknown channel enum / name
    SENTAI_FR_ERR_PARAMS    = -2,   // path null / empty / invalid
    SENTAI_FR_ERR_STATE     = -3,   // task not in expected state
    SENTAI_FR_ERR_FULL      = -4,   // queue full (push) — usually swallowed
    SENTAI_FR_ERR_IO        = -5,   // disk write failed
} sentai_fr_status_t;

// ---- Capacities (compile-time; tune via -D…) --------------------------
#ifndef SENTAI_FR_FRAMES_SLOTS
#define SENTAI_FR_FRAMES_SLOTS    16
#endif
#ifndef SENTAI_FR_FRAMES_MAX_W
#define SENTAI_FR_FRAMES_MAX_W    640
#endif
#ifndef SENTAI_FR_FRAMES_MAX_H
#define SENTAI_FR_FRAMES_MAX_H    480
#endif
#define  SENTAI_FR_FRAMES_BYTES   (SENTAI_FR_FRAMES_MAX_W * SENTAI_FR_FRAMES_MAX_H)

#ifndef SENTAI_FR_EVENTS_SLOTS
#define SENTAI_FR_EVENTS_SLOTS    256
#endif
#define  SENTAI_FR_EVENT_TYPE_LEN     20    // short tag like "abort"
#define  SENTAI_FR_EVENT_TEXT_LEN     96    // free-form text

#ifndef SENTAI_FR_SCALARS_SLOTS
#define SENTAI_FR_SCALARS_SLOTS   1024
#endif
#define  SENTAI_FR_SCALAR_LABEL_LEN  16

// ---- Per-channel stats (for sentai_fr_get_stats) ----------------------
typedef struct {
    uint8_t   enabled;
    uint8_t   _pad[3];
    uint32_t  pushes_total;       // every push attempt
    uint32_t  pushes_accepted;    // pushes_total - drops_full
    uint32_t  drops_full;         // pushes refused because queue full
    uint32_t  writes_ok;          // recorder wrote to disk successfully
    uint32_t  writes_fail;        // recorder I/O failure
    uint32_t  queue_depth;        // current items pending drain
    uint32_t  worst_queue_depth;
    char      path[128];          // current output path (dir or file)
} sentai_fr_channel_stats_t;

// =======================================================================
// API
// =======================================================================

// One-time init.  Idempotent.  Initialises slot pools, mutexes, etc.
// Does NOT start the recorder task — call sentai_fr_task_start() for
// that explicitly.  Returns 0.
int sentai_fr_init(void);

// Open (enable) a channel with an output path:
//   FRAMES : path is a DIR (created if missing) → PGM per push
//   EVENTS : path is a FILE → CSV append `<ts_ms>,<type>,<text>\n`
//   SCALARS: path is a FILE → CSV append `<ts_ms>,<label>,<value>\n`
//   KERNEL : path is a FILE → sentai_dmesg snapshot mirror
// Re-open is idempotent (path updated, stats preserved).
//
// EVENTS file gets a header line on first open:
//   `# sentai.fr events  ts_ms,type,text` (comments start with `#`)
// SCALARS likewise:
//   `# sentai.fr scalars ts_ms,label,value`
int sentai_fr_open(sentai_fr_channel_t ch, const char* path);

// Disable a channel.  In-flight items already in the queue are dropped
// (their slots returned to the pool) on the next recorder pass.
int sentai_fr_close(sentai_fr_channel_t ch);

// ── Recorder task lifecycle ─────────────────────────────────────────
// task_start spawns the drain thread.  task_stop signals + joins.
int sentai_fr_task_start(void);
int sentai_fr_task_stop(void);

// ── Producer push endpoints ─────────────────────────────────────────
// All push_* are O(1), non-blocking (brief mutex only), returning
// SENTAI_FR_OK on accept or SENTAI_FR_ERR_FULL on drop.  Callers
// SHOULD NOT branch on the return for control flow — drops are
// telemetry, not policy.
int sentai_fr_push_frame(const uint8_t* gray, int w, int h,
                          int n_dets, uint32_t seq, uint32_t ts_ms);

// Text event — `type` is a short tag (e.g. "abort", "n_dets_change"),
// `text` is free-form (commas are escaped/stripped at recorder time).
// Both must be NUL-terminated; truncated if longer than per-slot limits.
// NO JSON — operator-mandated minimal format (2026-05-18).
int sentai_fr_push_event(const char* type, const char* text);

// Scalar trace — single label + double per push.  Recorder emits
// `<ts_ms>,<label>,<value>\n`.  Caller provides ts_ms; FR does not
// timestamp internally (lets producers correlate across channels).
int sentai_fr_push_scalar(const char* label, double value, uint32_t ts_ms);

// ── Read-side ──────────────────────────────────────────────────────
int sentai_fr_get_stats(sentai_fr_channel_t ch,
                         sentai_fr_channel_stats_t* out);

#ifdef __cplusplus
}
#endif

// sentai_safety.h — ObjectsPlan OP-S10-W12: continuous mission-safety
// monitor in C/C++.  Mission MP code arms one or more checks at takeoff,
// then polls a single boolean flag; the firmware raises the flag as
// soon as any active check detects an unsafe condition.
//
// Per CLAUDE.md "compute-in-C, MP-as-glue" + [[no-safety-logic-in-explore]]
// + [[missions-run-in-sentai-only]]: safety monitoring belongs in C/C++
// on the firmware (ARM or sentai_sim), NEVER in host-side Python.  The
// host-side SafetyMonitor in s167/mission_flowbaseline2.py is an INTERIM
// hack — this module is its proper firmware replacement.
//
// ARCHITECTURAL PRINCIPLE — REUSE, DO NOT DUPLICATE:
//   sentai.safety is a STATE MACHINE only.  It does not:
//     - call detect_in_ppm / sentai_aruco_detect itself
//     - read camera frame buffers
//     - run PnP, FFT, or any per-pixel compute
//   It consumes results from upstream subsystems via push:
//     - sentai_safety_on_aruco_result(n_dets, seq, ts) — fed by the
//       SafetyTask worker which calls sentai_aruco_detect using the
//       existing sentai_camera_grab_gray_zerocopy() hook
//     - (future) sentai_safety_on_alt_pose(z_pnp) — fed when calib
//       computes a refreshed PnP-z
//     - (future) sentai_safety_on_battery(v_batt) — fed by sentai.crazy
//     - (future) sentai_safety_on_link_keepalive(ts) — fed by sentai.crazy
//
// The companion `sentai_safety_task.cc` is the ONLY new continuous loop:
// it polls the camera shared memory at camera FPS, runs ArUco via the
// existing detector, and pushes the result into this state machine.
// All other subsystems (aruco, camera, calib, pipeline) are reused.
//
// First check shipped: ArUco-FOV.  Mission says
// `sentai.safety.enable("aruco", n_min=4, max_loss_s=1.0)`; the
// SafetyTask updates the loss-streak timer; once
// `streak_s >= max_loss_s`, the abort flag latches and
// `sentai.safety.reason()` returns the diagnostic string.
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A)
// =========================================================================
// Fault model:
//   F1  enable(check)   with unknown check enum             -> -1 unknown
//   F2  enable(check)   with invalid params (NaN, <=0, …)   -> -2 bad_params
//   F3  enable(check)   when already enabled                -> 0 (idempotent)
//   F4  disable(check)  with unknown check                  -> -1 unknown
//   F5  on_aruco_result called before enable("aruco")       -> 0 (no-op)
//   F6  on_aruco_result with negative n_dets                -> clamp to 0
//   F7  read aborted/reason before init                     -> 0/""
//   F8  upstream feeder STOPS pushing (camera dead, etc.)   -> "stale data"
//        detected via (now - last_push_t > stale_timeout_s);
//        treated as a SEPARATE abort trigger so silent failures
//        of the upstream pipeline are not invisible.
//
// Execution model:
//   - Single-writer per check: on_aruco_result called by SafetyTask only,
//     never from MP or ISR.
//   - Multi-reader: MP bindings call aborted() / reason() / snapshot()
//     from any context (REPL, mission task).  Use atomic load on the
//     flag; reason / snapshot copied under a brief mutex.
//   - The abort flag is STICKY — once latched, only sentai_safety_clear()
//     resets it.  Mission cannot silently re-arm by re-enabling a check.
//
// Recovery:
//   - All errors are LOCAL (per-check, per-call).
//   - `sentai_safety_clear()` resets ALL flags + counters + event log
//     and deactivates all checks.  Use ONLY at mission re-arm
//     boundaries (e.g. start of a new trial).
//
// Memory:
//   - Static globals: per-check state structs (~ 256 B / check, max
//     SENTAI_SAFETY_CHK__COUNT = 8 checks defined = ~2 KB BSS) + event
//     ring (32 entries × 64 B = 2 KB).  Total ~4 KB BSS.
//   - Zero heap, zero per-call malloc.
//   - Routed to `.sdram_text` per ITCM budget [[itcm-budget]].
//
// Anti-cheat invariants (codified, audit-friendly):
//   - The ArUco feed comes from the real camera pipeline (camera_bridge_
//     recv on SIM, detection_task / CSI ISR on ARM).  Gazebo
//     /dynamic_pose is NEVER consumed.  See
//     [[sentai-sim-air-gapped-from-truth]] + [[cf2-sitl-cheat-odom-gt]].
//   - The abort flag is read-only from MP — there is no `ignore_safety`
//     API.  Only `clear()` resets, and only at known re-arm boundaries.
//   - Resetting `clear()` emits a SENTAI_SAFETY_EV_CLEAR event for the
//     audit trail (so silent re-arming can be detected post-mortem).
//
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Check registry ---------------------------------------------------
// Adding a new check = adding an enum entry + an `on_<name>_result`
// push function + the per-check state struct + dispatch in tick().
typedef enum {
    SENTAI_SAFETY_CHK_NONE      = 0,
    SENTAI_SAFETY_CHK_ARUCO     = 1,  // n_min markers visible for max_loss_s
    SENTAI_SAFETY_CHK_ALT_FLOOR = 2,  // PnP-z below floor for dwell_s (stub)
    SENTAI_SAFETY_CHK_EKF_CEIL  = 3,  // EKF z above ceiling (stub)
    SENTAI_SAFETY_CHK_BATTERY   = 4,  // V_batt below threshold (stub)
    SENTAI_SAFETY_CHK_LINK      = 5,  // CRTP keepalive older than … (stub)
    SENTAI_SAFETY_CHK__COUNT          // sentinel — keep last
} sentai_safety_check_t;

#define SENTAI_SAFETY_REASON_LEN        96
#define SENTAI_SAFETY_EVENTS_CAP        32      // ring buffer

// Stale-feed timeout: if the feeder hasn't pushed in this many seconds,
// the active check transitions to "stale" and contributes to abort the
// same way as a sustained loss.  Catches silent SafetyTask death.
#define SENTAI_SAFETY_STALE_TIMEOUT_S   2.0f

// ---- Per-check params --------------------------------------------------
typedef struct {
    int     n_min;          // required marker count (default 4)
    float   max_loss_s;     // abort if streak >= this (default 1.0)
} sentai_safety_aruco_params_t;

// (Future stubs — ABI placeholder; no implementation in T2.)
typedef struct { float z_floor_m; float dwell_s; }  sentai_safety_alt_floor_params_t;
typedef struct { float z_ceil_m;                  } sentai_safety_ekf_ceil_params_t;
typedef struct { float v_min;     float dwell_s;  } sentai_safety_battery_params_t;
typedef struct { float max_silent_s;              } sentai_safety_link_params_t;

// ---- Reject codes ------------------------------------------------------
typedef enum {
    SENTAI_SAFETY_OK            =  0,
    SENTAI_SAFETY_ERR_UNKNOWN   = -1,  // check name not recognised
    SENTAI_SAFETY_ERR_PARAMS    = -2,  // params invalid (non-finite, <=0, …)
    SENTAI_SAFETY_ERR_STATE     = -3,  // operation in invalid state
} sentai_safety_status_t;

// ---- Event log entry (~ 32 B) -----------------------------------------
typedef enum {
    SENTAI_SAFETY_EV_ENABLE     = 0,
    SENTAI_SAFETY_EV_DISABLE    = 1,
    SENTAI_SAFETY_EV_LT_N_MIN   = 2,   // n_dets dropped below n_min
    SENTAI_SAFETY_EV_RECOVER    = 3,   // n_dets returned to n_min
    SENTAI_SAFETY_EV_ABORT      = 4,   // streak hit max_loss_s
    SENTAI_SAFETY_EV_STALE      = 5,   // no feed for STALE_TIMEOUT_S
    SENTAI_SAFETY_EV_CLEAR      = 6,   // safety cleared (audit trail)
} sentai_safety_evkind_t;

typedef struct {
    uint32_t    t_ms;             // monotonic ms when event recorded
    uint8_t     check;            // sentai_safety_check_t
    uint8_t     kind;             // sentai_safety_evkind_t
    uint16_t    _pad;
    // Per-event payload (union to save space).  Fields meaningful only
    // for the (check, kind) combination; ignore otherwise.
    union {
        struct { int n_dets; int n_min; }                aruco_lt;
        struct { uint32_t prev_streak_ms; }              aruco_recover;
        struct { uint32_t streak_ms; int n_dets; }       aruco_abort;
        struct { uint32_t silent_ms; }                   stale;
        uint8_t                                          raw[16];
    } d;
} sentai_safety_event_t;

// ---- Aggregate snapshot for MP --------------------------------------
typedef struct {
    uint8_t     aborted;                    // 0/1, latched
    uint8_t     active_mask;                // bit per check_t
    uint8_t     abort_kind;                 // which check tripped
    uint8_t     _pad;
    uint32_t    abort_t_ms;                 // monotonic ms of latch
    char        reason[SENTAI_SAFETY_REASON_LEN];   // human-readable
    // ArUco check live state (useful for diag/REPL polling).
    int         aruco_last_n_dets;
    uint32_t    aruco_streak_ms;            // current streak below n_min
    uint32_t    aruco_n_frames_processed;
    uint32_t    aruco_last_push_t_ms;       // when SafetyTask last fed
} sentai_safety_snapshot_t;

// =======================================================================
// API
// =======================================================================

// One-time init.  Idempotent.  Clears all check state.  Returns 0.
int sentai_safety_init(void);

// Reset ALL checks + abort flag + event log.  Use ONLY at mission
// re-arm boundaries (e.g. start of a new trial).  Emits a CLEAR event
// for the audit trail.  Returns 0.
int sentai_safety_clear(void);

// ── Per-check enable / disable ─────────────────────────────────────────
//
// `aruco`:  params interpreted as sentai_safety_aruco_params_t*.
//   Validates: 1 <= n_min <= 16, max_loss_s > 0 && finite.
// Other checks (alt_floor, …): T-future, currently returns OK with no-op.
//
// On unknown check -> SENTAI_SAFETY_ERR_UNKNOWN.
// On invalid params -> SENTAI_SAFETY_ERR_PARAMS.
// Re-enable of an already-enabled check OVERWRITES its params and
// resets that check's streak/state (NOT the aborted flag — that
// remains latched until clear()).
int sentai_safety_enable(sentai_safety_check_t check, const void* params);
int sentai_safety_disable(sentai_safety_check_t check);

// ── PUSH endpoints (called by the feeders) ─────────────────────────────
//
// SafetyTask calls these after running upstream pipelines.  Each is
// idempotent vs duplicate seq (no double-counting if called twice with
// same frame_seq).  Returns 0 (no error path — feeders never fail safety).
//
// REUSE NOTE: SafetyTask invokes the EXISTING sentai_aruco_detect()
// (which uses the EXISTING sentai_camera_grab_gray_zerocopy()) and
// passes the resulting count + frame_seq + ts_ms here.  No detection
// pipeline is duplicated — sentai_aruco's cache is shared with any
// other consumer (mission, etc.) via sentai_aruco_get_latest().
int sentai_safety_on_aruco_result(int n_dets, uint32_t frame_seq, uint32_t ts_ms);

// (Future stubs — ABI placeholder; no-op in T2 implementation.)
int sentai_safety_on_alt_pose(float z_pnp_m, uint32_t ts_ms);
int sentai_safety_on_ekf_state(float ekf_z_m, uint32_t ts_ms);
int sentai_safety_on_battery(float v_batt_v, uint32_t ts_ms);
int sentai_safety_on_link_keepalive(uint32_t ts_ms);

// ── Read-side (MP-callable from any context) ───────────────────────────
//
// Lock-free snapshot — fast path for MP polling.  `out` is filled with
// the live aborted flag + per-check state.  Returns 0.
int sentai_safety_snapshot(sentai_safety_snapshot_t* out);

// Convenience accessors.
int          sentai_safety_is_aborted(void);
const char*  sentai_safety_reason(void);    // never NULL; "" if not aborted

// Read up to `cap` recent events; returns number written (0..cap).
int sentai_safety_get_events(sentai_safety_event_t* out, int cap);

// ── Internal: stale-feed watchdog tick ─────────────────────────────────
//
// SafetyTask (or any periodic tick) calls this every ~100 ms.  Checks
// each ACTIVE check's last-push timestamp against STALE_TIMEOUT_S; if
// stale, raises a STALE event and (if the check was already in a
// not-aborted state) may transition into abort.
//
// MP can also call this from `sentai.safety.tick()` as a fallback.
int sentai_safety_tick(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

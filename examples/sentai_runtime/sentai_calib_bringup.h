// sentai_calib_bringup.h — LEGACY OP-S10-W21-T4 unified bringup orchestrator.
//
// B5 note: this is from the old Flow-autotune bringup family.  It is not the
// target architecture for A3/B3 or A4/B4.  Keep temporarily for experiment
// replay; delete after the new sentai.calib/sentai.servo tasks replace it.
//
// Single on-board entry point that auto-discovers everything that varies
// drone-to-drone, persists the result, and leaves the drone hovering at
// z_hold for the caller to land.  Mirrors mission_flow_autotune.py's
// hand-stitched sequence in a single C state machine — the same code
// runs in Gazebo (digital twin) and on real HW.
//
// Phase sequence:
//   IDLE
//    └─ start()
//      ├─ SAMPLE       — hover at 4 corner poses, capture markers + EKF
//    │                  → cf2 EKF pose & per-marker tvec_cam → samples
//      ├─ KABSCH       — sentai_calib_run_kabsch on samples → R + quality
//      │                  cam_offset_B = mean(marker_W − R·tvec_cam − drone_W)
//      ├─ AUTOTUNE_X   — sentai_calib_task_start(X, ...) → kp_x
//      ├─ AUTOTUNE_Y   — sentai_calib_task_start(Y, ...) → kp_y
//      ├─ HOLD         — sentai_calib_hold_start(kp_x, kp_y, ...) validation
//      ├─ SAVE         — commit_R + commit_kp + save (calib.ini schema v2)
//      └─ DONE_OK or DONE_FAIL
//
// Contract:
//   - Caller has taken off; drone is airborne at ~z_hold.  Orchestrator
//     drives via hover() throughout, NEVER takes off or lands.  On any
//     phase fail, drone keeps hovering at last commanded setpoint; the
//     mission layer decides whether to land.  This mirrors [[no-safety-
//     logic-in-explore]]: bringup is a mission, not a safety FSM.
//   - sentai_markers_init(<backend>) + sentai_markers_set_marker_world()
//     done before start() — orchestrator only consumes detections.
//   - Triggered ONLY from REPL (sentai.calib.run_bringup(...)); NEVER
//     auto-flight at boot per [[sentai-calib-is-production-bringup]].
//
// SYSTEM MODEL (per agent/embeded.md §A) -----------------------------------
// Fault model:
//   F1 ctx ptr null / invalid geometry / dur fields out-of-range -> reject -1
//   F2 SAMPLE phase: no markers visible after settle              -> REJ_FEW_SAMPLES
//   F3 KABSCH phase: quality.accepted == 0                         -> REJ_KABSCH_QUAL
//   F4 AUTOTUNE_X/Y: SENTAI_CALIB_AT_DONE_FAIL / ABORTED           -> REJ_AUTOTUNE_X/Y
//   F5 HOLD: rms_drift > ctx.hold_rms_max_m                        -> REJ_HOLD_DRIFT
//   F6 SAVE: sentai_calib_save() returns 0                         -> REJ_SAVE
//   F7 sentai_safety_is_aborted() at any phase boundary            -> REJ_SAFETY
//
// Execution model:
//   - Single-writer: only one bringup task runs at a time (s_running gate).
//   - Spawns inner workers (sentai_calib_task_start / hold_start) which
//     OWN the drone command bus for their phases; orchestrator polls
//     task_is_done() and never sends hover() while inner worker active.
//   - Cooperative abort: bringup_abort() sets stop bit; observed at the
//     start of every phase + every sample-sweep tick.
//   - No ISR access.  Cold path (~30 Hz tick during SAMPLE; idle polling
//     1 Hz during inner phases).
//
// Recovery:
//   - All errors are LOCAL; on failure the partial result struct is
//     populated and last_phase records WHERE we stopped.  Caller decides
//     re-try vs land.  Persisted state on disk is only touched in SAVE
//     phase — earlier failures leave /system/calib.ini untouched.
//
// Memory:
//   - One FreeRTOS task @ configMINIMAL_STACK_SIZE * 4 (matches
//     sentai_calib_task — same callgraph depth).
//   - Static globals: ctx copy + result + 32-sample buffer ≈ 1.3 KB BSS.
//   - .sentai_slow placement (cold path).
// =========================================================================

#pragma once

#include <stdint.h>
#include "sentai_calib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENTAI_CALIB_BRINGUP_MAX_MARKERS  8
#define SENTAI_CALIB_BRINGUP_SWEEP_POSES  4
#define SENTAI_CALIB_BRINGUP_SAMPLES_MAX  SENTAI_CALIB_SAMPLES_MAX  // 32

typedef enum {
    SENTAI_CALIB_BRINGUP_OK               = 0,
    SENTAI_CALIB_BRINGUP_REJ_INVALID_CTX  = 1,
    SENTAI_CALIB_BRINGUP_REJ_FEW_SAMPLES  = 2,
    SENTAI_CALIB_BRINGUP_REJ_KABSCH_QUAL  = 3,
    SENTAI_CALIB_BRINGUP_REJ_AUTOTUNE_X   = 4,
    SENTAI_CALIB_BRINGUP_REJ_AUTOTUNE_Y   = 5,
    SENTAI_CALIB_BRINGUP_REJ_HOLD_DRIFT   = 6,
    SENTAI_CALIB_BRINGUP_REJ_SAVE         = 7,
    SENTAI_CALIB_BRINGUP_REJ_SAFETY       = 8,
    SENTAI_CALIB_BRINGUP_REJ_ABORTED      = 9,
} sentai_calib_bringup_reject_t;

typedef enum {
    SENTAI_CALIB_BRINGUP_PHASE_IDLE       = 0,
    SENTAI_CALIB_BRINGUP_PHASE_SAMPLE     = 1,
    SENTAI_CALIB_BRINGUP_PHASE_KABSCH     = 2,
    SENTAI_CALIB_BRINGUP_PHASE_AUTOTUNE_X = 3,
    SENTAI_CALIB_BRINGUP_PHASE_AUTOTUNE_Y = 4,
    SENTAI_CALIB_BRINGUP_PHASE_HOLD       = 5,
    SENTAI_CALIB_BRINGUP_PHASE_SAVE       = 6,
    SENTAI_CALIB_BRINGUP_PHASE_DONE_OK    = 7,
    SENTAI_CALIB_BRINGUP_PHASE_DONE_FAIL  = 8,
} sentai_calib_bringup_phase_t;

// Pad geometry + flight envelope + acceptance gates.  All caller-allocated;
// the bringup task takes a static copy at start() time.
typedef struct {
    // marker_world_n3[3*i + 0/1/2] = world (X, Y, Z) of marker with ID=i.
    // For ArUco IDs must match the dictionary IDs from the detector.
    // For WhyCon multi-marker constellation, IDs are sequence 0..N-1
    // assigned by the backend in scan order — caller is expected to
    // know that order from sentai_markers_set_marker_world().
    float    marker_world_n3[3 * SENTAI_CALIB_BRINGUP_MAX_MARKERS];
    int32_t  marker_n;          // 1..SENTAI_CALIB_BRINGUP_MAX_MARKERS
    float    marker_size_m;     // forwarded to sentai_calib_set_context

    // Hover envelope.
    float    z_hold_m;          // bringup altitude (e.g. 0.78)
    float    sweep_radius_m;    // half-width of corner sweep square (e.g. 0.10)
    float    settle_s;          // per-pose settle time (e.g. 2.0)
    float    vmax_m_s;          // autotune relay magnitude (e.g. 0.10)
    float    dur_relay_s;       // autotune per-axis timeout (e.g. 30.0)
    float    dur_hold_s;        // validation hold duration (e.g. 10.0)

    // Acceptance gates.
    float    hold_rms_max_m;    // hold rms threshold (e.g. 0.030 = 30 mm)
} sentai_calib_bringup_ctx_t;

typedef struct {
    int32_t  accepted;          // 0/1
    int32_t  reject_code;       // sentai_calib_bringup_reject_t
    int32_t  last_phase;        // sentai_calib_bringup_phase_t (where we stopped)

    // Recovered extrinsics.
    float    R_cam_to_body[9];
    float    cam_offset_B[3];
    sentai_calib_quality_t ext_quality;

    // Recovered gains.
    float    kp_x, kp_y;

    // Hold metrics.
    float    hold_max_drift_m;
    float    hold_rms_drift_m;

    // Sample / timing.
    int32_t  n_samples_used;
    uint32_t total_duration_ms;
} sentai_calib_bringup_result_t;

// Spawn the bringup task.  Non-blocking; poll via is_done() / get_phase().
// Preconditions (caller's responsibility — orchestrator only validates ctx):
//   - sentai_markers_init(...) done, marker_world_n3 matches geometry
//   - cf2 hl_takeoff completed + hl_stop issued (Generic Setpoints win)
//   - sentai_crazy_pose_subscribe(..) active (drone_W readable)
// Returns:
//    0 on spawn-success
//   -1 on invalid ctx
//   -2 on already-running
//   -3 on xTaskCreate failure
int sentai_calib_bringup_start(const sentai_calib_bringup_ctx_t* ctx);

// 1 once worker has reached DONE_OK or DONE_FAIL; 0 while running or idle.
int sentai_calib_bringup_is_done(void);

// Current phase — live; safe to poll any time.
sentai_calib_bringup_phase_t sentai_calib_bringup_get_phase(void);

// Read result.  Returns 0 on success (out populated).  Safe to call
// pre-done; populated fields reflect progress so far, others are zero.
int sentai_calib_bringup_get_result(sentai_calib_bringup_result_t* out);

// Cooperative abort.  Stop bit observed at every phase boundary.  Drone
// keeps hovering at last commanded setpoint; caller responsible for land.
// Returns 0 always (idempotent).
int sentai_calib_bringup_abort(void);

#ifdef __cplusplus
}
#endif

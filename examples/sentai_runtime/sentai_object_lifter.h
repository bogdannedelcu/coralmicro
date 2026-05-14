// sentai_object_lifter.h — ObjectsPlan L5 (Stage 5): inverse-depth EKF
// landmark lifter (Civera/Davison/Montiel TRO 2008).
//
// Per-tracklet 1-state EKF: scalar inverse depth ρ + variance σ_ρ². The
// (anchor_w, r_w) frame is captured at FIRST observation (anchored
// inverse-depth parametrization). Class-prior pseudo-depth seeds ρ₀
// from bbox apparent width: d₀ = fx · real_size / bbox_w_px.
//
// L5 scope: per-tracklet EKF storage + update + read.  Publication of
// converged landmarks (status == LIFTER_LIFTED) into sentai.objects is
// driven by the host (REPL / detection_task) via _publish_ready().
// No camera ISR access, no FreeRTOS primitives — all bounded math.
//
// Math reference: examples/sentai_runtime/experiments/s131_lifter_replay/
// — Python prototype validated 2026-05-15 (synth: 1.5mm/2.6cm err
// over near + far scenarios; commit 898e8d05).
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A)
// =========================================================================
// Fault model:
//   F1  init(non-finite bbox / drone pose) → -1 invalid_input
//   F2  init(bbox_w_px ≤ 1 px or real_size ≤ 0) → -2 invalid_class_prior
//   F3  init when full → evict oldest LIFTER_LOST; if none → -3 full
//   F4  update(unknown tracklet_id) → -1 unknown_tracklet
//   F5  update with rho_clamp_violated (ρ would leave [rho_min, rho_max])
//        → state UNCHANGED, return -2 reject_rho_clamp (Civera 2008 §V-B)
//   F6  update with S singular → state UNCHANGED, return -3 reject_S_singular
//   F7  update with var_rho non-finite post-Joseph → reject -4
//   F8  observation behind camera (δ_C.z ≤ 0) → reject -5 behind_camera
//   F9  bearing world projection NaN at init → -6 bearing_nan
//
// Execution model:
//   - Detection-task context only (single-writer).
//   - REPL/host can read via sentai_lifter_get / sentai_lifter_list
//     in MP-task context concurrently with detection-task writes;
//     individual reads are slot-snapshot (no internal locking; the
//     read tears at most one slot — caller may retry on inconsistency).
//   - No ISR access. No FreeRTOS primitives.
//
// Recovery model:
//   - All errors are LOCAL: state remains consistent on every reject.
//   - sentai_lifter_clear() is the only escalation (e.g., on mission
//     restart). Preserves lifetime counters for post-mortem.
//   - LIFTER_LOST slots are eviction candidates only — they are NOT
//     deleted automatically (debuggable post-mortem).
//
// Concurrency contract:
//   - Single writer (detection_task).
//   - Multi-reader (REPL, mission FSM) via slot-snapshot read.
//   - No CMSIS-DSP usage at this version — all matrix ops are 3×3
//     or smaller, unrolled, ≪ 1 µs each on M7.
//
// Memory:
//   - sentai_lifter_entry_t ≈ 96 B; 16 slots × 96 = 1536 B SDRAM.
//   - Zero heap, zero per-call malloc.
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Capacity ----------------------------------------------------------
// 16 lifters in flight is more than the typical 4-6 active tracklets;
// headroom matches sentai_objects (32 slots) — landmarks evicted out.
#define SENTAI_LIFTER_MAX            16
#define SENTAI_LIFTER_CLASS_MAX      80   // matches SENTAI_OBJECTS_DICT_MAX
#define SENTAI_LIFTER_RHO_MIN        0.05f   // depth ≤ 20 m
#define SENTAI_LIFTER_RHO_MAX        20.0f   // depth ≥ 5 cm
#define SENTAI_LIFTER_EPS_LIN        0.5f    // Civera 2008 linearization gate
#define SENTAI_LIFTER_ALPHA_INIT     0.5f    // σ_ρ₀ = α · ρ₀
#define SENTAI_LIFTER_OBS_PX_VAR     1.0f    // σ_obs² in pixels²

typedef enum {
    LIFTER_FREE     = 0,
    LIFTER_TRACKING = 1,   // receiving updates; σ_ρ ≥ ε·ρ² (not lifted yet)
    LIFTER_LIFTED   = 2,   // σ_ρ < ε·ρ²; ready to publish to sentai.objects
    LIFTER_LOST     = 3,   // stale: no obs > LIFTER_STALE_MS
} sentai_lifter_status_t;

// ---- Per-slot entry (~96 B) -------------------------------------------
typedef struct {
    uint8_t  status;            // sentai_lifter_status_t
    uint8_t  class_id;
    uint16_t tracklet_id;       // external ID (sentai_tracker)
    float    rho;               // inverse depth
    float    var_rho;           // σ_ρ²
    float    anchor_w[3];       // camera world pos at first obs
    float    r_w[3];            // unit bearing in world frame
    uint16_t n_obs;
    uint16_t n_rejected;
    uint32_t age_ms;            // time since first obs
    uint32_t last_obs_ms;       // ms since boot of last update
    uint32_t init_ms;           // ms since boot of init
    uint8_t  published_object_id;   // sentai.objects.id (0 = not yet)
    uint8_t  _pad[3];
} sentai_lifter_entry_t;

// ---- Stats ------------------------------------------------------------
typedef struct {
    uint32_t inits;
    uint32_t updates;
    uint32_t rejects_rho_clamp;
    uint32_t rejects_S_singular;
    uint32_t rejects_var_invalid;
    uint32_t rejects_behind_camera;
    uint32_t rejects_unknown_tracklet;
    uint32_t rejects_invalid_input;
    uint32_t lifted;            // count of LIFTER_LIFTED transitions
    uint32_t evictions;
    uint16_t hwm_used;
} sentai_lifter_stats_t;

// ---- Camera intrinsics + extrinsics setter -----------------------------
// Bootstrap values (s130 calibration): fx=577, fy=579, cx=320, cy=240,
// R_B_C={{0,1,0},{1,0,0},{0,0,-1}}, cam_offset_B={-0.04,0,-0.02}.
// Future sentai.calib will overwrite at takeoff (see objects_plan.md §21).
//
// All args copied; no lifetime issues.
// R_B_C_row_major: 9 floats. cam_offset_B: 3 floats. fx/fy/cx/cy: pixels.
int sentai_lifter_set_camera(float fx, float fy, float cx, float cy,
                             const float* R_B_C_row_major,
                             const float* cam_offset_B);

// ---- C API ------------------------------------------------------------

// Reset all slots to FREE. Preserves stats (lifetime counters).
// Returns number of slots that were non-FREE before reset.
int sentai_lifter_clear(void);

// Initialize a new landmark from bbox + class-prior pseudo-depth.
// drone_W3 = drone world position (3 floats); yaw_rad in radians.
// Returns: ≥0 slot index ; -1 invalid input ; -2 invalid class prior ;
//          -3 full ; -6 bearing NaN.
int sentai_lifter_init_from_bbox(uint16_t tracklet_id, uint8_t class_id,
                                 float u_c, float v_c,
                                 float bbox_w_px, float real_size_m,
                                 const float* drone_W3, float yaw_rad);

// EKF measurement update.
// Returns: 0 ok ; -1 unknown_tracklet ; -2 rho_clamp_rejected ;
//          -3 S_singular ; -4 var_invalid ; -5 behind_camera ; -6 invalid input.
int sentai_lifter_update_bbox(uint16_t tracklet_id,
                              float u_c, float v_c,
                              const float* drone_W3, float yaw_rad,
                              float dt_s);

// Copy slot into *out by tracklet id. Returns 0 ok, -1 unknown.
int sentai_lifter_get(uint16_t tracklet_id, sentai_lifter_entry_t* out);

// Compute landmark world position L_W = anchor + (1/ρ) · r_w.
// Caller passes 3-float buffer. Returns 0 ok, -1 unknown, -2 ρ out of bounds.
int sentai_lifter_world_pos(uint16_t tracklet_id, float* out3);

// Mark a tracklet LOST (no recent observations). Returns 0/-1.
int sentai_lifter_mark_lost(uint16_t tracklet_id);

// Snapshot compact-copy of non-FREE entries.
int sentai_lifter_list(sentai_lifter_entry_t* out, int max);

// Counts.
int sentai_lifter_count(void);

// Read stats + histogram.
void sentai_lifter_stats(sentai_lifter_stats_t* out_counters,
                         int* out_used,
                         int* out_tracking,
                         int* out_lifted,
                         int* out_lost);

#ifdef __cplusplus
}
#endif

// sentai_calib.h — ObjectsPlan OP-S6-W1 (Stage 6 sub-WP 1): camera-to-body
// rotation auto-calibration at takeoff via Kabsch 3D Procrustes.
//
// Mirrors examples/sentai_runtime/_shared/camera_calibration.py — the
// host-side Python reference (validated 2026-05-15) — and exposes the
// same API on-board through sentai.calib.*.  Kabsch SVD on N>=4 samples
// of the form (tvec_cam, marker_W, drone_W, yaw_rad) returns the rigid
// rotation R_cam_to_body that best aligns body-frame deltas to the
// camera-frame observations.
//
// Run-time profile:
//   - One-shot per takeoff (~15 ms wall, ~50 us SVD on 3x3).
//   - Cold path; lives in .sdram_text per CLAUDE.md "new code defaults
//     to SDRAM" rule.  No camera ISR access, no FreeRTOS primitives.
//   - Persists R + cam_offset_B to /system/cam_calib.json via FxUser.
//
// Math reference: §21 (split into ideas/objects_plan/11_camera_calib.md).
// Algorithm: Kabsch with H = body_centered @ cam_centered^T, then
// R = U * diag(1,1,sign(det(U*Vt))) * Vt where U, _S, Vt = SVD(H).
// 3x3 SVD uses Jacobi sweeps on H^T*H — bounded iteration, deterministic.
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A)
// =========================================================================
// Fault model:
//   F1  run_kabsch(non-finite floats in any sample)     -> -1 invalid_input
//   F2  run_kabsch(n_samples < 3)                       -> -2 too_few_samples
//   F3  run_kabsch(SVD diverges after JACOBI_MAX_SWEEPS)-> -3 svd_no_converge
//   F4  Kabsch produces |det(R)| < QUALITY_DET_THR      -> quality.accepted=0
//   F5  Kabsch produces mean_residual > QUALITY_RES_DEG -> quality.accepted=0
//   F6  drift_from_persisted > QUALITY_DRIFT_DEG        -> quality.accepted=0
//   F7  save() with non-finite R or wrong shape         -> -1 invalid_payload
//   F8  load() missing / corrupt / wrong schema         -> 0 (caller falls
//        back to identity per anti-brick rule)
//   F9  init() without persisted file                   -> defaults to
//        SENTAI_CALIB_DEFAULT_R_SIM (identity-like for SIM bring-up)
//
// Execution model:
//   - Single-writer (the mission FSM during takeoff calib phase).
//   - Multi-reader for sentai_calib_get_R_cam_to_body() — slot snapshot,
//     no internal locking (the 9-float R is updated atomically as a
//     whole via memcpy under a single mutation point in the writer).
//   - No ISR access.
//
// Recovery:
//   - All errors are LOCAL: state remains consistent on every reject.
//   - sentai_calib_clear() restores the default R (used by mission abort).
//   - Anti-brick: corrupt cam_calib.json must NEVER fail boot — load()
//     returns 0 and the cached R stays at default until next successful
//     calibration writes a new file.
//
// Memory:
//   - Static globals: 9 floats (R) + 3 floats (cam_offset_B) +
//     1 bool (is_calibrated) ≈ 49 bytes BSS.
//   - run_kabsch heap budget: caller allocates the samples array;
//     internal scratch is on stack (3 × N + 3×3 + 3×3 floats — for the
//     default N_MAX=32 this is < 600 bytes).
//   - Zero heap, zero per-call malloc.
//
// Persistence (cam_calib.json schema v1):
//   {
//     "schema": 1,
//     "R_B_C": [[r11,r12,r13],[r21,r22,r23],[r31,r32,r33]],
//     "cam_offset_B": [x, y, z]   // optional
//   }
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Constants ---------------------------------------------------------
#define SENTAI_CALIB_SAMPLES_MAX        32      // upper bound for kabsch run
#define SENTAI_CALIB_SAMPLES_MIN        3       // hard minimum (degenerate < 3)
#define SENTAI_CALIB_SCHEMA_VERSION     1
#define SENTAI_CALIB_PATH               "/system/cam_calib.json"
#define SENTAI_CALIB_JACOBI_MAX_SWEEPS  16      // 3x3 converges in ~3-5
#define SENTAI_CALIB_JACOBI_EPS         1e-9f   // off-diagonal threshold
#define SENTAI_CALIB_QUALITY_DET_THR    0.99f
// 8° is the empirical residual floor of the current DLT-PnP +
// heuristic-quad pipeline on the A4 layout (6 cm flat markers at
// drone z=0.4-0.6 m).  The 32-sample average gives drift < 1.5°
// from the true R even with per-sample residual ~7°, but the gate
// has to accommodate the per-sample noise floor.  Drop back to 3°
// once IPPE PnP + Douglas-Peucker quad upgrades land (s159 SOTA
// gaps).
#define SENTAI_CALIB_QUALITY_RES_DEG    8.0f
#define SENTAI_CALIB_QUALITY_DRIFT_DEG  10.0f

// ---- Default R_B_C for SIM bring-up (per s130 image-only nav) ---------
// Empirically validated 2026-05-14 against sentai_crazysim Gazebo SDF.
// This is the value the firmware bootstraps to when cam_calib.json does
// not yet exist.  On hardware, the takeoff calib refines this.
extern const float SENTAI_CALIB_DEFAULT_R_SIM[9];           // row-major 3x3
extern const float SENTAI_CALIB_DEFAULT_CAM_OFFSET_SIM[3];  // body-frame m

// ---- Reject codes for quality.reject_code ------------------------------
typedef enum {
    SENTAI_CALIB_OK             = 0,
    SENTAI_CALIB_REJ_DET_LOW    = 1,   // |det(R)| < QUALITY_DET_THR
    SENTAI_CALIB_REJ_REFLECTION = 2,   // det(R) < 0 (reflection, not rotation)
    SENTAI_CALIB_REJ_RES_HIGH   = 3,   // mean residual > QUALITY_RES_DEG
    SENTAI_CALIB_REJ_DRIFT_HIGH = 4,   // drift_from_persisted > QUALITY_DRIFT_DEG
    SENTAI_CALIB_REJ_TOO_FEW    = 5,   // n_samples < SENTAI_CALIB_SAMPLES_MIN
    SENTAI_CALIB_REJ_BAD_INPUT  = 6,   // non-finite sample data
    SENTAI_CALIB_REJ_SVD_NO_CV  = 7,   // Jacobi did not converge
} sentai_calib_reject_t;

// ---- One observation sample -------------------------------------------
// tvec_cam: marker pose in camera frame (from ArUco PnP).
// marker_W: marker world position (known from landing-pad layout).
// drone_W:  drone world position (from EKF / Gazebo / etc.).
// yaw_rad:  drone yaw, world frame.
typedef struct {
    float tvec_cam[3];
    float marker_W[3];
    float drone_W[3];
    float yaw_rad;
} sentai_calib_sample_t;

// ---- Quality result from a Kabsch run ---------------------------------
typedef struct {
    int   n_samples;
    float det_R;
    float mean_residual_deg;
    float max_residual_deg;
    float drift_from_persisted_deg;  // -1.0 if no persisted R provided
    int   accepted;                  // 0 / 1
    int   reject_code;               // sentai_calib_reject_t
} sentai_calib_quality_t;

// ---- API --------------------------------------------------------------

// Self-healing init.  Loads cam_calib.json (via FxUser if available;
// no-op on host SIM unless a /system path is mounted).  If absent /
// corrupt, falls back to SENTAI_CALIB_DEFAULT_R_SIM + DEFAULT_CAM_OFFSET.
// Call once during main_freertos boot (after FxUserInit).
void sentai_calib_init(void);

// Reset to the default sim R + cam_offset.  is_calibrated() returns 0
// after this until the next successful run_kabsch + commit_R.
void sentai_calib_clear(void);

// Run Kabsch 3D Procrustes on N samples; OPTIONAL persisted_R for the
// drift check.  Returns 0 on success (regardless of accepted/rejected
// quality), or a negative SENTAI_CALIB_REJ_* code on hard failure.
// R_out is always written to (identity on hard failure).
int sentai_calib_run_kabsch(const sentai_calib_sample_t* samples,
                            int n,
                            const float* persisted_R_or_null,
                            float R_out[9],
                            sentai_calib_quality_t* q_out);

// Atomically replace the cached R + cam_offset_B with the given values.
// This is the ONLY mutation point readers race against.  Validates
// shape + finiteness; returns 0 on success, -1 on invalid input.
int sentai_calib_commit_R(const float R[9], const float cam_offset_B[3]);

// Persist current cached R + cam_offset_B to /system/cam_calib.json via
// FxUser.  Returns 1 on success, 0 on failure.  Caller normally chains
// commit_R + save() after a quality.accepted Kabsch run.
int sentai_calib_save(void);

// Force a reload from cam_calib.json.  Returns 1 on success (R updated),
// 0 if the file is missing / corrupt (R stays at current cached value).
int sentai_calib_load(void);

// Public readers — slot-snapshot semantics (no internal locking).
// The 9-float R and 3-float offset are written atomically as wholes by
// commit_R; readers may observe either pre- or post-commit state but
// never a torn slot.
const float* sentai_calib_get_R_cam_to_body(void);  // 9 floats, row-major
const float* sentai_calib_get_cam_offset_B(void);   // 3 floats, body-frame m
int          sentai_calib_is_calibrated(void);      // 0/1

// Geodesic angle between two rotations (deg).  Public so tests + the
// mission FSM can use it for drift gates.
float sentai_calib_rotation_angle_deg(const float R1[9], const float R2[9]);

// =========================================================================
// OP-S10-W14 — in-flight Flow autotuner extension.
// =========================================================================
// Adds a long-running C++ task (sentai_calib_task) that drives a
// velocity-relay Åström-Hägglund auto-tune to learn the Flow loop
// gain Kp per axis.  Implementation lives in:
//   - sentai_calib_autotune.cc — state machine + relay + Ziegler-Nichols
//   - sentai_calib_task.cc      — FreeRTOS worker driving the relay
//                                 via sentai_crazy_hover, reading PnP
//                                 via sentai_aruco_get_latest, pushing
//                                 samples to sentai.fr, watching
//                                 sentai.safety.aborted.
// MP API (minimal per operator 2026-05-18 "MP doar comanda start/stop"):
//   sentai.calib.set_context(z_hold, gdx, gdy, msize)
//   sentai.calib.task_start(axis, dur_s, vmax)
//   sentai.calib.task_stop()
//   sentai.calib.is_done()  -> bool
//   sentai.calib.get_kp(axis_str) -> float (-1.0f if not converged)
// =========================================================================

// Axis selector for autotune endpoints.
typedef enum {
    SENTAI_CALIB_AXIS_X = 0,
    SENTAI_CALIB_AXIS_Y = 1,
} sentai_calib_axis_t;

// AUTOTUNE_RELAY sub-states (returned by sentai_calib_autotune_get_state).
typedef enum {
    SENTAI_CALIB_AT_IDLE       = 0,
    SENTAI_CALIB_AT_ARMING     = 1,
    SENTAI_CALIB_AT_EXCITING   = 2,
    SENTAI_CALIB_AT_SETTLING   = 3,
    SENTAI_CALIB_AT_DONE_OK    = 4,
    SENTAI_CALIB_AT_DONE_FAIL  = 5,
    SENTAI_CALIB_AT_ABORTED    = 6,
} sentai_calib_autotune_state_t;

// Persistent context (the geometry knobs the autotune task needs).
// Operator-suggested 2026-05-18 ("altitudinea la care se calibreaza ...
// pozitie markeri, dimensiuni").  Idempotent; can be called multiple
// times before task_start.  Returns 0 on success, -1 on invalid input.
int sentai_calib_set_context(float z_hold_m,
                              float marker_grid_dx_m,
                              float marker_grid_dy_m,
                              float marker_size_m);

// Read-side accessors for the learned Flow gains (-1.0f if not set).
float    sentai_calib_get_kp(sentai_calib_axis_t axis);
uint32_t sentai_calib_get_td_ms(void);

// Worker task lifecycle (full impl in sentai_calib_task.cc).
//   axis      ∈ {SENTAI_CALIB_AXIS_X, SENTAI_CALIB_AXIS_Y}
//   dur_s     hard upper bound (DONE_FAIL if not converged by then)
//   vmax_m_s  relay velocity magnitude (default 0.10 m/s)
// Returns 0 / negative.  Idempotent (already-running → 0).
int sentai_calib_task_start(sentai_calib_axis_t axis,
                             float dur_s,
                             float vmax_m_s);
int sentai_calib_task_stop(void);
int sentai_calib_task_is_done(void);          // 0/1

// OP-S10-W14-T12 — VALIDATION mode: use identified Kp_x, Kp_y in a
// closed-loop P-controller hold for dur_s seconds.  At each camera
// tick:  v_cmd_x = -kp_x * drift_x,  v_cmd_y = -kp_y * drift_y,
// hover(v_cmd_x, v_cmd_y, 0, z_hold).  Measures max + RMS drift
// during the hold for post-mortem (logged via sentai.fr).  Use to
// confirm the autotune-identified gains actually hold position when
// both axes are active simultaneously (vs the relay autotune which
// is single-axis).
int sentai_calib_hold_start(float kp_x, float kp_y,
                              float vmax_clip, float dur_s);
float sentai_calib_get_hold_max_drift_m(void);
float sentai_calib_get_hold_rms_drift_m(void);

#ifdef __cplusplus
}
#endif

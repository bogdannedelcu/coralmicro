// sentai_aruco.h — ObjectsPlan OP-S6-W3 (Stage 6 sub-WP 3): on-board
// ArUco fiducial-marker detector + PnP pose, in C/C++ per CLAUDE.md
// "compute-in-C, MP-as-glue" rule.
//
// Returns SMALL SCALARS to MicroPython (marker_id + 3-vector pose +
// corner pixels) — NEVER an image buffer crossing the MP boundary.
// Air-gap clean: the only input is the grayscale frame coming from
// the camera (or camera_bridge_recv on SIM); no Gazebo ground truth.
//
// Use cases unblocked by this module:
//   - OP-S6-W1-T7 / EXP-s158: feed tvec_cam into sentai.calib at
//     takeoff (camera-to-body Kabsch).
//   - Future re-build of an ArUco-VPE forwarder ("flow.aruco mode"
//     — replacing the legacy "flow.mode('anchor')" which conflated
//     phase-correlation anchor with ArUco anchor).
//   - L4.5 image-only nav (s130) and L5 lifter (s132) host-Python
//     ArUco dependencies, when those missions migrate to MP-only
//     per [[missions-run-in-sentai-only]].
//
// Reference (DESIGN-INSPIRATION ONLY): old `sentai_aruco_shim.h` from
// branch `feature/ov5640-camera-support` (commit a5f5a568).  Per
// [[no-broken-branch-test-reuse]], implementation is FRESH — the old
// branch hit a dead-end and was reverted.  API differs deliberately:
//   - Camera-frame pose (tvec/rvec) not world-frame (x,y,z,yaw)
//   - Multi-marker output, not just "the anchor"
//   - Same `.cc` compiles for both ARM and SIM (no shim-arm /
//     shim-sim split); platform features detected via CMake.
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A)
// =========================================================================
// Fault model:
//   F1  detect(null gray | w*h == 0)               -> -1 invalid_input
//   F2  detect before init() / no intrinsics set   -> -2 no_intrinsics
//   F3  contour finder overflow (> contours cap)   -> drop tail, count
//                                                       in overflow_total
//   F4  quadrilateral approx fails for all blobs   -> 0 markers, OK
//   F5  dictionary mismatch (hamming > MAX_HAMM)   -> rejected marker
//   F6  PnP residual > REPROJ_MAX_PX               -> rejected marker
//   F7  size_m <= 0 in pose_estimate               -> -3 invalid_marker_size
//   F8  out_capacity == 0                          -> 0 (caller wants count
//                                                       discovery via stats)
//
// Execution model:
//   - Single-writer (detection_task / camera_bridge_recv task).
//   - Multi-reader for sentai_aruco_get_latest — slot-snapshot, no
//     locking (the array is updated atomically per marker by memcpy).
//   - No ISR access.  No FreeRTOS primitives directly — module is
//     pure compute + memcpy; caller chooses scheduling.
//
// Recovery:
//   - All errors are LOCAL.
//   - sentai_aruco_clear() resets caches + counters (mission restart).
//   - Per-marker rejection does not abort the whole frame.
//
// Memory:
//   - Static globals: cache of last detection (~ 16 * sizeof(marker_t)
//     ≈ 1.5 KB), preprocessing buffers (binary + edge ~ 2 * 320*240 = 150 KB
//     in .sdram_bss).  All cold-path SDRAM.
//   - Zero heap, zero per-call malloc.
//
// =========================================================================

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Capacity ---------------------------------------------------------
// Up to 16 markers per frame.  ArUco MVP scenario is 1-6 markers; 16
// is a generous headroom for the indoor 5x5m room demo (§23.1).
#define SENTAI_ARUCO_MAX_MARKERS    16

// 4x4_50 dictionary — the only one we ship for thesis MVP.  Adding
// 5x5_100 / 6x6_250 would be a future WP.
#define SENTAI_ARUCO_DICT_SIZE      50
#define SENTAI_ARUCO_BITS_PER_DIM   4
#define SENTAI_ARUCO_BIT_COUNT      (SENTAI_ARUCO_BITS_PER_DIM * \
                                     SENTAI_ARUCO_BITS_PER_DIM)

// Quality / acceptance thresholds (§21.5-style).
#define SENTAI_ARUCO_MIN_QUAD_AREA  256.0f      // px², 16x16 minimum
#define SENTAI_ARUCO_MIN_PERIMETER  40.0f       // px
#define SENTAI_ARUCO_MAX_HAMMING    2           // bits flipped allowed
#define SENTAI_ARUCO_REPROJ_MAX_PX  3.0f        // PnP residual gate

// ---- Reject codes ------------------------------------------------------
typedef enum {
    SENTAI_ARUCO_OK             = 0,
    SENTAI_ARUCO_ERR_BAD_INPUT  = 1,
    SENTAI_ARUCO_ERR_NO_INTR    = 2,    // intrinsics not set
    SENTAI_ARUCO_ERR_OVERFLOW   = 3,    // contour cap exceeded
    SENTAI_ARUCO_ERR_NO_DICT    = 4,    // marker not in dictionary
    SENTAI_ARUCO_ERR_REPROJ     = 5,    // PnP residual too high
    SENTAI_ARUCO_ERR_BAD_MSIZE  = 6,    // marker_size_m <= 0
} sentai_aruco_status_t;

// ---- Per-marker output (~ 96 B) ---------------------------------------
// All quantities are CAMERA-FRAME by convention (rvec/tvec relative to
// the camera optical center).  World-frame is the caller's job to
// compose using sentai.calib's R_cam_to_body + drone pose telemetry.
typedef struct {
    uint8_t  marker_id;            // 0 .. SENTAI_ARUCO_DICT_SIZE-1
    uint8_t  hamming;              // bits flipped vs dict entry
    uint16_t _pad;
    float    tvec_cam[3];          // marker centre, camera frame (m)
    float    rvec_cam[3];          // Rodrigues rotation, camera frame
    float    corners_px[8];        // 4 corners: u0,v0,u1,v1,u2,v2,u3,v3
    float    reproj_err_px;        // mean per-corner reprojection
    uint32_t detect_us;            // last-frame detection cost (info)
    uint32_t src_ts_ms;            // monotonic ms when frame saw light
    uint32_t frame_seq;            // monotonic frame counter
} sentai_aruco_marker_t;

// ---- Stats (monitoring; published by get_stats) -----------------------
typedef struct {
    uint32_t frames_total;
    uint32_t frames_with_detect;
    uint32_t markers_total;            // sum of detected across frames
    uint32_t overflow_total;           // contour-cap exceeded
    uint32_t rejected_dict_total;      // marker not in dictionary
    uint32_t rejected_reproj_total;    // PnP residual too high
    uint32_t last_detect_us;
} sentai_aruco_stats_t;

// =======================================================================
// API
// =======================================================================

// One-time init.  Idempotent.  Pulls in compile-time defaults for
// intrinsics + marker size; caller can refine via _set_intrinsics +
// _set_marker_size before the first _detect call.  Returns 0 on
// success, negative on failure (always proceeds — a failed init
// leaves the module in a state where _detect returns no markers).
int sentai_aruco_init(void);

// Reset cache + stats.  Called on mission restart.
void sentai_aruco_clear(void);

// Set the pinhole intrinsics (fx, fy, cx, cy) in pixels.  Persists
// until next _set_intrinsics call.  Required before first PnP solve.
void sentai_aruco_set_intrinsics(float fx, float fy, float cx, float cy);

// Set the physical marker side length in metres.  Used by PnP to
// scale tvec.  Default = 0.0625 m (the s091 / s130 marker size; see
// [[pnp-bugs-fixed]] for the 8 cm vs 6.25 cm bug we already shipped).
int sentai_aruco_set_marker_size(float size_m);

// Run detection on a fresh grayscale frame.  `gray` is w*h Y8 pixels
// (row-major).  `out` is a caller-provided array of capacity
// `out_capacity`.  Returns the number of markers written (0 .. capacity),
// or a negative SENTAI_ARUCO_ERR_* on hard failure.
//
// `frame_seq` and `src_ts_ms` are stamped onto every output entry —
// caller passes the value associated with this frame from the camera
// pipeline (or 0 / xTaskGetTickCount if irrelevant).
int sentai_aruco_detect(const uint8_t* gray, int w, int h,
                        uint32_t frame_seq, uint32_t src_ts_ms,
                        sentai_aruco_marker_t* out, int out_capacity);

// Read the cached last-detection result without re-running.  Returns
// the number of markers in the cache (0 .. capacity).
int sentai_aruco_get_latest(sentai_aruco_marker_t* out, int out_capacity);

// OP-S10-W14 / s175 — load a P5 320x240 PGM file from disk and run
// detection on it (no camera involved).  Per-marker results printed
// to stderr.  Returns n_dets or negative error.  Used to test PnP
// rotation invariance — feed the SAME scene at different image
// rotations and see if tvec_cam[2] (Z) stays constant.
int sentai_aruco_detect_pgm_file(const char* path);

// Read the monitoring counters.
void sentai_aruco_get_stats(sentai_aruco_stats_t* out);

// Convenience helpers exported for tests + diagnostics.

// Convert a Rodrigues rvec (3) to a row-major 3x3 rotation matrix.
void sentai_aruco_rvec_to_R(const float rvec[3], float R_out[9]);

// Convert a row-major 3x3 rotation matrix to Rodrigues rvec.
void sentai_aruco_R_to_rvec(const float R[9], float rvec_out[3]);

#ifdef __cplusplus
}
#endif

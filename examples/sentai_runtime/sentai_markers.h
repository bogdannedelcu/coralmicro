// sentai_markers.h — unified pose-emitting fiducial-marker API.
//
// OP-S10-W19: hard-rename of the parallel sentai.aruco + sentai.whycon
// namespaces into a single sentai.markers namespace with a backend
// parameter at init.  All consumers (SafetyTask, calib autotune,
// missions, FlowBaseline gate) consume the unified API; backend
// choice is runtime-configurable.  NO compatibility shim — once T1
// lands, sentai_aruco.h / sentai_whycon.h disappear.  See
// `ideas/objects_plan/OP-S10-W19_markers_unified.md` for the WP doc
// and `memory/project_op_s10_w19_markers_plan.md` for the operator-
// stated constraints (NO dicts crossing the MP heap, structs only;
// NASA embedded discipline).
//
// File state: T1 scaffolding — this header is on disk, the .cc
// implementation, MP binding, and call-site rewrites land in a
// follow-up session.  Marked stable from here; downstream consumers
// can include this header without risk of ABI churn.

#ifndef EXAMPLES_SENTAI_RUNTIME_SENTAI_MARKERS_H_
#define EXAMPLES_SENTAI_RUNTIME_SENTAI_MARKERS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// Backend identifier (runtime).
// =====================================================================

typedef enum {
    SENTAI_MARKERS_BACKEND_NONE   = 0,  // not initialised
    SENTAI_MARKERS_BACKEND_ARUCO  = 1,  // ArUco 4x4 dict + IPPE_SQUARE PnP
    SENTAI_MARKERS_BACKEND_WHYCON = 2,  // Krajník/Nitsche annular markers
                                        // + closed-form PnP-z
} sentai_markers_backend_t;

// =====================================================================
// Public structs — caller-allocated; bindings pass a bytearray of the
// correct size and the C side writes it directly.  No MP dicts.
//
// ABI is LOCKED — any change here needs a parallel update in
// modsentai_markers.c's mirror struct, with a static_assert on size.
// Layout matches alignment-safe ordering (4-byte fields first, then
// 1-byte trailers + explicit padding).
// =====================================================================

typedef struct {
    int32_t  id;             // dictionary ID (ArUco) or sequence index
                             // (WhyCon — set to -1 by single-marker
                             // backends, multi-marker constellation
                             // backend assigns 0..N-1)
    float    pixel_cx;       // centroid x (sub-pixel)
    float    pixel_cy;       // centroid y (sub-pixel)
    float    tvec_cam[3];    // (x, y, z) in camera frame, metres
                             // NaN if !pose_valid
    float    rvec_cam[3];    // Rodrigues rotation vector, radians
                             // For WhyCon single-marker: yaw around
                             // marker normal is INDETERMINATE; the
                             // returned rvec encodes tilt magnitude
                             // + tilt direction only (W17 §6.2).
                             // NaN if !pose_valid.
    float    reproj_err_px;  // px residual (ArUco) or 0 (WhyCon
                             // closed-form PnP — self-consistent
                             // by construction)
    uint8_t  backend;        // sentai_markers_backend_t enum value
    uint8_t  pose_valid;     // 1 iff tvec/rvec/reproj populated
    uint16_t _pad;
    uint32_t _pad32;         // rounds size to 48 B for cache-line nice-ness
                             // + leaves room for a future per-marker field
                             // (e.g. detect_us)
} SentaiMarkersPose;

typedef struct {
    uint32_t frames_total;
    uint32_t frames_with_detect;
    uint32_t markers_total;
    uint32_t last_detect_us;
    uint8_t  backend;
    uint8_t  _pad[3];
} SentaiMarkersStats;

#ifdef __cplusplus
// Compile-time ABI guards.  When this changes, the MP binding's
// mirror typedefs MUST be updated in lockstep.
static_assert(sizeof(SentaiMarkersPose)  == 48,
              "SentaiMarkersPose ABI broken — update modsentai_markers.c too");
static_assert(sizeof(SentaiMarkersStats) == 20,
              "SentaiMarkersStats ABI broken — update modsentai_markers.c too");
#endif

// =====================================================================
// Lifecycle.
// =====================================================================

// Initialise with a chosen backend.  Returns 0 on success, negative
// on failure (e.g., unsupported backend, prior init still active —
// caller must clear() first).
int  sentai_markers_init(sentai_markers_backend_t backend);

// Free per-backend state and reset to BACKEND_NONE.
void sentai_markers_clear(void);

// Query the active backend.
sentai_markers_backend_t sentai_markers_get_backend(void);

// =====================================================================
// Configuration (must be called between init and detect_from_camera).
// =====================================================================

// Camera intrinsics (shared by both backends).
void sentai_markers_set_intrinsics(float fx, float fy, float cx, float cy);

// Physical marker size (metres).  For ArUco: side length of the square.
// For WhyCon: diameter of the outer dark ring.  Both backends require
// this for PnP to populate tvec_cam.
void sentai_markers_set_marker_size(float meters);

// =====================================================================
// Detection.
// =====================================================================

// Take the latest camera frame (zero-copy through the PrepTask SLOT
// pipeline) and run the active backend's detector.  Returns
// n_markers_detected, or negative on hard error.
//
// Internally calls sentai_aruco_detect_from_camera() or
// sentai_whycon_detect_from_camera() depending on the backend.
int  sentai_markers_detect_from_camera(void);

// Count of markers from the most recent detect call.
int  sentai_markers_get_count(void);

// Copy the i-th marker's pose into the caller-provided buffer.
// Buffer MUST be at least sizeof(SentaiMarkersPose) bytes.
// Returns 1 on success, 0 if i is out of range.
int  sentai_markers_get_pose(int i, SentaiMarkersPose* out);

// Copy the cumulative stats into the caller-provided buffer.
// Returns 1 on success.
int  sentai_markers_get_stats(SentaiMarkersStats* out);

// Convenience for cross-frame consumers (SafetyTask reuse pattern):
// returns the latest pose snapshot directly, without re-running
// detection.  Same buffer convention as get_pose.
int  sentai_markers_get_latest(int i, SentaiMarkersPose* out);

// =====================================================================
// Diagnostics — used by W17 perf-instrumentation paths.
// =====================================================================

// Total cycles elapsed during the most recent detect call.
uint32_t sentai_markers_detect_cyc_last(void);

// =====================================================================
// Naming policy reminder.
// =====================================================================
//
// 1. NO BACKEND-AGNOSTIC ALIASES of the old sentai_aruco_* /
//    sentai_whycon_* function names.  Call sites refactor to the
//    sentai_markers_* names directly; old names disappear.
//
// 2. Backend-specific diagnostics (e.g., WhyCon _stage_cyc5, ArUco
//    _stage_cyc, ArUco rvec_to_R / R_to_rvec, WhyCon _test_synth*)
//    stay in their respective `*.cc` files but are reachable only
//    via the binding's `sentai.markers.<backend>.<diag>` sub-table.
//
// 3. Bindings pass caller-allocated bytearrays.  NO MP dicts cross
//    the heap.  See `feedback_no_heavy_data_through_mp.md` for the
//    hard-rule.

#ifdef __cplusplus
}
#endif

#endif  // EXAMPLES_SENTAI_RUNTIME_SENTAI_MARKERS_H_

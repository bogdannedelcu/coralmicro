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

typedef struct {
    int32_t  id;             // dictionary ID (ArUco) or sequence index (WhyCon)
    float    pixel_cx;       // centroid x (sub-pixel)
    float    pixel_cy;       // centroid y (sub-pixel)
    float    axis_a;         // WhyCon fitted semi-major axis in pixels
                             // (0 for backends that do not expose it)
    float    axis_b;         // WhyCon fitted semi-minor axis in pixels
    float    angle_rad;      // WhyCon major-axis orientation in radians
    int32_t  comp_id;        // backend component id, or -1 if unavailable
    float    tvec_cam[3];    // pose payload, same convention as
                             // SentaiMarkersPose
    float    rvec_cam[3];
    float    reproj_err_px;
    uint8_t  backend;        // sentai_markers_backend_t enum value
    uint8_t  pose_valid;     // 1 iff tvec/rvec/reproj populated
    uint8_t  geometry_valid; // 1 iff axis_a/axis_b/angle are populated
    uint8_t  _pad;
    float    radius_outer;   // WhyCon bbox-derived outer radius in pixels
                             // (0 for backends that do not expose it)
} SentaiMarkersDetection;

typedef struct {
    int32_t  n_raw;          // raw detections from most recent detect call
    int32_t  n_full;         // detections fully inside image bounds
    int32_t  n_pose_valid;   // full-visible detections with valid positive Z
    float    centroid_x;     // mean full-visible centroid x, NaN if n_full == 0
    float    centroid_y;     // mean full-visible centroid y, NaN if n_full == 0
    float    radius_mean_px; // mean full-visible outer radius
    float    z_cam_mean_m;   // mean valid positive Z over full-visible markers
    float    bbox_min_x;     // full-visible marker envelope, NaN if n_full == 0
    float    bbox_min_y;
    float    bbox_max_x;
    float    bbox_max_y;
    uint32_t frame_seq;      // source frame sequence from last detect_frame
    uint32_t src_ts_ms;      // source timestamp from last detect_frame
    uint8_t  valid;          // 1 when observation is based on a detect call
    uint8_t  backend;        // sentai_markers_backend_t enum value
    uint16_t _pad;
} SentaiMarkersObservation;

typedef struct {
    int32_t count;           // samples currently stored in the rolling window
    int32_t size;            // configured window size
    int32_t min_full;        // minimum observed n_full in the current fill
    float   avg_full;        // mean n_full over the current fill
    uint8_t ready;           // count >= size
    uint8_t _pad[3];
} SentaiMarkersWindowStats;

// =====================================================================
// W19-T6b: drone-pose recovery from multi-marker Kabsch + yaw-anchor.
//
// Sits on top of sentai_kabsch_align (libs/.../sentai_svd3.h).  The
// mission registers known world-frame marker positions once via
// set_marker_world, then per-frame calls get_drone_pose(cf2_yaw) to
// recover the drone's pose in world frame.  The picker tries all P(N,K)
// permutations to find the best assignment of K observed markers to N
// registered ones, applies a Z-plane reflection if the unconstrained
// Kabsch put the drone below the marker plane, then applies the yaw-
// anchored sign-of-diagonal X/Y flip from memory entry
// `feedback_yaw_anchor_mirror_picker.md` to disambiguate the N-fold
// rotational symmetry that symmetric pads exhibit.
//
// Tvec source: post-T5 cam extrinsics are applied to s_cache[*].tvec_cam
// at the marker-detection step, so the cached tvecs are already in
// BODY frame.  Kabsch fits BODY -> WORLD, t is drone position in world.
// =====================================================================
typedef struct {
    float    x, y, z;       // drone position in world frame (m), NaN if invalid
    float    yaw_rad;       // recovered yaw (atan2(R[3], R[0])), radians
    float    res_max;       // worst per-marker residual (m)
    int32_t  n_used;        // markers fused (>= 3 for valid fit, 0 otherwise)
    uint8_t  flip_x;        // 1 if yaw-anchor disambiguator flipped X
    uint8_t  flip_y;        // 1 if it flipped Y
    uint8_t  flip_z;        // 1 if Z-plane reflection kicked in
    uint8_t  _pad;
} SentaiMarkersDronePose;

#ifdef __cplusplus
// Compile-time ABI guards.  When this changes, the MP binding's
// mirror typedefs MUST be updated in lockstep.
static_assert(sizeof(SentaiMarkersPose)      == 48,
              "SentaiMarkersPose ABI broken -- update modsentai_markers.c too");
static_assert(sizeof(SentaiMarkersStats)     == 20,
              "SentaiMarkersStats ABI broken -- update modsentai_markers.c too");
static_assert(sizeof(SentaiMarkersDetection) == 64,
              "SentaiMarkersDetection ABI broken -- update modsentai_markers.c too");
static_assert(sizeof(SentaiMarkersDronePose) == 28,
              "SentaiMarkersDronePose ABI broken -- update modsentai_markers.c too");
static_assert(sizeof(SentaiMarkersObservation) == 56,
              "SentaiMarkersObservation ABI broken -- update modsentai_markers.c too");
static_assert(sizeof(SentaiMarkersWindowStats) == 20,
              "SentaiMarkersWindowStats ABI broken -- update modsentai_markers.c too");
#endif

// Upper bound for registered world markers (P(N,K) permutation search
// at N=8: 8! = 40320, ~2 s on M7 - so 8 is the practical ceiling here).
#define SENTAI_MARKERS_MAX_WORLD  8

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
// Camera extrinsics — OPTIONAL.
//
// Configures where the camera is mounted on the drone body.  When
// set, the library transforms every detected marker's tvec from the
// camera OPTICAL frame into the BODY frame, so all downstream
// consumers (VPE forwarder, mission Kabsch, autotune) get marker
// positions relative to the drone's centre of mass instead of the
// camera optical centre.
//
//   tvec_body = R_opt_to_body · tvec_cam_optical  +  (tx, ty, tz)
//
// where R_opt_to_body is computed from:
//   - R_opt_to_link (fixed, ROS REP 103 convention: image-right ↔
//     -link-Y, image-down ↔ -link-Z, depth ↔ +link-X)
//   - R_link_to_body from the SDF <pose> RPY angles (extrinsic XYZ
//     order = R_z(yaw) · R_y(pitch) · R_x(roll))
//
// (tx, ty, tz) = sensor link origin in body frame, from SDF <pose>
// position component.
//
// DEFAULT (before this is called): identity transform — tvec stays
// in camera optical frame, backward-compatible with consumers that
// expect raw cam-optical tvec (sentai_calib's R_cam_to_body solver).
//
// For cf2 SIM downward camera, the SDF says:
//   <pose>-0.04 0 -0.02  0 1.5707963 3.1415927</pose>
//   → set_cam_extrinsics(-0.04, 0, -0.02, 0, M_PI/2, M_PI)
//
// Operator request 2026-05-20: "algoritmul nostru de detectie
// markeri ar trebui sa ii detecteze relativ la centrul de masa al
// dronei".
void sentai_markers_set_cam_extrinsics(float tx, float ty, float tz,
                                          float roll, float pitch, float yaw);

// Direct calibrated extrinsics setter.  This is the runtime path after
// sentai.calib.load(): R_opt_to_body is the persisted body<-camera-optical
// rotation, and (tx,ty,tz) is the camera origin in body frame.  It avoids
// reconstructing an SDF RPY convention when the on-flight calibration has
// already discovered the true axis/sign mapping.
void sentai_markers_set_cam_extrinsics_matrix(float tx, float ty, float tz,
                                               const float R_opt_to_body[9]);

// Return the currently configured camera extrinsics used by sentai.markers.
// `R_opt_to_body_out` is row-major body<-camera-optical.  `is_set_out` is 0
// when the default identity passthrough is active.
void sentai_markers_get_cam_extrinsics_matrix(float t_body_out[3],
                                               float R_opt_to_body_out[9],
                                               int* is_set_out);

// Reset extrinsics to identity (default state).  After this call, tvec
// is in cam optical frame again.
void sentai_markers_clear_cam_extrinsics(void);

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

// Run detection on a caller-supplied grayscale frame.  Used by
// SafetyTask (which already has the frame in hand from
// sentai_camera_grab_gray_zerocopy) to avoid the extra hop through
// the binding's camera grab.  Returns n_detected or negative on
// hard error.  Backend must already be initialised.
int  sentai_markers_detect_frame(const uint8_t* gray, int w, int h,
                                    uint32_t frame_seq,
                                    uint32_t src_ts_ms);

// Run detection on a canonical grayscale P5 PGM file path.  Used by SIM
// REPL/dataset tests to keep image parsing inside the marker subsystem.
// Returns n_detected or negative on hard error.  Backend must already be
// initialised.
int  sentai_markers_detect_pgm(const char* path);

// Count of markers from the most recent detect call.
int  sentai_markers_get_count(void);

// Copy the i-th marker's pose into the caller-provided buffer.
// Buffer MUST be at least sizeof(SentaiMarkersPose) bytes.
// Returns 1 on success, 0 if i is out of range.
int  sentai_markers_get_pose(int i, SentaiMarkersPose* out);

// Copy the i-th detection geometry into the caller-provided buffer.
// Buffer MUST be at least sizeof(SentaiMarkersDetection) bytes.
// Returns 1 on success, 0 if i is out of range.
int  sentai_markers_get_detection(int i, SentaiMarkersDetection* out);

// Copy the cumulative stats into the caller-provided buffer.
// Returns 1 on success.
int  sentai_markers_get_stats(SentaiMarkersStats* out);

// Convenience for cross-frame consumers (SafetyTask reuse pattern):
// returns the latest pose snapshot directly, without re-running
// detection.  Same buffer convention as get_pose.
int  sentai_markers_get_latest(int i, SentaiMarkersPose* out);

// Aggregate observation over the most recent detection cache.  This is the C++
// equivalent of the B3/B4 MP `_detect_features()` helper: it computes
// full-visible count, centroid, radius mean, positive-Z mean, and image
// envelope using caller-provided image bounds and margin.  Returns 1 on
// success, 0 on invalid arguments.  It does NOT run detection.
int  sentai_markers_get_observation(int img_w, int img_h, float margin_px,
                                    SentaiMarkersObservation* out);

// Small fixed rolling windows for marker-count policies.  These are generic
// marker-observation utilities, not calibration logic.  C++ tasks use them to
// avoid MP-side arrays/sums in camera-rate loops.
int  sentai_markers_window_reset(int slot, int size);
int  sentai_markers_window_push(int slot,
                                int n_full,
                                SentaiMarkersWindowStats* out);
int  sentai_markers_window_get(int slot,
                               SentaiMarkersWindowStats* out);

// =====================================================================
// W19-T6b: drone-pose recovery (see SentaiMarkersDronePose).
// =====================================================================

// Register N marker world-frame positions.  xyz_n3 is a flat
// N*3-floats array (row-major: marker i at xyz_n3[3*i + {0,1,2}]).
// Returns 0 on success, -1 if n is out of range or xyz_n3 is NULL.
// Calling with n=0 clears the registry.
int  sentai_markers_set_marker_world(int n, const float* xyz_n3);

// Number of currently-registered world markers.
int  sentai_markers_get_marker_world_count(void);

// Per-frame drone pose recovery.  Runs Kabsch with permutation
// assignment search on the cached observations (body-frame tvecs)
// vs the registered world markers, applies Z-plane reflection
// (coplanar-pad safety) and the yaw-anchored X/Y mirror flip.
// cf2_yaw_rad is the body-frame yaw from cf2's EKF (CRTP LOG); it
// MUST be independent of the marker observations themselves.
// Returns 1 on success, 0 if n_obs < 3 or no world markers
// registered (out is left untouched on failure).
int  sentai_markers_get_drone_pose(float cf2_yaw_rad,
                                   SentaiMarkersDronePose* out);

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

// sentai_tracker.h — SentAI-SORT: Lightweight multi-object tracker
//
// Adapted from BoT-SORT for Cortex-M7 @996MHz + EdgeTPU:
//   1. Kalman filter 8-state with block-diagonal covariance
//   2. ByteTrack two-stage association with greedy matching
//   3. Color histogram (HSV 4×4×8) with Bhattacharyya distance
//   4. IMU-based Camera Motion Compensation (replaces ORB+homography)
//
// Called from InferTask in detection_task.cc after NMS.
// All state is static — zero heap allocation.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Limits ----
#define TRACKER_MAX_TRACKS    32
#define TRACKER_HIST_BINS      8    // hue bins per spatial cell
#define TRACKER_HIST_GRID      4    // NxN spatial grid
#define TRACKER_HIST_SIZE    (TRACKER_HIST_BINS * TRACKER_HIST_GRID * TRACKER_HIST_GRID)  // 128

// ---- Track states (stored as uint8_t) ----
#define TRACK_TENTATIVE  0   // new, awaiting min_hits confirmations
#define TRACK_CONFIRMED  1   // stable, actively tracked
#define TRACK_LOST       2   // not matched recently, coasting on prediction

// ---- Track event types ----
#define TRACK_EVT_NEW      1   // track promoted to confirmed
#define TRACK_EVT_UPDATE   2   // confirmed track updated with new detection
#define TRACK_EVT_LOST     3   // confirmed → lost (started coasting)
#define TRACK_EVT_REMOVED  4   // track removed (too many misses)

// Forward declaration (full definition in detection_task.h)
struct Detection;

// ---- Public snapshot of a tracked object ----
typedef struct {
    uint16_t   id;              // unique track ID (1-65535, 0 = invalid)
    uint8_t    state;           // TRACK_TENTATIVE / CONFIRMED / LOST
    uint8_t    hits;            // consecutive successful matches
    uint8_t    misses;          // consecutive frames without match
    uint8_t    total_hits;      // total matches since creation
    int16_t    class_id;        // most recent detection class
    int16_t    x1, y1, x2, y2; // predicted bbox (model-input pixel space)
    int16_t    conf_permil;     // last matched confidence ×1000
    uint32_t   age_frames;      // frames since creation
    // Ground-plane projection (only valid when altitude > 0 via set_pose)
    int32_t    gx_cm;           // ground X: East(+) / West(-) if heading set, else camera-right
    int32_t    gy_cm;           // ground Y: North(+) / South(-) if heading set, else camera-forward
    int32_t    dist_cm;         // distance from nadir (directly below camera), always >=0
    int16_t    width_cm;        // estimated target width on ground, 0 if no pose
    // Absolute GPS (only valid when camera GPS set via set_pose)
    float      lat;             // target latitude (degrees), 0.0 if no GPS
    float      lon;             // target longitude (degrees), 0.0 if no GPS
} TrackedObject;

// ---- Track event (pushed to event queue for Python consumption) ----
typedef struct {
    uint8_t    type;            // TRACK_EVT_*
    uint8_t    _pad;
    uint16_t   id;              // track ID
    int16_t    class_id;
    int16_t    x1, y1, x2, y2; // bbox at event time
    int16_t    conf_permil;
    uint32_t   frame_seq;       // camera frame sequence
    int32_t    gx_cm;           // ground X (cm), 0 if no pose set
    int32_t    gy_cm;           // ground Y (cm), 0 if no pose set
    int32_t    dist_cm;         // distance from nadir (cm), 0 if no pose set
    float      lat;             // target latitude (degrees), 0.0 if no GPS
    float      lon;             // target longitude (degrees), 0.0 if no GPS
} TrackEvent;

// ---- Tracker configuration ----
typedef struct {
    uint8_t  min_hits;          // frames before tentative→confirmed (default 3)
    uint8_t  max_misses;        // frames before lost→removed (default 5)
    uint8_t  max_lost;          // frames before confirmed→lost (default 1)
    uint8_t  enable_histogram;  // 1 to enable color histogram matching (default 1)
    uint8_t  enable_imu_cmc;    // 1 to enable IMU camera motion compensation (default 1)
    uint8_t  _pad[3];
    int16_t  iou_thresh;        // ×1000, minimum IoU for match (default 300 = 0.3)
    int16_t  hist_weight;       // ×1000, histogram cost weight 0-1000 (default 300 = 0.3)
    int16_t  high_thresh;       // ×1000, high-conf threshold for stage 1 (default 500)
    int16_t  low_thresh;        // ×1000, low-conf threshold for stage 2 (default 100)
} TrackerConfig;

// ---- Maximum cameras ----
#define TRACKER_MAX_CAMERAS  2

// ---- Per-camera configuration (mount geometry relative to board) ----
// Convention: mount angles describe the camera's optical axis relative to
// the board's coordinate frame (board Z = gravity vector when level).
//   mount_pitch = 0  → camera looks straight DOWN (perpendicular to ground)
//   mount_pitch = 25 → camera tilted 25° from vertical (looking forward)
//   mount_roll  = 0  → landscape, sensor width = left/right
//   mount_roll  = 90 → portrait, sensor rotated 90° CW
//   mount_yaw   = 0  → camera forward = board forward
//   mount_yaw   = 180→ camera facing backward
//
// ground_ref controls which bbox anchor is projected to ground:
//   0 = CENTROID  — average of all 4 projected corners (default, best for overhead/drone)
//   1 = BOTTOM    — midpoint of bottom edge only (best for pole-mounted angled cameras,
//                   where the bottom of the bbox approximates ground contact)
typedef struct {
    float   fov_h_deg;       // horizontal FOV in degrees (default 70.8, OV5640 stock)
    float   fov_v_deg;       // vertical FOV in degrees (default 43.4)
    float   mount_pitch_deg; // tilt from vertical: 0=down, 90=horizon (default 0)
    float   mount_roll_deg;  // sensor roll: 0=landscape, 90=portrait CW (default 0)
    float   mount_yaw_deg;   // yaw offset from board forward (default 0)
    int     ground_ref;      // 0=centroid (drone), 1=bottom-center (pole mount)
} CameraConfig;

// ---- Public API ----

// Initialize tracker primitives (lazy, safe to call multiple times).
void sentai_tracker_init(void);

// Reset all tracks and events (call on pipeline restart).
void sentai_tracker_reset(void);

// Main update: process detections, advance tracker state.
//   dets / n_dets  : detection array from NMS
//   tensor_buf     : model input tensor (int8), used for histogram extraction
//   tw, th, tch    : tensor width, height, channels
//   zp             : tensor zero point (typically -128)
//   frame_seq      : monotonic camera frame sequence number
// Returns number of confirmed tracks.
int  sentai_tracker_update(const struct Detection* dets, int n_dets,
                           const uint8_t* tensor_buf, int tw, int th, int tch, int zp,
                           uint32_t frame_seq);

// Snapshot of active tracks (confirmed + lost). Copies up to max into out[].
// Returns number of tracks copied.
int  sentai_tracker_get_tracks(TrackedObject* out, int max);

// Get next track event. Blocks up to timeout_ms.
// Returns 1 if event filled, 0 on timeout.
int  sentai_tracker_get_event(TrackEvent* event, int timeout_ms);

// Number of confirmed tracks.
int  sentai_tracker_num_active(void);

// Configuration get/set.
void sentai_tracker_set_config(const TrackerConfig* cfg);
void sentai_tracker_get_config(TrackerConfig* cfg);

// IMU input for camera motion compensation.
// Call with current pitch/roll before sentai_tracker_update() each frame.
void sentai_tracker_set_imu(float pitch_deg, float roll_deg);

// Configure camera mount geometry. cam_id: 0 or 1.
// Must be called before pipeline start to set FOV and mount orientation.
void sentai_tracker_set_camera(int cam_id, const CameraConfig* cfg);
void sentai_tracker_get_camera(int cam_id, CameraConfig* cfg);

// Set active camera (selects which CameraConfig is used for projection).
// Should match sentai.camera.switch(id).
void sentai_tracker_set_active_camera(int cam_id);
int  sentai_tracker_get_active_camera(void);

// Set sensor pose for ground-plane projection.
//   altitude_cm: camera height above ground in centimeters (0 = disable projection)
//   heading_deg: compass heading 0=N, 90=E, 180=S, 270=W. -1 = no compass,
//                output gx/gy will be camera-relative (right/forward).
//   lat, lon:    camera GPS position in degrees (0.0, 0.0 = no GPS).
//                When set, tracks get absolute GPS coordinates.
// Pitch/roll come from set_imu() automatically.
void sentai_tracker_set_pose(int altitude_cm, int heading_deg, float lat, float lon);

// Enable/disable tracking.
void sentai_tracker_set_enabled(int enable);
int  sentai_tracker_is_enabled(void);

// Read back current pose values (set via set_pose / set_imu).
// Any output pointer may be NULL if not needed.
void sentai_tracker_get_pose(int* altitude_cm, int* heading_deg,
                             float* lat, float* lon,
                             float* pitch_deg, float* roll_deg);

// Compute camera footprint on ground (4 image corners → GPS).
// Uses active camera config + current IMU + pose.
// out_lat/out_lon: arrays of 4 floats [TL, TR, BR, BL].
// Returns number of corners that hit the ground (0-4). 0 = no valid footprint.
int  sentai_tracker_get_footprint(float* out_lat, float* out_lon);

#ifdef __cplusplus
}
#endif

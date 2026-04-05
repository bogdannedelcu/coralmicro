// sentai_tracker.cc — SentAI-SORT: Lightweight multi-object tracker
//
// Architecture: Called synchronously from InferTask after NMS (~0.9ms/frame).
//
//   1. Kalman predict all tracks (constant-velocity, block-diagonal P)
//   2. IMU camera motion compensation (shift predictions by IMU delta)
//   3. Compute HSV color histograms for all detections (from tensor)
//   4. ByteTrack Stage 1: high-conf dets → all tracks (IoU + histogram cost)
//   5. ByteTrack Stage 2: low-conf dets → unmatched confirmed/lost tracks
//   6. Update matched, create new, age unmatched, remove stale
//
// Memory: ~14 KB total (7 KB tracks + 6.4 KB histograms in SDRAM).
// All state is static — zero heap allocation.

#include "sentai_tracker.h"
#include "detection_task.h"

#include <cmath>
#include <cstring>
#include <cstdio>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/queue.h"
#include "third_party/freertos_kernel/include/semphr.h"

// =====================================================================
// Constants
// =====================================================================

// Default camera config (OV5640 stock lens, 1280×720 native, mounted straight down)
static const CameraConfig kDefaultCamCfg = {
    /* fov_h_deg       */ 70.8f,
    /* fov_v_deg       */ 43.4f,
    /* mount_pitch_deg */ 0.0f,   // 0 = straight down (perpendicular to ground)
    /* mount_roll_deg  */ 0.0f,   // 0 = landscape
    /* mount_yaw_deg   */ 0.0f,   // 0 = board forward
};

// Kalman process noise (tuned for ~15 fps, model-input pixel units)
static constexpr float kQ_pos  = 1.0f;     // position
static constexpr float kQ_a    = 0.01f;    // aspect ratio
static constexpr float kQ_h    = 1.0f;     // height
static constexpr float kQ_vp   = 0.1f;     // position velocity
static constexpr float kQ_va   = 0.001f;   // aspect velocity
static constexpr float kQ_vh   = 0.1f;     // height velocity

// Kalman measurement noise
static constexpr float kR_pos  = 4.0f;     // position
static constexpr float kR_a    = 0.1f;     // aspect ratio
static constexpr float kR_h    = 4.0f;     // height

// Histogram EMA smoothing factor (0 = fully new, 1 = fully old)
static constexpr float kHistAlpha = 0.7f;

// Event queue depth
static constexpr int kEvtQueueDepth = 16;

// =====================================================================
// Internal types
// =====================================================================

// Block-diagonal Kalman covariance: 12 floats instead of full 8×8 = 64.
// Properly tracks position-velocity cross-covariance for correct velocity updates.
struct KalmanState {
    float x[8];     // state: [cx, cy, a, h, vx, vy, va, vh]
    float pp[4];    // P position-position diagonal
    float pv[4];    // P position-velocity cross diagonal
    float vv[4];    // P velocity-velocity diagonal
};

struct InternalTrack {
    uint16_t    id;
    uint8_t     state;          // TRACK_TENTATIVE / CONFIRMED / LOST
    uint8_t     hits;           // consecutive matches
    uint8_t     misses;         // consecutive misses
    uint8_t     total_hits;     // lifetime matches (capped at 255)
    int16_t     class_id;
    int16_t     conf_permil;
    uint32_t    first_frame;
    uint32_t    last_frame;
    KalmanState kf;
    uint8_t     hist[TRACKER_HIST_SIZE];  // 128 bytes
    uint8_t     hist_valid;
};

// =====================================================================
// Static state
// =====================================================================

namespace {

// Track pool (~7.3 KB in SDRAM)
__attribute__((section(".sdram_bss"), aligned(4)))
static InternalTrack s_tracks[TRACKER_MAX_TRACKS];
static int      s_num_tracks = 0;
static uint16_t s_next_id    = 1;

// Configuration
static TrackerConfig s_cfg = {
    /* min_hits         */ 3,
    /* max_misses       */ 5,
    /* max_lost         */ 1,
    /* enable_histogram */ 1,
    /* enable_imu_cmc   */ 1,
    /* _pad             */ {0, 0, 0},
    /* iou_thresh       */ 300,     // 0.3
    /* hist_weight      */ 300,     // 0.3
    /* high_thresh      */ 500,     // 0.5
    /* low_thresh       */ 100,     // 0.1
};

static volatile int s_enabled = 0;

// Per-camera configuration (2 cameras max)
static CameraConfig s_cam_cfg[TRACKER_MAX_CAMERAS] = {
    { 70.8f, 43.4f, 0.0f, 0.0f, 0.0f },   // cam 0: stock OV5640, straight down
    { 70.8f, 43.4f, 0.0f, 0.0f, 0.0f },   // cam 1: stock OV5640, straight down
};
static int s_active_cam = 0;

// Convenience: get active camera config
static inline const CameraConfig* cam_cfg(void) {
    return &s_cam_cfg[s_active_cam];
}

// IMU state for CMC
static float s_imu_pitch = 0, s_imu_roll = 0;
static float s_imu_prev_pitch = 0, s_imu_prev_roll = 0;
static int   s_imu_has_prev = 0;

// Sensor pose for ground-plane projection
static int   s_pose_altitude_cm = 0;   // 0 = no projection
static int   s_pose_heading_deg = -1;  // -1 = no compass (camera-relative)
static float s_pose_lat = 0.0f;       // camera GPS latitude (degrees)
static float s_pose_lon = 0.0f;       // camera GPS longitude (degrees)

// Cached model input dimensions (set during update())
static int s_model_w = 0, s_model_h = 0;

// FreeRTOS primitives
static QueueHandle_t     s_evt_queue = nullptr;
static SemaphoreHandle_t s_mutex     = nullptr;
static int               s_inited    = 0;

// Scratch: detection histograms (~6.4 KB in SDRAM)
__attribute__((section(".sdram_bss"), aligned(4)))
static uint8_t s_det_hists[DETECTION_MAX_DETS][TRACKER_HIST_SIZE];

// Scratch: match arrays (DTCM — small, fast access)
static int s_match_t2d[TRACKER_MAX_TRACKS];  // track → det index (-1 = unmatched)
static int s_match_d2t[DETECTION_MAX_DETS];  // det → track index (-1 = unmatched)

}  // anonymous namespace

// =====================================================================
// Kalman filter (8-state, block-diagonal covariance)
// =====================================================================

static void kf_init(KalmanState* kf, float cx, float cy, float a, float h) {
    kf->x[0] = cx;   kf->x[1] = cy;   kf->x[2] = a;     kf->x[3] = h;
    kf->x[4] = 0.0f; kf->x[5] = 0.0f; kf->x[6] = 0.0f;  kf->x[7] = 0.0f;

    kf->pp[0] = 10.0f;  kf->pp[1] = 10.0f;  kf->pp[2] = 0.1f;   kf->pp[3] = 10.0f;
    kf->pv[0] = 0.0f;   kf->pv[1] = 0.0f;   kf->pv[2] = 0.0f;   kf->pv[3] = 0.0f;
    kf->vv[0] = 100.0f; kf->vv[1] = 100.0f; kf->vv[2] = 0.01f;  kf->vv[3] = 100.0f;
}

// Predict step: constant-velocity model, dt = 1 frame.
//   x_pos += x_vel
//   P_pp  = pp + 2*pv + vv + Q_pos
//   P_pv  = pv + vv
//   P_vv  = vv + Q_vel
static void kf_predict(KalmanState* kf) {
    kf->x[0] += kf->x[4];
    kf->x[1] += kf->x[5];
    kf->x[2] += kf->x[6];
    kf->x[3] += kf->x[7];

    static const float qp[4] = {kQ_pos, kQ_pos, kQ_a, kQ_h};
    static const float qv[4] = {kQ_vp,  kQ_vp,  kQ_va, kQ_vh};

    for (int i = 0; i < 4; i++) {
        kf->pp[i] += 2.0f * kf->pv[i] + kf->vv[i] + qp[i];
        kf->pv[i] += kf->vv[i];
        kf->vv[i] += qv[i];
    }
}

// Update step: measurement z = [cx, cy, a, h].
//   innovation = z - x_pos
//   S  = pp + R
//   Kp = pp / S   (position gain)
//   Kv = pv / S   (velocity gain — properly derived from block structure)
//   x_pos += Kp * innov;  x_vel += Kv * innov
//   pp' = (1-Kp) * pp;  pv' = (1-Kp) * pv;  vv' = vv - Kv * pv
static void kf_update(KalmanState* kf, float cx, float cy, float a, float h) {
    const float z[4] = {cx, cy, a, h};
    static const float r[4] = {kR_pos, kR_pos, kR_a, kR_h};

    for (int i = 0; i < 4; i++) {
        float innov = z[i] - kf->x[i];
        float s     = kf->pp[i] + r[i];
        float si    = 1.0f / s;
        float kp    = kf->pp[i] * si;      // position gain
        float kv    = kf->pv[i] * si;      // velocity gain

        kf->x[i]     += kp * innov;        // position update
        kf->x[i + 4] += kv * innov;        // velocity update

        float omp = 1.0f - kp;
        kf->vv[i] -= kv * kf->pv[i];
        kf->pp[i] *= omp;
        kf->pv[i] *= omp;
    }
}

// Extract bbox from Kalman state.  a = aspect ratio = w/h.
static inline void kf_bbox(const KalmanState* kf,
                            int16_t* x1, int16_t* y1, int16_t* x2, int16_t* y2) {
    float cx = kf->x[0], cy = kf->x[1];
    float a  = kf->x[2], h  = kf->x[3];
    float w  = a * h;
    *x1 = (int16_t)(cx - w * 0.5f);
    *y1 = (int16_t)(cy - h * 0.5f);
    *x2 = (int16_t)(cx + w * 0.5f);
    *y2 = (int16_t)(cy + h * 0.5f);
}

// =====================================================================
// Color histogram (HSV hue, 4×4 spatial grid, 8 bins per cell = 128D)
// =====================================================================

// Approximate hue (0–179) from RGB using integer arithmetic.
static inline uint8_t rgb_hue(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t mx = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    uint8_t mn = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    if (mx == mn) return 0;
    int d = mx - mn, hue;
    if      (mx == r) hue = 30 * (int)(g - b) / d;
    else if (mx == g) hue = 60 + 30 * (int)(b - r) / d;
    else              hue = 120 + 30 * (int)(r - g) / d;
    if (hue < 0) hue += 180;
    return (uint8_t)hue;
}

// Extract color histogram from a detection bbox in the int8 tensor.
// Tensor data is int8 with zero_point; RGB is recovered as (int8 + offset).
// Adaptive subsampling keeps cost ≤ ~50µs per bbox regardless of size.
static void hist_extract(const uint8_t* tensor, int tw, int th, int tch, int zp,
                         int bx1, int by1, int bx2, int by2,
                         uint8_t* out) {
    memset(out, 0, TRACKER_HIST_SIZE);
    // Clamp to tensor bounds
    if (bx1 < 0) bx1 = 0;
    if (by1 < 0) by1 = 0;
    if (bx2 > tw) bx2 = tw;
    if (by2 > th) by2 = th;
    int bw = bx2 - bx1, bh = by2 - by1;
    if (bw < 2 || bh < 2) return;

    // Adaptive step: keep ≤ ~2000 pixels per crop
    int step = 1;
    if (bw > 64 || bh > 64)  step = 2;
    if (bw > 128 || bh > 128) step = 4;

    int stride = tw * tch;
    int off = (zp == -128) ? 128 : -zp;  // int8 → uint8 conversion offset

    for (int y = by1; y < by2; y += step) {
        int gy = (y - by1) * TRACKER_HIST_GRID / bh;
        if (gy >= TRACKER_HIST_GRID) gy = TRACKER_HIST_GRID - 1;
        const int8_t* px = (const int8_t*)(tensor + y * stride + bx1 * tch);

        for (int x = bx1; x < bx2; x += step) {
            int gx = (x - bx1) * TRACKER_HIST_GRID / bw;
            if (gx >= TRACKER_HIST_GRID) gx = TRACKER_HIST_GRID - 1;

            uint8_t r = (uint8_t)(px[0] + off);
            uint8_t g = (uint8_t)(px[1] + off);
            uint8_t b = (uint8_t)(px[2] + off);

            uint8_t hue = rgb_hue(r, g, b);
            int bin = hue * TRACKER_HIST_BINS / 180;
            if (bin >= TRACKER_HIST_BINS) bin = TRACKER_HIST_BINS - 1;

            int idx = (gy * TRACKER_HIST_GRID + gx) * TRACKER_HIST_BINS + bin;
            if (out[idx] < 255) out[idx]++;

            px += tch * step;
        }
    }

    // Normalize each grid cell to sum ≈ 255
    for (int c = 0; c < TRACKER_HIST_GRID * TRACKER_HIST_GRID; c++) {
        int base = c * TRACKER_HIST_BINS;
        int sum = 0;
        for (int i = 0; i < TRACKER_HIST_BINS; i++) sum += out[base + i];
        if (sum > 0) {
            for (int i = 0; i < TRACKER_HIST_BINS; i++)
                out[base + i] = (uint8_t)((int)out[base + i] * 255 / sum);
        }
    }
}

// Bhattacharyya distance between two histograms.
// Returns 0.0 (identical) to 1.0 (completely different).
static float hist_bhattacharyya(const uint8_t* a, const uint8_t* b) {
    float bc = 0.0f;
    for (int c = 0; c < TRACKER_HIST_GRID * TRACKER_HIST_GRID; c++) {
        int base = c * TRACKER_HIST_BINS;
        float sa = 0, sb = 0, cell = 0;
        for (int i = 0; i < TRACKER_HIST_BINS; i++) {
            float ai = (float)a[base + i], bi = (float)b[base + i];
            sa += ai;  sb += bi;
            cell += sqrtf(ai * bi);
        }
        float norm = sqrtf(sa * sb);
        if (norm > 0.01f) bc += cell / norm;
    }
    bc /= (float)(TRACKER_HIST_GRID * TRACKER_HIST_GRID);
    return 1.0f - bc;
}

// Exponential moving average update of track histogram.
static void hist_ema(uint8_t* dst, const uint8_t* src, float alpha) {
    for (int i = 0; i < TRACKER_HIST_SIZE; i++) {
        float v = alpha * (float)dst[i] + (1.0f - alpha) * (float)src[i];
        dst[i] = (uint8_t)(v + 0.5f);
    }
}

// =====================================================================
// IoU
// =====================================================================

static float compute_iou(int16_t ax1, int16_t ay1, int16_t ax2, int16_t ay2,
                         int16_t bx1, int16_t by1, int16_t bx2, int16_t by2) {
    int ix1 = (ax1 > bx1) ? ax1 : bx1;
    int iy1 = (ay1 > by1) ? ay1 : by1;
    int ix2 = (ax2 < bx2) ? ax2 : bx2;
    int iy2 = (ay2 < by2) ? ay2 : by2;
    if (ix1 >= ix2 || iy1 >= iy2) return 0.0f;
    float inter = (float)(ix2 - ix1) * (float)(iy2 - iy1);
    float ua    = (float)(ax2 - ax1) * (float)(ay2 - ay1);
    float ub    = (float)(bx2 - bx1) * (float)(by2 - by1);
    float u     = ua + ub - inter;
    return (u > 0.0f) ? inter / u : 0.0f;
}

// =====================================================================
// Constants for angle conversions
// =====================================================================

static constexpr float kDeg2Rad = (float)(M_PI / 180.0);
static constexpr float kCmPerDegLat = 11132000.0f;  // 1° latitude ≈ 111,320 m

// =====================================================================
// IMU Camera Motion Compensation
// =====================================================================

// Shift all Kalman predictions by the IMU-derived pixel displacement.
// delta_pitch → vertical shift, delta_roll → horizontal shift.
// Uses active camera's FOV and mount_roll (portrait vs landscape swap).
static void apply_imu_cmc(int model_w, int model_h) {
    if (!s_cfg.enable_imu_cmc || !s_imu_has_prev) return;
    float dp = s_imu_pitch - s_imu_prev_pitch;
    float dr = s_imu_roll  - s_imu_prev_roll;
    const CameraConfig* cc = cam_cfg();
    float fov_h = cc->fov_h_deg;
    float fov_v = cc->fov_v_deg;
    // Account for mount_roll: if portrait (90°), sensor axes swap
    float mr = cc->mount_roll_deg * kDeg2Rad;
    float cmr = cosf(mr), smr = sinf(mr);
    // IMU delta in board frame → rotate into sensor frame
    float dp_sensor =  dp * cmr + dr * smr;   // along sensor V axis
    float dr_sensor = -dp * smr + dr * cmr;   // along sensor H axis
    // Convert degrees → pixels:  pix_per_deg = model_dim / fov_deg
    float dx = dr_sensor * ((float)model_w / fov_h);
    float dy = dp_sensor * ((float)model_h / fov_v);
    for (int i = 0; i < s_num_tracks; i++) {
        s_tracks[i].kf.x[0] += dx;
        s_tracks[i].kf.x[1] += dy;
    }
}

// =====================================================================
// Ground-plane projection (pixel → world coordinates)
// =====================================================================

// =====================================================================
// 4-corner ray-cast ground projection (trapezoid-aware)
// =====================================================================
//
// Convention:
//   mount_pitch = 0  → camera looks straight DOWN (perpendicular to ground)
//   mount_pitch = 25 → camera tilted 25° from vertical toward board-forward
//
// Camera frame (sensor plane):
//   Xc = right on sensor (col direction)
//   Yc = down on sensor (row direction)
//   Zc = optical axis (into scene)
//
// Board frame:
//   Xb = board right
//   Yb = board forward
//   Zb = board up
//
// World frame (with IMU + heading):
//   Xw = East  (or camera-right if no heading)
//   Yw = North (or camera-forward if no heading)
//   Zw = Up
//
// Rotation chain: camera → board → world
//   R_board = Rz(mount_yaw) * Rx(mount_pitch_from_down) * Rz(mount_roll)
//   where mount_pitch_from_down rotates optical axis from -Zb toward Yb
//   R_world = Rx(imu_pitch) * Ry(imu_roll)  (IMU stabilization)

// Cast one pixel (px, py) in model-input coords to ground plane.
// Returns 1 if ray hits ground, 0 if ray goes above horizon.
// out_gx, out_gy: ground coords in camera-centric frame (right, forward) in cm.
static int ray_to_ground(float px, float py, int tw, int th,
                         float alt_cm,
                         float tan_hfov_half, float tan_vfov_half,
                         float cp, float sp,   // cos/sin of total pitch from vertical
                         float cr, float sr,   // cos/sin of mount_roll
                         float cy_r, float sy_r, // cos/sin of mount_yaw
                         float* out_gx, float* out_gy) {
    // Normalized device coords: [-1, +1]
    float ndx = 2.0f * px / tw - 1.0f;
    float ndy = 2.0f * py / th - 1.0f;

    // Ray in camera frame (Xc=right, Yc=down, Zc=forward)
    float rx_c = tan_hfov_half * ndx;
    float ry_c = tan_vfov_half * ndy;
    float rz_c = 1.0f;

    // Apply mount_roll: rotate around Zc (optical axis)
    // This handles portrait (90°) vs landscape (0°) mounting
    float rx_r = rx_c * cr - ry_c * sr;
    float ry_r = rx_c * sr + ry_c * cr;
    float rz_r = rz_c;

    // Apply mount_pitch (tilt from vertical):
    // mount_pitch=0 → optical axis = -Zb (straight down)
    // mount_pitch=25° → optical axis tilted 25° from -Zb toward Yb
    // Rotation around Xb axis by (90° - mount_pitch)
    //   Actually: camera Zc maps to direction in board frame:
    //     At mount_pitch=0: Zc → -Zb (down)
    //     At mount_pitch=90: Zc → Yb (forward, horizon)
    //   So we rotate camera frame into board frame:
    //     Xb =  Xc_rotated
    //     Yb =  Yc*cos(mp) + Zc*sin(mp)    (mp from down)
    //     Zb = -Yc*sin(mp) + Zc*cos(mp)    (up)
    //   But we want Zc→-Zb when mp=0, so:
    //     Xb  =  rx_r
    //     Yb  =  ry_r * sin(mp) + rz_r * (-cos(mp))  ... wait
    //   Let’s think clearly:
    //     mp=0 (down): camera Zc → -Zb, camera Yc → -Yb (nadir: sensor down = forward)
    //     mp=90 (horizon): camera Zc → Yb, camera Yc → -Zb (bottom of image = ground)
    //   Rotation matrix for camera→board (pitch angle mp from down):
    float smp = sp, cmp = cp;  // sp=sin(mount_pitch), cp=cos(mount_pitch)
    float rx_b =  rx_r;
    float ry_b =  ry_r * (-cmp) + rz_r * smp;   // board forward component
    float rz_b =  ry_r * smp    + rz_r * cmp;   // board up component (negated = down)
    // Note: at mp=0: ry_b = -ry_r*1 + rz_r*0 = -ry_r (sensor down → board -forward → will fix with sign)
    //                rz_b = ry_r*0 + rz_r*1 = rz_r  ... but Zc should map to -Zb!
    // Fix: optical axis at mp=0 is -Zb, so:
    rz_b = -(ry_r * smp + rz_r * cmp);  // negate so Zc→-Zb at mp=0

    // Apply mount_yaw: rotate around Zb
    float rx_b2 = rx_b * cy_r - ry_b * sy_r;
    float ry_b2 = rx_b * sy_r + ry_b * cy_r;
    float rz_b2 = rz_b;

    // Now apply IMU pitch/roll to go board→world
    // For simplicity, we fold IMU into the total pitch (already done by caller)
    // So rx_b2, ry_b2, rz_b2 are already in world-ish frame
    // (caller pre-combined IMU pitch into 'total pitch')

    // World frame: X=right, Y=forward, Z=up
    // Ray must go downward (rz_b2 < 0) to hit ground
    if (rz_b2 >= -0.001f) return 0;

    // Intersect with ground plane Z=0, camera at (0, 0, alt_cm)
    float t = alt_cm / (-rz_b2);
    if (t > 100000.0f) t = 100000.0f;  // clamp to 1km

    *out_gx = t * rx_b2;   // right (camera-right or East after heading rotation)
    *out_gy = t * ry_b2;   // forward (camera-forward or North after heading rotation)
    return 1;
}

// Project bbox from model-input pixels to ground plane using 4-corner ray-cast.
// Returns the ground trapezoid's center, distance, and width.
// Uses active camera config for FOV and mount orientation.
static void project_bbox_to_ground(int16_t bx1, int16_t by1, int16_t bx2, int16_t by2,
                                   int tw, int th,
                                   int altitude_cm,
                                   int32_t* out_right, int32_t* out_fwd,
                                   int32_t* out_dist, int16_t* out_width) {
    *out_right = *out_fwd = *out_dist = 0;
    *out_width = 0;
    if (altitude_cm <= 0 || tw <= 0 || th <= 0) return;

    const CameraConfig* cc = cam_cfg();
    float tan_hh = tanf(cc->fov_h_deg * 0.5f * kDeg2Rad);
    float tan_vh = tanf(cc->fov_v_deg * 0.5f * kDeg2Rad);

    // Total pitch = mount_pitch + IMU pitch
    // mount_pitch: from vertical (0=down, 25=tilted forward)
    // IMU pitch: deviation from level (positive = nose up from board perspective)
    // Combined: effective tilt from vertical
    float total_pitch = cc->mount_pitch_deg + s_imu_pitch;
    float cp = cosf(total_pitch * kDeg2Rad);
    float sp = sinf(total_pitch * kDeg2Rad);

    float mr = cc->mount_roll_deg * kDeg2Rad;
    float cr = cosf(mr), sr = sinf(mr);

    float my = cc->mount_yaw_deg * kDeg2Rad;
    float cy_r = cosf(my), sy_r = sinf(my);

    // Cast all 4 corners of the bbox
    float gx[4], gy[4];
    float corners_x[4] = { (float)bx1, (float)bx2, (float)bx2, (float)bx1 };
    float corners_y[4] = { (float)by1, (float)by1, (float)by2, (float)by2 };
    // TL, TR, BR, BL

    int hits = 0;
    float sum_gx = 0, sum_gy = 0;
    for (int i = 0; i < 4; i++) {
        if (ray_to_ground(corners_x[i], corners_y[i], tw, th,
                          (float)altitude_cm, tan_hh, tan_vh,
                          cp, sp, cr, sr, cy_r, sy_r,
                          &gx[i], &gy[i])) {
            sum_gx += gx[i];
            sum_gy += gy[i];
            hits++;
        } else {
            gx[i] = gy[i] = 0;
        }
    }

    if (hits < 2) return;  // not enough corners hit ground

    // Center = average of hit corners
    float center_gx = sum_gx / (float)hits;
    float center_gy = sum_gy / (float)hits;
    float dist = sqrtf(center_gx * center_gx + center_gy * center_gy);

    *out_right = (int32_t)center_gx;
    *out_fwd   = (int32_t)center_gy;
    *out_dist  = (int32_t)dist;

    // Width = average of top edge and bottom edge widths on ground (trapezoid)
    // Top edge: TL→TR, Bottom edge: BL→BR
    float top_w = sqrtf((gx[1]-gx[0])*(gx[1]-gx[0]) + (gy[1]-gy[0])*(gy[1]-gy[0]));
    float bot_w = sqrtf((gx[2]-gx[3])*(gx[2]-gx[3]) + (gy[2]-gy[3])*(gy[2]-gy[3]));
    float avg_w = (top_w + bot_w) * 0.5f;
    if (avg_w > 32767.0f) avg_w = 32767.0f;
    *out_width = (int16_t)avg_w;
}

// Rotate camera-relative (right, forward) by heading to get (East, North).
static void rotate_by_heading(int32_t* gx, int32_t* gy, int heading_deg) {
    if (heading_deg < 0) return;  // no compass → keep camera-relative
    float h_rad = (float)heading_deg * kDeg2Rad;
    float ch = cosf(h_rad), sh = sinf(h_rad);
    float right = (float)*gx, fwd = (float)*gy;
    // Camera-forward = heading direction, camera-right = heading + 90°
    // East  = forward * sin(heading) + right * cos(heading)
    // North = forward * cos(heading) - right * sin(heading)
    *gx = (int32_t)(fwd * sh + right * ch);   // East
    *gy = (int32_t)(fwd * ch - right * sh);   // North
}

// Convert ground offset (East/North cm from camera) to absolute GPS.
static void offset_to_gps(int32_t east_cm, int32_t north_cm,
                          float cam_lat, float cam_lon,
                          float* out_lat, float* out_lon) {
    if (cam_lat == 0.0f && cam_lon == 0.0f) {
        *out_lat = *out_lon = 0.0f;
        return;
    }
    *out_lat = cam_lat + (float)north_cm / kCmPerDegLat;
    float cos_lat = cosf(cam_lat * kDeg2Rad);
    if (cos_lat < 0.01f) cos_lat = 0.01f;  // avoid div by zero near poles
    *out_lon = cam_lon + (float)east_cm / (kCmPerDegLat * cos_lat);
}

// Fill ground-plane fields in a TrackedObject.
static void fill_ground_coords(TrackedObject* o) {
    if (s_pose_altitude_cm <= 0 || s_model_w <= 0) {
        o->gx_cm = o->gy_cm = o->dist_cm = 0;
        o->width_cm = 0;
        o->lat = o->lon = 0.0f;
        return;
    }
    project_bbox_to_ground(o->x1, o->y1, o->x2, o->y2,
                           s_model_w, s_model_h,
                           s_pose_altitude_cm,
                           &o->gx_cm, &o->gy_cm, &o->dist_cm, &o->width_cm);
    rotate_by_heading(&o->gx_cm, &o->gy_cm, s_pose_heading_deg);
    offset_to_gps(o->gx_cm, o->gy_cm, s_pose_lat, s_pose_lon,
                  &o->lat, &o->lon);
}

// =====================================================================
// Track lifecycle helpers
// =====================================================================

static void emit_event(int type, const InternalTrack* t, uint32_t seq) {
    if (!s_evt_queue) return;
    TrackEvent e;
    e.type        = (uint8_t)type;
    e._pad        = 0;
    e.id          = t->id;
    e.class_id    = t->class_id;
    kf_bbox(&t->kf, &e.x1, &e.y1, &e.x2, &e.y2);
    e.conf_permil = t->conf_permil;
    e.frame_seq   = seq;
    // Ground-plane projection
    e.gx_cm = e.gy_cm = e.dist_cm = 0;
    e.lat = e.lon = 0.0f;
    if (s_pose_altitude_cm > 0 && s_model_w > 0) {
        int16_t width_cm;
        project_bbox_to_ground(e.x1, e.y1, e.x2, e.y2,
                               s_model_w, s_model_h,
                               s_pose_altitude_cm,
                               &e.gx_cm, &e.gy_cm, &e.dist_cm, &width_cm);
        rotate_by_heading(&e.gx_cm, &e.gy_cm, s_pose_heading_deg);
        offset_to_gps(e.gx_cm, e.gy_cm, s_pose_lat, s_pose_lon,
                      &e.lat, &e.lon);
    }
    // Non-blocking push; drop oldest if full
    if (xQueueSend(s_evt_queue, &e, 0) != pdTRUE) {
        TrackEvent discard;
        xQueueReceive(s_evt_queue, &discard, 0);
        xQueueSend(s_evt_queue, &e, 0);
    }
}

static void create_track(const Detection* det, uint32_t seq, const uint8_t* dhist) {
    if (s_num_tracks >= TRACKER_MAX_TRACKS) return;
    InternalTrack* t = &s_tracks[s_num_tracks++];
    t->id          = s_next_id++;
    if (s_next_id == 0) s_next_id = 1;  // skip 0 (reserved)
    t->state       = TRACK_TENTATIVE;
    t->hits        = 1;
    t->misses      = 0;
    t->total_hits  = 1;
    t->class_id    = det->class_id;
    t->conf_permil = det->conf_permil;
    t->first_frame = seq;
    t->last_frame  = seq;

    float cx = (det->x1 + det->x2) * 0.5f;
    float cy = (det->y1 + det->y2) * 0.5f;
    float w  = (float)(det->x2 - det->x1);
    float h  = (float)(det->y2 - det->y1);
    float a  = (h > 0.1f) ? w / h : 1.0f;
    kf_init(&t->kf, cx, cy, a, h);

    if (dhist) {
        memcpy(t->hist, dhist, TRACKER_HIST_SIZE);
        t->hist_valid = 1;
    } else {
        memset(t->hist, 0, TRACKER_HIST_SIZE);
        t->hist_valid = 0;
    }
}

static void match_track(InternalTrack* t, const Detection* det, uint32_t seq,
                        const uint8_t* dhist) {
    float cx = (det->x1 + det->x2) * 0.5f;
    float cy = (det->y1 + det->y2) * 0.5f;
    float w  = (float)(det->x2 - det->x1);
    float h  = (float)(det->y2 - det->y1);
    float a  = (h > 0.1f) ? w / h : 1.0f;
    kf_update(&t->kf, cx, cy, a, h);

    t->class_id    = det->class_id;
    t->conf_permil = det->conf_permil;
    t->last_frame  = seq;
    t->hits++;
    if (t->total_hits < 255) t->total_hits++;
    t->misses = 0;

    // Histogram EMA update
    if (dhist && s_cfg.enable_histogram) {
        if (t->hist_valid)
            hist_ema(t->hist, dhist, kHistAlpha);
        else {
            memcpy(t->hist, dhist, TRACKER_HIST_SIZE);
            t->hist_valid = 1;
        }
    }

    // State transitions
    if (t->state == TRACK_TENTATIVE && t->hits >= s_cfg.min_hits) {
        t->state = TRACK_CONFIRMED;
        emit_event(TRACK_EVT_NEW, t, seq);
    } else if (t->state == TRACK_CONFIRMED) {
        emit_event(TRACK_EVT_UPDATE, t, seq);
    } else if (t->state == TRACK_LOST) {
        t->state = TRACK_CONFIRMED;
        emit_event(TRACK_EVT_UPDATE, t, seq);
    }
}

// =====================================================================
// Main update — ByteTrack two-stage greedy association
// =====================================================================

extern "C"
int sentai_tracker_update(const Detection* dets, int n_dets,
                          const uint8_t* tensor_buf, int tw, int th, int tch, int zp,
                          uint32_t frame_seq) {
    sentai_tracker_init();  // lazy init

    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);

    // Cache model input dimensions for ground projection
    s_model_w = tw;
    s_model_h = th;

    int nd = (n_dets > DETECTION_MAX_DETS) ? DETECTION_MAX_DETS : n_dets;

    // ---- 1. Kalman predict all tracks ----
    for (int i = 0; i < s_num_tracks; i++)
        kf_predict(&s_tracks[i].kf);

    // ---- 2. IMU camera motion compensation ----
    apply_imu_cmc(tw, th);
    s_imu_prev_pitch = s_imu_pitch;
    s_imu_prev_roll  = s_imu_roll;
    s_imu_has_prev   = 1;

    // ---- 3. Compute histograms for detections ----
    int use_hist = s_cfg.enable_histogram && tensor_buf && tch >= 3;
    if (use_hist) {
        for (int d = 0; d < nd; d++)
            hist_extract(tensor_buf, tw, th, tch, zp,
                         dets[d].x1, dets[d].y1, dets[d].x2, dets[d].y2,
                         s_det_hists[d]);
    }

    // ---- 4. Initialize match arrays ----
    for (int i = 0; i < s_num_tracks; i++) s_match_t2d[i] = -1;
    for (int d = 0; d < nd; d++)           s_match_d2t[d] = -1;

    float cost_thresh = 1.0f - (float)s_cfg.iou_thresh / 1000.0f;
    float hw = (float)s_cfg.hist_weight / 1000.0f;

    // ---- 5. Stage 1: high-confidence detections → all tracks ----
    // Greedy: for each high-conf detection, find best-cost track
    for (int d = 0; d < nd; d++) {
        if (dets[d].conf_permil < s_cfg.high_thresh) continue;

        float best = cost_thresh;
        int   bt   = -1;

        for (int t = 0; t < s_num_tracks; t++) {
            if (s_match_t2d[t] >= 0) continue;

            int16_t tx1, ty1, tx2, ty2;
            kf_bbox(&s_tracks[t].kf, &tx1, &ty1, &tx2, &ty2);

            float iou = compute_iou(tx1, ty1, tx2, ty2,
                                    dets[d].x1, dets[d].y1, dets[d].x2, dets[d].y2);
            float c = 1.0f - iou;

            // Blend histogram distance into cost
            if (use_hist && s_tracks[t].hist_valid)
                c = (1.0f - hw) * c + hw * hist_bhattacharyya(s_tracks[t].hist, s_det_hists[d]);

            if (c < best) { best = c; bt = t; }
        }

        if (bt >= 0) {
            s_match_t2d[bt] = d;
            s_match_d2t[d]  = bt;
        }
    }

    // ---- 6. Stage 2: low-confidence detections → unmatched confirmed/lost tracks ----
    // IoU-only matching (no histogram for partial/noisy low-conf boxes)
    for (int d = 0; d < nd; d++) {
        if (dets[d].conf_permil >= s_cfg.high_thresh) continue;  // handled in stage 1
        if (dets[d].conf_permil <  s_cfg.low_thresh)  continue;  // below threshold

        float best = cost_thresh;
        int   bt   = -1;

        for (int t = 0; t < s_num_tracks; t++) {
            if (s_match_t2d[t] >= 0) continue;
            if (s_tracks[t].state == TRACK_TENTATIVE) continue;  // stage 2: confirmed/lost only

            int16_t tx1, ty1, tx2, ty2;
            kf_bbox(&s_tracks[t].kf, &tx1, &ty1, &tx2, &ty2);

            float iou = compute_iou(tx1, ty1, tx2, ty2,
                                    dets[d].x1, dets[d].y1, dets[d].x2, dets[d].y2);
            float c = 1.0f - iou;

            if (c < best) { best = c; bt = t; }
        }

        if (bt >= 0) {
            s_match_t2d[bt] = d;
            s_match_d2t[d]  = bt;
        }
    }

    // ---- 7. Update matched tracks ----
    for (int t = 0; t < s_num_tracks; t++) {
        if (s_match_t2d[t] < 0) continue;
        int d = s_match_t2d[t];
        const uint8_t* dh = use_hist ? s_det_hists[d] : nullptr;
        match_track(&s_tracks[t], &dets[d], frame_seq, dh);
    }

    // ---- 8. Create new tracks from unmatched high-confidence detections ----
    int n_old = s_num_tracks;  // save count before new tracks
    for (int d = 0; d < nd; d++) {
        if (s_match_d2t[d] >= 0) continue;                      // matched
        if (dets[d].conf_permil < s_cfg.high_thresh) continue;  // only high-conf
        const uint8_t* dh = use_hist ? s_det_hists[d] : nullptr;
        create_track(&dets[d], frame_seq, dh);
    }

    // ---- 9. Handle unmatched old tracks ----
    for (int t = 0; t < n_old; t++) {
        if (s_match_t2d[t] >= 0) continue;

        s_tracks[t].misses++;
        s_tracks[t].hits = 0;

        if (s_tracks[t].state == TRACK_CONFIRMED &&
            s_tracks[t].misses >= s_cfg.max_lost) {
            s_tracks[t].state = TRACK_LOST;
            emit_event(TRACK_EVT_LOST, &s_tracks[t], frame_seq);
        }

        bool remove = false;
        if (s_tracks[t].state == TRACK_LOST &&
            s_tracks[t].misses >= s_cfg.max_misses) {
            emit_event(TRACK_EVT_REMOVED, &s_tracks[t], frame_seq);
            remove = true;
        }
        if (s_tracks[t].state == TRACK_TENTATIVE &&
            s_tracks[t].misses >= 2) {
            remove = true;  // tentative tracks die quickly
        }

        if (remove)
            s_tracks[t].id = 0;  // mark for removal
    }

    // ---- 10. Compact: remove marked tracks (id == 0) ----
    int w = 0;
    for (int r = 0; r < s_num_tracks; r++) {
        if (s_tracks[r].id != 0) {
            if (w != r) s_tracks[w] = s_tracks[r];
            w++;
        }
    }
    s_num_tracks = w;

    // Count confirmed
    int active = 0;
    for (int i = 0; i < s_num_tracks; i++)
        if (s_tracks[i].state == TRACK_CONFIRMED) active++;

    if (s_mutex) xSemaphoreGive(s_mutex);
    return active;
}

// =====================================================================
// Public API
// =====================================================================

extern "C"
void sentai_tracker_init(void) {
    if (s_inited) return;
    s_mutex     = xSemaphoreCreateMutex();
    s_evt_queue = xQueueCreate(kEvtQueueDepth, sizeof(TrackEvent));
    s_inited    = 1;
}

extern "C"
void sentai_tracker_reset(void) {
    sentai_tracker_init();
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_num_tracks   = 0;
    s_next_id      = 1;
    s_imu_has_prev = 0;
    if (s_evt_queue) xQueueReset(s_evt_queue);
    if (s_mutex) xSemaphoreGive(s_mutex);
}

extern "C"
int sentai_tracker_get_tracks(TrackedObject* out, int max) {
    if (!s_inited) return 0;
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = 0;
    for (int i = 0; i < s_num_tracks && n < max; i++) {
        const InternalTrack* t = &s_tracks[i];
        if (t->state == TRACK_TENTATIVE) continue;  // only confirmed + lost
        out[n].id         = t->id;
        out[n].state      = t->state;
        out[n].hits       = t->hits;
        out[n].misses     = t->misses;
        out[n].total_hits = t->total_hits;
        out[n].class_id   = t->class_id;
        kf_bbox(&t->kf, &out[n].x1, &out[n].y1, &out[n].x2, &out[n].y2);
        out[n].conf_permil = t->conf_permil;
        out[n].age_frames  = (t->last_frame >= t->first_frame)
                             ? t->last_frame - t->first_frame : 0;
        fill_ground_coords(&out[n]);
        n++;
    }
    if (s_mutex) xSemaphoreGive(s_mutex);
    return n;
}

extern "C"
int sentai_tracker_get_event(TrackEvent* event, int timeout_ms) {
    if (!s_evt_queue) return 0;
    TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY
                                        : pdMS_TO_TICKS(timeout_ms);
    return (xQueueReceive(s_evt_queue, event, ticks) == pdTRUE) ? 1 : 0;
}

extern "C"
int sentai_tracker_num_active(void) {
    int n = 0;
    for (int i = 0; i < s_num_tracks; i++)
        if (s_tracks[i].state == TRACK_CONFIRMED) n++;
    return n;
}

extern "C"
void sentai_tracker_set_config(const TrackerConfig* cfg) { s_cfg = *cfg; }

extern "C"
void sentai_tracker_get_config(TrackerConfig* cfg) { *cfg = s_cfg; }

extern "C"
void sentai_tracker_set_imu(float pitch_deg, float roll_deg) {
    s_imu_pitch = pitch_deg;
    s_imu_roll  = roll_deg;
}

extern "C"
void sentai_tracker_set_pose(int altitude_cm, int heading_deg,
                             float lat, float lon) {
    s_pose_altitude_cm = altitude_cm;
    s_pose_heading_deg = heading_deg;
    s_pose_lat = lat;
    s_pose_lon = lon;
}

extern "C"
void sentai_tracker_set_camera(int cam_id, const CameraConfig* cfg) {
    if (cam_id >= 0 && cam_id < TRACKER_MAX_CAMERAS && cfg)
        s_cam_cfg[cam_id] = *cfg;
}

extern "C"
void sentai_tracker_get_camera(int cam_id, CameraConfig* cfg) {
    if (cam_id >= 0 && cam_id < TRACKER_MAX_CAMERAS && cfg)
        *cfg = s_cam_cfg[cam_id];
    else if (cfg)
        *cfg = kDefaultCamCfg;
}

extern "C"
void sentai_tracker_set_active_camera(int cam_id) {
    if (cam_id >= 0 && cam_id < TRACKER_MAX_CAMERAS)
        s_active_cam = cam_id;
}

extern "C"
int sentai_tracker_get_active_camera(void) { return s_active_cam; }

extern "C"
void sentai_tracker_get_pose(int* altitude_cm, int* heading_deg,
                             float* lat, float* lon,
                             float* pitch_deg, float* roll_deg) {
    if (altitude_cm) *altitude_cm = s_pose_altitude_cm;
    if (heading_deg) *heading_deg = s_pose_heading_deg;
    if (lat)         *lat         = s_pose_lat;
    if (lon)         *lon         = s_pose_lon;
    if (pitch_deg)   *pitch_deg   = s_imu_pitch;
    if (roll_deg)    *roll_deg    = s_imu_roll;
}

extern "C"
void sentai_tracker_set_enabled(int enable) { s_enabled = enable; }

extern "C"
int sentai_tracker_is_enabled(void) { return s_enabled; }

extern "C"
int sentai_tracker_get_footprint(float* out_lat, float* out_lon) {
    if (!out_lat || !out_lon) return 0;
    if (s_pose_altitude_cm <= 0 || s_model_w <= 0) return 0;

    const CameraConfig* cc = cam_cfg();
    float tan_hh = tanf(cc->fov_h_deg * 0.5f * kDeg2Rad);
    float tan_vh = tanf(cc->fov_v_deg * 0.5f * kDeg2Rad);

    float total_pitch = cc->mount_pitch_deg + s_imu_pitch;
    float cp = cosf(total_pitch * kDeg2Rad);
    float sp = sinf(total_pitch * kDeg2Rad);
    float cr = cosf(cc->mount_roll_deg * kDeg2Rad);
    float sr = sinf(cc->mount_roll_deg * kDeg2Rad);
    float cy_r = cosf(cc->mount_yaw_deg * kDeg2Rad);
    float sy_r = sinf(cc->mount_yaw_deg * kDeg2Rad);

    // Image corners in pixel coords: TL, TR, BR, BL
    float cx[4] = { 0.0f, (float)s_model_w, (float)s_model_w, 0.0f };
    float cy[4] = { 0.0f, 0.0f,             (float)s_model_h, (float)s_model_h };

    int hits = 0;
    for (int i = 0; i < 4; i++) {
        float gx, gy;
        if (ray_to_ground(cx[i], cy[i], s_model_w, s_model_h,
                          (float)s_pose_altitude_cm, tan_hh, tan_vh,
                          cp, sp, cr, sr, cy_r, sy_r,
                          &gx, &gy)) {
            // Apply heading rotation
            if (s_pose_heading_deg >= 0) {
                float h_rad = (float)s_pose_heading_deg * kDeg2Rad;
                float ch = cosf(h_rad), sh = sinf(h_rad);
                float r = gx, f = gy;
                gx = (int32_t)(f * sh + r * ch);   // East
                gy = (int32_t)(f * ch - r * sh);   // North
            }
            // Convert to GPS
            if (s_pose_lat != 0.0f || s_pose_lon != 0.0f) {
                out_lat[i] = s_pose_lat + gy / kCmPerDegLat;
                float cos_lat = cosf(s_pose_lat * kDeg2Rad);
                if (cos_lat < 0.01f) cos_lat = 0.01f;
                out_lon[i] = s_pose_lon + gx / (kCmPerDegLat * cos_lat);
            } else {
                out_lat[i] = 0.0f;
                out_lon[i] = 0.0f;
            }
            hits++;
        } else {
            out_lat[i] = 0.0f;
            out_lon[i] = 0.0f;
        }
    }
    return hits;
}

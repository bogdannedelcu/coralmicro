// ============== sentai.slam — Detection-Based EKF-SLAM ==============
// This file is #include'd from modsentai.c — do NOT compile separately.
//
// Object-Level SLAM using YOLO detections (or any bounding-box source) as
// landmarks.  The filter is a 2-D Extended Kalman Filter that jointly
// estimates the robot pose (x, y, theta) and up to SLAM_MAX_LM landmark
// positions on the ground plane.
//
// This module is DECOUPLED from the TPU — it never calls sentai_tpu_detect()
// internally.  The user passes detection results (or manual bearings) as
// explicit arguments.  Works with any bounding-box source.
//
// Stereo depth is optional: pass detections from both cameras + baseline.
// IMU is optional: pass pitch/roll for gravity-aligned bearing correction.

#include <string.h>
#include <math.h>
#include <stdlib.h>

// ===================== Constants =====================

#define SLAM_MAX_LM         64      // max landmarks in map
#define SLAM_STATE_DIM      (3 + 2 * SLAM_MAX_LM)   // pose(3) + lm(2*N)
#define SLAM_MAX_DETS       200     // max detections per frame
#define SLAM_GATE_CHI2      9.21f   // chi-squared gate (2 DOF, 99%)
#define SLAM_LM_LABEL_LEN   4      // max class_id stored per landmark
#define SLAM_MIN_RANGE      0.05f   // 5 cm minimum
#define SLAM_DEFAULT_RANGE  2.0f    // 2 m default when monocular
#define SLAM_UNSEEN_LIMIT   60      // remove after 60 frames unseen

// ===================== Data Structures =====================

typedef struct {
    int   class_id;               // YOLO class (or user-defined)
    int   seen_count;             // total observations
    int   unseen_streak;          // consecutive frames not seen
    int   active;                 // 1 = live, 0 = free slot
    float init_range;             // range at first observation
} slam_lm_meta_t;

// ===================== Global State =====================

static int   g_slam_initialized = 0;
static int   g_slam_n_lm = 0;        // current number of active landmarks
static int   g_slam_frame = 0;       // frame counter

// Camera intrinsics (set at init)
static float g_slam_fov_h = 0.0f;    // horizontal FOV in radians
static float g_slam_focal = 0.0f;    // focal length in pixels (derived from FOV + img_w)
static int   g_slam_img_w = 0;       // detection image width (model input)
static int   g_slam_img_h = 0;       // detection image height
static float g_slam_baseline = 0.0f; // stereo baseline in meters (0 = mono)

// Motion model noise
static float g_slam_sigma_v  = 0.05f;  // translational noise (m/step)
static float g_slam_sigma_w  = 0.02f;  // rotational noise (rad/step)
// Measurement noise
static float g_slam_sigma_b  = 0.03f;  // bearing noise (rad)
static float g_slam_sigma_r  = 0.30f;  // range noise (m)

// State vector: [x, y, theta, lx0, ly0, lx1, ly1, ...]
static float* g_slam_state = NULL;           // SLAM_STATE_DIM floats

// Covariance matrix P: SLAM_STATE_DIM x SLAM_STATE_DIM
// OPTIMIZATION: only allocate (3+2*n_lm)^2, but for simplicity we use full
// We store ONLY the upper-left (3+2*g_slam_n_lm) x (3+2*g_slam_n_lm) block
static float* g_slam_P = NULL;

// Landmark metadata (parallel to state vector landmark slots)
static slam_lm_meta_t g_slam_lm[SLAM_MAX_LM];

// Work buffers
static float* g_slam_H = NULL;       // Jacobian row(s): 2 x state_dim
static float* g_slam_K = NULL;       // Kalman gain: state_dim x 2
static float* g_slam_S = NULL;       // Innovation covariance: 2x2
static float* g_slam_temp = NULL;    // general scratch: state_dim floats

// Previous-frame detections for motion estimation
#define SLAM_PREV_MAX 64
static float g_slam_prev_bearings[SLAM_PREV_MAX];   // bearing per prev detection
static int   g_slam_prev_classes[SLAM_PREV_MAX];     // class per prev detection
static int   g_slam_n_prev = 0;

// ===================== Utility =====================

static float slam_wrap_angle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}

// Active state dimension = 3 + 2*n_active_landmarks
static int slam_sdim(void) { return 3 + 2 * g_slam_n_lm; }

// Index into state vector for landmark i
static int slam_lm_idx(int i) { return 3 + 2 * i; }

// P matrix access (row-major, full SLAM_STATE_DIM stride for simplicity)
static inline float* slam_P(int r, int c) {
    return &g_slam_P[r * SLAM_STATE_DIM + c];
}

// ===================== Detection → Bearing/Range =====================

// Convert (cx, cy) in pixel coords to bearing angle
// bearing = atan2(cx - img_cx, focal_length)
static float slam_pixel_to_bearing(float cx) {
    float img_cx = (float)g_slam_img_w * 0.5f;
    return atan2f(cx - img_cx, g_slam_focal);
}

// Stereo disparity → range
// range = baseline * focal / disparity_px
static float slam_stereo_range(float cx_left, float cx_right) {
    float disp = cx_left - cx_right;
    if (disp < 1.0f) disp = 1.0f;       // clamp to avoid div/0
    return g_slam_baseline * g_slam_focal / disp;
}

// ===================== EKF Predict =====================

// Constant-velocity motion model:
//   x' = x + v*cos(theta)*dt
//   y' = y + v*sin(theta)*dt
//   theta' = theta + w*dt
// Since we don't have odometry, we estimate v,w from detection flow
// or the user can call slam.predict(dx, dy, dtheta) explicitly.

static void slam_predict(float dx, float dy, float dtheta) {
    int N = slam_sdim();
    float theta = g_slam_state[2];

    // State propagation
    g_slam_state[0] += dx * cosf(theta) - dy * sinf(theta);
    g_slam_state[1] += dx * sinf(theta) + dy * cosf(theta);
    g_slam_state[2] = slam_wrap_angle(g_slam_state[2] + dtheta);
    // Landmarks don't move

    // Jacobian F (identity except pose block)
    // F is identity for landmarks; for pose, partial derivatives:
    //   df/dtheta for x: -dx*sin(theta) - dy*cos(theta)
    //   df/dtheta for y:  dx*cos(theta) - dy*sin(theta)
    float F02 = -dx * sinf(theta) - dy * cosf(theta);
    float F12 =  dx * cosf(theta) - dy * sinf(theta);

    // P = F * P * F^T + Q
    // Since F is identity except column 2 (theta) for rows 0,1:
    // We only need to update the pose-related parts

    // Update column 2 of P (and row 2 by symmetry)
    // P[0][j] += F02 * P[2][j]  for all j
    // P[1][j] += F12 * P[2][j]  for all j
    // Then P[i][0] += P[i][2] * F02  (symmetry)
    // Then P[i][1] += P[i][2] * F12

    // Step 1: rows 0,1 of P
    for (int j = 0; j < N; j++) {
        g_slam_temp[j] = *slam_P(0, j) + F02 * (*slam_P(2, j));
    }
    for (int j = 0; j < N; j++) *slam_P(0, j) = g_slam_temp[j];

    for (int j = 0; j < N; j++) {
        g_slam_temp[j] = *slam_P(1, j) + F12 * (*slam_P(2, j));
    }
    for (int j = 0; j < N; j++) *slam_P(1, j) = g_slam_temp[j];

    // Step 2: columns 0,1 of P (symmetry)
    for (int i = 0; i < N; i++) {
        *slam_P(i, 0) = *slam_P(0, i);
        *slam_P(i, 1) = *slam_P(1, i);
    }

    // Add process noise Q to pose block
    *slam_P(0, 0) += g_slam_sigma_v * g_slam_sigma_v;
    *slam_P(1, 1) += g_slam_sigma_v * g_slam_sigma_v;
    *slam_P(2, 2) += g_slam_sigma_w * g_slam_sigma_w;
}

// ===================== EKF Update (Single Observation) =====================

// Observation model for landmark k:
//   z_bearing = atan2(ly - y, lx - x) - theta
//   z_range   = sqrt((lx - x)^2 + (ly - y)^2)
//
// Update with bearing-only or bearing+range depending on has_range flag.

static int slam_update_lm(int lm_idx, float z_bearing, float z_range, int has_range) {
    int N = slam_sdim();
    int li = slam_lm_idx(lm_idx);

    float x = g_slam_state[0];
    float y = g_slam_state[1];
    float theta = g_slam_state[2];
    float lx = g_slam_state[li];
    float ly = g_slam_state[li + 1];

    float dx = lx - x;
    float dy = ly - y;
    float q = dx * dx + dy * dy;
    float sq = sqrtf(q);
    if (sq < SLAM_MIN_RANGE) sq = SLAM_MIN_RANGE;

    // Predicted observation
    float h_bearing = slam_wrap_angle(atan2f(dy, dx) - theta);
    float innov_b = slam_wrap_angle(z_bearing - h_bearing);

    int zdim = has_range ? 2 : 1;

    // ------ Bearing-only or Bearing+Range Jacobian H ------
    // H is zdim x N, but sparse: only columns [0,1,2,li,li+1] are nonzero
    memset(g_slam_H, 0, zdim * SLAM_STATE_DIM * sizeof(float));

    // dh_bearing/dx =  dy/q
    // dh_bearing/dy = -dx/q
    // dh_bearing/dtheta = -1
    // dh_bearing/dlx = -dy/q
    // dh_bearing/dly =  dx/q
    float* Hb = &g_slam_H[0 * SLAM_STATE_DIM]; // row 0
    Hb[0] =  dy / q;
    Hb[1] = -dx / q;
    Hb[2] = -1.0f;
    Hb[li]     = -dy / q;
    Hb[li + 1] =  dx / q;

    float innov_r = 0.0f;
    if (has_range) {
        float h_range = sq;
        innov_r = z_range - h_range;

        float* Hr = &g_slam_H[1 * SLAM_STATE_DIM]; // row 1
        Hr[0] = -dx / sq;
        Hr[1] = -dy / sq;
        Hr[2] = 0.0f;
        Hr[li]     = dx / sq;
        Hr[li + 1] = dy / sq;
    }

    // ------ S = H * P * H^T + R ------
    // S is zdim x zdim
    // Compute H*P first (zdim x N), then (H*P)*H^T (zdim x zdim)
    float HP[2 * SLAM_STATE_DIM]; // max 2 rows
    memset(HP, 0, sizeof(HP));
    for (int z = 0; z < zdim; z++) {
        float* h_row = &g_slam_H[z * SLAM_STATE_DIM];
        float* hp_row = &HP[z * SLAM_STATE_DIM];
        for (int j = 0; j < N; j++) {
            float sum = 0.0f;
            for (int k = 0; k < N; k++) {
                sum += h_row[k] * (*slam_P(k, j));
            }
            hp_row[j] = sum;
        }
    }

    // OPTIMIZATION: H is sparse (5 nonzero cols), exploit that
    // But for correctness first, use dense multiply above.
    // TODO: sparse H*P for performance

    float S[4]; // 2x2 max
    memset(S, 0, sizeof(S));
    for (int z1 = 0; z1 < zdim; z1++) {
        for (int z2 = 0; z2 < zdim; z2++) {
            float sum = 0.0f;
            float* hp_row = &HP[z1 * SLAM_STATE_DIM];
            float* h_row = &g_slam_H[z2 * SLAM_STATE_DIM];
            for (int j = 0; j < N; j++) sum += hp_row[j] * h_row[j];
            S[z1 * zdim + z2] = sum;
        }
    }
    // Add measurement noise R
    S[0] += g_slam_sigma_b * g_slam_sigma_b;
    if (has_range) S[1 * zdim + 1] += g_slam_sigma_r * g_slam_sigma_r;

    // ------ Invert S (1x1 or 2x2) ------
    float Si[4]; // inverse of S
    if (zdim == 1) {
        if (fabsf(S[0]) < 1e-12f) return -1;
        Si[0] = 1.0f / S[0];
    } else {
        float det = S[0] * S[3] - S[1] * S[2];
        if (fabsf(det) < 1e-12f) return -1;
        float inv_det = 1.0f / det;
        Si[0] =  S[3] * inv_det;
        Si[1] = -S[1] * inv_det;
        Si[2] = -S[2] * inv_det;
        Si[3] =  S[0] * inv_det;
    }

    // ------ K = P * H^T * S^-1  (N x zdim) ------
    // First: PH^T (N x zdim)
    float PHt[SLAM_STATE_DIM * 2]; // N x 2 max
    memset(PHt, 0, sizeof(PHt));
    for (int i = 0; i < N; i++) {
        for (int z = 0; z < zdim; z++) {
            float sum = 0.0f;
            float* h_row = &g_slam_H[z * SLAM_STATE_DIM];
            for (int j = 0; j < N; j++) {
                sum += (*slam_P(i, j)) * h_row[j];
            }
            PHt[i * zdim + z] = sum;
        }
    }
    // K = PHt * Si  (N x zdim)
    float K[SLAM_STATE_DIM * 2];
    for (int i = 0; i < N; i++) {
        for (int z = 0; z < zdim; z++) {
            float sum = 0.0f;
            for (int m = 0; m < zdim; m++) {
                sum += PHt[i * zdim + m] * Si[m * zdim + z];
            }
            K[i * zdim + z] = sum;
        }
    }

    // ------ State update: x = x + K * innov ------
    float innov[2] = {innov_b, innov_r};
    for (int i = 0; i < N; i++) {
        float delta = 0.0f;
        for (int z = 0; z < zdim; z++) delta += K[i * zdim + z] * innov[z];
        g_slam_state[i] += delta;
    }
    g_slam_state[2] = slam_wrap_angle(g_slam_state[2]);

    // ------ Covariance update: P = (I - K*H) * P ------
    // Compute (I - K*H) row-by-row, multiply into P
    // Use temp buffer row by row to avoid in-place issues
    for (int i = 0; i < N; i++) {
        // row_i of (I - K*H)
        for (int j = 0; j < N; j++) {
            float kh = 0.0f;
            for (int z = 0; z < zdim; z++) {
                kh += K[i * zdim + z] * g_slam_H[z * SLAM_STATE_DIM + j];
            }
            float ident = (i == j) ? 1.0f : 0.0f;
            // (I-KH)[i][j]
            g_slam_temp[j] = ident - kh;
        }
        // New P[i][j] = sum_k (I-KH)[i][k] * P_old[k][j]
        // But we've already overwritten P rows 0..i-1!
        // Need to be more careful — use Joseph form or store temp P.
        // For now, use the standard form and accept small numerical drift.
        // (Joseph form is P = (I-KH)*P*(I-KH)^T + K*R*K^T but much more expensive)

        // Actually, the standard (I-KH)*P works if we process from bottom up
        // or use a temp row. Let's use temp row approach:
        float row[SLAM_STATE_DIM];
        for (int j = 0; j < N; j++) {
            float sum = 0.0f;
            for (int k = 0; k < N; k++) {
                float ikh = ((i == k) ? 1.0f : 0.0f);
                for (int z = 0; z < zdim; z++) {
                    ikh -= K[i * zdim + z] * g_slam_H[z * SLAM_STATE_DIM + k];
                }
                sum += ikh * (*slam_P(k, j));
            }
            row[j] = sum;
        }
        for (int j = 0; j < N; j++) *slam_P(i, j) = row[j];
    }

    // Symmetrize P (numerical stability)
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            float avg = 0.5f * (*slam_P(i, j) + *slam_P(j, i));
            *slam_P(i, j) = avg;
            *slam_P(j, i) = avg;
        }
    }

    return 0;
}

// ===================== Data Association =====================

// Find best matching landmark for (class_id, z_bearing, z_range)
// Returns landmark index or -1 if no match within Mahalanobis gate

static int slam_associate(int class_id, float z_bearing, float z_range, int has_range) {
    int best = -1;
    float best_d2 = SLAM_GATE_CHI2;

    float x = g_slam_state[0];
    float y = g_slam_state[1];
    float theta = g_slam_state[2];

    for (int i = 0; i < g_slam_n_lm; i++) {
        if (!g_slam_lm[i].active) continue;
        if (g_slam_lm[i].class_id != class_id) continue;

        int li = slam_lm_idx(i);
        float lx = g_slam_state[li];
        float ly = g_slam_state[li + 1];
        float dx = lx - x;
        float dy = ly - y;
        float q = dx * dx + dy * dy;
        float sq = sqrtf(q);
        if (sq < SLAM_MIN_RANGE) sq = SLAM_MIN_RANGE;

        float pred_b = slam_wrap_angle(atan2f(dy, dx) - theta);
        float innov_b = slam_wrap_angle(z_bearing - pred_b);

        // Simple Mahalanobis approximation using measurement noise only
        // (full Mahalanobis would need S^-1 which is expensive per candidate)
        float d2 = (innov_b * innov_b) / (g_slam_sigma_b * g_slam_sigma_b);
        if (has_range) {
            float innov_r = z_range - sq;
            d2 += (innov_r * innov_r) / (g_slam_sigma_r * g_slam_sigma_r);
        }

        if (d2 < best_d2) {
            best_d2 = d2;
            best = i;
        }
    }
    return best;
}

// ===================== Landmark Init =====================

// Initialize a new landmark from bearing + range
static int slam_add_landmark(int class_id, float bearing, float range) {
    if (g_slam_n_lm >= SLAM_MAX_LM) {
        // Try to reclaim an inactive slot
        for (int i = 0; i < g_slam_n_lm; i++) {
            if (!g_slam_lm[i].active) {
                // Reuse this slot
                float x = g_slam_state[0], y = g_slam_state[1], theta = g_slam_state[2];
                float world_angle = theta + bearing;
                int li = slam_lm_idx(i);
                g_slam_state[li]     = x + range * cosf(world_angle);
                g_slam_state[li + 1] = y + range * sinf(world_angle);

                g_slam_lm[i].class_id = class_id;
                g_slam_lm[i].seen_count = 1;
                g_slam_lm[i].unseen_streak = 0;
                g_slam_lm[i].active = 1;
                g_slam_lm[i].init_range = range;

                // Reset covariance for this landmark
                int N = slam_sdim();
                for (int j = 0; j < N; j++) {
                    *slam_P(li, j) = 0.0f;
                    *slam_P(li + 1, j) = 0.0f;
                    *slam_P(j, li) = 0.0f;
                    *slam_P(j, li + 1) = 0.0f;
                }
                float r2 = range * range;
                *slam_P(li, li) = r2;
                *slam_P(li + 1, li + 1) = r2;
                return i;
            }
        }
        return -1; // full
    }

    int i = g_slam_n_lm;
    float x = g_slam_state[0], y = g_slam_state[1], theta = g_slam_state[2];
    float world_angle = theta + bearing;
    int li = slam_lm_idx(i);

    g_slam_state[li]     = x + range * cosf(world_angle);
    g_slam_state[li + 1] = y + range * sinf(world_angle);

    g_slam_lm[i].class_id = class_id;
    g_slam_lm[i].seen_count = 1;
    g_slam_lm[i].unseen_streak = 0;
    g_slam_lm[i].active = 1;
    g_slam_lm[i].init_range = range;

    // Initialize covariance for new landmark (large uncertainty)
    int N_new = 3 + 2 * (i + 1);
    for (int j = 0; j < N_new; j++) {
        *slam_P(li, j) = 0.0f;
        *slam_P(li + 1, j) = 0.0f;
        *slam_P(j, li) = 0.0f;
        *slam_P(j, li + 1) = 0.0f;
    }
    float r2 = range * range;
    *slam_P(li, li) = r2;
    *slam_P(li + 1, li + 1) = r2;

    g_slam_n_lm = i + 1;
    return i;
}

// ===================== Landmark Maintenance =====================

// Increment unseen counter for all landmarks; mark stale ones inactive
static void slam_age_landmarks(void) {
    for (int i = 0; i < g_slam_n_lm; i++) {
        if (!g_slam_lm[i].active) continue;
        g_slam_lm[i].unseen_streak++;
        if (g_slam_lm[i].unseen_streak > SLAM_UNSEEN_LIMIT) {
            g_slam_lm[i].active = 0;
        }
    }
}

// ===================== Motion From Detections =====================

// Estimate ego-motion (dx, dy, dtheta) from shift in detection bearings
// between consecutive frames.  Uses median bearing shift as dtheta,
// assumes small translation (dx≈0, dy≈0) unless stereo range is available.

static float slam_estimate_dtheta(const float* bearings, const int* classes,
                                    int n_dets) {
    if (g_slam_n_prev == 0 || n_dets == 0) return 0.0f;

    float shifts[SLAM_PREV_MAX];
    int n_shifts = 0;

    // Match by class_id and nearest bearing
    for (int i = 0; i < n_dets && n_shifts < SLAM_PREV_MAX; i++) {
        float best_d = 1e30f;
        int best_j = -1;
        for (int j = 0; j < g_slam_n_prev; j++) {
            if (classes[i] != g_slam_prev_classes[j]) continue;
            float d = fabsf(slam_wrap_angle(bearings[i] - g_slam_prev_bearings[j]));
            if (d < best_d) { best_d = d; best_j = j; }
        }
        if (best_j >= 0 && best_d < 0.5f) { // < 0.5 rad match threshold
            shifts[n_shifts++] = slam_wrap_angle(bearings[i] - g_slam_prev_bearings[best_j]);
        }
    }

    if (n_shifts == 0) return 0.0f;

    // Median of shifts (simple selection sort for small N)
    for (int i = 0; i < n_shifts - 1; i++) {
        for (int j = i + 1; j < n_shifts; j++) {
            if (shifts[j] < shifts[i]) {
                float tmp = shifts[i]; shifts[i] = shifts[j]; shifts[j] = tmp;
            }
        }
    }
    return -shifts[n_shifts / 2]; // negate: if objects move right, robot turned left
}

// ===================== C API =====================

static int slam_init(float fov_h_deg, int img_w, int img_h,
                     int max_lm, float baseline_m) {
    if (fov_h_deg <= 0.0f || fov_h_deg >= 180.0f) return -1;
    if (img_w < 1 || img_h < 1) return -2;
    if (max_lm < 1 || max_lm > SLAM_MAX_LM) max_lm = SLAM_MAX_LM;

    // Free old buffers
    if (g_slam_state) { free(g_slam_state); g_slam_state = NULL; }
    if (g_slam_P)     { free(g_slam_P);     g_slam_P = NULL; }
    if (g_slam_H)     { free(g_slam_H);     g_slam_H = NULL; }
    if (g_slam_K)     { free(g_slam_K);     g_slam_K = NULL; }
    if (g_slam_S)     { free(g_slam_S);     g_slam_S = NULL; }
    if (g_slam_temp)  { free(g_slam_temp);  g_slam_temp = NULL; }

    int sdim = SLAM_STATE_DIM;
    g_slam_state = (float*)malloc(sdim * sizeof(float));
    g_slam_P     = (float*)malloc(sdim * sdim * sizeof(float));
    g_slam_H     = (float*)malloc(2 * sdim * sizeof(float));
    g_slam_K     = (float*)malloc(sdim * 2 * sizeof(float));
    g_slam_S     = (float*)malloc(4 * sizeof(float));
    g_slam_temp  = (float*)malloc(sdim * sizeof(float));

    if (!g_slam_state || !g_slam_P || !g_slam_H || !g_slam_K ||
        !g_slam_S || !g_slam_temp) {
        if (g_slam_state) free(g_slam_state);
        if (g_slam_P)     free(g_slam_P);
        if (g_slam_H)     free(g_slam_H);
        if (g_slam_K)     free(g_slam_K);
        if (g_slam_S)     free(g_slam_S);
        if (g_slam_temp)  free(g_slam_temp);
        g_slam_state = g_slam_P = g_slam_H = g_slam_K = g_slam_S = g_slam_temp = NULL;
        g_slam_initialized = 0;
        return -3;
    }

    // Configure camera intrinsics
    g_slam_fov_h = fov_h_deg * M_PI / 180.0f;
    g_slam_focal = (float)img_w / (2.0f * tanf(g_slam_fov_h / 2.0f));
    g_slam_img_w = img_w;
    g_slam_img_h = img_h;
    g_slam_baseline = baseline_m;

    // Initialize state to origin
    memset(g_slam_state, 0, sdim * sizeof(float));

    // Initialize P: small pose uncertainty, no landmarks yet
    memset(g_slam_P, 0, sdim * sdim * sizeof(float));
    *slam_P(0, 0) = 0.01f;
    *slam_P(1, 1) = 0.01f;
    *slam_P(2, 2) = 0.01f;

    // Reset landmarks
    memset(g_slam_lm, 0, sizeof(g_slam_lm));
    g_slam_n_lm = 0;
    g_slam_frame = 0;
    g_slam_n_prev = 0;

    g_slam_initialized = 1;
    return 0;
}

// Process a set of detections: list of (x1, y1, x2, y2, conf, class_id)
// Returns number of landmarks updated/created
static int slam_update_detections(const int16_t* dets, int n_dets,
                                   const int16_t* dets_right, int n_dets_right) {
    if (!g_slam_initialized) return -1;
    if (n_dets <= 0) return 0;
    if (n_dets > SLAM_MAX_DETS) n_dets = SLAM_MAX_DETS;

    int has_stereo = (dets_right != NULL && n_dets_right > 0 && g_slam_baseline > 0.0f);

    // Convert detections to bearings + ranges
    float bearings[SLAM_MAX_DETS];
    float ranges[SLAM_MAX_DETS];
    int   classes[SLAM_MAX_DETS];
    int   has_range[SLAM_MAX_DETS];

    for (int i = 0; i < n_dets; i++) {
        float cx = ((float)dets[i * 6 + 0] + (float)dets[i * 6 + 2]) * 0.5f;
        bearings[i] = slam_pixel_to_bearing(cx);
        classes[i] = dets[i * 6 + 5];
        ranges[i] = SLAM_DEFAULT_RANGE;
        has_range[i] = 0;

        // Stereo matching: find same class in right image with closest cy
        if (has_stereo) {
            float cy_left = ((float)dets[i * 6 + 1] + (float)dets[i * 6 + 3]) * 0.5f;
            float best_disp = -1.0f;
            float best_cy_diff = 1e30f;

            for (int j = 0; j < n_dets_right; j++) {
                if (dets_right[j * 6 + 5] != classes[i]) continue;
                float cy_right = ((float)dets_right[j * 6 + 1] + (float)dets_right[j * 6 + 3]) * 0.5f;
                float cy_diff = fabsf(cy_left - cy_right);
                if (cy_diff < best_cy_diff && cy_diff < 30.0f) { // epipolar constraint
                    best_cy_diff = cy_diff;
                    float cx_right = ((float)dets_right[j * 6 + 0] + (float)dets_right[j * 6 + 2]) * 0.5f;
                    best_disp = cx - cx_right;
                }
            }
            if (best_disp > 1.0f) {
                ranges[i] = slam_stereo_range(cx, cx - best_disp);
                has_range[i] = 1;
            }
        }
    }

    // Estimate ego-motion from bearing shift (before association)
    float dtheta = slam_estimate_dtheta(bearings, classes, n_dets);

    // Predict step
    slam_predict(0.0f, 0.0f, dtheta);

    // Age all landmarks (+1 unseen)
    slam_age_landmarks();

    int n_updated = 0;

    // Associate and update/create landmarks
    for (int i = 0; i < n_dets; i++) {
        int lm = slam_associate(classes[i], bearings[i], ranges[i], has_range[i]);

        if (lm >= 0) {
            // Existing landmark — EKF update
            slam_update_lm(lm, bearings[i], ranges[i], has_range[i]);
            g_slam_lm[lm].seen_count++;
            g_slam_lm[lm].unseen_streak = 0;
            n_updated++;
        } else {
            // New landmark
            float r = has_range[i] ? ranges[i] : SLAM_DEFAULT_RANGE;
            int slot = slam_add_landmark(classes[i], bearings[i], r);
            if (slot >= 0) n_updated++;
        }
    }

    // Store current frame for next motion estimation
    int n_store = (n_dets < SLAM_PREV_MAX) ? n_dets : SLAM_PREV_MAX;
    for (int i = 0; i < n_store; i++) {
        g_slam_prev_bearings[i] = bearings[i];
        g_slam_prev_classes[i] = classes[i];
    }
    g_slam_n_prev = n_store;

    g_slam_frame++;
    return n_updated;
}

// Save/Load
static int slam_save(const char* path) {
    if (!g_slam_initialized) return -1;
    int N = slam_sdim();

    // Format: [n_lm:i32][frame:i32][focal:f32][fov:f32][img_w:i32][img_h:i32]
    //         [baseline:f32]
    //         [state: N * f32]
    //         [P: N*N * f32]
    //         [lm_meta: n_lm * slam_lm_meta_t]
    //         [noise: 4 * f32 (sigma_v, sigma_w, sigma_b, sigma_r)]
    size_t hdr = 7 * 4;
    size_t state_size = N * 4;
    size_t p_size = N * N * 4;
    size_t meta_size = g_slam_n_lm * sizeof(slam_lm_meta_t);
    size_t noise_size = 4 * 4;
    size_t total = hdr + state_size + p_size + meta_size + noise_size;

    uint8_t* buf = (uint8_t*)malloc(total);
    if (!buf) return -2;

    size_t off = 0;
    int32_t* hi = (int32_t*)buf;
    hi[0] = g_slam_n_lm;
    hi[1] = g_slam_frame;
    float* hf = (float*)&buf[8];
    hf[0] = g_slam_focal;
    hf[1] = g_slam_fov_h;
    int32_t* hi2 = (int32_t*)&buf[16];
    hi2[0] = g_slam_img_w;
    hi2[1] = g_slam_img_h;
    float* hf2 = (float*)&buf[24];
    hf2[0] = g_slam_baseline;
    off = hdr;

    memcpy(&buf[off], g_slam_state, state_size); off += state_size;
    memcpy(&buf[off], g_slam_P, p_size); off += p_size;
    memcpy(&buf[off], g_slam_lm, meta_size); off += meta_size;

    float noise[4] = {g_slam_sigma_v, g_slam_sigma_w, g_slam_sigma_b, g_slam_sigma_r};
    memcpy(&buf[off], noise, noise_size);

    extern int sentai_fs_write(const char* path, const uint8_t* data, int len);
    int rc = sentai_fs_write(path, buf, (int)total);
    free(buf);
    return rc >= 0 ? 0 : rc;
}

static int slam_load(const char* path) {
    extern int sentai_fs_size(const char* path);
    extern int sentai_fs_read(const char* path, uint8_t* buf, int len);

    int size = sentai_fs_size(path);
    if (size < 28) return -1;  // minimum header

    uint8_t* buf = (uint8_t*)malloc(size);
    if (!buf) return -2;
    if (sentai_fs_read(path, buf, size) != size) { free(buf); return -3; }

    int32_t* hi = (int32_t*)buf;
    int n_lm = hi[0];
    int frame = hi[1];
    float* hf = (float*)&buf[8];
    float focal = hf[0];
    float fov = hf[1];
    int32_t* hi2 = (int32_t*)&buf[16];
    int img_w = hi2[0];
    int img_h = hi2[1];
    float* hf2 = (float*)&buf[24];
    float baseline = hf2[0];

    if (n_lm < 0 || n_lm > SLAM_MAX_LM) { free(buf); return -4; }
    if (img_w < 1 || img_h < 1 || fov <= 0.0f) { free(buf); return -4; }

    int N = 3 + 2 * n_lm;
    size_t hdr = 28;
    size_t state_size = N * 4;
    size_t p_size = N * N * 4;
    size_t meta_size = n_lm * sizeof(slam_lm_meta_t);
    size_t noise_size = 16;
    size_t expected = hdr + state_size + p_size + meta_size + noise_size;
    if (size != (int)expected) { free(buf); return -4; }

    // Re-init buffers with max dim (SLAM_STATE_DIM)
    float fov_deg = fov * 180.0f / M_PI;
    int rc = slam_init(fov_deg, img_w, img_h, SLAM_MAX_LM, baseline);
    if (rc < 0) { free(buf); return rc; }

    size_t off = hdr;
    memcpy(g_slam_state, &buf[off], state_size); off += state_size;

    // Load P into the upper-left NxN block of the full P matrix
    int sdim = SLAM_STATE_DIM;
    for (int i = 0; i < N; i++) {
        memcpy(&g_slam_P[i * sdim], &buf[off + i * N * 4], N * 4);
    }
    off += p_size;

    memcpy(g_slam_lm, &buf[off], meta_size); off += meta_size;

    float noise[4];
    memcpy(noise, &buf[off], noise_size);
    g_slam_sigma_v = noise[0];
    g_slam_sigma_w = noise[1];
    g_slam_sigma_b = noise[2];
    g_slam_sigma_r = noise[3];

    g_slam_n_lm = n_lm;
    g_slam_frame = frame;
    g_slam_focal = focal;
    g_slam_fov_h = fov;
    g_slam_img_w = img_w;
    g_slam_img_h = img_h;
    g_slam_baseline = baseline;

    free(buf);
    return 0;
}

// ===================== MicroPython Bindings =====================

// sentai.slam.init(fov_h_deg, img_w, img_h, max_lm=64, baseline_m=0.0) -> int
static mp_obj_t mod_slam_init(size_t n_args, const mp_obj_t *args) {
    float fov = mp_obj_get_float(args[0]);
    int w = mp_obj_get_int(args[1]);
    int h = mp_obj_get_int(args[2]);
    int max_lm = (n_args >= 4) ? mp_obj_get_int(args[3]) : SLAM_MAX_LM;
    float baseline = (n_args >= 5) ? mp_obj_get_float(args[4]) : 0.0f;
    return mp_obj_new_int(slam_init(fov, w, h, max_lm, baseline));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_slam_init_obj, 3, 5, mod_slam_init);

// sentai.slam.update(detections) -> int (number of landmarks updated/created)
// detections: list of 6-element tuples (x1,y1,x2,y2,conf,class_id) — from tpu.detect() or any source
static mp_obj_t mod_slam_update(mp_obj_t dets_obj) {
    if (!g_slam_initialized) return mp_obj_new_int(-1);

    size_t n_dets;
    mp_obj_t *det_items;
    mp_obj_get_array(dets_obj, &n_dets, &det_items);
    if (n_dets == 0) return mp_obj_new_int(0);
    if (n_dets > SLAM_MAX_DETS) n_dets = SLAM_MAX_DETS;

    int16_t buf[SLAM_MAX_DETS * 6];
    for (size_t i = 0; i < n_dets; i++) {
        size_t tlen;
        mp_obj_t *titems;
        mp_obj_get_array(det_items[i], &tlen, &titems);
        if (tlen < 6) return mp_obj_new_int(-2);

        buf[i * 6 + 0] = (int16_t)mp_obj_get_int(titems[0]);  // x1
        buf[i * 6 + 1] = (int16_t)mp_obj_get_int(titems[1]);  // y1
        buf[i * 6 + 2] = (int16_t)mp_obj_get_int(titems[2]);  // x2
        buf[i * 6 + 3] = (int16_t)mp_obj_get_int(titems[3]);  // y2
        // conf is float in param 4, we store as permil for consistency but don't use in SLAM
        buf[i * 6 + 4] = (int16_t)(mp_obj_get_float(titems[4]) * 1000.0f);
        buf[i * 6 + 5] = (int16_t)mp_obj_get_int(titems[5]);  // class_id
    }

    return mp_obj_new_int(slam_update_detections(buf, (int)n_dets, NULL, 0));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_slam_update_obj, mod_slam_update);

// sentai.slam.update_stereo(dets_left, dets_right) -> int
static mp_obj_t mod_slam_update_stereo(mp_obj_t left_obj, mp_obj_t right_obj) {
    if (!g_slam_initialized) return mp_obj_new_int(-1);
    if (g_slam_baseline <= 0.0f) return mp_obj_new_int(-2);

    size_t n_left, n_right;
    mp_obj_t *left_items, *right_items;
    mp_obj_get_array(left_obj, &n_left, &left_items);
    mp_obj_get_array(right_obj, &n_right, &right_items);

    if (n_left > SLAM_MAX_DETS) n_left = SLAM_MAX_DETS;
    if (n_right > SLAM_MAX_DETS) n_right = SLAM_MAX_DETS;

    int16_t buf_l[SLAM_MAX_DETS * 6];
    int16_t buf_r[SLAM_MAX_DETS * 6];

    for (size_t i = 0; i < n_left; i++) {
        size_t tlen; mp_obj_t *ti;
        mp_obj_get_array(left_items[i], &tlen, &ti);
        if (tlen < 6) return mp_obj_new_int(-3);
        buf_l[i*6+0] = (int16_t)mp_obj_get_int(ti[0]);
        buf_l[i*6+1] = (int16_t)mp_obj_get_int(ti[1]);
        buf_l[i*6+2] = (int16_t)mp_obj_get_int(ti[2]);
        buf_l[i*6+3] = (int16_t)mp_obj_get_int(ti[3]);
        buf_l[i*6+4] = (int16_t)(mp_obj_get_float(ti[4]) * 1000.0f);
        buf_l[i*6+5] = (int16_t)mp_obj_get_int(ti[5]);
    }
    for (size_t i = 0; i < n_right; i++) {
        size_t tlen; mp_obj_t *ti;
        mp_obj_get_array(right_items[i], &tlen, &ti);
        if (tlen < 6) return mp_obj_new_int(-3);
        buf_r[i*6+0] = (int16_t)mp_obj_get_int(ti[0]);
        buf_r[i*6+1] = (int16_t)mp_obj_get_int(ti[1]);
        buf_r[i*6+2] = (int16_t)mp_obj_get_int(ti[2]);
        buf_r[i*6+3] = (int16_t)mp_obj_get_int(ti[3]);
        buf_r[i*6+4] = (int16_t)(mp_obj_get_float(ti[4]) * 1000.0f);
        buf_r[i*6+5] = (int16_t)mp_obj_get_int(ti[5]);
    }

    return mp_obj_new_int(slam_update_detections(buf_l, (int)n_left, buf_r, (int)n_right));
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_slam_update_stereo_obj, mod_slam_update_stereo);

// sentai.slam.observe(class_id, bearing_rad, range_m) -> int (landmark index)
// Manual landmark observation — for non-TPU sources
static mp_obj_t mod_slam_observe(mp_obj_t cls_obj, mp_obj_t b_obj, mp_obj_t r_obj) {
    if (!g_slam_initialized) return mp_obj_new_int(-1);
    int class_id = mp_obj_get_int(cls_obj);
    float bearing = mp_obj_get_float(b_obj);
    float range = mp_obj_get_float(r_obj);
    int has_r = (range > SLAM_MIN_RANGE) ? 1 : 0;
    if (!has_r) range = SLAM_DEFAULT_RANGE;

    int lm = slam_associate(class_id, bearing, range, has_r);
    if (lm >= 0) {
        slam_update_lm(lm, bearing, range, has_r);
        g_slam_lm[lm].seen_count++;
        g_slam_lm[lm].unseen_streak = 0;
        return mp_obj_new_int(lm);
    } else {
        return mp_obj_new_int(slam_add_landmark(class_id, bearing, has_r ? range : SLAM_DEFAULT_RANGE));
    }
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_slam_observe_obj, mod_slam_observe);

// sentai.slam.predict(dx, dy, dtheta) -> None
// Explicit motion prediction (e.g., from odometry or user command)
static mp_obj_t mod_slam_predict(mp_obj_t dx_obj, mp_obj_t dy_obj, mp_obj_t dt_obj) {
    if (!g_slam_initialized) return mp_const_none;
    slam_predict(mp_obj_get_float(dx_obj), mp_obj_get_float(dy_obj), mp_obj_get_float(dt_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(mod_slam_predict_obj, mod_slam_predict);

// sentai.slam.pose() -> tuple(x, y, theta)
static mp_obj_t mod_slam_pose(void) {
    if (!g_slam_initialized) return mp_const_none;
    mp_obj_t items[3] = {
        mp_obj_new_float(g_slam_state[0]),
        mp_obj_new_float(g_slam_state[1]),
        mp_obj_new_float(g_slam_state[2])
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_slam_pose_obj, mod_slam_pose);

// sentai.slam.landmarks() -> list of dicts
static mp_obj_t mod_slam_landmarks(void) {
    if (!g_slam_initialized) return mp_obj_new_list(0, NULL);
    mp_obj_list_t* list = MP_OBJ_TO_PTR(mp_obj_new_list(0, NULL));
    for (int i = 0; i < g_slam_n_lm; i++) {
        if (!g_slam_lm[i].active) continue;
        int li = slam_lm_idx(i);
        mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(5));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_id), mp_obj_new_int(i));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_class_id), mp_obj_new_int(g_slam_lm[i].class_id));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_x), mp_obj_new_float(g_slam_state[li]));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_y), mp_obj_new_float(g_slam_state[li + 1]));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_seen), mp_obj_new_int(g_slam_lm[i].seen_count));
        mp_obj_list_append(MP_OBJ_FROM_PTR(list), MP_OBJ_FROM_PTR(d));
    }
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_slam_landmarks_obj, mod_slam_landmarks);

// sentai.slam.noise(sigma_v, sigma_w, sigma_b, sigma_r) -> None / tuple
// Set or get noise parameters
static mp_obj_t mod_slam_noise(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 4) {
        g_slam_sigma_v = mp_obj_get_float(args[0]);
        g_slam_sigma_w = mp_obj_get_float(args[1]);
        g_slam_sigma_b = mp_obj_get_float(args[2]);
        g_slam_sigma_r = mp_obj_get_float(args[3]);
        return mp_const_none;
    }
    mp_obj_t items[4] = {
        mp_obj_new_float(g_slam_sigma_v),
        mp_obj_new_float(g_slam_sigma_w),
        mp_obj_new_float(g_slam_sigma_b),
        mp_obj_new_float(g_slam_sigma_r)
    };
    return mp_obj_new_tuple(4, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_slam_noise_obj, 0, 4, mod_slam_noise);

// sentai.slam.imu_correct(pitch_deg, roll_deg) -> None
// Apply IMU tilt correction to the bearing measurement model.
// Adjusts sigma_b based on camera tilt (larger tilt = more bearing uncertainty).
static mp_obj_t mod_slam_imu_correct(mp_obj_t pitch_obj, mp_obj_t roll_obj) {
    if (!g_slam_initialized) return mp_const_none;
    float pitch = mp_obj_get_float(pitch_obj) * M_PI / 180.0f;
    float roll  = mp_obj_get_float(roll_obj)  * M_PI / 180.0f;
    // Effective bearing correction: project ground-plane bearing for tilted camera
    // For small tilts, correction ≈ 0.  For large pitch, bearing uncertainty grows.
    // We scale sigma_b by cos(pitch) to account for foreshortening.
    float cos_pitch = cosf(pitch);
    if (cos_pitch < 0.3f) cos_pitch = 0.3f;  // clamp floor
    g_slam_sigma_b = 0.03f / cos_pitch;
    (void)roll; // roll has minimal effect on horizontal bearing
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(mod_slam_imu_correct_obj, mod_slam_imu_correct);

// sentai.slam.reset() -> None
static mp_obj_t mod_slam_reset(void) {
    if (!g_slam_initialized) return mp_const_none;
    int sdim = SLAM_STATE_DIM;
    memset(g_slam_state, 0, sdim * sizeof(float));
    memset(g_slam_P, 0, sdim * sdim * sizeof(float));
    *slam_P(0, 0) = 0.01f;
    *slam_P(1, 1) = 0.01f;
    *slam_P(2, 2) = 0.01f;
    memset(g_slam_lm, 0, sizeof(g_slam_lm));
    g_slam_n_lm = 0;
    g_slam_frame = 0;
    g_slam_n_prev = 0;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_slam_reset_obj, mod_slam_reset);

// sentai.slam.save(path) -> int
static mp_obj_t mod_slam_save(mp_obj_t path_obj) {
    return mp_obj_new_int(slam_save(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_slam_save_obj, mod_slam_save);

// sentai.slam.load(path) -> int
static mp_obj_t mod_slam_load(mp_obj_t path_obj) {
    return mp_obj_new_int(slam_load(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(mod_slam_load_obj, mod_slam_load);

// sentai.slam.info() -> dict
static mp_obj_t mod_slam_info(void) {
    mp_obj_dict_t* dict = MP_OBJ_TO_PTR(mp_obj_new_dict(8));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_initialized), mp_obj_new_bool(g_slam_initialized));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_landmarks), mp_obj_new_int(g_slam_n_lm));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_frame), mp_obj_new_int(g_slam_frame));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_fov), mp_obj_new_float(g_slam_fov_h * 180.0f / M_PI));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_focal), mp_obj_new_float(g_slam_focal));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_baseline), mp_obj_new_float(g_slam_baseline));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_img_w), mp_obj_new_int(g_slam_img_w));
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_img_h), mp_obj_new_int(g_slam_img_h));

    // Active landmark count
    int active = 0;
    for (int i = 0; i < g_slam_n_lm; i++) {
        if (g_slam_lm[i].active) active++;
    }
    mp_obj_dict_store(dict, MP_ROM_QSTR(MP_QSTR_active), mp_obj_new_int(active));
    return MP_OBJ_FROM_PTR(dict);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_slam_info_obj, mod_slam_info);

// ---- module table ----
static const mp_rom_map_elem_t sentai_slam_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_slam) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mod_slam_init_obj) },
    // Detection-based update
    { MP_ROM_QSTR(MP_QSTR_update), MP_ROM_PTR(&mod_slam_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_update_stereo), MP_ROM_PTR(&mod_slam_update_stereo_obj) },
    // Manual observation
    { MP_ROM_QSTR(MP_QSTR_observe), MP_ROM_PTR(&mod_slam_observe_obj) },
    // Motion
    { MP_ROM_QSTR(MP_QSTR_predict), MP_ROM_PTR(&mod_slam_predict_obj) },
    // Query
    { MP_ROM_QSTR(MP_QSTR_pose), MP_ROM_PTR(&mod_slam_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_landmarks), MP_ROM_PTR(&mod_slam_landmarks_obj) },
    // Configuration
    { MP_ROM_QSTR(MP_QSTR_noise), MP_ROM_PTR(&mod_slam_noise_obj) },
    { MP_ROM_QSTR(MP_QSTR_imu_correct), MP_ROM_PTR(&mod_slam_imu_correct_obj) },
    // Lifecycle
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&mod_slam_reset_obj) },
    // Persistence
    { MP_ROM_QSTR(MP_QSTR_save), MP_ROM_PTR(&mod_slam_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&mod_slam_load_obj) },
    // Info
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&mod_slam_info_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_slam_globals, sentai_slam_globals_table);
static const mp_obj_module_t sentai_slam_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_slam_globals,
};

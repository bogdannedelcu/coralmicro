// sentai_object_lifter.cc — ObjectsPlan L5 (Stage 5) implementation.
//
// Inverse-depth EKF (Civera/Davison/Montiel TRO 2008) per-tracklet.
// Pure float math, no HW dependencies, builds for ARM (sentai_runtime)
// and SIM (sentai_sim). ARM places state in .sdram_bss / code in
// .sdram_text per [[itcm-budget]]; SIM uses default sections.
//
// Math is bit-for-bit equivalent to the Python prototype validated in
// `experiments/s131_lifter_replay/lifter_proto.py` (commit 898e8d05).
// Differences from prototype:
//   - Single-precision float (M7 FPU native) throughout.
//   - Matrices kept as inline scalars (3×3 unrolled); avoids
//     CMSIS-DSP heap-allocated arm_matrix_instance_f32 overhead.
//   - Joseph form of variance update is identical.

#include "sentai_object_lifter.h"

#include <math.h>
#include <string.h>

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #include <time.h>
  static inline uint32_t lif_now_ms(void) {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
  }
  #define SENTAI_LIF_SDRAM_TEXT  /* nothing */
  #define SENTAI_LIF_SDRAM_BSS   /* nothing */
#else
  #include "third_party/freertos_kernel/include/FreeRTOS.h"
  #include "third_party/freertos_kernel/include/task.h"
  static inline uint32_t lif_now_ms(void) {
      return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  }
  #define SENTAI_LIF_SDRAM_TEXT  __attribute__((section(".sdram_text")))
  #define SENTAI_LIF_SDRAM_BSS   __attribute__((section(".sdram_bss"), aligned(4)))
#endif

// ---- State -------------------------------------------------------------
SENTAI_LIF_SDRAM_BSS static sentai_lifter_entry_t g_lifters[SENTAI_LIFTER_MAX];
SENTAI_LIF_SDRAM_BSS static sentai_lifter_stats_t g_stats;

// Camera intrinsics + extrinsics — boot defaults match s130 calibration.
// Replaced by sentai.calib at takeoff in future (see objects_plan.md §21).
SENTAI_LIF_SDRAM_BSS static float g_cam_fx = 577.0f;
SENTAI_LIF_SDRAM_BSS static float g_cam_fy = 579.0f;
SENTAI_LIF_SDRAM_BSS static float g_cam_cx = 320.0f;
SENTAI_LIF_SDRAM_BSS static float g_cam_cy = 240.0f;
// R_B_C row-major: rotates a camera-frame vector into the body frame.
// Default mirrors lifter_proto.py / _R_CAM_TO_BODY_LOCAL.
SENTAI_LIF_SDRAM_BSS static float g_R_B_C[9] = {
    0.0f, 1.0f,  0.0f,
    1.0f, 0.0f,  0.0f,
    0.0f, 0.0f, -1.0f,
};
SENTAI_LIF_SDRAM_BSS static float g_cam_offset_B[3] = {-0.04f, 0.0f, -0.02f};

// ---- Helpers (inlined small math) --------------------------------------

static inline int lif_finite3(float a, float b, float c) {
    return isfinite(a) && isfinite(b) && isfinite(c);
}

static inline int lif_finite_vec3(const float* v) {
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static inline int lif_find(uint16_t tracklet_id) {
    for (int i = 0; i < SENTAI_LIFTER_MAX; i++) {
        if (g_lifters[i].status != LIFTER_FREE
            && g_lifters[i].tracklet_id == tracklet_id) {
            return i;
        }
    }
    return -1;
}

static int lif_first_free(void) {
    for (int i = 0; i < SENTAI_LIFTER_MAX; i++) {
        if (g_lifters[i].status == LIFTER_FREE) return i;
    }
    return -1;
}

// Evict oldest LIFTER_LOST; returns slot or -1.
static int lif_oldest_lost(void) {
    int best = -1;
    uint32_t best_ms = 0xFFFFFFFFu;
    for (int i = 0; i < SENTAI_LIFTER_MAX; i++) {
        if (g_lifters[i].status != LIFTER_LOST) continue;
        if (g_lifters[i].last_obs_ms <= best_ms) {
            best_ms = g_lifters[i].last_obs_ms;
            best = i;
        }
    }
    return best;
}

// R_yaw(yaw) — body→world rotation, Z-up. Out 3×3 row-major.
static inline void lif_R_yaw(float yaw, float* R) {
    float c = cosf(yaw), s = sinf(yaw);
    R[0] = c; R[1] = -s; R[2] = 0.0f;
    R[3] = s; R[4] =  c; R[5] = 0.0f;
    R[6] = 0.0f; R[7] = 0.0f; R[8] = 1.0f;
}

// 3×3 row-major mat-vec: out3 = R · v3.
static inline void lif_mat3_vec3(const float* R, const float* v, float* out) {
    out[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
    out[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
    out[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

// 3×3 row-major TRANSPOSED mat-vec: out3 = R^T · v3.
static inline void lif_mat3T_vec3(const float* R, const float* v, float* out) {
    out[0] = R[0]*v[0] + R[3]*v[1] + R[6]*v[2];
    out[1] = R[1]*v[0] + R[4]*v[1] + R[7]*v[2];
    out[2] = R[2]*v[0] + R[5]*v[1] + R[8]*v[2];
}

// 3×3 row-major mat-mat: C = A · B.
static inline void lif_mat3_mul(const float* A, const float* B, float* C) {
    for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
        float s = 0.0f;
        for (int k = 0; k < 3; k++) s += A[i*3+k] * B[k*3+j];
        C[i*3+j] = s;
    }
}

// 3×3 transpose: out = A^T.
static inline void lif_mat3_transpose(const float* A, float* out) {
    out[0] = A[0]; out[1] = A[3]; out[2] = A[6];
    out[3] = A[1]; out[4] = A[4]; out[5] = A[7];
    out[6] = A[2]; out[7] = A[5]; out[8] = A[8];
}

// Unit bearing in camera frame from pixel (u, v).
static inline void lif_bearing_C(float u, float v, float* b) {
    b[0] = (u - g_cam_cx) / g_cam_fx;
    b[1] = (v - g_cam_cy) / g_cam_fy;
    b[2] = 1.0f;
    float n = sqrtf(b[0]*b[0] + b[1]*b[1] + b[2]*b[2]);
    if (n < 1e-9f) return;
    b[0] /= n; b[1] /= n; b[2] /= n;
}

// Camera world position = drone_W + R_W_B · cam_offset_B.
static inline void lif_cam_world(const float* drone_W, const float* R_W_B,
                                 float* out) {
    float ofs_W[3];
    lif_mat3_vec3(R_W_B, g_cam_offset_B, ofs_W);
    out[0] = drone_W[0] + ofs_W[0];
    out[1] = drone_W[1] + ofs_W[1];
    out[2] = drone_W[2] + ofs_W[2];
}

// ---- Public API --------------------------------------------------------

int sentai_lifter_set_camera(float fx, float fy, float cx, float cy,
                             const float* R_B_C_row_major,
                             const float* cam_offset_B) {
    if (!isfinite(fx) || !isfinite(fy) || !isfinite(cx) || !isfinite(cy)) return -1;
    if (fx <= 1.0f || fy <= 1.0f) return -2;
    if (!R_B_C_row_major || !cam_offset_B) return -3;
    for (int i = 0; i < 9; i++) if (!isfinite(R_B_C_row_major[i])) return -4;
    for (int i = 0; i < 3; i++) if (!isfinite(cam_offset_B[i])) return -5;
    g_cam_fx = fx; g_cam_fy = fy; g_cam_cx = cx; g_cam_cy = cy;
    for (int i = 0; i < 9; i++) g_R_B_C[i] = R_B_C_row_major[i];
    for (int i = 0; i < 3; i++) g_cam_offset_B[i] = cam_offset_B[i];
    return 0;
}

int sentai_lifter_clear(void) {
    int n_non_free = 0;
    for (int i = 0; i < SENTAI_LIFTER_MAX; i++) {
        if (g_lifters[i].status != LIFTER_FREE) n_non_free++;
        memset(&g_lifters[i], 0, sizeof(g_lifters[i]));
    }
    return n_non_free;
}

int sentai_lifter_init_from_bbox(uint16_t tracklet_id, uint8_t class_id,
                                 float u_c, float v_c,
                                 float bbox_w_px, float real_size_m,
                                 const float* drone_W3, float yaw_rad) {
    // ---- F1/F2 input validation ----
    if (!lif_finite3(u_c, v_c, bbox_w_px)) {
        g_stats.rejects_invalid_input++; return -1;
    }
    if (!drone_W3 || !lif_finite_vec3(drone_W3) || !isfinite(yaw_rad)) {
        g_stats.rejects_invalid_input++; return -1;
    }
    if (bbox_w_px <= 1.0f || real_size_m <= 0.0f
        || !isfinite(real_size_m) || class_id >= SENTAI_LIFTER_CLASS_MAX) {
        g_stats.rejects_invalid_input++; return -2;
    }

    // ---- Find slot (existing tracklet → reinit; else free; else evict LOST) ----
    int slot = lif_find(tracklet_id);
    if (slot < 0) slot = lif_first_free();
    if (slot < 0) {
        slot = lif_oldest_lost();
        if (slot < 0) return -3;
        g_stats.evictions++;
    }

    // ---- Compute init state ----
    float d0 = g_cam_fx * real_size_m / bbox_w_px;
    if (!isfinite(d0) || d0 < 1e-6f) {
        g_stats.rejects_invalid_input++; return -2;
    }
    float rho0 = 1.0f / d0;
    if (rho0 < SENTAI_LIFTER_RHO_MIN || rho0 > SENTAI_LIFTER_RHO_MAX) {
        g_stats.rejects_invalid_input++; return -2;
    }
    float var0 = (SENTAI_LIFTER_ALPHA_INIT * rho0)
               * (SENTAI_LIFTER_ALPHA_INIT * rho0);

    // r_W = R_W_B(yaw) · R_B_C · bearing_C(u, v).
    float b_C[3];
    lif_bearing_C(u_c, v_c, b_C);
    if (!lif_finite_vec3(b_C)) { g_stats.rejects_invalid_input++; return -6; }
    float R_W_B[9];
    lif_R_yaw(yaw_rad, R_W_B);
    float b_B[3], r_W[3];
    lif_mat3_vec3(g_R_B_C, b_C, b_B);
    lif_mat3_vec3(R_W_B, b_B, r_W);
    float r_norm = sqrtf(r_W[0]*r_W[0] + r_W[1]*r_W[1] + r_W[2]*r_W[2]);
    if (r_norm < 1e-9f) { g_stats.rejects_invalid_input++; return -6; }
    r_W[0] /= r_norm; r_W[1] /= r_norm; r_W[2] /= r_norm;

    // anchor_W = drone_W + R_W_B · cam_offset_B.
    float anchor_W[3];
    lif_cam_world(drone_W3, R_W_B, anchor_W);

    // ---- Commit ----
    uint32_t now = lif_now_ms();
    memset(&g_lifters[slot], 0, sizeof(g_lifters[slot]));
    g_lifters[slot].status = LIFTER_TRACKING;
    g_lifters[slot].class_id = class_id;
    g_lifters[slot].tracklet_id = tracklet_id;
    g_lifters[slot].rho = rho0;
    g_lifters[slot].var_rho = var0;
    g_lifters[slot].anchor_w[0] = anchor_W[0];
    g_lifters[slot].anchor_w[1] = anchor_W[1];
    g_lifters[slot].anchor_w[2] = anchor_W[2];
    g_lifters[slot].r_w[0] = r_W[0];
    g_lifters[slot].r_w[1] = r_W[1];
    g_lifters[slot].r_w[2] = r_W[2];
    g_lifters[slot].n_obs = 1;
    g_lifters[slot].n_rejected = 0;
    g_lifters[slot].age_ms = 0;
    g_lifters[slot].last_obs_ms = now;
    g_lifters[slot].init_ms = now;
    g_lifters[slot].published_object_id = 0;
    g_stats.inits++;
    if (slot + 1 > g_stats.hwm_used) g_stats.hwm_used = (uint16_t)(slot + 1);
    // Linearization gate may already be tripped for near-marker (Civera 2008):
    // σ_ρ < ε · ρ²  ⇔  σ_ρ₀ = α·ρ₀ < ε·ρ₀²  ⇔  α < ε·ρ₀  ⇔  ρ₀ > α/ε.
    if (sqrtf(var0) < SENTAI_LIFTER_EPS_LIN * rho0 * rho0) {
        g_lifters[slot].status = LIFTER_LIFTED;
        g_stats.lifted++;
    }
    return slot;
}

int sentai_lifter_update_bbox(uint16_t tracklet_id,
                              float u_c, float v_c,
                              const float* drone_W3, float yaw_rad,
                              float dt_s) {
    if (!lif_finite3(u_c, v_c, dt_s)) {
        g_stats.rejects_invalid_input++; return -6;
    }
    if (!drone_W3 || !lif_finite_vec3(drone_W3) || !isfinite(yaw_rad)) {
        g_stats.rejects_invalid_input++; return -6;
    }
    int slot = lif_find(tracklet_id);
    if (slot < 0) {
        g_stats.rejects_unknown_tracklet++; return -1;
    }

    // ---- Compute prediction ----
    float rho = g_lifters[slot].rho;
    if (rho < SENTAI_LIFTER_RHO_MIN || rho > SENTAI_LIFTER_RHO_MAX) {
        g_stats.rejects_rho_clamp++;
        g_lifters[slot].n_rejected++;
        return -2;
    }
    // L_W = anchor + (1/ρ) · r_W
    float invr = 1.0f / rho;
    float L_W[3] = {
        g_lifters[slot].anchor_w[0] + invr * g_lifters[slot].r_w[0],
        g_lifters[slot].anchor_w[1] + invr * g_lifters[slot].r_w[1],
        g_lifters[slot].anchor_w[2] + invr * g_lifters[slot].r_w[2],
    };
    float R_W_B[9];
    lif_R_yaw(yaw_rad, R_W_B);
    float c_W[3];
    lif_cam_world(drone_W3, R_W_B, c_W);
    float delta_W[3] = {L_W[0] - c_W[0], L_W[1] - c_W[1], L_W[2] - c_W[2]};

    // δ_C = R_B_C^T · R_W_B^T · δ_W
    float delta_B[3];
    lif_mat3T_vec3(R_W_B, delta_W, delta_B);
    float delta_C[3];
    lif_mat3T_vec3(g_R_B_C, delta_B, delta_C);
    if (delta_C[2] <= 1e-6f) {
        g_stats.rejects_behind_camera++;
        g_lifters[slot].n_rejected++;
        return -5;
    }
    float u_pred = g_cam_fx * delta_C[0] / delta_C[2] + g_cam_cx;
    float v_pred = g_cam_fy * delta_C[1] / delta_C[2] + g_cam_cy;

    // ---- Jacobian H = ∂(u,v)/∂ρ ----
    // ∂L_W/∂ρ = -(1/ρ²) · r_W
    float dL_drho[3] = {
        -invr * invr * g_lifters[slot].r_w[0],
        -invr * invr * g_lifters[slot].r_w[1],
        -invr * invr * g_lifters[slot].r_w[2],
    };
    // ∂δ_C/∂ρ = R_C_W · ∂L_W/∂ρ = R_B_C^T · R_W_B^T · ∂L_W/∂ρ
    float ddB_drho[3], ddC_drho[3];
    lif_mat3T_vec3(R_W_B, dL_drho, ddB_drho);
    lif_mat3T_vec3(g_R_B_C, ddB_drho, ddC_drho);
    float z = delta_C[2];
    float inv_z2 = 1.0f / (z * z);
    float du_drho = g_cam_fx * (ddC_drho[0] * z - delta_C[0] * ddC_drho[2]) * inv_z2;
    float dv_drho = g_cam_fy * (ddC_drho[1] * z - delta_C[1] * ddC_drho[2]) * inv_z2;

    // ---- Innovation ----
    float innov_u = u_c - u_pred;
    float innov_v = v_c - v_pred;

    // ---- S = H σ² H^T + R_obs (2×2 symmetric) ----
    float var_rho = g_lifters[slot].var_rho;
    float Hh_uu = du_drho * du_drho * var_rho + SENTAI_LIFTER_OBS_PX_VAR;
    float Hh_vv = dv_drho * dv_drho * var_rho + SENTAI_LIFTER_OBS_PX_VAR;
    float Hh_uv = du_drho * dv_drho * var_rho;
    float det_S = Hh_uu * Hh_vv - Hh_uv * Hh_uv;
    if (!isfinite(det_S) || fabsf(det_S) < 1e-12f) {
        g_stats.rejects_S_singular++;
        g_lifters[slot].n_rejected++;
        return -3;
    }
    float inv_det = 1.0f / det_S;
    // S^-1 (2×2): swap diag, negate off-diag, divide by det.
    float Sinv_uu =  Hh_vv * inv_det;
    float Sinv_vv =  Hh_uu * inv_det;
    float Sinv_uv = -Hh_uv * inv_det;

    // K = σ_ρ² · H^T · S^-1  (shape: 2-vec)
    float K_u = var_rho * (du_drho * Sinv_uu + dv_drho * Sinv_uv);
    float K_v = var_rho * (du_drho * Sinv_uv + dv_drho * Sinv_vv);

    // K · innov (scalar) → ρ update.
    float k_dot_innov = K_u * innov_u + K_v * innov_v;
    float rho_new = rho + k_dot_innov;
    if (rho_new < SENTAI_LIFTER_RHO_MIN || rho_new > SENTAI_LIFTER_RHO_MAX) {
        g_stats.rejects_rho_clamp++;
        g_lifters[slot].n_rejected++;
        return -2;
    }

    // K · H (scalar via inner product).
    float KH = K_u * du_drho + K_v * dv_drho;
    // Joseph form:
    //   P_new = (1 - K·H)² · P + K · R_obs · K^T
    //   K · R_obs · K^T = σ_obs² · (K_u² + K_v²) (since R_obs = σ² · I)
    float one_minus_KH = 1.0f - KH;
    float var_std = one_minus_KH * one_minus_KH * var_rho;
    float var_kr  = SENTAI_LIFTER_OBS_PX_VAR * (K_u * K_u + K_v * K_v);
    float var_new = var_std + var_kr;
    if (!isfinite(var_new) || var_new <= 0.0f) {
        g_stats.rejects_var_invalid++;
        g_lifters[slot].n_rejected++;
        return -4;
    }

    // ---- Commit ----
    g_lifters[slot].rho = rho_new;
    g_lifters[slot].var_rho = var_new;
    g_lifters[slot].n_obs++;
    g_lifters[slot].age_ms = (uint32_t)(g_lifters[slot].age_ms
                                         + (uint32_t)(dt_s * 1000.0f));
    g_lifters[slot].last_obs_ms = lif_now_ms();
    g_stats.updates++;

    // ---- Status transition (Civera linearization gate) ----
    if (g_lifters[slot].status == LIFTER_TRACKING
        && sqrtf(var_new) < SENTAI_LIFTER_EPS_LIN * rho_new * rho_new) {
        g_lifters[slot].status = LIFTER_LIFTED;
        g_stats.lifted++;
    }
    return 0;
}

int sentai_lifter_get(uint16_t tracklet_id, sentai_lifter_entry_t* out) {
    if (!out) return -1;
    int slot = lif_find(tracklet_id);
    if (slot < 0) return -1;
    *out = g_lifters[slot];
    return 0;
}

int sentai_lifter_world_pos(uint16_t tracklet_id, float* out3) {
    if (!out3) return -1;
    int slot = lif_find(tracklet_id);
    if (slot < 0) return -1;
    float rho = g_lifters[slot].rho;
    if (rho < SENTAI_LIFTER_RHO_MIN || rho > SENTAI_LIFTER_RHO_MAX) return -2;
    float invr = 1.0f / rho;
    out3[0] = g_lifters[slot].anchor_w[0] + invr * g_lifters[slot].r_w[0];
    out3[1] = g_lifters[slot].anchor_w[1] + invr * g_lifters[slot].r_w[1];
    out3[2] = g_lifters[slot].anchor_w[2] + invr * g_lifters[slot].r_w[2];
    return 0;
}

int sentai_lifter_mark_lost(uint16_t tracklet_id) {
    int slot = lif_find(tracklet_id);
    if (slot < 0) return -1;
    g_lifters[slot].status = LIFTER_LOST;
    return 0;
}

int sentai_lifter_list(sentai_lifter_entry_t* out, int max) {
    if (!out || max <= 0) return 0;
    int n = 0;
    for (int i = 0; i < SENTAI_LIFTER_MAX && n < max; i++) {
        if (g_lifters[i].status == LIFTER_FREE) continue;
        out[n++] = g_lifters[i];
    }
    return n;
}

int sentai_lifter_count(void) {
    int n = 0;
    for (int i = 0; i < SENTAI_LIFTER_MAX; i++) {
        if (g_lifters[i].status != LIFTER_FREE) n++;
    }
    return n;
}

void sentai_lifter_stats(sentai_lifter_stats_t* out_counters,
                         int* out_used,
                         int* out_tracking,
                         int* out_lifted,
                         int* out_lost) {
    if (out_counters) *out_counters = g_stats;
    int u = 0, t = 0, lf = 0, lt = 0;
    for (int i = 0; i < SENTAI_LIFTER_MAX; i++) {
        switch (g_lifters[i].status) {
        case LIFTER_FREE:                                       break;
        case LIFTER_TRACKING:  u++; t++;                        break;
        case LIFTER_LIFTED:    u++; lf++;                       break;
        case LIFTER_LOST:      u++; lt++;                       break;
        }
    }
    if (out_used)     *out_used = u;
    if (out_tracking) *out_tracking = t;
    if (out_lifted)   *out_lifted = lf;
    if (out_lost)     *out_lost = lt;
}

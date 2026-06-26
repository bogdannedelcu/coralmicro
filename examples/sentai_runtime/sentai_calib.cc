// sentai_calib.cc — Kabsch 3D Procrustes + Jacobi 3x3 SVD + FxUser persist.
// See sentai_calib.h for the full design rationale and fault model.

#include "sentai_calib.h"

#include "sentai_fr.h"
#include "sentai_health.h"
#include "sentai_markers.h"
#include "sentai_svd3.h"

// OP-S10-W21-T6 — health hooks.  sentai_health.cc is ARM-only (it
// pulls fsl_soc_src.h), so on SIM these resolve to NULL at link
// time.  Guard each call against the function pointer being null.
extern "C" void sentai_health_success(SubsystemId_t) __attribute__((weak));
extern "C" void sentai_health_set_unavailable(SubsystemId_t) __attribute__((weak));
static inline void calib_health_healthy_(void) {
    if (&sentai_health_success) sentai_health_success(SUBSYS_CALIB);
}
static inline void calib_health_unavailable_(void) {
    if (&sentai_health_set_unavailable) sentai_health_set_unavailable(SUBSYS_CALIB);
}

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FxUser persistence is only available in the ARM firmware build.
// Per Sim.md §2 rule 2 ("no `#ifdef SENTAI_SIM` in core SentAI source"),
// we feature-detect via `SENTAI_HAVE_FXUSER` — defined as a compile
// flag in `examples/sentai_runtime/CMakeLists.txt` for the ARM target
// only.  On SIM (no define) we fall back to a stdio-backed shim that
// reads/writes the schema-versioned JSON in the launching cwd.
#if defined(SENTAI_HAVE_FXUSER)
// fx_user_fs.h already declares its C entry points with extern "C"
// guards internally + a separate coralmicro_fx:: C++ namespace block.
// Do NOT wrap it ourselves — that re-declares the C++ overloads as C
// linkage and fails to compile.
#include "libs/base/fx_user_fs.h"
#define SENTAI_CALIB_USE_FXUSER 1
#else
#define SENTAI_CALIB_USE_FXUSER 0
#endif

// =========================================================================
// Module state (cold; SDRAM is fine since access frequency is one-shot
// per takeoff + occasional reads from the mission FSM).
// =========================================================================
static float s_R[9]            = { 0.0f, 1.0f, 0.0f,
                                   1.0f, 0.0f, 0.0f,
                                   0.0f, 0.0f, -1.0f };
static float s_cam_offset_B[3] = { -0.04f, 0.0f, -0.02f };
static int   s_is_calibrated   = 0;
static float s_extpos_signs[3] = { -1.0f, -1.0f, 1.0f };
static float s_axis_roll_vec_px[2] = { 0.0f, 0.0f };
static float s_axis_pitch_vec_px[2] = { 0.0f, 0.0f };
static float s_axis_roll_sign = -1.0f;
static float s_axis_pitch_sign = -1.0f;
static int   s_axis_seed_valid = 0;
static sentai_calib_ini_status_t s_ini_status = {};
// OP-S10-W21-T3: persisted PID gains.  -1.0f sentinel = "not calibrated"
// (the autotune writes its converged value; bringup orchestrator commits
// the result to this slot; save() persists to /system/calib.ini).
// Indexed by sentai_calib_axis_t {X=0, Y=1, YAW=2}.
static float s_kp_persisted[SENTAI_CALIB_AXIS_COUNT] = { -1.0f, -1.0f, -1.0f };

extern "C" int sentai_camera_grab_gray_zerocopy(const uint8_t** out_buf,
                                                 int* out_w,
                                                 int* out_h,
                                                 uint32_t* out_seq,
                                                 uint32_t* out_ts_ms)
    __attribute__((weak));
extern "C" int sentai_cam_init_full(int streaming, int fps, int hflip, int vflip)
    __attribute__((weak));
extern "C" void sentai_sleep_ms(uint32_t ms) __attribute__((weak));
extern "C" int sentai_markers_init(sentai_markers_backend_t backend)
    __attribute__((weak));
extern "C" void sentai_markers_clear(void) __attribute__((weak));
extern "C" void sentai_markers_set_intrinsics(float fx, float fy,
                                               float cx, float cy)
    __attribute__((weak));
extern "C" void sentai_markers_set_marker_size(float meters)
    __attribute__((weak));
extern "C" void sentai_markers_clear_cam_extrinsics(void)
    __attribute__((weak));
extern "C" int sentai_markers_detect_frame(const uint8_t* gray, int w, int h,
                                            uint32_t frame_seq,
                                            uint32_t src_ts_ms)
    __attribute__((weak));
extern "C" int sentai_markers_get_observation(int img_w,
                                               int img_h,
                                               float margin_px,
                                               SentaiMarkersObservation* out)
    __attribute__((weak));
extern "C" int sentai_markers_set_marker_world(int n, const float* xyz_n3)
    __attribute__((weak));

static const sentai_calib_defaults_t kCalibDefaults = {
    320, 240,
    288.3f, 288.3f,
    160.0f, 120.0f,
    0.0544f,
    7,
    2.0f,
};

static const sentai_calib_limits_t kCalibLimits = {
    4,
    7,
    6.0f,
    10,
    5.0f,
    4,
    6.0f,
    2.0f,
    2.0f,
    0.35f,
};

static const float kCalibMarkerWorld[7][3] = {
    { -0.082857143f,  0.065714286f, 0.005f },
    {  0.077142857f,  0.065714286f, 0.005f },
    { -0.062857143f, -0.014285714f, 0.005f },
    {  0.057142857f, -0.014285714f, 0.005f },
    { -0.082857143f, -0.094285714f, 0.005f },
    {  0.077142857f, -0.094285714f, 0.005f },
    {  0.017142857f,  0.085714286f, 0.005f },
};

// Defaults exposed for external use (e.g., tests that need to compare
// against the sim baseline without touching the cached state).
const float SENTAI_CALIB_DEFAULT_R_SIM[9] = {
    0.0f, 1.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, -1.0f,
};
const float SENTAI_CALIB_DEFAULT_CAM_OFFSET_SIM[3] = { -0.04f, 0.0f, -0.02f };

// =========================================================================
// 3x3 lin-alg helpers + Jacobi/SVD live in sentai_svd3.{h,cc} (extracted
// 2026-05-21 for OP-S10-W19-T6 reuse).  This file uses the sentai_*
// namespaced API directly -- no local aliases.
// =========================================================================

// =========================================================================
// Internal: yaw -> R_W_B (rotation about Z, body axes aligned with W
// when yaw=0).
// =========================================================================
static void R_yaw(float yaw_rad, float Rwb[9]) {
    const float c = cosf(yaw_rad);
    const float s = sinf(yaw_rad);
    Rwb[0] = c;  Rwb[1] = -s; Rwb[2] = 0.0f;
    Rwb[3] = s;  Rwb[4] =  c; Rwb[5] = 0.0f;
    Rwb[6] = 0.0f; Rwb[7] = 0.0f; Rwb[8] = 1.0f;
}

static int is_finite_vec3(const float v[3]) {
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

// =========================================================================
// Public API
// =========================================================================
extern "C" float sentai_calib_rotation_angle_deg(const float R1[9],
                                                  const float R2[9]) {
    // M = R1^T * R2; angle = acos((tr(M) - 1) / 2)
    float M[9];
    sentai_mat3_mul_AtB(R1, R2, M);
    float tr  = M[0] + M[4] + M[8];
    float cos_t = (tr - 1.0f) * 0.5f;
    if (cos_t > 1.0f) cos_t = 1.0f;
    if (cos_t < -1.0f) cos_t = -1.0f;
    return acosf(cos_t) * 180.0f / (float)M_PI;
}

extern "C" int sentai_calib_run_kabsch(
    const sentai_calib_sample_t* samples,
    int n,
    const float* persisted_R_or_null,
    float R_out[9],
    sentai_calib_quality_t* q_out) {

    if (!samples || !R_out || !q_out) return -SENTAI_CALIB_REJ_BAD_INPUT;

    // Initialise outputs to identity + rejected state — any early return
    // leaves a defined state for the caller.
    sentai_mat3_identity(R_out);
    q_out->n_samples                  = n;
    q_out->det_R                      = 1.0f;
    q_out->mean_residual_deg          = 180.0f;
    q_out->max_residual_deg           = 180.0f;
    q_out->drift_from_persisted_deg   = -1.0f;
    q_out->accepted                   = 0;
    q_out->reject_code                = SENTAI_CALIB_REJ_BAD_INPUT;

    if (n < SENTAI_CALIB_SAMPLES_MIN) {
        q_out->reject_code = SENTAI_CALIB_REJ_TOO_FEW;
        return -SENTAI_CALIB_REJ_TOO_FEW;
    }
    if (n > SENTAI_CALIB_SAMPLES_MAX) n = SENTAI_CALIB_SAMPLES_MAX;

    // Compute cam_vecs[N] and body_vecs[N] (3 floats each).
    // Body-frame expected: body_i = R_W_B(yaw_i)^T * (marker_W - drone_W).
    // (cam_offset_B is intentionally folded into the centering step
    // below — the Procrustes is translation-invariant after centering.)
    float cam_x[SENTAI_CALIB_SAMPLES_MAX];
    float cam_y[SENTAI_CALIB_SAMPLES_MAX];
    float cam_z[SENTAI_CALIB_SAMPLES_MAX];
    float bod_x[SENTAI_CALIB_SAMPLES_MAX];
    float bod_y[SENTAI_CALIB_SAMPLES_MAX];
    float bod_z[SENTAI_CALIB_SAMPLES_MAX];

    for (int i = 0; i < n; ++i) {
        const sentai_calib_sample_t* sp = &samples[i];
        if (!is_finite_vec3(sp->tvec_cam) ||
            !is_finite_vec3(sp->marker_W) ||
            !is_finite_vec3(sp->drone_W)  ||
            !isfinite(sp->yaw_rad)) {
            q_out->reject_code = SENTAI_CALIB_REJ_BAD_INPUT;
            return -SENTAI_CALIB_REJ_BAD_INPUT;
        }
        cam_x[i] = sp->tvec_cam[0];
        cam_y[i] = sp->tvec_cam[1];
        cam_z[i] = sp->tvec_cam[2];

        float delta_W[3] = {
            sp->marker_W[0] - sp->drone_W[0],
            sp->marker_W[1] - sp->drone_W[1],
            sp->marker_W[2] - sp->drone_W[2],
        };
        float Rwb[9];
        R_yaw(sp->yaw_rad, Rwb);
        // body = Rwb^T * delta_W
        bod_x[i] = Rwb[0]*delta_W[0] + Rwb[3]*delta_W[1] + Rwb[6]*delta_W[2];
        bod_y[i] = Rwb[1]*delta_W[0] + Rwb[4]*delta_W[1] + Rwb[7]*delta_W[2];
        bod_z[i] = Rwb[2]*delta_W[0] + Rwb[5]*delta_W[1] + Rwb[8]*delta_W[2];
    }

    // Mean center both sets.
    float cm[3] = { 0.0f, 0.0f, 0.0f };
    float bm[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < n; ++i) {
        cm[0] += cam_x[i]; cm[1] += cam_y[i]; cm[2] += cam_z[i];
        bm[0] += bod_x[i]; bm[1] += bod_y[i]; bm[2] += bod_z[i];
    }
    const float inv_n = 1.0f / (float)n;
    cm[0] *= inv_n; cm[1] *= inv_n; cm[2] *= inv_n;
    bm[0] *= inv_n; bm[1] *= inv_n; bm[2] *= inv_n;
    for (int i = 0; i < n; ++i) {
        cam_x[i] -= cm[0]; cam_y[i] -= cm[1]; cam_z[i] -= cm[2];
        bod_x[i] -= bm[0]; bod_y[i] -= bm[1]; bod_z[i] -= bm[2];
    }

    // H = body_centered * cam_centered^T  (3x3).
    float H[9] = { 0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f };
    for (int i = 0; i < n; ++i) {
        H[0] += bod_x[i] * cam_x[i];
        H[1] += bod_x[i] * cam_y[i];
        H[2] += bod_x[i] * cam_z[i];
        H[3] += bod_y[i] * cam_x[i];
        H[4] += bod_y[i] * cam_y[i];
        H[5] += bod_y[i] * cam_z[i];
        H[6] += bod_z[i] * cam_x[i];
        H[7] += bod_z[i] * cam_y[i];
        H[8] += bod_z[i] * cam_z[i];
    }

    float U[9], s[3], Vt[9];
    if (sentai_svd3(H, U, s, Vt) != 0) {
        q_out->reject_code = SENTAI_CALIB_REJ_SVD_NO_CV;
        return -SENTAI_CALIB_REJ_SVD_NO_CV;
    }

    // d = sign(det(U * Vt)) — controls the reflection-vs-rotation flip
    // applied to the smallest singular value (column 2 after sorting).
    float UVt[9];
    sentai_mat3_mul(U, Vt, UVt);
    const float det_UVt = sentai_mat3_det(UVt);
    const float dflip   = (det_UVt >= 0.0f) ? 1.0f : -1.0f;

    // R = U * diag(1, 1, dflip) * Vt.
    // Equivalent: scale column 2 of U by dflip, then R = U * Vt.
    float U2[9];
    memcpy(U2, U, sizeof(U2));
    U2[0*3 + 2] *= dflip;
    U2[1*3 + 2] *= dflip;
    U2[2*3 + 2] *= dflip;

    float R[9];
    sentai_mat3_mul(U2, Vt, R);
    memcpy(R_out, R, sizeof(R));
    const float det_R = sentai_mat3_det(R);
    q_out->det_R = det_R;

    // Residuals: per-sample angle between R*cam_i and body_i (uncentered).
    float sum_res = 0.0f, max_res = 0.0f;
    int   counted = 0;
    for (int i = 0; i < n; ++i) {
        // Use the raw (un-centered) sample vectors for the residual.
        const sentai_calib_sample_t* sp = &samples[i];
        float cam_raw[3] = { sp->tvec_cam[0], sp->tvec_cam[1], sp->tvec_cam[2] };
        float delta_W[3] = {
            sp->marker_W[0] - sp->drone_W[0],
            sp->marker_W[1] - sp->drone_W[1],
            sp->marker_W[2] - sp->drone_W[2],
        };
        float Rwb[9];
        R_yaw(sp->yaw_rad, Rwb);
        float body_raw[3] = {
            Rwb[0]*delta_W[0] + Rwb[3]*delta_W[1] + Rwb[6]*delta_W[2],
            Rwb[1]*delta_W[0] + Rwb[4]*delta_W[1] + Rwb[7]*delta_W[2],
            Rwb[2]*delta_W[0] + Rwb[5]*delta_W[1] + Rwb[8]*delta_W[2],
        };
        float Rcam[3] = {
            R[0]*cam_raw[0] + R[1]*cam_raw[1] + R[2]*cam_raw[2],
            R[3]*cam_raw[0] + R[4]*cam_raw[1] + R[5]*cam_raw[2],
            R[6]*cam_raw[0] + R[7]*cam_raw[1] + R[8]*cam_raw[2],
        };
        const float na = sentai_vec3_norm(Rcam);
        const float nb = sentai_vec3_norm(body_raw);
        if (na < 1e-6f || nb < 1e-6f) continue;
        float cos_t = sentai_vec3_dot(Rcam, body_raw) / (na * nb);
        if (cos_t > 1.0f) cos_t = 1.0f;
        if (cos_t < -1.0f) cos_t = -1.0f;
        const float deg = acosf(cos_t) * 180.0f / (float)M_PI;
        sum_res += deg;
        if (deg > max_res) max_res = deg;
        counted++;
    }
    q_out->mean_residual_deg = (counted > 0) ? (sum_res / (float)counted)
                                              : 180.0f;
    q_out->max_residual_deg  = max_res;

    // Drift check vs persisted (if provided).
    if (persisted_R_or_null) {
        q_out->drift_from_persisted_deg =
            sentai_calib_rotation_angle_deg(R, persisted_R_or_null);
    } else {
        q_out->drift_from_persisted_deg = -1.0f;
    }

    // Accept / reject per §21.5 fault model.
    q_out->reject_code = SENTAI_CALIB_OK;
    if (fabsf(det_R) < SENTAI_CALIB_QUALITY_DET_THR) {
        q_out->reject_code = SENTAI_CALIB_REJ_DET_LOW;
    } else if (det_R < 0.0f) {
        q_out->reject_code = SENTAI_CALIB_REJ_REFLECTION;
    } else if (q_out->mean_residual_deg > SENTAI_CALIB_QUALITY_RES_DEG) {
        q_out->reject_code = SENTAI_CALIB_REJ_RES_HIGH;
    } else if (q_out->drift_from_persisted_deg >= 0.0f &&
               q_out->drift_from_persisted_deg > SENTAI_CALIB_QUALITY_DRIFT_DEG) {
        q_out->reject_code = SENTAI_CALIB_REJ_DRIFT_HIGH;
    }
    q_out->accepted = (q_out->reject_code == SENTAI_CALIB_OK) ? 1 : 0;
    return 0;
}

extern "C" int sentai_calib_commit_R(const float R[9],
                                     const float cam_offset_B[3]) {
    if (!R) return -1;
    for (int i = 0; i < 9; ++i) {
        if (!isfinite(R[i])) return -1;
    }
    if (cam_offset_B) {
        for (int i = 0; i < 3; ++i) {
            if (!isfinite(cam_offset_B[i])) return -1;
        }
    }
    // Atomic-as-block: 9-float + 3-float copies. Readers that race with
    // this either see the old or new state (single-writer contract).
    memcpy(s_R, R, sizeof(s_R));
    if (cam_offset_B) memcpy(s_cam_offset_B, cam_offset_B, sizeof(s_cam_offset_B));
    s_is_calibrated = 1;
    calib_health_healthy_();                 // OP-S10-W21-T6
    return 0;
}

extern "C" const float* sentai_calib_get_R_cam_to_body(void) { return s_R; }
extern "C" const float* sentai_calib_get_cam_offset_B(void)   { return s_cam_offset_B; }
extern "C" int          sentai_calib_is_calibrated(void)      { return s_is_calibrated; }
extern "C" const float* sentai_calib_get_extpos_signs(void)   { return s_extpos_signs; }

extern "C" int sentai_calib_get_axis_seed(float roll_vec_px[2],
                                           float* roll_sign,
                                           float pitch_vec_px[2],
                                           float* pitch_sign) {
    if (roll_vec_px) {
        roll_vec_px[0] = 0.0f;
        roll_vec_px[1] = 0.0f;
    }
    if (pitch_vec_px) {
        pitch_vec_px[0] = 0.0f;
        pitch_vec_px[1] = 0.0f;
    }
    if (roll_sign) *roll_sign = 0.0f;
    if (pitch_sign) *pitch_sign = 0.0f;
    if (!s_axis_seed_valid) return 0;
    if (roll_vec_px) {
        roll_vec_px[0] = s_axis_roll_vec_px[0];
        roll_vec_px[1] = s_axis_roll_vec_px[1];
    }
    if (pitch_vec_px) {
        pitch_vec_px[0] = s_axis_pitch_vec_px[0];
        pitch_vec_px[1] = s_axis_pitch_vec_px[1];
    }
    if (roll_sign) *roll_sign = s_axis_roll_sign;
    if (pitch_sign) *pitch_sign = s_axis_pitch_sign;
    return 1;
}

extern "C" int sentai_calib_get_ini_status(sentai_calib_ini_status_t* out) {
    if (!out) return 0;
    memcpy(out, &s_ini_status, sizeof(*out));
    return 1;
}

extern "C" int sentai_calib_get_defaults(
        sentai_calib_defaults_t* out) {
    if (!out) return 0;
    *out = kCalibDefaults;
    return 1;
}

extern "C" int sentai_calib_get_limits(sentai_calib_limits_t* out) {
    if (!out) return 0;
    *out = kCalibLimits;
    return 1;
}

extern "C" int sentai_calib_setup_defaults(void) {
    sentai_calib_init();

    if (sentai_cam_init_full) {
        const int cam_rc = sentai_cam_init_full(1, 30, 0, 1);
        if (cam_rc != 0 && cam_rc != -11) {
            char ev[64];
            snprintf(ev, sizeof(ev), "camera_rc=%d", cam_rc);
            sentai_fr_push_event("calib_setup", ev);
            return -10;
        }
    }

    if (!sentai_markers_clear || !sentai_markers_init ||
        !sentai_markers_set_intrinsics || !sentai_markers_set_marker_size ||
        !sentai_markers_clear_cam_extrinsics ||
        !sentai_markers_set_marker_world) {
        sentai_fr_push_event("calib_setup", "markers_hook_missing");
        return -20;
    }

    sentai_markers_clear();
    const int init_rc = sentai_markers_init(SENTAI_MARKERS_BACKEND_WHYCON);
    if (init_rc != 0) {
        char ev[64];
        snprintf(ev, sizeof(ev), "markers_init_rc=%d", init_rc);
        sentai_fr_push_event("calib_setup", ev);
        return -20;
    }
    sentai_markers_set_intrinsics(kCalibDefaults.fx, kCalibDefaults.fy,
                                  kCalibDefaults.cx, kCalibDefaults.cy);
    sentai_markers_set_marker_size(kCalibDefaults.marker_diameter_m);
    sentai_markers_clear_cam_extrinsics();
    const int world_rc = sentai_markers_set_marker_world(
        kCalibDefaults.marker_world_count, &kCalibMarkerWorld[0][0]);
    if (world_rc != 0) {
        char ev[64];
        snprintf(ev, sizeof(ev), "marker_world_rc=%d", world_rc);
        sentai_fr_push_event("calib_setup", ev);
        return -30;
    }
    sentai_fr_push_event("calib_setup", "ok=1 layout=A3_SMALL");
    return 0;
}

static float det3_(const float R[9]) {
    return R[0] * (R[4] * R[8] - R[5] * R[7]) -
           R[1] * (R[3] * R[8] - R[5] * R[6]) +
           R[2] * (R[3] * R[7] - R[4] * R[6]);
}

static int axis_code_(const char* axis) {
    if (!axis) return -1;
    if (axis[0] == 'x' || axis[0] == 'X') return 0;
    if (axis[0] == 'y' || axis[0] == 'Y') return 1;
    return -1;
}

static void expected_from_row_(const float row[3], int* axis_code, int* sign) {
    int ax = 0;
    float val = row[0];
    if (fabsf(row[1]) > fabsf(val)) {
        ax = 1;
        val = row[1];
    }
    *axis_code = ax;
    *sign = (val >= 0.0f) ? 1 : -1;
}

extern "C" int sentai_calib_expected_from_row(float r0,
                                               float r1,
                                               float r2,
                                               int* axis_code_out,
                                               int* sign_out) {
    if (!axis_code_out || !sign_out) return 0;
    if (!isfinite(r0) || !isfinite(r1) || !isfinite(r2)) return 0;
    const float row[3] = {r0, r1, r2};
    expected_from_row_(row, axis_code_out, sign_out);
    return 1;
}

extern "C" int sentai_calib_axis_observation_from_delta(
        float dx,
        float dy,
        int* axis_code_out,
        int* sign_out,
        float* dominance_out,
        float* strength_out) {
    if (!axis_code_out || !sign_out || !dominance_out || !strength_out) {
        return 0;
    }
    if (!isfinite(dx) || !isfinite(dy)) return 0;

    int axis_code = 0;
    float dominant = dx;
    float secondary = dy;
    if (fabsf(dy) > fabsf(dx)) {
        axis_code = 1;
        dominant = dy;
        secondary = dx;
    }
    float dominance = 999.0f;
    if (fabsf(secondary) > 0.0001f) {
        dominance = fabsf(dominant) / fabsf(secondary);
    }
    *axis_code_out = axis_code;
    *sign_out = (dominant >= 0.0f) ? 1 : -1;
    *dominance_out = dominance;
    *strength_out = sqrtf(dx * dx + dy * dy);
    return 1;
}

extern "C" int sentai_calib_marker_avg_lock_ok(int ready,
                                                float avg_full,
                                                float threshold) {
    if (!isfinite(avg_full) || !isfinite(threshold)) return 0;
    if (threshold < 0.0f) threshold = kCalibLimits.marker_avg_full_lock;
    return (ready != 0 && avg_full > threshold) ? 1 : 0;
}

extern "C" int sentai_calib_marker_avg_unsafe(int ready,
                                               float avg_full,
                                               float threshold) {
    if (!isfinite(avg_full) || !isfinite(threshold)) return 1;
    if (threshold < 0.0f) threshold = (float)kCalibLimits.min_full_markers;
    return (ready != 0 && avg_full < threshold) ? 1 : 0;
}

extern "C" int sentai_calib_feature_has_lock(int n_full,
                                              float radius_mean_px,
                                              int centroid_valid,
                                              int avg_ready,
                                              float avg_full) {
    if (!isfinite(radius_mean_px) || !isfinite(avg_full)) return 0;
    int marker_ok = n_full >= kCalibLimits.acq_full_markers;
    if (avg_ready) {
        marker_ok = sentai_calib_marker_avg_lock_ok(
            avg_ready, avg_full, kCalibLimits.marker_avg_full_lock);
    }
    return (marker_ok &&
            radius_mean_px >= kCalibLimits.min_lock_radius_px &&
            centroid_valid) ? 1 : 0;
}

extern "C" int sentai_calib_marker_lock_ok(int min_full,
                                            float avg_full) {
    if (!isfinite(avg_full)) return 0;
    return (min_full >= kCalibLimits.axis_hard_min_full_markers &&
            avg_full > kCalibLimits.axis_min_avg_full_markers) ? 1 : 0;
}

extern "C" int sentai_calib_score_axis_candidate(
        const char* roll_axis,
        int roll_sign,
        const char* pitch_axis,
        int pitch_sign,
        sentai_calib_axis_candidate_t* out) {
    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    const int roll_axis_code = axis_code_(roll_axis);
    const int pitch_axis_code = axis_code_(pitch_axis);
    if (roll_axis_code < 0 || pitch_axis_code < 0) return 0;
    if (roll_sign == 0 || pitch_sign == 0) return 0;
    roll_sign = (roll_sign >= 0) ? 1 : -1;
    pitch_sign = (pitch_sign >= 0) ? 1 : -1;

    static const float rows[6][3] = {
        { 1.0f, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f }, { 0.0f, -1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f },
    };
    float best_score = 1.0e9f;
    float second_score = 1.0e9f;
    float best_R[9] = {0.0f};
    int best_idx = -1;
    int idx = 0;
    for (int ix = 0; ix < 6; ++ix) {
        for (int iy = 0; iy < 6; ++iy) {
            const float dot = rows[ix][0] * rows[iy][0] +
                              rows[ix][1] * rows[iy][1] +
                              rows[ix][2] * rows[iy][2];
            if (fabsf(dot) > 0.0001f) continue;
            float R[9] = {
                rows[ix][0], rows[ix][1], rows[ix][2],
                rows[iy][0], rows[iy][1], rows[iy][2],
                rows[ix][1] * rows[iy][2] - rows[ix][2] * rows[iy][1],
                rows[ix][2] * rows[iy][0] - rows[ix][0] * rows[iy][2],
                rows[ix][0] * rows[iy][1] - rows[ix][1] * rows[iy][0],
            };
            if (det3_(R) <= 0.5f) continue;

            int exp_axis = 0;
            int exp_sign = 1;
            expected_from_row_(&R[0], &exp_axis, &exp_sign);
            float score = ((exp_axis == pitch_axis_code) ? 0.0f : 10.0f) +
                          ((exp_sign == pitch_sign) ? 0.0f : 1.0f);
            expected_from_row_(&R[3], &exp_axis, &exp_sign);
            score += ((exp_axis == roll_axis_code) ? 0.0f : 10.0f) +
                     ((exp_sign == roll_sign) ? 0.0f : 1.0f);

            if (score < best_score) {
                second_score = best_score;
                best_score = score;
                best_idx = idx;
                memcpy(best_R, R, sizeof(best_R));
            } else if (score < second_score) {
                second_score = score;
            }
            ++idx;
        }
    }
    if (best_idx < 0) return 0;
    out->best_idx = best_idx;
    out->best_score = best_score;
    out->second_score = second_score;
    out->margin = second_score - best_score;
    out->det = det3_(best_R);
    memcpy(out->R_cam_to_body, best_R, sizeof(best_R));
    out->ok = (best_score <= 0.1f && out->margin >= 1.0f) ? 1 : 0;
    char ev[96];
    snprintf(ev, sizeof(ev),
             "ok=%d idx=%d score=%.2f margin=%.2f roll=%c%d pitch=%c%d",
             out->ok, out->best_idx, (double)out->best_score,
             (double)out->margin,
             roll_axis_code == 0 ? 'x' : 'y', roll_sign,
             pitch_axis_code == 0 ? 'x' : 'y', pitch_sign);
    sentai_fr_push_event("calib_axis_score", ev);
    sentai_fr_push_scalar("calib_score", out->best_score, 0);
    sentai_fr_push_scalar("calib_margin", out->margin, 0);
    return 1;
}

extern "C" int sentai_calib_sample_observation(
        int img_w,
        int img_h,
        float margin_px,
        SentaiMarkersObservation* out) {
    if (!out || img_w <= 0 || img_h <= 0 || !isfinite(margin_px)) return -1;
    memset(out, 0, sizeof(*out));
    out->n_raw = -1;
    out->centroid_x = NAN;
    out->centroid_y = NAN;
    out->bbox_min_x = NAN;
    out->bbox_min_y = NAN;
    out->bbox_max_x = NAN;
    out->bbox_max_y = NAN;

    if (!sentai_camera_grab_gray_zerocopy) {
        sentai_fr_push_event("calib_obs", "camera_hook_missing");
        return -2;
    }

    const uint8_t* gray = nullptr;
    int w = 0;
    int h = 0;
    uint32_t seq = 0;
    uint32_t ts = 0;
    const int grab_rc = sentai_camera_grab_gray_zerocopy(&gray, &w, &h,
                                                          &seq, &ts);
    if (grab_rc != 0 || !gray || w <= 0 || h <= 0) {
        char ev[80];
        snprintf(ev, sizeof(ev), "grab_rc=%d w=%d h=%d", grab_rc, w, h);
        sentai_fr_push_event("calib_obs", ev);
        // Preserve the old MP binding contract exactly: detect_from_camera()
        // returned 0 when no camera frame was ready yet.  The observation is
        // invalid, but this is not a hard mission error.
        out->n_raw = 0;
        return 0;
    }

    if (!sentai_markers_detect_frame || !sentai_markers_get_observation) {
        sentai_fr_push_event("calib_obs", "markers_hook_missing");
        return -3;
    }

    const int n = sentai_markers_detect_frame(gray, w, h, seq, ts);
    if (n < 0) {
        char ev[80];
        snprintf(ev, sizeof(ev), "detect_rc=%d seq=%lu", n,
                 (unsigned long)seq);
        sentai_fr_push_event("calib_obs", ev);
        out->n_raw = n;
        out->frame_seq = seq;
        out->src_ts_ms = ts;
        return n;
    }

    if (!sentai_markers_get_observation(img_w, img_h, margin_px, out)) {
        sentai_fr_push_event("calib_obs", "aggregate_failed");
        return -4;
    }

    sentai_fr_push_scalar("obs_n_raw", out->n_raw, ts);
    sentai_fr_push_scalar("obs_n_full", out->n_full, ts);
    sentai_fr_push_scalar("obs_n_pose", out->n_pose_valid, ts);
    if (out->n_full > 0) {
        sentai_fr_push_scalar("obs_cx", out->centroid_x, ts);
        sentai_fr_push_scalar("obs_cy", out->centroid_y, ts);
        sentai_fr_push_scalar("obs_radius", out->radius_mean_px, ts);
    }
    if (out->n_pose_valid > 0) {
        sentai_fr_push_scalar("obs_z", out->z_cam_mean_m, ts);
    }
    return n;
}

extern "C" int sentai_calib_vertical_rate_thrust(
        float z_m,
        float z_prev_m,
        float vz_filt_m_s,
        int base_thrust_u16,
        float target_vz_m_s,
        float dt_s,
        float lpf_alpha,
        float kd_thrust_per_m_s,
        int thrust_floor_u16,
        int thrust_ceil_u16,
        int* thrust_out_u16,
        float* z_prev_out_m,
        float* vz_filt_out_m_s) {
    if (!thrust_out_u16 || !z_prev_out_m || !vz_filt_out_m_s) return 0;
    if (!isfinite(z_m) || !isfinite(z_prev_m) || !isfinite(vz_filt_m_s) ||
        !isfinite(target_vz_m_s) || !isfinite(dt_s) ||
        !isfinite(lpf_alpha) || !isfinite(kd_thrust_per_m_s) ||
        dt_s <= 0.0f) {
        return 0;
    }
    if (lpf_alpha < 0.0f) lpf_alpha = 0.0f;
    if (lpf_alpha > 1.0f) lpf_alpha = 1.0f;
    if (thrust_floor_u16 < 0) thrust_floor_u16 = 0;
    if (thrust_ceil_u16 < thrust_floor_u16) thrust_ceil_u16 = thrust_floor_u16;

    float z_prev_out = z_prev_m;
    float vz_out = vz_filt_m_s;
    int thrust = thrust_floor_u16;
    if (z_m > 0.0f && z_prev_m > 0.0f) {
        const float vz = (z_m - z_prev_m) / dt_s;
        vz_out = (1.0f - lpf_alpha) * vz_out + lpf_alpha * vz;
    } else if (z_m > 0.0f) {
        vz_out = 0.0f;
    }
    if (z_m > 0.0f) {
        z_prev_out = z_m;
        const float vz_err = vz_out - target_vz_m_s;
        thrust = (int)((float)base_thrust_u16 -
                       kd_thrust_per_m_s * vz_err);
    }
    if (thrust < thrust_floor_u16) thrust = thrust_floor_u16;
    if (thrust > thrust_ceil_u16) thrust = thrust_ceil_u16;

    *thrust_out_u16 = thrust;
    *z_prev_out_m = z_prev_out;
    *vz_filt_out_m_s = vz_out;
    return 1;
}

extern "C" int sentai_calib_z_hold_thrust(
        float z_m,
        float z_prev_m,
        float vz_filt_m_s,
        float target_z_m,
        int base_thrust_u16,
        float kp_thrust_per_m,
        float kd_thrust_per_m_s,
        float dt_s,
        float lpf_alpha,
        int thrust_floor_u16,
        int thrust_ceil_u16,
        int* thrust_out_u16,
        float* z_prev_out_m,
        float* vz_filt_out_m_s) {
    if (!thrust_out_u16 || !z_prev_out_m || !vz_filt_out_m_s) return 0;
    if (!isfinite(z_m) || !isfinite(z_prev_m) || !isfinite(vz_filt_m_s) ||
        !isfinite(target_z_m) || !isfinite(kp_thrust_per_m) ||
        !isfinite(kd_thrust_per_m_s) || !isfinite(dt_s) ||
        !isfinite(lpf_alpha) || dt_s <= 0.0f) {
        return 0;
    }
    if (lpf_alpha < 0.0f) lpf_alpha = 0.0f;
    if (lpf_alpha > 1.0f) lpf_alpha = 1.0f;
    if (thrust_floor_u16 < 0) thrust_floor_u16 = 0;
    if (thrust_ceil_u16 < thrust_floor_u16) thrust_ceil_u16 = thrust_floor_u16;

    float z_prev_out = z_prev_m;
    float vz_out = vz_filt_m_s;
    int thrust = base_thrust_u16;
    if (z_m > 0.0f && z_prev_m > 0.0f) {
        const float vz = (z_m - z_prev_m) / dt_s;
        vz_out = (1.0f - lpf_alpha) * vz_out + lpf_alpha * vz;
    } else if (z_m > 0.0f) {
        vz_out = 0.0f;
    }
    if (z_m > 0.0f) {
        z_prev_out = z_m;
        const float err = target_z_m - z_m;
        thrust = (int)((float)base_thrust_u16 +
                       kp_thrust_per_m * err -
                       kd_thrust_per_m_s * vz_out);
    }
    if (thrust < thrust_floor_u16) thrust = thrust_floor_u16;
    if (thrust > thrust_ceil_u16) thrust = thrust_ceil_u16;

    *thrust_out_u16 = thrust;
    *z_prev_out_m = z_prev_out;
    *vz_filt_out_m_s = vz_out;
    return 1;
}

// OP-S10-W21-T3 — persisted Kp API.
extern "C" int sentai_calib_commit_kp(sentai_calib_axis_t axis, float kp) {
    if (axis < SENTAI_CALIB_AXIS_X || axis >= SENTAI_CALIB_AXIS_COUNT) {
        return -1;
    }
    // Accept exactly -1.0f as the "uncalibrated" sentinel, OR a finite
    // positive gain.  Reject NaN / inf / negative-other / zero.
    if (kp == -1.0f) {
        s_kp_persisted[axis] = -1.0f;
        return 0;
    }
    if (!isfinite(kp) || kp <= 0.0f) return -1;
    s_kp_persisted[axis] = kp;
    return 0;
}

extern "C" float sentai_calib_get_persisted_kp(sentai_calib_axis_t axis) {
    if (axis < SENTAI_CALIB_AXIS_X || axis >= SENTAI_CALIB_AXIS_COUNT) {
        return -1.0f;
    }
    return s_kp_persisted[axis];
}

extern "C" void sentai_calib_clear(void) {
    memcpy(s_R, SENTAI_CALIB_DEFAULT_R_SIM, sizeof(s_R));
    memcpy(s_cam_offset_B, SENTAI_CALIB_DEFAULT_CAM_OFFSET_SIM,
           sizeof(s_cam_offset_B));
    s_is_calibrated = 0;
    s_extpos_signs[0] = -1.0f;
    s_extpos_signs[1] = -1.0f;
    s_extpos_signs[2] = 1.0f;
    s_axis_roll_vec_px[0] = s_axis_roll_vec_px[1] = 0.0f;
    s_axis_pitch_vec_px[0] = s_axis_pitch_vec_px[1] = 0.0f;
    s_axis_roll_sign = -1.0f;
    s_axis_pitch_sign = -1.0f;
    s_axis_seed_valid = 0;
    memset(&s_ini_status, 0, sizeof(s_ini_status));
    for (int i = 0; i < SENTAI_CALIB_AXIS_COUNT; ++i) {
        s_kp_persisted[i] = -1.0f;
    }
    calib_health_unavailable_();             // OP-S10-W21-T6
}

// =========================================================================
// Persistence — schema v2 INI (OP-S10-W21-T2).  Newline-separated
// key=value, ASCII.  Forward-compatible: unknown keys are silently ignored,
// so experiment metadata can live in the same calib.ini as the runtime keys
// without requiring a schema bump.  The schema= key is here as a tripwire for
// future incompatible layout changes, not for additive features.
// =========================================================================
static constexpr size_t SENTAI_CALIB_INI_MAX_BYTES = 4096;

static int format_ini(char* buf, size_t cap,
                      const float R[9], const float cam_off[3],
                      const float kp[SENTAI_CALIB_AXIS_COUNT]) {
    // Always write all 3 kp keys: a -1.0f value tells future loads
    // "not calibrated", and explicit presence is friendlier to cat
    // inspection than silently-omitted keys.
    int n = snprintf(buf, cap,
        "schema=%d\n"
        "R_B_C=%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n"
        "cam_offset_B=%.9f,%.9f,%.9f\n"
        "kp_x=%.6f\n"
        "kp_y=%.6f\n"
        "kp_yaw=%.6f\n",
        SENTAI_CALIB_SCHEMA_VERSION,
        R[0], R[1], R[2],
        R[3], R[4], R[5],
        R[6], R[7], R[8],
        cam_off[0], cam_off[1], cam_off[2],
        kp[SENTAI_CALIB_AXIS_X],
        kp[SENTAI_CALIB_AXIS_Y],
        kp[SENTAI_CALIB_AXIS_YAW]);
    return n;
}

// Parse N comma-separated floats from a NUL- or newline-terminated
// value string.  Returns count parsed (may be < N if value runs out),
// or -1 on any non-finite float.
static int parse_csv_floats(const char* val, float* out, int N) {
    int i = 0;
    const char* p = val;
    while (*p && *p != '\n' && i < N) {
        // Skip leading whitespace + commas.
        while (*p == ' ' || *p == '\t' || *p == ',') p++;
        if (!*p || *p == '\n') break;
        char* end = NULL;
        double v  = strtod(p, &end);
        if (end == p) break;
        if (!isfinite((float)v)) return -1;
        out[i++] = (float)v;
        p = end;
    }
    return i;
}

static int parse_one_float(const char* val, float* out) {
    if (!val || !out) return 0;
    float v = 0.0f;
    if (parse_csv_floats(val, &v, 1) != 1) return 0;
    if (!isfinite(v)) return 0;
    *out = v;
    return 1;
}

// Parse a single decimal integer from a NUL- or newline-terminated
// value string.  Returns 0 if no digits.
static long parse_long(const char* val) {
    char* end = NULL;
    long v = strtol(val, &end, 10);
    if (end == val) return 0;
    return v;
}

// Scan the INI buffer line by line for `key=...` (skipping blank lines
// and `#` comments).  On hit, returns a pointer to the first char of the
// value (just past `=`), still inside the same buffer.  Returns NULL if
// the key is absent.  Tolerant of trailing whitespace, LF/CRLF line ends.
static const char* ini_find_value(const char* buf, const char* key) {
    const size_t klen = strlen(key);
    const char* p = buf;
    while (*p) {
        // Skip blank lines + leading whitespace.
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
        if (!*p) break;
        if (*p == '#' || *p == ';') {
            while (*p && *p != '\n') p++;
            continue;
        }
        // Compare key prefix.
        if (strncmp(p, key, klen) == 0) {
            const char* q = p + klen;
            while (*q == ' ' || *q == '\t') q++;
            if (*q == '=') {
                q++;
                while (*q == ' ' || *q == '\t') q++;
                return q;
            }
        }
        // Skip to end of line.
        while (*p && *p != '\n') p++;
    }
    return NULL;
}

enum {
    CALIB_MISS_SCHEMA = 1u << 0,
    CALIB_MISS_TASK_ID = 1u << 1,
    CALIB_MISS_STATUS = 1u << 2,
    CALIB_MISS_ACCEPTED = 1u << 3,
    CALIB_MISS_LAYOUT_ID = 1u << 4,
    CALIB_MISS_R_B_C = 1u << 5,
    CALIB_MISS_CAM_OFFSET_B = 1u << 6,
    CALIB_MISS_FX = 1u << 7,
    CALIB_MISS_FY = 1u << 8,
    CALIB_MISS_CX = 1u << 9,
    CALIB_MISS_CY = 1u << 10,
    CALIB_MISS_IMAGE_W = 1u << 11,
    CALIB_MISS_IMAGE_H = 1u << 12,
    CALIB_MISS_MARKER_COUNT = 1u << 13,
    CALIB_MISS_MARKER_DIAMETER = 1u << 14,
    CALIB_MISS_EXTPOS_SIGN_X = 1u << 15,
    CALIB_MISS_EXTPOS_SIGN_Y = 1u << 16,
    CALIB_MISS_EXTPOS_SIGN_Z = 1u << 17,
    CALIB_MISS_LANDING_OK = 1u << 18,
    CALIB_MISS_DISARM_FULL = 1u << 19,
    CALIB_MISS_AXIS_ROLL_SIGN = 1u << 20,
    CALIB_MISS_AXIS_ROLL_VEC = 1u << 21,
    CALIB_MISS_AXIS_PITCH_SIGN = 1u << 22,
    CALIB_MISS_AXIS_PITCH_VEC = 1u << 23,
};

static void copy_line_value_(const char* val, char* out, size_t cap) {
    if (!out || cap == 0) return;
    out[0] = '\0';
    if (!val) return;
    size_t n = 0;
    while (val[n] && val[n] != '\r' && val[n] != '\n' && n + 1 < cap) {
        out[n] = val[n];
        n++;
    }
    out[n] = '\0';
}

static int value_equals_(const char* val, const char* want) {
    if (!val || !want) return 0;
    size_t i = 0;
    while (want[i]) {
        if (val[i] != want[i]) return 0;
        i++;
    }
    return (val[i] == '\0' || val[i] == '\r' || val[i] == '\n' ||
            val[i] == ' ' || val[i] == '\t');
}

static int ini_strict_lines_ok_(const char* buf) {
    const char* p = buf;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == '\r') p++;
        if (*p == '\n') {
            p++;
            continue;
        }
        if (*p == '#' || *p == ';') return 0;
        int has_eq = 0;
        while (*p && *p != '\n') {
            if (*p == '=') has_eq = 1;
            p++;
        }
        if (!has_eq) return 0;
        if (*p == '\n') p++;
    }
    return 1;
}

static void update_ini_status_(const char* buf, sentai_calib_ini_status_t* st) {
    memset(st, 0, sizeof(*st));
    st->present = (buf && buf[0]) ? 1 : 0;
    st->strict_lines_ok = st->present ? (uint8_t)ini_strict_lines_ok_(buf) : 0;
    if (!st->present) return;

    const char* v = ini_find_value(buf, "schema");
    if (!v) st->missing_mask |= CALIB_MISS_SCHEMA;
    else st->schema_ok = (parse_long(v) == SENTAI_CALIB_SCHEMA_VERSION) ? 1 : 0;

    v = ini_find_value(buf, "task_id");
    if (!v) st->missing_mask |= CALIB_MISS_TASK_ID;
    else st->task_ok = value_equals_(v, "TD-S10-B3") ? 1 : 0;

    v = ini_find_value(buf, "status");
    if (!v) st->missing_mask |= CALIB_MISS_STATUS;
    else {
        copy_line_value_(v, st->status, sizeof(st->status));
        st->status_ok = value_equals_(v, "FINAL_VALIDATION_OK") ? 1 : 0;
    }

    v = ini_find_value(buf, "accepted");
    if (!v) st->missing_mask |= CALIB_MISS_ACCEPTED;
    else st->accepted_ok = value_equals_(v, "1") ? 1 : 0;

    v = ini_find_value(buf, "layout_id");
    if (!v) st->missing_mask |= CALIB_MISS_LAYOUT_ID;
    else {
        copy_line_value_(v, st->layout_id, sizeof(st->layout_id));
        st->layout_ok = value_equals_(v, "sentai_whycon_small_centroid_7_marker_v1") ? 1 : 0;
    }

    if (!ini_find_value(buf, "R_B_C")) st->missing_mask |= CALIB_MISS_R_B_C;
    if (!ini_find_value(buf, "cam_offset_B")) st->missing_mask |= CALIB_MISS_CAM_OFFSET_B;
    if (!ini_find_value(buf, "fx")) st->missing_mask |= CALIB_MISS_FX;
    if (!ini_find_value(buf, "fy")) st->missing_mask |= CALIB_MISS_FY;
    if (!ini_find_value(buf, "cx")) st->missing_mask |= CALIB_MISS_CX;
    if (!ini_find_value(buf, "cy")) st->missing_mask |= CALIB_MISS_CY;
    if (!ini_find_value(buf, "image_w")) st->missing_mask |= CALIB_MISS_IMAGE_W;
    if (!ini_find_value(buf, "image_h")) st->missing_mask |= CALIB_MISS_IMAGE_H;
    if (!ini_find_value(buf, "marker_count")) st->missing_mask |= CALIB_MISS_MARKER_COUNT;
    if (!ini_find_value(buf, "marker_diameter_m")) st->missing_mask |= CALIB_MISS_MARKER_DIAMETER;
    if (!ini_find_value(buf, "extpos_sign_x")) st->missing_mask |= CALIB_MISS_EXTPOS_SIGN_X;
    if (!ini_find_value(buf, "extpos_sign_y")) st->missing_mask |= CALIB_MISS_EXTPOS_SIGN_Y;
    if (!ini_find_value(buf, "extpos_sign_z")) st->missing_mask |= CALIB_MISS_EXTPOS_SIGN_Z;
    if (!ini_find_value(buf, "camera_referenced_landing_ok")) st->missing_mask |= CALIB_MISS_LANDING_OK;
    if (!ini_find_value(buf, "disarm_full_markers")) st->missing_mask |= CALIB_MISS_DISARM_FULL;
    if (!ini_find_value(buf, "axis_roll_sign")) st->missing_mask |= CALIB_MISS_AXIS_ROLL_SIGN;
    if (!ini_find_value(buf, "axis_roll_vec_px")) st->missing_mask |= CALIB_MISS_AXIS_ROLL_VEC;
    if (!ini_find_value(buf, "axis_pitch_sign")) st->missing_mask |= CALIB_MISS_AXIS_PITCH_SIGN;
    if (!ini_find_value(buf, "axis_pitch_vec_px")) st->missing_mask |= CALIB_MISS_AXIS_PITCH_VEC;

    st->required_ok = (st->schema_ok && st->accepted_ok && st->status_ok &&
                       st->task_ok && st->layout_ok && st->strict_lines_ok &&
                       st->missing_mask == 0) ? 1 : 0;
}

static int parse_ini(const char* buf, float R_out[9], float cam_off_out[3],
                     float kp_out[SENTAI_CALIB_AXIS_COUNT]) {
    sentai_calib_ini_status_t st;
    update_ini_status_(buf, &st);
    memcpy(&s_ini_status, &st, sizeof(s_ini_status));

    const char* p = ini_find_value(buf, "schema");
    if (!p) return -1;
    const long schema = parse_long(p);
    if (schema != SENTAI_CALIB_SCHEMA_VERSION) return -1;

    p = ini_find_value(buf, "R_B_C");
    if (!p) return -1;
    float Rtmp[9];
    if (parse_csv_floats(p, Rtmp, 9) != 9) return -1;
    for (int i = 0; i < 9; ++i) if (!isfinite(Rtmp[i])) return -1;
    memcpy(R_out, Rtmp, sizeof(Rtmp));

    p = ini_find_value(buf, "cam_offset_B");
    if (p) {
        float otmp[3];
        if (parse_csv_floats(p, otmp, 3) == 3 &&
            isfinite(otmp[0]) && isfinite(otmp[1]) && isfinite(otmp[2])) {
            memcpy(cam_off_out, otmp, sizeof(otmp));
        }
    }

    // Kp keys — optional (forward-compat: a schema-v2 file written by
    // T2 firmware lacks them; treat as -1.0f = "not calibrated").
    static const char* const kp_keys[SENTAI_CALIB_AXIS_COUNT] = {
        "kp_x", "kp_y", "kp_yaw"
    };
    for (int i = 0; i < SENTAI_CALIB_AXIS_COUNT; ++i) {
        const char* kp_p = ini_find_value(buf, kp_keys[i]);
        if (!kp_p) continue;   // leave caller's default in place
        float v;
        if (parse_csv_floats(kp_p, &v, 1) == 1 && isfinite(v)) {
            kp_out[i] = v;
        }
    }

    float signs[3] = { s_extpos_signs[0], s_extpos_signs[1], s_extpos_signs[2] };
    if ((p = ini_find_value(buf, "extpos_sign_x"))) (void)parse_one_float(p, &signs[0]);
    if ((p = ini_find_value(buf, "extpos_sign_y"))) (void)parse_one_float(p, &signs[1]);
    if ((p = ini_find_value(buf, "extpos_sign_z"))) (void)parse_one_float(p, &signs[2]);
    memcpy(s_extpos_signs, signs, sizeof(s_extpos_signs));

    float rv[2] = { 0.0f, 0.0f };
    float pv[2] = { 0.0f, 0.0f };
    float rs = s_axis_roll_sign;
    float ps = s_axis_pitch_sign;
    int axis_ok = 1;
    p = ini_find_value(buf, "axis_roll_sign");
    axis_ok = axis_ok && p && parse_one_float(p, &rs);
    p = ini_find_value(buf, "axis_pitch_sign");
    axis_ok = axis_ok && p && parse_one_float(p, &ps);
    p = ini_find_value(buf, "axis_roll_vec_px");
    axis_ok = axis_ok && p && parse_csv_floats(p, rv, 2) == 2;
    p = ini_find_value(buf, "axis_pitch_vec_px");
    axis_ok = axis_ok && p && parse_csv_floats(p, pv, 2) == 2;
    if (axis_ok) {
        s_axis_roll_sign = rs;
        s_axis_pitch_sign = ps;
        memcpy(s_axis_roll_vec_px, rv, sizeof(rv));
        memcpy(s_axis_pitch_vec_px, pv, sizeof(pv));
        s_axis_seed_valid = 1;
    } else {
        s_axis_seed_valid = 0;
    }
    return 0;
}

// File I/O — branches on FxUser availability so the same module compiles
// for both ARM (FileX) and SIM (host stdio).
#if !SENTAI_CALIB_USE_FXUSER
extern "C" const char* sim_fs_root(void) __attribute__((weak));

static FILE* open_calib_file_posix(const char* mode) {
    FILE* f = fopen(SENTAI_CALIB_PATH, mode);
    if (f) return f;

    char path[256];
    const char* root = nullptr;
    if (sim_fs_root) root = sim_fs_root();
    if (!root || !root[0]) root = getenv("SENTAI_SIM_ROOT");
    if (root && root[0] && SENTAI_CALIB_PATH[0] == '/') {
        const int n = snprintf(path, sizeof(path), "%s%s", root,
                               SENTAI_CALIB_PATH);
        if (n > 0 && (size_t)n < sizeof(path)) {
            f = fopen(path, mode);
            if (f) return f;
        }
    }

    if (SENTAI_CALIB_PATH[0] == '/') {
        // Last-resort host fallback for small unit tests launched with cwd
        // equal to the simulated filesystem root.
        const int n = snprintf(path, sizeof(path), ".%s", SENTAI_CALIB_PATH);
        if (n > 0 && (size_t)n < sizeof(path)) {
            f = fopen(path, mode);
            if (f) return f;
        }
    }

    return nullptr;
}
#endif

static int read_calib_file(char* buf, size_t cap) {
#if SENTAI_CALIB_USE_FXUSER
    ssize_t sz = FxUserSize(SENTAI_CALIB_PATH);
    if (sz <= 0 || (size_t)sz >= cap) return 0;
    size_t n = FxUserReadFile(SENTAI_CALIB_PATH, (uint8_t*)buf, cap - 1);
    if (n == 0) return 0;
    buf[n] = '\0';
    return 1;
#else
    FILE* f = open_calib_file_posix("rb");
    if (!f) return 0;
    size_t n = fread(buf, 1, cap - 1, f);
    fclose(f);
    if (n == 0) return 0;
    buf[n] = '\0';
    return 1;
#endif
}

static int write_calib_file(const char* buf, size_t n) {
#if SENTAI_CALIB_USE_FXUSER
    return FxUserWriteFile(SENTAI_CALIB_PATH, (const uint8_t*)buf, n) ? 1 : 0;
#else
    FILE* f = open_calib_file_posix("wb");
    if (!f) return 0;
    const size_t w = fwrite(buf, 1, n, f);
    fclose(f);
    return (w == n) ? 1 : 0;
#endif
}

extern "C" int sentai_calib_save_contract(
        const char* status,
        int accepted,
        float axis_roll_sign,
        const float axis_roll_vec_px[2],
        float axis_pitch_sign,
        const float axis_pitch_vec_px[2]) {
    if (!status || !axis_roll_vec_px || !axis_pitch_vec_px) return 0;
    if (!isfinite(axis_roll_sign) || !isfinite(axis_pitch_sign)) return 0;
    for (int i = 0; i < 2; ++i) {
        if (!isfinite(axis_roll_vec_px[i])) return 0;
        if (!isfinite(axis_pitch_vec_px[i])) return 0;
    }

    const char* roll_axis =
        (fabsf(axis_roll_vec_px[0]) >= fabsf(axis_roll_vec_px[1])) ? "x" : "y";
    const char* pitch_axis =
        (fabsf(axis_pitch_vec_px[0]) >= fabsf(axis_pitch_vec_px[1])) ? "x" : "y";

    char buf[SENTAI_CALIB_INI_MAX_BYTES];
    const int n = snprintf(buf, sizeof(buf),
        "schema=2\n"
        "task_id=TD-S10-B3\n"
        "status=%s\n"
        "accepted=%d\n"
        "layout_id=sentai_whycon_small_centroid_7_marker_v1\n"
        "R_B_C=%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n"
        "cam_offset_B=%.9f,%.9f,%.9f\n"
        "fx=288.3\n"
        "fy=288.3\n"
        "cx=160.0\n"
        "cy=120.0\n"
        "image_w=320\n"
        "image_h=240\n"
        "marker_count=7\n"
        "marker_diameter_m=0.05440001\n"
        "extpos_sign_x=%.6f\n"
        "extpos_sign_y=%.6f\n"
        "extpos_sign_z=%.6f\n"
        "camera_referenced_landing_ok=1\n"
        "disarm_full_markers=4\n"
        "axis_roll_image_axis=%s\n"
        "axis_roll_sign=%.6f\n"
        "axis_roll_vec_px=%.6f,%.6f\n"
        "axis_pitch_image_axis=%s\n"
        "axis_pitch_sign=%.6f\n"
        "axis_pitch_vec_px=%.6f,%.6f\n",
        status,
        accepted ? 1 : 0,
        s_R[0], s_R[1], s_R[2],
        s_R[3], s_R[4], s_R[5],
        s_R[6], s_R[7], s_R[8],
        s_cam_offset_B[0], s_cam_offset_B[1], s_cam_offset_B[2],
        s_extpos_signs[0], s_extpos_signs[1], s_extpos_signs[2],
        roll_axis,
        axis_roll_sign,
        axis_roll_vec_px[0], axis_roll_vec_px[1],
        pitch_axis,
        axis_pitch_sign,
        axis_pitch_vec_px[0], axis_pitch_vec_px[1]);
    if (n <= 0 || (size_t)n >= sizeof(buf)) return 0;
    if (!write_calib_file(buf, (size_t)n)) return 0;

    s_axis_roll_sign = axis_roll_sign;
    s_axis_pitch_sign = axis_pitch_sign;
    s_axis_roll_vec_px[0] = axis_roll_vec_px[0];
    s_axis_roll_vec_px[1] = axis_roll_vec_px[1];
    s_axis_pitch_vec_px[0] = axis_pitch_vec_px[0];
    s_axis_pitch_vec_px[1] = axis_pitch_vec_px[1];
    s_axis_seed_valid = 1;
    update_ini_status_(buf, &s_ini_status);
    char ev[96];
    snprintf(ev, sizeof(ev),
             "status=%s accepted=%d roll=%.1f pitch=%.1f",
             status, accepted ? 1 : 0,
             (double)axis_roll_sign, (double)axis_pitch_sign);
    sentai_fr_push_event("calib_contract_save", ev);
    return 1;
}

extern "C" int sentai_calib_save(void) {
    char buf[SENTAI_CALIB_INI_MAX_BYTES];
    const int n = format_ini(buf, sizeof(buf), s_R, s_cam_offset_B,
                              s_kp_persisted);
    if (n <= 0 || (size_t)n >= sizeof(buf)) return 0;
    return write_calib_file(buf, (size_t)n);
}

extern "C" int sentai_calib_load(void) {
    char buf[SENTAI_CALIB_INI_MAX_BYTES];
    if (!read_calib_file(buf, sizeof(buf))) {
        memset(&s_ini_status, 0, sizeof(s_ini_status));
        return 0;
    }
    float R[9];
    float cam_off[3];
    float kp[SENTAI_CALIB_AXIS_COUNT];
    memcpy(R,       s_R,            sizeof(R));
    memcpy(cam_off, s_cam_offset_B, sizeof(cam_off));
    memcpy(kp,      s_kp_persisted, sizeof(kp));
    if (parse_ini(buf, R, cam_off, kp) != 0) return 0;
    if (sentai_calib_commit_R(R, cam_off) != 0) return 0;
    // Commit Kp atomically AFTER commit_R succeeds, so a failed R
    // parse never leaks a partial Kp restore.
    memcpy(s_kp_persisted, kp, sizeof(s_kp_persisted));
    return 1;
}

extern "C" void sentai_calib_init(void) {
    sentai_calib_clear();
    // Best-effort load; on miss the defaults stay and is_calibrated=0.
    if (sentai_calib_load()) {
        // Loaded from disk — already committed by load().
        s_is_calibrated = 1;
    }
    // OP-S10-W21-T6 — surface calibration state via the health subsystem.
    // HEALTHY iff calib loaded successfully; UNAVAILABLE pre-bringup
    // ([[sentai-calib-is-production-bringup]]: missions must refuse to
    // take off when uncalibrated, but the boot path NEVER auto-flies).
    if (s_is_calibrated) calib_health_healthy_();
    else                 calib_health_unavailable_();
}

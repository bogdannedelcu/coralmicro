// sentai_calib.cc — Kabsch 3D Procrustes + Jacobi 3x3 SVD + FxUser persist.
// See sentai_calib.h for the full design rationale and fault model.

#include "sentai_calib.h"

#include "sentai_svd3.h"

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
    return 0;
}

extern "C" const float* sentai_calib_get_R_cam_to_body(void) { return s_R; }
extern "C" const float* sentai_calib_get_cam_offset_B(void)   { return s_cam_offset_B; }
extern "C" int          sentai_calib_is_calibrated(void)      { return s_is_calibrated; }

extern "C" void sentai_calib_clear(void) {
    memcpy(s_R, SENTAI_CALIB_DEFAULT_R_SIM, sizeof(s_R));
    memcpy(s_cam_offset_B, SENTAI_CALIB_DEFAULT_CAM_OFFSET_SIM,
           sizeof(s_cam_offset_B));
    s_is_calibrated = 0;
}

// =========================================================================
// Persistence — schema v1 JSON.  Tiny hand-rolled parser/serializer; the
// file is < 256 bytes and we control both ends.
// =========================================================================
static int format_json(char* buf, size_t cap,
                       const float R[9], const float cam_off[3]) {
    int n = snprintf(buf, cap,
        "{\n"
        "  \"schema\": %d,\n"
        "  \"R_B_C\": [\n"
        "    [%.9f, %.9f, %.9f],\n"
        "    [%.9f, %.9f, %.9f],\n"
        "    [%.9f, %.9f, %.9f]\n"
        "  ],\n"
        "  \"cam_offset_B\": [%.9f, %.9f, %.9f]\n"
        "}\n",
        SENTAI_CALIB_SCHEMA_VERSION,
        R[0], R[1], R[2],
        R[3], R[4], R[5],
        R[6], R[7], R[8],
        cam_off[0], cam_off[1], cam_off[2]);
    return n;
}

// Scan past one JSON key like "schema": ... and return a pointer to the
// value character; nullptr if not found.  Caller has already null-terminated.
static const char* find_key(const char* s, const char* key) {
    char needle[64];
    int n = snprintf(needle, sizeof(needle), "\"%s\"", key);
    if (n <= 0 || (size_t)n >= sizeof(needle)) return NULL;
    const char* p = strstr(s, needle);
    if (!p) return NULL;
    p += n;
    while (*p && (*p == ' ' || *p == '\t' || *p == ':' || *p == '\n' ||
                  *p == '\r')) p++;
    return p;
}

// Read N floats from a "[a, b, c]"-style array starting at p.  Skips
// nested brackets so "[[1,2,3],[4,5,6]]" feeds a flat 6-vector.
static int read_floats(const char* p, float* out, int N) {
    int i = 0;
    while (*p && i < N) {
        if (*p == '[' || *p == ',' || *p == ' ' || *p == '\t' ||
            *p == '\n' || *p == '\r') {
            p++;
            continue;
        }
        if (*p == ']') { p++; continue; }
        char*  end = NULL;
        double v   = strtod(p, &end);
        if (end == p) return i;
        if (!isfinite((float)v)) return -1;
        out[i++] = (float)v;
        p = end;
    }
    return i;
}

static int parse_json(const char* buf, float R_out[9], float cam_off_out[3]) {
    const char* p = find_key(buf, "schema");
    if (!p) return -1;
    char* end = NULL;
    long  schema = strtol(p, &end, 10);
    if (end == p || schema != SENTAI_CALIB_SCHEMA_VERSION) return -1;

    p = find_key(buf, "R_B_C");
    if (!p) return -1;
    float Rtmp[9];
    if (read_floats(p, Rtmp, 9) != 9) return -1;
    for (int i = 0; i < 9; ++i) if (!isfinite(Rtmp[i])) return -1;
    memcpy(R_out, Rtmp, sizeof(Rtmp));

    p = find_key(buf, "cam_offset_B");
    if (p) {
        float otmp[3];
        if (read_floats(p, otmp, 3) == 3 &&
            isfinite(otmp[0]) && isfinite(otmp[1]) && isfinite(otmp[2])) {
            memcpy(cam_off_out, otmp, sizeof(otmp));
        }
    }
    return 0;
}

// File I/O — branches on FxUser availability so the same module compiles
// for both ARM (FileX) and SIM (host stdio).
static int read_calib_file(char* buf, size_t cap) {
#if SENTAI_CALIB_USE_FXUSER
    ssize_t sz = FxUserSize(SENTAI_CALIB_PATH);
    if (sz <= 0 || (size_t)sz >= cap) return 0;
    size_t n = FxUserReadFile(SENTAI_CALIB_PATH, (uint8_t*)buf, cap - 1);
    if (n == 0) return 0;
    buf[n] = '\0';
    return 1;
#else
    FILE* f = fopen(SENTAI_CALIB_PATH, "rb");
    if (!f) {
        // SIM-side fallback to a local cwd file so smoke tests can run
        // without /system being mounted.
        f = fopen("./cam_calib.json", "rb");
        if (!f) return 0;
    }
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
    FILE* f = fopen(SENTAI_CALIB_PATH, "wb");
    if (!f) {
        f = fopen("./cam_calib.json", "wb");
        if (!f) return 0;
    }
    const size_t w = fwrite(buf, 1, n, f);
    fclose(f);
    return (w == n) ? 1 : 0;
#endif
}

extern "C" int sentai_calib_save(void) {
    char buf[512];
    const int n = format_json(buf, sizeof(buf), s_R, s_cam_offset_B);
    if (n <= 0 || (size_t)n >= sizeof(buf)) return 0;
    return write_calib_file(buf, (size_t)n);
}

extern "C" int sentai_calib_load(void) {
    char buf[512];
    if (!read_calib_file(buf, sizeof(buf))) return 0;
    float R[9];
    float cam_off[3];
    memcpy(R,       s_R,            sizeof(R));
    memcpy(cam_off, s_cam_offset_B, sizeof(cam_off));
    if (parse_json(buf, R, cam_off) != 0) return 0;
    if (sentai_calib_commit_R(R, cam_off) != 0) return 0;
    return 1;
}

extern "C" void sentai_calib_init(void) {
    sentai_calib_clear();
    // Best-effort load; on miss the defaults stay and is_calibrated=0.
    if (sentai_calib_load()) {
        // Loaded from disk — already committed by load().
        s_is_calibrated = 1;
    }
}

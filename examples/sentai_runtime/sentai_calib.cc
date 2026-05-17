// sentai_calib.cc — Kabsch 3D Procrustes + Jacobi 3x3 SVD + FxUser persist.
// See sentai_calib.h for the full design rationale and fault model.

#include "sentai_calib.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FxUser persistence is only available in the ARM firmware build.  On
// the SIM (and host bring-up) we fall back to a stdio-backed shim that
// reads/writes the schema-versioned JSON without going through FileX.
#if defined(SENTAI_HAVE_FXUSER) || defined(__ARM_ARCH)
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
// Linear algebra helpers (3x3 only, hand-rolled — CMSIS-DSP overkill).
// All matrices are row-major: M[i*3 + j] = M(row=i, col=j).
// =========================================================================
static inline float det3(const float M[9]) {
    return  M[0] * (M[4] * M[8] - M[5] * M[7])
          - M[1] * (M[3] * M[8] - M[5] * M[6])
          + M[2] * (M[3] * M[7] - M[4] * M[6]);
}

static void mat3_mul(const float A[9], const float B[9], float C[9]) {
    float tmp[9];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += A[i*3 + k] * B[k*3 + j];
            tmp[i*3 + j] = s;
        }
    }
    memcpy(C, tmp, sizeof(tmp));
}

static void mat3_mul_T_left(const float A[9], const float B[9], float C[9]) {
    // C = A^T * B
    float tmp[9];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += A[k*3 + i] * B[k*3 + j];
            tmp[i*3 + j] = s;
        }
    }
    memcpy(C, tmp, sizeof(tmp));
}

static void mat3_transpose(const float A[9], float AT[9]) {
    float tmp[9];
    tmp[0] = A[0]; tmp[1] = A[3]; tmp[2] = A[6];
    tmp[3] = A[1]; tmp[4] = A[4]; tmp[5] = A[7];
    tmp[6] = A[2]; tmp[7] = A[5]; tmp[8] = A[8];
    memcpy(AT, tmp, sizeof(tmp));
}

static void mat3_identity(float M[9]) {
    M[0] = 1.0f; M[1] = 0.0f; M[2] = 0.0f;
    M[3] = 0.0f; M[4] = 1.0f; M[5] = 0.0f;
    M[6] = 0.0f; M[7] = 0.0f; M[8] = 1.0f;
}

static inline float vec3_dot(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static inline float vec3_norm(const float v[3]) {
    return sqrtf(vec3_dot(v, v));
}

static inline void vec3_cross(const float a[3], const float b[3], float r[3]) {
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

// =========================================================================
// Jacobi eigen-decomposition of a 3x3 symmetric matrix.
// On exit:  A_initial = V * diag(d) * V^T  (within JACOBI_EPS).
// A is destroyed; V holds the eigenvectors as columns; d the eigenvalues.
// Eigenpairs are sorted in DESCENDING order of |d| so the smallest σ²
// lands in slot 2 — that is the position we apply the Kabsch sign flip.
// =========================================================================
static int jacobi_sym3(float A[9], float V[9], float d[3]) {
    mat3_identity(V);

    for (int sweep = 0; sweep < SENTAI_CALIB_JACOBI_MAX_SWEEPS; ++sweep) {
        // Find the largest |off-diagonal|: (0,1), (0,2), (1,2).
        float a01 = fabsf(A[0*3 + 1]);
        float a02 = fabsf(A[0*3 + 2]);
        float a12 = fabsf(A[1*3 + 2]);
        int   p, q;
        if (a01 >= a02 && a01 >= a12) { p = 0; q = 1; }
        else if (a02 >= a12)          { p = 0; q = 2; }
        else                          { p = 1; q = 2; }
        const float apq = A[p*3 + q];
        if (fabsf(apq) < SENTAI_CALIB_JACOBI_EPS) {
            // Converged — off-diagonals are negligible.
            break;
        }

        // Compute the Givens rotation (c, s) that zeroes A[p, q].
        const float app = A[p*3 + p];
        const float aqq = A[q*3 + q];
        float t;
        if (fabsf(aqq - app) < SENTAI_CALIB_JACOBI_EPS) {
            // app == aqq: rotation is 45 degrees.
            t = (apq >= 0.0f) ? 1.0f : -1.0f;
        } else {
            const float theta = (aqq - app) / (2.0f * apq);
            float       sgn   = (theta >= 0.0f) ? 1.0f : -1.0f;
            t = sgn / (fabsf(theta) + sqrtf(1.0f + theta * theta));
        }
        const float c = 1.0f / sqrtf(1.0f + t * t);
        const float s = t * c;

        // Apply: A -> G^T * A * G  (only rows/cols p, q change).
        const float new_app = app - t * apq;
        const float new_aqq = aqq + t * apq;
        A[p*3 + p] = new_app;
        A[q*3 + q] = new_aqq;
        A[p*3 + q] = 0.0f;
        A[q*3 + p] = 0.0f;

        for (int r = 0; r < 3; ++r) {
            if (r == p || r == q) continue;
            const float arp = A[r*3 + p];
            const float arq = A[r*3 + q];
            const float new_arp = c * arp - s * arq;
            const float new_arq = s * arp + c * arq;
            A[r*3 + p] = new_arp;
            A[p*3 + r] = new_arp;   // symmetry
            A[r*3 + q] = new_arq;
            A[q*3 + r] = new_arq;
        }

        // Update V (columns p, q rotate the same way).
        for (int r = 0; r < 3; ++r) {
            const float vrp = V[r*3 + p];
            const float vrq = V[r*3 + q];
            V[r*3 + p] = c * vrp - s * vrq;
            V[r*3 + q] = s * vrp + c * vrq;
        }
    }

    d[0] = A[0*3 + 0];
    d[1] = A[1*3 + 1];
    d[2] = A[2*3 + 2];

    // Sort eigenpairs descending by |d_i|.  Swap helpers operate on
    // column i of V (the eigenvector for d_i).
    auto swap_pair = [](float V_[9], float d_[3], int i, int j) {
        float td = d_[i]; d_[i] = d_[j]; d_[j] = td;
        for (int r = 0; r < 3; ++r) {
            float tv = V_[r*3 + i];
            V_[r*3 + i] = V_[r*3 + j];
            V_[r*3 + j] = tv;
        }
    };
    if (fabsf(d[0]) < fabsf(d[1])) swap_pair(V, d, 0, 1);
    if (fabsf(d[0]) < fabsf(d[2])) swap_pair(V, d, 0, 2);
    if (fabsf(d[1]) < fabsf(d[2])) swap_pair(V, d, 1, 2);

    // Sanity check: the largest |off-diagonal| must be below threshold.
    const float a01 = fabsf(A[0*3 + 1]);
    const float a02 = fabsf(A[0*3 + 2]);
    const float a12 = fabsf(A[1*3 + 2]);
    const float worst = (a01 > a02) ? ((a01 > a12) ? a01 : a12)
                                    : ((a02 > a12) ? a02 : a12);
    if (worst > 1e-3f * (fabsf(d[0]) + fabsf(d[1]) + fabsf(d[2]) + 1e-12f)) {
        return -1;   // did not converge to tolerance
    }
    return 0;
}

// =========================================================================
// SVD of a 3x3 matrix via H^T*H eigen-decomp.
//   H = U * diag(s) * V^T
// Caller receives U (3x3, row-major), s (3 floats, descending), Vt (3x3).
// Handles degenerate (near-zero) singular values by deriving the
// corresponding left singular vector as the cross product of the
// other two, keeping det(U) positive.
// =========================================================================
static int svd3(const float H[9], float U[9], float s[3], float Vt[9]) {
    // A = H^T * H  (3x3 symmetric).
    float A[9];
    mat3_mul_T_left(H, H, A);

    float V[9];
    float d[3];
    if (jacobi_sym3(A, V, d) != 0) return -1;

    // Singular values.
    for (int i = 0; i < 3; ++i) {
        if (d[i] < 0.0f) d[i] = 0.0f;   // numerical safety
        s[i] = sqrtf(d[i]);
    }

    // U columns: u_i = H * v_i / s_i, with degeneracy handling.
    int   zero_count = 0;
    int   zero_idx   = -1;
    const float s_max  = s[0];
    const float s_tol  = (s_max > 1e-6f) ? (1e-6f * s_max) : 1e-9f;
    for (int i = 0; i < 3; ++i) {
        if (s[i] < s_tol) {
            zero_count++;
            zero_idx = i;
            continue;
        }
        float v[3] = { V[0*3 + i], V[1*3 + i], V[2*3 + i] };
        float u[3];
        u[0] = H[0]*v[0] + H[1]*v[1] + H[2]*v[2];
        u[1] = H[3]*v[0] + H[4]*v[1] + H[5]*v[2];
        u[2] = H[6]*v[0] + H[7]*v[1] + H[8]*v[2];
        const float inv = 1.0f / s[i];
        U[0*3 + i] = u[0] * inv;
        U[1*3 + i] = u[1] * inv;
        U[2*3 + i] = u[2] * inv;
    }
    if (zero_count == 1) {
        // Fill the missing column as the cross product of the other two
        // (keeps U orthonormal; sign chosen to make det(U)=+1).
        int i0 = (zero_idx + 1) % 3;
        int i1 = (zero_idx + 2) % 3;
        float u0[3] = { U[0*3 + i0], U[1*3 + i0], U[2*3 + i0] };
        float u1[3] = { U[0*3 + i1], U[1*3 + i1], U[2*3 + i1] };
        float cr[3];
        vec3_cross(u0, u1, cr);
        // det(U) = sign(cr_z) when columns ordered (i0, i1, zero_idx)
        // becomes the orientation of the third column.  Match positive
        // orientation for cyclic ordering (i0, i1, zero_idx) = parity ±1.
        const int parity = ((zero_idx == 0) || (zero_idx == 2)) ? 1 : -1;
        const float sgn = (parity > 0) ? 1.0f : -1.0f;
        U[0*3 + zero_idx] = sgn * cr[0];
        U[1*3 + zero_idx] = sgn * cr[1];
        U[2*3 + zero_idx] = sgn * cr[2];
    } else if (zero_count >= 2) {
        // Severely degenerate (rank 0 or 1); fall back to identity to
        // keep the contract that U is always orthonormal on return.
        mat3_identity(U);
    }

    // Vt = V^T (V has eigenvectors as columns, Vt has them as rows).
    mat3_transpose(V, Vt);
    return 0;
}

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
    mat3_mul_T_left(R1, R2, M);
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
    mat3_identity(R_out);
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
    if (svd3(H, U, s, Vt) != 0) {
        q_out->reject_code = SENTAI_CALIB_REJ_SVD_NO_CV;
        return -SENTAI_CALIB_REJ_SVD_NO_CV;
    }

    // d = sign(det(U * Vt)) — controls the reflection-vs-rotation flip
    // applied to the smallest singular value (column 2 after sorting).
    float UVt[9];
    mat3_mul(U, Vt, UVt);
    const float det_UVt = det3(UVt);
    const float dflip   = (det_UVt >= 0.0f) ? 1.0f : -1.0f;

    // R = U * diag(1, 1, dflip) * Vt.
    // Equivalent: scale column 2 of U by dflip, then R = U * Vt.
    float U2[9];
    memcpy(U2, U, sizeof(U2));
    U2[0*3 + 2] *= dflip;
    U2[1*3 + 2] *= dflip;
    U2[2*3 + 2] *= dflip;

    float R[9];
    mat3_mul(U2, Vt, R);
    memcpy(R_out, R, sizeof(R));
    const float det_R = det3(R);
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
        const float na = vec3_norm(Rcam);
        const float nb = vec3_norm(body_raw);
        if (na < 1e-6f || nb < 1e-6f) continue;
        float cos_t = vec3_dot(Rcam, body_raw) / (na * nb);
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

// sentai_svd3.h -- Shared 3x3 linear algebra: mat/vec ops, Jacobi eigen,
// SVD, and a 3D-3D rigid-body Kabsch aligner (R + t).
//
// All matrices are row-major: M[i*3 + j] = M(row=i, col=j).
// All vectors are 3 floats.  Float-only -- float32 is sufficient for the
// 3x3 condition numbers seen in marker pose recovery (ranges 0.01..1.0 m)
// and avoids pulling in double math libs on the M7 .ramfunc path.
//
// =========================================================================
// SYSTEM MODEL (per agent/embeded.md §A)
// =========================================================================
// Fault model:
//   F1  jacobi_sym3 fails to converge within JACOBI_MAX_SWEEPS  -> -1
//   F2  svd3 inherits F1                                        -> -1
//   F3  kabsch_align with n<3                                   -> -1
//   F4  kabsch_align inputs contain non-finite floats           -> -1
//   F5  kabsch_align SVD fails (collinear / degenerate cloud)   -> -1
//
// Execution model:
//   - Pure compute, stateless, no globals, no FreeRTOS primitives.
//   - Caller-allocated outputs; zero heap, zero per-call malloc.
//   - Re-entrant from any task; safe to call from ISR if .ramfunc'd
//     (current cost ~50 us for SVD, ~80 us for full Kabsch on 6 points).
//
// Recovery:
//   - All errors local — non-zero return; outputs are not modified on
//     error (caller's buffers stay at whatever they were before).
// =========================================================================

#pragma once

#include <math.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Tunables ---------------------------------------------------------
#define SENTAI_SVD3_JACOBI_MAX_SWEEPS  16      // 3x3 converges in ~3-5
#define SENTAI_SVD3_JACOBI_EPS         1e-9f   // off-diagonal threshold

// ---- 3x3 matrix helpers (inline, no call overhead) --------------------

static inline float sentai_mat3_det(const float M[9]) {
    return  M[0] * (M[4] * M[8] - M[5] * M[7])
          - M[1] * (M[3] * M[8] - M[5] * M[6])
          + M[2] * (M[3] * M[7] - M[4] * M[6]);
}

static inline void sentai_mat3_identity(float M[9]) {
    M[0] = 1.0f; M[1] = 0.0f; M[2] = 0.0f;
    M[3] = 0.0f; M[4] = 1.0f; M[5] = 0.0f;
    M[6] = 0.0f; M[7] = 0.0f; M[8] = 1.0f;
}

static inline void sentai_mat3_mul(const float A[9], const float B[9], float C[9]) {
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

// C = A^T * B
static inline void sentai_mat3_mul_AtB(const float A[9], const float B[9], float C[9]) {
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

static inline void sentai_mat3_transpose(const float A[9], float AT[9]) {
    float tmp[9];
    tmp[0] = A[0]; tmp[1] = A[3]; tmp[2] = A[6];
    tmp[3] = A[1]; tmp[4] = A[4]; tmp[5] = A[7];
    tmp[6] = A[2]; tmp[7] = A[5]; tmp[8] = A[8];
    memcpy(AT, tmp, sizeof(tmp));
}

// y = M * x (3x3 row-major M, 3-vec x, 3-vec y).  In-place if y == x.
static inline void sentai_mat3_vec(const float M[9], const float x[3], float y[3]) {
    float t0 = M[0]*x[0] + M[1]*x[1] + M[2]*x[2];
    float t1 = M[3]*x[0] + M[4]*x[1] + M[5]*x[2];
    float t2 = M[6]*x[0] + M[7]*x[1] + M[8]*x[2];
    y[0] = t0; y[1] = t1; y[2] = t2;
}

// ---- 3-vec helpers ----------------------------------------------------

static inline float sentai_vec3_dot(const float a[3], const float b[3]) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline float sentai_vec3_norm(const float v[3]) {
    return sqrtf(sentai_vec3_dot(v, v));
}

static inline void sentai_vec3_cross(const float a[3], const float b[3], float r[3]) {
    r[0] = a[1]*b[2] - a[2]*b[1];
    r[1] = a[2]*b[0] - a[0]*b[2];
    r[2] = a[0]*b[1] - a[1]*b[0];
}

// ---- Jacobi 3x3 symmetric eigen ---------------------------------------
// Input:   A[9] symmetric 3x3, row-major.  A is DESTROYED on return.
// Output:  V[9] eigenvectors as columns; d[3] eigenvalues, sorted by
//          DESCENDING |d_i| (smallest in slot 2 — that's the position the
//          reflection-safe Kabsch flip is applied to).
// Returns: 0 on success, -1 if the largest |off-diagonal| at exit
//          exceeds 1e-3 * trace_abs (did not converge).
int sentai_jacobi_sym3(float A[9], float V[9], float d[3]);

// ---- 3x3 SVD ----------------------------------------------------------
// H = U * diag(s) * V^T, computed via eigen-decomp of H^T*H.
// Output:  U (3x3 row-major), s (descending, >= 0), Vt (V^T row-major).
// Handles single-singular-value degeneracy by deriving the missing left
// singular vector as the cross product of the other two (keeps det(U)=+1).
// Returns: 0 on success, -1 if the underlying Jacobi diverges.
int sentai_svd3(const float H[9], float U[9], float s[3], float Vt[9]);

// ---- 3D-3D rigid-body Kabsch aligner ----------------------------------
// Best-fit R, t such that  R @ a_i + t  ~=  b_i  (minimising sum sq.).
// This is the closed-form Procrustes solver of Kabsch (1976) / Arun,
// Huang, Blostein (1987), with the Umeyama (1991) reflection-safe det
// correction applied so R is always a proper rotation (det(R) = +1).
//
// Inputs:
//   a_xyz_n3, b_xyz_n3  - flat N x 3 arrays of float, row-major
//   n                   - number of points (n >= 3 required)
//
// Outputs (caller-allocated):
//   R_out[9]            - rotation matrix, row-major
//   t_out[3]            - translation vector
//   res_max_out         - (optional, may be NULL) max per-point
//                         residual ||R @ a_i + t - b_i|| in input units
//
// Returns 0 on success, negative on:
//   -1  n < 3
//   -2  inputs contain non-finite floats
//   -3  SVD diverged (likely collinear / degenerate cloud)
//
// On error R_out/t_out/res_max_out are left untouched.
//
// Cost: O(n) for centroid + H accumulation, then ~50 us SVD on 3x3.
// Allocation: zero heap (stack only: 2 floats per point for centred sums
// is folded into the running H computation — see implementation).
int sentai_kabsch_align(const float* a_xyz_n3,
                        const float* b_xyz_n3,
                        int n,
                        float R_out[9],
                        float t_out[3],
                        float* res_max_out);

// ---- N-by-N symmetric Jacobi eigendecomposition (W21-T4d) -------------
// Generalised version of jacobi_sym3 for arbitrary N (typically N=9 for
// DLT homography null-space recovery).
// Input:   A[N*N] symmetric row-major.  A is DESTROYED on return.
// Output:  V[N*N] eigenvectors as columns; d[N] eigenvalues.
//          Sorted by DESCENDING |d_i| (smallest in slot N-1).
// Returns: 0 on success, -1 if did not converge.
// Caller-allocated outputs; zero heap.
// Max N supported is 16 (compile-time scratch in sentai_svd3.cc).
int sentai_jacobi_symN(float* A, int N, float* V, float* d);

// ---- Coplanar multi-marker PnP (W21-T4d) ------------------------------
// Closed-form solver for camera pose given N>=4 coplanar markers at
// known world XY (Z=0 implied) + measured image (px, py) + intrinsics.
// Algorithm: DLT homography null-space via symmetric eigen on A^TA,
// then K^-1 decomposition + 3x3 SVD orthonormalisation.
// Inputs:
//   img_pts_xy_n2  - flat N x 2 array of (px, py)
//   world_pts_xy_n2 - flat N x 2 array of (X_w, Y_w)
//   n              - 4 <= n <= 16
//   fx,fy,cx,cy    - camera intrinsics
// Outputs:
//   cam_world_out[3] - camera centre in WORLD frame
//   R_w2c_out[9]     - row-major R world→cam
//   reproj_max_px_out - (optional, NULL OK) max reproj residual
// Returns 0 on success, -1 if n<4 or solver fails.
int sentai_coplanar_pnp(const float* img_pts_xy_n2,
                        const float* world_pts_xy_n2,
                        int n,
                        float fx, float fy, float cx, float cy,
                        float cam_world_out[3],
                        float R_w2c_out[9],
                        float* reproj_max_px_out);

#ifdef __cplusplus
}  // extern "C"
#endif

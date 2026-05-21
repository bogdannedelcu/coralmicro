// sentai_svd3.cc -- 3x3 Jacobi eigen, SVD, and Kabsch (3D-3D rigid).
// See sentai_svd3.h for the public API + fault model.

#include "sentai_svd3.h"

#include <math.h>
#include <string.h>

// =========================================================================
// Jacobi eigen-decomposition of a 3x3 symmetric matrix.
// On exit:  A_initial = V * diag(d) * V^T  (within JACOBI_EPS).
// A is destroyed; V holds the eigenvectors as columns; d the eigenvalues.
// Eigenpairs are sorted in DESCENDING order of |d| so the smallest sigma^2
// lands in slot 2 -- that is the position the reflection-safe Kabsch flip
// is applied to.
// =========================================================================
extern "C" int sentai_jacobi_sym3(float A[9], float V[9], float d[3]) {
    sentai_mat3_identity(V);

    for (int sweep = 0; sweep < SENTAI_SVD3_JACOBI_MAX_SWEEPS; ++sweep) {
        // Find the largest |off-diagonal|: (0,1), (0,2), (1,2).
        float a01 = fabsf(A[0*3 + 1]);
        float a02 = fabsf(A[0*3 + 2]);
        float a12 = fabsf(A[1*3 + 2]);
        int   p, q;
        if (a01 >= a02 && a01 >= a12) { p = 0; q = 1; }
        else if (a02 >= a12)          { p = 0; q = 2; }
        else                          { p = 1; q = 2; }
        const float apq = A[p*3 + q];
        if (fabsf(apq) < SENTAI_SVD3_JACOBI_EPS) {
            // Converged -- off-diagonals are negligible.
            break;
        }

        // Compute the Givens rotation (c, s) that zeroes A[p, q].
        const float app = A[p*3 + p];
        const float aqq = A[q*3 + q];
        float t;
        if (fabsf(aqq - app) < SENTAI_SVD3_JACOBI_EPS) {
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

    // Sort eigenpairs descending by |d_i|.  swap_pair operates on
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
extern "C" int sentai_svd3(const float H[9], float U[9], float s[3], float Vt[9]) {
    // A = H^T * H  (3x3 symmetric).
    float A[9];
    sentai_mat3_mul_AtB(H, H, A);

    float V[9];
    float d[3];
    if (sentai_jacobi_sym3(A, V, d) != 0) return -1;

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
        sentai_vec3_cross(u0, u1, cr);
        // Match positive orientation for cyclic ordering (i0, i1, zero_idx).
        const int parity = ((zero_idx == 0) || (zero_idx == 2)) ? 1 : -1;
        const float sgn = (parity > 0) ? 1.0f : -1.0f;
        U[0*3 + zero_idx] = sgn * cr[0];
        U[1*3 + zero_idx] = sgn * cr[1];
        U[2*3 + zero_idx] = sgn * cr[2];
    } else if (zero_count >= 2) {
        // Severely degenerate (rank 0 or 1); fall back to identity to
        // keep the contract that U is always orthonormal on return.
        sentai_mat3_identity(U);
    }

    // Vt = V^T (V has eigenvectors as columns, Vt has them as rows).
    sentai_mat3_transpose(V, Vt);
    return 0;
}

// =========================================================================
// 3D-3D rigid Kabsch.  See sentai_svd3.h for the contract + references.
// Pipeline:
//   1. Centroids cA, cB; centred a' = a - cA, b' = b - cB
//   2. H = sum_i b'_i (outer) a'_i  -- b is the LEFT operand because
//      we want R mapping a -> b (H = B_centered @ A_centered^T)
//   3. SVD(H) = U s V^T
//   4. d = sign(det(U V^T))      reflection-safe correction
//   5. R = U * diag(1, 1, d) * V^T
//   6. t = cB - R @ cA
// =========================================================================
extern "C" int sentai_kabsch_align(const float* a_xyz_n3,
                                   const float* b_xyz_n3,
                                   int n,
                                   float R_out[9],
                                   float t_out[3],
                                   float* res_max_out) {
    if (!a_xyz_n3 || !b_xyz_n3 || !R_out || !t_out) return -1;
    if (n < 3) return -1;

    // Pass 1: centroids + finiteness check.
    float cA[3] = { 0.0f, 0.0f, 0.0f };
    float cB[3] = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < n; ++i) {
        const float* a = a_xyz_n3 + 3*i;
        const float* b = b_xyz_n3 + 3*i;
        for (int k = 0; k < 3; ++k) {
            if (!isfinite(a[k]) || !isfinite(b[k])) return -2;
            cA[k] += a[k];
            cB[k] += b[k];
        }
    }
    const float inv_n = 1.0f / (float)n;
    cA[0] *= inv_n; cA[1] *= inv_n; cA[2] *= inv_n;
    cB[0] *= inv_n; cB[1] *= inv_n; cB[2] *= inv_n;

    // Pass 2: accumulate H = sum_i (b_i - cB) (a_i - cA)^T.
    float H[9] = { 0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f,
                   0.0f, 0.0f, 0.0f };
    for (int i = 0; i < n; ++i) {
        const float* a = a_xyz_n3 + 3*i;
        const float* b = b_xyz_n3 + 3*i;
        const float ax = a[0] - cA[0], ay = a[1] - cA[1], az = a[2] - cA[2];
        const float bx = b[0] - cB[0], by = b[1] - cB[1], bz = b[2] - cB[2];
        H[0] += bx*ax; H[1] += bx*ay; H[2] += bx*az;
        H[3] += by*ax; H[4] += by*ay; H[5] += by*az;
        H[6] += bz*ax; H[7] += bz*ay; H[8] += bz*az;
    }

    // SVD.
    float U[9], s[3], Vt[9];
    if (sentai_svd3(H, U, s, Vt) != 0) return -3;

    // Reflection-safe composition: R = U * diag(1, 1, sign(det(U*Vt))) * Vt
    float UVt[9];
    sentai_mat3_mul(U, Vt, UVt);
    const float det_UVt = sentai_mat3_det(UVt);
    const float dflip   = (det_UVt >= 0.0f) ? 1.0f : -1.0f;

    float U2[9];
    memcpy(U2, U, sizeof(U2));
    U2[0*3 + 2] *= dflip;
    U2[1*3 + 2] *= dflip;
    U2[2*3 + 2] *= dflip;

    float R[9];
    sentai_mat3_mul(U2, Vt, R);
    memcpy(R_out, R, sizeof(R));

    // t = cB - R @ cA
    float Rca[3];
    sentai_mat3_vec(R, cA, Rca);
    t_out[0] = cB[0] - Rca[0];
    t_out[1] = cB[1] - Rca[1];
    t_out[2] = cB[2] - Rca[2];

    // Max residual.
    if (res_max_out) {
        float max_r = 0.0f;
        for (int i = 0; i < n; ++i) {
            const float* a = a_xyz_n3 + 3*i;
            const float* b = b_xyz_n3 + 3*i;
            float Ra[3];
            sentai_mat3_vec(R, a, Ra);
            const float ex = Ra[0] + t_out[0] - b[0];
            const float ey = Ra[1] + t_out[1] - b[1];
            const float ez = Ra[2] + t_out[2] - b[2];
            const float r2 = ex*ex + ey*ey + ez*ez;
            if (r2 > max_r) max_r = r2;
        }
        *res_max_out = sqrtf(max_r);
    }

    return 0;
}

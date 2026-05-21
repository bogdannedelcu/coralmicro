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

// =========================================================================
// W21-T4d — N-by-N symmetric Jacobi eigendecomposition.
// =========================================================================
// Same algorithm as sentai_jacobi_sym3 but generalised to N (capped at
// 16).  Each sweep iterates through ALL upper-triangle entries (cyclic
// Jacobi) instead of finding the single largest off-diagonal — for
// N>3 cyclic-by-row is faster than scanning for the global max each
// rotation.  Convergence: ~10-50 sweeps for N=9 in our DLT context.
//
// Caller's outputs are written even on early break (so the partial
// decomposition is usable if the residual off-diagonal is acceptable).
#define SENTAI_JACOBI_SYMN_MAX_N      16
#define SENTAI_JACOBI_SYMN_MAX_SWEEPS 100
#define SENTAI_JACOBI_SYMN_EPS        1e-9f

extern "C" int sentai_jacobi_symN(float* A, int N, float* V, float* d) {
    if (N < 2 || N > SENTAI_JACOBI_SYMN_MAX_N) return -1;
    // Initialise V to identity.
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            V[i*N + j] = (i == j) ? 1.0f : 0.0f;

    int converged = 0;
    for (int sweep = 0; sweep < SENTAI_JACOBI_SYMN_MAX_SWEEPS; ++sweep) {
        // Compute off-diagonal sum-of-squares (convergence test).
        float off = 0.0f;
        for (int p = 0; p < N - 1; ++p) {
            for (int q = p + 1; q < N; ++q) {
                const float apq = A[p*N + q];
                off += apq * apq;
            }
        }
        if (off < SENTAI_JACOBI_SYMN_EPS) { converged = 1; break; }

        // Cyclic Jacobi: iterate all (p, q) pairs.
        for (int p = 0; p < N - 1; ++p) {
            for (int q = p + 1; q < N; ++q) {
                const float apq = A[p*N + q];
                if (fabsf(apq) < SENTAI_JACOBI_SYMN_EPS) continue;
                const float app = A[p*N + p];
                const float aqq = A[q*N + q];

                // Givens rotation angle.
                float t;
                if (fabsf(aqq - app) < SENTAI_JACOBI_SYMN_EPS) {
                    t = (apq >= 0.0f) ? 1.0f : -1.0f;
                } else {
                    const float theta = (aqq - app) / (2.0f * apq);
                    if (theta >= 0.0f) {
                        t = 1.0f / (theta + sqrtf(1.0f + theta * theta));
                    } else {
                        t = 1.0f / (theta - sqrtf(1.0f + theta * theta));
                    }
                }
                const float c = 1.0f / sqrtf(1.0f + t * t);
                const float s = t * c;

                // Update A: rotate rows/cols p, q.
                A[p*N + p] = app - t * apq;
                A[q*N + q] = aqq + t * apq;
                A[p*N + q] = 0.0f;
                A[q*N + p] = 0.0f;
                for (int k = 0; k < N; ++k) {
                    if (k == p || k == q) continue;
                    const float akp = A[k*N + p];
                    const float akq = A[k*N + q];
                    A[k*N + p] = c * akp - s * akq;
                    A[k*N + q] = s * akp + c * akq;
                    A[p*N + k] = A[k*N + p];
                    A[q*N + k] = A[k*N + q];
                }
                // Update V (columns p, q).
                for (int k = 0; k < N; ++k) {
                    const float vkp = V[k*N + p];
                    const float vkq = V[k*N + q];
                    V[k*N + p] = c * vkp - s * vkq;
                    V[k*N + q] = s * vkp + c * vkq;
                }
            }
        }
    }

    // Extract eigenvalues from diagonal of (now-diagonal) A.
    for (int i = 0; i < N; ++i) d[i] = A[i*N + i];

    // Sort by |d| descending (so smallest |eigval| lands in slot N-1 —
    // matches sentai_jacobi_sym3 convention; this is the null-space).
    for (int i = 0; i < N - 1; ++i) {
        int max_j = i;
        for (int j = i + 1; j < N; ++j) {
            if (fabsf(d[j]) > fabsf(d[max_j])) max_j = j;
        }
        if (max_j != i) {
            float tmp = d[i]; d[i] = d[max_j]; d[max_j] = tmp;
            // Swap columns i and max_j of V.
            for (int k = 0; k < N; ++k) {
                float v = V[k*N + i];
                V[k*N + i] = V[k*N + max_j];
                V[k*N + max_j] = v;
            }
        }
    }
    return converged ? 0 : -1;
}

// =========================================================================
// W21-T4d — Coplanar multi-marker PnP via DLT homography + decomposition.
// =========================================================================
// Algorithm:
//   1. For each (Mx, My) ↔ (px, py):
//        row 2i:   [Mx, My, 1,  0,  0, 0,  -px*Mx, -px*My, -px]
//        row 2i+1: [ 0,  0, 0, Mx, My, 1,  -py*Mx, -py*My, -py]
//   2. A^T A is 9x9 symmetric.  Smallest-eigenvalue eigenvector of A^T A
//      is the homography H (3x3) up to scale.
//   3. M = K^-1 @ H; columns are [r1 r2 t] up to a scalar lambda.
//      lambda = 1 / |m1|; sign chosen so t_z > 0.
//   4. r3 = r1 × r2.  Orthonormalise R via 3x3 SVD (Procrustes).
//   5. cam_world = -R^T @ t.
// =========================================================================
extern "C" int sentai_coplanar_pnp(const float* img_pts_xy_n2,
                                     const float* world_pts_xy_n2,
                                     int n,
                                     float fx, float fy, float cx, float cy,
                                     float cam_world_out[3],
                                     float R_w2c_out[9],
                                     float* reproj_max_px_out) {
    if (n < 4 || n > SENTAI_JACOBI_SYMN_MAX_N) return -1;

    // Build A^T A directly (avoid storing 2N x 9 A).
    // A^T A = sum over rows of A: a_i a_i^T (outer product, accumulate).
    float AtA[9 * 9] = {0};
    for (int i = 0; i < n; ++i) {
        const float Mx = world_pts_xy_n2[2*i + 0];
        const float My = world_pts_xy_n2[2*i + 1];
        const float px = img_pts_xy_n2[2*i + 0];
        const float py = img_pts_xy_n2[2*i + 1];
        // Two rows per correspondence.
        const float row0[9] = {Mx, My, 1.0f,  0.0f, 0.0f, 0.0f,
                                -px*Mx, -px*My, -px};
        const float row1[9] = {0.0f, 0.0f, 0.0f,  Mx, My, 1.0f,
                                -py*Mx, -py*My, -py};
        for (int a = 0; a < 9; ++a) {
            for (int b = 0; b < 9; ++b) {
                AtA[a*9 + b] += row0[a] * row0[b] + row1[a] * row1[b];
            }
        }
    }

    // Eigendecompose A^T A: smallest eigenvalue's eigenvector = H.
    float V[9 * 9];
    float d[9];
    int jrc = sentai_jacobi_symN(AtA, 9, V, d);
    if (jrc != 0) return -1;
    // Smallest |eigval| is in slot 8 (last) per the sort order.
    float H[9];
    for (int k = 0; k < 9; ++k) H[k] = V[k*9 + 8];

    // M = K^-1 H.  K is upper-triangular so K^-1 is easy:
    //   K^-1 = [[1/fx,  0,    -cx/fx],
    //           [0,     1/fy, -cy/fy],
    //           [0,     0,     1    ]]
    const float inv_fx = 1.0f / fx;
    const float inv_fy = 1.0f / fy;
    float M[9];
    for (int col = 0; col < 3; ++col) {
        const float h0 = H[0*3 + col];
        const float h1 = H[1*3 + col];
        const float h2 = H[2*3 + col];
        M[0*3 + col] = inv_fx * h0 - (cx * inv_fx) * h2;
        M[1*3 + col] = inv_fy * h1 - (cy * inv_fy) * h2;
        M[2*3 + col] = h2;
    }

    // Columns m1, m2, t.  Normalise so |m1| = 1.
    const float m1x = M[0], m1y = M[3], m1z = M[6];
    const float m1_norm = sqrtf(m1x*m1x + m1y*m1y + m1z*m1z);
    if (m1_norm < 1e-9f) return -1;
    float lam = 1.0f / m1_norm;

    float r1[3] = { m1x*lam, m1y*lam, m1z*lam };
    float r2[3] = { M[1]*lam, M[4]*lam, M[7]*lam };
    float t[3]  = { M[2]*lam, M[5]*lam, M[8]*lam };

    // Sign: ensure camera is ABOVE the marker plane (t_z > 0).
    // t_z is the camera Z coord IN THE PLANE FRAME — must be > 0 for
    // a physically meaningful solution.
    if (t[2] < 0.0f) {
        r1[0] = -r1[0]; r1[1] = -r1[1]; r1[2] = -r1[2];
        r2[0] = -r2[0]; r2[1] = -r2[1]; r2[2] = -r2[2];
        t[0]  = -t[0];  t[1]  = -t[1];  t[2]  = -t[2];
    }
    // r3 = r1 × r2.
    float r3[3] = {
        r1[1]*r2[2] - r1[2]*r2[1],
        r1[2]*r2[0] - r1[0]*r2[2],
        r1[0]*r2[1] - r1[1]*r2[0],
    };

    // Build R_world_to_cam = [r1 | r2 | r3] (column-major into row-major).
    float R[9] = {
        r1[0], r2[0], r3[0],
        r1[1], r2[1], r3[1],
        r1[2], r2[2], r3[2],
    };

    // Orthonormalise R via 3x3 SVD: R_ortho = U @ V^T (Procrustes).
    float U[9], sv[3], Vt[9];
    if (sentai_svd3(R, U, sv, Vt) != 0) return -1;
    // R_ortho = U @ Vt.  Use sentai_mat3_mul.
    float R_ortho[9];
    sentai_mat3_mul(U, Vt, R_ortho);
    // Reflection-safe: if det < 0, flip sign of U's last column.
    if (sentai_mat3_det(R_ortho) < 0.0f) {
        U[2] = -U[2]; U[5] = -U[5]; U[8] = -U[8];
        sentai_mat3_mul(U, Vt, R_ortho);
    }

    // Copy R out.
    for (int k = 0; k < 9; ++k) R_w2c_out[k] = R_ortho[k];

    // cam_in_world = -R^T @ t.
    cam_world_out[0] = -(R_ortho[0]*t[0] + R_ortho[3]*t[1] + R_ortho[6]*t[2]);
    cam_world_out[1] = -(R_ortho[1]*t[0] + R_ortho[4]*t[1] + R_ortho[7]*t[2]);
    cam_world_out[2] = -(R_ortho[2]*t[0] + R_ortho[5]*t[1] + R_ortho[8]*t[2]);

    // Reproj residual (optional).
    if (reproj_max_px_out) {
        float max_r2 = 0.0f;
        for (int i = 0; i < n; ++i) {
            const float Mx = world_pts_xy_n2[2*i + 0];
            const float My = world_pts_xy_n2[2*i + 1];
            // pred = H @ [Mx, My, 1]
            const float u = H[0]*Mx + H[1]*My + H[2];
            const float v = H[3]*Mx + H[4]*My + H[5];
            const float w = H[6]*Mx + H[7]*My + H[8];
            if (fabsf(w) < 1e-9f) continue;
            const float up = u / w;
            const float vp = v / w;
            const float du = up - img_pts_xy_n2[2*i + 0];
            const float dv = vp - img_pts_xy_n2[2*i + 1];
            const float r2 = du*du + dv*dv;
            if (r2 > max_r2) max_r2 = r2;
        }
        *reproj_max_px_out = sqrtf(max_r2);
    }
    return 0;
}

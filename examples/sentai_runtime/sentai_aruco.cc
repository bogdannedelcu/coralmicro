// sentai_aruco.cc — on-board ArUco detector + PnP.
// See sentai_aruco.h for the API + design rationale.
//
// Pipeline (per OP-S6-W3 T2..T4, all in this file):
//   T2.A  Adaptive threshold (7x7 mean - C) via integral image (O(1)/px)
//   T2.B  Single-pass connected-component labeling (4-conn flood fill)
//         + bounding-box / pixel-count per component
//   T3.A  Quadrilateral corner extraction from each large component
//         (max-distance-from-centroid in 4 angular sectors)
//   T3.B  4x4_50 dictionary lookup + 4-rotation hamming match
//   T4    IPPE planar-marker PnP (analytical 2-solution, pick by reproj)
//
// All compute lives in C/C++ — per CLAUDE.md the MP binding only sees
// resulting scalars.  All buffers live in .sdram_bss (cold-path SDRAM).
// No heap, no per-call malloc, no FreeRTOS primitives.
//
// Numerical reference: aruco_bench.cc (s111) measured the threshold
// kernel at 1.1 ms on M7 via PXP+SIMD; the scalar version below is
// ~10 ms on M7 (acceptable for the calib takeoff one-shot at OP-S6-W1,
// future optimisation work).  SIM build runs scalar at <1 ms thanks
// to x86 caches.

#include "sentai_aruco.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>     // fopen/fprintf for s175 detect_pgm_file test
#include <string.h>

// =========================================================================
// Compile-time configuration.
// =========================================================================
#define ARUCO_MAX_W                    640
#define ARUCO_MAX_H                    480
#define ARUCO_BUF_SZ                   (ARUCO_MAX_W * ARUCO_MAX_H)
// Adaptive threshold parameters.  block_size must be LARGER than the
// expected marker side so the local box-mean is dominated by the
// background — otherwise the uniform-dark marker interior reads as
// "mean = dark" and the threshold fails to fire.
//
// Empirical sweep (EXP-s160, 2026-05-17): with a 0.08 m marker, fx=240
// on 320x240 images, the projected marker side spans 192 px @ z=0.3 m
// down to 24 px @ z=1.0 m.  block=151 catches z≥0.7 but fails at
// z≤0.5 (block ≲ marker).  block=201 covers markers up to ~190 px,
// which spans our entire useful altitude range (0.3-1.5 m).
// Default threshold block size (T18-G provides multi-scale at runtime;
// this constant remains as a fallback / single-scale path reference).
#define ARUCO_THRESH_BLOCK             201
#define ARUCO_THRESH_C                 7
// Component-labeling capacity.  More components than this and we drop
// the tail (overflow counter rises).
#define ARUCO_MAX_COMPONENTS           96
// Flood-fill stack depth.  Worst-case a fully connected blob; bounded
// by image size, but in practice 4096 is generous for 320x240.
#define ARUCO_FILL_STACK_SZ            4096
// Marker patch dimensions for bit decoding.  ArUco 4x4 = 4x4 data
// inner with 1-bit black border on each side, sampled on a 6x6 grid.
#define ARUCO_PATCH_DIM                6

// Reprojection accept gate (already in header but used locally).
#ifndef ARUCO_REPROJ_GATE_PX
#define ARUCO_REPROJ_GATE_PX           SENTAI_ARUCO_REPROJ_MAX_PX
#endif

// =========================================================================
// Static buffers — .sdram_bss so they don't consume ITCM budget.
// =========================================================================
#ifdef __arm__
#define ARUCO_BSS_ATTR  __attribute__((section(".sdram_bss")))
#else
#define ARUCO_BSS_ATTR
#endif

static uint8_t  s_binary[ARUCO_BUF_SZ]   ARUCO_BSS_ATTR;
static uint8_t  s_labels[ARUCO_BUF_SZ]   ARUCO_BSS_ATTR;
static int32_t  s_integral[(ARUCO_MAX_W + 1) * (ARUCO_MAX_H + 1)] ARUCO_BSS_ATTR;
static int32_t  s_fill_stack[ARUCO_FILL_STACK_SZ] ARUCO_BSS_ATTR;

typedef struct {
    int      x0, y0, x1, y1;     // bounding box (inclusive)
    int      cx_sum, cy_sum;     // centroid accumulator
    int      n_pix;
    uint8_t  touches_border;
} aruco_comp_t;

static aruco_comp_t s_components[ARUCO_MAX_COMPONENTS] ARUCO_BSS_ATTR;

// =========================================================================
// Module state.
// =========================================================================
namespace {

// Intrinsics defaults match s091 / s130 (320x240 @ ~fov 60° downward).
float s_fx = 240.0f;
float s_fy = 240.0f;
float s_cx = 160.0f;
float s_cy = 120.0f;
// OP-S10-W14 iter #7 (2026-05-18) — corrected from 0.125 to 0.094.
//
// Previous 0.125 was a wrong 2× scale of the OLD 0.0625, ignoring
// that 0.0625 already accounted for ArUco texture padding inside the
// physical marker face (0.08 m * 0.781 ratio).  s172 trial #6 had
// PnP report z_pnp = 0.90 m when GT z = 0.45 m — exact 2× scale
// error, traced to this constant + texture-padding mishandling.
//
// New value 0.094 = (0.12 m physical face) × (0.781 effective-ArUco
// ratio from old s130 calibration).  Until we re-measure the actual
// padding ratio in the doubled textures, this is the consistent
// scaling.  If z_pnp still mismatches GT after this fix, the ratio
// itself needs re-measuring (e.g. detect a static marker at known
// pose and back-solve s_marker_size_m).
float s_marker_size_m = 0.094f;
int   s_initialised   = 0;

sentai_aruco_marker_t s_cache[SENTAI_ARUCO_MAX_MARKERS];
int                   s_cache_count = 0;

sentai_aruco_stats_t  s_stats;

}  // anon namespace

// =========================================================================
// 4x4 ArUco dictionary — REAL OpenCV 4x4_50 entries ids 0..3.
//
// Decoded 2026-05-17 from sim/gazebo/materials/textures/aruco_4x4_50_id*.png
// (the textures Gazebo loads onto the landing-pad / mission markers).
// Bit ordering: row-major top-left = LSB (bit 0).  Inner 4x4 cell at
// (col i, row j) ∈ {1..4} maps to bit (j-1)*4 + (i-1).
//
// To extend to the full 50-entry OpenCV table, run the same PNG-decode
// pass on the rest of the dictionary (textures TBD or generate via
// cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)).  For the
// thesis §23.1 scenario (4-6 markers in the indoor room) ids 0..3
// suffice.
// =========================================================================
static const uint16_t s_aruco_dict[] = {
    0xB352,  // id=0 (OpenCV 4x4_50)
    0xA60F,  // id=1
    0x4B33,  // id=2
    0x9D66,  // id=3
};
static const int s_aruco_dict_n = (int)(sizeof(s_aruco_dict) /
                                         sizeof(s_aruco_dict[0]));

// =========================================================================
// Linear-algebra helpers (3x3, row-major).
// =========================================================================
static inline float vec3_norm3(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

// =========================================================================
// Rvec <-> R conversions (Rodrigues).
// =========================================================================
extern "C" void sentai_aruco_rvec_to_R(const float rvec[3], float R_out[9]) {
    const float theta = vec3_norm3(rvec[0], rvec[1], rvec[2]);
    if (theta < 1e-9f) {
        R_out[0] = 1.0f; R_out[1] = 0.0f; R_out[2] = 0.0f;
        R_out[3] = 0.0f; R_out[4] = 1.0f; R_out[5] = 0.0f;
        R_out[6] = 0.0f; R_out[7] = 0.0f; R_out[8] = 1.0f;
        return;
    }
    const float inv_t = 1.0f / theta;
    const float kx = rvec[0] * inv_t;
    const float ky = rvec[1] * inv_t;
    const float kz = rvec[2] * inv_t;
    const float s  = sinf(theta);
    const float c  = cosf(theta);
    const float C  = 1.0f - c;
    R_out[0] = c + kx*kx*C;
    R_out[1] = kx*ky*C - kz*s;
    R_out[2] = kx*kz*C + ky*s;
    R_out[3] = ky*kx*C + kz*s;
    R_out[4] = c + ky*ky*C;
    R_out[5] = ky*kz*C - kx*s;
    R_out[6] = kz*kx*C - ky*s;
    R_out[7] = kz*ky*C + kx*s;
    R_out[8] = c + kz*kz*C;
}

extern "C" void sentai_aruco_R_to_rvec(const float R[9], float rvec_out[3]) {
    const float tr    = R[0] + R[4] + R[8];
    float       cos_t = (tr - 1.0f) * 0.5f;
    if (cos_t > 1.0f)  cos_t = 1.0f;
    if (cos_t < -1.0f) cos_t = -1.0f;
    const float theta = acosf(cos_t);
    if (theta < 1e-9f) {
        rvec_out[0] = rvec_out[1] = rvec_out[2] = 0.0f;
        return;
    }
    if (theta > 3.0f) {
        // Fallback for near-pi: recover from diagonal.
        float kx2 = (R[0] + 1.0f) * 0.5f; if (kx2 < 0) kx2 = 0;
        float ky2 = (R[4] + 1.0f) * 0.5f; if (ky2 < 0) ky2 = 0;
        float kz2 = (R[8] + 1.0f) * 0.5f; if (kz2 < 0) kz2 = 0;
        float kx, ky, kz;
        if (kx2 >= ky2 && kx2 >= kz2) {
            kx = sqrtf(kx2);
            ky = (kx > 1e-6f) ? ((R[1] + R[3]) * 0.5f / kx) : 0.0f;
            kz = (kx > 1e-6f) ? ((R[2] + R[6]) * 0.5f / kx) : 0.0f;
        } else if (ky2 >= kz2) {
            ky = sqrtf(ky2);
            kx = (ky > 1e-6f) ? ((R[1] + R[3]) * 0.5f / ky) : 0.0f;
            kz = (ky > 1e-6f) ? ((R[5] + R[7]) * 0.5f / ky) : 0.0f;
        } else {
            kz = sqrtf(kz2);
            kx = (kz > 1e-6f) ? ((R[2] + R[6]) * 0.5f / kz) : 0.0f;
            ky = (kz > 1e-6f) ? ((R[5] + R[7]) * 0.5f / kz) : 0.0f;
        }
        rvec_out[0] = theta * kx;
        rvec_out[1] = theta * ky;
        rvec_out[2] = theta * kz;
        return;
    }
    const float sin_t = sinf(theta);
    const float inv = 1.0f / (2.0f * sin_t);
    rvec_out[0] = (R[7] - R[5]) * theta * inv;
    rvec_out[1] = (R[2] - R[6]) * theta * inv;
    rvec_out[2] = (R[3] - R[1]) * theta * inv;
}

// =========================================================================
// Pipeline stage A — adaptive threshold via integral image.
//
// Integral image:    I(x,y) = sum_{u<=x, v<=y} gray(u,v)
// Box sum (W x H window centered at (x,y)):
//   sum = I(x+W/2, y+H/2) - I(x-W/2-1, y+H/2)
//       - I(x+W/2, y-H/2-1) + I(x-W/2-1, y-H/2-1)
// Threshold: binary(x,y) = 1 (BLACK marker pixel) iff gray < mean - C.
//
// Output is binary[i] = 0 or 1.
// =========================================================================
static void aruco_adaptive_threshold(const uint8_t* gray, int w, int h,
                                       int block) {
    const int W = w, H = h;
    const int stride_i = W + 1;
    // Build integral image (zeroes in row 0 / col 0 simplify boundary).
    // i[(x+1) + (y+1) * stride] = i[(x) + (y+1) * stride]
    //                            + i[(x+1) + (y) * stride]
    //                            - i[(x) + (y) * stride]
    //                            + gray[x + y*W]
    for (int x = 0; x <= W; ++x) s_integral[x] = 0;
    for (int y = 1; y <= H; ++y) {
        int32_t row_sum = 0;
        s_integral[y * stride_i] = 0;
        for (int x = 1; x <= W; ++x) {
            row_sum += gray[(x-1) + (y-1) * W];
            s_integral[x + y * stride_i] = s_integral[x + (y-1) * stride_i]
                                           + row_sum;
        }
    }
    const int half = block / 2;
    for (int y = 0; y < H; ++y) {
        int y0 = y - half;        if (y0 < 0) y0 = 0;
        int y1 = y + half;        if (y1 >= H) y1 = H - 1;
        for (int x = 0; x < W; ++x) {
            int x0 = x - half;    if (x0 < 0) x0 = 0;
            int x1 = x + half;    if (x1 >= W) x1 = W - 1;
            const int32_t A = s_integral[(x1+1) + (y1+1) * stride_i];
            const int32_t B = s_integral[(x0)   + (y1+1) * stride_i];
            const int32_t C = s_integral[(x1+1) + (y0)   * stride_i];
            const int32_t D = s_integral[(x0)   + (y0)   * stride_i];
            const int32_t box_sum = A - B - C + D;
            const int32_t box_area = (x1 - x0 + 1) * (y1 - y0 + 1);
            const int32_t mean = box_sum / box_area;
            const uint8_t v = gray[x + y * W];
            s_binary[x + y * W] = ((int32_t)v < mean - ARUCO_THRESH_C) ? 1u : 0u;
        }
    }
}

// =========================================================================
// Pipeline stage B — connected-component labeling (8-connected flood fill).
//
// Iterate each pixel; on the first BLACK unlabeled pixel, flood-fill all
// 8-connected BLACK pixels and assign them the next label.  Record
// bounding box + pixel count + border-touch flag.
//
// 8-connectivity matches cv2.connectedComponents(connectivity=8) used by
// cv2.aruco.  Critical for rotated markers: a 45°-rotated marker's black
// border ring is connected only diagonally between pixel rows, so 4-conn
// fragments it into many small pieces and the area gate then rejects them.
//
// Single-pass + DFS via explicit stack (no recursion — embedded NASA/JPL
// rule).  Stack depth bounded by ARUCO_FILL_STACK_SZ.
// =========================================================================
static int aruco_label_components(int w, int h) {
    memset(s_labels, 0, (size_t)w * (size_t)h);
    int next_label = 1;     // 0 = unlabeled
    int n_components = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const int idx0 = x + y * w;
            if (s_binary[idx0] == 0 || s_labels[idx0] != 0) continue;
            // Start a new component.  Cap at 254 (uint8 0 = unlabeled,
            // 255 = "more than 254").
            uint8_t lab = (next_label <= 254) ? (uint8_t)next_label : 255u;
            if (next_label > 254) {
                // Skip — too many components; mark and don't grow array.
                s_labels[idx0] = 255u;
                continue;
            }
            int stack_top = 0;
            s_fill_stack[stack_top++] = idx0;
            s_labels[idx0] = lab;
            aruco_comp_t* c = &s_components[n_components];
            c->x0 = x; c->x1 = x;
            c->y0 = y; c->y1 = y;
            c->cx_sum = 0; c->cy_sum = 0;
            c->n_pix = 0;
            c->touches_border = 0;
            while (stack_top > 0) {
                const int idx = s_fill_stack[--stack_top];
                const int px = idx % w;
                const int py = idx / w;
                if (px < c->x0) c->x0 = px;
                if (px > c->x1) c->x1 = px;
                if (py < c->y0) c->y0 = py;
                if (py > c->y1) c->y1 = py;
                c->cx_sum += px;
                c->cy_sum += py;
                c->n_pix++;
                if (px == 0 || px == w-1 || py == 0 || py == h-1) {
                    c->touches_border = 1;
                }
                // 8-connected neighbours (matches cv2 default).
                static const int dx[8] = { -1, +1,  0, 0, -1, -1, +1, +1 };
                static const int dy[8] = {  0,  0, -1, +1, -1, +1, -1, +1 };
                for (int k = 0; k < 8; ++k) {
                    const int nx = px + dx[k];
                    const int ny = py + dy[k];
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    const int nidx = nx + ny * w;
                    if (s_binary[nidx] == 0 || s_labels[nidx] != 0) continue;
                    if (stack_top >= ARUCO_FILL_STACK_SZ) {
                        // Overflow — bail on this component but keep
                        // already-labelled pixels (caller will reject
                        // by area mismatch / border touch).
                        s_stats.overflow_total++;
                        goto next_pixel;
                    }
                    s_labels[nidx] = lab;
                    s_fill_stack[stack_top++] = nidx;
                }
            }
next_pixel:
            n_components++;
            next_label++;
            if (n_components >= ARUCO_MAX_COMPONENTS) {
                // Cap reached; return what we have.
                return n_components;
            }
        }
    }
    return n_components;
}

// =========================================================================
// Pipeline stage C — quad corner extraction.
//
// For each accepted component, scan all of its labelled pixels (via
// s_labels) and find the 4 most extreme corner-candidates:
//   TL = min(x + y) — closest to image origin
//   TR = min(-x + y) = max(x - y) — top-right
//   BR = max(x + y) — bottom-right
//   BL = max(-x + y) = min(x - y) — bottom-left
//
// The ordering above is by image-coordinate convention (y down).
//
// Returns 0 on success + fills corners[8] = u0,v0,...,u3,v3 (TL,TR,BR,BL).
// Returns -1 if degenerate (any two corners coincide).
// =========================================================================
// =========================================================================
// T18-E — Moore-Neighbor border tracing + Ramer-Douglas-Peucker polygon
// simplification.  This replaces the integer-pixel extrema heuristic
// (aruco_extract_quad_legacy below) which produces 1-3 px corner offsets
// for in-plane-rotated markers.
//
// Border-following: Moore-Neighbor (8-connected), starting from the
// top-most-then-left-most pixel of the labeled component.  Produces
// the outer boundary as an ordered CW sequence of integer pixel
// coordinates.
//
// Polygon simplification: Ramer (1972) / Douglas-Peucker (1973).  Algo
// ported from OpenCV's modules/imgproc/src/approx.cpp (Intel
// Corporation 2000, BSD-3-Clause).  The iterative-stack form
// matches the OpenCV implementation closely; adapted to fixed-size
// int16 buffers and float arithmetic for M7 portability.  See
// experiments/s177_corner_subpix_prototype/FINDINGS.md for the
// motivation (cv2.aruco detects 4/4 markers on our rot45 frame
// where the legacy extractor finds 0).
//
// Output: 4 corner candidates in CW cyclic order.  The downstream
// bit decoder tries all 4 rotations, so identifying TL is not
// required here.
// =========================================================================
#define ARUCO_BORDER_MAX        2048       // outer-perimeter pixel cap
#define ARUCO_DP_STACK_MAX      64
#define ARUCO_DP_EPS_FRAC       0.04f      // cv2.aruco: 0.04 * perimeter

static int16_t s_border[2 * ARUCO_BORDER_MAX]   ARUCO_BSS_ATTR;
static uint8_t s_dp_keep[ARUCO_BORDER_MAX]      ARUCO_BSS_ATTR;

// Moore-Neighbor 8-connected outer-border trace.  Starts at (sx, sy)
// which must be on the boundary of the labeled component.  Returns
// number of border pixels (closed loop, no duplicate at end), or -1
// on overflow / degenerate input.  CW order assuming start pixel was
// reached scanning rows top-to-bottom, left-to-right.
static int aruco_trace_border(uint8_t lab, int w, int h,
                               int sx, int sy,
                               int16_t* out) {
    static const int8_t DX[8] = { +1, +1,  0, -1, -1, -1,  0, +1 };
    static const int8_t DY[8] = {  0, -1, -1, -1,  0, +1, +1, +1 };
    int x = sx, y = sy;
    int came = 4;           // came from west — search starts NW going CW
    int n = 0;
    for (;;) {
        if (n >= ARUCO_BORDER_MAX) return -1;
        out[n*2 + 0] = (int16_t)x;
        out[n*2 + 1] = (int16_t)y;
        n++;
        // Search CW from (came - 1) mod 8.
        const int start_dir = (came + 7) & 7;
        int found = -1;
        for (int k = 0; k < 8; ++k) {
            const int dir = (start_dir + 8 - k) & 7;
            const int nx = x + DX[dir];
            const int ny = y + DY[dir];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            if (s_labels[nx + ny*w] != lab) continue;
            found = dir;
            break;
        }
        if (found < 0) return n;   // isolated pixel
        x += DX[found];
        y += DY[found];
        came = (found + 4) & 7;
        if (x == sx && y == sy && n >= 2) break;
    }
    return n;
}

// Iterative Douglas-Peucker on a closed polygon.  Sets s_dp_keep[i]=1
// for points retained.  eps_sq is squared perpendicular threshold.
static void aruco_dp_mark(const int16_t* pts, int n, float eps_sq) {
    memset(s_dp_keep, 0, (size_t)n);
    if (n < 3) {
        for (int i = 0; i < n; ++i) s_dp_keep[i] = 1;
        return;
    }
    // Seed: point #0 plus the point farthest from #0.
    int seed_b = 0;
    float max_d = -1.0f;
    for (int j = 1; j < n; ++j) {
        const float dx = (float)(pts[j*2 + 0] - pts[0]);
        const float dy = (float)(pts[j*2 + 1] - pts[1]);
        const float d  = dx*dx + dy*dy;
        if (d > max_d) { max_d = d; seed_b = j; }
    }
    s_dp_keep[0]      = 1;
    s_dp_keep[seed_b] = 1;
    // Stack of (start, end) index pairs, closed-polygon convention
    // where end may equal start + n to wrap.
    int stack_s[ARUCO_DP_STACK_MAX];
    int stack_e[ARUCO_DP_STACK_MAX];
    int top = 0;
    stack_s[top] = 0;       stack_e[top] = seed_b;       top++;
    stack_s[top] = seed_b;  stack_e[top] = n;            top++;   // wraps to 0

    while (top > 0) {
        --top;
        const int s = stack_s[top];
        const int e = stack_e[top];
        if (e - s < 2) continue;
        const int ee = e % n;
        const float x0 = (float)pts[s*2 + 0];
        const float y0 = (float)pts[s*2 + 1];
        const float x1 = (float)pts[ee*2 + 0];
        const float y1 = (float)pts[ee*2 + 1];
        const float dx = x1 - x0;
        const float dy = y1 - y0;
        const float len_sq = dx*dx + dy*dy + 1e-9f;
        int   best_k     = -1;
        float best_d_sq  = -1.0f;
        for (int j = s + 1; j < e; ++j) {
            const int jj = j % n;
            const float px = (float)pts[jj*2 + 0];
            const float py = (float)pts[jj*2 + 1];
            const float cross = (px - x0) * dy - (py - y0) * dx;
            const float d_sq  = (cross * cross) / len_sq;
            if (d_sq > best_d_sq) { best_d_sq = d_sq; best_k = j; }
        }
        if (best_k >= 0 && best_d_sq > eps_sq) {
            s_dp_keep[best_k % n] = 1;
            if (top + 2 > ARUCO_DP_STACK_MAX) continue;   // overflow → skip
            stack_s[top] = s;       stack_e[top] = best_k;  top++;
            stack_s[top] = best_k;  stack_e[top] = e;       top++;
        }
    }
}

// New aruco_extract_quad replacement.  Returns 0 on success with 4
// corners in CW order; -1 on failure (not a 4-vertex polygon).
static int aruco_extract_quad(uint8_t lab_target, int w, int h,
                              const aruco_comp_t* c, float corners[8]) {
    // Find a top-left starting boundary pixel of the component.
    int sx = -1, sy = -1;
    for (int y = c->y0; y <= c->y1 && sy < 0; ++y) {
        for (int x = c->x0; x <= c->x1; ++x) {
            if (s_labels[x + y*w] == lab_target) {
                sx = x; sy = y;
                break;
            }
        }
    }
    if (sy < 0) return -1;

    const int n = aruco_trace_border(lab_target, w, h, sx, sy, s_border);
    if (n < 8) return -1;

    // Iterative eps: cv2.aruco uses 0.05 but noisy rendered contours
    // sometimes need 0.07-0.10 to collapse minor wobbles into 4 vertices.
    // Sweep [0.03..0.10] coarsely; first eps that yields exactly 4
    // wins.  Cheap on M7 (≤8 DP passes, each O(n)).
    int n_kept = 0;
    int kept_idx[16];
    static const float EPS_FRACS[] = {
        0.04f, 0.05f, 0.06f, 0.07f, 0.03f, 0.08f, 0.10f, 0.02f
    };
    for (unsigned ei = 0; ei < sizeof(EPS_FRACS) / sizeof(EPS_FRACS[0]); ++ei) {
        const float eps = EPS_FRACS[ei] * (float)n;
        aruco_dp_mark(s_border, n, eps * eps);
        n_kept = 0;
        for (int i = 0; i < n && n_kept < 16; ++i) {
            if (s_dp_keep[i]) kept_idx[n_kept++] = i;
        }
        if (n_kept == 4) break;
    }
    if (n_kept != 4) return -1;
    for (int i = 0; i < 4; ++i) {
        corners[i*2 + 0] = (float)s_border[kept_idx[i]*2 + 0];
        corners[i*2 + 1] = (float)s_border[kept_idx[i]*2 + 1];
    }
    return 0;
}

// Kept for reference; superseded by the DP-based extractor above.
static int aruco_extract_quad_legacy(uint8_t lab_target, int w, int h,
                              const aruco_comp_t* c, float corners[8]) {
    int   best_tl_sum = INT32_MAX, best_tl_px = 0, best_tl_py = 0;
    int   best_br_sum = INT32_MIN, best_br_px = 0, best_br_py = 0;
    int   best_tr_xm  = INT32_MIN, best_tr_px = 0, best_tr_py = 0;
    int   best_bl_xm  = INT32_MAX, best_bl_px = 0, best_bl_py = 0;
    for (int y = c->y0; y <= c->y1; ++y) {
        for (int x = c->x0; x <= c->x1; ++x) {
            if (s_labels[x + y * w] != lab_target) continue;
            const int s = x + y;
            const int d = x - y;
            if (s < best_tl_sum) {
                best_tl_sum = s; best_tl_px = x; best_tl_py = y;
            }
            if (s > best_br_sum) {
                best_br_sum = s; best_br_px = x; best_br_py = y;
            }
            if (d > best_tr_xm) {
                best_tr_xm  = d; best_tr_px = x; best_tr_py = y;
            }
            if (d < best_bl_xm) {
                best_bl_xm  = d; best_bl_px = x; best_bl_py = y;
            }
        }
    }
    // Coincidence check — degenerate component.
    if ((best_tl_px == best_tr_px && best_tl_py == best_tr_py) ||
        (best_tl_px == best_br_px && best_tl_py == best_br_py) ||
        (best_tl_px == best_bl_px && best_tl_py == best_bl_py) ||
        (best_tr_px == best_br_px && best_tr_py == best_br_py)) {
        return -1;
    }
    corners[0] = (float)best_tl_px; corners[1] = (float)best_tl_py;
    corners[2] = (float)best_tr_px; corners[3] = (float)best_tr_py;
    corners[4] = (float)best_br_px; corners[5] = (float)best_br_py;
    corners[6] = (float)best_bl_px; corners[7] = (float)best_bl_py;
    return 0;
}

// =========================================================================
// T18-F — Port of cv2.aruco _extractBits (modules/objdetect/src/aruco/
// aruco_detector.cpp:324, BSD-3-Clause, Intel/OpenCV).
//
// Step 1: Compute perspective transform from quad corners to a canonical
//         N×N output via DLT homography (we re-use aruco_dlt_homography).
// Step 2: Warp the grayscale image to the canonical buffer (nearest-
//         neighbour sample, matching cv2's INTER_NEAREST default).
// Step 3: Apply Otsu's global threshold on the warped buffer.
// Step 4: Count DARK pixels in each inner cell — bit = 1 if majority is
//         dark, matching our dictionary's bit-1-is-black convention.
// =========================================================================
#define ARUCO_BITGRID_CELL      4       // px per marker cell in canonical
#define ARUCO_BITGRID_BORDER    1       // ArUco border bits = 1
#define ARUCO_BITGRID_SIDE      (ARUCO_PATCH_DIM * ARUCO_BITGRID_CELL)

static uint8_t s_warp_buf[ARUCO_BITGRID_SIDE * ARUCO_BITGRID_SIDE] ARUCO_BSS_ATTR;

// Forward decl — definition lower in file (used by T18-F before the
// PnP block where DLT was originally introduced).
static int aruco_dlt_homography(const float mx[4], const float my[4],
                                 const float u[4], const float v[4],
                                 float H[9]);

static int aruco_warp_to_canonical(const uint8_t* img, int W, int H_img,
                                    const float corners[8],
                                    uint8_t* out, int N) {
    // Source (canonical) corners — output pixel positions for TL, TR, BR, BL.
    const float mx[4] = { 0.0f, (float)(N - 1), (float)(N - 1), 0.0f };
    const float my[4] = { 0.0f, 0.0f,           (float)(N - 1), (float)(N - 1) };
    const float u[4]  = { corners[0], corners[2], corners[4], corners[6] };
    const float v[4]  = { corners[1], corners[3], corners[5], corners[7] };
    float H[9];
    if (aruco_dlt_homography(mx, my, u, v, H) != 0) return -1;
    for (int yd = 0; yd < N; ++yd) {
        for (int xd = 0; xd < N; ++xd) {
            const float wd = H[6] * (float)xd + H[7] * (float)yd + H[8];
            if (fabsf(wd) < 1e-9f) { out[yd*N + xd] = 0; continue; }
            const float xs = (H[0]*(float)xd + H[1]*(float)yd + H[2]) / wd;
            const float ys = (H[3]*(float)xd + H[4]*(float)yd + H[5]) / wd;
            int xi = (int)(xs + 0.5f);
            int yi = (int)(ys + 0.5f);
            if (xi < 0) xi = 0;
            if (yi < 0) yi = 0;
            if (xi >= W)     xi = W - 1;
            if (yi >= H_img) yi = H_img - 1;
            out[yd*N + xd] = img[yi*W + xi];
        }
    }
    return 0;
}

// Otsu's global threshold (1979) — between-class-variance maximisation.
// Returns threshold value in [0, 255].
static uint8_t aruco_otsu_threshold(const uint8_t* img, int n_pixels) {
    int hist[256];
    memset(hist, 0, sizeof(hist));
    for (int i = 0; i < n_pixels; ++i) hist[img[i]]++;
    long total = n_pixels;
    long sum = 0;
    for (int i = 0; i < 256; ++i) sum += (long)i * hist[i];
    long sum_b = 0;
    long w_b   = 0;
    float max_var = 0.0f;
    int   best   = 127;
    for (int t = 0; t < 256; ++t) {
        w_b += hist[t];
        if (w_b == 0) continue;
        long w_f = total - w_b;
        if (w_f == 0) break;
        sum_b += (long)t * hist[t];
        const float mean_b = (float)sum_b / (float)w_b;
        const float mean_f = (float)(sum - sum_b) / (float)w_f;
        const float var_between = (float)w_b * (float)w_f *
                                  (mean_b - mean_f) * (mean_b - mean_f);
        if (var_between > max_var) {
            max_var = var_between;
            best = t;
        }
    }
    return (uint8_t)best;
}

// Decode the 4×4 inner data cells from an already-warped canonical
// buffer.  `t` is the Otsu threshold for the buffer.  Returns the
// 16-bit pattern with bit=1 where the cell majority is DARK.
static uint16_t aruco_decode_canonical_(const uint8_t* warped, int N,
                                          int cell_size, int marker_border,
                                          uint8_t t) {
    const int margin = (int)(0.13f * (float)cell_size + 0.5f);   // cv2 default
    uint16_t pattern = 0;
    for (int cy = 0; cy < 4; ++cy) {
        const int yc = (cy + marker_border) * cell_size;
        for (int cx = 0; cx < 4; ++cx) {
            const int xc = (cx + marker_border) * cell_size;
            int n_dark = 0, n_tot = 0;
            for (int dy = margin; dy < cell_size - margin; ++dy) {
                for (int dx = margin; dx < cell_size - margin; ++dx) {
                    if (warped[(yc + dy) * N + (xc + dx)] < t) n_dark++;
                    n_tot++;
                }
            }
            if (n_dark * 2 > n_tot) {
                pattern |= (uint16_t)(1u << (cy * 4 + cx));
            }
        }
    }
    return pattern;
}

// =========================================================================
// Pipeline stage D — decode marker bits + dictionary lookup.
//
// Sample a 6x6 grid inside the quad (1-pixel border + 4x4 data).  Each
// grid cell is sampled at its centre, with sub-pixel bilinear
// interpolation in the binary buffer.  Bits 0..15 are the inner 4x4
// (border is expected to be all-1 = BLACK and we DON'T enforce that
// here — the dictionary hamming gate will reject malformed borders).
//
// Tries all 4 rotations and returns the best (lowest hamming) match.
// Returns marker_id (0..s_aruco_dict_n-1) or -1 if no match within
// SENTAI_ARUCO_MAX_HAMMING; out_rotation tells the caller how many CW
// 90° rotations are needed to align the decoded pattern to the dict.
// =========================================================================
static inline uint8_t aruco_sample_at(int x, int y, int w, int h) {
    if (x < 0) x = 0;
    if (x >= w) x = w - 1;
    if (y < 0) y = 0;
    if (y >= h) y = h - 1;
    return s_binary[x + y * w];
}

static int aruco_decode_marker(const uint8_t* gray,
                               const float corners[8], int w, int h,
                               int* out_rotation, int* out_hamming) {
    // T18-F: cv2.aruco _extractBits pipeline — perspective warp, Otsu
    // threshold, per-cell majority count of DARK pixels (bit-1-is-black
    // matches our dictionary convention).
    if (aruco_warp_to_canonical(gray, w, h, corners,
                                  s_warp_buf, ARUCO_BITGRID_SIDE) != 0) {
        if (out_hamming) *out_hamming = 17;
        return -1;
    }
    const uint8_t t = aruco_otsu_threshold(s_warp_buf,
                                            ARUCO_BITGRID_SIDE * ARUCO_BITGRID_SIDE);
    const uint16_t pattern = aruco_decode_canonical_(
        s_warp_buf, ARUCO_BITGRID_SIDE,
        ARUCO_BITGRID_CELL, ARUCO_BITGRID_BORDER, t);
    // Try 4 rotations.  Rotation by 90° CW maps bit at (i, j) to
    // (j, 3 - i) in a 4x4 grid.  We rotate the BIT PATTERN.
    int best_id = -1;
    int best_hamm = 17;          // strictly greater than any possible hamming (16)
    int best_rot = 0;
    uint16_t cur = pattern;
    for (int r = 0; r < 4; ++r) {
        for (int id = 0; id < s_aruco_dict_n; ++id) {
            const uint16_t xor_val = (uint16_t)(cur ^ s_aruco_dict[id]);
            int hamm = __builtin_popcount(xor_val);
            if (hamm < best_hamm) {
                best_hamm = hamm;
                best_id = id;
                best_rot = r;
            }
        }
        // Rotate cur 90° CW within the 4x4 grid.
        uint16_t rot = 0;
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i) {
                if (cur & (uint16_t)(1u << (j * 4 + i))) {
                    rot |= (uint16_t)(1u << (i * 4 + (3 - j)));
                }
            }
        }
        cur = rot;
    }
    if (best_hamm > SENTAI_ARUCO_MAX_HAMMING) {
        if (out_hamming) *out_hamming = best_hamm;
        return -1;
    }
    if (out_rotation) *out_rotation = best_rot;
    if (out_hamming)  *out_hamming  = best_hamm;
    return best_id;
}

static void aruco_realign_corners(float corners[8], int rotation_steps) {
    // The decoder found the dictionary match at `rotation_steps` 90°
    // CW rotations of the SAMPLED bit pattern.  That means the marker
    // was drawn at `rotation_steps` 90° CW rotations from canonical.
    // To pair image corners with canonical marker-frame corners
    // (TL, TR, BR, BL) we shift the array such that
    //   new[k] = old[(k + rotation_steps) % 4]
    // — i.e. the canonical-TL corner is currently sitting at image
    // position rotation_steps; we move it to position 0.  Each step
    // of the loop shifts values one position to the LEFT.
    for (int t = 0; t < rotation_steps; ++t) {
        const float u0 = corners[0], v0 = corners[1];
        const float u1 = corners[2], v1 = corners[3];
        const float u2 = corners[4], v2 = corners[5];
        const float u3 = corners[6], v3 = corners[7];
        corners[0] = u1; corners[1] = v1;
        corners[2] = u2; corners[3] = v2;
        corners[4] = u3; corners[5] = v3;
        corners[6] = u0; corners[7] = v0;
    }
}

// =========================================================================
// Pipeline stage D.5 — Förstner corner sub-pixel refinement (T18-C).
//
// Ported byte-for-byte from OpenCV cv::cornerSubPix (cornersubpix.cpp,
// 4.x), validated against the reference in Python at
// examples/sentai_runtime/experiments/s177_corner_subpix_prototype/
// our_subpix_pure.py — match to 0.0000 px on real-frame test inputs.
//
// Why we need this: contour-finder corners are integer-pixel; under
// in-plane rotation the rendered marker corners drift sub-pixel; this
// propagates to PnP Z at ~20-60 mm/frame (see s177 FINDINGS.md).
// Refinement cuts the per-frame Z noise by ~5×.
//
// Window: 5×5 inner gradient region inside a 7×7 bilinear-sampled
// patch.  Up to 30 iterations or |Δ| < 0.01 px.  Safety: revert to
// initial corner if final |Δc| > 2 px on either axis (oscillation).
// =========================================================================
// win_half = 5 matches cv2.aruco DetectorParameters default
// (cornerRefinementWinSize = 5 ⇒ 11x11 window).  Larger window is
// required to capture corner offsets > 2 px that occur with rotated
// quads (contour extrema land at integer-pixel near the true corner
// but can be 2-3 px off the actual sub-pixel corner location).
#define ARUCO_SUBPIX_WIN_HALF   2          // 5x5 inner window (cv2.aruco
                                            // default = 5 = 11x11 window)
#define ARUCO_SUBPIX_WIN        (2 * ARUCO_SUBPIX_WIN_HALF + 1)
#define ARUCO_SUBPIX_BIG        (ARUCO_SUBPIX_WIN + 2)
#define ARUCO_SUBPIX_MAX_ITER   30
#define ARUCO_SUBPIX_EPS_PX     0.01f

// cv2 Gaussian mask, computed at runtime once (11x11 → 121 floats,
// constant after first call).  Build with:
//   vy[i] = exp(-((i - win_half) / win_half)^2) for i in 0..2*win_half
//   mask[i,j] = vy[i] * vy[j]
static float ARUCO_SUBPIX_MASK[ARUCO_SUBPIX_WIN * ARUCO_SUBPIX_WIN];
static int   ARUCO_SUBPIX_MASK_READY = 0;

static void aruco_subpix_init_mask_(void) {
    if (ARUCO_SUBPIX_MASK_READY) return;
    const float inv_h = 1.0f / (float)ARUCO_SUBPIX_WIN_HALF;
    for (int i = 0; i < ARUCO_SUBPIX_WIN; ++i) {
        const float ry = (float)(i - ARUCO_SUBPIX_WIN_HALF) * inv_h;
        const float vy = expf(-ry * ry);
        for (int j = 0; j < ARUCO_SUBPIX_WIN; ++j) {
            const float rx = (float)(j - ARUCO_SUBPIX_WIN_HALF) * inv_h;
            ARUCO_SUBPIX_MASK[i * ARUCO_SUBPIX_WIN + j] = vy * expf(-rx * rx);
        }
    }
    ARUCO_SUBPIX_MASK_READY = 1;
}

static inline float aruco_bilinear_(const uint8_t* img, int W, int H,
                                     float x, float y) {
    if (x < 0.0f) x = 0.0f;
    if (y < 0.0f) y = 0.0f;
    const float xmax = (float)(W - 1);
    const float ymax = (float)(H - 1);
    if (x > xmax) x = xmax;
    if (y > ymax) y = ymax;
    const int xi = (int)x;
    const int yi = (int)y;
    const float ax = x - (float)xi;
    const float ay = y - (float)yi;
    const int xi1 = (xi + 1 < W) ? xi + 1 : xi;
    const int yi1 = (yi + 1 < H) ? yi + 1 : yi;
    const float i00 = (float)img[yi  * W + xi ];
    const float i01 = (float)img[yi  * W + xi1];
    const float i10 = (float)img[yi1 * W + xi ];
    const float i11 = (float)img[yi1 * W + xi1];
    return (1.0f - ax) * (1.0f - ay) * i00 +
           ax          * (1.0f - ay) * i01 +
           (1.0f - ax) * ay          * i10 +
           ax          * ay          * i11;
}

static void aruco_refine_corner_subpix(const uint8_t* img, int W, int H,
                                        float* cx_io, float* cy_io) {
    aruco_subpix_init_mask_();
    const float x_init = *cx_io;
    const float y_init = *cy_io;
    float x = x_init;
    float y = y_init;
    const float half = (float)(ARUCO_SUBPIX_BIG - 1) * 0.5f;
    const float eps_sq = ARUCO_SUBPIX_EPS_PX * ARUCO_SUBPIX_EPS_PX;
    float patch[ARUCO_SUBPIX_BIG][ARUCO_SUBPIX_BIG];

    for (int iter = 0; iter < ARUCO_SUBPIX_MAX_ITER; ++iter) {
        // Bounds: bilinear handles edges but we need room for the
        // 7x7 patch + 1 px margin for the inner gradient stencil.
        if (x - half - 1.0f < 0.0f) break;
        if (y - half - 1.0f < 0.0f) break;
        if (x + half + 1.0f >= (float)W) break;
        if (y + half + 1.0f >= (float)H) break;

        for (int i = 0; i < ARUCO_SUBPIX_BIG; ++i) {
            const float sy = y + ((float)i - half);
            for (int j = 0; j < ARUCO_SUBPIX_BIG; ++j) {
                const float sx = x + ((float)j - half);
                patch[i][j] = aruco_bilinear_(img, W, H, sx, sy);
            }
        }

        float A00 = 0.0f, A01 = 0.0f, A11 = 0.0f;
        float b0 = 0.0f, b1 = 0.0f;
        for (int di = 0; di < ARUCO_SUBPIX_WIN; ++di) {
            const float pix_ry = (float)(di - ARUCO_SUBPIX_WIN_HALF);
            for (int dj = 0; dj < ARUCO_SUBPIX_WIN; ++dj) {
                const float pix_rx = (float)(dj - ARUCO_SUBPIX_WIN_HALF);
                const float m   = ARUCO_SUBPIX_MASK[di * ARUCO_SUBPIX_WIN + dj];
                const float tgx = patch[di + 1][dj + 2] - patch[di + 1][dj];
                const float tgy = patch[di + 2][dj + 1] - patch[di    ][dj + 1];
                const float gxx = tgx * tgx * m;
                const float gxy = tgx * tgy * m;
                const float gyy = tgy * tgy * m;
                A00 += gxx;
                A01 += gxy;
                A11 += gyy;
                b0 += gxx * pix_rx + gxy * pix_ry;
                b1 += gxy * pix_rx + gyy * pix_ry;
            }
        }
        const float det = A00 * A11 - A01 * A01;
        if (fabsf(det) < 1e-12f) break;
        const float inv = 1.0f / det;
        const float dx  = ( A11 * b0 - A01 * b1) * inv;
        const float dy  = (-A01 * b0 + A00 * b1) * inv;
        x += dx;
        y += dy;
        if (dx*dx + dy*dy < eps_sq) break;
    }

    // cv2 safety revert.
    const float wh = (float)ARUCO_SUBPIX_WIN_HALF;
    if (fabsf(x - x_init) > wh || fabsf(y - y_init) > wh) {
        x = x_init;
        y = y_init;
    }
    *cx_io = x;
    *cy_io = y;
}

// =========================================================================
// Pipeline stage E — planar PnP via homography decomposition.
//
// For a planar marker, given 4 image corners + camera intrinsics, the
// homography H decomposes into [r1 r2 t] (column-major) after K^{-1}
// normalisation.  See Collins & Bartoli IJCV 2014 (IPPE) for the full
// 2-solution analytical treatment.  In our scene (markers ≈ parallel
// to image plane), the IPPE two-fold ambiguity collapses (both
// solutions identical) so we keep just one solution.  s176 Python
// sweep validated this empirically across 0°-90° corner rotations.
//
// Marker is centered at origin, side L; corners in marker frame:
//   M0 = (-L/2, +L/2, 0)  TL
//   M1 = (+L/2, +L/2, 0)  TR
//   M2 = (+L/2, -L/2, 0)  BR
//   M3 = (-L/2, -L/2, 0)  BL
//
// (Note: marker frame y-axis points UP; image v-axis points DOWN.
//  This sign convention is consistent with OpenCV's solvePnP output.)
// =========================================================================

// Solve a 4-point homography H (3x3 row-major) mapping marker corners
// (x_m, y_m) to image (u, v) via the Direct Linear Transform.
// Returns 0 on success, -1 if the linear system is singular.
static int aruco_dlt_homography(const float mx[4], const float my[4],
                                 const float u[4], const float v[4],
                                 float H[9]) {
    // 8x9 system A*h = 0.  We pin h[8] = 1 and solve A_8x8 * h_8 = b_8.
    // Two rows per point:
    //   [mx, my, 1, 0,  0,  0, -mx*u, -my*u]  h_0..7 = u
    //   [0,  0,  0, mx, my, 1, -mx*v, -my*v]  h_0..7 = v
    float A[8][8];
    float b[8];
    for (int i = 0; i < 4; ++i) {
        A[2*i + 0][0] = mx[i]; A[2*i + 0][1] = my[i]; A[2*i + 0][2] = 1.0f;
        A[2*i + 0][3] = 0.0f;  A[2*i + 0][4] = 0.0f;  A[2*i + 0][5] = 0.0f;
        A[2*i + 0][6] = -mx[i] * u[i];
        A[2*i + 0][7] = -my[i] * u[i];
        b[2*i + 0]    = u[i];
        A[2*i + 1][0] = 0.0f;  A[2*i + 1][1] = 0.0f;  A[2*i + 1][2] = 0.0f;
        A[2*i + 1][3] = mx[i]; A[2*i + 1][4] = my[i]; A[2*i + 1][5] = 1.0f;
        A[2*i + 1][6] = -mx[i] * v[i];
        A[2*i + 1][7] = -my[i] * v[i];
        b[2*i + 1]    = v[i];
    }
    // Gaussian elimination with partial pivoting.
    for (int i = 0; i < 8; ++i) {
        int   piv = i;
        float mxv = fabsf(A[i][i]);
        for (int j = i + 1; j < 8; ++j) {
            float a = fabsf(A[j][i]);
            if (a > mxv) { mxv = a; piv = j; }
        }
        if (mxv < 1e-9f) return -1;
        if (piv != i) {
            for (int k = 0; k < 8; ++k) {
                float tmp = A[i][k]; A[i][k] = A[piv][k]; A[piv][k] = tmp;
            }
            float tmp = b[i]; b[i] = b[piv]; b[piv] = tmp;
        }
        const float inv = 1.0f / A[i][i];
        for (int j = i + 1; j < 8; ++j) {
            const float f = A[j][i] * inv;
            if (f == 0.0f) continue;
            for (int k = i; k < 8; ++k) A[j][k] -= f * A[i][k];
            b[j] -= f * b[i];
        }
    }
    float h[9];
    h[8] = 1.0f;
    for (int i = 7; i >= 0; --i) {
        float s = b[i];
        for (int j = i + 1; j < 8; ++j) s -= A[i][j] * h[j];
        h[i] = s / A[i][i];
    }
    memcpy(H, h, sizeof(h));
    return 0;
}

// Mean L2 reprojection error of the 4 marker corners through (R, t).
// Returns 0 on success; -1 if a corner falls behind the camera.
static int aruco_reproj_err(const float R[9], const float t[3],
                             const float mx[4], const float my[4],
                             const float u[4], const float v[4],
                             float fx, float fy, float cx, float cy,
                             float* err_out) {
    float err = 0.0f;
    for (int i = 0; i < 4; ++i) {
        const float Xc = R[0]*mx[i] + R[1]*my[i] + t[0];
        const float Yc = R[3]*mx[i] + R[4]*my[i] + t[1];
        const float Zc = R[6]*mx[i] + R[7]*my[i] + t[2];
        if (Zc <= 1e-9f) return -1;
        const float u_p = fx * (Xc / Zc) + cx;
        const float v_p = fy * (Yc / Zc) + cy;
        const float du = u_p - u[i];
        const float dv = v_p - v[i];
        err += sqrtf(du*du + dv*dv);
    }
    *err_out = err * 0.25f;
    return 0;
}

static int aruco_pnp_from_corners(const float corners[8],
                                  float fx, float fy, float cx, float cy,
                                  float marker_size_m,
                                  float tvec_out[3], float rvec_out[3],
                                  float* reproj_err_out) {
    const float L2 = marker_size_m * 0.5f;
    // Marker corners in marker frame (y up).
    const float mx[4] = { -L2, +L2, +L2, -L2 };
    const float my[4] = { +L2, +L2, -L2, -L2 };
    const float u[4]  = { corners[0], corners[2], corners[4], corners[6] };
    const float v[4]  = { corners[1], corners[3], corners[5], corners[7] };
    float H[9];
    if (aruco_dlt_homography(mx, my, u, v, H) != 0) return -1;
    // Normalise H via K^{-1}: H' = K^{-1} H.  K^{-1} = diag(1/fx, 1/fy, 1)
    // with cx, cy offsets:
    //   K^{-1} = [[1/fx, 0, -cx/fx], [0, 1/fy, -cy/fy], [0, 0, 1]]
    float Hn[9];
    const float ifx = 1.0f / fx;
    const float ify = 1.0f / fy;
    Hn[0] = ifx * H[0] - (cx * ifx) * H[6];
    Hn[1] = ifx * H[1] - (cx * ifx) * H[7];
    Hn[2] = ifx * H[2] - (cx * ifx) * H[8];
    Hn[3] = ify * H[3] - (cy * ify) * H[6];
    Hn[4] = ify * H[4] - (cy * ify) * H[7];
    Hn[5] = ify * H[5] - (cy * ify) * H[8];
    Hn[6] = H[6];
    Hn[7] = H[7];
    Hn[8] = H[8];
    // Two columns of R from h1, h2; scale lambda chosen so ||r1|| = 1.
    const float n_h1 = sqrtf(Hn[0]*Hn[0] + Hn[3]*Hn[3] + Hn[6]*Hn[6]);
    const float n_h2 = sqrtf(Hn[1]*Hn[1] + Hn[4]*Hn[4] + Hn[7]*Hn[7]);
    if (n_h1 < 1e-9f || n_h2 < 1e-9f) return -1;
    const float lambda = 2.0f / (n_h1 + n_h2);   // geometric average
    float r1[3] = { Hn[0] * lambda, Hn[3] * lambda, Hn[6] * lambda };
    float r2[3] = { Hn[1] * lambda, Hn[4] * lambda, Hn[7] * lambda };
    float t[3]  = { Hn[2] * lambda, Hn[5] * lambda, Hn[8] * lambda };
    // Re-orthogonalise: r2 := r2 - (r1 . r2) r1; r3 = r1 x r2.
    const float r1_dot_r2 = r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2];
    r2[0] -= r1_dot_r2 * r1[0];
    r2[1] -= r1_dot_r2 * r1[1];
    r2[2] -= r1_dot_r2 * r1[2];
    const float n_r1 = sqrtf(r1[0]*r1[0] + r1[1]*r1[1] + r1[2]*r1[2]);
    const float n_r2 = sqrtf(r2[0]*r2[0] + r2[1]*r2[1] + r2[2]*r2[2]);
    if (n_r1 < 1e-9f || n_r2 < 1e-9f) return -1;
    r1[0] /= n_r1; r1[1] /= n_r1; r1[2] /= n_r1;
    r2[0] /= n_r2; r2[1] /= n_r2; r2[2] /= n_r2;
    float r3[3] = {
        r1[1]*r2[2] - r1[2]*r2[1],
        r1[2]*r2[0] - r1[0]*r2[2],
        r1[0]*r2[1] - r1[1]*r2[0],
    };
    // tvec_z > 0 required (marker in front of camera).  If t[2] < 0,
    // negate (r1, r2, r3, t) to put the marker in front.  This is a
    // chirality flip (R → -R changes det sign; we also negate r3 so
    // det stays +1 since flipping any column twice nets out).
    if (t[2] < 0.0f) {
        r1[0] = -r1[0]; r1[1] = -r1[1]; r1[2] = -r1[2];
        r2[0] = -r2[0]; r2[1] = -r2[1]; r2[2] = -r2[2];
        r3[0] = -r3[0]; r3[1] = -r3[1]; r3[2] = -r3[2];
        t[0] = -t[0]; t[1] = -t[1]; t[2] = -t[2];
    }
    // Build R (columns are r1, r2, r3) row-major.
    const float R[9] = {
        r1[0], r2[0], r3[0],
        r1[1], r2[1], r3[1],
        r1[2], r2[2], r3[2],
    };
    float err;
    if (aruco_reproj_err(R, t, mx, my, u, v,
                         fx, fy, cx, cy, &err) != 0) return -1;
    tvec_out[0] = t[0];
    tvec_out[1] = t[1];
    tvec_out[2] = t[2];
    sentai_aruco_R_to_rvec(R, rvec_out);
    if (reproj_err_out) *reproj_err_out = err;
    return 0;
}

// =========================================================================
// Test helper — generate a synthetic 320x240 frame with one marker
// centered, then run detection on it.  Static-buffer C path so the
// 76 KB grayscale frame never crosses the MP boundary (heap-bound).
//
// rotation_cw is the number of 90° CW rotations applied to the printed
// pattern (the detector should report the same id back with the
// corresponding orientation handled internally).
//
// Returns the number of markers detected (the cache holds the
// per-marker breakdown for the caller via get_latest()).
// =========================================================================
static uint8_t s_test_gray[ARUCO_BUF_SZ] ARUCO_BSS_ATTR;

static uint16_t aruco_rotate_pattern_cw(uint16_t pattern, int times) {
    uint16_t cur = pattern;
    for (int t = 0; t < times; ++t) {
        uint16_t rot = 0;
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i) {
                if (cur & (uint16_t)(1u << (j * 4 + i))) {
                    rot |= (uint16_t)(1u << (i * 4 + (3 - j)));
                }
            }
        }
        cur = rot;
    }
    return cur;
}

// =========================================================================
// OP-S10-W14-T16/T19 — load a PGM file (P5, 320x240, 8-bit gray) and
// run detection on it.  Used by s175 to compare per-marker tvec_cam
// across image rotations (drone yaw) and confirm/refute the
// planar-marker pose ambiguity hypothesis.
//
// Anti-cheat: caller hands a file path; C does the file read + detect.
// No heavy data crosses MP.
//
// Returns n_dets on success, -1 if file can't be opened, -2 if format
// is unexpected.
extern "C" int sentai_aruco_detect_pgm_file(const char* path) {
    FILE* fp = fopen(path, "rb");
    if (!fp) return -1;
    char header[3] = {0};
    int w = 0, h = 0, maxval = 0;
    if (fscanf(fp, "%2s %d %d %d", header, &w, &h, &maxval) != 4
        || header[0] != 'P' || header[1] != '5') {
        fclose(fp); return -2;
    }
    if (w != 320 || h != 240 || maxval != 255) {
        fprintf(stderr, "detect_pgm: expected 320x240 maxval=255 P5, "
                "got %dx%d maxval=%d header=%s\n", w, h, maxval, header);
        fclose(fp); return -2;
    }
    // Skip the single whitespace after maxval, then read 76800 bytes.
    fgetc(fp);
    const size_t expected = (size_t)w * (size_t)h;
    if (expected > sizeof(s_test_gray)) { fclose(fp); return -2; }
    if (fread(s_test_gray, 1, expected, fp) != expected) {
        fclose(fp); return -2;
    }
    fclose(fp);
    sentai_aruco_marker_t local[SENTAI_ARUCO_MAX_MARKERS];
    int n = sentai_aruco_detect(s_test_gray, w, h, 0, 0,
                                  local, SENTAI_ARUCO_MAX_MARKERS);
    // Per-marker dump to stderr for the s175 verdict.  C-side print,
    // no MP heap crossing.
    if (n > 0) {
        for (int i = 0; i < n; ++i) {
            fprintf(stderr,
              "PGM_RESULT id=%u tvec=(%+.4f,%+.4f,%+.4f) "
              "rvec=(%+.4f,%+.4f,%+.4f) reproj=%.3f path=%s\n",
              (unsigned)local[i].marker_id,
              (double)local[i].tvec_cam[0], (double)local[i].tvec_cam[1],
              (double)local[i].tvec_cam[2],
              (double)local[i].rvec_cam[0], (double)local[i].rvec_cam[1],
              (double)local[i].rvec_cam[2],
              (double)local[i].reproj_err_px, path);
        }
    } else {
        fprintf(stderr, "PGM_RESULT n=%d path=%s\n", n, path);
    }
    return n;
}

extern "C" int sentai_aruco_test_synth_and_detect(int marker_id,
                                                   int side_px,
                                                   int rotation_cw) {
    if (marker_id < 0 || marker_id >= s_aruco_dict_n) return -1;
    if (side_px < 30 || side_px > 200) return -2;
    const int W = 320, H = 240;
    const int CX = W / 2;
    const int CY = H / 2;
    const int half = side_px / 2;
    // 6 cells in marker (1 border on each side + 4 data inner).
    const int cell = side_px / 6;
    if (cell < 4) return -3;
    // Background: light gray 200.
    memset(s_test_gray, 200, (size_t)W * (size_t)H);

    // Compose the rotated marker pattern.
    const uint16_t pat = aruco_rotate_pattern_cw(s_aruco_dict[marker_id],
                                                  rotation_cw);
    // Draw 6x6 grid: outer ring all BLACK; inner 4x4 per pattern.
    for (int j = 0; j < 6; ++j) {
        for (int i = 0; i < 6; ++i) {
            int is_black;
            if (i == 0 || i == 5 || j == 0 || j == 5) {
                is_black = 1;       // border
            } else {
                const int bi = i - 1;
                const int bj = j - 1;
                is_black = (pat >> (bj * 4 + bi)) & 1;
            }
            if (!is_black) continue;
            const int px0 = CX - half + i * cell;
            const int py0 = CY - half + j * cell;
            for (int dy = 0; dy < cell; ++dy) {
                for (int dx = 0; dx < cell; ++dx) {
                    const int x = px0 + dx;
                    const int y = py0 + dy;
                    if (x >= 0 && x < W && y >= 0 && y < H) {
                        s_test_gray[x + y * W] = 50;     // dark
                    }
                }
            }
        }
    }
    sentai_aruco_marker_t local[SENTAI_ARUCO_MAX_MARKERS];
    return sentai_aruco_detect(s_test_gray, W, H, 0, 0,
                                local, SENTAI_ARUCO_MAX_MARKERS);
}

// =========================================================================
// Public API
// =========================================================================
extern "C" int sentai_aruco_init(void) {
    sentai_aruco_clear();
    s_initialised = 1;
    return 0;
}

extern "C" void sentai_aruco_clear(void) {
    memset(s_cache, 0, sizeof(s_cache));
    s_cache_count = 0;
    memset(&s_stats, 0, sizeof(s_stats));
}

extern "C" void sentai_aruco_set_intrinsics(float fx, float fy,
                                             float cx, float cy) {
    if (!isfinite(fx) || !isfinite(fy) || !isfinite(cx) || !isfinite(cy)) {
        return;
    }
    s_fx = fx; s_fy = fy; s_cx = cx; s_cy = cy;
}

extern "C" int sentai_aruco_set_marker_size(float size_m) {
    if (!isfinite(size_m) || size_m <= 0.0f) {
        return -SENTAI_ARUCO_ERR_BAD_MSIZE;
    }
    s_marker_size_m = size_m;
    return 0;
}

extern "C" int sentai_aruco_detect(const uint8_t* gray, int w, int h,
                                    uint32_t frame_seq, uint32_t src_ts_ms,
                                    sentai_aruco_marker_t* out,
                                    int out_capacity) {
    if (!gray || w <= 0 || h <= 0 || w > ARUCO_MAX_W || h > ARUCO_MAX_H) {
        return -SENTAI_ARUCO_ERR_BAD_INPUT;
    }
    if (!s_initialised) return -SENTAI_ARUCO_ERR_NO_INTR;

    s_stats.frames_total++;
    if (out_capacity <= 0) return 0;

    // T18-G: cv2.aruco-style multi-scale adaptive threshold.  cv2 default
    // is (winSizeMin=3, winSizeMax=23, winSizeStep=10) → blocks 3,13,23.
    // We use a coarser set tuned for our 320x240 scene (markers at
    // ~25 px to ~80 px).  The full pipeline runs at each scale; markers
    // detected at multiple scales are deduplicated by marker_id (best
    // reproj wins).
    static const int SCALE_BLOCKS[] = { 23, 51, 101, 201 };
    constexpr int N_SCALES = (int)(sizeof(SCALE_BLOCKS) / sizeof(SCALE_BLOCKS[0]));

    // Per-id best candidate (one slot per known marker, ids 0..N-1).
    sentai_aruco_marker_t best[SENTAI_ARUCO_MAX_MARKERS];
    bool best_set[SENTAI_ARUCO_MAX_MARKERS];
    for (int i = 0; i < SENTAI_ARUCO_MAX_MARKERS; ++i) best_set[i] = false;

    for (int si = 0; si < N_SCALES; ++si) {
        aruco_adaptive_threshold(gray, w, h, SCALE_BLOCKS[si]);
        const int n_comp = aruco_label_components(w, h);

        for (int ci = 0; ci < n_comp; ++ci) {
            const aruco_comp_t* c = &s_components[ci];
            if (c->touches_border) continue;
            const float area = (float)c->n_pix;
            if (area < SENTAI_ARUCO_MIN_QUAD_AREA) continue;
            const int bbox_w = c->x1 - c->x0 + 1;
            const int bbox_h = c->y1 - c->y0 + 1;
            if (bbox_w < 8 || bbox_h < 8) continue;
            const float aspect = (float)bbox_w / (float)bbox_h;
            if (aspect < 0.33f || aspect > 3.0f) continue;
            const float fill_ratio = area / (float)(bbox_w * bbox_h);
            if (fill_ratio < 0.30f) continue;

            const uint8_t lab = (uint8_t)(ci + 1);
            float corners[8];
            if (aruco_extract_quad(lab, w, h, c, corners) != 0) continue;

            // T18-K: enforce CW winding (js-aruco / cv2.aruco standard).
            // Cross product (c1-c0) × (c2-c0): negative → CCW → swap c1,c3.
            {
                const float dx1 = corners[2] - corners[0];
                const float dy1 = corners[3] - corners[1];
                const float dx2 = corners[4] - corners[0];
                const float dy2 = corners[5] - corners[1];
                if (dx1 * dy2 - dy1 * dx2 < 0.0f) {
                    float tx = corners[2], ty = corners[3];
                    corners[2] = corners[6]; corners[3] = corners[7];
                    corners[6] = tx;         corners[7] = ty;
                }
            }

            for (int k = 0; k < 4; ++k) {
                aruco_refine_corner_subpix(gray, w, h,
                                             &corners[k*2 + 0],
                                             &corners[k*2 + 1]);
            }

            int rotation = 0;
            int hamming = 0;
            int mid = aruco_decode_marker(gray, corners, w, h,
                                                 &rotation, &hamming);
            // T18-I: cv2's findContours produces CCW for outer contours
            // while our Moore-Neighbor produces CW.  If dict-decode fails
            // on the CW ordering, try the CCW (reverse) ordering before
            // giving up.  Reverses the marker pattern in 4×4 grid, which
            // dict-decode's 4-rotation search alone cannot recover.
            if (mid < 0) {
                float corners_rev[8] = {
                    corners[0], corners[1],   // TL stays
                    corners[6], corners[7],   // BL → "TR"
                    corners[4], corners[5],   // BR stays
                    corners[2], corners[3],   // TR → "BL"
                };
                int rot2 = 0, hamm2 = 0;
                int mid2 = aruco_decode_marker(gray, corners_rev, w, h,
                                                 &rot2, &hamm2);
                if (mid2 >= 0) {
                    mid = mid2;
                    rotation = rot2;
                    hamming = hamm2;
                    memcpy(corners, corners_rev, sizeof(corners));
                }
            }
            if (mid < 0) {
                s_stats.rejected_dict_total++;
                continue;
            }
            aruco_realign_corners(corners, (4 - rotation) % 4);

            float tvec[3], rvec[3], reproj;
            if (aruco_pnp_from_corners(corners, s_fx, s_fy, s_cx, s_cy,
                                        s_marker_size_m,
                                        tvec, rvec, &reproj) != 0) {
                s_stats.rejected_reproj_total++;
                continue;
            }
            if (reproj > ARUCO_REPROJ_GATE_PX) {
                s_stats.rejected_reproj_total++;
                continue;
            }

            if (mid < 0 || mid >= SENTAI_ARUCO_MAX_MARKERS) continue;
            if (best_set[mid] && best[mid].reproj_err_px <= reproj) continue;
            sentai_aruco_marker_t* m = &best[mid];
            memset(m, 0, sizeof(*m));
            m->marker_id     = (uint8_t)mid;
            m->hamming       = (uint8_t)hamming;
            memcpy(m->tvec_cam,    tvec,    sizeof(tvec));
            memcpy(m->rvec_cam,    rvec,    sizeof(rvec));
            memcpy(m->corners_px,  corners, sizeof(corners));
            m->reproj_err_px = reproj;
            m->detect_us     = 0;
            m->src_ts_ms     = src_ts_ms;
            m->frame_seq     = frame_seq;
            best_set[mid] = true;
        }
    }

    int n_out = 0;
    for (int mid = 0; mid < SENTAI_ARUCO_MAX_MARKERS && n_out < out_capacity; ++mid) {
        if (!best_set[mid]) continue;
        out[n_out++] = best[mid];
        s_stats.markers_total++;
    }

    s_stats.frames_with_detect += (n_out > 0) ? 1u : 0u;
    if (n_out > 0) {
        const int n_cache = (n_out < SENTAI_ARUCO_MAX_MARKERS)
                                ? n_out : SENTAI_ARUCO_MAX_MARKERS;
        memcpy(s_cache, out, n_cache * sizeof(s_cache[0]));
        s_cache_count = n_cache;
    } else {
        s_cache_count = 0;
    }
    return n_out;
}

extern "C" int sentai_aruco_get_latest(sentai_aruco_marker_t* out,
                                        int out_capacity) {
    if (!out || out_capacity <= 0) return 0;
    const int n = (s_cache_count < out_capacity) ? s_cache_count : out_capacity;
    memcpy(out, s_cache, n * sizeof(sentai_aruco_marker_t));
    return n;
}

extern "C" void sentai_aruco_get_stats(sentai_aruco_stats_t* out) {
    if (!out) return;
    memcpy(out, &s_stats, sizeof(*out));
}

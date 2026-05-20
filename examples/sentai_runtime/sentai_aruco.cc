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

// ARM Cortex-M7 DSP-extension intrinsics (aruco_usub8, aruco_sel, __UQADD8).
// Toolchain `arm_acle.h` doesn't ship these on this gcc 9.3 SDK; we wrap
// the inline-asm forms directly (verbatim from CMSIS/Core cmsis_gcc.h).
// SIM (x86_64) builds use scalar fallback.
#ifdef __arm__
  __attribute__((always_inline)) static inline uint32_t aruco_usub8(uint32_t a, uint32_t b) {
      uint32_t r;
      __asm volatile ("usub8 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
      return r;
  }
  __attribute__((always_inline)) static inline uint32_t aruco_sel(uint32_t a, uint32_t b) {
      uint32_t r;
      __asm volatile ("sel %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
      return r;
  }
  #define ARUCO_HAVE_DSP_SIMD 1
#else
  #define ARUCO_HAVE_DSP_SIMD 0
#endif

// =========================================================================
// Compile-time configuration.
// =========================================================================
// Reduced from 640x480 to 320x240 (production resolution) at OP-S10-W17
// (2026-05-19): PrepTask SLOT_GRAY_NATIVE produces 320x240 Y8, never
// 640x480, so the over-provisioned buffers (300+300+1230 KB) wasted
// 1.4 MB SDRAM that wasn't usable for anything else AND blocked these
// buffers from being moved into the 103 KB-free M7 OCRAM region.  No
// external call sites depend on the 640 max; verified by grep before
// the change.  Re-raise to 640 ONLY if a future PrepTask slot at full
// camera native enters the ArUco/WhyCon pipelines.
#define ARUCO_MAX_W                    320
#define ARUCO_MAX_H                    240
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
// All hot buffers are 32 B aligned to match the Cortex-M7 D-cache line —
// avoids partial-line invalidates on mixed read/write paths and lets the
// compiler emit wide LDM/STM in the threshold inner loop without crossing
// a cache line per 4-pixel chunk.  Per OP-S10-W15 alignment policy.
// =========================================================================
#ifdef __arm__
#define ARUCO_BSS_ATTR   __attribute__((section(".sdram_bss"), aligned(32)))
// OP-S10-W16-T3.9: hottest random-access buffers in OCRAM.  s_binary is
// read repeatedly by flood-fill (Phase B) + warp_to_canonical (Phase E),
// and written by threshold (Phase A).  s_fill_stack is push/pop hot in
// DFS.  s_labels is intentionally kept in SDRAM — putting all three in
// OCRAM would overflow the 103 KB free post-.tpu_input budget; D-cache
// absorbs s_labels' mostly-sequential access pattern.
#define ARUCO_OCRAM_ATTR __attribute__((section(".ocram_bss"), aligned(32)))
#else
#define ARUCO_BSS_ATTR   __attribute__((aligned(32)))
#define ARUCO_OCRAM_ATTR __attribute__((aligned(32)))
#endif

// OP-S10-W17 memory placement strategy:
//  - s_test_gray  (75 KB) → OCRAM: hottest read in Phase A (rolling
//    threshold's full-frame column-sum scan).  PXP places the camera
//    Y8 here in production, so OCRAM is also the "image already lives"
//    spot — zero copy needed.
//  - s_fill_stack (16 KB) → OCRAM: hottest push/pop in Phase B flood
//    fill DFS.  Stack ops are random-access by access frequency, so
//    OCRAM 3-cyc beats SDRAM 50-cyc cache-miss every push.
//  - s_binary, s_labels, s_components, s_integral → stay in SDRAM:
//    103 KB OCRAM free after .tpu_input fits exactly 91 KB (gray+stack);
//    s_binary (75 KB) doesn't fit alongside.  D-cache absorbs s_binary
//    sequential writes in Phase A and the (mostly) sequential reads in
//    Phase B label assignment.  s_labels is touched once per pixel per
//    Phase W2 bbox scan; SDRAM with prefetch is acceptable there.
static uint8_t  s_binary[ARUCO_BUF_SZ]   ARUCO_BSS_ATTR;
static uint8_t  s_labels[ARUCO_BUF_SZ]   ARUCO_BSS_ATTR;
static int32_t  s_integral[(ARUCO_MAX_W + 1) * (ARUCO_MAX_H + 1)] ARUCO_BSS_ATTR;
static int32_t  s_fill_stack[ARUCO_FILL_STACK_SZ] ARUCO_OCRAM_ATTR;

typedef struct {
    int      x0, y0, x1, y1;     // bounding box (inclusive)
    int      cx_sum, cy_sum;     // first-order centroid accumulator (m10, m01)
    int      n_pix;
    // OP-S10-W17-T2: 2nd-order moment accumulators, populated in
    // aruco_label_components flood-fill so the WhyCon path doesn't
    // need a separate Phase W2 rescan over the bbox.  64-bit because
    // m20 = Σ x² can reach ~76800 × 320² ≈ 8 G for a full-frame blob.
    int64_t  m20_sum;
    int64_t  m02_sum;
    int64_t  m11_sum;
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

// Verbatim copy of the pre-2026-05-19 scalar implementation, kept as a
// reference for the runtime byte-equivalence check
// (`sentai_aruco_adaptive_threshold_verify`).  NOT called from production
// — that uses `aruco_adaptive_threshold` below.
static void aruco_adaptive_threshold_scalar_ref(const uint8_t* gray,
                                                 int w, int h, int block,
                                                 uint8_t* out_binary) {
    const int W = w, H = h;
    const int stride_i = W + 1;
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
            out_binary[x + y * W] = ((int32_t)v < mean - ARUCO_THRESH_C)
                                      ? 1u : 0u;
        }
    }
}

// OP-S10-W14-T18-T iteration history (kept here so the dead ends and
// the win are both documented in-line):
//
//   - Iter 1: per-column LUTs + division-elimination in pure C.
//     Measured 24 % SLOWER; gcc -O2 already hoists invariants and
//     converts the divide to multiply-by-reciprocal.  LUT loads
//     added 3 cacheable hits per pixel and broke pipelining.
//   - Iter 2: ITCM placement via `__attribute__((section(".ramfunc")))`.
//     Measured 16 % SLOWER; function-call overhead from breaking
//     inlining outweighed the ITCM fetch win on an 870-byte kernel.
//   - Iter 3: explicit CMSIS-DSP intrinsics (aruco_usub8 / aruco_sel /
//     __UQADD8) for the interior columns + divide-free comparison
//     ((gray+C+1)*box_area <= box_sum, math-identical to mean-based
//     form).  This is the current production variant
//     (`aruco_adaptive_threshold` below).
//
// Byte-equivalence vs the scalar reference is asserted at runtime by
// sentai_aruco_adaptive_threshold_verify() — any future Phase-2 refactor
// MUST keep that at 0 mismatches.  Cycle counts queryable from MP via
// sentai.aruco._thresh_cycles().
//
// Production threshold — Phase 1 scalar (serial prefix sum), Phase 2 split:
//   - Interior columns (x_factor = block, constant per row): 4-wide SIMD
//     compare via aruco_usub8/aruco_sel on Cortex-M7 DSP extensions.  Divide-free
//     via algebraic identity (gray<mean-C ⟺ mean-gray >= C+1).
//   - Border columns (x_factor varies per pixel): scalar reference path.
// On non-ARM (SIM) builds, falls back to scalar throughout — same math,
// confirmed byte-identical by sentai_aruco_adaptive_threshold_verify().
static void aruco_adaptive_threshold(const uint8_t* gray, int w, int h,
                                       int block) {
    const int W = w, H = h;
    const int stride_i = W + 1;
    // Phase 1 — integral image build (serial prefix sum; hard to vectorise).
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
    // Interior x range: [x_int_start .. x_int_end] inclusive, where both
    // x-half >= 0 and x+half < W.  For W=320 block=201: [100, 219], 120
    // cols (37.5 % of frame).  Borders use scalar path.
    const int x_int_start = (half <= W - 1) ? half : W;
    const int x_int_end   = (W - half - 1 >= 0) ? (W - half - 1) : -1;
#if ARUCO_HAVE_DSP_SIMD
    const uint32_t C_plus_1_pack = (uint32_t)(ARUCO_THRESH_C + 1) * 0x01010101u;
    const uint32_t ones_pack     = 0x01010101u;
    const uint32_t zeros_pack    = 0x00000000u;
#endif
    for (int y = 0; y < H; ++y) {
        int y0 = y - half;        if (y0 < 0) y0 = 0;
        int y1 = y + half;        if (y1 >= H) y1 = H - 1;
        const int32_t y_factor   = y1 - y0 + 1;
        const int32_t* int_top   = s_integral + (y0)     * stride_i;
        const int32_t* int_bot   = s_integral + (y1 + 1) * stride_i;
        const uint8_t* gray_row  = gray     + y * W;
        uint8_t*       bin_row   = s_binary + y * W;
        // Left border — scalar with x clamping.
        for (int x = 0; x < x_int_start; ++x) {
            int x0 = x - half; if (x0 < 0) x0 = 0;
            int x1 = x + half; if (x1 >= W) x1 = W - 1;
            const int32_t bs = int_bot[x1 + 1] - int_bot[x0]
                              - int_top[x1 + 1] + int_top[x0];
            const int32_t ba = (x1 - x0 + 1) * y_factor;
            const int32_t mean = bs / ba;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C)
                          ? 1u : 0u;
        }
#if ARUCO_HAVE_DSP_SIMD
        // Interior — 4-wide SIMD compare.  Divide-free: compare
        //   (gray + C + 1) * box_area <= box_sum
        // box_area = block * y_factor is constant inside this row.
        const int32_t box_area_int = (int32_t)block * y_factor;
        int x = x_int_start;
        for (; x + 4 <= x_int_end + 1; x += 4) {
            // Four box_sums.  Hardcoded offsets (no LUT) so the compiler
            // can keep everything in registers.
            const int32_t bs0 = int_bot[x + 0 + half + 1] - int_bot[x + 0 - half]
                              - int_top[x + 0 + half + 1] + int_top[x + 0 - half];
            const int32_t bs1 = int_bot[x + 1 + half + 1] - int_bot[x + 1 - half]
                              - int_top[x + 1 + half + 1] + int_top[x + 1 - half];
            const int32_t bs2 = int_bot[x + 2 + half + 1] - int_bot[x + 2 - half]
                              - int_top[x + 2 + half + 1] + int_top[x + 2 - half];
            const int32_t bs3 = int_bot[x + 3 + half + 1] - int_bot[x + 3 - half]
                              - int_top[x + 3 + half + 1] + int_top[x + 3 - half];
            // Mean = bs/ba (gcc-O2 converts to multiply-by-reciprocal
            // since box_area_int is loop-invariant).  Bounded 0..255.
            const uint32_t m0 = (uint32_t)(bs0 / box_area_int) & 0xFFu;
            const uint32_t m1 = (uint32_t)(bs1 / box_area_int) & 0xFFu;
            const uint32_t m2 = (uint32_t)(bs2 / box_area_int) & 0xFFu;
            const uint32_t m3 = (uint32_t)(bs3 / box_area_int) & 0xFFu;
            const uint32_t m_pack = m0 | (m1 << 8) | (m2 << 16) | (m3 << 24);
            // Load 4 gray bytes as one uint32_t (M7 supports unaligned LDR.W).
            const uint32_t g_pack = *(const uint32_t*)(gray_row + x);
            // Two-step compare so the mean<gray underflow doesn't fool us:
            //   1) raw = USUB8(mean, gray); GE[i]=1 iff mean[i]>=gray[i]
            //   2) clamped = SEL(raw, 0)   — zero out lanes where mean<gray
            //   3) USUB8(clamped, C+1)     — GE[i]=1 iff diff >= C+1
            //   4) bin = SEL(1, 0)
            const uint32_t raw      = aruco_usub8(m_pack, g_pack);
            const uint32_t clamped  = aruco_sel(raw, zeros_pack);
            (void)              aruco_usub8(clamped, C_plus_1_pack);
            const uint32_t bin_pack = aruco_sel(ones_pack, zeros_pack);
            *(uint32_t*)(bin_row + x) = bin_pack;
        }
        // Tail of interior (1-3 pixels) — scalar, no clamping needed.
        for (; x <= x_int_end; ++x) {
            const int32_t bs = int_bot[x + half + 1] - int_bot[x - half]
                              - int_top[x + half + 1] + int_top[x - half];
            const int32_t mean = bs / box_area_int;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C)
                          ? 1u : 0u;
        }
#else
        // Non-ARM (SIM) fallback for interior — scalar, no clamps.
        const int32_t box_area_int = (int32_t)block * y_factor;
        for (int x = x_int_start; x <= x_int_end; ++x) {
            const int32_t bs = int_bot[x + half + 1] - int_bot[x - half]
                              - int_top[x + half + 1] + int_top[x - half];
            const int32_t mean = bs / box_area_int;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C)
                          ? 1u : 0u;
        }
#endif
        // Right border — scalar with x1 clamping.
        for (int x = x_int_end + 1; x < W; ++x) {
            int x0 = x - half; if (x0 < 0) x0 = 0;
            int x1 = x + half; if (x1 >= W) x1 = W - 1;
            const int32_t bs = int_bot[x1 + 1] - int_bot[x0]
                              - int_top[x1 + 1] + int_top[x0];
            const int32_t ba = (x1 - x0 + 1) * y_factor;
            const int32_t mean = bs / ba;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C)
                          ? 1u : 0u;
        }
    }
}

// (sentai_aruco_adaptive_threshold_verify is defined after s_test_gray below.)

// =========================================================================
// Pipeline stage A.2 — rolling-integral Bradley adaptive threshold.
//
// Rationale: the full integral image (W+1)*(H+1)*4 = 309 KB @ 320×240 is
// the largest single buffer in the ArUco pipeline, far too large to place
// in M7 OCRAM alongside the 900 KB .tpu_input staging tensor.  With SDRAM-
// backed integral image, the 32 KB D-cache cannot hold the full working set
// during Phase 2's row-stride scan, so SDRAM accesses dominate the kernel.
//
// This variant uses NO integral image.  It maintains a single running
// column-sum (col_sum[x] = vertical sum over the current [y_top..y_bot]
// band) and, per output row, builds a 1D prefix sum (prefix_x[]) that
// gives O(1) box-sum lookup.  Memory cost: 4*W + 4*(W+1) = ~2.6 KB scratch,
// comfortably in OCRAM.
//
// Math identity:
//   prefix_x[i+1] - prefix_x[i] = col_sum[i] = sum_{yy=y_top..y_bot} gray[yy][i]
//   box_sum = prefix_x[x1+1] - prefix_x[x0]
//           = sum_{x'=x0..x1} col_sum[x']
//           = sum over rectangle [x0..x1] × [y_top..y_bot]  ✓
//   Byte-identical to scalar reference; asserted at runtime via
//   sentai_aruco_adaptive_threshold_rolling_verify().
#ifdef __arm__
static int32_t s_rolling_col_sum [ARUCO_MAX_W]
    __attribute__((section(".ocram_bss"), aligned(32)));
static int32_t s_rolling_prefix_x[ARUCO_MAX_W + 1]
    __attribute__((section(".ocram_bss"), aligned(32)));
#else
static int32_t s_rolling_col_sum [ARUCO_MAX_W];
static int32_t s_rolling_prefix_x[ARUCO_MAX_W + 1];
#endif

static void aruco_adaptive_threshold_rolling(const uint8_t* gray, int w, int h,
                                              int block, uint8_t* out_binary) {
    const int W = w, H = h;
    const int half = block / 2;

    // Initialise running column sums for the y=0 band: [0..min(half,H-1)].
    int y_top = 0;
    int y_bot = (half < H - 1) ? half : H - 1;
    for (int x = 0; x < W; ++x) s_rolling_col_sum[x] = 0;
    for (int yy = y_top; yy <= y_bot; ++yy) {
        const uint8_t* row = gray + yy * W;
        for (int x = 0; x < W; ++x) s_rolling_col_sum[x] += row[x];
    }

    for (int y = 0; y < H; ++y) {
        const int y_top_new = (y - half >= 0) ? y - half : 0;
        const int y_bot_new = (y + half < H)  ? y + half : H - 1;

        // Expand bottom (add rows y_bot+1..y_bot_new).
        while (y_bot < y_bot_new) {
            ++y_bot;
            const uint8_t* row = gray + y_bot * W;
            for (int x = 0; x < W; ++x) s_rolling_col_sum[x] += row[x];
        }
        // Contract top (remove rows y_top..y_top_new-1).
        while (y_top < y_top_new) {
            const uint8_t* row = gray + y_top * W;
            for (int x = 0; x < W; ++x) s_rolling_col_sum[x] -= row[x];
            ++y_top;
        }

        const int32_t box_h = y_bot - y_top + 1;

        // Build prefix_x from col_sum.
        s_rolling_prefix_x[0] = 0;
        int32_t acc = 0;
        for (int x = 0; x < W; ++x) {
            acc += s_rolling_col_sum[x];
            s_rolling_prefix_x[x + 1] = acc;
        }

        // Threshold each pixel in this row.  Split into left-border /
        // interior (constant box_w = block, SIMD-able) / right-border —
        // same shape as the production aruco_adaptive_threshold but
        // operating on the row's prefix_x instead of a 2-D integral image.
        const uint8_t* gray_row = gray + y * W;
        uint8_t* bin_row = out_binary + y * W;
        const int x_int_start = (half <= W - 1) ? half : W;
        const int x_int_end   = (W - half - 1 >= 0) ? (W - half - 1) : -1;
        const int32_t box_area_int = (int32_t)block * box_h;
#if ARUCO_HAVE_DSP_SIMD
        const uint32_t C_plus_1_pack = (uint32_t)(ARUCO_THRESH_C + 1) * 0x01010101u;
        const uint32_t ones_pack     = 0x01010101u;
        const uint32_t zeros_pack    = 0x00000000u;
#endif
        // Left border: x in [0 .. x_int_start) — variable box_w.
        for (int x = 0; x < x_int_start; ++x) {
            int x0 = x - half; if (x0 < 0) x0 = 0;
            int x1 = x + half; if (x1 >= W) x1 = W - 1;
            const int32_t bs = s_rolling_prefix_x[x1 + 1] - s_rolling_prefix_x[x0];
            const int32_t ba = (x1 - x0 + 1) * box_h;
            const int32_t mean = bs / ba;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C) ? 1u : 0u;
        }
#if ARUCO_HAVE_DSP_SIMD
        // Interior: x_int_start..x_int_end inclusive, box_w = block.
        // Divide-free compare: (gray+C+1)*box_area_int <= box_sum.
        // 4-wide using USUB8 + SEL.
        int x = x_int_start;
        for (; x + 4 <= x_int_end + 1; x += 4) {
            const int32_t bs0 = s_rolling_prefix_x[x + 0 + half + 1]
                              - s_rolling_prefix_x[x + 0 - half];
            const int32_t bs1 = s_rolling_prefix_x[x + 1 + half + 1]
                              - s_rolling_prefix_x[x + 1 - half];
            const int32_t bs2 = s_rolling_prefix_x[x + 2 + half + 1]
                              - s_rolling_prefix_x[x + 2 - half];
            const int32_t bs3 = s_rolling_prefix_x[x + 3 + half + 1]
                              - s_rolling_prefix_x[x + 3 - half];
            const uint32_t m0 = (uint32_t)(bs0 / box_area_int) & 0xFFu;
            const uint32_t m1 = (uint32_t)(bs1 / box_area_int) & 0xFFu;
            const uint32_t m2 = (uint32_t)(bs2 / box_area_int) & 0xFFu;
            const uint32_t m3 = (uint32_t)(bs3 / box_area_int) & 0xFFu;
            const uint32_t m_pack = m0 | (m1 << 8) | (m2 << 16) | (m3 << 24);
            const uint32_t g_pack = *(const uint32_t*)(gray_row + x);
            const uint32_t raw      = aruco_usub8(m_pack, g_pack);
            const uint32_t clamped  = aruco_sel(raw, zeros_pack);
            (void)              aruco_usub8(clamped, C_plus_1_pack);
            const uint32_t bin_pack = aruco_sel(ones_pack, zeros_pack);
            *(uint32_t*)(bin_row + x) = bin_pack;
        }
        // Tail of interior — scalar, no clamps needed.
        for (; x <= x_int_end; ++x) {
            const int32_t bs = s_rolling_prefix_x[x + half + 1]
                              - s_rolling_prefix_x[x - half];
            const int32_t mean = bs / box_area_int;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C) ? 1u : 0u;
        }
#else
        // Non-ARM fallback for interior — scalar, no clamps.
        for (int x = x_int_start; x <= x_int_end; ++x) {
            const int32_t bs = s_rolling_prefix_x[x + half + 1]
                              - s_rolling_prefix_x[x - half];
            const int32_t mean = bs / box_area_int;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C) ? 1u : 0u;
        }
#endif
        // Right border: x in (x_int_end .. W) — variable box_w.
        for (int x = x_int_end + 1; x < W; ++x) {
            int x0 = x - half; if (x0 < 0) x0 = 0;
            int x1 = x + half; if (x1 >= W) x1 = W - 1;
            const int32_t bs = s_rolling_prefix_x[x1 + 1] - s_rolling_prefix_x[x0];
            const int32_t ba = (x1 - x0 + 1) * box_h;
            const int32_t mean = bs / ba;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C) ? 1u : 0u;
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
            c->m20_sum = 0; c->m02_sum = 0; c->m11_sum = 0;
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
                c->m20_sum += (int64_t)px * px;
                c->m02_sum += (int64_t)py * py;
                c->m11_sum += (int64_t)px * py;
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
    // cv2 init_iters: iterate 3× finding farthest point from current
    // seed to converge on a well-spread (seed_a, seed_b) pair.  Without
    // this, a single farthest-from-0 search can pick a suboptimal pair
    // for non-axis-aligned shapes and miss the actual marker corners.
    int seed_a = 0;
    int seed_b = 0;
    for (int it = 0; it < 3; ++it) {
        float max_d = -1.0f;
        const int ax = pts[seed_a * 2 + 0];
        const int ay = pts[seed_a * 2 + 1];
        int new_b = seed_a;
        for (int j = 0; j < n; ++j) {
            if (j == seed_a) continue;
            const float dx = (float)(pts[j*2 + 0] - ax);
            const float dy = (float)(pts[j*2 + 1] - ay);
            const float d  = dx*dx + dy*dy;
            if (d > max_d) { max_d = d; new_b = j; }
        }
        seed_b = new_b;
        if (it < 2) seed_a = seed_b;     // advance for next iter; keep
                                          // final pair distinct
    }
    if (seed_a == seed_b) return;        // degenerate (n<2)
    // Final seed_a and seed_b are roughly diametrically opposite.
    s_dp_keep[seed_a] = 1;
    s_dp_keep[seed_b] = 1;
    // Stack of (start, end) index pairs, closed-polygon convention
    // where end may equal start + n to wrap.
    int stack_s[ARUCO_DP_STACK_MAX];
    int stack_e[ARUCO_DP_STACK_MAX];
    int top = 0;
    // Two slices: (seed_a, seed_b) and (seed_b, seed_a + n).
    const int sa = seed_a;
    const int sb = (seed_b < seed_a) ? seed_b + n : seed_b;
    stack_s[top] = sa;       stack_e[top] = sb;        top++;
    stack_s[top] = sb;       stack_e[top] = sa + n;    top++;

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

// T18-O step 2: cv2 isContourConvex check for a quadrilateral.
// A 4-vertex polygon is convex iff all 4 cross-products of adjacent
// edges have the same sign.  Returns 1 if convex, 0 if not.
static int aruco_is_quad_convex_(const float corners[8]) {
    int sign = 0;
    for (int i = 0; i < 4; ++i) {
        const float ax = corners[((i+1)%4)*2 + 0] - corners[i*2 + 0];
        const float ay = corners[((i+1)%4)*2 + 1] - corners[i*2 + 1];
        const float bx = corners[((i+2)%4)*2 + 0] - corners[((i+1)%4)*2 + 0];
        const float by = corners[((i+2)%4)*2 + 1] - corners[((i+1)%4)*2 + 1];
        const float cross = ax * by - ay * bx;
        if (cross > 0.0f) {
            if (sign < 0) return 0;
            sign = +1;
        } else if (cross < 0.0f) {
            if (sign > 0) return 0;
            sign = -1;
        }
    }
    return 1;
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

    // cv2 uses single-pass approxPolyDP eps=perim*0.03.  Our
    // Moore-Neighbor contour is staircase-noisy vs cv2's Suzuki-Abe
    // output, so we sweep eps from cv2's default upward until DP
    // yields exactly 4 vertices.  Reverting to single-pass dropped
    // detection 72 % → 49 %.  Until findContours port (T18-P), the
    // sweep is the correct compensation.
    int n_kept = 0;
    int kept_idx[16];
    static const float EPS_FRACS[] = {
        0.03f, 0.04f, 0.05f, 0.06f, 0.07f, 0.08f, 0.10f, 0.02f
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

// Forward decls — definitions lower in file.
static int aruco_dlt_homography(const float mx[4], const float my[4],
                                 const float u[4], const float v[4],
                                 float H[9]);
static int aruco_reproj_err(const float R[9], const float t[3],
                             const float mx[4], const float my[4],
                             const float u[4], const float v[4],
                             float fx, float fy, float cx, float cy,
                             float* err_out);

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

// T18-O step 1: port cv2 _getBorderErrors.
// Counts BRIGHT cells in the border ring of the warped marker grid.
// markerSizeWithBorders = 6 (4 inner + 2 border), border = 1.
// Border cells: row 0, row 5, col 0, col 5 — total 4*6 - 4 = 20 cells.
// Returns the number of cells where DARK majority FAILS (i.e. cell
// looks bright/white = bit=1 in cv2's convention).  cv2 rejects if
// borderErrors > markerSize² * maxErroneousBitsInBorderRate = 16 * 0.35 = 5.6.
static int aruco_decode_border_errors_(const uint8_t* warped, int N,
                                         int cell_size, int marker_border,
                                         uint8_t t) {
    const int margin = (int)(0.13f * (float)cell_size + 0.5f);
    const int side = (4 + 2 * marker_border);    // 6 for our case
    int errors = 0;
    for (int cy = 0; cy < side; ++cy) {
        for (int cx = 0; cx < side; ++cx) {
            // Skip inner data cells (only check border ring).
            const int is_border = (cy < marker_border)
                                 || (cy >= side - marker_border)
                                 || (cx < marker_border)
                                 || (cx >= side - marker_border);
            if (!is_border) continue;
            const int yc = cy * cell_size;
            const int xc = cx * cell_size;
            int n_dark = 0, n_tot = 0;
            for (int dy = margin; dy < cell_size - margin; ++dy) {
                for (int dx = margin; dx < cell_size - margin; ++dx) {
                    if (warped[(yc + dy) * N + (xc + dx)] < t) n_dark++;
                    n_tot++;
                }
            }
            // BRIGHT (non-dark majority) = error in border ring.
            if (n_dark * 2 <= n_tot) errors++;
        }
    }
    return errors;
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
    // T18-O step 1: cv2 _getBorderErrors gate.  cv2's threshold:
    // borderErrors > markerSize² * maxErroneousBitsInBorderRate
    //              = 16 * 0.35 = 5.6  → reject if > 5.
    const int border_errors = aruco_decode_border_errors_(
        s_warp_buf, ARUCO_BITGRID_SIDE,
        ARUCO_BITGRID_CELL, ARUCO_BITGRID_BORDER, t);
    if (border_errors > 5) {
        if (out_hamming) *out_hamming = 17;
        return -1;
    }
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
// T18-M — Port of cv2 IPPE_SQUARE PnP solver
// (opencv/modules/calib3d/src/ippe.cpp, BSD-3-Clause, OpenCV 4.x).
//
// Why we need this: our DLT-homography-decomposition PnP (below) is
// numerically unstable under sub-pixel corner noise.  On real-flight
// frames where extracted corners are 1-2 px off the true position,
// our DLT gives tvec_z errors up to 24% and reproj 4-7 px (fails
// the 3-px gate).  IPPE_SQUARE uses an analytical 2-pose formulation
// based on the Jacobian of the homography at the marker centroid;
// it stays numerically stable for noisy inputs.
//
// Empirical s174 frame 133 measurement (same corners):
//   our DLT:        tvec_z=0.826m  reproj=3.98 px ✗
//   cv2 IPPE_SQUARE: tvec_z=1.089m reproj=0.30 px ✓
//
// This single change closes most of the 40% → 92% real-flight
// detection gap.
// =========================================================================

// Rotation matrix that rotates the input vector a onto +z axis.
// Output Ra is row-major 3x3.  Port of PoseSolver::rotateVec2ZAxis.
static void aruco_ippe_rotate_vec2zaxis(const float a[3], float Ra[9]) {
    const float nrm = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    const float ax = a[0] / nrm;
    const float ay = a[1] / nrm;
    const float az = a[2] / nrm;
    const float c = az;
    if (fabsf(1.0f + c) < 1e-7f) {
        Ra[0]=1; Ra[1]=0; Ra[2]= 0;
        Ra[3]=0; Ra[4]=1; Ra[5]= 0;
        Ra[6]=0; Ra[7]=0; Ra[8]=-1;
        return;
    }
    const float d = 1.0f / (1.0f + c);
    const float ax2 = ax * ax;
    const float ay2 = ay * ay;
    const float axay = ax * ay;
    Ra[0] = -ax2 * d + 1.0f;  Ra[1] = -axay * d;        Ra[2] = -ax;
    Ra[3] = -axay * d;        Ra[4] = -ay2 * d + 1.0f;  Ra[5] = -ay;
    Ra[6] = ax;               Ra[7] = ay;               Ra[8] = 1.0f - (ax2 + ay2) * d;
}

// Analytical homography from 4 (normalized) target points to a square
// of half-length halfL centered at origin.  Port of
// PoseSolver::homographyFromSquarePoints.  Returns H 3x3 row-major.
// Returns -1 on degenerate input.
static int aruco_ippe_h_from_square(const float xn[4], const float yn[4],
                                     float halfL, float H[9]) {
    const float p1x = -xn[0], p1y = -yn[0];
    const float p2x = -xn[1], p2y = -yn[1];
    const float p3x = -xn[2], p3y = -yn[2];
    const float p4x = -xn[3], p4y = -yn[3];
    const float det = halfL * (p1x*p2y - p2x*p1y - p1x*p4y + p2x*p3y
                              - p3x*p2y + p4x*p1y + p3x*p4y - p4x*p3y);
    if (fabsf(det) < 1e-9f) return -1;
    const float di = -1.0f / det;
    H[0] = di * (p1x*p3x*p2y - p2x*p3x*p1y - p1x*p4x*p2y + p2x*p4x*p1y
               - p1x*p3x*p4y + p1x*p4x*p3y + p2x*p3x*p4y - p2x*p4x*p3y);
    H[1] = di * (p1x*p2x*p3y - p1x*p3x*p2y - p1x*p2x*p4y + p2x*p4x*p1y
               + p1x*p3x*p4y - p3x*p4x*p1y - p2x*p4x*p3y + p3x*p4x*p2y);
    H[2] = di * halfL * (p1x*p2x*p3y - p2x*p3x*p1y - p1x*p2x*p4y
               + p1x*p4x*p2y - p1x*p4x*p3y + p3x*p4x*p1y + p2x*p3x*p4y
               - p3x*p4x*p2y);
    H[3] = di * (p1x*p2y*p3y - p2x*p1y*p3y - p1x*p2y*p4y + p2x*p1y*p4y
               - p3x*p1y*p4y + p4x*p1y*p3y + p3x*p2y*p4y - p4x*p2y*p3y);
    H[4] = di * (p2x*p1y*p3y - p3x*p1y*p2y - p1x*p2y*p4y + p4x*p1y*p2y
               + p1x*p3y*p4y - p4x*p1y*p3y - p2x*p3y*p4y + p3x*p2y*p4y);
    H[5] = di * halfL * (p1x*p2y*p3y - p3x*p1y*p2y - p2x*p1y*p4y
               + p4x*p1y*p2y - p1x*p3y*p4y + p3x*p1y*p4y + p2x*p3y*p4y
               - p4x*p2y*p3y);
    H[6] = -di * (p1x*p3y - p3x*p1y - p1x*p4y - p2x*p3y + p3x*p2y
               + p4x*p1y + p2x*p4y - p4x*p2y);
    H[7] = di * (p1x*p2y - p2x*p1y - p1x*p3y + p3x*p1y + p2x*p4y
               - p4x*p2y - p3x*p4y + p4x*p3y);
    H[8] = 1.0f;
    return 0;
}

// Compute the two rotation candidates from Jacobian of H at origin.
// Port of PoseSolver::computeRotations.  R1, R2 row-major 3x3.
static int aruco_ippe_compute_rotations(float j00, float j01,
                                          float j10, float j11,
                                          float p, float q,
                                          float R1[9], float R2[9]) {
    const float v[3] = { p, q, 1.0f };
    float RvT[9];
    aruco_ippe_rotate_vec2zaxis(v, RvT);
    // Rv = RvT transposed (cv2 transposes after rotateVec2ZAxis).
    const float rv00 = RvT[0], rv01 = RvT[3], rv02 = RvT[6];
    const float rv10 = RvT[1], rv11 = RvT[4], rv12 = RvT[7];
    const float rv20 = RvT[2], rv21 = RvT[5], rv22 = RvT[8];
    const float b00 = rv00 - p * rv20;
    const float b01 = rv01 - p * rv21;
    const float b10 = rv10 - q * rv20;
    const float b11 = rv11 - q * rv21;
    const float bdet = b00 * b11 - b01 * b10;
    if (fabsf(bdet) < 1e-9f) return -1;
    const float dtinv = 1.0f / bdet;
    const float binv00 =  dtinv * b11;
    const float binv01 = -dtinv * b01;
    const float binv10 = -dtinv * b10;
    const float binv11 =  dtinv * b00;
    const float a00 = binv00 * j00 + binv01 * j10;
    const float a01 = binv00 * j01 + binv01 * j11;
    const float a10 = binv10 * j00 + binv11 * j10;
    const float a11 = binv10 * j01 + binv11 * j11;
    // Largest singular value of A.
    const float ata00 = a00 * a00 + a01 * a01;
    const float ata01 = a00 * a10 + a01 * a11;
    const float ata11 = a10 * a10 + a11 * a11;
    const float disc = (ata00 - ata11) * (ata00 - ata11) + 4.0f * ata01 * ata01;
    const float g2 = 0.5f * (ata00 + ata11 + sqrtf(disc));
    if (g2 < 0.0f) return -1;
    const float gamma = sqrtf(g2);
    if (fabsf(gamma) < 1e-9f) return -1;
    const float rt00 = a00 / gamma, rt01 = a01 / gamma;
    const float rt10 = a10 / gamma, rt11 = a11 / gamma;
    const float b0_2 = 1.0f - rt00 * rt00 - rt10 * rt10;
    const float b1_2 = 1.0f - rt01 * rt01 - rt11 * rt11;
    if (b0_2 < 0.0f || b1_2 < 0.0f) return -1;
    const float b0 = sqrtf(b0_2);
    float b1   = sqrtf(b1_2);
    const float sp = -rt00 * rt01 - rt10 * rt11;
    if (sp < 0.0f) b1 = -b1;
    // r3 column = cross of (rt0, rt1, b)
    const float c00 = b1 * rt10 - b0 * rt11;
    const float c01 = b0 * rt01 - b1 * rt00;
    const float c02 = rt00 * rt11 - rt01 * rt10;
    R1[0] = rt00*rv00 + rt10*rv01 + b0*rv02;
    R1[1] = rt01*rv00 + rt11*rv01 + b1*rv02;
    R1[2] = c00*rv00 + c01*rv01 + c02*rv02;
    R1[3] = rt00*rv10 + rt10*rv11 + b0*rv12;
    R1[4] = rt01*rv10 + rt11*rv11 + b1*rv12;
    R1[5] = c00*rv10 + c01*rv11 + c02*rv12;
    R1[6] = rt00*rv20 + rt10*rv21 + b0*rv22;
    R1[7] = rt01*rv20 + rt11*rv21 + b1*rv22;
    R1[8] = c00*rv20 + c01*rv21 + c02*rv22;
    // R2 has b0, b1, c00, c01 negated (per cv2 source).
    R2[0] = rt00*rv00 + rt10*rv01 + (-b0)*rv02;
    R2[1] = rt01*rv00 + rt11*rv01 + (-b1)*rv02;
    R2[2] = -c00*rv00 + -c01*rv01 + c02*rv02;
    R2[3] = rt00*rv10 + rt10*rv11 + (-b0)*rv12;
    R2[4] = rt01*rv10 + rt11*rv11 + (-b1)*rv12;
    R2[5] = -c00*rv10 + -c01*rv11 + c02*rv12;
    R2[6] = rt00*rv20 + rt10*rv21 + (-b0)*rv22;
    R2[7] = rt01*rv20 + rt11*rv21 + (-b1)*rv22;
    R2[8] = -c00*rv20 + -c01*rv21 + c02*rv22;
    return 0;
}

// Compute translation t given R, 4 marker 2D pts, and 4 normalized
// image pts.  Port of PoseSolver::computeTranslation.  Solves
// A^T A t = A^T b via the closed-form 3x3 inverse for n=4.
static int aruco_ippe_compute_translation(const float mx[4], const float my[4],
                                            const float xn[4], const float yn[4],
                                            const float R[9],
                                            float t[3]) {
    const int n = 4;
    float ATA00 = (float)n, ATA02 = 0.0f, ATA11 = (float)n;
    float ATA12 = 0.0f, ATA20 = 0.0f, ATA21 = 0.0f, ATA22 = 0.0f;
    float ATb0 = 0.0f, ATb1 = 0.0f, ATb2 = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float rx = R[0]*mx[i] + R[1]*my[i];
        const float ry = R[3]*mx[i] + R[4]*my[i];
        const float rz = R[6]*mx[i] + R[7]*my[i];
        const float a2 = -xn[i];
        const float b2 = -yn[i];
        ATA02 += a2; ATA12 += b2;
        ATA20 += a2; ATA21 += b2;
        ATA22 += a2 * a2 + b2 * b2;
        const float bx = -a2 * rz - rx;
        const float by = -b2 * rz - ry;
        ATb0 += bx;
        ATb1 += by;
        ATb2 += a2 * bx + b2 * by;
    }
    const float detA = ATA00 * ATA11 * ATA22 - ATA00 * ATA12 * ATA21
                        - ATA02 * ATA11 * ATA20;
    if (fabsf(detA) < 1e-9f) return -1;
    const float dAi = 1.0f / detA;
    const float S00 = ATA11 * ATA22 - ATA12 * ATA21;
    const float S01 = ATA02 * ATA21;
    const float S02 = -ATA02 * ATA11;
    const float S10 = ATA12 * ATA20;
    const float S11 = ATA00 * ATA22 - ATA02 * ATA20;
    const float S12 = -ATA00 * ATA12;
    const float S20 = -ATA11 * ATA20;
    const float S21 = -ATA00 * ATA21;
    const float S22 = ATA00 * ATA11;
    t[0] = dAi * (S00 * ATb0 + S01 * ATb1 + S02 * ATb2);
    t[1] = dAi * (S10 * ATb0 + S11 * ATb1 + S12 * ATb2);
    t[2] = dAi * (S20 * ATb0 + S21 * ATb1 + S22 * ATb2);
    return 0;
}

// Top-level IPPE_SQUARE solver.  Input: 4 image-pixel corners (TL,
// TR, BR, BL in marker frame), intrinsics, marker side L.  Output:
// best (R, t) by reproj error.  Returns 0 on success, -1 on failure.
static int aruco_pnp_ippe_square(const float corners[8],
                                   float fx, float fy, float cx, float cy,
                                   float L,
                                   float tvec_out[3], float rvec_out[3],
                                   float* reproj_err_out) {
    // Normalize image points: xn = (u - cx) / fx, yn = (v - cy) / fy.
    const float xn[4] = {
        (corners[0] - cx) / fx,
        (corners[2] - cx) / fx,
        (corners[4] - cx) / fx,
        (corners[6] - cx) / fx,
    };
    const float yn[4] = {
        (corners[1] - cy) / fy,
        (corners[3] - cy) / fy,
        (corners[5] - cy) / fy,
        (corners[7] - cy) / fy,
    };
    const float halfL = 0.5f * L;
    // Marker corners 2D in marker frame (y-up).
    const float mx[4] = { -halfL, +halfL, +halfL, -halfL };
    const float my[4] = { +halfL, +halfL, -halfL, -halfL };
    // Analytical H from 4 normalized pixels + halfL.
    float H[9];
    if (aruco_ippe_h_from_square(xn, yn, halfL, H) != 0) return -1;
    // Jacobian of H at origin.
    const float j00 = H[0] - H[6] * H[2];
    const float j01 = H[1] - H[7] * H[2];
    const float j10 = H[3] - H[6] * H[5];
    const float j11 = H[4] - H[7] * H[5];
    const float v0 = H[2];
    const float v1 = H[5];
    // Compute two rotation candidates.
    float R1[9], R2[9];
    if (aruco_ippe_compute_rotations(j00, j01, j10, j11, v0, v1, R1, R2) != 0)
        return -1;
    // Compute translation for each.
    float t1[3], t2[3];
    if (aruco_ippe_compute_translation(mx, my, xn, yn, R1, t1) != 0) return -1;
    if (aruco_ippe_compute_translation(mx, my, xn, yn, R2, t2) != 0) return -1;
    // Pick the candidate with smaller reproj error.
    const float u[4] = { corners[0], corners[2], corners[4], corners[6] };
    const float v[4] = { corners[1], corners[3], corners[5], corners[7] };
    float err1, err2;
    int r1_ok = (aruco_reproj_err(R1, t1, mx, my, u, v, fx, fy, cx, cy, &err1) == 0);
    int r2_ok = (aruco_reproj_err(R2, t2, mx, my, u, v, fx, fy, cx, cy, &err2) == 0);
    if (!r1_ok && !r2_ok) return -1;
    const float* Rbest;
    const float* tbest;
    float ebest;
    if (r1_ok && (!r2_ok || err1 <= err2)) { Rbest = R1; tbest = t1; ebest = err1; }
    else                                    { Rbest = R2; tbest = t2; ebest = err2; }
    tvec_out[0] = tbest[0];
    tvec_out[1] = tbest[1];
    tvec_out[2] = tbest[2];
    sentai_aruco_R_to_rvec(Rbest, rvec_out);
    if (reproj_err_out) *reproj_err_out = ebest;
    return 0;
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
    // T18-M: route through IPPE_SQUARE (numerically stable under
    // sub-pixel corner noise; ablation showed +10 pp vs DLT-only).
    if (aruco_pnp_ippe_square(corners, fx, fy, cx, cy, marker_size_m,
                                tvec_out, rvec_out, reproj_err_out) == 0) {
        return 0;
    }
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
// s_test_gray in OCRAM: this is where the synthesized / PGM-loaded
// grayscale frame lives during the WhyCon and ArUco benches.  Reading
// it lands in the threshold's column-sum scan — the hot Phase A path.
// Placing it in OCRAM gives 3-cyc access vs ~50-cyc SDRAM cache miss
// (per OP-S10-W16 ablation), AND mirrors where PXP DMA already lands
// the production camera Y8.
static uint8_t s_test_gray[ARUCO_BUF_SZ] ARUCO_OCRAM_ATTR;

// DWT cycle counter for Cortex-M7 (no-op on POSIX SIM — returns 0).
static inline uint32_t aruco_dwt_cyc(void) {
#if defined(__ARM_ARCH) && (__ARM_ARCH >= 7)
    return *((volatile uint32_t*)0xE0001004u);  // DWT->CYCCNT
#else
    return 0u;
#endif
}

// Per-bench cycle counts, written by sentai_aruco_threshold_bench(),
// read by the MP binding.
extern "C" {
uint32_t s_aruco_thresh_old_cyc = 0;
uint32_t s_aruco_thresh_new_cyc = 0;
}

// Runtime byte-equivalence check for the SIMD-friendly adaptive threshold
// (OP-S10-W14-T18-T).  Builds a deterministic synth gray frame
// (gradient + LFSR noise + central dark square — same shape as
// aruco_bench.cc's init_pattern), runs both the scalar reference and the
// optimized variant, and returns the count of differing s_binary bytes.
// 0 == math-identical (expected).  Also leaves cycle counts for each
// variant in s_aruco_thresh_*_cyc — read via sentai.aruco._thresh_cycles().
// Called from MP via sentai.aruco._verify_threshold(block).
extern "C" int sentai_aruco_adaptive_threshold_verify(int block) {
    const int W = 320, H = 240;
    if (block < 3 || block > 511) return -1;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint32_t lfsr = (uint32_t)(y * W + x) * 2654435761u;
            uint8_t noise = (lfsr >> 16) & 0x1F;
            int v = 180 + (x * 40) / W + (int)noise - 8;
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            s_test_gray[y * W + x] = (uint8_t)v;
        }
    }
    const int cx = W / 2, cy = H / 2;
    for (int y = cy - 40; y < cy + 40; ++y) {
        for (int x = cx - 40; x < cx + 40; ++x) {
            s_test_gray[y * W + x] = 30;
        }
    }
    // Run scalar reference → write into s_labels (also 320*240 byte buffer).
    const uint32_t t0 = aruco_dwt_cyc();
    aruco_adaptive_threshold_scalar_ref(s_test_gray, W, H, block, s_labels);
    const uint32_t t1 = aruco_dwt_cyc();
    // Run optimized variant → writes into s_binary.
    aruco_adaptive_threshold(s_test_gray, W, H, block);
    const uint32_t t2 = aruco_dwt_cyc();
    s_aruco_thresh_old_cyc = t1 - t0;
    s_aruco_thresh_new_cyc = t2 - t1;
    int mismatches = 0;
    for (int i = 0; i < W * H; ++i) {
        if (s_labels[i] != s_binary[i]) mismatches++;
    }
    return mismatches;
}

extern "C" uint32_t sentai_aruco_thresh_old_cyc(void) {
    return s_aruco_thresh_old_cyc;
}
extern "C" uint32_t sentai_aruco_thresh_new_cyc(void) {
    return s_aruco_thresh_new_cyc;
}

// OP-S10-W16-T3.8 — rolling-integral threshold bench + verify.
// Returns: byte-mismatch count between scalar reference and rolling
// kernel (0 means byte-identical).  Cycle count for the rolling kernel
// is left in s_aruco_thresh_rolling_cyc, retrievable via
// sentai_aruco_thresh_rolling_cyc().
static uint32_t s_aruco_thresh_rolling_cyc = 0;

extern "C" int sentai_aruco_thresh_rolling_verify(int block) {
    const int W = 320, H = 240;
    if (block < 3 || block > 511) return -1;
    // Build synth frame (same shape as adaptive_threshold_verify).
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint32_t lfsr = (uint32_t)(y * W + x) * 2654435761u;
            uint8_t noise = (lfsr >> 16) & 0x1F;
            int v = 180 + (x * 40) / W + (int)noise - 8;
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            s_test_gray[y * W + x] = (uint8_t)v;
        }
    }
    const int cx = W / 2, cy = H / 2;
    for (int y = cy - 40; y < cy + 40; ++y) {
        for (int x = cx - 40; x < cx + 40; ++x) {
            s_test_gray[y * W + x] = 30;
        }
    }
    // Scalar reference into s_labels (reused as scratch).
    aruco_adaptive_threshold_scalar_ref(s_test_gray, W, H, block, s_labels);
    // Rolling kernel into s_binary, timed.
    const uint32_t t0 = aruco_dwt_cyc();
    aruco_adaptive_threshold_rolling(s_test_gray, W, H, block, s_binary);
    const uint32_t t1 = aruco_dwt_cyc();
    s_aruco_thresh_rolling_cyc = t1 - t0;
    int mismatches = 0;
    for (int i = 0; i < W * H; ++i) {
        if (s_labels[i] != s_binary[i]) mismatches++;
    }
    return mismatches;
}

extern "C" uint32_t sentai_aruco_thresh_rolling_cyc(void) {
    return s_aruco_thresh_rolling_cyc;
}

// OP-S10-W16 ablation 2026-05-19: re-run scalar threshold on the
// synth frame with M7's D-cache DISABLED, return DWT cycles.  This
// isolates the cache-thrash penalty contribution to M7's per-pixel
// inefficiency vs M4 (which has no D-cache by design).  Expected
// result: M7 cycle count drops from ~12.8M to closer to M4's ~3.9M
// at 400 MHz × 2 → ~7-8M cycles, validating that the cache miss
// penalty is the dominant cost on 309 KB integral-image stride
// patterns.
//
// Only compiles on ARM (SCB_* MMIO is Cortex-M specific).
#ifdef __arm__
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/cm7/fsl_cache.h"
extern "C" uint32_t sentai_aruco_thresh_nocache(int block) {
    if (block < 3 || block > 511) return 0;
    const int W = 320, H = 240;
    // Build synth frame (same shape as verify).
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint32_t lfsr = (uint32_t)(y * W + x) * 2654435761u;
            uint8_t noise = (lfsr >> 16) & 0x1F;
            int v = 180 + (x * 40) / W + (int)noise - 8;
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            s_test_gray[y * W + x] = (uint8_t)v;
        }
    }
    const int cx = W / 2, cy = H / 2;
    for (int y = cy - 40; y < cy + 40; ++y) {
        for (int x = cx - 40; x < cx + 40; ++x) {
            s_test_gray[y * W + x] = 30;
        }
    }
    SCB_CleanInvalidateDCache();   // flush dirty + invalidate all lines
    SCB_DisableDCache();           // all subsequent loads bypass cache
    const uint32_t t0 = aruco_dwt_cyc();
    aruco_adaptive_threshold_scalar_ref(s_test_gray, W, H, block, s_binary);
    const uint32_t t1 = aruco_dwt_cyc();
    SCB_EnableDCache();            // restore for normal operation
    return t1 - t0;
}
#else
extern "C" uint32_t sentai_aruco_thresh_nocache(int) { return 0; }
#endif

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
// PGM parse helper — given a memory buffer containing a full PGM file
// (P5 raw), validate the 320x240 maxval=255 header and copy the pixel
// payload into s_test_gray.  Returns 0 on success, -2 on bad header,
// -3 on bad pixel-data size.
static int aruco_parse_pgm_buffer_(const uint8_t* buf, size_t len) {
    if (!buf || len < 16) return -2;
    // Find header: "P5\n<W> <H>\n<maxval>\n<pixels>"
    const char* p = (const char*)buf;
    const char* end = p + len;
    if (p[0] != 'P' || p[1] != '5') return -2;
    p += 2;
    int w = 0, h = 0, maxval = 0;
    // Skip whitespace + parse w
    while (p < end && (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r')) ++p;
    while (p < end && *p >= '0' && *p <= '9') { w = w * 10 + (*p - '0'); ++p; }
    while (p < end && (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r')) ++p;
    while (p < end && *p >= '0' && *p <= '9') { h = h * 10 + (*p - '0'); ++p; }
    while (p < end && (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r')) ++p;
    while (p < end && *p >= '0' && *p <= '9') { maxval = maxval * 10 + (*p - '0'); ++p; }
    // Exactly ONE whitespace separates maxval from pixel data per PGM P5.
    if (p < end) ++p;
    if (w != 320 || h != 240 || maxval != 255) return -2;
    const size_t expected = (size_t)w * (size_t)h;
    if ((size_t)(end - p) < expected) return -3;
    if (expected > sizeof(s_test_gray)) return -3;
    memcpy(s_test_gray, p, expected);
    return 0;
}

// FxUser typed C API for FAT user-partition access (preferred per
// agent.md §12).  Pure C linkage block in the header.
#include "libs/base/fx_user_fs.h"

extern "C" int sentai_aruco_detect_pgm_file(const char* path) {
    // Path 1: try newlib fopen (works under SIM Linux + on LittleFS-
    // backed system partition).
    FILE* fp = fopen(path, "rb");
    if (fp) {
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
        fgetc(fp);
        const size_t expected = (size_t)w * (size_t)h;
        if (expected > sizeof(s_test_gray)) { fclose(fp); return -2; }
        if (fread(s_test_gray, 1, expected, fp) != expected) {
            fclose(fp); return -2;
        }
        fclose(fp);
    } else {
        // Path 2: HW board fallback — read via FxUser FAT API.  PGM
        // files dropped via USB MSC mount live on the FAT volume and
        // are NOT visible to libnewlib's fopen.
        //
        // Staging: reuse s_integral (1.2 MB int32_t buffer, idle pre-
        // threshold) as a byte-buffer scratch.  A 320×240 P5 PGM with
        // its ~15 B ASCII header is ~76.8 KB; s_labels at exactly
        // ARUCO_BUF_SZ = 76800 B was 15 B short for the full file
        // and rejected legit PGM uploads with -SENTAI_ARUCO_ERR_OVERFLOW
        // (W17-T8 finding via s180 bench).  s_integral has room.
        ssize_t sz = FxUserSize(path);
        if (sz < 0) return -1;
        uint8_t* stage = (uint8_t*)s_integral;
        const size_t max_buf = sizeof(s_integral);
        if ((size_t)sz > max_buf) return -3;
        size_t got = FxUserReadFile(path, stage, max_buf);
        if (got == 0) return -1;
        int rc = aruco_parse_pgm_buffer_(stage, got);
        if (rc != 0) return rc;
    }
    // Both paths produce s_test_gray with the canonical 320x240 layout.
    const int w = 320, h = 240;
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

// OP-S10-W16-T3.8 — runtime toggle for rolling-integral threshold path.
// 0 = use production aruco_adaptive_threshold (SIMD + full 309 KB integral image).
// 1 = use aruco_adaptive_threshold_rolling (scalar, 2.6 KB OCRAM scratch).
static volatile int s_aruco_use_rolling = 0;
// Last call's full detect() cycle count (DWT @ M7 clock).  Updated whether
// or not any markers were found, and whether or not the toggle is on.
static volatile uint32_t s_aruco_detect_cyc_last = 0;

extern "C" void sentai_aruco_set_use_rolling(int on) {
    s_aruco_use_rolling = on ? 1 : 0;
}
extern "C" int sentai_aruco_get_use_rolling(void) {
    return s_aruco_use_rolling;
}
extern "C" uint32_t sentai_aruco_detect_cyc_last(void) {
    return s_aruco_detect_cyc_last;
}

// OP-S10-W17-T6: Per-stage cycle counters — accumulated across the
// scale-loop / component-loop nesting inside sentai_aruco_detect().
// Reset to 0 at the top of each detect() call; queryable through
// `sentai.aruco._stage_cyc()`.  These give the breakdown needed for
// the mere-cu-mere comparison with WhyCon `_stage_cyc5()`:
//
//   t_thresh : Bradley adaptive threshold (all scales summed)
//   t_flood  : 8-conn flood-fill labeling (all scales summed)
//   t_quad   : per-component bbox/aspect/fill filter + extract_quad +
//              convex / minDistanceToBorder / minCornerDistance / CW
//              winding (the "geometric gates" — analogue of WhyCon
//              W1+W2 size/circularity/axes).
//   t_decode : aruco_decode_marker (perspective warp + Otsu + bit
//              extract + dictionary lookup + hamming).  This is the
//              ID-emitting part of ArUco that WhyCon-lite skips.
//   t_pnp    : aruco_pnp_from_corners (IPPE_SQUARE) + reproj gate.
static volatile uint32_t s_aruco_t_thresh = 0;
static volatile uint32_t s_aruco_t_flood  = 0;
static volatile uint32_t s_aruco_t_quad   = 0;
static volatile uint32_t s_aruco_t_decode = 0;
static volatile uint32_t s_aruco_t_pnp    = 0;

extern "C" void sentai_aruco_stage_cyc(uint32_t* t_thresh, uint32_t* t_flood,
                                         uint32_t* t_quad, uint32_t* t_decode,
                                         uint32_t* t_pnp) {
    if (t_thresh) *t_thresh = s_aruco_t_thresh;
    if (t_flood)  *t_flood  = s_aruco_t_flood;
    if (t_quad)   *t_quad   = s_aruco_t_quad;
    if (t_decode) *t_decode = s_aruco_t_decode;
    if (t_pnp)    *t_pnp    = s_aruco_t_pnp;
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

    // T6: reset per-stage counters at the start of every detect().
    s_aruco_t_thresh = 0;
    s_aruco_t_flood  = 0;
    s_aruco_t_quad   = 0;
    s_aruco_t_decode = 0;
    s_aruco_t_pnp    = 0;

    const uint32_t detect_t0 = aruco_dwt_cyc();

    // T18-G: cv2.aruco-style multi-scale adaptive threshold.  cv2 default
    // is (winSizeMin=3, winSizeMax=23, winSizeStep=10) → blocks 3,13,23.
    // We use a coarser set tuned for our 320x240 scene (markers at
    // ~25 px to ~80 px).  The full pipeline runs at each scale; markers
    // detected at multiple scales are deduplicated by marker_id (best
    // reproj wins).
    // T18-Q ablation result: single-scale block=201 matches full
    // [3, 13, 23, 51, 101, 201] within 1 pp (89 % vs 90 %).  The
    // multi-scale loop costs 6× threshold + flood-fill per frame for
    // marginal gain — single-scale is the right ARM trade.
    static const int SCALE_BLOCKS[] = { 201 };
    constexpr int N_SCALES = (int)(sizeof(SCALE_BLOCKS) / sizeof(SCALE_BLOCKS[0]));

    // Per-id best candidate (one slot per known marker, ids 0..N-1).
    sentai_aruco_marker_t best[SENTAI_ARUCO_MAX_MARKERS];
    bool best_set[SENTAI_ARUCO_MAX_MARKERS];
    for (int i = 0; i < SENTAI_ARUCO_MAX_MARKERS; ++i) best_set[i] = false;

    for (int si = 0; si < N_SCALES; ++si) {
        const uint32_t t_thr_0 = aruco_dwt_cyc();
        if (s_aruco_use_rolling) {
            aruco_adaptive_threshold_rolling(gray, w, h, SCALE_BLOCKS[si], s_binary);
        } else {
            aruco_adaptive_threshold(gray, w, h, SCALE_BLOCKS[si]);
        }
        s_aruco_t_thresh += aruco_dwt_cyc() - t_thr_0;

        const uint32_t t_fl_0 = aruco_dwt_cyc();
        const int n_comp = aruco_label_components(w, h);
        s_aruco_t_flood += aruco_dwt_cyc() - t_fl_0;

        // T18-O step 6: cv2 minMarkerPerimeterRate=0.03,
        // maxMarkerPerimeterRate=4.0 (proxies via bbox diagonal which
        // bounds the perimeter).
        const int max_dim = (w > h) ? w : h;
        const int min_perim_px = (int)(0.03f * (float)max_dim);  // ~10 for 320x240
        const int max_perim_px = (int)(4.0f  * (float)max_dim);  // ~1280
        const int min_bbox_diag_sq = (min_perim_px * min_perim_px) / 16;  // perim≈4*side
        const int max_bbox_diag_sq = (max_perim_px * max_perim_px) / 4;

        for (int ci = 0; ci < n_comp; ++ci) {
            // Early-exit: all known ids already covered.  Skip remaining
            // components (and break out of scale loop below).
            if (best_set[0] && best_set[1] &&
                best_set[2] && best_set[3]) goto all_ids_found;
            const uint32_t t_q_0 = aruco_dwt_cyc();
            const aruco_comp_t* c = &s_components[ci];
            if (c->touches_border) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }
            const int bbox_w = c->x1 - c->x0 + 1;
            const int bbox_h = c->y1 - c->y0 + 1;
            const int diag_sq = bbox_w * bbox_w + bbox_h * bbox_h;
            if (diag_sq < min_bbox_diag_sq) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }
            if (diag_sq > max_bbox_diag_sq) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }
            // Aspect gate (markers are roughly square; reject wide-rectangle noise).
            const float aspect = (float)bbox_w / (float)bbox_h;
            if (aspect < 0.33f || aspect > 3.0f) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }
            const float area = (float)c->n_pix;
            const float fill_ratio = area / (float)(bbox_w * bbox_h);
            if (fill_ratio < 0.30f) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }

            const uint8_t lab = (uint8_t)(ci + 1);
            float corners[8];
            if (aruco_extract_quad(lab, w, h, c, corners) != 0) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }

            // T18-O step 2: cv2 isContourConvex check.
            if (!aruco_is_quad_convex_(corners)) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }

            // T18-O step 3: cv2 minDistanceToBorder (3 px default).
            {
                int tooNear = 0;
                for (int k = 0; k < 4; ++k) {
                    const float u = corners[k*2 + 0];
                    const float v = corners[k*2 + 1];
                    if (u < 3.0f || v < 3.0f ||
                        u > (float)(w - 1 - 3) || v > (float)(h - 1 - 3)) {
                        tooNear = 1; break;
                    }
                }
                if (tooNear) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }
            }

            // T18-O step 4: cv2 minCornerDistance (perim * 0.05 default).
            // Compute squared perimeter and squared min edge length.
            {
                float min_edge_sq = 1e18f;
                float total_edge = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    const float dx = corners[((k+1)%4)*2 + 0] - corners[k*2 + 0];
                    const float dy = corners[((k+1)%4)*2 + 1] - corners[k*2 + 1];
                    const float e_sq = dx*dx + dy*dy;
                    if (e_sq < min_edge_sq) min_edge_sq = e_sq;
                    total_edge += sqrtf(e_sq);
                }
                const float min_thresh = total_edge * 0.05f;
                if (min_edge_sq < min_thresh * min_thresh) { s_aruco_t_quad += aruco_dwt_cyc() - t_q_0; continue; }
            }

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
            s_aruco_t_quad += aruco_dwt_cyc() - t_q_0;

            // T18-Q ablation (2026-05-18): Förstner subpix refinement
            // regressed detection on real flight frames (322 → 320 @ 4/4)
            // when combined with IPPE_SQUARE.  Removed 2026-05-19 —
            // integer-pixel corners from aruco_extract_quad + IPPE_SQUARE
            // PnP handle 1-2 px noise well enough that the safety-revert
            // path (win_half=2) sometimes drifted corners onto neighbour
            // gradients.  See diary/2026-05-19.md for the deletion log.

            const uint32_t t_dec_0 = aruco_dwt_cyc();
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
            s_aruco_t_decode += aruco_dwt_cyc() - t_dec_0;
            if (mid < 0) {
                s_stats.rejected_dict_total++;
                continue;
            }
            aruco_realign_corners(corners, (4 - rotation) % 4);

            const uint32_t t_pnp_0 = aruco_dwt_cyc();
            float tvec[3], rvec[3], reproj;
            const int pnp_rc = aruco_pnp_from_corners(corners, s_fx, s_fy, s_cx, s_cy,
                                                        s_marker_size_m,
                                                        tvec, rvec, &reproj);
            s_aruco_t_pnp += aruco_dwt_cyc() - t_pnp_0;
            if (pnp_rc != 0) {
                s_stats.rejected_reproj_total++;
                continue;
            }
            if (reproj > ARUCO_REPROJ_GATE_PX) {
                s_stats.rejected_reproj_total++;
                continue;
            }

            if (mid < 0 || mid >= SENTAI_ARUCO_MAX_MARKERS) continue;
            // Early-exit speedup: if we have all expected IDs already
            // and current candidate is no better, skip the marker
            // copy.  Also break out of component loop when we have
            // SENTAI_ARUCO_DICT_N_KNOWN ids covered (saves processing
            // the remaining noise candidates in this scale).
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
all_ids_found: ;

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
    s_aruco_detect_cyc_last = aruco_dwt_cyc() - detect_t0;
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

// =========================================================================
// OP-S10-W17-T1 — WhyCon-lite (timing prototype).
//
// Goal of THIS file's WhyCon block: measure end-to-end latency on M7
// for the circular-marker pipeline (Krajník/Nitsche 2013 family).
// It re-uses the existing ArUco scratch buffers and pipeline stages
// A (threshold) + B (8-conn connected components) and adds:
//   - Stage W1: filter components by area, bbox aspect, fill ratio
//                 (rejects rectangles + thin shapes — circles only).
//   - Stage W2: compute centroid sub-pixel + 2nd-order moments for
//                 axis ratio (eccentricity proxy) on filtered candidates.
//   - Stage W3 (optional): concentric-circle validation — checks for
//                 an inner WHITE blob centered on each dark detection.
//
// Concentric check is the difference between "WhyCon-lite" (this file,
// timing-focused) and full WhyCon — the original always validates the
// inner ring.  Disabled by default so the timing reflects the minimum
// detection pipeline.  Toggle via sentai_whycon_set_concentric_check(1).
//
// NOT integrated with safety/mission yet — pure perf instrumentation.
// =========================================================================

// OP-S10-W17-T5 / W19-T2 — pose-emitting WhyCon: Phase W3 concentric
// inner-disc validation + closed-form PnP-z from semi-major axis +
// physical diameter.  ABI extension: previous lite-only fields (cx,
// cy, axis_a, axis_b, angle, comp_id) preserved at the head so old
// binding's 24-byte view still reads correct values.  New fields
// trail; binding's `sentai_whycon_marker_pub_t` must mirror this
// layout (verified by static_assert in binding TU after rebuild).
typedef struct {
    float cx;      // centroid x sub-pixel
    float cy;      // centroid y sub-pixel
    float axis_a;  // semi-major axis length (pixels)
    float axis_b;  // semi-minor axis length (pixels)
    float angle;   // orientation of major axis (radians, [-π/2, π/2])
    int   comp_id; // index in s_components
    // ---- W17-T5 / W19-T2 trailing fields ----
    float tvec_cam[3];     // (x, y, z) in camera frame, metres.  All
                           // 0 if !pose_valid (intrinsics or diameter
                           // unset, or W3 disabled and caller still
                           // queried PnP).
    float rvec_cam[3];     // partial axis-angle: magnitude = tilt φ,
                           // direction = rotation axis in image plane
                           // perpendicular to projected major axis.
                           // Yaw around marker normal is indeterminate
                           // from a single circular marker (W17 §6.2).
                           // Multi-marker constellation (W19-T3)
                           // resolves yaw.
    float reproj_err_px;   // self-consistent by construction (closed-
                           // form); kept as a field for API parity
                           // with ArUco's marker struct.
    uint8_t pose_valid;    // 1 iff intrinsics + diameter populated
                           // AND detection accepted (including W3 if
                           // enabled).
    uint8_t _pad[3];
} sentai_whycon_marker_t;

#define SENTAI_WHYCON_MAX_DETS    16

static sentai_whycon_marker_t s_whycon_markers[SENTAI_WHYCON_MAX_DETS];
static int s_whycon_n_markers = 0;
static int s_whycon_concentric_check = 0;  // 0 = WhyCon-lite, 1 = full WhyCon

// Filter knobs — tuned for 320×240 frames, marker diameter 12-60 px.
// W17-T8: min_area raised from 25 to 100 to reject centre-dot blobs
// of Krajník-pattern markers.  The dark centre dot of a real Krajník
// marker is a separate blob from the outer annulus (the white inner
// disc disconnects them); both pass the W1 fill gate, but the centre
// dot's tiny area (~40 px² for an 18 px outer-radius marker) means
// Phase W3 can't sample the surrounding annulus at meaningful radii.
// Raising min_area kicks the centre-dot blob out at W1, leaving only
// the annulus for W3 to validate.  Filter is still inclusive enough
// for solid-disc lite markers (40 px² blobs allowed by the original
// 25 → too small to be useful pose targets anyway).
// W1 min_fill stays at 0.55: annulus fill = 0.50 for a 0.6R-to-R
// donut, just below this gate — so we ALSO need to either lower
// fill OR accept annulus by topology.  Decision: lower min_fill to
// 0.40 to include annulus markers, then rely on min_area + W3 to
// reject false positives.
static int   s_whycon_min_area     = 100;    // pixels (≥ 11 px diameter blob)
static int   s_whycon_max_area     = 4000;   // pixels (~70 px diameter)
static float s_whycon_min_fill     = 0.40f;  // annulus ≈ 0.50, disc ≈ 0.79
static float s_whycon_max_bbox_ar  = 1.5f;   // bbox w/h ratio: circle ≈ 1.0
static float s_whycon_max_axis_ratio = 2.0f; // a/b axis ratio: circle ≈ 1.0

// W17-T5: Phase W3 concentric-validation knobs.  WhyCon markers
// (Krajník/Nitsche style) are a dark outer annulus surrounding a
// white inner disc with a small dark sub-pixel localiser dot.  The
// flood-fill upstream picked up the outer DARK annulus as one blob
// (centre dot connects to ring via the outer dark region, but the
// inner white disc bisects the blob — except that 8-connected fill
// over-bridges thin separations.  See `s180` ablation log for the
// reasoning behind the radial sample set).
//
// Pattern-aware sampling (cheap):
//   - centre 3×3 mean: expect DARK (sub-pixel localiser dot)
//   - 0.55·a radial samples (8 angles): expect WHITE (inner disc)
//   - 0.95·a radial samples (8 angles): expect DARK (outer ring)
//
// All radii are pre-scaled by axis_a; ellipse-tilted markers project
// each radius onto a · (axis_b/axis_a) along the minor axis direction,
// but for tilts < 30° (cos(30°) ≈ 0.87) the major-axis radius alone
// is within the band width on both axes — no per-sample axis_b
// scaling needed.
#define WHYCON_W3_INNER_RADIUS_FRAC   0.55f
#define WHYCON_W3_OUTER_RADIUS_FRAC   0.95f
#define WHYCON_W3_N_RADIAL_SAMPLES    8
#define WHYCON_W3_GRAY_DARK_MAX       100   // pixel < this counts as DARK
#define WHYCON_W3_GRAY_LIGHT_MIN      127   // pixel >= this counts as WHITE
#define WHYCON_W3_INNER_WHITE_HITS_MIN  6   // 6/8 inner samples must be white
#define WHYCON_W3_OUTER_DARK_HITS_MIN   6   // 6/8 outer samples must be dark

// W19-T2: WhyCon physical diameter (metres).  Required for closed-form
// PnP-z `z = fx * d / (2 * axis_a)`.  Intrinsics are shared with ArUco
// (s_fx / s_fy / s_cx / s_cy) — the camera is the same.
static float s_whycon_diameter_m = 0.0f;

// Per-stage cycle counters for W3 + PnP — defined here so the
// filter+moments writer (further down) sees them; the rest of the
// counters (t_a / t_b / t_w) live below near whycon_detect_inplace_
// for back-compat with the original 3-tuple ordering.
static volatile uint32_t s_whycon_t_w3  = 0;
static volatile uint32_t s_whycon_t_pnp = 0;

extern "C" void sentai_whycon_set_concentric_check(int on) {
    s_whycon_concentric_check = on ? 1 : 0;
}

extern "C" void sentai_whycon_set_diameter(float meters) {
    s_whycon_diameter_m = (meters > 0.0f) ? meters : 0.0f;
}

extern "C" float sentai_whycon_get_diameter(void) {
    return s_whycon_diameter_m;
}

// W17-T5 / T8 Phase W3 — concentric inner-disc validation.  Returns
// 1 if the candidate centred at (cx, cy) with outer radius `bbox_R`
// passes the pattern check, 0 otherwise.  See WHYCON_W3_* constants
// above for the sample geometry.  Cheap: 9 + 2·N_RADIAL = 25 pixel
// lookups per candidate, with N candidates after Phase W1+W2
// filtering (typically ≤ 8 on a clean frame).  No floating-point
// divides in the hot loop.
//
// Why bbox_R and not axis_a (eigenvalue):
//   - For a SOLID dark disc: axis_a ≈ R (the disc radius)
//   - For a DARK ANNULUS (0.6R..R): axis_a ≈ 1.17·R (eigenvalues of
//     the annular mass distribution)
//   - For a small CENTRE DOT (≤ 0.2R): axis_a ≈ 0.2·R
// Anchoring on axis_a would put inner/outer ring samples at vastly
// different physical fractions of marker radius depending on which
// blob the upstream stage picked up.  bbox_R = max(bbox_w, bbox_h)/2
// is invariant: it's always the PHYSICAL outer radius of the blob
// (the bbox encloses the whole marker for both annulus and disc).
// Annular fill_ratio = 0.50, disc fill_ratio = 0.78 — W1's lowered
// 0.40 gate admits both.  Centre-dot blobs are rejected upstream by
// min_area = 100.
static int whycon_w3_check_(const uint8_t* gray, int W, int H,
                              float cx, float cy, float bbox_R) {
    const int icx = (int)(cx + 0.5f);
    const int icy = (int)(cy + 0.5f);
    if (icx < 1 || icx >= W - 1 || icy < 1 || icy >= H - 1) return 0;

    // Centre 3×3 mean.  For a Krajník marker WITH a centre localiser
    // dot, the centre reads DARK; for plain annulus markers (no
    // localiser dot), the centre reads WHITE (the inner light disc).
    // We accept EITHER — the inner-ring + outer-ring checks below
    // are the load-bearing pattern test.  Centre 3×3 used only to
    // assert there IS a clean pattern (centre + ring agree).
    int csum = 0;
    csum += gray[(icx - 1) + (icy - 1) * W];
    csum += gray[(icx    ) + (icy - 1) * W];
    csum += gray[(icx + 1) + (icy - 1) * W];
    csum += gray[(icx - 1) + (icy    ) * W];
    csum += gray[(icx    ) + (icy    ) * W];
    csum += gray[(icx + 1) + (icy    ) * W];
    csum += gray[(icx - 1) + (icy + 1) * W];
    csum += gray[(icx    ) + (icy + 1) * W];
    csum += gray[(icx + 1) + (icy + 1) * W];
    const int cmean = csum / 9;
    /* centre check (informational): either dark or light is OK. */

    // Inner-disc ring samples — must be WHITE.  For an annulus marker
    // of outer radius R, the inner disc spans 0.20R < r < 0.60R; we
    // sample at 0.55·R which is inside the white region.
    const float r_inner = bbox_R * WHYCON_W3_INNER_RADIUS_FRAC;
    int white_hits = 0;
    for (int s = 0; s < WHYCON_W3_N_RADIAL_SAMPLES; ++s) {
        const float ang = (float)s * (6.2831853f /
                                       (float)WHYCON_W3_N_RADIAL_SAMPLES);
        const int sx = (int)(cx + r_inner * __builtin_cosf(ang) + 0.5f);
        const int sy = (int)(cy + r_inner * __builtin_sinf(ang) + 0.5f);
        if (sx < 0 || sx >= W || sy < 0 || sy >= H) continue;
        if (gray[sx + sy * W] >= WHYCON_W3_GRAY_LIGHT_MIN) white_hits++;
    }
    if (white_hits < WHYCON_W3_INNER_WHITE_HITS_MIN) return 0;

    // Outer-ring samples — must be DARK.  For an annulus marker, the
    // dark annulus spans 0.60R < r < R; we sample at 0.95·R which is
    // inside the dark region.  For a solid disc this also reads DARK
    // (the entire disc is dark).  Rejects false-positive blobs whose
    // outer ring isn't dark (e.g., partial occlusions, isolated dots).
    const float r_outer = bbox_R * WHYCON_W3_OUTER_RADIUS_FRAC;
    int dark_hits = 0;
    for (int s = 0; s < WHYCON_W3_N_RADIAL_SAMPLES; ++s) {
        const float ang = (float)s * (6.2831853f /
                                       (float)WHYCON_W3_N_RADIAL_SAMPLES);
        const int sx = (int)(cx + r_outer * __builtin_cosf(ang) + 0.5f);
        const int sy = (int)(cy + r_outer * __builtin_sinf(ang) + 0.5f);
        if (sx < 0 || sx >= W || sy < 0 || sy >= H) continue;
        if (gray[sx + sy * W] <= WHYCON_W3_GRAY_DARK_MAX) dark_hits++;
    }
    if (dark_hits < WHYCON_W3_OUTER_DARK_HITS_MIN) return 0;

    /* Centre 3×3 is informational; sink it into an unused-warning-
     * suppression no-op so the compile doesn't whine. */
    (void)cmean;
    return 1;
}

// W19-T2 Closed-form PnP for circular markers.  Given semi-major axis
// `a_px` and physical diameter `d_m` and pinhole intrinsics (fx, fy,
// cx_intr, cy_intr), recover the 3D pose:
//
//   z       = fx · d / (2 · a_px)
//   tvec.x  = (cx_px - cx_intr) · z / fx
//   tvec.y  = (cy_px - cy_intr) · z / fy
//   tilt φ  = acos(min(1, b/a))
//   rvec    = φ · (axis perpendicular to projected major axis in
//                  the image plane)
//
// Yaw around the marker normal is NOT recoverable from a single
// circular marker (W17 §6.2 — multi-marker constellation in W19-T3
// resolves it).  Writes results into the marker in-place.  No-op if
// intrinsics or diameter unset (pose_valid left at 0).
// Annulus-correction factor for closed-form PnP.  For a Krajník
// marker with outer radius R and inner-white-disc radius r1, the
// 2nd-order moment along the major axis is
//   mu20 = (R² + r1²) / 4
// so the eigenvalue-derived semi-major axis is
//   axis_a = 2·sqrt(mu20) = sqrt(R² + r1²) = R·sqrt(1 + (r1/R)²)
// With the synth's r1/R = 0.6, that gives axis_a = R · sqrt(1.36)
// = R · 1.16619.  The closed-form PnP `z = fx · d / (2·axis_a)`
// assumes axis_a == R (solid disc), so for an annulus it under-
// estimates Z by a factor of 1/sqrt(1 + (r1/R)²).  Apply the
// inverse factor to recover the correct Z.
//
// Source: see s181 sweep — bias measured as 0.857× across the
// 0.2–1.2 m altitude band, matching the 1/1.166 = 0.857 prediction.
#define WHYCON_PNP_ANNULUS_FACTOR  1.16619f

static void whycon_pnp_inplace_(sentai_whycon_marker_t* m) {
    m->tvec_cam[0] = 0.0f;
    m->tvec_cam[1] = 0.0f;
    m->tvec_cam[2] = 0.0f;
    m->rvec_cam[0] = 0.0f;
    m->rvec_cam[1] = 0.0f;
    m->rvec_cam[2] = 0.0f;
    m->reproj_err_px = 0.0f;
    m->pose_valid = 0;
    if (s_fx <= 0.0f || s_fy <= 0.0f) return;
    if (s_whycon_diameter_m <= 0.0f) return;
    if (m->axis_a <= 0.001f) return;

    const float z = s_fx * s_whycon_diameter_m /
                       (2.0f * m->axis_a) * WHYCON_PNP_ANNULUS_FACTOR;
    m->tvec_cam[2] = z;
    m->tvec_cam[0] = (m->cx - s_cx) * z / s_fx;
    m->tvec_cam[1] = (m->cy - s_cy) * z / s_fy;

    // Tilt magnitude from axis ratio.  Clamp to [0, 1] before acos.
    float br = (m->axis_a > 0.0f) ? (m->axis_b / m->axis_a) : 1.0f;
    if (br > 1.0f) br = 1.0f;
    if (br < 0.0f) br = 0.0f;
    const float tilt = __builtin_acosf(br);
    // Tilt axis is perpendicular to the projected major axis (which
    // is at orientation `angle`); axis direction in the image plane.
    const float ux = -__builtin_sinf(m->angle);
    const float uy =  __builtin_cosf(m->angle);
    m->rvec_cam[0] = tilt * ux;
    m->rvec_cam[1] = tilt * uy;
    m->rvec_cam[2] = 0.0f;  // yaw around normal indeterminate

    m->pose_valid = 1;
}

// Filter + moments stage.  Runs over s_components (already populated
// by aruco_label_components) + s_labels (label map for moment scan).
//
// Algorithm per candidate component:
//   1) Coarse filter on area + bbox aspect + fill ratio  (rejects
//      non-circles cheaply).
//   2) Walk all pixels labelled `lab` within the bbox, accumulate
//      m10, m01, m20, m02, m11 (zeroth-order m00 already known as
//      c->n_pix).  ~5-15 cyc/px; bbox-bounded so it's much cheaper
//      than a full-frame scan even for large blobs.
//   3) Compute central moments + eigenvalues to extract semi-axes
//      a, b and axis angle.  Reject if a/b > max_axis_ratio.
//   4) Phase W3 (optional) — concentric inner-disc validation against
//      the source grayscale frame.  Adds ~17 pixel reads / candidate.
//   5) Closed-form PnP (optional) — if intrinsics + diameter are set.
//      Adds 4 divides + 2 trigs / candidate.
//
// Output written to s_whycon_markers[].  Returns count.  Per-substage
// cycles accumulated into s_whycon_t_w3 / s_whycon_t_pnp; the W1+W2
// time (filter + moments + eigenvalues) is timed by the caller as a
// single span and written to s_whycon_t_w.
static int whycon_filter_and_moments_(const uint8_t* gray,
                                        int n_comp, int W, int H) {
    s_whycon_n_markers = 0;
    uint32_t cyc_w3  = 0;
    uint32_t cyc_pnp = 0;
    for (int ci = 0; ci < n_comp; ++ci) {
        if (s_whycon_n_markers >= SENTAI_WHYCON_MAX_DETS) break;
        const aruco_comp_t* c = &s_components[ci];
        if (c->touches_border) continue;
        if (c->n_pix < s_whycon_min_area) continue;
        if (c->n_pix > s_whycon_max_area) continue;
        const int bw = c->x1 - c->x0 + 1;
        const int bh = c->y1 - c->y0 + 1;
        // bbox aspect — reject elongated rectangles.
        float ar = (bw > bh)
                     ? (float)bw / (float)bh
                     : (float)bh / (float)bw;
        if (ar > s_whycon_max_bbox_ar) continue;
        // Fill ratio — reject hollow / sparse shapes.
        float fill = (float)c->n_pix / (float)(bw * bh);
        if (fill < s_whycon_min_fill) continue;

        // OP-S10-W17-T2: 2nd-order moments are pre-accumulated by the
        // flood-fill in aruco_label_components, so this stage no longer
        // needs a bbox rescan.  c->m20_sum / m02_sum / m11_sum give the
        // raw moments; central moments + eigenvalues stay the same.
        const float m00 = (float)c->n_pix;
        const float cx  = (float)c->cx_sum / m00;
        const float cy  = (float)c->cy_sum / m00;
        const float mu20 = (float)c->m20_sum / m00 - cx * cx;
        const float mu02 = (float)c->m02_sum / m00 - cy * cy;
        const float mu11 = (float)c->m11_sum / m00 - cx * cy;
        // Eigenvalues of the covariance [[mu20, mu11],[mu11, mu02]].
        const float tr   = mu20 + mu02;
        const float det  = mu20 * mu02 - mu11 * mu11;
        const float disc = tr * tr * 0.25f - det;
        const float sq   = disc > 0.0f ? __builtin_sqrtf(disc) : 0.0f;
        const float l1   = tr * 0.5f + sq;
        const float l2   = tr * 0.5f - sq;
        const float a    = (l1 > 0.0f) ? 2.0f * __builtin_sqrtf(l1) : 0.0f;
        const float b    = (l2 > 0.0f) ? 2.0f * __builtin_sqrtf(l2) : 0.0f;
        if (b < 0.001f) continue;
        const float ax_ratio = a / b;
        if (ax_ratio > s_whycon_max_axis_ratio) continue;
        // Orientation of major axis.
        const float theta = 0.5f * __builtin_atan2f(2.0f * mu11, mu20 - mu02);

        // Phase W3 — concentric validation against grayscale frame.
        // Anchored on bbox_R (physical outer radius), not axis_a
        // (eigenvalue) — see whycon_w3_check_ comments.
        if (s_whycon_concentric_check) {
            const float bbox_R = (float)((bw > bh ? bw : bh)) * 0.5f;
            const uint32_t t_w3_0 = aruco_dwt_cyc();
            const int accept = whycon_w3_check_(gray, W, H, cx, cy, bbox_R);
            cyc_w3 += aruco_dwt_cyc() - t_w3_0;
            if (!accept) continue;
        }

        sentai_whycon_marker_t* m = &s_whycon_markers[s_whycon_n_markers++];
        m->cx = cx; m->cy = cy;
        m->axis_a = a; m->axis_b = b; m->angle = theta;
        m->comp_id = ci;

        // Closed-form PnP — emits tvec/rvec into the marker struct.
        const uint32_t t_pnp_0 = aruco_dwt_cyc();
        whycon_pnp_inplace_(m);
        cyc_pnp += aruco_dwt_cyc() - t_pnp_0;
    }
    s_whycon_t_w3  = cyc_w3;
    s_whycon_t_pnp = cyc_pnp;
    return s_whycon_n_markers;
}

// W17-T5 / W19-T2 Synth: Krajník-pattern marker = dark outer annulus
// + white inner disc + dark sub-pixel localiser dot.  This is what
// Phase W3 expects.  Three concentric zones — radii chosen to match
// the WHYCON_W3_INNER_RADIUS_FRAC (0.55) / OUTER_RADIUS_FRAC (0.95)
// sample geometry so the pattern is decidable by the radial samples.
//
// Geometry (outer radius R = full marker radius):
//   r <= 0.20 R           → DARK  (centre localiser, 9-px Phase W3 mean)
//   0.20 R < r <= 0.60 R  → WHITE (inner disc, 0.55 R radial samples)
//   0.60 R < r <= 1.00 R  → DARK  (outer ring, 0.95 R radial samples)
//   r > 1.00 R            → WHITE (background)
//
// Background and white intensities = 220 (matches existing lite synth);
// dark = 20.  Output written to s_test_gray; returns number drawn.

// W19-T1 / s181 — single Krajník marker at arbitrary pixel position.
// Returns 1 if a marker was drawn (geometry fit at all in frame), else 0.
static int whycon_synth_one_krajnik_(int cx, int cy, int radius,
                                        int W, int H) {
    memset(s_test_gray, 220, (size_t)W * (size_t)H);
    if (radius < 6) radius = 6;
    if (radius > 60) radius = 60;
    const int r_outer_sq  = radius * radius;
    const int r_white_sq  = (int)((float)r_outer_sq * 0.60f * 0.60f);
    const int r_centre_sq = (int)((float)r_outer_sq * 0.20f * 0.20f);
    int touched = 0;
    for (int y = cy - radius; y <= cy + radius; ++y) {
        if (y < 0 || y >= H) continue;
        for (int x = cx - radius; x <= cx + radius; ++x) {
            if (x < 0 || x >= W) continue;
            const int dxp = x - cx;
            const int dyp = y - cy;
            const int r2  = dxp * dxp + dyp * dyp;
            if (r2 > r_outer_sq) continue;
            uint8_t v;
            if (r2 <= r_centre_sq)      v = 20;
            else if (r2 <= r_white_sq)  v = 220;
            else                        v = 20;
            s_test_gray[x + y * W] = v;
            touched = 1;
        }
    }
    return touched;
}

static int whycon_synth_frame_krajnik_(int n_circles, int radius,
                                          int W, int H) {
    memset(s_test_gray, 220, (size_t)W * (size_t)H);
    if (n_circles <= 0) return 0;
    if (n_circles > 8) n_circles = 8;
    if (radius < 6) radius = 6;
    if (radius > 30) radius = 30;
    const int cols = (n_circles > 4) ? 4 : n_circles;
    const int rows = (n_circles + cols - 1) / cols;
    const int dx = W / (cols + 1);
    const int dy = H / (rows + 1);
    const int r_outer_sq = radius * radius;
    const int r_white_sq = (int)((float)r_outer_sq * 0.60f * 0.60f);
    const int r_centre_sq = (int)((float)r_outer_sq * 0.20f * 0.20f);
    int drawn = 0;
    for (int i = 0; i < n_circles; ++i) {
        const int col = i % cols;
        const int row = i / cols;
        const int cx = (col + 1) * dx + (i * 7) % 5;
        const int cy = (row + 1) * dy + (i * 13) % 5;
        for (int y = cy - radius; y <= cy + radius; ++y) {
            if (y < 0 || y >= H) continue;
            for (int x = cx - radius; x <= cx + radius; ++x) {
                if (x < 0 || x >= W) continue;
                const int dxp = x - cx;
                const int dyp = y - cy;
                const int r2  = dxp * dxp + dyp * dyp;
                if (r2 > r_outer_sq) continue;
                uint8_t v;
                if (r2 <= r_centre_sq)      v = 20;   // centre dark dot
                else if (r2 <= r_white_sq)  v = 220;  // inner white
                else                        v = 20;   // outer dark ring
                s_test_gray[x + y * W] = v;
            }
        }
        ++drawn;
    }
    return drawn;
}

// Synth frame generator — draws N filled black disks on a white
// background.  Deterministic positions on a coarse grid + small
// jitter from index, so the bench is reproducible.  Output written
// to s_test_gray, returns number of disks actually drawn.
static int whycon_synth_frame_(int n_circles, int radius, int W, int H) {
    // White background.
    memset(s_test_gray, 220, (size_t)W * (size_t)H);
    if (n_circles <= 0) return 0;
    if (n_circles > 8) n_circles = 8;
    if (radius < 4) radius = 4;
    if (radius > 30) radius = 30;
    // Lay out on up-to-4x2 grid.
    const int cols = (n_circles > 4) ? 4 : n_circles;
    const int rows = (n_circles + cols - 1) / cols;
    const int dx = W / (cols + 1);
    const int dy = H / (rows + 1);
    int drawn = 0;
    for (int i = 0; i < n_circles; ++i) {
        const int col = i % cols;
        const int row = i / cols;
        const int cx = (col + 1) * dx + (i * 7) % 5;   // small jitter
        const int cy = (row + 1) * dy + (i * 13) % 5;
        for (int y = cy - radius; y <= cy + radius; ++y) {
            if (y < 0 || y >= H) continue;
            for (int x = cx - radius; x <= cx + radius; ++x) {
                if (x < 0 || x >= W) continue;
                const int dxp = x - cx;
                const int dyp = y - cy;
                if (dxp * dxp + dyp * dyp <= radius * radius) {
                    s_test_gray[x + y * W] = 20;   // dark disk
                }
            }
        }
        ++drawn;
    }
    return drawn;
}

// Top-level WhyCon-lite detect — used both standalone (synth bench)
// and from the public _test_pgm wrapper.  Frame size is fixed 320×240
// (production resolution).
// Per-stage cycle counters — populated by whycon_detect_inplace_ on
// every call, queryable via sentai.whycon._stage_cyc() for honest
// per-stage breakdown (OP-S10-W17 follow-up after the Flow-opt
// review showed the USAD8-style tricks don't apply to bulk WhyCon).
// W17-T5 / W19-T2 extension: separate counters for Phase W3
// (concentric inner-disc validation) and PnP (closed-form z + xy).
// When W3 is disabled, t_w3 stays 0; when intrinsics+diameter are
// unset, t_pnp stays 0 — the detector skips those sub-stages.
static volatile uint32_t s_whycon_t_a   = 0;  // Phase A: rolling threshold
static volatile uint32_t s_whycon_t_b   = 0;  // Phase B: 8-conn flood fill
static volatile uint32_t s_whycon_t_w   = 0;  // Phase W1+W2: filter + axes (eigenvalues)
// s_whycon_t_w3 / s_whycon_t_pnp forward-declared near the top of the
// WhyCon section so the filter+moments writer (defined above) compiles.

static int whycon_detect_inplace_(int W, int H) {
    const uint32_t t0 = aruco_dwt_cyc();
    // Stage A — rolling-integral Bradley threshold.
    aruco_adaptive_threshold_rolling(s_test_gray, W, H, 31, s_binary);
    const uint32_t t1 = aruco_dwt_cyc();
    // Stage B — 8-connected flood-fill labeling.  Per-component
    // 2nd-order moments accumulated inline (OP-S10-W17-T2 8eccf31b).
    const int n_comp = aruco_label_components(W, H);
    const uint32_t t2 = aruco_dwt_cyc();
    // Stage W1+W2 — filter + axes from moments.  W3 concentric +
    // PnP are timed internally and written to s_whycon_t_w3/_pnp.
    const int n = whycon_filter_and_moments_(s_test_gray, n_comp, W, H);
    const uint32_t t3 = aruco_dwt_cyc();
    s_whycon_t_a = t1 - t0;
    s_whycon_t_b = t2 - t1;
    // Subtract W3 + PnP so t_w reflects ONLY filter + moments + axes.
    const uint32_t span_w = t3 - t2;
    const uint32_t inner  = s_whycon_t_w3 + s_whycon_t_pnp;
    s_whycon_t_w = (span_w > inner) ? (span_w - inner) : 0;
    return n;
}

extern "C" void sentai_whycon_stage_cyc(uint32_t* t_a, uint32_t* t_b,
                                          uint32_t* t_w) {
    if (t_a) *t_a = s_whycon_t_a;
    if (t_b) *t_b = s_whycon_t_b;
    if (t_w) *t_w = s_whycon_t_w;
}

// W17-T5 / W19-T2: full 5-stage breakdown including W3 + PnP.
extern "C" void sentai_whycon_stage_cyc5(uint32_t* t_a, uint32_t* t_b,
                                           uint32_t* t_w, uint32_t* t_w3,
                                           uint32_t* t_pnp) {
    if (t_a)   *t_a   = s_whycon_t_a;
    if (t_b)   *t_b   = s_whycon_t_b;
    if (t_w)   *t_w   = s_whycon_t_w;
    if (t_w3)  *t_w3  = s_whycon_t_w3;
    if (t_pnp) *t_pnp = s_whycon_t_pnp;
}

// Timing wrapper: build/load gray frame into s_test_gray, then run
// the WhyCon-lite pipeline with DWT cycle counting around the
// detection cost only (NOT the synth/PGM-load cost).
static volatile uint32_t s_whycon_cyc_last = 0;

extern "C" int sentai_whycon_test_synth(int n_circles, int radius) {
    const int W = 320, H = 240;
    if (n_circles < 0) n_circles = 0;
    whycon_synth_frame_(n_circles, radius, W, H);
    const uint32_t t0 = aruco_dwt_cyc();
    const int n = whycon_detect_inplace_(W, H);
    const uint32_t t1 = aruco_dwt_cyc();
    s_whycon_cyc_last = t1 - t0;
    return n;
}

// W17-T5 / W19-T2 Krajník-pattern variant for the W3 + PnP bench.
extern "C" int sentai_whycon_test_synth_krajnik(int n_circles, int radius) {
    const int W = 320, H = 240;
    if (n_circles < 0) n_circles = 0;
    whycon_synth_frame_krajnik_(n_circles, radius, W, H);
    const uint32_t t0 = aruco_dwt_cyc();
    const int n = whycon_detect_inplace_(W, H);
    const uint32_t t1 = aruco_dwt_cyc();
    s_whycon_cyc_last = t1 - t0;
    return n;
}

// W19-T1 / s181 — single Krajník marker at arbitrary pixel position
// + radius.  Used by the SIM evaluator to forward-project known
// world poses (X, Y, Z) through the pinhole intrinsics into image
// space, then verify the closed-form PnP recovers the world pose.
extern "C" int sentai_whycon_synth_one(int cx_px, int cy_px, int radius_px) {
    const int W = 320, H = 240;
    if (!whycon_synth_one_krajnik_(cx_px, cy_px, radius_px, W, H)) {
        s_whycon_n_markers = 0;
        return 0;
    }
    const uint32_t t0 = aruco_dwt_cyc();
    const int n = whycon_detect_inplace_(W, H);
    const uint32_t t1 = aruco_dwt_cyc();
    s_whycon_cyc_last = t1 - t0;
    return n;
}

extern "C" int sentai_whycon_test_pgm(const char* path) {
    const int W = 320, H = 240;
    // Re-use the ArUco PGM loader — both fopen and FxUser fallback
    // paths populate s_test_gray with the 320×240 grayscale payload.
    // We replicate the loader here as a thin call to keep timing
    // pure (the actual ArUco function would also run detect after).
    FILE* fp = fopen(path, "rb");
    if (fp) {
        char header[3] = {0};
        int wpgm = 0, hpgm = 0, maxval = 0;
        if (fscanf(fp, "%2s %d %d %d", header, &wpgm, &hpgm, &maxval) != 4
            || header[0] != 'P' || header[1] != '5'
            || wpgm != W || hpgm != H || maxval != 255) {
            fclose(fp); return -2;
        }
        fgetc(fp);
        if (fread(s_test_gray, 1, (size_t)W * (size_t)H, fp)
              != (size_t)W * (size_t)H) {
            fclose(fp); return -2;
        }
        fclose(fp);
    } else {
        ssize_t sz = FxUserSize(path);
        if (sz < 0) return -1;
        if ((size_t)sz > sizeof(s_labels)) return -3;
        size_t got = FxUserReadFile(path, s_labels, sizeof(s_labels));
        if (got == 0) return -1;
        int rc = aruco_parse_pgm_buffer_(s_labels, got);
        if (rc != 0) return rc;
    }
    const uint32_t t0 = aruco_dwt_cyc();
    const int n = whycon_detect_inplace_(W, H);
    const uint32_t t1 = aruco_dwt_cyc();
    s_whycon_cyc_last = t1 - t0;
    return n;
}

extern "C" uint32_t sentai_whycon_detect_cyc_last(void) {
    return s_whycon_cyc_last;
}

extern "C" int sentai_whycon_get_markers(sentai_whycon_marker_t* out,
                                           int out_capacity) {
    if (!out || out_capacity <= 0) return 0;
    const int n = (s_whycon_n_markers < out_capacity)
                    ? s_whycon_n_markers : out_capacity;
    memcpy(out, s_whycon_markers, (size_t)n * sizeof(sentai_whycon_marker_t));
    return n;
}

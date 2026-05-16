// sentai_phog.h — Pyramid Histogram of Oriented Gradients (PHOG).
//
// Track A foundation per ideas/objects_plan.md §22.2.  Computes a
// 168-D float descriptor from a grayscale image:
//   3 pyramid levels (L0=1 cell, L1=2×2=4 cells, L2=4×4=16 cells)
//   8 orientation bins per cell (unsigned gradient, [0, π))
//   total: 8 × (1 + 4 + 16) = 168 floats
// Per-cell L2 normalization (standard PHOG / Bosch 2007).
//
// Caller responsibility:
//   - Provide gray image as uint8 buffer, row-major, w × h pixels.
//   - Provide output buffer ≥ 168 floats.
//   - Image bounds w ≥ 8, h ≥ 8 (must accommodate 4×4 cells × 2 px each).
//
// References:
//   Bosch, Zisserman, Munoz "Representing Shape with a Spatial Pyramid
//   Kernel", CIVR 2007 — primary PHOG.
//   Dalal & Triggs CVPR 2005 — HOG foundation.
//
// Cold-path: called at ~1 Hz from the place-fingerprint task once
// Track A is wired in.  ~10 ms on M7 expected.  ARM placement
// .sdram_text per [[itcm-budget]].

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENTAI_PHOG_BINS          8
#define SENTAI_PHOG_LEVELS        3
#define SENTAI_PHOG_N_CELLS       (1 + 4 + 16)            // 21
#define SENTAI_PHOG_DIM           (SENTAI_PHOG_BINS * SENTAI_PHOG_N_CELLS)  // 168

// Compute PHOG descriptor from a gray image.
//
// gray         — row-major uint8 pixels, length w*h
// w, h         — image dimensions (≥ 8 each)
// out          — output buffer, ≥ SENTAI_PHOG_DIM floats
//
// Returns: 0 on success, -1 on invalid input.
//
// Notes:
//   - Uses Sobel 3×3 for gradient (clamped to int16 to avoid overflow).
//   - Orientation in [0, π) (unsigned gradient — standard for PHOG).
//   - L2 normalization per cell with epsilon=1e-6 to avoid div by zero.
int sentai_phog_compute(const uint8_t* gray, int w, int h, float* out);

#ifdef __cplusplus
}
#endif

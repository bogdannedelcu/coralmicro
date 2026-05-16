// sentai_gist.h — GIST-lite coarse scene descriptor.
//
// Track A piece 2/4 per ideas/objects_plan.md §22.2.  GIST captures
// scene-level "spatial envelope" (Oliva & Torralba IJCV 2001) by
// applying oriented filters at coarse resolution and pooling spatially.
//
// This implementation is a thesis-MVP simplification of full Gabor-GIST:
//   - Decimation:    input downsampled to (w/4, h/4) by 4-pixel block
//                     averaging (low-pass + downsample combined).
//   - Filters:       4 directional gradient kernels — DX (Sobel-X),
//                     DY (Sobel-Y), D45 (diagonal NW-SE), D135 (NE-SW).
//                     These approximate the 4 dominant Gabor orientations
//                     without the full multi-scale Gabor bank.
//   - Spatial pool:  4×4 grid on decimated image, mean |response| per cell.
//   - Output:        4 orientations × 16 cells = 64 floats (L2-normalized
//                     per orientation across the 16 cells).
//
// Trade-off:
//   - Full Gabor-GIST: 4 scales × 8 orientations = 32 filters × full image
//     ≈ 120 M ops/frame on RT1176 → too slow.  Decimated 4-orient is
//     ~1 ms M7, well within cold-path budget.
//   - Discriminability vs full GIST: coarser; complemented by PHOG
//     (s139, multi-scale edges) and FFT-mag (s142, rotation-invariant).
//
// Reference:
//   Oliva, A., Torralba, A. "Modeling the shape of the scene: A holistic
//   representation of the spatial envelope". IJCV 42(3), 2001.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENTAI_GIST_ORIENT     4    // DX, DY, D45, D135
#define SENTAI_GIST_GRID       4    // 4×4 spatial pool
#define SENTAI_GIST_CELLS      (SENTAI_GIST_GRID * SENTAI_GIST_GRID)   // 16
#define SENTAI_GIST_DIM        (SENTAI_GIST_ORIENT * SENTAI_GIST_CELLS) // 64

// Compute GIST-lite from gray image.
//   gray  — w*h uint8 pixels, row-major
//   w, h  — image dims, both must be ≥ 16 (so 4x decim leaves ≥ 4×4)
//   out   — output buffer, ≥ SENTAI_GIST_DIM floats
// Returns 0 ok, -1 invalid input.
int sentai_gist_compute(const uint8_t* gray, int w, int h, float* out);

#ifdef __cplusplus
}
#endif

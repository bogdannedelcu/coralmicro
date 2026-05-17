// sentai_hsv.h — HSV-histogram descriptor (Track A piece 3/4).
//
// OP-S10-W4 per ideas/objects_plan.md §22.2 (Places — Track A).
// Joint Hue×Saturation histogram on the camera RGB888 frame:
//   16 hue bins  (0-360°, 22.5° each)
//   ×  4 sat bins (0-255, 64 each)
//   = 64 bins total, each stored as uint8 (count, normalized to 255)
//
// The output is BYTE-FOR-BYTE slot-compatible with the existing
// sentai.places L3 gallery (`SENTAI_PLACES_DESC_DIM = 64`, comment
// "16 hue × 4 sat bins (uint8_t each)" — this descriptor IS the
// intended slot layout per the original L3 design).  No quantize
// step needed (unlike PHOG/GIST which return floats and get packed
// in hex_helpers.py).
//
// V channel is intentionally DISCARDED.  HSV's V is just max(R,G,B)
// and tracks illumination directly; dropping it makes the descriptor
// brightness-invariant, a desirable property for revisit recognition
// under changing lighting.  Pixels darker than SENTAI_HSV_V_MIN are
// excluded from the histogram (hue is unreliable in near-black
// pixels — chroma is dominated by sensor noise).
//
// Algorithm: Smith (1978) "Color gamut transform pairs" RGB↔HSV via
// the sectoral max-channel formula.  Pure integer math (no atan2 /
// sqrt) — H is recovered from which of R/G/B is the max plus a
// linear interpolation of the other two, scaled by ∆ = max-min.
// CMSIS-DSP not applicable on this kernel (the inner loop is
// per-pixel integer compare + subtract + divide — no matrix / FFT
// to dispatch).
//
// Cold-path: ~30 cyc/pixel × 4096 px (64×64) ≈ 150 µs on M7.
// Placement: .sdram_text per [[itcm-budget]], matching the PHOG /
// GIST routing in the linker script.
//
// References:
//   Smith, A.R. "Color gamut transform pairs", SIGGRAPH 1978.
//   Swain, M., Ballard, D. "Color indexing", IJCV 7(1), 1991.
//   §22.5 ideas/objects_plan.md — Track A descriptor stack.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENTAI_HSV_HBINS   16     // 22.5° per hue bin
#define SENTAI_HSV_SBINS    4     // 64 per saturation bin
#define SENTAI_HSV_DIM     (SENTAI_HSV_HBINS * SENTAI_HSV_SBINS)   // 64
#define SENTAI_HSV_V_MIN   16     // exclude pixels with V < this

// Compute HS histogram from an RGB888-packed image.
//
// rgb   — 3*w*h bytes, packed [R, G, B, R, G, B, ...]  (NOT XRGB —
//          caller must already have RGB888 packed; on ARM use
//          pxp_scale_xrgb_to_rgb to convert from CSI native XRGB8888).
// w, h  — image dimensions (≥ 8 each).
// out   — output buffer, ≥ SENTAI_HSV_DIM bytes (= 64 bytes).
//
// Output layout: bin index = h_bin * SENTAI_HSV_SBINS + s_bin
// (consistent with the slot comment in sentai_places.h).
//
// Returns: 0 on success, -1 on invalid input.  All bins are 0 if
// every pixel was excluded by the V_MIN gate (caller should treat
// the result as a "dark scene, no descriptor" signal).
int sentai_hsv_compute(const uint8_t* rgb, int w, int h, uint8_t* out);

#ifdef __cplusplus
}
#endif

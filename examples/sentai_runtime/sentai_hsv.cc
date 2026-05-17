// sentai_hsv.cc — HSV histogram descriptor (Track A piece 3/4).
// See sentai_hsv.h for the design + references.
//
// Pure integer kernel — no float math, no atan2/sqrt.  Standard
// Smith-1978 sectoral RGB→HSV: H is recovered from which RGB channel
// is the max + linear interpolation of the other two scaled by
// ∆ = max − min.  Three integer divisions per pixel (one for S, one
// for the H interpolation, one for the final normalisation pass);
// the divisor is bounded so this stays under ~30 cyc/pixel on M7.

#include "sentai_hsv.h"

#include <stdint.h>
#include <string.h>

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #define SENTAI_HSV_TEXT   /* nothing */
#else
  #define SENTAI_HSV_TEXT   __attribute__((section(".sdram_text"), noinline))
#endif

// Per-pixel: returns 1 if the pixel was included in the histogram, 0
// if it was masked out by the V_MIN gate.  Writes h_bin and s_bin via
// pointers.
//
// Hue convention (Smith 1978):
//   sector 0  R is max, G ≥ B: H ∈ [0, 60)        — red→yellow
//   sector 1  G is max, R ≥ B: H ∈ [60, 120)      — yellow→green
//   sector 2  G is max, B > R: H ∈ [120, 180)     — green→cyan
//   sector 3  B is max, G ≥ R: H ∈ [180, 240)     — cyan→blue
//   sector 4  B is max, R > G: H ∈ [240, 300)     — blue→magenta
//   sector 5  R is max, B > G: H ∈ [300, 360)     — magenta→red
//
// We avoid float by scaling the interpolation: H_scaled = 60 * (b-c)/Δ,
// then add the sector offset.  All arithmetic stays in int32_t and
// the result is wrapped into the 16-bin grid via (H_scaled * 16) / 360.
//
// S = ∆ / V scaled to [0, 255] via S = (∆ * 256) / V (clamped at 255).
// S_bin = S / 64 (i.e. 4 saturation bins: [0,64), [64,128), [128,192),
// [192,256)).  At S=255 the bin is 3.
//
// V < V_MIN → pixel excluded (return 0).
static inline int hsv_pixel_bins(uint8_t r, uint8_t g, uint8_t b,
                                  int* h_bin, int* s_bin) {
    // Find max + min.
    uint8_t mx = r;  uint8_t mn = r;  int max_ch = 0;
    if (g > mx) { mx = g; max_ch = 1; }
    if (b > mx) { mx = b; max_ch = 2; }
    if (g < mn) mn = g;
    if (b < mn) mn = b;
    if (mx < SENTAI_HSV_V_MIN) return 0;

    const int delta = (int)mx - (int)mn;
    if (delta == 0) {
        // Achromatic — saturation 0, hue undefined.  We bin it as
        // (H=0, S=0) so all gray/white/black pixels land in bin 0.
        *h_bin = 0;
        *s_bin = 0;
        return 1;
    }

    // Saturation in [0, 255].  S = ∆ / V × 255.  V = mx (∈ [V_MIN, 255]).
    int s = (delta * 255 + (mx >> 1)) / mx;
    if (s > 255) s = 255;
    int sb = s >> 6;                  // /64; clip via (s>>6) ≤ 3 since s≤255
    if (sb > 3) sb = 3;

    // Hue in 0..359 (degrees scaled).  Numerator (g-b)/(b-r)/(r-g) ∈
    // [-delta, +delta] so we keep ints small.
    int h;
    if (max_ch == 0) {
        h = (60 * ((int)g - (int)b)) / delta;
        if (h < 0) h += 360;
    } else if (max_ch == 1) {
        h = 120 + (60 * ((int)b - (int)r)) / delta;
    } else {
        h = 240 + (60 * ((int)r - (int)g)) / delta;
    }
    if (h < 0)   h += 360;
    if (h >= 360) h -= 360;

    int hb = (h * SENTAI_HSV_HBINS) / 360;
    if (hb >= SENTAI_HSV_HBINS) hb = SENTAI_HSV_HBINS - 1;

    *h_bin = hb;
    *s_bin = sb;
    return 1;
}

SENTAI_HSV_TEXT int sentai_hsv_compute(const uint8_t* rgb, int w, int h,
                                        uint8_t* out) {
    if (!rgb || !out || w < 8 || h < 8) return -1;
    // Initialise output to zeros so the caller's stale memory doesn't
    // leak when the V_MIN gate excludes everything.
    memset(out, 0, SENTAI_HSV_DIM);

    // Histogram in uint32 — at 64×64 max bin count is 4096; at the
    // expected 320×240 it's 76800, which still fits comfortably.
    uint32_t bins[SENTAI_HSV_DIM] = {0};
    uint32_t total_valid = 0;

    const int n_pixels = w * h;
    const uint8_t* p = rgb;
    for (int i = 0; i < n_pixels; ++i) {
        int hb, sb;
        if (hsv_pixel_bins(p[0], p[1], p[2], &hb, &sb)) {
            bins[hb * SENTAI_HSV_SBINS + sb]++;
            total_valid++;
        }
        p += 3;
    }

    if (total_valid == 0) {
        // Dark scene — out already zero; signal-as-zero per header.
        return 0;
    }

    // Normalise each bin to a uint8 fraction-of-total.
    //   out[i] = clip255(bins[i] * 256 / total_valid)
    // The fraction of any single bin in [0, 1]; multiplying by 256 then
    // clipping to 255 gives a uint8 that L1-distance-compares as a
    // discrete probability mass.
    for (int i = 0; i < SENTAI_HSV_DIM; ++i) {
        uint32_t v = (bins[i] * 256u + (total_valid >> 1)) / total_valid;
        if (v > 255u) v = 255u;
        out[i] = (uint8_t)v;
    }
    return 0;
}

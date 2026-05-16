// sentai_phog.cc — PHOG descriptor compute.
//
// Cold-path (~1 Hz, ~10 ms on M7).  Cross-platform: same .cc compiles
// for ARM and SIM via SENTAI_PLATFORM_SIM / __arm__ guard.

#include "sentai_phog.h"

#include <math.h>
#include <string.h>

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #define SENTAI_PHOG_TEXT  /* nothing */
#else
  // Per [[itcm-budget]]: cold-path code lives in .sdram_text on ARM.
  // noinline keeps the section attribute alive through LTO.
  #define SENTAI_PHOG_TEXT  __attribute__((section(".sdram_text"), noinline))
#endif

// Pi for orientation binning.
#define PHOG_PI  3.14159265358979323846f

// Inline 3×3 Sobel at (x, y).  Borders extended (clamped index).
static inline int sobel_x_at(const uint8_t* gray, int w, int h, int x, int y) {
    int xm = (x > 0)        ? (x - 1) : x;
    int xp = (x < w - 1)    ? (x + 1) : x;
    int ym = (y > 0)        ? (y - 1) : y;
    int yp = (y < h - 1)    ? (y + 1) : y;
    int a = (int)gray[ym*w + xm];
    int b = (int)gray[y *w + xm];
    int c = (int)gray[yp*w + xm];
    int d = (int)gray[ym*w + xp];
    int e = (int)gray[y *w + xp];
    int f = (int)gray[yp*w + xp];
    return (d + 2*e + f) - (a + 2*b + c);
}

static inline int sobel_y_at(const uint8_t* gray, int w, int h, int x, int y) {
    int xm = (x > 0)        ? (x - 1) : x;
    int xp = (x < w - 1)    ? (x + 1) : x;
    int ym = (y > 0)        ? (y - 1) : y;
    int yp = (y < h - 1)    ? (y + 1) : y;
    int a = (int)gray[ym*w + xm];
    int b = (int)gray[ym*w + x ];
    int c = (int)gray[ym*w + xp];
    int d = (int)gray[yp*w + xm];
    int e = (int)gray[yp*w + x ];
    int f = (int)gray[yp*w + xp];
    return (d + 2*e + f) - (a + 2*b + c);
}

// Assign orientation in [0, π) to bin index ∈ [0, BINS).
static inline int phog_bin(float angle_rad) {
    // Unfold angle to [0, π): atan2 returns (-π, π]; we fold by abs of
    // (sin/cos) for unsigned gradient.  Equivalently, angle_unsigned =
    // angle mod π.
    float a = angle_rad;
    while (a < 0.f)         a += PHOG_PI;
    while (a >= PHOG_PI)    a -= PHOG_PI;
    int b = (int)(a * (float)SENTAI_PHOG_BINS / PHOG_PI);
    if (b < 0) b = 0;
    if (b >= SENTAI_PHOG_BINS) b = SENTAI_PHOG_BINS - 1;
    return b;
}

// Accumulate gradient magnitude into a cell histogram by orientation bin.
// Cell range: [x0, x1) × [y0, y1).
static SENTAI_PHOG_TEXT void phog_cell_accumulate(
        const uint8_t* gray, int w, int h,
        int x0, int y0, int x1, int y1, float* cell_hist /* BINS */) {
    for (int b = 0; b < SENTAI_PHOG_BINS; ++b) cell_hist[b] = 0.f;
    if (x0 < 0)  x0 = 0;
    if (y0 < 0)  y0 = 0;
    if (x1 > w)  x1 = w;
    if (y1 > h)  y1 = h;
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            int gx = sobel_x_at(gray, w, h, x, y);
            int gy = sobel_y_at(gray, w, h, x, y);
            float mag = sqrtf((float)(gx*gx + gy*gy));
            if (mag < 1.f) continue;  // suppress noise
            float ang = atan2f((float)gy, (float)gx);
            int b = phog_bin(ang);
            cell_hist[b] += mag;
        }
    }
}

// L2-normalize a cell histogram in-place.
static inline void phog_l2_norm(float* hist) {
    float sum2 = 0.f;
    for (int b = 0; b < SENTAI_PHOG_BINS; ++b) sum2 += hist[b] * hist[b];
    float n = sqrtf(sum2);
    if (n < 1e-6f) {
        // empty histogram → leave zeros (caller can detect via sum)
        return;
    }
    float inv = 1.f / n;
    for (int b = 0; b < SENTAI_PHOG_BINS; ++b) hist[b] *= inv;
}

SENTAI_PHOG_TEXT int sentai_phog_compute(const uint8_t* gray, int w, int h,
                                         float* out) {
    if (gray == NULL || out == NULL) return -1;
    if (w < 8 || h < 8) return -1;

    int write_idx = 0;

    // Level L0 — 1 cell over whole image
    phog_cell_accumulate(gray, w, h, 0, 0, w, h, &out[write_idx]);
    phog_l2_norm(&out[write_idx]);
    write_idx += SENTAI_PHOG_BINS;

    // Level L1 — 2×2 cells
    {
        int cw = w / 2;
        int ch = h / 2;
        for (int cy = 0; cy < 2; ++cy) {
            for (int cx = 0; cx < 2; ++cx) {
                int x0 = cx * cw;
                int y0 = cy * ch;
                int x1 = (cx == 1) ? w : x0 + cw;
                int y1 = (cy == 1) ? h : y0 + ch;
                phog_cell_accumulate(gray, w, h, x0, y0, x1, y1, &out[write_idx]);
                phog_l2_norm(&out[write_idx]);
                write_idx += SENTAI_PHOG_BINS;
            }
        }
    }

    // Level L2 — 4×4 cells
    {
        int cw = w / 4;
        int ch = h / 4;
        for (int cy = 0; cy < 4; ++cy) {
            for (int cx = 0; cx < 4; ++cx) {
                int x0 = cx * cw;
                int y0 = cy * ch;
                int x1 = (cx == 3) ? w : x0 + cw;
                int y1 = (cy == 3) ? h : y0 + ch;
                phog_cell_accumulate(gray, w, h, x0, y0, x1, y1, &out[write_idx]);
                phog_l2_norm(&out[write_idx]);
                write_idx += SENTAI_PHOG_BINS;
            }
        }
    }

    return 0;
}

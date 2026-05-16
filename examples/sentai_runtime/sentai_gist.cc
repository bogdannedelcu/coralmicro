// sentai_gist.cc — GIST-lite descriptor compute.
//
// Cold-path ~1 Hz on M7 (~1-2 ms estimated post-decimation).  Cross-
// platform via SENTAI_PLATFORM_SIM / __arm__ guard.

#include "sentai_gist.h"

#include <math.h>
#include <string.h>

#if defined(SENTAI_PLATFORM_SIM) || !defined(__arm__)
  #define SENTAI_GIST_TEXT  /* nothing */
#else
  #define SENTAI_GIST_TEXT  __attribute__((section(".sdram_text"), noinline))
#endif

// Decimate gray w*h → w/4 * h/4 by 4×4 block average.
// Caller provides `decim` buffer of size (w/4)*(h/4).
static SENTAI_GIST_TEXT void gist_decimate_4x(const uint8_t* gray, int w, int h,
                                               uint8_t* decim) {
    int dw = w / 4;
    int dh = h / 4;
    for (int dy = 0; dy < dh; ++dy) {
        for (int dx = 0; dx < dw; ++dx) {
            int sum = 0;
            int x0 = dx * 4;
            int y0 = dy * 4;
            for (int yy = 0; yy < 4; ++yy) {
                const uint8_t* row = &gray[(y0 + yy) * w + x0];
                sum += row[0] + row[1] + row[2] + row[3];
            }
            decim[dy * dw + dx] = (uint8_t)(sum >> 4);  // /16
        }
    }
}

// 3×3 kernels applied at (x, y) on decimated buffer.  Borders clamped.
// Returns the 4 oriented responses (raw, can be negative).
static inline void gist_kernels(const uint8_t* d, int dw, int dh, int x, int y,
                                 int* dx, int* dy_g, int* d45, int* d135) {
    int xm = (x > 0)       ? (x - 1) : x;
    int xp = (x < dw - 1)  ? (x + 1) : x;
    int ym = (y > 0)       ? (y - 1) : y;
    int yp = (y < dh - 1)  ? (y + 1) : y;
    int p00 = (int)d[ym*dw + xm];
    int p01 = (int)d[ym*dw + x ];
    int p02 = (int)d[ym*dw + xp];
    int p10 = (int)d[y *dw + xm];
    int p12 = (int)d[y *dw + xp];
    int p20 = (int)d[yp*dw + xm];
    int p21 = (int)d[yp*dw + x ];
    int p22 = (int)d[yp*dw + xp];
    // Sobel X: [-1 0 +1; -2 0 +2; -1 0 +1]
    *dx     = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20);
    // Sobel Y: [-1 -2 -1; 0 0 0; +1 +2 +1]
    *dy_g   = (p20 + 2*p21 + p22) - (p00 + 2*p01 + p02);
    // Diagonal NW-SE (D45):  +1 along NW→SE direction
    //   [-1  0  0;
    //     0  0  0;
    //     0  0 +1]
    // Scaled 2× to keep magnitude comparable.
    *d45    = 2 * (p22 - p00);
    // Diagonal NE-SW (D135):
    //   [ 0  0 -1;
    //     0  0  0;
    //    +1  0  0]
    *d135   = 2 * (p20 - p02);
}

SENTAI_GIST_TEXT int sentai_gist_compute(const uint8_t* gray, int w, int h,
                                         float* out) {
    if (gray == NULL || out == NULL) return -1;
    if (w < 16 || h < 16) return -1;

    int dw = w / 4;
    int dh = h / 4;
    if (dw < SENTAI_GIST_GRID || dh < SENTAI_GIST_GRID) return -1;

    // Stack decimated buffer.  Max use ~120*90 = 10800 bytes for 480×360
    // input; conservatively cap at 80×60 = 4800 bytes for typical 320×240.
    // For larger images, we'd need a heap buffer.
    static uint8_t s_decim[160 * 120];  // covers up to 640×480 input
    if (dw * dh > (int)sizeof(s_decim)) return -1;

    gist_decimate_4x(gray, w, h, s_decim);

    // Accumulate |response| per (orientation, cell).
    // out layout: out[orient * CELLS + cell] where orient = 0..3.
    for (int i = 0; i < SENTAI_GIST_DIM; ++i) out[i] = 0.f;
    float counts[SENTAI_GIST_CELLS] = {0.f};

    int cell_w = dw / SENTAI_GIST_GRID;
    int cell_h = dh / SENTAI_GIST_GRID;
    for (int y = 0; y < dh; ++y) {
        // Map y to row cell, with the last cell taking the remainder.
        int cy = y / cell_h;
        if (cy >= SENTAI_GIST_GRID) cy = SENTAI_GIST_GRID - 1;
        for (int x = 0; x < dw; ++x) {
            int cx = x / cell_w;
            if (cx >= SENTAI_GIST_GRID) cx = SENTAI_GIST_GRID - 1;
            int cell = cy * SENTAI_GIST_GRID + cx;
            int dx, dy_g, d45, d135;
            gist_kernels(s_decim, dw, dh, x, y, &dx, &dy_g, &d45, &d135);
            out[0 * SENTAI_GIST_CELLS + cell] += (float)(dx  < 0 ?  -dx  :  dx);
            out[1 * SENTAI_GIST_CELLS + cell] += (float)(dy_g < 0 ? -dy_g : dy_g);
            out[2 * SENTAI_GIST_CELLS + cell] += (float)(d45  < 0 ? -d45  : d45);
            out[3 * SENTAI_GIST_CELLS + cell] += (float)(d135 < 0 ? -d135 : d135);
            counts[cell] += 1.f;
        }
    }

    // Mean per cell (divide accumulated by pixel count).
    for (int c = 0; c < SENTAI_GIST_CELLS; ++c) {
        float n = counts[c];
        if (n < 1.f) continue;
        float inv = 1.f / n;
        for (int o = 0; o < SENTAI_GIST_ORIENT; ++o) {
            out[o * SENTAI_GIST_CELLS + c] *= inv;
        }
    }

    // L2-normalize per CELL across the 4 orientations.  This keeps
    // cross-orientation discrimination (a cell with strong DX vs strong
    // DY stays distinguishable) while still being lighting-invariant
    // (overall magnitude per cell is normalized away).  Empty cells
    // (no gradient) stay at zero.
    for (int c = 0; c < SENTAI_GIST_CELLS; ++c) {
        float s2 = 0.f;
        for (int o = 0; o < SENTAI_GIST_ORIENT; ++o) {
            float v = out[o * SENTAI_GIST_CELLS + c];
            s2 += v * v;
        }
        float n = sqrtf(s2);
        if (n < 1e-6f) continue;
        float inv = 1.f / n;
        for (int o = 0; o < SENTAI_GIST_ORIENT; ++o) {
            out[o * SENTAI_GIST_CELLS + c] *= inv;
        }
    }

    return 0;
}

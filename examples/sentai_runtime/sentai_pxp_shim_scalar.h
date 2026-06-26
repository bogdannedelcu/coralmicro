// sentai_pxp_shim_scalar.h -- internal scalar fallback for PXP shims.
//
// This helper is shared by SIM and ARM_EMU so semantic fixes land in one
// place. It is deliberately header-only because the physical board already
// defines the public PXP symbols in sentai_runtime.cc.

#ifndef SENTAI_PXP_SHIM_SCALAR_H_
#define SENTAI_PXP_SHIM_SCALAR_H_

#include "sentai_pxp_shim.h"

#include <stdint.h>

static inline int sentai_pxp_scalar_transform(
        const uint8_t* src, int src_w, int src_h,
        sentai_pxp_format_t src_format,
        uint8_t* dst, int dst_w, int dst_h,
        sentai_pxp_format_t dst_format) {
    if (!src || !dst || src_w <= 0 || src_h <= 0 ||
            dst_w <= 0 || dst_h <= 0) {
        return -1;
    }
    if (dst_w > src_w || dst_h > src_h) {
        return -2;
    }
    if (src_format != SENTAI_PXP_FORMAT_XRGB8888 &&
            src_format != SENTAI_PXP_FORMAT_RGB888) {
        return -3;
    }
    if (dst_format != SENTAI_PXP_FORMAT_RGB888 &&
            dst_format != SENTAI_PXP_FORMAT_Y8) {
        return -4;
    }

    const int src_bpp = (src_format == SENTAI_PXP_FORMAT_RGB888) ? 3 : 4;
    for (int oy = 0; oy < dst_h; ++oy) {
        const int y0 = (oy * src_h) / dst_h;
        const int y1 = ((oy + 1) * src_h) / dst_h;
        const int yh = (y1 > y0) ? (y1 - y0) : 1;
        for (int ox = 0; ox < dst_w; ++ox) {
            const int x0 = (ox * src_w) / dst_w;
            const int x1 = ((ox + 1) * src_w) / dst_w;
            const int xw = (x1 > x0) ? (x1 - x0) : 1;
            const int n = xw * yh;
            uint32_t sum_r = 0;
            uint32_t sum_g = 0;
            uint32_t sum_b = 0;
            uint32_t sum_y = 0;

            for (int y = y0; y < y1; ++y) {
                const uint8_t* srow = src + y * src_w * src_bpp +
                                      x0 * src_bpp;
                for (int x = 0; x < xw; ++x) {
                    uint8_t r, g, b;
                    if (src_format == SENTAI_PXP_FORMAT_RGB888) {
                        r = srow[x * 3 + 0];
                        g = srow[x * 3 + 1];
                        b = srow[x * 3 + 2];
                    } else {
                        b = srow[x * 4 + 0];
                        g = srow[x * 4 + 1];
                        r = srow[x * 4 + 2];
                    }
                    sum_r += r;
                    sum_g += g;
                    sum_b += b;
                    sum_y += (uint32_t)((77u * r + 150u * g + 29u * b) >> 8);
                }
            }

            const uint32_t half = (uint32_t)n / 2u;
            if (dst_format == SENTAI_PXP_FORMAT_Y8) {
                dst[oy * dst_w + ox] =
                    (uint8_t)((sum_y + half) / (uint32_t)n);
            } else {
                uint8_t* d = dst + (oy * dst_w + ox) * 3;
                d[0] = (uint8_t)((sum_r + half) / (uint32_t)n);
                d[1] = (uint8_t)((sum_g + half) / (uint32_t)n);
                d[2] = (uint8_t)((sum_b + half) / (uint32_t)n);
            }
        }
    }
    return 0;
}

#endif  // SENTAI_PXP_SHIM_SCALAR_H_

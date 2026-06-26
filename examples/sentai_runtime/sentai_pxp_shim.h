// sentai_pxp_shim.h -- shared declaration of the PXP transform boundary.
//
// ARM target: implementation lives in sentai_runtime.cc and wraps the
//             real PXP DMA hardware for native XRGB8888 camera frames.
// SIM target: implementation lives in sentai_pxp_shim_sim.c and is a
//             pure-C area-average resize on the host CPU.
// ARM_EMU:    implementation lives in sentai_pxp_shim_emu.c and may route
//             the same semantic transform to a Renode-side accelerator.
//
// Keep callers at this semantic boundary. Platform-specific acceleration,
// fallback, cache maintenance, and Renode MMIO details belong behind it.

#ifndef SENTAI_PXP_SHIM_H_
#define SENTAI_PXP_SHIM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENTAI_PXP_FORMAT_XRGB8888 = 0,
    SENTAI_PXP_FORMAT_RGB888 = 1,
    SENTAI_PXP_FORMAT_Y8 = 2,
} sentai_pxp_format_t;

int sentai_pxp_transform(const uint8_t* src, int src_w, int src_h,
                         sentai_pxp_format_t src_format,
                         uint8_t* dst, int dst_w, int dst_h,
                         sentai_pxp_format_t dst_format);

int sentai_pxp_scale(const uint8_t* src, int src_w, int src_h,
                      uint8_t* dst,       int dst_w, int dst_h);

int sentai_pxp_xrgb_to_y8(const uint8_t* src, int src_w, int src_h,
                          uint8_t* dst, int dst_w, int dst_h);

int sentai_pxp_rgb888_scale(const uint8_t* src, int src_w, int src_h,
                            uint8_t* dst, int dst_w, int dst_h);

int sentai_pxp_rgb888_to_y8(const uint8_t* src, int src_w, int src_h,
                            uint8_t* dst, int dst_w, int dst_h);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_PXP_SHIM_H_

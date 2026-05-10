// sentai_pxp_shim.h — shared declaration of the PXP downscale entry point.
//
// ARM target: implementation lives in sentai_runtime.cc and wraps the
//             real PXP DMA hardware (XRGB8888 -> RGB888P).
// SIM target: implementation lives in sentai_pxp_shim_sim.c and is a
//             pure-C area-average resize on the host CPU.  Compiled only
//             when SENTAI_PLATFORM_SIM is defined.
//
// Both implementations:
//   - take src in XRGB8888 (4 bytes/pixel, 32-bit pixel padded with 0xFF
//     in alpha channel — what the OV5640 CSI receiver produces)
//   - write dst in tightly-packed RGB888 (3 bytes/pixel, no padding)
//   - return 0 on success
//
// The only behavioural difference is timing (HW DMA ~1.15 ms vs CPU
// ~0.2 ms for 640x480 -> 80x60 on x86) and rounding (HW PXP uses fixed-
// point bilinear, SIM uses true area-average — within ±1 LSB).

#ifndef SENTAI_PXP_SHIM_H_
#define SENTAI_PXP_SHIM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int sentai_pxp_scale(const uint8_t* src, int src_w, int src_h,
                      uint8_t* dst,       int dst_w, int dst_h);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_PXP_SHIM_H_

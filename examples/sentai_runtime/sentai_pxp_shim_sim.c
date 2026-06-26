// sentai_pxp_shim_sim.c — SIM-only pure-C area-average resize.
//
// Drop-in replacement for the ARM PXP_SCALE block.  Same signature as
// sentai_pxp_scale() in sentai_runtime.cc; both translation units may
// not be linked together (this one is built only when SENTAI_PLATFORM_SIM
// is defined and selected in sim/CMakeLists.txt).
//
// Input semantics (must match the ARM CSI receiver):
//   - src is XRGB8888: 4 bytes/pixel, byte order [B, G, R, X]
//     (the OV5640 RGB888 mode + CSI BPP=4 produces little-endian 0xXXRRGGBB
//      stored as [BB GG RR XX] in memory).
//   - src is laid out without padding: pitch = src_w * 4.
//   - dst is RGB888P: 3 bytes/pixel, byte order [R, G, B], pitch = dst_w * 3.
//
// Algorithm: true area-average.  For each output pixel (ox, oy), compute
// the source rectangle [ox*sx .. (ox+1)*sx) x [oy*sy .. (oy+1)*sy) and
// average the source pixels covered.  Sub-pixel coverage at boundaries
// is approximated by integer rounding — the typical 640x480 -> 80x60
// case (8x downscale, integer ratio) is exact; non-integer ratios get
// a small bias that's bounded by 1 source-pixel weight.
//
// Speed: ~0.2 ms for 640x480 -> 80x60 single-thread on a Ryzen 7 5800X3D.
// That's well under the 33 ms frame budget at 30 fps and well under PXP's
// 1.15 ms on the RT1176 — so SIM is at least as fast as hardware here.
//
// embeded.md compliance (also enforced in SIM for symmetry):
//   - bounded loops: O(src_w * src_h)
//   - no dynamic alloc
//   - no float arithmetic in the hot path (integer accumulation only)

#include "sentai_pxp_shim.h"
#include "sentai_pxp_shim_scalar.h"

int sentai_pxp_transform(const uint8_t* src, int src_w, int src_h,
                         sentai_pxp_format_t src_format,
                         uint8_t* dst, int dst_w, int dst_h,
                         sentai_pxp_format_t dst_format) {
    return sentai_pxp_scalar_transform(src, src_w, src_h, src_format,
                                       dst, dst_w, dst_h, dst_format);
}

int sentai_pxp_scale(const uint8_t* src, int src_w, int src_h,
                      uint8_t* dst,       int dst_w, int dst_h) {
    return sentai_pxp_transform(src, src_w, src_h,
                                SENTAI_PXP_FORMAT_XRGB8888,
                                dst, dst_w, dst_h,
                                SENTAI_PXP_FORMAT_RGB888);
}

int sentai_pxp_xrgb_to_y8(const uint8_t* src, int src_w, int src_h,
                          uint8_t* dst, int dst_w, int dst_h) {
    return sentai_pxp_transform(src, src_w, src_h,
                                SENTAI_PXP_FORMAT_XRGB8888,
                                dst, dst_w, dst_h,
                                SENTAI_PXP_FORMAT_Y8);
}

int sentai_pxp_rgb888_scale(const uint8_t* src, int src_w, int src_h,
                            uint8_t* dst, int dst_w, int dst_h) {
    return sentai_pxp_transform(src, src_w, src_h,
                                SENTAI_PXP_FORMAT_RGB888,
                                dst, dst_w, dst_h,
                                SENTAI_PXP_FORMAT_RGB888);
}

int sentai_pxp_rgb888_to_y8(const uint8_t* src, int src_w, int src_h,
                            uint8_t* dst, int dst_w, int dst_h) {
    return sentai_pxp_transform(src, src_w, src_h,
                                SENTAI_PXP_FORMAT_RGB888,
                                dst, dst_w, dst_h,
                                SENTAI_PXP_FORMAT_Y8);
}

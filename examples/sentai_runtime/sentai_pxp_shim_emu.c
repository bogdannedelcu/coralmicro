// sentai_pxp_shim_emu.c -- ARM-emulator PXP semantic bridge.
//
// Same policy as the emulator TPU SendParameters bridge: the guest keeps a
// narrow production-like semantic API and Renode supplies an emulator-only
// accelerator behind an explicit MMIO request contract. If the bridge rejects
// a transform, the guest falls back to the shared scalar implementation.

#include "sentai_pxp_shim.h"
#include "sentai_pxp_shim_scalar.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <stddef.h>

#define SENTAI_EMU_PXP_BASE 0x40902C00u

enum {
    PXP_STATUS_IDLE = 0,
    PXP_STATUS_PENDING = 1,
    PXP_STATUS_DONE = 2,
    PXP_STATUS_ERROR = 3,
};

enum {
    PXP_CMD_TRANSFORM = 1,
};

enum {
    PXP_REG_STATUS = 0x00,
    PXP_REG_COMMAND = 0x04,
    PXP_REG_SRC_PTR = 0x08,
    PXP_REG_SRC_W = 0x0C,
    PXP_REG_SRC_H = 0x10,
    PXP_REG_SRC_FMT = 0x14,
    PXP_REG_DST_PTR = 0x18,
    PXP_REG_DST_W = 0x1C,
    PXP_REG_DST_H = 0x20,
    PXP_REG_DST_FMT = 0x24,
    PXP_REG_RESULT = 0x28,
    PXP_REG_SEQ = 0x2C,
};

volatile uint32_t g_sentai_emu_pxp_accel_calls = 0;
volatile uint32_t g_sentai_emu_pxp_accel_ok = 0;
volatile uint32_t g_sentai_emu_pxp_accel_fallback = 0;
volatile uint32_t g_sentai_emu_pxp_accel_last_rc = 0;

static volatile uint32_t* pxp_reg(uint32_t offset) {
    return (volatile uint32_t*)(uintptr_t)(SENTAI_EMU_PXP_BASE + offset);
}

static int pxp_bridge_request(const uint8_t* src, int src_w, int src_h,
                              sentai_pxp_format_t src_format,
                              uint8_t* dst, int dst_w, int dst_h,
                              sentai_pxp_format_t dst_format) {
    if (!src || !dst || src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
        return -1;
    }

    ++g_sentai_emu_pxp_accel_calls;
    const uint32_t seq = *pxp_reg(PXP_REG_SEQ) + 1u;
    *pxp_reg(PXP_REG_SRC_PTR) = (uint32_t)(uintptr_t)src;
    *pxp_reg(PXP_REG_SRC_W) = (uint32_t)src_w;
    *pxp_reg(PXP_REG_SRC_H) = (uint32_t)src_h;
    *pxp_reg(PXP_REG_SRC_FMT) = (uint32_t)src_format;
    *pxp_reg(PXP_REG_DST_PTR) = (uint32_t)(uintptr_t)dst;
    *pxp_reg(PXP_REG_DST_W) = (uint32_t)dst_w;
    *pxp_reg(PXP_REG_DST_H) = (uint32_t)dst_h;
    *pxp_reg(PXP_REG_DST_FMT) = (uint32_t)dst_format;
    *pxp_reg(PXP_REG_RESULT) = 0xFFFFFFFFu;
    *pxp_reg(PXP_REG_COMMAND) = PXP_CMD_TRANSFORM;
    *pxp_reg(PXP_REG_SEQ) = seq;
    *pxp_reg(PXP_REG_STATUS) = PXP_STATUS_PENDING;

    for (int i = 0; i < 20; ++i) {
        const uint32_t status = *pxp_reg(PXP_REG_STATUS);
        if (status == PXP_STATUS_DONE && *pxp_reg(PXP_REG_SEQ) == seq) {
            const uint32_t rc = *pxp_reg(PXP_REG_RESULT);
            g_sentai_emu_pxp_accel_last_rc = rc;
            if (rc == 0u) {
                ++g_sentai_emu_pxp_accel_ok;
                return 0;
            }
            ++g_sentai_emu_pxp_accel_fallback;
            return -100;
        }
        if (status == PXP_STATUS_ERROR) {
            g_sentai_emu_pxp_accel_last_rc = *pxp_reg(PXP_REG_RESULT);
            ++g_sentai_emu_pxp_accel_fallback;
            return -101;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    g_sentai_emu_pxp_accel_last_rc = 0xFFFFFF00u;
    ++g_sentai_emu_pxp_accel_fallback;
    return -102;
}

int sentai_pxp_transform(const uint8_t* src, int src_w, int src_h,
                         sentai_pxp_format_t src_format,
                         uint8_t* dst, int dst_w, int dst_h,
                         sentai_pxp_format_t dst_format) {
    if (pxp_bridge_request(src, src_w, src_h, src_format,
                           dst, dst_w, dst_h, dst_format) == 0) {
        return 0;
    }
    return sentai_pxp_scalar_transform(src, src_w, src_h, src_format,
                                       dst, dst_w, dst_h, dst_format);
}

int sentai_pxp_scale(const uint8_t* src, int src_w, int src_h,
                     uint8_t* dst, int dst_w, int dst_h) {
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

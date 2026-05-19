// m4_aruco_bench.cc — Cortex-M4 worker for the ArUco-threshold cycle
// bench (OP-S10-W16-T3).
//
// Architecture (per the canonical examples/multi_core_ipc pattern):
//   - Entry point is `app_main(void* param)`; the SDK + FreeRTOS
//     startup in libs_base-m4_freertos handles SystemInit, MPU,
//     SysTick + scheduler bring-up.
//   - We register an IpcM4 app-message handler.  On `kBenchGo` the
//     handler synthesises a deterministic 80×60 grayscale frame in
//     M4-local memory (LFSR pattern + central dark square — same
//     shape as sentai_aruco_adaptive_threshold_verify on the M7
//     side, just smaller), runs `aruco_adaptive_threshold` once
//     with DWT cycle counting, and sends back `kBenchDone` with
//     the cycle delta.
//   - All buffers live in M4-local memory (default `.bss` → m_data
//     per the M4 linker script).  No SDRAM access from M4 during
//     the inner loop → zero SEMC contention with M7.
//
// Constraints respected (per OP-S10-W16 scoping):
//   - NO custom mailbox at hardcoded addresses (RT1176 OCRAM
//     aliases between cores differ; rely on the SDK's RPMSG
//     section attributes + IpcM4 framework instead).
//   - NO PXP, no camera, no FxUser — pure compute.

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "libs/base/ipc_m4.h"
#include "examples/sentai_runtime/m4_bench_message.h"

// ─────────────────────────────────────────────────────────────────
// Threshold buffers — entirely M4-local.
// Frame: 80×60 = 4800 pixels.  Integral image (81×61)·4 = 19 764 B.
// Output binary: 4800 B.  Total ≈ 28 KB — fits comfortably in
// M4 m_data (128 KB).
// ─────────────────────────────────────────────────────────────────
#define M4_FRAME_W  80
#define M4_FRAME_H  60

static uint8_t  s_gray   [M4_FRAME_W * M4_FRAME_H]                  __attribute__((aligned(32)));
static uint8_t  s_binary [M4_FRAME_W * M4_FRAME_H]                  __attribute__((aligned(32)));
static int32_t  s_integral[(M4_FRAME_W + 1) * (M4_FRAME_H + 1)]     __attribute__((aligned(32)));

#define ARUCO_THRESH_C  7

// DWT cycle counter — same MMIO on M4 as M7 (architectural Cortex-M).
static inline uint32_t dwt_cyc(void) {
    return *((volatile uint32_t*)0xE0001004u);
}
static void dwt_init(void) {
    *((volatile uint32_t*)0xE000EDFCu) |= (1u << 24);   // DEMCR.TRCENA
    *((volatile uint32_t*)0xE0001000u) |= 1u;           // DWT.CTRL.CYCCNTENA
}

// Deterministic synth frame: gradient + LFSR noise + dark square.
// Same algebraic shape as sentai_aruco_adaptive_threshold_verify on M7
// (only the resolution differs), so qualitatively comparable.
static void synth_frame_(void) {
    for (int y = 0; y < M4_FRAME_H; ++y) {
        for (int x = 0; x < M4_FRAME_W; ++x) {
            uint32_t lfsr = (uint32_t)(y * M4_FRAME_W + x) * 2654435761u;
            uint8_t noise = (lfsr >> 16) & 0x1F;
            int v = 180 + (x * 40) / M4_FRAME_W + (int)noise - 8;
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            s_gray[y * M4_FRAME_W + x] = (uint8_t)v;
        }
    }
    const int cx = M4_FRAME_W / 2, cy = M4_FRAME_H / 2;
    const int half = 20;
    for (int y = cy - half; y < cy + half; ++y) {
        for (int x = cx - half; x < cx + half; ++x) {
            if (x >= 0 && x < M4_FRAME_W && y >= 0 && y < M4_FRAME_H) {
                s_gray[y * M4_FRAME_W + x] = 30;
            }
        }
    }
}

// Bradley adaptive threshold — verbatim algorithm from sentai_aruco.cc
// (the scalar reference path).  Math IDENTICAL to the production
// M7 kernel; this run produces the M4 cycle count for the same
// algorithm.  Kept as inline-compiled C (no DSP intrinsics here —
// raw-C baseline first, optional CMSIS opt later if we choose to
// keep ArUco on M4 permanently).
static void aruco_adaptive_threshold_m4(int block) {
    const int W = M4_FRAME_W, H = M4_FRAME_H;
    const int stride_i = W + 1;
    for (int x = 0; x <= W; ++x) s_integral[x] = 0;
    for (int y = 1; y <= H; ++y) {
        int32_t row_sum = 0;
        s_integral[y * stride_i] = 0;
        for (int x = 1; x <= W; ++x) {
            row_sum += s_gray[(x-1) + (y-1) * W];
            s_integral[x + y * stride_i] = s_integral[x + (y-1) * stride_i]
                                           + row_sum;
        }
    }
    const int half = block / 2;
    for (int y = 0; y < H; ++y) {
        int y0 = y - half;        if (y0 < 0) y0 = 0;
        int y1 = y + half;        if (y1 >= H) y1 = H - 1;
        for (int x = 0; x < W; ++x) {
            int x0 = x - half;    if (x0 < 0) x0 = 0;
            int x1 = x + half;    if (x1 >= W) x1 = W - 1;
            const int32_t A = s_integral[(x1+1) + (y1+1) * stride_i];
            const int32_t B = s_integral[(x0)   + (y1+1) * stride_i];
            const int32_t C = s_integral[(x1+1) + (y0)   * stride_i];
            const int32_t D = s_integral[(x0)   + (y0)   * stride_i];
            const int32_t box_sum = A - B - C + D;
            const int32_t box_area = (x1 - x0 + 1) * (y1 - y0 + 1);
            const int32_t mean = box_sum / box_area;
            const uint8_t v = s_gray[x + y * W];
            s_binary[x + y * W] = ((int32_t)v < mean - ARUCO_THRESH_C)
                                    ? 1u : 0u;
        }
    }
}

// IpcM4 message handler — runs in the IpcM4 RX task context, with
// the FreeRTOS scheduler active (SysTick alive).  When a kBenchGo
// arrives, runs one threshold pass and answers with kBenchDone.
static void handle_m7_message_(const uint8_t data[coralmicro::kIpcMessageBufferDataSize]) {
    const auto* msg = reinterpret_cast<const M4BenchAppMessage*>(data);
    if (msg->type != M4BenchMessageType::kBenchGo) return;
    int block = (int)msg->block;
    if (block < 3) block = 3;
    if (block > 511) block = 511;

    synth_frame_();
    const uint32_t t0 = dwt_cyc();
    aruco_adaptive_threshold_m4(block);
    const uint32_t t1 = dwt_cyc();

    coralmicro::IpcMessage ack{};
    ack.type = coralmicro::IpcMessageType::kApp;
    auto* app = reinterpret_cast<M4BenchAppMessage*>(&ack.message.data);
    app->type   = M4BenchMessageType::kBenchDone;
    app->block  = msg->block;
    app->cycles = t1 - t0;
    app->n_dets = 0;     // not running PnP at this stage
    coralmicro::IpcM4::GetSingleton()->SendMessage(ack);
}

extern "C" void app_main(void* param) {
    (void)param;
    dwt_init();
    coralmicro::IpcM4::GetSingleton()->RegisterAppMessageHandler(
        handle_m7_message_);
    // Idle forever — the IpcM4 RX task handles all work in callbacks.
    vTaskSuspend(nullptr);
}

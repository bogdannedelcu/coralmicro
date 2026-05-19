// m4_aruco_bench.cc — Cortex-M4 worker for the ArUco-threshold cycle
// bench (OP-S10-W16-T3 v3).
//
// Architecture (NASA/JPL embedded discipline per agent/embeded.md):
//   - Entry: `app_main(void* param)`.  SDK + FreeRTOS startup in
//     libs_base-m4_freertos handles SystemInit, MPU, SysTick +
//     scheduler bring-up.
//   - IPC callback is EVENT-CAPTURE ONLY: it stores the requested
//     block size + gives a binary semaphore.  No compute inside the
//     callback (the v2 attempt did ~1 ms of threshold compute from
//     callback context, which is suspected to have starved the
//     IpcM4 RX-task message-buffer service).
//   - A dedicated FreeRTOS "bench worker" task blocks on the
//     semaphore.  When kicked, it synthesises a deterministic 80×60
//     gray frame in M4-local memory, runs the threshold with DWT
//     cycle counting, sends back the kBenchDone message via
//     IpcM4::SendMessage from clean task context (no ISR / RX
//     constraint).
//   - All buffers in M4-local m_data.  No SDRAM access on the hot
//     path → zero SEMC contention with M7.

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "libs/base/ipc_m4.h"
#include "examples/sentai_runtime/m4_bench_message.h"

// M4F has the same ARM DSP-extension instruction set as M7 (USUB8, SEL,
// UQADD8 etc).  Inline-asm wrappers — verbatim from the sentai M7
// production path in sentai_aruco.cc — let us measure the SIMD upper
// bound on the same algorithm on the M4 core.
#ifdef __arm__
__attribute__((always_inline)) static inline uint32_t m4_usub8(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm volatile ("usub8 %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
__attribute__((always_inline)) static inline uint32_t m4_sel(uint32_t a, uint32_t b) {
    uint32_t r;
    __asm volatile ("sel %0, %1, %2" : "=r"(r) : "r"(a), "r"(b));
    return r;
}
#endif

// ─────────────────────────────────────────────────────────────────
// Threshold buffers — entirely M4-local but split across regions:
//   - s_gray, s_binary, s_integral → m_ocram (.ocram_bss section).
//     Total at 320×240 = 76 + 76 + 309 = 461 KB out of 504 KB
//     m_ocram, ~43 KB margin.
//   - Stack, task TCBs, mailbox, work-sem buf stay in m_data
//     (128 KB) where the default .bss lands.
// Production frame size; identical math to M7 SafetyArucoBaseline.
// ─────────────────────────────────────────────────────────────────
#define M4_FRAME_W  320
#define M4_FRAME_H  240

static uint8_t  s_gray   [M4_FRAME_W * M4_FRAME_H]                  __attribute__((section(".ocram_bss"), aligned(32)));
static uint8_t  s_binary [M4_FRAME_W * M4_FRAME_H]                  __attribute__((section(".ocram_bss"), aligned(32)));
static int32_t  s_integral[(M4_FRAME_W + 1) * (M4_FRAME_H + 1)]     __attribute__((section(".ocram_bss"), aligned(32)));

#define ARUCO_THRESH_C  7

// ─────────────────────────────────────────────────────────────────
// Worker task wakeup signal + pending request slot.
// `s_pending_block` is written by the IPC callback, read by the
// worker task.  Single-producer / single-consumer — no mutex
// needed; the binary semaphore provides ordering.
// ─────────────────────────────────────────────────────────────────
static StaticSemaphore_t s_work_sem_buf;
static SemaphoreHandle_t s_work_sem = nullptr;
static volatile uint16_t s_pending_block = 0;

static constexpr size_t kBenchTaskStackWords = 1024;
static StackType_t  s_bench_task_stack[kBenchTaskStackWords];
static StaticTask_t s_bench_task_tcb;

// ─────────────────────────────────────────────────────────────────
// DWT cycle counter on M4 (architectural Cortex-M MMIO).
// ─────────────────────────────────────────────────────────────────
static inline uint32_t dwt_cyc(void) {
    return *((volatile uint32_t*)0xE0001004u);
}
static void dwt_init(void) {
    *((volatile uint32_t*)0xE000EDFCu) |= (1u << 24);   // DEMCR.TRCENA
    *((volatile uint32_t*)0xE0001000u) |= 1u;           // DWT.CTRL.CYCCNTENA
}

// Deterministic synth frame: gradient + LFSR noise + central dark
// square.  Algebraically identical shape to the M7 verify pattern.
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

// Bradley adaptive threshold — SIMD path mirrors M7 production
// (sentai_aruco.cc commit 35b18c9d T18-T+U).  Math IDENTICAL.
// Phase 2 splits interior (4-wide __USUB8/__SEL compare,
// divide-eliminated) from border (scalar with x clamping).
static void aruco_adaptive_threshold_m4(int block) {
    const int W = M4_FRAME_W, H = M4_FRAME_H;
    const int stride_i = W + 1;
    // Phase 1 — serial prefix sum integral image.
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
    const int x_int_start = (half <= W - 1) ? half : W;
    const int x_int_end   = (W - half - 1 >= 0) ? (W - half - 1) : -1;
    const uint32_t C_plus_1_pack = (uint32_t)(ARUCO_THRESH_C + 1) * 0x01010101u;
    const uint32_t ones_pack     = 0x01010101u;
    const uint32_t zeros_pack    = 0x00000000u;
    for (int y = 0; y < H; ++y) {
        int y0 = y - half; if (y0 < 0) y0 = 0;
        int y1 = y + half; if (y1 >= H) y1 = H - 1;
        const int32_t y_factor = y1 - y0 + 1;
        const int32_t* int_top = s_integral + (y0)     * stride_i;
        const int32_t* int_bot = s_integral + (y1 + 1) * stride_i;
        const uint8_t* gray_row = s_gray + y * W;
        uint8_t*       bin_row  = s_binary + y * W;
        // Left border (scalar, x clamped).
        for (int x = 0; x < x_int_start; ++x) {
            int x0 = x - half; if (x0 < 0) x0 = 0;
            int x1 = x + half; if (x1 >= W) x1 = W - 1;
            const int32_t bs = int_bot[x1 + 1] - int_bot[x0]
                              - int_top[x1 + 1] + int_top[x0];
            const int32_t ba = (x1 - x0 + 1) * y_factor;
            const int32_t mean = bs / ba;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C)
                          ? 1u : 0u;
        }
        // Interior — 4-wide SIMD compare on M4F DSP extensions.
        const int32_t box_area_int = (int32_t)block * y_factor;
        int x = x_int_start;
        for (; x + 4 <= x_int_end + 1; x += 4) {
            const int32_t bs0 = int_bot[x + 0 + half + 1] - int_bot[x + 0 - half]
                              - int_top[x + 0 + half + 1] + int_top[x + 0 - half];
            const int32_t bs1 = int_bot[x + 1 + half + 1] - int_bot[x + 1 - half]
                              - int_top[x + 1 + half + 1] + int_top[x + 1 - half];
            const int32_t bs2 = int_bot[x + 2 + half + 1] - int_bot[x + 2 - half]
                              - int_top[x + 2 + half + 1] + int_top[x + 2 - half];
            const int32_t bs3 = int_bot[x + 3 + half + 1] - int_bot[x + 3 - half]
                              - int_top[x + 3 + half + 1] + int_top[x + 3 - half];
            const uint32_t m0 = (uint32_t)(bs0 / box_area_int) & 0xFFu;
            const uint32_t m1 = (uint32_t)(bs1 / box_area_int) & 0xFFu;
            const uint32_t m2 = (uint32_t)(bs2 / box_area_int) & 0xFFu;
            const uint32_t m3 = (uint32_t)(bs3 / box_area_int) & 0xFFu;
            const uint32_t m_pack = m0 | (m1 << 8) | (m2 << 16) | (m3 << 24);
            const uint32_t g_pack = *(const uint32_t*)(gray_row + x);
            const uint32_t raw      = m4_usub8(m_pack, g_pack);
            const uint32_t clamped  = m4_sel(raw, zeros_pack);
            (void)              m4_usub8(clamped, C_plus_1_pack);
            const uint32_t bin_pack = m4_sel(ones_pack, zeros_pack);
            *(uint32_t*)(bin_row + x) = bin_pack;
        }
        // Interior tail (1-3 leftover pixels).
        for (; x <= x_int_end; ++x) {
            const int32_t bs = int_bot[x + half + 1] - int_bot[x - half]
                              - int_top[x + half + 1] + int_top[x - half];
            const int32_t mean = bs / box_area_int;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C)
                          ? 1u : 0u;
        }
        // Right border (scalar, x1 clamped).
        for (int x = x_int_end + 1; x < W; ++x) {
            int x0 = x - half; if (x0 < 0) x0 = 0;
            int x1 = x + half; if (x1 >= W) x1 = W - 1;
            const int32_t bs = int_bot[x1 + 1] - int_bot[x0]
                              - int_top[x1 + 1] + int_top[x0];
            const int32_t ba = (x1 - x0 + 1) * y_factor;
            const int32_t mean = bs / ba;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C)
                          ? 1u : 0u;
        }
    }
}

// IpcM4 message handler.
//
// HELLO-WORLD MODE (operator request 2026-05-19 "fa un hello world
// pt M4 sa verifici ca poti vorbi cu el din M7 REPL"): if the
// requested block is the sentinel 0xCAFE, reply IMMEDIATELY from
// the RX callback context (no semaphore, no worker task) — exactly
// the multi_core_ipc_m4.cc pattern.  Mostly to bisect whether the
// reply path itself works at all.
//
// Otherwise: event-capture only (give work-task semaphore).
static void handle_m7_message_(const uint8_t data[coralmicro::kIpcMessageBufferDataSize]) {
    const auto* msg = reinterpret_cast<const M4BenchAppMessage*>(data);
    if (msg->type != M4BenchMessageType::kBenchGo) return;
    uint16_t block = msg->block;

    if (block == 0xCAFEu) {
        // Hello-world: reply right here from RX context.
        coralmicro::IpcMessage ack{};
        ack.type = coralmicro::IpcMessageType::kApp;
        auto* app = reinterpret_cast<M4BenchAppMessage*>(&ack.message.data);
        app->type   = M4BenchMessageType::kBenchDone;
        app->block  = 0xCAFEu;
        app->cycles = 0xCAFEBABEu;
        app->n_dets = 0;
        coralmicro::IpcM4::GetSingleton()->SendMessage(ack);
        return;
    }

    if (block < 3)   block = 3;
    if (block > 511) block = 511;
    s_pending_block = block;
    if (s_work_sem) {
        xSemaphoreGive(s_work_sem);
    }
}

// Bench worker task.  Blocks on the semaphore; on take, snapshots
// the pending block, runs the threshold, sends back kBenchDone.
//
// DIAGNOSTIC v3.2 — bisect IPC-reply vs compute-crash:
//   - If block == 0xCAFE (special sentinel from M7): skip compute,
//     reply with cycles = 0xCAFEBABE immediately.  Confirms the
//     IPC reply path works in isolation.
//   - Otherwise: normal compute + DWT timing.
[[noreturn]] static void bench_worker_(void* arg) {
    (void)arg;
    while (true) {
        if (xSemaphoreTake(s_work_sem, portMAX_DELAY) != pdTRUE) continue;
        const uint16_t block = s_pending_block;

        coralmicro::IpcMessage ack{};
        ack.type = coralmicro::IpcMessageType::kApp;
        auto* app = reinterpret_cast<M4BenchAppMessage*>(&ack.message.data);
        app->type   = M4BenchMessageType::kBenchDone;
        app->block  = block;
        app->n_dets = 0;

        if (block == 0xCAFEu) {
            // IPC-only smoke test — no compute.
            app->cycles = 0xCAFEBABEu;
        } else {
            int b = (int)block;
            if (b < 3)   b = 3;
            if (b > 511) b = 511;
            synth_frame_();
            const uint32_t t0 = dwt_cyc();
            aruco_adaptive_threshold_m4(b);
            const uint32_t t1 = dwt_cyc();
            app->cycles = t1 - t0;
        }
        coralmicro::IpcM4::GetSingleton()->SendMessage(ack);
    }
}

extern "C" void app_main(void* param) {
    (void)param;
    dwt_init();

    // Create the worker semaphore + task BEFORE registering the RX
    // handler, so any inbound kBenchGo finds the worker ready.
    s_work_sem = xSemaphoreCreateBinaryStatic(&s_work_sem_buf);
    configASSERT(s_work_sem);

    auto handle = xTaskCreateStatic(bench_worker_, "m4_bench", kBenchTaskStackWords,
                                     nullptr,
                                     tskIDLE_PRIORITY + 3,
                                     s_bench_task_stack, &s_bench_task_tcb);
    configASSERT(handle);

    coralmicro::IpcM4::GetSingleton()->RegisterAppMessageHandler(
        handle_m7_message_);

    // app_main is itself a task; we have nothing else to do here.
    vTaskSuspend(nullptr);
}

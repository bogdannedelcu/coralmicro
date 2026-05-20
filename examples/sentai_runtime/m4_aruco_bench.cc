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
#include "examples/sentai_runtime/flow_bench_shared.h"

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
// USADA8: 4-byte sum-of-abs-differences + accumulate (DSP-extension
// SIMD on Cortex-M4F, same as M7).  arm_acle.h doesn't ship __USADA8
// on the toolchain we use, so inline-asm wrapper.
__attribute__((always_inline)) static inline uint32_t m4_usada8(uint32_t a, uint32_t b, uint32_t acc) {
    uint32_t r;
    __asm volatile ("usada8 %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}
#endif

/* Unaligned LDR helper — same packed-struct trick as M7 flow_task.cc:57.
 * Cortex-M4 supports unaligned LDR transparently.  Without this the
 * inner SAD loop emits memcpy() calls and tanks USADA8's win (see
 * agent.md hard-rule). */
typedef struct { uint32_t v; } __attribute__((packed)) m4_u32u_t;
#define M4_LD32U(p) (((const m4_u32u_t*)(p))->v)

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

// OP-S10-W17-T2 M4 ablation — rolling-integral scratch (2.6 KB total,
// in OCRAM).  Mirrors the M7 sentai_aruco.cc OCRAM-resident rolling
// scratch.  Used by aruco_threshold_rolling_m4 for the WhyCon
// timing comparison.
static int32_t s_rolling_col_sum_m4 [M4_FRAME_W]     __attribute__((section(".ocram_bss"), aligned(32)));
static int32_t s_rolling_prefix_x_m4[M4_FRAME_W + 1] __attribute__((section(".ocram_bss"), aligned(32)));

// XOR sink — keeps s_binary store-side load-bearing so the M4
// LTO+O3 doesn't DCE the entire Phase-2 loop body.  Reads back
// the threshold result before reporting the cycle count.  The
// alternative was `volatile s_binary` but that forces per-byte
// store-byte without coalesce/SIMD, biasing the measurement
// pessimistically.
static volatile uint8_t s_bench_sink;

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

// OP-S10-W17-T2 M4 ablation — WhyCon synth + rolling-integral threshold.
// Mirrors the M7 path (whycon_synth_frame_ + aruco_adaptive_threshold_rolling)
// but stays plain scalar (M4F single-issue; SIMD-pack attempt earlier
// measured 2.5× slower than scalar — kept).  All buffers in OCRAM.
//
// Frame: N filled black disks on white background, layout matches the M7
// whycon_synth_frame_ pattern so results are directly comparable.
static void whycon_synth_frame_m4(int n_circles, int radius) {
    const int W = M4_FRAME_W, H = M4_FRAME_H;
    for (int i = 0; i < W * H; ++i) s_gray[i] = 220;  // white background
    if (n_circles <= 0) return;
    if (n_circles > 8) n_circles = 8;
    if (radius < 4) radius = 4;
    if (radius > 30) radius = 30;
    const int cols = (n_circles > 4) ? 4 : n_circles;
    const int rows = (n_circles + cols - 1) / cols;
    const int dx = W / (cols + 1);
    const int dy = H / (rows + 1);
    for (int i = 0; i < n_circles; ++i) {
        const int col = i % cols;
        const int row = i / cols;
        const int cx = (col + 1) * dx + (i * 7) % 5;
        const int cy = (row + 1) * dy + (i * 13) % 5;
        for (int y = cy - radius; y <= cy + radius; ++y) {
            if (y < 0 || y >= H) continue;
            for (int x = cx - radius; x <= cx + radius; ++x) {
                if (x < 0 || x >= W) continue;
                const int dxp = x - cx;
                const int dyp = y - cy;
                if (dxp * dxp + dyp * dyp <= radius * radius) {
                    s_gray[x + y * W] = 20;
                }
            }
        }
    }
}

static void aruco_threshold_rolling_m4(int block) {
    const int W = M4_FRAME_W, H = M4_FRAME_H;
    const int half = block / 2;
    int y_top = 0;
    int y_bot = (half < H - 1) ? half : H - 1;
    for (int x = 0; x < W; ++x) s_rolling_col_sum_m4[x] = 0;
    for (int yy = y_top; yy <= y_bot; ++yy) {
        const uint8_t* row = s_gray + yy * W;
        for (int x = 0; x < W; ++x) s_rolling_col_sum_m4[x] += row[x];
    }
    for (int y = 0; y < H; ++y) {
        const int y_top_new = (y - half >= 0) ? y - half : 0;
        const int y_bot_new = (y + half < H)  ? y + half : H - 1;
        while (y_bot < y_bot_new) {
            ++y_bot;
            const uint8_t* row = s_gray + y_bot * W;
            for (int x = 0; x < W; ++x) s_rolling_col_sum_m4[x] += row[x];
        }
        while (y_top < y_top_new) {
            const uint8_t* row = s_gray + y_top * W;
            for (int x = 0; x < W; ++x) s_rolling_col_sum_m4[x] -= row[x];
            ++y_top;
        }
        const int32_t box_h = y_bot - y_top + 1;
        s_rolling_prefix_x_m4[0] = 0;
        int32_t acc = 0;
        for (int x = 0; x < W; ++x) {
            acc += s_rolling_col_sum_m4[x];
            s_rolling_prefix_x_m4[x + 1] = acc;
        }
        const uint8_t* gray_row = s_gray + y * W;
        uint8_t* bin_row = s_binary + y * W;
        for (int x = 0; x < W; ++x) {
            int x0 = (x - half >= 0) ? x - half : 0;
            int x1 = (x + half < W)  ? x + half : W - 1;
            const int32_t bs = s_rolling_prefix_x_m4[x1 + 1] - s_rolling_prefix_x_m4[x0];
            const int32_t ba = (x1 - x0 + 1) * box_h;
            const int32_t mean = bs / ba;
            bin_row[x] = ((int32_t)gray_row[x] < mean - ARUCO_THRESH_C) ? 1u : 0u;
        }
    }
}

// Bradley adaptive threshold — plain scalar (M4F single-issue;
// the SIMD pack-overhead attempt in the earlier commit measured
// 24 ms vs 10 ms for plain scalar, so the SIMD path is reverted).
// Math IDENTICAL to the M7 scalar reference in sentai_aruco.cc.
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

// OP-S10-W17-T4 DTCM-variant — local M4 DTCM mirror buffers.
// Operator-requested "doar DTCM test, fara alte optimizari pe algoritm"
// 2026-05-20.  Same algorithm, same SIMD intrinsics, same loop body
// as aruco_flow_sad_m4 below — only the source pointers point to
// DTCM-resident buffers instead of OCRAM-shared ones.  Pre-copy
// OCRAM→DTCM is done OUTSIDE the timed region (operator instruction:
// "pre-copy nu intra in masuratoare").
//
// M4 DTCM = m_data section at 0x20220000+NCACHE, 1-cycle access (vs
// OCRAM 3-cycle).  Default .bss attribute on M4 lands here, no
// special section needed.
static uint8_t s_m4_flow_curr_dt[FLOW_BENCH_GRAY_PIX] __attribute__((aligned(4)));
static uint8_t s_m4_flow_prev_dt[FLOW_BENCH_GRAY_PIX] __attribute__((aligned(4)));

static uint32_t aruco_flow_sad_m4_dtcm(int* out_dx, int* out_dy) {
    const uint8_t* curr = s_m4_flow_curr_dt;
    const uint8_t* prev = s_m4_flow_prev_dt;
    const int W = FLOW_BENCH_GRAY_W;
    const int bx = (W - FLOW_BENCH_BLOCK_W) / 2;
    const int by = (FLOW_BENCH_GRAY_H - FLOW_BENCH_BLOCK_H) / 2;
    uint32_t best = 0xFFFFFFFFu;
    int bdx = 0, bdy = 0;
    for (int dy = -FLOW_BENCH_SEARCH; dy <= FLOW_BENCH_SEARCH; ++dy) {
        for (int dx = -FLOW_BENCH_SEARCH; dx <= FLOW_BENCH_SEARCH; ++dx) {
            uint32_t sad = 0;
            for (int y = 0; y < FLOW_BENCH_BLOCK_H; ++y) {
                const uint8_t* c = curr + (by + y) * W + bx;
                const uint8_t* p = prev + (by + y + dy) * W + (bx + dx);
                sad = m4_usada8(M4_LD32U(c +  0), M4_LD32U(p +  0), sad);
                sad = m4_usada8(M4_LD32U(c +  4), M4_LD32U(p +  4), sad);
                sad = m4_usada8(M4_LD32U(c +  8), M4_LD32U(p +  8), sad);
                sad = m4_usada8(M4_LD32U(c + 12), M4_LD32U(p + 12), sad);
                sad = m4_usada8(M4_LD32U(c + 16), M4_LD32U(p + 16), sad);
                sad = m4_usada8(M4_LD32U(c + 20), M4_LD32U(p + 20), sad);
                sad = m4_usada8(M4_LD32U(c + 24), M4_LD32U(p + 24), sad);
                sad = m4_usada8(M4_LD32U(c + 28), M4_LD32U(p + 28), sad);
            }
            if (sad < best) { best = sad; bdx = dx; bdy = dy; }
        }
    }
    *out_dx = bdx; *out_dy = bdy;
    return best;
}

// OP-S10-W17-T4 M4 ablation — Flow SAD inner loop on the SHARED
// .tpu_input OCRAM region (M4 reads same physical addresses M7 wrote).
// Algorithm IDENTICAL to flow_task.cc:sad_match — 25×25 search ×
// 32×32 block, USADA8+LD32U inner loop.
//
// Returns best_sad, dx, dy via the volatile sink to defeat DCE.
static volatile uint32_t s_m4_flow_sink = 0;

static uint32_t aruco_flow_sad_m4(int* out_dx, int* out_dy) {
    const uint8_t* curr = FLOW_BENCH_CURR_PTR;
    const uint8_t* prev = FLOW_BENCH_PREV_PTR;
    const int W = FLOW_BENCH_GRAY_W;
    const int bx = (W - FLOW_BENCH_BLOCK_W) / 2;
    const int by = (FLOW_BENCH_GRAY_H - FLOW_BENCH_BLOCK_H) / 2;
    uint32_t best = 0xFFFFFFFFu;
    int bdx = 0, bdy = 0;
    for (int dy = -FLOW_BENCH_SEARCH; dy <= FLOW_BENCH_SEARCH; ++dy) {
        for (int dx = -FLOW_BENCH_SEARCH; dx <= FLOW_BENCH_SEARCH; ++dx) {
            uint32_t sad = 0;
            for (int y = 0; y < FLOW_BENCH_BLOCK_H; ++y) {
                const uint8_t* c = curr + (by + y) * W + bx;
                const uint8_t* p = prev + (by + y + dy) * W + (bx + dx);
                sad = m4_usada8(M4_LD32U(c +  0), M4_LD32U(p +  0), sad);
                sad = m4_usada8(M4_LD32U(c +  4), M4_LD32U(p +  4), sad);
                sad = m4_usada8(M4_LD32U(c +  8), M4_LD32U(p +  8), sad);
                sad = m4_usada8(M4_LD32U(c + 12), M4_LD32U(p + 12), sad);
                sad = m4_usada8(M4_LD32U(c + 16), M4_LD32U(p + 16), sad);
                sad = m4_usada8(M4_LD32U(c + 20), M4_LD32U(p + 20), sad);
                sad = m4_usada8(M4_LD32U(c + 24), M4_LD32U(p + 24), sad);
                sad = m4_usada8(M4_LD32U(c + 28), M4_LD32U(p + 28), sad);
            }
            if (sad < best) { best = sad; bdx = dx; bdy = dy; }
        }
    }
    *out_dx = bdx; *out_dy = bdy;
    return best;
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

    // WhyCon sentinel range: 0xC100..0xC108 = synth(N disks) + rolling threshold.
    // Worker decodes N = (block & 0x000F).
    if (block >= 0xC100u && block <= 0xC108u) {
        s_pending_block = block;
        if (s_work_sem) xSemaphoreGive(s_work_sem);
        return;
    }
    // OP-S10-W17-T4 Flow SAD sentinels:
    //   0xF10F = M4 reads shared OCRAM directly (3-cyc/load)
    //   0xF1D7 = M4 pre-copies OCRAM → local DTCM, then SAD (1-cyc/load)
    if (block == 0xF10Fu || block == 0xF1D7u) {
        s_pending_block = block;
        if (s_work_sem) xSemaphoreGive(s_work_sem);
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
        } else if (block == 0xF10Fu) {
            // OP-S10-W17-T4: Flow SAD on shared OCRAM curr/prev.
            // M7 has already synth'd both into FLOW_BENCH_CURR/PREV_PTR
            // before dispatching this message — M4 just runs SAD.
            int dx_out = 0, dy_out = 0;
            const uint32_t t0 = dwt_cyc();
            const uint32_t best = aruco_flow_sad_m4(&dx_out, &dy_out);
            const uint32_t t1 = dwt_cyc();
            s_m4_flow_sink = best ^ (uint32_t)dx_out ^ ((uint32_t)dy_out << 16);
            s_bench_sink = (uint8_t)s_m4_flow_sink;
            app->cycles = t1 - t0;
            app->n_dets = 0;
        } else if (block == 0xF1D7u) {
            // OP-S10-W17-T4 DTCM variant: M4 pre-copies OCRAM shared
            // into DTCM local scratch (1-cyc access) BEFORE the timed
            // SAD region.  Pre-copy intentionally excluded from
            // measurement per operator instruction.
            memcpy(s_m4_flow_curr_dt, FLOW_BENCH_CURR_PTR, FLOW_BENCH_GRAY_PIX);
            memcpy(s_m4_flow_prev_dt, FLOW_BENCH_PREV_PTR, FLOW_BENCH_GRAY_PIX);
            int dx_out = 0, dy_out = 0;
            const uint32_t t0 = dwt_cyc();
            const uint32_t best = aruco_flow_sad_m4_dtcm(&dx_out, &dy_out);
            const uint32_t t1 = dwt_cyc();
            s_m4_flow_sink = best ^ (uint32_t)dx_out ^ ((uint32_t)dy_out << 16);
            s_bench_sink = (uint8_t)s_m4_flow_sink;
            app->cycles = t1 - t0;
            app->n_dets = 1;  /* DTCM variant flag */
        } else if (block >= 0xC100u && block <= 0xC108u) {
            // OP-S10-W17-T2 M4 WhyCon ablation — synth disks + rolling
            // threshold (same kernel as M7's optimized WhyCon Phase A).
            const int n_circles = (int)(block & 0xFu);
            whycon_synth_frame_m4(n_circles, 15);
            const uint32_t t0 = dwt_cyc();
            aruco_threshold_rolling_m4(31);   // block_size matches M7 WhyCon
            const uint32_t t1 = dwt_cyc();
            uint32_t fold = 0;
            for (int i = 0; i < M4_FRAME_W * M4_FRAME_H; i += 16) {
                fold ^= s_binary[i];
            }
            s_bench_sink = (uint8_t)fold;
            app->cycles = t1 - t0;
            app->n_dets = (uint16_t)n_circles;
        } else {
            int b = (int)block;
            if (b < 3)   b = 3;
            if (b > 511) b = 511;
            synth_frame_();
            const uint32_t t0 = dwt_cyc();
            aruco_adaptive_threshold_m4(b);
            // XOR-fold s_binary into the volatile sink so the
            // Phase-2 stores can't be DCE'd by LTO.  The fold
            // itself is outside the timed region — it's there
            // only to keep the stores load-bearing.
            const uint32_t t1 = dwt_cyc();
            uint32_t fold = 0;
            for (int i = 0; i < M4_FRAME_W * M4_FRAME_H; i += 16) {
                fold ^= s_binary[i];
            }
            s_bench_sink = (uint8_t)fold;
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

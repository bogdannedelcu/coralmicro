// m4_aruco_bench.cc — bare-metal M4 ArUco threshold cycle bench
// (OP-S10-W16-T2 / T3).
//
// Goal: measure how long `aruco_adaptive_threshold` takes on the
// Cortex-M4 @ 400 MHz, without changing the algorithm.  The bench
// runs as a polling main() on M4 — no FreeRTOS scheduler, no
// SysTick handler — so we sidestep the build #1130 freeze mode
// entirely and get a clean cycle count.
//
// Architecture (per operator constraint 2026-05-19: "lucram izolati
// de celalalt core, fara PXP"):
//   - All ArUco buffers live in M4-local fast memory (m_text /
//     m_ocram) so M4 never goes through SEMC during the inner loop.
//   - M7 writes a 320x240 synthetic gray frame into the shared
//     RPMSG window once at startup.
//   - M7 sets `s_m4_go = 1` to kick off one threshold pass.
//   - M4 runs Phase 1 + Phase 2, captures DWT cycles, writes
//     `s_m4_result_cyc`, sets `s_m4_done = 1`.
//   - M7 reads cycle count + reports.
//
// Bare-metal (no FreeRTOS): main() spins on go flag; no SysTick
// handler is required, eliminating the historical freeze risk.

#include <stdint.h>
#include <string.h>

#include "fsl_common.h"
#include "MIMXRT1176_cm4.h"

// MPU + MCMGR shims from the SDK (needed so the shared OCRAM
// window M7 reads is non-cacheable on M4's side).  Same calls as
// flow_task_m4.cc:285-299.
extern "C" void BOARD_ConfigMPU(void);
#include "fsl_mu.h"

// ─────────────────────────────────────────────────────────────────
// Shared M7↔M4 mailbox in the RPMSG region at 0x202C0000 (per the
// M4 linker script `rpmsg_sh_mem`).  Same physical OCRAM cell from
// both sides, but M4 sees the OCRAM2 alias starting at 0x202C0000
// while M7 sees it at 0x20340000 - RPMSG_SHMEM_SIZE.
// MUST be marked non-cacheable on both cores' MPU.
// Layout (16 bytes, all uint32_t):
//   [0] magic   = 0x4D34 ("M4" + "4")  set by M4 on boot
//   [1] go      = M7 writes 1 to start; M4 clears to 0
//   [2] done    = M4 writes 1 when result ready; M7 clears to 0
//   [3] result  = M4 writes DWT cycle delta here
// Followed by 320*240 bytes of gray frame at offset 16.
// ─────────────────────────────────────────────────────────────────
// Mailbox lives inside rpmsg_sh_mem (0x202C0000-0x202C4000, 16 KB),
// guaranteed non-cacheable on both M7 and M4 (the standard SDK MPU
// configures this region as device-memory).  IpcM7 itself uses ~400
// bytes near the start for its tx/rx message queues; we place our
// mailbox + frame at 0x202C2000 (8 KB into the region) to be safely
// past those.
//
// Frame size 80×60 (4800 bytes) keeps the bench payload inside the
// 16 KB rpmsg window with room to spare.  Throughput numbers scale
// predictably with pixel count; cross-core wiring soundness is the
// thing we measure first.
#define M4_MAILBOX_BASE   0x202C2000u
#define M4_MAILBOX_MAGIC  0x4D344D34u  // "M4M4"
#define M4_FRAME_W        80
#define M4_FRAME_H        60

static volatile uint32_t* const s_mbox  = (volatile uint32_t*)M4_MAILBOX_BASE;
static const    uint8_t*  const s_gray  = (const    uint8_t*)(M4_MAILBOX_BASE + 16);

// ─────────────────────────────────────────────────────────────────
// ArUco buffers — entirely M4-local, never reach SDRAM/SEMC.
// Placed in default .bss → m_ocram per the M4 linker script
// (384 KB, easily holds 309 KB integral + 76 KB binary).
// ─────────────────────────────────────────────────────────────────
static uint8_t  s_binary [M4_FRAME_W * M4_FRAME_H]                 __attribute__((aligned(32)));
static int32_t  s_integral[(M4_FRAME_W + 1) * (M4_FRAME_H + 1)]    __attribute__((aligned(32)));

#define ARUCO_THRESH_C  7

// ─────────────────────────────────────────────────────────────────
// DWT cycle counter on M4.  Same MMIO address as on M7
// (architectural Cortex-M).  Init = enable TRC + CYCCNTENA.
// ─────────────────────────────────────────────────────────────────
static inline uint32_t dwt_cyc(void) {
    return *((volatile uint32_t*)0xE0001004u);
}
static void dwt_init(void) {
    *((volatile uint32_t*)0xE000EDFCu) |= (1u << 24);   // DEMCR.TRCENA
    *((volatile uint32_t*)0xE0001000u) |= 1u;           // DWT.CTRL.CYCCNTENA
}

// ─────────────────────────────────────────────────────────────────
// Bradley adaptive threshold — verbatim from sentai_aruco.cc, the
// scalar reference path.  Math IDENTICAL to the production M7
// kernel; this run produces the M4 cycle count for the same
// algorithm.
// ─────────────────────────────────────────────────────────────────
static void aruco_adaptive_threshold_m4(const uint8_t* gray, int w, int h,
                                          int block) {
    const int W = w, H = h;
    const int stride_i = W + 1;
    for (int x = 0; x <= W; ++x) s_integral[x] = 0;
    for (int y = 1; y <= H; ++y) {
        int32_t row_sum = 0;
        s_integral[y * stride_i] = 0;
        for (int x = 1; x <= W; ++x) {
            row_sum += gray[(x-1) + (y-1) * W];
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
            const uint8_t v = gray[x + y * W];
            s_binary[x + y * W] = ((int32_t)v < mean - ARUCO_THRESH_C)
                                    ? 1u : 0u;
        }
    }
}

// ─────────────────────────────────────────────────────────────────
// M4 bare-metal entry.  SDK startup runs SystemInit then calls
// main(); we MUST NOT enable SysTick interrupt (the historical
// freeze mode).  No FreeRTOS scheduler — just poll the shared
// mailbox.
// ─────────────────────────────────────────────────────────────────
extern "C" int main(int argc, char** argv) {
    (void)argc; (void)argv;

    // MPU: mark the mailbox region non-cacheable so M7's writes
    // land in physical OCRAM cells M4 reads from, and vice versa.
    BOARD_ConfigMPU();

    // Disable SysTick — no scheduler, no need for periodic tick.
    // This avoids the build #1130 freeze mode entirely.
    SysTick->CTRL = 0;

    dwt_init();

    // Announce we're alive.  M7 polls magic before sending go.
    s_mbox[0] = M4_MAILBOX_MAGIC;
    s_mbox[1] = 0;        // go
    s_mbox[2] = 0;        // done
    s_mbox[3] = 0;        // result_cyc
    __DSB();

    while (1) {
        if (s_mbox[1] != 0) {
            const uint32_t block = s_mbox[1];   // M7 passes block size via go
            s_mbox[1] = 0;
            const uint32_t t0 = dwt_cyc();
            aruco_adaptive_threshold_m4(s_gray, M4_FRAME_W, M4_FRAME_H,
                                         (int)block);
            const uint32_t t1 = dwt_cyc();
            s_mbox[3] = t1 - t0;
            __DSB();
            s_mbox[2] = 1;
        }
        // Tight spin — DWT free-runs, no sleep needed.
        __asm__ volatile ("nop");
    }
    return 0;
}

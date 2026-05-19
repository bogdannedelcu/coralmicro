// m4_bench_host.cc — M7-side driver for the M4 ArUco-threshold bench
// (OP-S10-W16-T2/T3).
//
// Boots the M4 core via IpcM7::StartM4(), populates the shared mailbox
// at 0x20330000 with a synthetic 160×120 grayscale frame, kicks the
// "go" flag with a block-size parameter, polls for the M4-published
// "done" flag, returns the DWT cycle count.
//
// See m4_aruco_bench.cc for the mailbox layout.  Both sides agree on
// the absolute address 0x20330000 (shared OCRAM2, past the TPU
// staging tensor and past M4's linker-managed m_ocram region).

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "libs/base/ipc_m7.h"

// Mailbox addresses match m4_aruco_bench.cc:
#define M4_MAILBOX_BASE   0x202C2000u
#define M4_MAILBOX_MAGIC  0x4D344D34u  // "M4M4"
#define M4_FRAME_W        80
#define M4_FRAME_H        60

static volatile uint32_t* const s_mbox = (volatile uint32_t*)M4_MAILBOX_BASE;
static          uint8_t*  const s_gray = (uint8_t*)(M4_MAILBOX_BASE + 16);

// Did we already start the M4 core in this firmware boot?
static int s_m4_started = 0;

// Synth pattern: gradient 180..220 + LFSR noise + central dark square.
// Same shape as sentai_aruco_adaptive_threshold_verify on M7 so the
// two cycle counts measure the same algorithm on the same input.
static void synth_frame_(uint8_t* dst, int w, int h) {
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            uint32_t lfsr = (uint32_t)(y * w + x) * 2654435761u;
            uint8_t noise = (lfsr >> 16) & 0x1F;
            int v = 180 + (x * 40) / w + (int)noise - 8;
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            dst[y * w + x] = (uint8_t)v;
        }
    }
    // Central 40x40 dark square (proportional to the 80x80 we use at
    // 320×240 — keeps the marker-ish geometry roughly similar).
    const int cx = w / 2, cy = h / 2;
    const int half = 20;
    for (int y = cy - half; y < cy + half; ++y) {
        for (int x = cx - half; x < cx + half; ++x) {
            if (x >= 0 && x < w && y >= 0 && y < h) {
                dst[y * w + x] = 30;
            }
        }
    }
}

// Returns 1 if M4 came up + published its magic within `timeout_ms`,
// 0 otherwise.  Idempotent: subsequent calls are no-ops once M4 is up.
extern "C" int sentai_m4_bench_start(uint32_t timeout_ms) {
    if (s_m4_started) return 1;
    // Pre-clear the mailbox so a stale magic from a previous boot
    // can't fool us.
    s_mbox[0] = 0;  // magic
    s_mbox[1] = 0;  // go
    s_mbox[2] = 0;  // done
    s_mbox[3] = 0;  // result
    __asm volatile ("dsb sy" ::: "memory");
    // Start the M4 core.  No-op if no M4 binary is linked; in that
    // case the magic poll below will time out and we return 0.
    coralmicro::IpcM7::GetSingleton()->StartM4();
    // Poll for magic with bounded wait.
    const TickType_t deadline = xTaskGetTickCount() +
                                  pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        if (s_mbox[0] == M4_MAILBOX_MAGIC) {
            s_m4_started = 1;
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return 0;
}

// Run one bench iteration on M4 with the given block size.  Returns 1
// on success (out_cyc filled), 0 on timeout / M4 dead.  Caller MUST
// have called sentai_m4_bench_start first and gotten 1 back.
extern "C" int sentai_m4_bench_run(int block, uint32_t* out_cyc,
                                     uint32_t timeout_ms) {
    if (!s_m4_started) return 0;
    if (!out_cyc) return 0;
    if (block < 3 || block > 511) return 0;
    // Populate frame.
    synth_frame_(s_gray, M4_FRAME_W, M4_FRAME_H);
    // Clear done flag, then kick the go flag with block as payload.
    s_mbox[2] = 0;       // done
    s_mbox[3] = 0;       // result
    __asm volatile ("dsb sy" ::: "memory");
    s_mbox[1] = (uint32_t)block;  // go (also conveys block size)
    __asm volatile ("dsb sy" ::: "memory");
    // Poll for done.
    const TickType_t deadline = xTaskGetTickCount() +
                                  pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        if (s_mbox[2] != 0) {
            *out_cyc = s_mbox[3];
            return 1;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return 0;
}

extern "C" int sentai_m4_bench_is_alive(void) {
    return s_m4_started && (s_mbox[0] == M4_MAILBOX_MAGIC);
}

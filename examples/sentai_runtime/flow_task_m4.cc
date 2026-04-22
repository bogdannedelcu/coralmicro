// flow_task_m4.cc — M4-core worker for sentai.flow offload.
//
// Architecture (strictly one-way data flow, lock-free):
//
//   M7 PrepTask                 shared OCRAM             M4 app_main
//   ──────────                 ─────────────              ──────────
//   grab raw XRGB                                         poll cmd
//   PXP → tensor                                          if frame_valid:
//   decimate → gray[80×60] ──▶  gray buffer   ──────────▶    SAD vs prev
//   frame_seq++ ; valid=1 ──▶  header                         publish dx,dy
//                                              ◀──────────   valid=0
//
// M7 stays fast: camera + PXP go to the detection pipeline as usual,
// plus a ~50 µs step-8 CPU decimation to the shared gray buffer.  M4
// owns the SAD loop entirely — no PXP, no camera, no I2C.
//
// Startup bypasses `libs_base-m4_freertos`'s default main() because
// that calls BOARD_InitHardware() which resets pin-mux + PLLs the
// M7 camera is already using.  We only need MPU + MCMGR here.

#include <cstdio>
#include <cstring>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/MIMXRT1176_cm4.h"
#include "third_party/nxp/rt1176-sdk/middleware/multicore/mcmgr/src/mcmgr.h"

extern "C" {
void BOARD_ConfigMPU(void);
}

#include "examples/sentai_runtime/flow_shared.h"

// =============================================================
// SAD block-match
// =============================================================

// Reference block at the centre of the 80×60 frame.  Kept smaller
// than the frame on both axes so the ±kSearchRange window stays
// inside the image.  Same constants as the M7 reference implementation
// so results can be cross-checked.
namespace {

// ------------------------------------------------------------------
// DWT cycle counter — the reliable high-resolution timer on the M4.
// FreeRTOS tick on this core is unreliable as a wall clock because
// our custom main() skips BOARD_InitBootClocks (needed to avoid
// resetting the M7 camera PLLs), so SysTick-derived `tick_ms`
// reports bogus absolute time.  DWT->CYCCNT free-runs at whatever
// the core clock actually is; the **delta** between two samples
// always equals the elapsed cycles regardless of the absolute Hz.
// We publish cycles in shared-mem and let the host convert if a
// Hz value is available.  Publishing cycles (not µs) avoids a
// false-precision lie.
// ------------------------------------------------------------------
#define DWT_CTRL_REG   (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT_REG (*(volatile uint32_t*)0xE0001004)
#define CoreDebug_DEMCR_REG (*(volatile uint32_t*)0xE000EDFC)

static inline uint32_t dwt_cycles(void) { return DWT_CYCCNT_REG; }

static void dwt_init(void) {
    CoreDebug_DEMCR_REG |= (1u << 24);   // TRCENA
    DWT_CYCCNT_REG       = 0;
    DWT_CTRL_REG        |= 1u;           // CYCCNTENA
}

// Block + search-range scaled to the 80×60 grid (was 40×30 before the
// 2026-04-21 rpmsg-window expansion).  Block 32×32 centred, search
// ±12 px ⇒ worst-case SAD = 25² × 32² ≈ 640 000 add/abs operations.
// On the M4 @ ~240-400 MHz boot-default core clock that's 2-3 ms —
// still well under the 22 ms VGA/45 frame period.  Compared to the
// previous 40×30 config we get 4× more spatial resolution and 2×
// more search range at only ~5× the compute.
constexpr int kBlockW      = 32;
constexpr int kBlockH      = 32;
constexpr int kSearchRange = 12;

// Ping-pong gray buffers local to M4 so we keep the previous frame
// even after M7 overwrites shared.gray with a newer one.  Living in
// the default linker section on M4 (cacheable OCRAM) is INTENTIONAL
// and safe because M7 NEVER reads these buffers — only M4 writes via
// memcpy from the non-cacheable shared window and later reads back
// from the same slot.  Single-core access ⇒ no coherency issue.
// Do NOT share these with M7; the MPU would need region surgery and
// the shared_gray buffer in flow_shared_t already covers that role.
static uint8_t s_gray[2][FLOW_GRAY_PIXELS] __attribute__((aligned(4)));

static void sad_match(const uint8_t* curr, const uint8_t* prev,
                      int* dx_out, int* dy_out, uint32_t* sad_out) {
    const int bx = (FLOW_GRAY_W - kBlockW) / 2;
    const int by = (FLOW_GRAY_H - kBlockH) / 2;

    uint32_t best = 0xFFFFFFFFu;
    int best_dx = 0, best_dy = 0;

    for (int dy = -kSearchRange; dy <= kSearchRange; ++dy) {
        for (int dx = -kSearchRange; dx <= kSearchRange; ++dx) {
            uint32_t sad = 0;
            for (int y = 0; y < kBlockH; ++y) {
                const uint8_t* c = curr + (by + y) * FLOW_GRAY_W + bx;
                const uint8_t* p = prev + (by + y + dy) * FLOW_GRAY_W + (bx + dx);
                for (int x = 0; x < kBlockW; ++x) {
                    int d = (int)c[x] - (int)p[x];
                    sad += (uint32_t)(d < 0 ? -d : d);
                }
                if (sad >= best) break;
            }
            if (sad < best) {
                best = sad;
                best_dx = dx;
                best_dy = dy;
            }
        }
    }
    *dx_out = best_dx;
    *dy_out = best_dy;
    *sad_out = best;
}

static uint8_t sad_to_confidence(uint32_t sad) {
    const uint32_t block_pixels = kBlockW * kBlockH;
    const uint32_t mad = sad / block_pixels;
    if (mad >= 64) return 0;
    return (uint8_t)(255u - ((mad * 255u) / 64u));
}

}  // namespace

// =============================================================
// app_main — M4 main loop
// =============================================================

extern "C" [[noreturn]] void app_main(void* /*param*/) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    dwt_init();

    // Zero the M4-owned half of the struct before publishing magic,
    // so a stale reading from a previous warm-reset boot doesn't mix
    // into the new lifetime.
    sh->version          = FLOW_SHARED_VERSION;
    sh->m4_heartbeat     = 0;
    sh->m4_tick_ms       = 0;
    sh->m4_boot_ts_ms    = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    sh->m4_state         = FLOW_STATE_IDLE;
    sh->last_dx          = 0;
    sh->last_dy          = 0;
    sh->last_sad         = 0;
    sh->last_frame_seq   = 0;
    sh->last_confidence  = 0;
    sh->frames_processed = 0;
    sh->frames_dropped   = 0;
    sh->last_compute_us  = 0;
    sh->avg_compute_us   = 0;
    sh->cmd_ack_seq      = 0;
    __DSB();
    sh->magic = FLOW_SHARED_MAGIC;
    __DSB();

    bool     have_prev = false;
    int      prev_slot = 0;
    uint32_t seen_frame_seq = 0;
    uint32_t last_hb_ms = 0;
    uint32_t beat = 0;
    uint64_t avg_accum_us = 0;

    // 1 ms loop — fast enough to not miss frames at ~45 Hz, slow
    // enough to never starve FreeRTOS idle / timer ticks.
    while (true) {
        // Heartbeat — every 100 ms.
        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (now_ms - last_hb_ms >= 100) {
            last_hb_ms = now_ms;
            sh->m4_tick_ms = now_ms;
            __DMB();
            sh->m4_heartbeat = ++beat;
        }

        // Command handshake (one-shot).  Apply, then mirror the seq
        // so M7 sees the ack.  Changes to m4_state take effect on
        // the next iteration.  The __DMB() after reading cmd_seq is
        // the reader-side pair for M7's write fence between updating
        // `cmd` and bumping `cmd_seq`; without it the M4 could see
        // the new cmd_seq but still read a stale `cmd`.
        const uint32_t cmd_seq = sh->cmd_seq;
        if (cmd_seq != sh->cmd_ack_seq) {
            __DMB();
            switch (sh->cmd) {
                case FLOW_CMD_START:
                    sh->m4_state = FLOW_STATE_RUNNING;
                    have_prev = false;   // start from a clean baseline
                    sh->frames_processed = 0;
                    sh->frames_dropped   = 0;
                    avg_accum_us         = 0;
                    break;
                case FLOW_CMD_STOP:
                    sh->m4_state = FLOW_STATE_IDLE;
                    break;
                default:
                    break;
            }
            __DMB();
            sh->cmd_ack_seq = cmd_seq;
        }

        // Process a new frame only when we're in RUNNING state and
        // M7 has flagged one as unread.
        if (sh->m4_state == FLOW_STATE_RUNNING && sh->frame_valid) {
            // Acquire fence — after observing frame_valid=1 we must
            // __DMB() before reading frame_seq / gray.  Without this,
            // Cortex-M4's load buffer may hand us a stale row of the
            // gray buffer even though M7 already executed its __DMB()
            // write fence.  The M7's barrier only orders M7's own
            // writes; cross-core visibility at the reader requires
            // the reader to barrier too.  NASA/JPL §E — explicit
            // reader-side ordering, no implicit coherency assumptions.
            __DMB();
            const uint32_t new_seq = sh->frame_seq;

            // Copy the shared gray (non-cacheable) into M4-local
            // cacheable OCRAM so the SAD loop runs at L1 speed.
            const int curr_slot = prev_slot ^ 1;
            memcpy(s_gray[curr_slot], (const void*)sh->gray, FLOW_GRAY_PIXELS);

            // Release the publish slot as early as possible so M7
            // can fill the next frame without having to wait.
            sh->frame_valid = 0;
            __DSB();

            const uint32_t c0 = dwt_cycles();
            if (have_prev) {
                int dx = 0, dy = 0;
                uint32_t sad = 0;
                sad_match(s_gray[curr_slot], s_gray[prev_slot],
                          &dx, &dy, &sad);

                const uint32_t c1 = dwt_cycles();
                // Delta is modular-subtract on uint32 — correct across
                // the 2^32-cycle wrap boundary (~10.7 s at 400 MHz).
                // We expose the field as "compute_us" for API
                // compatibility, but the value is actually **cycles**
                // — divide by core-clock-Hz to convert to seconds if
                // a host-side mapping is available.  Tick-based µs
                // was unreliable because BOARD_InitBootClocks is
                // skipped on this M4.
                const uint32_t compute_cycles = c1 - c0;

                sh->last_dx         = dx;
                sh->last_dy         = dy;
                sh->last_sad        = sad;
                sh->last_frame_seq  = new_seq;
                sh->last_confidence = sad_to_confidence(sad);
                sh->last_compute_us = compute_cycles;

                avg_accum_us += compute_cycles;
                const uint32_t fp = sh->frames_processed + 1;
                sh->frames_processed = fp;
                sh->avg_compute_us   = (uint32_t)(avg_accum_us / fp);
            } else {
                have_prev = true;
                sh->frames_processed = 1;  // counting the baseline frame
            }

            prev_slot     = curr_slot;
            seen_frame_seq = new_seq;
            (void)seen_frame_seq;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =============================================================
// main — minimal startup (no BOARD_InitHardware)
// =============================================================

namespace {
constexpr size_t kM4StackSize = configMINIMAL_STACK_SIZE * 10;
static StaticTask_t s_m4_task_buf;
static StackType_t  s_m4_stack[kM4StackSize];
}  // namespace

extern "C" int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    // MPU is per-core.  Configure region 15 as non-cacheable so
    // writes to the shared window land in the physical OCRAM cell
    // the M7 reads from (otherwise they'd be swallowed by M4's L1).
    BOARD_ConfigMPU();

    // MU interrupt plumbing — required for MCMGR/RPMsg handshake.
    // We do NOT call IpcM4::Init() because the M7 doesn't use RPMsg
    // for this feature; M4IsAlive() will therefore time out on the
    // M7, which is why `sentai.flow.m4_enable()` returns rc=-2 even
    // though the M4 is up (check the shared magic instead).
    MCMGR_Init();

    xTaskCreateStatic(app_main, "flow_m4", kM4StackSize,
                      nullptr, 2, s_m4_stack, &s_m4_task_buf);
    vTaskStartScheduler();
    while (true) { /* unreachable */ }
    return 0;
}

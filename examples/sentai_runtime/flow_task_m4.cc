// flow_task_m4.cc — M4-core worker for sentai.flow offload (s240 rewrite).
//
// Architecture (strictly one-way data flow, lock-free):
//
//   M7 PrepTask                 shared OCRAM             M4 (bare-metal)
//   ──────────                 ─────────────              ──────────
//   grab raw XRGB                                         poll cmd
//   PXP → tensor                                          if frame_valid:
//   decimate → gray[80×60] ──▶  gray buffer   ──────────▶    USADA8 SAD vs prev
//   frame_seq++ ; valid=1 ──▶  header                         publish dx,dy
//                                              ◀──────────   valid=0
//
// The M4 owns the SAD loop entirely — no PXP, no camera, no I2C, no
// scheduler.  The value of this offload is NOT compute speed (M4 is
// ~2.4× slower than M7 per the OP-S10-W16 ablation); it is that the
// SAD + its buffers run on the M4 core's OWN memory, off BOTH of the
// M7's contended buses (OCRAM = 900 KB TPU-input DMA/invoke; SEMC =
// camera + PXP).  See experiments/s240.../m4_offload_review.md.
//
// s240 rewrite — three defects fixed vs the pre-#1130 version:
//   1. FREEZE: the old code ran a FreeRTOS scheduler on a SysTick that
//      was miscalibrated (custom main skips BOARD_InitBootClocks to
//      protect the M7 camera PLLs, which left configCPU_CLOCK_HZ vs the
//      real M4 clock mismatched → tick ISR ~360× too fast → 1 fps +
//      hang).  FIX: run BARE-METAL — no scheduler, no vTaskDelay, no
//      xTaskGetTickCount.  All timing from DWT->CYCCNT (the only
//      reliable timer on this core; the M4 clock is already programmed
//      by the M7 before the M4 leaves reset, so DWT counts real cycles).
//   2. SLOW: the old SAD did byte-wise abs-diff.  FIX: USADA8 DSP SIMD
//      (4 bytes / instruction), same intrinsic the M7 flow path uses.
//   3. OVERLAP: FLOW_SHARED_ADDR was a stale pointer into .tpu_input.
//      Fixed in flow_shared.h (relocated off the TPU input tensor).
//
// Startup deliberately overrides the weak libs_base-m4_freertos main()
// so we do NOT call BOARD_InitHardware()/BOARD_InitBootClocks() — those
// reset pin-mux + PLLs + PMIC + I2C the running M7 camera depends on.
// We only need MPU (for the non-cacheable shared window) + MCMGR (for
// the M7 StartM4 boot handshake).

#include <cstdint>
#include <cstring>

#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/MIMXRT1176_cm4.h"
#include "third_party/nxp/rt1176-sdk/middleware/multicore/mcmgr/src/mcmgr.h"

extern "C" {
void BOARD_ConfigMPU(void);
}

#include "examples/sentai_runtime/flow_shared.h"

namespace {

// ------------------------------------------------------------------
// DWT cycle counter — the reliable high-resolution timer on the M4.
// FreeRTOS SysTick is unusable here (we skip BOARD_InitBootClocks), so
// we run bare-metal and derive all timing from DWT->CYCCNT.  The M4
// core clock is programmed by the M7 before releasing this core from
// reset, so CYCCNT free-runs at the real core Hz.
// ------------------------------------------------------------------
#define DWT_CTRL_REG        (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT_REG      (*(volatile uint32_t*)0xE0001004)
#define CoreDebug_DEMCR_REG (*(volatile uint32_t*)0xE000EDFC)

static inline uint32_t dwt_cycles(void) { return DWT_CYCCNT_REG; }

static void dwt_init(void) {
    CoreDebug_DEMCR_REG |= (1u << 24);   // TRCENA
    DWT_CYCCNT_REG       = 0;
    DWT_CTRL_REG        |= 1u;           // CYCCNTENA
}

// Heartbeat cadence: ~100 ms at an assumed ~400 MHz M4 clock.  The
// exact Hz is irrelevant — this only has to change regularly so the
// M7 liveness check (magic + advancing heartbeat) sees the M4 alive.
constexpr uint32_t kHeartbeatCycles = 40000000u;

// Poll throttle — CRITICAL (s240).  Without it, this bare-metal loop
// reads FLOW_SHARED (frame_valid/cmd_seq, in OCRAM) at the full core
// rate (~100 M reads/s), saturating the OCRAM bus against the M7 TPU's
// 900 KB input DMA -> "edgetpu-custom-op failed to invoke".  We spin on
// DWT->CYCCNT (a core-local PPB register, NOT OCRAM) between polls, so
// no OCRAM traffic occurs during the wait.  ~500 µs @ 400 MHz is far
// below the ~22 ms frame period, so it adds negligible pickup latency
// while cutting idle OCRAM reads ~1000x.
constexpr uint32_t kPollThrottleCycles = 200000u;   // ~500 µs @ 400 MHz

static inline void dwt_spin(uint32_t cyc) {
    const uint32_t start = DWT_CYCCNT_REG;              // read once
    while ((uint32_t)(DWT_CYCCNT_REG - start) < cyc) {
        // spin on the core-local cycle counter only — no OCRAM access
        __asm volatile("" ::: "memory");
    }
}

// USADA8: 4-byte sum-of-abs-differences + accumulate (DSP-extension
// SIMD on Cortex-M4F).  arm_acle's __USADA8 isn't shipped on our
// toolchain, so inline-asm.  Mirrors m4_aruco_bench.cc::m4_usada8.
__attribute__((always_inline)) static inline uint32_t
m4_usada8(uint32_t a, uint32_t b, uint32_t acc) {
    uint32_t r;
    __asm volatile ("usada8 %0, %1, %2, %3" : "=r"(r) : "r"(a), "r"(b), "r"(acc));
    return r;
}

// Unaligned 32-bit load — the search offset makes the prev pointer
// unaligned; Cortex-M4 does unaligned LDR transparently but the
// compiler needs this packed-struct hint or it emits memcpy() and
// kills the USADA8 win.  Same trick as flow_task.cc:57.
typedef struct { uint32_t v; } __attribute__((packed)) m4_u32u_t;
#define M4_LD32U(p) (((const m4_u32u_t*)(p))->v)

// Block 32×32 centred in the 80×60 grid, search ±12 px — same
// constants as the M7 reference so results cross-check against
// s083_flow_m7_validate.  32 is a multiple of 4 → clean USADA8 unroll.
constexpr int kBlockW      = 32;
constexpr int kBlockH      = 32;
constexpr int kSearchRange = 12;

// M4-LOCAL ping-pong gray buffers.  No section attribute → default
// .bss, which on the M4 linker lands in m_data (DTCM, 1-cycle access).
// This is the whole point: the SAD inner loop touches ONLY DTCM +
// registers, never the M7's OCRAM/SEMC buses.  M7 never reads these.
static uint8_t s_gray[2][FLOW_GRAY_PIXELS] __attribute__((aligned(4)));

// USADA8 SAD for one (dx,dy) candidate.  Fully unrolled 8× over the
// 32-wide block row (32 bytes = 8 × 4).  No early-exit (branchless is
// faster here than the mispredict + the whole exhaustive sweep is
// ~0.4 ms).  curr/prev point at DTCM-local buffers.
__attribute__((always_inline)) static inline uint32_t
sad_at(const uint8_t* curr, const uint8_t* prev, int dx, int dy) {
    const int bx = (FLOW_GRAY_W - kBlockW) / 2;
    const int by = (FLOW_GRAY_H - kBlockH) / 2;
    uint32_t sad = 0;
    for (int y = 0; y < kBlockH; ++y) {
        const uint8_t* c = curr + (by + y) * FLOW_GRAY_W + bx;
        const uint8_t* p = prev + (by + y + dy) * FLOW_GRAY_W + (bx + dx);
        sad = m4_usada8(M4_LD32U(c +  0), M4_LD32U(p +  0), sad);
        sad = m4_usada8(M4_LD32U(c +  4), M4_LD32U(p +  4), sad);
        sad = m4_usada8(M4_LD32U(c +  8), M4_LD32U(p +  8), sad);
        sad = m4_usada8(M4_LD32U(c + 12), M4_LD32U(p + 12), sad);
        sad = m4_usada8(M4_LD32U(c + 16), M4_LD32U(p + 16), sad);
        sad = m4_usada8(M4_LD32U(c + 20), M4_LD32U(p + 20), sad);
        sad = m4_usada8(M4_LD32U(c + 24), M4_LD32U(p + 24), sad);
        sad = m4_usada8(M4_LD32U(c + 28), M4_LD32U(p + 28), sad);
    }
    return sad;
}

static void sad_match(const uint8_t* curr, const uint8_t* prev,
                      int* dx_out, int* dy_out, uint32_t* sad_out) {
    uint32_t best = 0xFFFFFFFFu;
    int best_dx = 0, best_dy = 0;
    for (int dy = -kSearchRange; dy <= kSearchRange; ++dy) {
        for (int dx = -kSearchRange; dx <= kSearchRange; ++dx) {
            const uint32_t sad = sad_at(curr, prev, dx, dy);
            if (sad < best) { best = sad; best_dx = dx; best_dy = dy; }
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

// ------------------------------------------------------------------
// Bare-metal worker loop — never returns.  Tight poll of frame_valid;
// no scheduler, no SysTick, DWT for the heartbeat cadence only.
// ------------------------------------------------------------------
[[noreturn]] static void flow_loop(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();

    // Publish a fresh lifetime: zero the M4-owned fields, then magic.
    sh->version          = FLOW_SHARED_VERSION;
    sh->m4_heartbeat     = 0;
    sh->m4_state         = FLOW_STATE_IDLE;
    sh->last_dx          = 0;
    sh->last_dy          = 0;
    sh->last_sad         = 0;
    sh->last_frame_seq   = 0;
    sh->last_confidence  = 0;
    sh->frames_processed = 0;
    sh->frames_dropped   = 0;
    sh->last_compute_us  = 0;
    sh->cmd_ack_seq      = 0;
    __DSB();
    sh->magic = FLOW_SHARED_MAGIC;
    __DSB();

    bool     have_prev   = false;
    int      prev_slot   = 0;
    uint32_t beat        = 0;
    uint32_t last_hb_cyc = dwt_cycles();

    for (;;) {
        // Heartbeat on a DWT cadence (no ticks).  Modular subtract is
        // correct across the 2^32 wrap.
        const uint32_t now_cyc = dwt_cycles();
        if ((uint32_t)(now_cyc - last_hb_cyc) >= kHeartbeatCycles) {
            last_hb_cyc = now_cyc;
            __DMB();
            sh->m4_heartbeat = ++beat;
        }

        // Command handshake (one-shot).  __DMB pairs with M7's fence
        // between writing `cmd` and bumping `cmd_seq`.
        const uint32_t cmd_seq = sh->cmd_seq;
        if (cmd_seq != sh->cmd_ack_seq) {
            __DMB();
            switch (sh->cmd) {
                case FLOW_CMD_START:
                    sh->m4_state = FLOW_STATE_RUNNING;
                    have_prev = false;   // clean baseline
                    sh->frames_processed = 0;
                    sh->frames_dropped   = 0;
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

        // Process a new frame only in RUNNING state with an unread one.
        if (sh->m4_state == FLOW_STATE_RUNNING && sh->frame_valid) {
            __DMB();   // acquire fence before reading frame_seq / gray
            const uint32_t new_seq = sh->frame_seq;

            // Copy shared (non-cacheable) gray into a DTCM-local slot so
            // the SAD sweep runs entirely on local RAM.
            const int curr_slot = prev_slot ^ 1;
            memcpy(s_gray[curr_slot], (const void*)sh->gray, FLOW_GRAY_PIXELS);

            // Release the publish slot ASAP so M7 can fill the next.
            sh->frame_valid = 0;
            __DSB();

            const uint32_t c0 = dwt_cycles();
            if (have_prev) {
                int dx = 0, dy = 0;
                uint32_t sad = 0;
                sad_match(s_gray[curr_slot], s_gray[prev_slot], &dx, &dy, &sad);
                const uint32_t compute_cycles = dwt_cycles() - c0;

                sh->last_dx         = dx;
                sh->last_dy         = dy;
                sh->last_sad        = sad;
                sh->last_frame_seq  = new_seq;
                sh->last_confidence = sad_to_confidence(sad);
                sh->last_compute_us = compute_cycles;   // cycles, not µs
                sh->frames_processed = sh->frames_processed + 1;
            } else {
                have_prev = true;
                sh->frames_processed = 1;   // baseline frame
            }
            prev_slot = curr_slot;
        }

        // Throttle the poll so idle OCRAM reads don't contend with the
        // M7 TPU input DMA (s240).  Spins on DWT (core-local), not OCRAM.
        dwt_spin(kPollThrottleCycles);
    }
}

}  // namespace

// =============================================================
// main — minimal bare-metal startup (overrides the weak SDK main).
// No BOARD_InitHardware / BOARD_InitBootClocks: those touch PMIC/I2C/
// PLLs the running M7 camera owns.  No scheduler.
// =============================================================
extern "C" int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    // MPU is per-core: mark the shared window non-cacheable so writes
    // land in the physical OCRAM cell the M7 reads (else swallowed by
    // M4 L1).  (Region set up in BOARD_ConfigMPU.)
    BOARD_ConfigMPU();

    // MU plumbing for the MCMGR/StartM4 boot handshake.  We do NOT call
    // IpcM4::Init() (no RPMsg for this feature — M7 checks the shared
    // magic for liveness, not M4IsAlive()).
    MCMGR_Init();

    dwt_init();
    flow_loop();       // never returns
    return 0;          // unreachable
}

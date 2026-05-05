// flow_task.cc -- M7-only optical flow stack for `sentai.flow`.
//
// History:
//   v1 (~build 720): legacy M7 SAD, mutually exclusive with TPU pipeline.
//   v2 (~build 730): SAD offloaded to M4, M7 stayed publisher.
//   v3 (build 1130, 2026-05-05): SAD moved BACK to M7 because M4
//        was running with mis-calibrated SysTick (FreeRTOS thought one
//        clock domain, real M4_CLK_ROOT was another) -- empirically
//        produced ~12 fps instead of expected 25 fps and froze after
//        ~5 sec of activity.  M7 has correct clocks, ITCM/cache,
//        WDOG + HardFault hooks, and ~1.5 ms SAD compute headroom.
//        M4 is no longer started.
//
// Pipeline (single core, all C++ on M7):
//
//   publisher_task (this file):
//     1. grab raw XRGB camera buffer
//     2. PXP downscale 640x480 -> 80x60 RGB888 (sentai_pxp_scale)
//     3. RGB->Y conversion -> shared gray + local M7 ping-pong slot
//     4. SAD block-match (curr vs prev) + parabolic sub-pixel fit
//     5. Apply conf-floor + deadband
//     6. Publish (dx, dy, sad, conf, frame_seq) into FLOW_SHARED().
//
// The flow_shared_t struct stays put for binding compatibility --
// MicroPython callers (`sentai.flow.read()` etc.) read from it.
// No cross-core sync is needed; M7 is the sole writer/reader.
//
// Algorithmic constants (see embeded.md sec.B/L for bounded loops):
//   gray:           80 x 60 (decimated 8:1 from camera native 640x480)
//   block:          32 x 32 centred
//   search range:   +/- 12 grid-px on each axis (25 x 25 = 625 candidates)
//   parabolic fit:  on the 4 axis-neighbour SADs of the integer minimum
//   conf floor:     150  (sad_to_confidence < 150 -> output (0, 0))
//   deadband:       50 milli-grid-px on both axes (kills sub-pixel noise)
//
// SERR codes (see sentai_error.h, module 0x0E):
//   0x0E02 PUB_GRAB           publisher cam_grab fail streak >= kPubGrabFailEscalate
//   0x0E03 PUB_TASK_CREATE    xTaskCreate failed at start
//   0x0E20 BAD_CAM_ID         cmd_start with cam_id != 0/1

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <cstdint>
#include <cstring>
#include <arm_acle.h>     // __USADA8 (Cortex-M7 DSP-extension SIMD intrinsic)

// Unaligned 32-bit read as a single LDR instruction.  GCC inlined
// memcpy(&u32, p, 4) as a *function call* to libc memcpy when 'p'
// alignment couldn't be proven -- 30+ cycles of overhead per call,
// ruining USADA8's whole point.  The packed-struct trick tells the
// compiler "treat *p as a uint32_t with no alignment requirement",
// and ARM Cortex-M7 handles unaligned LDR in hardware at ~1 extra
// cycle.  See agent/experiment.md for the disasm that proved this.
typedef struct { uint32_t v; } __attribute__((packed, aligned(1))) u32_unaligned;
#define LD32U(p) (((const u32_unaligned*)(p))->v)

#include "libs/camera/camera_support.h"   // DEMO_CAMERA_WIDTH/HEIGHT
#include "examples/sentai_runtime/flow_shared.h"
#include "examples/sentai_runtime/sentai_error.h"

extern "C" {
    int      sentai_cam_grab_latest(uint8_t** raw);
    void     sentai_cam_return_raw(int idx);
    uint32_t sentai_cam_get_frame_seq(void);
    uint32_t sentai_cam_get_sensor_frames(void);
    int      sentai_cam_is_initialized(void);
    int      sentai_detection_is_running(void);
    int      sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                              uint8_t* dst, int dw, int dh);
}

// =====================================================================
// Static buffers -- everything in SDRAM (.sdram_bss zeroed at boot).
// =====================================================================

// PXP output scratch (80x60 packed RGB888 = 14400 bytes).  PXP is an
// SDRAM master; pxp_scale_xrgb_to_rgb does its own cache invalidate
// around the DMA, so this is safe to share with CPU reads afterwards.
static uint8_t s_pxp_scratch[FLOW_GRAY_W * FLOW_GRAY_H * 3]
    __attribute__((aligned(32), section(".sdram_bss")));

// M7-local ping-pong gray buffers: SAD compares curr vs prev.  Cached
// SDRAM is fine since M7 is the sole writer + reader (no cross-core
// coherency to worry about).  4-byte aligned for fast 32-bit reads.
static uint8_t s_gray[2][FLOW_GRAY_PIXELS]
    __attribute__((aligned(4), section(".sdram_bss")));

// =====================================================================
// Algorithm constants
// =====================================================================

namespace {
constexpr int kBlockW       = 32;
constexpr int kBlockH       = 32;
constexpr int kSearchRange  = 12;
constexpr int kSurfDim      = 2 * kSearchRange + 1;
constexpr uint8_t kConfFloor = 150;
constexpr int kDeadbandQ1000 = 50;       // 0.05 grid-px

// Auto-level (histogram min-max stretch) on the 80x60 gray buffer.
// Off by default.  See OV5640 AEC notes in agent.md.
volatile int     s_gray_stretch_enabled  = 0;
volatile uint8_t s_gray_stretch_last_vmin = 0;
volatile uint8_t s_gray_stretch_last_vmax = 0;

// Currently-selected camera (mirrored into shared struct so callers
// can correlate dx/dy with which lens produced it).
volatile int s_cam_id = 0;

// frame_seq published into shared struct by the SAD path.
volatile uint32_t s_publish_seq = 0;

// Set by sentai.flow.start(), cleared by sentai.flow.stop().  When 0,
// publisher_task drops everything (acquire-load early exit).
volatile int s_compute_enabled = 0;

// "have_prev" guard -- first frame after start has no prior to compare.
volatile int s_have_prev = 0;
volatile int s_prev_slot = 0;

// Diag counters for sentai.diag.flow_stats() / SERR escalation.
volatile uint32_t s_pub_grab_fail_total  = 0;
volatile uint32_t s_pub_grab_fail_streak = 0;
volatile uint32_t s_pub_frames           = 0;

// Per-stage cycle timings (DWT) -- updated each frame.  M7 DWT runs
// at the core clock (800 MHz) so 1 cycle = 1.25 ns.
volatile uint32_t s_t_pxp_cyc      = 0;  // PXP downscale stage
volatile uint32_t s_t_rgb2y_cyc    = 0;  // RGB->Y + dual write
volatile uint32_t s_t_stretch_cyc  = 0;  // optional gray_stretch
volatile uint32_t s_t_sad_cyc      = 0;  // SAD + parabolic
volatile uint32_t s_t_total_cyc    = 0;  // publish_frame end-to-end
volatile uint32_t s_t_grab_cyc     = 0;  // sentai_cam_grab_latest
volatile uint32_t s_t_loop_cyc     = 0;  // full publisher_task iter
}  // namespace

// DWT cycle counter access.  Already enabled by SystemInit on M7 in
// most NXP setups; if not, our first read returns 0 forever -- in
// that case the diag will show 0 values and we know to enable.
#define DWT_CTRL   (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT (*(volatile uint32_t*)0xE0001004)
#define DEMCR      (*(volatile uint32_t*)0xE000EDFC)
static inline uint32_t dwt_now(void) { return DWT_CYCCNT; }
static void dwt_enable_once(void) {
    static int s_dwt_armed = 0;
    if (s_dwt_armed) return;
    DEMCR    |= (1u << 24);   // TRCENA
    DWT_CYCCNT = 0;
    DWT_CTRL  |= 1u;          // CYCCNTENA
    s_dwt_armed = 1;
}

// =====================================================================
// Gray stretch (optional auto-level)
// =====================================================================

static inline void gray_stretch_in_place(volatile uint8_t* gray, int n) {
    uint32_t hist[256] = {0};
    for (int i = 0; i < n; ++i) ++hist[gray[i]];
    int vmin = 0;
    while (vmin < 255 && hist[vmin] == 0) ++vmin;
    int vmax = 255;
    while (vmax > 0 && hist[vmax] == 0) --vmax;
    s_gray_stretch_last_vmin = (uint8_t)vmin;
    s_gray_stretch_last_vmax = (uint8_t)vmax;
    if (vmax <= vmin) return;
    const int span = vmax - vmin;
    uint8_t lut[256];
    for (int i = 0; i < 256; ++i) {
        int v = i - vmin;
        if (v <= 0)         lut[i] = 0;
        else if (v >= span) lut[i] = 255;
        else                lut[i] = (uint8_t)((v * 255) / span);
    }
    for (int i = 0; i < n; ++i) gray[i] = lut[gray[i]];
}

extern "C" int sentai_flow_gray_stretch_set(int enable) {
    return __atomic_exchange_n(&s_gray_stretch_enabled,
                               enable ? 1 : 0, __ATOMIC_RELEASE);
}

extern "C" int sentai_flow_gray_stretch_get(uint8_t* vmin, uint8_t* vmax) {
    if (vmin) *vmin = s_gray_stretch_last_vmin;
    if (vmax) *vmax = s_gray_stretch_last_vmax;
    return __atomic_load_n(&s_gray_stretch_enabled, __ATOMIC_ACQUIRE);
}

// =====================================================================
// SAD (block matching)
// =====================================================================

// Parabolic sub-pixel refinement.  See M4-port history for the
// rejection rationale (denom <= 0, shallow surface, |delta| > 0.5).
static inline int parabolic_q1000(uint32_t a, uint32_t b, uint32_t c) {
    int64_t denom = (int64_t)a + (int64_t)c - 2 * (int64_t)b;
    if (denom <= 0) return 0;
    int64_t arm_max = (a > c) ? ((int64_t)a - (int64_t)b)
                              : ((int64_t)c - (int64_t)b);
    if (denom * 8 < arm_max) return 0;
    int64_t num     = (int64_t)a - (int64_t)c;
    int64_t d_q1000 = (num * 1000) / (2 * denom);
    if (d_q1000 >  500 || d_q1000 < -500) return 0;
    return (int)d_q1000;
}

static uint8_t sad_to_confidence(uint32_t sad) {
    constexpr uint32_t bp = (uint32_t)(kBlockW * kBlockH);
    const uint32_t mad = sad / bp;
    if (mad >= 64) return 0;
    return (uint8_t)(255u - ((mad * 255u) / 64u));
}

static inline uint32_t* surf_at(uint32_t* surf, int dy, int dx) {
    return &surf[(dy + kSearchRange) * kSurfDim + (dx + kSearchRange)];
}

// Integer SAD search + parabolic sub-pixel + conf floor + deadband.
// Outputs in milli-grid-px units (1000 = 1 grid-px).  Best SAD also
// returned so the caller can derive confidence externally.
//
// Placed in ITCM (.ramfunc) so instruction fetch hits the M7's
// tightly-coupled instruction memory at 0-wait state instead of the
// default text section (which lives in OCRAM/SDRAM via the system
// bus and pays cache miss penalties on the hot inner loop).  See
// agent.md sec.2 for the same pattern applied to CSI ISR.  Requires
// MIMXRT1176xxxxx_cm7_ram_mp.ld to have a .ramfunc rule mapping to
// m_text (already present per the same agent.md note).
__attribute__((section(".ramfunc")))
static void sad_match(const uint8_t* curr, const uint8_t* prev,
                      int* dx_q1000_out, int* dy_q1000_out,
                      uint32_t* sad_out) {
    const int bx = (FLOW_GRAY_W - kBlockW) / 2;
    const int by = (FLOW_GRAY_H - kBlockH) / 2;

    static uint32_t s_surf[kSurfDim * kSurfDim];
    uint32_t best   = 0xFFFFFFFFu;
    int      bdx = 0, bdy = 0;

    // Inner SAD loop using Cortex-M7 DSP extension USADA8 instruction:
    // each call computes sum(|op1[i] - op2[i]|) over 4 packed bytes
    // and adds to the accumulator.  Replaces 4 byte ops + 4 sub +
    // 4 abs + 4 add (~16 cycles) with a single 1-cycle USADA8 = ~8x
    // speedup on the inner loop.
    //
    // Pre-condition: kBlockW must be a multiple of 4 (32 ✓), and
    // both s_gray buffers are 4-byte aligned (declared __aligned(4)).
    // The pointer 'p' is shifted by (bx + dx) which CAN be unaligned
    // when dx is not a multiple of 4 -- in that case the access is
    // unaligned but Cortex-M7 supports unaligned reads transparently
    // (a few extra cycles for boundary-crossing words).  Profile
    // shows the SIMD path is still net-positive even with mixed
    // alignment.  An aligned-only fast path could be added later
    // (preload aligned word + shifted-mux for the trailing bytes).
    for (int dy = -kSearchRange; dy <= kSearchRange; ++dy) {
        for (int dx = -kSearchRange; dx <= kSearchRange; ++dx) {
            uint32_t sad = 0;
            for (int y = 0; y < kBlockH; ++y) {
                const uint8_t* c = curr + (by + y) * FLOW_GRAY_W + bx;
                const uint8_t* p = prev + (by + y + dy) * FLOW_GRAY_W + (bx + dx);
                // 32 bytes / row -> 8 USADA8 ops, single LDR per load.
                sad = __USADA8(LD32U(c +  0), LD32U(p +  0), sad);
                sad = __USADA8(LD32U(c +  4), LD32U(p +  4), sad);
                sad = __USADA8(LD32U(c +  8), LD32U(p +  8), sad);
                sad = __USADA8(LD32U(c + 12), LD32U(p + 12), sad);
                sad = __USADA8(LD32U(c + 16), LD32U(p + 16), sad);
                sad = __USADA8(LD32U(c + 20), LD32U(p + 20), sad);
                sad = __USADA8(LD32U(c + 24), LD32U(p + 24), sad);
                sad = __USADA8(LD32U(c + 28), LD32U(p + 28), sad);
            }
            *surf_at(s_surf, dy, dx) = sad;
            if (sad < best) { best = sad; bdx = dx; bdy = dy; }
        }
    }

    int delta_x = 0, delta_y = 0;
    if (bdx > -kSearchRange && bdx < kSearchRange) {
        uint32_t a = *surf_at(s_surf, bdy, bdx - 1);
        uint32_t c = *surf_at(s_surf, bdy, bdx + 1);
        delta_x = parabolic_q1000(a, best, c);
    }
    if (bdy > -kSearchRange && bdy < kSearchRange) {
        uint32_t a = *surf_at(s_surf, bdy - 1, bdx);
        uint32_t c = *surf_at(s_surf, bdy + 1, bdx);
        delta_y = parabolic_q1000(a, best, c);
    }

    int raw_dx = bdx * 1000 + delta_x;
    int raw_dy = bdy * 1000 + delta_y;

    const uint8_t conf = sad_to_confidence(best);
    if (conf < kConfFloor) { raw_dx = 0; raw_dy = 0; }
    if (raw_dx > -kDeadbandQ1000 && raw_dx < kDeadbandQ1000 &&
        raw_dy > -kDeadbandQ1000 && raw_dy < kDeadbandQ1000) {
        raw_dx = 0; raw_dy = 0;
    }

    *dx_q1000_out = raw_dx;
    *dy_q1000_out = raw_dy;
    *sad_out      = best;
}

// =====================================================================
// Frame publish (PXP -> gray + optional stretch + write to shared mem).
// Called by the publisher_task below AND by detection_task::PrepTask
// when the TPU pipeline is also running -- keeps shared gray fresh
// for any consumer that only reads it (e.g. m4_gray_snap binding).
// =====================================================================

extern "C" void sentai_flow_publish_frame(const uint8_t* raw,
                                          int raw_w, int raw_h,
                                          int cam_id) {
    if (!__atomic_load_n(&s_compute_enabled, __ATOMIC_ACQUIRE)) return;
    if (raw_w < FLOW_GRAY_W || raw_h < FLOW_GRAY_H) return;

    volatile flow_shared_t* sh = &FLOW_SHARED();
    dwt_enable_once();
    const uint32_t t0 = dwt_now();

    int rc = sentai_pxp_scale(raw, raw_w, raw_h,
                              s_pxp_scratch, FLOW_GRAY_W, FLOW_GRAY_H);
    if (rc != 0) return;
    const uint32_t t_after_pxp = dwt_now();
    s_t_pxp_cyc = t_after_pxp - t0;

    // RGB888 -> Y (Rec.601 approximation: Y = (R + 2G + B) >> 2).
    const int curr_slot = s_prev_slot ^ 1;
    uint8_t* dst_local = s_gray[curr_slot];
    volatile uint8_t* dst_shared = sh->gray;
    const uint8_t* src = s_pxp_scratch;
    for (int i = 0; i < FLOW_GRAY_PIXELS; ++i) {
        uint32_t r = src[3 * i + 0];
        uint32_t g = src[3 * i + 1];
        uint32_t b = src[3 * i + 2];
        uint8_t y = (uint8_t)((r + (g << 1) + b) >> 2);
        dst_local[i]  = y;
        dst_shared[i] = y;
    }
    const uint32_t t_after_rgb2y = dwt_now();
    s_t_rgb2y_cyc = t_after_rgb2y - t_after_pxp;

    if (__atomic_load_n(&s_gray_stretch_enabled, __ATOMIC_ACQUIRE)) {
        gray_stretch_in_place(sh->gray, FLOW_GRAY_PIXELS);
        for (int i = 0; i < FLOW_GRAY_PIXELS; ++i) {
            dst_local[i] = sh->gray[i];
        }
    }
    const uint32_t t_after_stretch = dwt_now();
    s_t_stretch_cyc = t_after_stretch - t_after_rgb2y;

    sh->frame_cam_id = (uint8_t)cam_id;

    int    dx_q = 0, dy_q = 0;
    uint32_t sad = 0;
    uint8_t  conf = 0;
    if (s_have_prev) {
        sad_match(s_gray[curr_slot], s_gray[s_prev_slot],
                  &dx_q, &dy_q, &sad);
        conf = sad_to_confidence(sad);
    } else {
        s_have_prev = 1;
    }
    const uint32_t t_after_sad = dwt_now();
    s_t_sad_cyc = t_after_sad - t_after_stretch;
    s_t_total_cyc = t_after_sad - t0;

    s_publish_seq++;
    sh->last_dx          = dx_q;
    sh->last_dy          = dy_q;
    sh->last_sad         = sad;
    sh->last_confidence  = conf;
    sh->last_frame_seq   = s_publish_seq;
    sh->frames_processed = s_publish_seq;
    __DMB();
    sh->frame_seq   = s_publish_seq;
    sh->frame_valid = 1;          // legacy field, kept for any reader

    s_prev_slot = curr_slot;
}

// Keep the legacy name alive for detection_task.cc which still calls
// it from the TPU pipeline path.
extern "C" void sentai_flow_m4_publish_frame(const uint8_t* raw,
                                             int raw_w, int raw_h,
                                             int cam_id) {
    sentai_flow_publish_frame(raw, raw_w, raw_h, cam_id);
}

extern "C" int sentai_flow_publish_set(int enable) {
    int prev = __atomic_exchange_n(&s_compute_enabled,
                                   enable ? 1 : 0, __ATOMIC_RELEASE);
    if (enable && !prev) {
        // Reset state for clean session
        s_have_prev = 0;
        s_publish_seq = 0;
        s_prev_slot = 0;
    }
    return prev;
}
extern "C" int sentai_flow_m4_publish_set(int enable) {
    return sentai_flow_publish_set(enable);
}

// =====================================================================
// Standalone publisher task -- runs ONLY when no TPU pipeline is
// already publishing through detection_task::PrepTask.
// =====================================================================

static volatile bool s_pub_running = false;
static TaskHandle_t  s_pub_task    = nullptr;

// ISR-side rendezvous handle.  CSI_IRQHandler reads this atomically;
// when non-NULL it calls vTaskNotifyGiveFromISR exactly once per
// sensor frame (fb1_done OR fb2_done).  Single writer (this file,
// outside ISR) -- the ISR is read-only.  Aligned 32-bit pointer is
// atomic on Cortex-M7.
extern "C" volatile TaskHandle_t g_flow_pub_isr_task = nullptr;

// Diag counters owned by the publisher task.  All writes from task
// context only (read freely from REPL / SERR_LOG).
static volatile uint32_t s_pub_notify_timeouts = 0;  // bumped per take=0
static volatile uint32_t s_pub_notify_overruns = 0;  // ISR notified while we were still computing previous frame
static volatile uint32_t s_pub_last_take_ms    = 0;

constexpr uint32_t kPubMaxIter            = 24u * 60u * 60u * 1000u;  // 24h cap
constexpr uint32_t kPubGrabFailEscalate   = 25;
// Bounded notification wait (NASA §2): 100 ms = 3 sensor frames at 30 fps.
// If no notify arrives in that window, sensor or CSI ISR is stalled --
// log and re-enter the wait so REPL/HTTP stay supervisable.  The
// publisher itself never blocks "forever".
constexpr TickType_t kPubNotifyWaitTicks  = pdMS_TO_TICKS(100);
constexpr uint32_t   kPubNotifyTimeoutEscalate = 5;  // ~500 ms silence -> SERR

// ISR-paced publisher (notify-from-ISR pattern).
//
// Why notify, not poll:
//   * embeded.md §C: ISR captures event, defers work to task.  Polling
//     loop with taskYIELD burns CPU, jitters latency, and dedups via a
//     read of a counter the ISR just bumped — fragile.
//   * NASA §1: explicit deterministic flow.  ulTaskNotifyTake is a
//     counted semaphore: every ISR notify increments, every take
//     decrements.  No lost or duplicated frames.
//   * NASA §2: bounded wait (kPubNotifyWaitTicks).  If sensor stalls,
//     we time out, log SERR_FLOW_NOTIFY_TIMEOUT, and re-enter the wait.
//
// Failure semantics:
//   take=0 (timeout)                  -> bump s_pub_notify_timeouts;
//                                         escalate to SERR after a streak
//   take >1 (overrun)                 -> ISR fired while task was busy;
//                                         we still process the latest
//                                         frame, but log the overrun
//   grab fail                         -> existing PUB_GRAB escalation
//
// Single writer of g_flow_pub_isr_task: this function (set/clear at
// start/stop).  ISR is read-only.  No lock needed.
static void publisher_task_fn(void* /*arg*/) {
    uint32_t iters       = 0;
    uint32_t timeout_streak = 0;
    dwt_enable_once();
    s_pub_last_take_ms = (uint32_t)xTaskGetTickCount();

    // Drain any stale notifications left from a previous run.
    (void)ulTaskNotifyTake(pdTRUE, 0);

    // Publish handle for the ISR.  Done AFTER the drain so the ISR
    // never observes a stale-pending count for this run.
    g_flow_pub_isr_task = xTaskGetCurrentTaskHandle();

    while (s_pub_running && iters++ < kPubMaxIter) {
        // Wait for the next sensor-frame notification.  pdTRUE clears
        // the counter on take, so n>1 indicates a missed-deadline event.
        uint32_t pending = ulTaskNotifyTake(pdTRUE, kPubNotifyWaitTicks);
        if (pending == 0) {
            s_pub_notify_timeouts++;
            timeout_streak++;
            if (timeout_streak == kPubNotifyTimeoutEscalate) {
                uint32_t now_ms = (uint32_t)xTaskGetTickCount();
                SERR_LOG(SERR_FLOW_NOTIFY_TIMEOUT,
                         now_ms - s_pub_last_take_ms);
            }
            continue;  // re-enter wait; bounded loop owns liveness
        }
        timeout_streak = 0;
        s_pub_last_take_ms = (uint32_t)xTaskGetTickCount();
        if (pending > 1) {
            // ISR fired while we were still processing the previous
            // frame.  Diagnostic only: we always grab the LATEST below,
            // so consumers still see a fresh frame; we just dropped
            // (pending-1) intermediate frames.
            s_pub_notify_overruns += (pending - 1);
            // Escalate at first overrun -- if compute is overrunning the
            // sensor period, downstream cadence is broken.
            if (s_pub_notify_overruns == (pending - 1)) {
                SERR_LOG(SERR_FLOW_NOTIFY_OVERRUN, pending);
            }
        }

        const uint32_t loop_t0 = dwt_now();
        uint8_t* raw = nullptr;
        const uint32_t grab_t0 = dwt_now();
        int idx = sentai_cam_grab_latest(&raw);
        s_t_grab_cyc = dwt_now() - grab_t0;
        if (idx < 0 || !raw) {
            s_pub_grab_fail_total++;
            uint32_t streak = ++s_pub_grab_fail_streak;
            if (streak == kPubGrabFailEscalate) {
                SERR_LOG(SERR_FLOW_PUB_GRAB, streak);
            }
            // No vTaskDelay -- next notify will pace us.  If notifies
            // also stop, kPubNotifyWaitTicks bounds the recovery wait.
            continue;
        }
        s_pub_grab_fail_streak = 0;

        sentai_flow_publish_frame(raw, DEMO_CAMERA_WIDTH,
                                  DEMO_CAMERA_HEIGHT, s_cam_id);
        s_pub_frames++;
        s_t_loop_cyc = dwt_now() - loop_t0;
        sentai_cam_return_raw(idx);
    }

    // Single-writer detach: ISR will see NULL on its next entry and
    // skip the notify call -- no spurious wake of a deleted task.
    g_flow_pub_isr_task = nullptr;
    s_pub_task = nullptr;
    vTaskDelete(nullptr);
}

extern "C" void sentai_flow_pub_stats(uint32_t* frames_published,
                                       uint32_t* grab_fail_total,
                                       uint32_t* grab_fail_streak,
                                       int* running) {
    if (frames_published) *frames_published = s_pub_frames;
    if (grab_fail_total)  *grab_fail_total  = s_pub_grab_fail_total;
    if (grab_fail_streak) *grab_fail_streak = s_pub_grab_fail_streak;
    if (running)          *running          = s_pub_running ? 1 : 0;
}

extern "C" void sentai_flow_pub_health(uint32_t* notify_timeouts,
                                        uint32_t* notify_overruns,
                                        uint32_t* last_take_ms) {
    if (notify_timeouts) *notify_timeouts = s_pub_notify_timeouts;
    if (notify_overruns) *notify_overruns = s_pub_notify_overruns;
    if (last_take_ms)    *last_take_ms    = s_pub_last_take_ms;
}

// DWT cycle stats from the most recent publish_frame + publisher
// loop iteration.  M7 core clock = 800 MHz, so 1 us = 800 cycles.
extern "C" void sentai_flow_perf_cyc(uint32_t* pxp, uint32_t* rgb2y,
                                      uint32_t* stretch, uint32_t* sad,
                                      uint32_t* total, uint32_t* grab,
                                      uint32_t* loop) {
    if (pxp)     *pxp     = s_t_pxp_cyc;
    if (rgb2y)   *rgb2y   = s_t_rgb2y_cyc;
    if (stretch) *stretch = s_t_stretch_cyc;
    if (sad)     *sad     = s_t_sad_cyc;
    if (total)   *total   = s_t_total_cyc;
    if (grab)    *grab    = s_t_grab_cyc;
    if (loop)    *loop    = s_t_loop_cyc;
}

// =====================================================================
// Public C API consumed by modsentai_flow.c MicroPython bindings.
// New names drop the "m4_" prefix; legacy names (sentai_flow_m4_*)
// kept as inline forwarders for transition compatibility.
// =====================================================================

// Initialise the M7 flow stack.  Idempotent.  Returns 0 on success.
// Mostly a magic stamp for the binding's "alive?" check now that M4
// is no longer involved.
extern "C" int sentai_flow_enable(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    sh->magic   = FLOW_SHARED_MAGIC;
    sh->version = FLOW_SHARED_VERSION;
    sh->m4_state         = FLOW_STATE_IDLE;
    sh->frames_processed = 0;
    sh->frames_dropped   = 0;
    sh->m4_heartbeat     = 0xFFFFFFFFu;   // sentinel: M4 NOT used
    return 0;
}
extern "C" int sentai_flow_m4_enable(void) { return sentai_flow_enable(); }

extern "C" int sentai_flow_start(int cam_id) {
    if (cam_id != 0 && cam_id != 1) {
        SERR_LOG(SERR_FLOW_BAD_CAM_ID, (uint32_t)cam_id);
        return -2;
    }
    volatile flow_shared_t* sh = &FLOW_SHARED();
    sentai_flow_enable();
    sh->cmd          = FLOW_CMD_START;
    sh->cmd_seq      = sh->cmd_seq + 1;
    sh->m4_state     = FLOW_STATE_RUNNING;
    sh->frame_cam_id = (uint8_t)cam_id;
    sentai_flow_publish_set(1);

    s_cam_id = cam_id;
    s_pub_grab_fail_total  = 0;
    s_pub_grab_fail_streak = 0;
    s_pub_frames           = 0;

    if (!s_pub_running && !sentai_detection_is_running()) {
        s_pub_running = true;
        BaseType_t r = xTaskCreate(publisher_task_fn, "flow_pub",
                                   configMINIMAL_STACK_SIZE * 3,
                                   nullptr, tskIDLE_PRIORITY + 2,
                                   &s_pub_task);
        if (r != pdPASS) {
            s_pub_running = false;
            SERR_LOG(SERR_FLOW_PUB_TASK_CREATE, (uint32_t)r);
            return -3;
        }
    }
    return 0;
}
extern "C" int sentai_flow_m4_cmd_start(int cam_id) { return sentai_flow_start(cam_id); }

extern "C" int sentai_flow_stop(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    sentai_flow_publish_set(0);
    sh->cmd      = FLOW_CMD_STOP;
    sh->cmd_seq  = sh->cmd_seq + 1;
    sh->m4_state = FLOW_STATE_IDLE;

    if (s_pub_running) {
        s_pub_running = false;
        for (int i = 0; i < 50 && s_pub_task; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    return 0;
}
extern "C" int sentai_flow_m4_cmd_stop(void) { return sentai_flow_stop(); }

// Detail-score: gradient energy of the current 80x60 gray buffer.
extern "C" uint32_t sentai_flow_detail_score(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    if (sh->magic != FLOW_SHARED_MAGIC) return 0;
    uint32_t sum = 0;
    const volatile uint8_t* g = sh->gray;
    for (int y = 0; y < FLOW_GRAY_H; ++y) {
        const volatile uint8_t* row = g + y * FLOW_GRAY_W;
        for (int x = 0; x < FLOW_GRAY_W - 1; ++x) {
            int d = (int)row[x + 1] - (int)row[x];
            sum += (uint32_t)(d < 0 ? -d : d);
        }
    }
    for (int y = 0; y < FLOW_GRAY_H - 1; ++y) {
        const volatile uint8_t* r0 = g + y * FLOW_GRAY_W;
        const volatile uint8_t* r1 = r0 + FLOW_GRAY_W;
        for (int x = 0; x < FLOW_GRAY_W; ++x) {
            int d = (int)r1[x] - (int)r0[x];
            sum += (uint32_t)(d < 0 ? -d : d);
        }
    }
    const uint32_t divisor = (FLOW_GRAY_W - 1) * FLOW_GRAY_H
                           + FLOW_GRAY_W * (FLOW_GRAY_H - 1);
    return (sum * 100u) / divisor;
}

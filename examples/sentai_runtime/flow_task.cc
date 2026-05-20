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
    // FFT phase-correlation matcher (flow_phase_corr.cc).  Caches its
    // own prev FFT internally; reset at flow.start.
    void     sentai_flow_phase_corr_compute(const uint8_t* gray80x60,
                                              int* dx_q1000,
                                              int* dy_q1000,
                                              uint8_t* conf);
    void     sentai_flow_phase_corr_reset(void);
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

// Rate-aware deadband (embeded.md §J: policy in physical units).
//
// Previous design: kDeadbandQ1000 = 50 milli-grid-px / FRAME — magic
// number tuned at 15 fps.  When sensor rate doubled to 30 fps the
// per-frame motion at constant scene velocity halved, so a fixed
// per-frame threshold ate slow motion at the end of each side and
// trajectory closure regressed (74 → 350 raw-px in s001 vs s002).
//
// New design: deadband expressed as a VELOCITY in physical units
// (milli-grid-px per second).  Per-frame threshold is recomputed from
// the measured sensor period, so the same physical motion-rejection
// policy applies whether the camera runs at 15 / 30 / 45 / 60 / 90 fps.
//   per-frame_mgp = velocity_mgp_per_s * period_ms / 1000
//
// Calibration: 1500 mgp/s = 1.5 grid-px/s = 12 raw-px/s.  Anchor: at
// 30 fps (current production rate, ISR notifying every sensor frame)
// this resolves to 50 mgp/frame -- the historical hardcoded value.
// Same physical threshold then auto-resolves to ~100 mgp at 15 fps,
// ~33 mgp at 45 fps, ~25 mgp at 60 fps.
constexpr uint32_t kDeadbandVelocityMgpPerSec = 1500;
// Bounded sanity caps -- if sensor rate collapses or surges, clamp
// the resulting per-frame deadband.  Bounded behaviour, embeded.md §B.
constexpr uint32_t kDeadbandMgpMin =  10;   // never below noise floor
constexpr uint32_t kDeadbandMgpMax = 500;   // never reject normal motion
// Cached per-frame deadband (mgp).  Recomputed in publisher_task on
// the cadence below using period derived from g_camera_sensor_frames.
// Single-writer (publisher), single-reader (sad_match via flow_publish).
// Bootstrap = 30 fps production value (50 mgp). Auto-rescales after
// first window completes.
volatile uint32_t s_deadband_mgp  = 50;
volatile uint32_t s_period_ms_x10 = 333;     // 33.3 ms = 30 fps bootstrap
constexpr uint32_t kDeadbandRecalcEveryN = 16;  // every ~0.5 s @ 30 fps

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
    // Snapshot the deadband ONCE per call so dx and dy axes use the
    // same threshold (publisher_task may recompute mid-frame).
    const int db = (int)s_deadband_mgp;
    if (raw_dx > -db && raw_dx < db &&
        raw_dy > -db && raw_dy < db) {
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
    // Phase-correlation matcher (flow_phase_corr.cc) re-wired with
    // breadcrumbs for JTAG-aided crash isolation.
    sentai_flow_phase_corr_compute(s_gray[curr_slot], &dx_q, &dy_q, &conf);
    s_have_prev = 1;
    if (conf < kConfFloor) { dx_q = 0; dy_q = 0; }
    const int db = (int)s_deadband_mgp;
    if (dx_q > -db && dx_q < db && dy_q > -db && dy_q < db) {
        dx_q = 0; dy_q = 0;
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

    // Rate-aware deadband state: track sensor-frame count + wall time
    // over a fixed window, derive period_ms, recompute deadband_mgp.
    // embeded.md §A: explicit policy in physical units (mgp/s).
    uint32_t db_window_t0    = (uint32_t)xTaskGetTickCount();
    uint32_t db_window_seq0  = sentai_cam_get_sensor_frames();
    uint32_t db_window_iters = 0;

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

        // Rate-aware deadband recompute.  Bounded period derivation
        // (NASA §2): every kDeadbandRecalcEveryN frames sample
        // (sensor_frames, ticks) and update s_deadband_mgp.  All
        // bounded math, no division by zero (frames_seen >= 1
        // guaranteed by the increment below, and we only divide when
        // frames_seen > 0).  Saturating clamps protect against
        // pathological rates.
        if (++db_window_iters >= kDeadbandRecalcEveryN) {
            uint32_t now_ms     = (uint32_t)xTaskGetTickCount();
            uint32_t seq_now    = sentai_cam_get_sensor_frames();
            uint32_t frames_seen = seq_now - db_window_seq0;
            uint32_t elapsed_ms = now_ms - db_window_t0;
            if (frames_seen > 0u && elapsed_ms > 0u) {
                // period_ms_x10 = elapsed_ms * 10 / frames_seen
                uint32_t per_x10 = (elapsed_ms * 10u) / frames_seen;
                s_period_ms_x10  = per_x10;
                // deadband_mgp = velocity_mgp_per_s * period_ms / 1000
                //              = velocity * per_x10 / 10000
                uint32_t mgp = (kDeadbandVelocityMgpPerSec * per_x10)
                               / 10000u;
                if (mgp < kDeadbandMgpMin) mgp = kDeadbandMgpMin;
                if (mgp > kDeadbandMgpMax) mgp = kDeadbandMgpMax;
                s_deadband_mgp = mgp;
            }
            // Slide the window forward.
            db_window_t0    = now_ms;
            db_window_seq0  = seq_now;
            db_window_iters = 0;
        }
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

// Rate-aware deadband live state.  Lets host probes verify policy
// is in physical units and tracks sensor rate (embeded.md §I).
//   period_ms_x10 = current sensor period × 10 (so 30 fps -> 333)
//   deadband_mgp  = current per-frame deadband in milli-grid-px
extern "C" void sentai_flow_deadband_state(uint32_t* period_ms_x10,
                                             uint32_t* deadband_mgp,
                                             uint32_t* velocity_mgp_per_s) {
    if (period_ms_x10)      *period_ms_x10      = s_period_ms_x10;
    if (deadband_mgp)       *deadband_mgp       = s_deadband_mgp;
    if (velocity_mgp_per_s) *velocity_mgp_per_s = kDeadbandVelocityMgpPerSec;
}

// =====================================================================
// OP-S10-W17-T4 — Flow SAD M7 vs M4 ablation test point.
//
// Standalone bench equivalent of sad_match — same algorithm, same
// USAD8+LD32U pattern, but with deterministic synth curr/prev pair
// (curr = gradient + central dark square, prev = same shifted by
// `shift_px` pixels in both X and Y) so the bench is reproducible
// and decoupled from camera/PXP/RGB2Y.
//
// Buffers SHARED with M4 via FLOW_BENCH_*_PTR in the m_ocram .tpu_input
// region (operator-stated 2026-05-20: "refoloseste zona de tensor de
// la M7, e ceva temporar").  Both cores read/write the SAME physical
// OCRAM address so the ablation measures pure core difference, not
// memory tier difference.  See flow_bench_shared.h for the hard-rule
// exception rationale.
//
// Anti-DCE: XOR-fold best_sad + dx + dy into a volatile sink after
// the timed region (same methodology as OP-S10-W16 M4 bench).
//
// Excludes: PXP downscale (PrepTask responsibility), RGB->Y
// conversion (PrepTask responsibility), publisher_task wrapper,
// frame_seq dedup, gray_stretch.  Pure SAD inner loop only.
// =====================================================================
#include "examples/sentai_runtime/flow_bench_shared.h"

static volatile uint32_t s_flow_test_sink = 0;
static volatile uint32_t s_flow_test_cyc  = 0;

// OP-S10-W18-T1: search-mode toggle.  Default exhaustive (production
// accuracy preserved).  Diamond search (LDSP+SDSP, Tham 1998) trades
// global-optimum guarantee for ~12-25× compute reduction.  Set via
// sentai.flow.set_search_mode("exhaustive"|"diamond").
//   0 = exhaustive ±12 (625 candidates)
//   1 = diamond search (LDSP→SDSP, ~25-50 evaluations typical)
static volatile int s_flow_search_mode = 0;

extern "C" void sentai_flow_set_search_mode(int mode) {
    s_flow_search_mode = (mode == 1) ? 1 : 0;
}
extern "C" int sentai_flow_get_search_mode(void) { return s_flow_search_mode; }

// Forward-decl SAD evaluator for diamond search (reused from sad_match).
// Computes raw SAD on the 32×32 block at offset (dx, dy) relative to
// the centered curr block.  No parabolic / conf / deadband — just SAD.
__attribute__((always_inline)) static inline uint32_t
flow_sad_at_(const uint8_t* curr, const uint8_t* prev, int dx, int dy) {
    const int bx = (FLOW_GRAY_W - kBlockW) / 2;
    const int by = (FLOW_GRAY_H - kBlockH) / 2;
    uint32_t sad = 0;
    for (int y = 0; y < kBlockH; ++y) {
        const uint8_t* c = curr + (by + y) * FLOW_GRAY_W + bx;
        const uint8_t* p = prev + (by + y + dy) * FLOW_GRAY_W + (bx + dx);
        sad = __USADA8(LD32U(c +  0), LD32U(p +  0), sad);
        sad = __USADA8(LD32U(c +  4), LD32U(p +  4), sad);
        sad = __USADA8(LD32U(c +  8), LD32U(p +  8), sad);
        sad = __USADA8(LD32U(c + 12), LD32U(p + 12), sad);
        sad = __USADA8(LD32U(c + 16), LD32U(p + 16), sad);
        sad = __USADA8(LD32U(c + 20), LD32U(p + 20), sad);
        sad = __USADA8(LD32U(c + 24), LD32U(p + 24), sad);
        sad = __USADA8(LD32U(c + 28), LD32U(p + 28), sad);
    }
    return sad;
}

// Diamond search (Tham, Ranganath, Ramakrishnan, Kasahara 1998).
// LDSP = Large Diamond Search Pattern (9 points, 2-pixel max radius)
// SDSP = Small Diamond Search Pattern (5 points, 1-pixel radius)
// Algorithm:
//   1. center = (0,0).  Evaluate 9 LDSP points centered on origin.
//   2. If best is center, go to step 4 (refinement).
//   3. Move center to best LDSP point, repeat step 1.
//   4. Evaluate 5 SDSP points around current center.  Output is best.
// Converges in 3-5 LDSP iterations + 1 SDSP for typical drone motion.
// Total evaluations: ~25-50 vs 625 for exhaustive ±12 search.
static void diamond_search_(const uint8_t* curr, const uint8_t* prev,
                              int* out_dx, int* out_dy, uint32_t* out_sad) {
    /* LDSP — 9 points, signed offsets (dx, dy) from center. */
    static const int8_t LDSP_DX[9] = { 0, -1, +1, -2,  0, +2, -1, +1,  0 };
    static const int8_t LDSP_DY[9] = {-2, -1, -1,  0,  0,  0, +1, +1, +2 };
    /* SDSP — 5 points. */
    static const int8_t SDSP_DX[5] = { 0, -1,  0, +1,  0 };
    static const int8_t SDSP_DY[5] = {-1,  0,  0,  0, +1 };

    int cx = 0, cy = 0;
    uint32_t cbest = flow_sad_at_(curr, prev, 0, 0);
    int max_ldsp_iter = 8;   /* bounded; LDSP normally converges in 3-5 */
    while (max_ldsp_iter-- > 0) {
        int   blx = 0, bly = 0;
        uint32_t blbest = cbest;
        for (int k = 0; k < 9; ++k) {
            const int dx = cx + LDSP_DX[k];
            const int dy = cy + LDSP_DY[k];
            /* Bounds check against search range (±kSearchRange = ±12). */
            if (dx < -kSearchRange || dx > kSearchRange) continue;
            if (dy < -kSearchRange || dy > kSearchRange) continue;
            /* Skip the center (we already have it as cbest). */
            if (LDSP_DX[k] == 0 && LDSP_DY[k] == 0) continue;
            const uint32_t sad = flow_sad_at_(curr, prev, dx, dy);
            if (sad < blbest) { blbest = sad; blx = LDSP_DX[k]; bly = LDSP_DY[k]; }
        }
        if (blbest >= cbest) break;   /* center remained best → SDSP phase */
        cx += blx; cy += bly;
        cbest = blbest;
    }
    /* SDSP refinement around current center. */
    int sbx = cx, sby = cy;
    uint32_t sbest = cbest;
    for (int k = 0; k < 5; ++k) {
        const int dx = cx + SDSP_DX[k];
        const int dy = cy + SDSP_DY[k];
        if (dx < -kSearchRange || dx > kSearchRange) continue;
        if (dy < -kSearchRange || dy > kSearchRange) continue;
        if (SDSP_DX[k] == 0 && SDSP_DY[k] == 0) continue;
        const uint32_t sad = flow_sad_at_(curr, prev, dx, dy);
        if (sad < sbest) { sbest = sad; sbx = dx; sby = dy; }
    }
    *out_dx = sbx; *out_dy = sby; *out_sad = sbest;
}

extern "C" uint32_t sentai_flow_test_sad(int shift_px) {
    if (shift_px < -8) shift_px = -8;
    if (shift_px >  8) shift_px =  8;
    const int W = FLOW_BENCH_GRAY_W, H = FLOW_BENCH_GRAY_H;
    uint8_t* curr = FLOW_BENCH_CURR_PTR;
    uint8_t* prev = FLOW_BENCH_PREV_PTR;
    // Synth gradient + central dark square in curr; copy to prev with
    // shift (clamping at borders).
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            uint32_t lfsr = (uint32_t)(y * W + x) * 2654435761u;
            uint8_t noise = (lfsr >> 16) & 0x1F;
            int v = 120 + (x * 60) / W + (int)noise - 8;
            if (v < 0) v = 0; if (v > 255) v = 255;
            curr[x + y * W] = (uint8_t)v;
        }
    }
    const int cx = W / 2, cy = H / 2;
    for (int y = cy - 10; y < cy + 10; ++y) {
        for (int x = cx - 10; x < cx + 10; ++x) {
            if (x >= 0 && x < W && y >= 0 && y < H) curr[x + y * W] = 30;
        }
    }
    // prev = curr shifted by shift_px in both axes (with edge clamp).
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int sx = x - shift_px; if (sx < 0) sx = 0; if (sx >= W) sx = W - 1;
            int sy = y - shift_px; if (sy < 0) sy = 0; if (sy >= H) sy = H - 1;
            prev[x + y * W] = curr[sx + sy * W];
        }
    }
    // Time the SAD core only.
    int dx = 0, dy = 0;
    uint32_t best_sad = 0;
    dwt_enable_once();
    const uint32_t t0 = dwt_now();
    if (s_flow_search_mode == 1) {
        diamond_search_(curr, prev, &dx, &dy, &best_sad);
    } else {
        sad_match(curr, prev, &dx, &dy, &best_sad);
    }
    const uint32_t t1 = dwt_now();
    s_flow_test_cyc = t1 - t0;
    // XOR-sink for DCE — best_sad / dx / dy all consumed.
    s_flow_test_sink = (uint32_t)dx ^ (uint32_t)(dy << 16) ^ best_sad;
    return s_flow_test_cyc;
}

extern "C" uint32_t sentai_flow_test_sad_cyc(void) { return s_flow_test_cyc; }

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
    // Drop any stale prev FFT so the first frame after start doesn't
    // produce bogus motion against an unrelated previous run.
    sentai_flow_phase_corr_reset();
    s_have_prev = 0;

    if (!s_pub_running && !sentai_detection_is_running()) {
        s_pub_running = true;
        // Stack 1.5 KB (* 3) was sized for SAD path which has flat
        // call tree (sad_match -> USAD8 inner loops, no nested fns).
        // Phase-corr replacement adds CMSIS arm_cfft_f32 + radix4 +
        // radix8 + bitreversal2 nesting plus 32 KB memcpy with locals.
        // 2026-05-05 reproducible crash at frame 73 with STACK_OVF
        // (code 0x0FF1, BFAR=0x666C6F77 "flow"); bumped to 4 KB
        // (* 8) with measured headroom; add uxTaskGetStackHighWaterMark
        // probe via flow.pub_health() to track in steady state.
        BaseType_t r = xTaskCreate(publisher_task_fn, "flow_pub",
                                   configMINIMAL_STACK_SIZE * 8,
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

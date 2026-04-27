/*
 * Copyright  2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "camera_support.h"
#include "fsl_gpio.h"
#include "fsl_csi.h"
#include "fsl_csi_camera_adapter.h"
#include "fsl_ov5640.h"
#include "fsl_mipi_csi2rx.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_pxp.h"
/* DWT->CYCCNT for CSI ISR latency measurement (build #924) is
 * already available — the NXP SDK pulls in the device's CMSIS core
 * header transitively via the fsl_*.h chain.  An explicit
 * `#include "core_cm7.h"` here causes a duplicate-type-definitions
 * conflict because the device file (MIMXRT1176_cm7.h) already
 * imports the same CMSIS core types under different guards. */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_CSI_CLK_FREQ          (CLOCK_GetRootClockFreq(kCLOCK_Root_Bus))
#define DEMO_MIPI_CSI2_UI_CLK_FREQ (CLOCK_GetRootClockFreq(kCLOCK_Root_Csi2_Ui))

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static status_t BOARD_VerifyCameraClockSource(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Camera connect to CSI. */
static csi_resource_t csiResource = {
    .csiBase = CSI,
    .dataBus = kCSI_DataBus24Bit,
};

static csi_private_data_t csiPrivateData;

camera_receiver_handle_t cameraReceiver = {
    .resource    = &csiResource,
    .ops         = &csi_ops,
    .privateData = &csiPrivateData,
};

static ov5640_resource_t ov5640Resource = {
    .i2cSendFunc      = BOARD_Camera_I2C_SendSCCB,
    .i2cReceiveFunc   = BOARD_Camera_I2C_ReceiveSCCB,
    .pullResetPin     = BOARD_PullCameraResetPin,
    .pullPowerDownPin = BOARD_PullCameraPowerDownPin,
};

camera_device_handle_t cameraDevice = {
    .resource = &ov5640Resource,
    .ops      = &ov5640_ops,
};

#ifdef CPU_MIMXRT1176CVM8A_cm4
__attribute__((section(".sdram_bss,\"aw\",%nobits @")))
__attribute__((aligned(DEMO_CAMERA_BUFFER_ALIGN)))
uint8_t
    framebuffers[DEMO_CAMERA_BUFFER_COUNT][DEMO_CAMERA_HEIGHT][(DEMO_CAMERA_WIDTH + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP];
#else
__attribute__((section("NonCacheableCamera,\"aw\",%nobits @")))
__attribute__((aligned(DEMO_CAMERA_BUFFER_ALIGN)))
uint8_t
    framebuffers[DEMO_CAMERA_BUFFER_COUNT]
        [(DEMO_CAMERA_HEIGHT) * (DEMO_CAMERA_WIDTH + LINE_PADDING) * DEMO_CAMERA_BUFFER_BPP];
#endif

/* Monotonic frame counter — incremented by CSI ISR on each DMA frame
 * completion.  Never reset (stays monotonic across camera switches).
 * Read freely from any context (aligned uint32_t read is atomic on
 * Cortex-M7).  Wraps at 2^32 (~9 years at 15fps). */
volatile uint32_t g_camera_frame_seq = 0;

/* sentai 2026-04-25 source tagging: snapshot of g_cam_current_id at the
 * moment the most recent buffer COMPLETED filling.  Captured BEFORE the
 * MUX flip in CSI ISR, so it reflects the camera that actually wrote
 * the just-completed buffer.  Differs from g_cam_current_id which
 * reflects the MUX state for the NEXT capture.
 * Reader: sentai.camera.last_capture_id(); writer: CSI ISR (single store
 * per FB2-done, atomic on Cortex-M7).
 */
volatile int g_cam_last_completed_id = 0;

/* Per-buffer source tagging — DEPRECATED ISR-side path.
 *
 * Visual A/B (build #885) revealed this array carries shuffled tags
 * because in BASEADDR_SWITCH mode `DMASA_FBn` may be advanced by
 * hardware to the next slot before the IRQ samples it, so the ISR
 * write `g_cam_buf_id[FramebufferPtrToIndex(fb_addr)] = active_cam`
 * lands on the WRONG slot ~half the time.  Kept for diagnostics,
 * NOT trusted by consumers.  See `g_cam_buf_id_task[]` below. */
volatile uint8_t g_cam_buf_id[DEMO_CAMERA_BUFFER_COUNT] = {
    0xFF, 0xFF, 0xFF, 0xFF};

/* Authoritative per-buffer source tagging — task-context (build #886+).
 *
 * Written by `HandleFrameRequest` (CameraTask, in libs/camera/camera.cc)
 * AFTER `CAMERA_RECEIVER_GetFullBuffer` succeeds: `(idx, source)` are
 * both known reliably at that point — `idx` from the actual buffer
 * pointer the driver dequeued, `source` from `g_cam_last_completed_id`
 * (set by ISR at FB-done with the camera that filled the buffer that
 * just completed).  No DMASA snapshot dependency.
 *
 * Read by `sentai_cam_get_raw_with_recovery` to populate
 * `g_cam_grabbed_id`.  Single writer (CameraTask) + multiple readers
 * (sentai_runtime.cc, MP bindings) — single-store atomic on
 * Cortex-M7, no lock needed.  0xFF = never tagged.  */
volatile uint8_t g_cam_buf_id_task[DEMO_CAMERA_BUFFER_COUNT] = {
    0xFF, 0xFF, 0xFF, 0xFF};

/* Tag of the most recently grabbed buffer; updated by
 * sentai_cam_get_raw_with_recovery when it returns. */
volatile int g_cam_grabbed_id = -1;

/* Build #958 — runtime fps.  Replaces compile-time
 * DEMO_CAMERA_FRAME_RATE in BOARD_InitCamera so sentai_cam_set_fps
 * can re-init at a different rate without rebuild + reflash. */
volatile uint32_t g_runtime_fps = DEMO_CAMERA_FRAME_RATE;

/* Build #942 — IN-FLIGHT-AT-FLIP dirty-buffer marker.
 *
 * When the CSI ISR flips the MUX, the buffer currently being filled
 * by the CSI receiver carries a mid-frame mix: top rows from old
 * camera (recorded before the flip), bottom rows from new camera
 * (continued after the flip).  This buffer is unrecoverable — the
 * consumer must skip it and read the NEXT one (the first one that
 * starts cleanly at the new camera's SOF after the flip).
 *
 *  - g_cam_dirty_pending: set true by the post-flip block of
 *    CSI_IRQHandler.  Consumed (mark + clear) by the next FB-done
 *    block in the SAME or following IRQ.
 *  - g_cam_buf_dirty[i]:  per-slot dirty bit.  Set by the IRQ at
 *    FB-done time for the buffer that was in-flight at flip.
 *    Cleared by sentai_cam_get_raw_with_recovery when it discards
 *    the buffer.
 *  - g_cam_buf_dirty_marks: total times we set a dirty bit (one
 *    per MUX flip in steady state).
 *  - g_cam_buf_dirty_skips: total times consumer discarded a dirty
 *    buffer (matches dirty_marks if no buffers leaked). */
/* Build #944 — extended to a COUNTDOWN.  Set to N at MUX flip to
 * mark the next N FB-done buffers as dirty (covers second-order
 * CSI re-sync transients beyond the immediate in-flight buffer).
 * Decremented per FB-done that consumes a mark.  Default kCamDirtyConsecutive
 * is 2 (one for the in-flight buffer at flip + one for the
 * post-flip re-sync buffer).  bool semantics preserved by treating
 * non-zero as "pending". */
volatile uint32_t g_cam_dirty_pending_count = 0u;
/* Build #952 — runtime-settable.  Pattern_31 sweep N=1..6 to find
 * the sweet spot at the user's ratio.  Default N=1 keeps the
 * baseline behaviour. */
volatile uint32_t g_cam_dirty_consecutive_n = 1u;
volatile uint8_t  g_cam_buf_dirty[DEMO_CAMERA_BUFFER_COUNT] = {0, 0, 0, 0};
volatile uint32_t g_cam_buf_dirty_marks    = 0;
volatile uint32_t g_cam_buf_dirty_skips    = 0;
/* DIAGNOSTIC counters for cam_id tagging path (build #881+).
 * Bumped from CSI ISR when an FB-done event is observed.
 *  - g_cam_buf_tag_writes   : successful writes to g_cam_buf_id[idx]
 *  - g_cam_buf_tag_idx_miss : FramebufferPtrToIndex returned -1 or OOB
 * Single-store atomics; safe ISR observability.  Confirms whether the
 * tag path is reached at all. */
volatile uint32_t g_cam_buf_tag_writes   = 0;
volatile uint32_t g_cam_buf_tag_idx_miss = 0;
/* Task-side counters for the authoritative tag path (build #886+).
 *  - g_cam_buf_task_writes  : HandleFrameRequest tagged a slot
 *  - g_cam_buf_task_unknown : g_cam_last_completed_id was -1 at tag time */
volatile uint32_t g_cam_buf_task_writes  = 0;
volatile uint32_t g_cam_buf_task_unknown = 0;
/* Counter for ambiguous-IRQ case (both FB1+FB2 done flags set).
 * NXP driver skips both buffers; we skip tag writes.  Bumped per
 * such event for diagnostic visibility. */
volatile uint32_t g_cam_buf_tag_skip_both = 0;

/* VBLANK-gated MUX flip diagnostics (build #905+).
 * g_cam_vblank_flips      : MUX flips committed inside VBLANK
 *                           (sof_seen_pre==false && SR_SOF_INT==0).
 * g_cam_flips_deferred    : total IRQ events where the flip was
 *                           deferred because the gate detected an
 *                           SOF in/before the ISR.
 * g_cam_flips_deferred_streak : consecutive deferrals for the
 *                           currently-pending flip; bounded by
 *                           kVblankGateMaxDefer (=3 IRQs).
 * g_cam_flips_forced      : flips committed because the streak
 *                           exceeded kVblankGateMaxDefer despite
 *                           SOF being set — diagnostic-only counter,
 *                           a non-zero value means VBLANK gate is
 *                           starving and we degraded to the legacy
 *                           "flip anyway" behaviour for safety. */
volatile uint32_t g_cam_vblank_flips         = 0;
volatile uint32_t g_cam_flips_deferred       = 0;
volatile uint32_t g_cam_flips_deferred_streak = 0;
volatile uint32_t g_cam_flips_forced         = 0;

/* Build #924 — CSI ISR latency instrumentation (M7 @ 800 MHz).
 * Sampled via DWT->CYCCNT at ISR entry and exit; (exit - entry)
 * converted to microseconds via /800.  Goal: measure whether our
 * ISR runs inside the sensor's VBLANK or sometimes leaks into the
 * next active frame (which would explain the 10 % residual mistag
 * rate at alt 3:1 — see agent.md §9.2).
 *
 * Expose to MicroPython via sentai.camera.flip_stats() so a host
 * test can dump distribution after a pattern_31 run, no extra
 * driver-side timing needed. */
volatile uint32_t g_csi_isr_count        = 0;
volatile uint32_t g_csi_isr_dur_us_last  = 0;
volatile uint32_t g_csi_isr_dur_us_max   = 0;
volatile uint32_t g_csi_isr_dur_us_sum   = 0;  /* /count = avg us */
/* 6-bucket histogram of ISR durations (microseconds):
 *   [0]: 0..49        [1]: 50..99
 *   [2]: 100..199     [3]: 200..499
 *   [4]: 500..999     [5]: 1000+   (likely > VBLANK at VGA45) */
volatile uint32_t g_csi_isr_hist[6]      = {0, 0, 0, 0, 0, 0};

/* Counter for the authoritative NXP-CSI-driver hook firings.
 * Bumped from inside fsl_csi.c only when the driver decides a frame
 * is real and inserts it into the user-visible queue (line 888).
 * If this matches the actual sensor frame count, our tag writes are
 * landing on EXACTLY the buffers the consumer will dequeue. */
volatile uint32_t g_cam_csi_hook_fires = 0;

/* AUTHORITATIVE per-buffer source tag — written from the NXP CSI
 * driver hook called at fsl_csi.c:889 when a real frame is queued
 * for the consumer.  Indexed by slot (0..3) found by looking up the
 * buffer address in framebuffers[].  Read by
 * sentai_cam_get_raw_with_recovery for grabbed_id reporting.
 * 0xFF = never tagged. */
volatile uint8_t g_cam_buf_id_csi[DEMO_CAMERA_BUFFER_COUNT] = {
    0xFF, 0xFF, 0xFF, 0xFF};

/* NXP CSI driver hook — invoked from fsl_csi.c with the address of
 * the buffer that was JUST validated as a real frame and inserted
 * into the user-visible queue.  This is the single
 * point where (slot, frame_done_event) are guaranteed to align.
 * Single-store atomic; no locks needed (CSI ISR context only).  */
#ifndef CPU_MIMXRT1176CVM8A_cm4
extern volatile int g_cam_current_id;
extern int FramebufferPtrToIndex(const uint8_t* framebuffer_ptr);

/* Build #944: counter-only hooks, placed in ITCM (.ramfunc) per
 * agent.md §2.2 — these are called from inside the NXP CSI ISR
 * and are now on the hot path. */
__attribute__((section(".ramfunc")))
void coralmicro_csi_on_frame_complete(uint32_t bufferAddr) {
    g_cam_csi_hook_fires++;
    (void)bufferAddr;
}

volatile uint32_t g_cam_buf_arm_writes  = 0;
volatile uint32_t g_cam_buf_arm_idx_miss = 0;
__attribute__((section(".ramfunc")))
void coralmicro_csi_on_buffer_arm(uint32_t bufferAddr) {
    int slot = FramebufferPtrToIndex((const uint8_t*)bufferAddr);
    if (slot >= 0 && slot < (int)DEMO_CAMERA_BUFFER_COUNT) {
        g_cam_buf_arm_writes++;
    } else {
        g_cam_buf_arm_idx_miss++;
    }
    (void)bufferAddr;
}
#else
/* M4 stub: g_cam_current_id is M7-only.  M4 doesn't drive cameras
 * directly; provide no-ops so fsl_csi.c links cleanly on M4 too. */
void coralmicro_csi_on_frame_complete(uint32_t bufferAddr) {
    (void)bufferAddr;
}
void coralmicro_csi_on_buffer_arm(uint32_t bufferAddr) {
    (void)bufferAddr;
}
#endif

/* ---------------------------------------------------------------------------
 * Glitch-free dual-camera switching — "flip on EOF"
 *
 * Armed from sentai_cam_switch() (task context) as an int in {-1, 0, 1}.
 * The CSI ISR below consumes it immediately after g_camera_frame_seq++,
 * which is the exact moment a DMA buffer has just finished filling and
 * the MIPI lane is idle until the next SOF — i.e. we are in VBLANK.
 * Flipping the analogue MUX here guarantees the next DMA buffer is
 * filled 100% by the new sensor; no mid-buffer seam.
 *
 * - g_cam_pending_mux_id : -1 when no switch pending, 0/1 = target cam
 * - g_cam_switch_seq     : written here right after the flip; used by
 *                          sentai_cam_get_raw_with_recovery to detect
 *                          fresh frames from the new sensor
 * - g_cam_switch_pending : set here; cleared by the recovery path after
 *                          the drain completes
 *
 * Owned by sentai_runtime.cc (extern below) — ISR is writer, task-side
 * cam_switch is reader/arm-er.  All three are aligned volatile and
 * single-word, so updates are atomic on Cortex-M7; no lock needed.
 * --------------------------------------------------------------------------- */
extern volatile int      g_cam_pending_mux_id;
extern volatile uint32_t g_cam_switch_seq;
extern volatile bool     g_cam_switch_pending;
extern volatile int      g_cam_current_id;
/* Stateless ratio-alternate scheduler.  Single packed 32-bit word so
 * the ISR gets an atomic snapshot of (a, b) — see sentai_runtime.cc.
 * Layout: high 16 bits = a (cam0 quota), low 16 = b (cam1 quota). */
extern volatile uint32_t g_cam_ratio_packed;

/* Dedicated ISR-safe MUX-flip helper provided by libs/base/gpio.cc —
 * uses atomic DR_SET / DR_CLEAR registers instead of taking g_mutex.
 * MUX polarity shared with camera.cc via cam_mux.h (single source of
 * truth — flipping that header flips both call sites consistently). */
#include "cam_mux.h"
extern void SentaiCamMuxSetFromIsr(bool enable);

/*******************************************************************************
 * Code
 ******************************************************************************/
extern void CSI_DriverIRQHandler(void);

#ifndef CPU_MIMXRT1176CVM8A_cm4
/* CSI_IRQHandler is M7-only — references globals defined in
 * sentai_runtime.cc (M7 binary).  M4 doesn't drive the camera.
 *
 * Section .ramfunc => placed in ITCM (m_text) by linker — single-
 * cycle instruction fetch, no SEMC bus traffic.  Critical because
 * this ISR fires at sensor rate (30-90 Hz) AND contains the early-
 * VBLANK MUX flip whose latency directly affects whether the flip
 * lands in the brief VBLANK window.  Previous default placement
 * (SDRAM via .camera section in linker script) added ~200 ns
 * instruction-fetch latency per branch + bus contention with CSI
 * DMA itself.  See git log for the empirical mid-frame mix bug
 * (build #904 visual A/B i6) that motivated this move. */
__attribute__((section(".ramfunc")))
void CSI_IRQHandler(void)
{
    /* 2026-04-22: gate the counter on the FB2-done flag only.
     *
     * NXP's CSI in BASEADDR_SWITCH mode alternates whole sensor
     * frames between FB1 and FB2 (fsl_csi.c:692-702).  Each IRQ
     * carries one flag.  But under active buffer drain the re-arm
     * path (fsl_csi.c:910-917) causes IRQs to fire about twice per
     * sensor period — we measured 87 Hz with the sensor at 45 Hz.
     *
     * Since FB1-done and FB2-done alternate 1:1 with true sensor
     * frames, gating the counter on either ONE of them gives a
     * stable "one tick per 2 sensor frames" rhythm if we pick FB2,
     * or we match sensor rate exactly if both flags always pair up.
     *
     * Concrete consumers of g_camera_frame_seq:
     *   - post-MUX-switch drain loop  (wants 2 fresh sensor frames)
     *   - ratio-alternate scheduler   (wants uniform cadence)
     *   - PrepTask label s_stg_frame_seq (wants monotonic label)
     *   - flow_task.cc frame_seq stamp (dedup motion samples)
     * None care about raw IRQ count — they all want "per sensor
     * frame" cadence.  Gating solves the drain-threshold off-by-2
     * bug latent in the old code. */
    uint32_t isr_t0 = DWT->CYCCNT;
    uint32_t sr_at_entry = CSI_REG_SR(CSI);
    bool fb2_done = (0U != (sr_at_entry & CSI_SR_DMA_TSF_DONE_FB2_MASK));
    bool fb1_done = (0U != (sr_at_entry & CSI_SR_DMA_TSF_DONE_FB1_MASK));
    /* Build #919-#925 had a SOF-based VBLANK gate here that deferred
     * the MUX flip when SOF fired during the ISR.  Build #932 removed
     * the gate logic — empirical measurement (DWT->CYCCNT histogram
     * over 218 IRQs across two runs) showed ISR latency caps at 2 µs
     * vs ~3000 µs of VBLANK at VGA45.  The gate never deferred a
     * single flip, so it was pure complexity per embeded.md §J.
     * The `g_cam_flips_deferred` / _streak / _forced counters are
     * retained as dead-zero diagnostics for future profiling, in
     * case a future workload extends the ISR. */
    uint32_t fb1_addr = CSI_REG_DMASA_FB1(CSI);
    uint32_t fb2_addr = CSI_REG_DMASA_FB2(CSI);

    /* ITCM-resident ISR (build #905+) — function placed in
     * .ramfunc → m_text (ITCM) for single-cycle instruction fetch.
     * SDRAM placement caused ~200 ns latency per branch; ITCM
     * eliminates that.  ISR ordering kept identical to the
     * stable build #904: NXP driver first, tag writes, then MUX
     * flip last.  Reorg attempts to flip earlier destabilised the
     * board (NXP CSI driver assumes MUX state stable for its
     * queue logic). */
    /* Diagnostic — IRQ delivery jitter signal.  NXP's
     * CSI_DriverIRQHandler (line 864 of fsl_csi.c) skips both
     * buffers when BOTH FB1+FB2 done flags are set at entry — that
     * happens only when our ISR was so delayed that two sensor
     * frames completed before we got serviced.  Counting these
     * tells us whether USB / TPU / other higher-priority ISRs are
     * preempting CSI for a duration > 1 sensor frame (~22 ms at
     * VGA45). Counter writeable from this ISR only — no race. */
    if (fb1_done && fb2_done) {
        g_cam_buf_tag_skip_both++;
    }

    CSI_DriverIRQHandler();
    __DSB();

    /* Build #944 — countdown of pending dirty marks.  Set to N at
     * MUX flip; decremented per FB-done that consumes a mark.
     * NASA §11: ISR is the single writer (this site + post-flip),
     * single read here, atomic 32-bit store. */
    uint32_t pend_in = g_cam_dirty_pending_count;
    bool dirty_now = (pend_in > 0u);
    int active_cam = g_cam_current_id;
    if (fb2_done) g_camera_frame_seq++;
    /* Bounded 2-iteration loop (NASA §1, §2). */
    uint32_t pend_consumed = 0u;
    for (int k = 0; k < 2; ++k) {
        bool done   = (k == 0) ? fb1_done : fb2_done;
        uint32_t a  = (k == 0) ? fb1_addr : fb2_addr;
        if (!done) continue;
        g_cam_last_completed_id = active_cam;
        int idx = FramebufferPtrToIndex((const uint8_t*)a);
        if (idx < 0 || idx >= (int)DEMO_CAMERA_BUFFER_COUNT) {
            g_cam_buf_tag_idx_miss++;
            continue;
        }
        g_cam_buf_id[idx] = (uint8_t)active_cam;
        g_cam_buf_tag_writes++;
        if (dirty_now && pend_in - pend_consumed > 0u) {
            g_cam_buf_dirty[idx] = 1u;
            g_cam_buf_dirty_marks++;
            pend_consumed++;
        } else {
            /* Build #953 — clear stale dirty bit on a fresh fill.
             * Without this, a dirty mark from an earlier flip
             * lingers across buffer reuse cycles and makes the
             * consumer skip CLEAN data; the consumer then falls
             * through to the blocking-grab path that never checked
             * dirty, returning a different buffer that might have
             * BEEN dirty (the in-flight at THIS flip).  Net effect:
             * scrambled buffers slip through despite the marker.
             * Clearing here is safe: we are in the FB-done IRQ for
             * THIS slot, holding the only writer of g_cam_buf_dirty
             * for this slot in this instant. */
            g_cam_buf_dirty[idx] = 0u;
        }
    }
    if (pend_consumed > 0u) {
        g_cam_dirty_pending_count = pend_in - pend_consumed;
    }

    /* Stateless ratio scheduler. */
    uint32_t packed = g_cam_ratio_packed;
    uint32_t ra = (packed >> 16) & 0xFFFFu;
    uint32_t rb = packed & 0xFFFFu;
    uint32_t total = ra + rb;
    if (total > 0u && g_cam_pending_mux_id < 0) {
        uint32_t pos = g_camera_frame_seq % total;
        int target = (pos < ra) ? 0 : 1;
        if (target != g_cam_current_id) {
            g_cam_pending_mux_id = target;
        }
    }

    /* Consume pending MUX flip in VBLANK (build #932 simplified).
     *
     * No software VBLANK gate — empirical CSI ISR latency is 2 µs
     * peak, ~0 µs avg (DWT cycle counter, build #925 measurement).
     * That's a 1500× margin against the ~3 ms sensor VBLANK at
     * VGA45, so the flip ALWAYS lands in VBLANK as long as the ISR
     * gets serviced (NVIC priority + ITCM placement guarantees).
     * Keeping the post-flip re-tag — empirically load-bearing for
     * the multi-writer architecture (build #911 vs #912 diff). */
    int pending = g_cam_pending_mux_id;
    if (pending >= 0) {
        bool level = (CAM_MUX_LEVEL_FOR_ID(pending) != 0);
        SentaiCamMuxSetFromIsr(level);
        g_cam_switch_seq     = g_camera_frame_seq;
        g_cam_switch_pending = true;
        g_cam_current_id     = pending;
        g_cam_pending_mux_id = -1;
        g_cam_vblank_flips++;
        /* Build #952 — runtime-settable via g_cam_dirty_consecutive_n. */
        g_cam_dirty_pending_count = g_cam_dirty_consecutive_n;
    }

    /* Build #942: ISR-exit timing instrumentation removed (#925
     * established peak = 2 µs across thousands of IRQs; the 1500×
     * margin vs the ~3 ms VBLANK at VGA45 settled the question, so
     * the histogram pays no further cost).  The DWT->CYCCNT read
     * stays in case future profiling needs it. */
    (void)isr_t0;
}
#endif /* !CPU_MIMXRT1176CVM8A_cm4 */

void BOARD_EarlyInitCamera(void)
{
    /* If the camera I2C bus should be released by sending I2C sequence,
     * add the code here.
     */
}

void BOARD_InitCameraResource(void)
{
    // BOARD_Camera_I2C_Init();

    /* CSI MCLK is connect to dedicated 24M OSC, so don't need to configure it. */
}

void BOARD_InitMipiCsi(void)
{
    csi2rx_config_t csi2rxConfig = {0};

    /* This clock should be equal or faster than the receive byte clock,
     * D0_HS_BYTE_CLKD, from the RX DPHY. For this board, there are two
     * data lanes, the MIPI CSI pixel format is 16-bit per pixel, the
     * max resolution supported is 720*1280@30Hz, so the MIPI CSI2 clock
     * should be faster than 720*1280*30 = 27.6MHz, choose 60MHz here.
     */
    const clock_root_config_t csi2ClockConfig = {
        .clockOff = false,
        .mux      = 5,
        .div      = 8,
    };

    /* ESC clock should be in the range of 60~80 MHz */
    const clock_root_config_t csi2EscClockConfig = {
        .clockOff = false,
        .mux      = 5,
        .div      = 8,
    };

    /* UI clock should be equal or faster than the input pixel clock.
     * The camera max resolution supported is 720*1280@30Hz, so this clock
     * should be faster than 720*1280*30 = 27.6MHz, choose 60MHz here.
     */
    const clock_root_config_t csi2UiClockConfig = {
        .clockOff = false,
        .mux      = 5,
        .div      = 8,
    };

    if (kStatus_Success != BOARD_VerifyCameraClockSource())
    {
        PRINTF("MIPI CSI clock source not valid\r\n");
        while (1)
        {
        }
    }

    /* MIPI CSI2 connect to CSI. */
    CLOCK_EnableClock(kCLOCK_Video_Mux);
    VIDEO_MUX->VID_MUX_CTRL.SET = (VIDEO_MUX_VID_MUX_CTRL_CSI_SEL_MASK);

    CLOCK_SetRootClock(kCLOCK_Root_Csi2, &csi2ClockConfig);
    CLOCK_SetRootClock(kCLOCK_Root_Csi2_Esc, &csi2EscClockConfig);
    CLOCK_SetRootClock(kCLOCK_Root_Csi2_Ui, &csi2UiClockConfig);

    /* The CSI clock should be faster than MIPI CSI2 clk_ui. The CSI clock
     * is bus clock.
     */
    if (DEMO_CSI_CLK_FREQ < DEMO_MIPI_CSI2_UI_CLK_FREQ)
    {
        PRINTF("CSI clock should be faster than MIPI CSI2 ui clock.\r\n");
        while (1)
        {
        }
    }

    /* MIPI DPHY power on and isolation off. */
    PGMC_BPC4->BPC_POWER_CTRL |= (PGMC_BPC_BPC_POWER_CTRL_PSW_ON_SOFT_MASK | PGMC_BPC_BPC_POWER_CTRL_ISO_OFF_SOFT_MASK);

    /*
     * Initialize the MIPI CSI2
     *
     * From D-PHY specification, the T-HSSETTLE should in the range of 85ns+6*UI to 145ns+10*UI
     * UI is Unit Interval, equal to the duration of any HS state on the Clock Lane
     *
     * T-HSSETTLE = csi2rxConfig.tHsSettle_EscClk * (Tperiod of RxClkInEsc)
     *
     * csi2rxConfig.tHsSettle_EscClk setting for camera:
     *
     *    Resolution  |  frame rate  |  T_HS_SETTLE
     *  =============================================
     *     720P       |     30       |     0x12
     *  ---------------------------------------------
     *     720P       |     15       |     0x17
     *  ---------------------------------------------
     *      VGA       |     30       |     0x1F
     *  ---------------------------------------------
     *      VGA       |     15       |     0x24
     *  ---------------------------------------------
     *     QVGA       |     30       |     0x1F
     *  ---------------------------------------------
     *     QVGA       |     15       |     0x24
     *  ---------------------------------------------
     */
    static const uint32_t csi2rxHsSettle[][3] = {
        {
            /* sentai 2026-04-25: VGA 640x480 @ 90 fps.  Pixel rate
             * 27.6 MP/s = identical to 720P/30 lane rate -> use 0x12
             * (same as 720P/30 entry below). */
            kVIDEO_ResolutionVGA,
            90,
            0x12,
        },
        {
            /* sentai 2026-04-25: VGA 640x480 @ 60 fps.  Pixel rate
             * 18.4 MP/s, between VGA45 (13.8) and VGA90 (27.6) ->
             * use 0x16 (between VGA45's 0x18 and 720P/30's 0x12). */
            kVIDEO_ResolutionVGA,
            60,
            0x16,
        },
        {
            /* sentai 2026-04-25: SXGA 1280x960 @ 15 fps.  Pixel rate
             * 18.4 MP/s, between 720P/15 (13.8) and 1080P/15 (31.1) ->
             * pick 0x14 (between 0x17 and 0x12). */
            FSL_VIDEO_RESOLUTION(1280, 960),
            15,
            0x14,
        },
        {
            /* sentai 2026-04-25: SXGA 1280x960 @ 30 fps.  Pixel rate
             * 36.9 MP/s, slightly above 720P/30 (27.6) -> use 0x12. */
            FSL_VIDEO_RESOLUTION(1280, 960),
            30,
            0x12,
        },
        {
            kVIDEO_Resolution1080P,
            15,
            0x12,
        },
        {
            kVIDEO_Resolution720P,
            30,
            0x12,
        },
        {
            kVIDEO_Resolution720P,
            15,
            0x17,
        },
        {
            kVIDEO_ResolutionVGA,
            30,
            0x1F,
        },
        {
            /* sentai VGA @ 45 fps (2nd attempt) — paired with the
             * VGA/45 row in fsl_ov5640.c (pllCtrl2=0x54 PLL mult 84,
             * 1.5× VGA/30's mult 56).  T-HSSETTLE shorter because
             * lane rate scales with PLL; 0x18 = 3/4 of VGA/30's
             * 0x1F. Tune down to 0x15 / 0x12 if lane-sync is robust. */
            kVIDEO_ResolutionVGA,
            45,
            0x18,
        },
        {
            /* sentai VGA @ 60 fps — lane rate 2× VGA/30; T-HSSETTLE
             * interpolated downward: 0x1F × 1/2 ≈ 0x10.  Start at 0x12
             * (same as 720p/30 which worked).  Paired with VGA/60 row
             * in fsl_ov5640.c. */
            kVIDEO_ResolutionVGA,
            60,
            0x12,
        },
        {
            kVIDEO_ResolutionVGA,
            15,
            0x24,
        },
        {
            kVIDEO_ResolutionQVGA,
            30,
            0x1F,
        },
        {
            kVIDEO_ResolutionQVGA,
            15,
            0x24,
        },
    };

    csi2rxConfig.laneNum          = DEMO_CAMERA_MIPI_CSI_LANE;
    csi2rxConfig.tHsSettle_EscClk = 0x12;

    /* Lookup keyed on the RUNTIME fps (g_runtime_fps), not the
     * compile-time DEMO_CAMERA_FRAME_RATE.  When sentai_cam_init_fps()
     * boots the camera at a non-default rate (e.g. 45 fps), the MIPI
     * D-PHY lane rate scales with the OV5640 PLL and the receiver
     * needs the matching T-HSSETTLE — otherwise the first MIPI sync
     * lands outside the lane-settling window, the receiver mis-samples,
     * and the warm-up `select(); select(); select()` sequence dead-
     * locks (CSI ISR never sees a clean EOF, drain timeout fires,
     * fallback path takes the mutex while ISR is mid-recovery →
     * REPL wedge).  Caught 2026-04-26 build #986 retro by diffing
     * `80d574c9 "45 fps stabil si switch"` against current tree:
     * stable build paired the table row with the macro by setting
     * DEMO_CAMERA_FRAME_RATE=45 at compile time. */
    for (uint8_t i = 0; i < ARRAY_SIZE(csi2rxHsSettle); i++)
    {
        if ((FSL_VIDEO_RESOLUTION(DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT) == csi2rxHsSettle[i][0]) &&
            (csi2rxHsSettle[i][1] == g_runtime_fps))
        {
            csi2rxConfig.tHsSettle_EscClk = csi2rxHsSettle[i][2];
            break;
        }
    }

    CSI2RX_Init(MIPI_CSI2RX, &csi2rxConfig);
}

static status_t BOARD_VerifyCameraClockSource(void)
{
    status_t status;
    uint32_t srcClkFreq;
    /*
     * The MIPI CSI clk_ui, clk_esc, and core_clk are all from
     * System PLL3 (PLL_480M). Verify the clock source to ensure
     * it is ready to use.
     */
    srcClkFreq = CLOCK_GetPllFreq(kCLOCK_PllSys3);

    if (480 != (srcClkFreq / 1000000))
    {
        status = kStatus_Fail;
    }
    else
    {
        status = kStatus_Success;
    }

    return status;
}

void BOARD_InitPxp(void)
{
    /*
     * Configure the PXP for rotate and scale.
     */
    PXP_Init(DEMO_PXP);

    PXP_SetProcessSurfaceBackGroundColor(DEMO_PXP, 0U);

#if DEMO_ROTATE_FRAME
    PXP_SetProcessSurfacePosition(DEMO_PXP, 0U, 0U, DEMO_BUFFER_HEIGHT - 1U, DEMO_BUFFER_WIDTH - 1U);
#else
    PXP_SetProcessSurfacePosition(DEMO_PXP, 0U, 0U, DEMO_BUFFER_WIDTH - 1U, DEMO_BUFFER_HEIGHT - 1U);
#endif

    /* Disable AS. */
    PXP_SetAlphaSurfacePosition(DEMO_PXP, 0xFFFFU, 0xFFFFU, 0U, 0U);

    PXP_EnableCsc1(DEMO_PXP, false);
}

void BOARD_InitCamera(void)
{
    status_t status;
    camera_config_t cameraConfig;

    memset(&cameraConfig, 0, sizeof(cameraConfig));

    BOARD_InitCameraResource();

    /* CSI receiver stores XRGB8888 (32-bit per pixel).  Required for
     * fsl_csi.c CSI_Init() to enable CR18.PARALLEL24_EN — the 24-bit
     * parallel data path that matches MIPI CSI2RX's default RGB888
     * output (after sensor RGB565 -> upconversion in MIPI pipeline).
     * RGB565 receiver attempted 2026-04-25 but produced 2x2-tile
     * garbage; see header comment for details. */
    cameraConfig.pixelFormat                = kVIDEO_PixelFormatXRGB8888;
    cameraConfig.bytesPerPixel              = DEMO_CAMERA_BUFFER_BPP;
    cameraConfig.resolution                 = FSL_VIDEO_RESOLUTION(DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT);
    cameraConfig.frameBufferLinePitch_Bytes = DEMO_CAMERA_WIDTH * DEMO_CAMERA_BUFFER_BPP;
    cameraConfig.interface                  = kCAMERA_InterfaceGatedClock;
    cameraConfig.controlFlags               = DEMO_CAMERA_CONTROL_FLAGS;
    cameraConfig.framePerSec                = g_runtime_fps;

    status = CAMERA_RECEIVER_Init(&cameraReceiver, &cameraConfig, NULL, NULL);
    printf("CAMERA_RECEIVER_Init = %ld\r\n", status);

    /* sentai TPU-pipeline fix (2026-04-22): CSI_IRQHandler runs at
     * the ARM NVIC default priority = 0 = HIGHER than USB_OTG2
     * (which is at configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 2).
     * At 45 fps + a few µs of ISR work, CSI steadily preempts
     * USB_OTG2 completion handling, causing ~43% of bulk URBs to
     * miss their 50 ms sema timeout under pipeline load.  The CSI
     * handler does NOT call FreeRTOS APIs (only GPIO + atomic
     * counter increments), so it is safe to move BELOW the FreeRTOS
     * syscall priority boundary.  Priority 5 puts CSI below USB
     * (2) so USB IOC handling is never preempted by the camera. */
    NVIC_SetPriority(CSI_IRQn, 5);

    BOARD_InitMipiCsi();

    cameraConfig.pixelFormat   = kVIDEO_PixelFormatRGB565;
    cameraConfig.bytesPerPixel = 2;
    cameraConfig.resolution    = FSL_VIDEO_RESOLUTION(DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT);
    cameraConfig.interface     = kCAMERA_InterfaceMIPI;
    cameraConfig.controlFlags  = DEMO_CAMERA_CONTROL_FLAGS;
    cameraConfig.framePerSec   = g_runtime_fps;
    cameraConfig.csiLanes      = DEMO_CAMERA_MIPI_CSI_LANE;

    status = CAMERA_DEVICE_Init(&cameraDevice, &cameraConfig);
    printf("CAMERA_DEVICE_Init = %ld\n", status);

    status = CAMERA_DEVICE_Start(&cameraDevice);
    printf("CAMERA_DEVICE_Start = %ld\n", status);

    /* Submit the empty frame buffers to buffer queue. */
    for (uint32_t i = 0; i < DEMO_CAMERA_BUFFER_COUNT; i++)
    {
        status = CAMERA_RECEIVER_SubmitEmptyBuffer(&cameraReceiver, (uint32_t)(framebuffers[i]));
        // printf("CAMERA_RECEIVER_SubmitEmptyBuffer (%08lX) = %ld\n", (uint32_t)framebuffers[i], status);
    }
}

void BOARD_PxpConfig(void)
{
    PXP_SetProcessSurfaceBackGroundColor(DEMO_PXP, 0);
    /* Rotate and scale the camera input to fit display output. */
#if DEMO_ROTATE_FRAME
    /* The PS rotate and scale could not work at the same time, so rotate the output. */
    PXP_SetRotateConfig(DEMO_PXP, kPXP_RotateOutputBuffer, kPXP_Rotate90, kPXP_FlipDisable);
    PXP_SetProcessSurfaceScaler(DEMO_PXP, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, DEMO_BUFFER_HEIGHT, DEMO_BUFFER_WIDTH);
#else
    PXP_SetProcessSurfaceScaler(DEMO_PXP, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, DEMO_BUFFER_WIDTH, DEMO_BUFFER_HEIGHT);
#endif
}

uint8_t* IndexToFramebufferPtr(int index) {
  if (index < 0 || index >= DEMO_CAMERA_BUFFER_COUNT) {
    return NULL;
  }
  return (uint8_t*)(framebuffers[index]);
}

/* Build #925 — kept in SDRAM (m_text full from base CSI ISR + fsl_csi
 * driver already in ITCM).  Moving this here was tried in #922 and
 * gave no measurable accuracy improvement; reverted to free m_text
 * for the timing instrumentation.  If a future profiling round
 * shows the SEMC fetch is on the hot path, revisit by also moving
 * something else OUT of ITCM. */
int FramebufferPtrToIndex(const uint8_t* framebuffer_ptr) {
  for (int i = 0; i < DEMO_CAMERA_BUFFER_COUNT; ++i) {
    if ((uint8_t*)(framebuffers[i]) == framebuffer_ptr) {
      return i;
    }
  }
  return -1;
}

typedef struct {
	uint16_t start;
	uint16_t end;
} reg_range_t;

void CamDumpRegistersOnly(void)
{
    uint8_t val;
    reg_range_t ov5640_regs[] = {
            {0x3000, 0x3052},
            {0x3100, 0x3108},
            {0x3200, 0x3211},
            {0x3400, 0x3406},
            {0x3500, 0x350D},
            {0x3600, 0x3606},
            {0x3800, 0x3821},
            {0x3A00, 0x3A25},
            {0x3B00, 0x3B0C},
            {0x3c00, 0x3c1e},
            {0x3d00, 0x3d21},
            {0x3f00, 0x3f02},
            {0x4000, 0x4033},
            {0x4201, 0x4202},
            {0x4300, 0x430d},
            {0x4400, 0x4431},
            {0x4600, 0x460d},
            {0x4709, 0x4745},
            {0x4800, 0x4837},
            {0x4900, 0x4902},
            {0x5000, 0x5063},
            {0x5180, 0x51d0},
            {0x5300, 0x530f},
            {0x5380, 0x538d},
            {0x5480, 0x5490},
            {0x5580, 0x558c},
            {0x5600, 0x5606},
            {0x5680, 0x56a2},
            {0x5800, 0x5849},
            {0x6000, 0x603f}
    };

    printf("Camera %dx%d@%d %d bits per pixel\n",
    DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, (int)g_runtime_fps, DEMO_CAMERA_BUFFER_BPP * 8);

    for (int n=0; n<sizeof(ov5640_regs)/sizeof(ov5640_regs[0]); n++)
    {
        for (uint16_t reg = ov5640_regs[n].start; reg <= ov5640_regs[n].end; reg++)
        {
            status_t ret = BOARD_Camera_I2C_ReceiveSCCB(0x3c, reg, 2, &val, sizeof(val));
            printf ("0x%04X = 0x%02X (err: %ld)\n", reg, val, ret);
        }
    }
}

void CamDumpRegisters(void)
{
    uint16_t ov5640_regs[] = {
        0x3034, 0x3035, 0x3036, 0x3037, 0x4837, 0x3108
    };
    uint8_t val;

    printf("Camera %dx%d@%d %d bits per pixel\n",
    DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT, (int)g_runtime_fps, DEMO_CAMERA_BUFFER_BPP * 8);

    for (int n=0; n<sizeof(ov5640_regs)/sizeof(ov5640_regs[0]); n++)
    {
        status_t ret = BOARD_Camera_I2C_ReceiveSCCB(0x3c, ov5640_regs[n], 2, &val, sizeof(val));
        printf ("0x%04X = 0x%02X (err: %ld)\n", ov5640_regs[n], val, ret);
    }

    uint32_t base1 = 0x40CC0000;
    uint16_t offset1[] = {0x2480, 0x2580, 0x2500};

    for (int n=0; n<sizeof(offset1)/sizeof(offset1[0]); n++)
    {
        printf ("0x%08lX=%08lX\n", base1 + offset1[n], *(uint32_t*)(base1 + offset1[n]));
    }

    uint32_t base2 = 0x400E4000;
    uint16_t offset2[] = {0x00EC};

    for (int n=0; n<sizeof(offset2)/sizeof(offset2[0]); n++)
    {
        printf ("0x%08lX=%08lX\n", base2 + offset2[n], *(uint32_t*)(base2 + offset2[n]));
    }

    for (int n=0; n<14; n++)
    {
        printf ("40810%03X=%08lX\n",0x100 + (n * 4),
            *(uint32_t*)(0x40810000 + 0x100 + (n * 4)));
    }
}

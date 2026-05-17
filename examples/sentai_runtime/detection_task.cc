// detection_task.cc — Pipelined camera→PXP→TPU detection engine
//
// Architecture (double-buffered staging):
//
//   PrepTask (prio 2)                InferTask (prio 3)
//   ──────────────────               ──────────────────────────
//   wait(staging_free)               wait(prep_done)
//   cam_get_raw()                    memcpy staging→tensor  ~0.5ms
//   PXP → staging_buf   ~2ms        give(staging_free)     ← PrepTask can start next
//   int8 quant staging   ~1ms       Invoke()               ~30ms (USB blocks→PrepTask runs)
//   give(prep_done)                  NMS + push to queue
//   ↑ loop                           ↑ loop
//
// Memory safety:
//   - staging_buf: only PrepTask writes (via PXP+quant), only InferTask reads (memcpy)
//   - tensor_buf:  only InferTask writes (memcpy), only Invoke reads (USB bounce copy)
//   - Semaphores enforce strict handoff — never concurrent access.
//   - PXP hardware: only used by PrepTask (InferTask never touches PXP).

#include "detection_task.h"
#include "sentai_tracker.h"
#include "sentai_error.h"
#include "sentai_health.h"
#include "sentai_prep.h"        // Phase 1b: aux slot fan-out

// Forward decl — defined in sentai_runtime.cc (.sdram_text).  PXP HW
// path for XRGB→Y8 conversion (kPXP_OutputPixelFormatY8) used by the
// aux slot publishing block in prep_task_fn.
extern "C" int sentai_pxp_xrgb_to_y8(const uint8_t* src, int src_w, int src_h,
                                      uint8_t* dst, int dst_w, int dst_h);

#include <cmath>
#include <cstdio>
#include <cstring>

#include "libs/camera/camera.h"
#include "libs/camera/camera_support.h"
#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"
#include "third_party/freertos_kernel/include/semphr.h"
#include "third_party/freertos_kernel/include/queue.h"
#if (__CORTEX_M == 7)
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/cm7/fsl_cache.h"
#endif
#include "fsl_pxp.h"
#include "third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/fsl_edma.h"

// ---------------------------------------------------------------------------
// External C functions from sentai_runtime.cc (thin wrappers)
// ---------------------------------------------------------------------------
extern "C" {
    int sentai_tpu_is_ready(void);
    int sentai_tpu_invoke_internal(void);  // no pipeline guard
    int sentai_tpu_invoke_with_input(uint8_t* input_buf);  // Step 4 direct path
    int sentai_tpu_invoke_slot_with_input(int slot, uint8_t* buf);  // Phase 2 multi-slot
    /* Stateless ratio-alternate scheduler word, OWNED by sentai_runtime.cc.
     * Hi 16 = a (cam0 quota), lo 16 = b (cam1 quota); zero = inactive.
     * Read here as the override key for force_parity (see PrepTask body). */
    extern volatile uint32_t g_cam_ratio_packed;
    // Cale 1 HYBRID fine-grained sync (2026-04-25): arms SendInputs to
    // xSemaphoreGive(sema) at the END of its USB bulk-OUT phase, which
    // lets PrepTask start writing the next .tpu_input the instant USB
    // is done reading the current one -- not after the full invoke
    // (compute + output) finishes.  Pass nullptr to disarm.
    void sentai_tpu_set_input_done_sema(SemaphoreHandle_t sema);
    int sentai_detection_is_running(void);  // used by direct_tensor_set guard
    int sentai_tpu_detect(int conf_permil, int iou_permil, int max_dets,
                          int16_t* out_buf, int* out_count);
    void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp);
    // Wrappers added for pipeline access:
    int  sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                          uint8_t* dst, int dw, int dh);
    int  sentai_get_tensor_info(int* w, int* h, int* ch,
                                uint8_t** buf, int* type, int* zp);
    int  sentai_cam_grab_latest(uint8_t** raw);
    // Flow-offload hook (flow_task.cc).  Early-returns cheaply when the
    // M4 publish flag is off; otherwise CPU-decimates a 80×60 gray frame
    // into shared OCRAM for the M4 to consume.
    void sentai_flow_m4_publish_frame(const uint8_t* raw,
                                      int raw_w, int raw_h,
                                      int cam_id);
    int  sentai_cam_current_id(void);
    int  sentai_cam_grabbed_id(void);  // tag of the most recently grabbed buffer
    void sentai_cam_return_raw(int idx);
    uint32_t sentai_cam_get_frame_seq(void);
    int  sentai_cam_is_initialized(void);
    int  sentai_imu_read_accel(float* x_mg, float* y_mg, float* z_mg, float* temp_c);
}

// ---------------------------------------------------------------------------
// Pipeline state
// ---------------------------------------------------------------------------
namespace {

// Tensor-input ping-pong buffers.  Two 64-byte aligned buffers in SDRAM:
//   s_tpu_input_buf[0] — also acts as the LEGACY path's staging buffer so
//                        we don't double-allocate.  PrepTask writes here
//                        when feature flag is OFF.
//   s_tpu_input_buf[1] — second ping-pong buffer, used only in DIRECT mode.
//
// Sized to accommodate up to 640×480×3 = 921 600 bytes — the OV5640
// camera's native 4:3 crop and the largest model input we realistically
// plan to feed the TPU.  Square 512×512 fits with 135 KB headroom per
// buffer.  Total SDRAM footprint of the pair: 2 × 921 600 = 1.76 MB.
// Raise this only if a larger model is introduced; note the SDRAM region
// has ~400 KB of slack beyond this layout, so any further growth needs a
// linker-script check.
// Single OCRAM buffer.  Attempted asymmetric double-buffer (slot 0
// OCRAM + slot 1 SDRAM) 2026-04-22: every other invoke read from
// SDRAM wedged the TPU (SEMC bus contention with CSI) — 0 ok / 175
// fail in 5 s.  Hard lesson: partial SDRAM involvement = full wedge.
static constexpr int kMaxStagingSize = 640 * 480 * 3;  // 921 600 B (iarna 640x480) — also fits 512x512
static uint8_t s_tpu_input_buf_single[kMaxStagingSize]
    __attribute__((aligned(64), section(".tpu_input")));
static uint8_t* const s_tpu_input_buf[2] = {
    s_tpu_input_buf_single, s_tpu_input_buf_single
};
// s_staging_buf kept as an alias of s_tpu_input_buf[0] for legacy code
// paths that pre-date the ping-pong refactor (they write here and then
// memcpy → tensor_buf).  No memory cost — same bytes.
#define s_staging_buf (s_tpu_input_buf[0])

// ---------------------------------------------------------------------------
// Direct-to-tensor ping-pong state (Step 4 — feature-flagged, default OFF)
// ---------------------------------------------------------------------------
// Safety envelope (embeded.md §A7 + §F):
//   - Counting semaphore `s_sem_bufs_free` enforces a hard cap of 2
//     in-flight frames; PrepTask blocks (bounded 100 ms take) when both
//     buffers are owned by InferTask.
//   - Counting semaphore `s_sem_prep_done_c` carries one permit per ready
//     frame (max 2); replaces the binary `s_sem_prep_done` only when the
//     flag is ON.
//   - PrepTask and InferTask each keep a monotonic counter so buffer
//     ownership = (count & 1); no shared "current index" variable to race.
//   - Per-frame rollback: every Invoke restores the input tensor's arena
//     pointer (see sentai_tpu_invoke_with_input), so toggling the flag
//     OFF at runtime leaves TFLite in its original state.
//   - Flag mutation is refused while the pipeline is running (see
//     sentai_pipeline_direct_tensor_set) — prevents mid-frame mode flip.
// Default ON — validated at +31% FPS on E15 (513×513 model) with zero
// drops and zero swap failures across repeated trials.  Can still be
// toggled OFF at runtime via sentai.pipeline.direct_tensor(0) for A/B.
static volatile int s_direct_tensor_enabled = 1;
static SemaphoreHandle_t s_sem_bufs_free   = nullptr;  // counting, max 2, init 2
static SemaphoreHandle_t s_sem_prep_done_c = nullptr;  // counting, max 2, init 0
static uint32_t s_prep_count_dt  = 0;  // monotonic, PrepTask sole writer
static uint32_t s_infer_count_dt = 0;  // monotonic, InferTask sole writer
// Supervision counters (embeded.md §I).  Read via sentai.diag.pipeline_stats().
static volatile uint32_t s_dt_frames            = 0;  // direct-path frames completed
static volatile uint32_t s_dt_prep_buf_timeout  = 0;  // PrepTask take bufs_free timeout
static volatile uint32_t s_dt_infer_wait_timeout= 0;  // InferTask take prep_done timeout
static volatile uint32_t s_dt_pointer_swap_fail = 0;  // input_tensor(0) returned null

// PrepTask per-stage cumulative timings (ms).  Divide by s_prep_stage_frames
// for average.  Populated on every iteration with xTaskGetTickCount deltas
// (1 ms resolution) so the hot path overhead is just four uint32 adds.
static volatile uint32_t s_prep_stage_frames       = 0;
static volatile uint32_t s_prep_stage_sem_wait_ms  = 0;  // block on sem_input_free
static volatile uint32_t s_prep_stage_cam_grab_ms  = 0;  // sentai_cam_grab_latest
static volatile uint32_t s_prep_stage_pxp_ms       = 0;  // sentai_pxp_scale
static volatile uint32_t s_prep_stage_quant_ms     = 0;  // in-place int8 quant
static volatile uint32_t s_prep_stage_total_ms     = 0;  // full iter wall time
extern "C" void sentai_prep_stage_stats(uint32_t* frames,
                                        uint32_t* sem_wait, uint32_t* cam_grab,
                                        uint32_t* pxp, uint32_t* quant,
                                        uint32_t* total) {
    if (frames)    *frames    = s_prep_stage_frames;
    if (sem_wait)  *sem_wait  = s_prep_stage_sem_wait_ms;
    if (cam_grab)  *cam_grab  = s_prep_stage_cam_grab_ms;
    if (pxp)       *pxp       = s_prep_stage_pxp_ms;
    if (quant)     *quant     = s_prep_stage_quant_ms;
    if (total)     *total     = s_prep_stage_total_ms;
}
extern "C" void sentai_prep_stage_reset(void) {
    s_prep_stage_frames      = 0;
    s_prep_stage_sem_wait_ms = 0;
    s_prep_stage_cam_grab_ms = 0;
    s_prep_stage_pxp_ms      = 0;
    s_prep_stage_quant_ms    = 0;
    s_prep_stage_total_ms    = 0;
}
extern "C" int  sentai_pipeline_direct_tensor_get(void) { return s_direct_tensor_enabled; }
extern "C" int  sentai_pipeline_direct_tensor_set(int v) {
    // Only allow toggling when the pipeline is STOPPED — switching modes
    // mid-flight would leave a half-consumed buffer and a stale tensor
    // pointer.  Return -1 if the caller tries.  Upper layers propagate this
    // as a ValueError to Python.
    if (sentai_detection_is_running()) return -1;
    s_direct_tensor_enabled = v ? 1 : 0;
    return 0;
}
extern "C" void sentai_pipeline_direct_tensor_stats(uint32_t* frames,
                                                    uint32_t* prep_to,
                                                    uint32_t* infer_to,
                                                    uint32_t* swap_fail) {
    if (frames)   *frames   = s_dt_frames;
    if (prep_to)  *prep_to  = s_dt_prep_buf_timeout;
    if (infer_to) *infer_to = s_dt_infer_wait_timeout;
    if (swap_fail)*swap_fail= s_dt_pointer_swap_fail;
}

// Pipeline configuration (set by start, read by tasks)
static int s_conf_permil = 500;
static int s_iou_permil  = 450;
static int s_max_dets    = DETECTION_MAX_DETS;

// FPS throttle for InferTask.  0 = unthrottled (run as fast as TPU
// can); >0 = cap at this rate.  Default 45 to match camera, so the
// TPU doesn't outrun the frame source and doesn't sustain peak
// current draw (user theory: 75 FPS unthrottled = brown-out).
// Runtime-tunable via sentai.pipeline.target_fps(n).
static volatile int s_target_fps = 45;

extern "C" int  sentai_pipeline_target_fps_get(void) { return s_target_fps; }
extern "C" void sentai_pipeline_target_fps_set(int v) {
    if (v < 0) v = 0;
    if (v > 120) v = 120;
    s_target_fps = v;
}

// Per-iteration sleep injected at the END of PrepTask.  Conceptually
// sibling to `prep_target_fps` but expressed as raw milliseconds so
// the user can sweep the alternation sweet-spot in fine 1-ms steps
// (via calibrate(delay_ms=N) or the standalone runtime knob).  0 =
// no extra sleep.  Clamped [0, 200].
static volatile int s_pipeline_loop_delay_ms = 0;
extern "C" int  sentai_pipeline_loop_delay_get(void) { return s_pipeline_loop_delay_ms; }
extern "C" void sentai_pipeline_loop_delay_set(int v) {
    if (v < 0)   v = 0;
    if (v > 200) v = 200;
    s_pipeline_loop_delay_ms = v;
}

// Throttle PrepTask (cam_grab + PXP + quant).  0 = unthrottled.
// User convention (2026-04-22): set prep_fps and target_fps in a
// rational ratio (e.g. prep=15, tpu=30) so the two tasks don't
// over-subscribe SDRAM bus bandwidth simultaneously.  Camera is
// capped at 45 fps by the sensor — anything over that is clipped.
static volatile int s_prep_target_fps = 0;
extern "C" int  sentai_pipeline_prep_fps_get(void) { return s_prep_target_fps; }
extern "C" void sentai_pipeline_prep_fps_set(int v) {
    if (v < 0) v = 0;
    if (v > 120) v = 120;
    s_prep_target_fps = v;
}

// Incremental-isolation diagnostics (2026-04-22 session).
// When s_debug_no_invoke == 1, InferTask skips sentai_tpu_invoke_* and
// pretends success.  This isolates whether PrepTask's cam_grab + PXP +
// quant + InferTask's memcpy (but no TPU USB traffic) is enough to
// wedge the TPU.  Toggle via sentai.pipeline.debug_no_invoke(1).
static volatile int      s_debug_no_invoke = 0;

// Simulate "multi-patch per frame" workloads: run N TPU invokes per
// PrepTask iteration, all on the same input buffer.  User-facing:
// sentai.pipeline.invokes_per_frame(n).  Default 1 = baseline.
static volatile int      s_debug_invokes_per_frame = 1;
extern "C" int  sentai_pipeline_invokes_per_frame_get(void) {
    return s_debug_invokes_per_frame;
}
extern "C" void sentai_pipeline_invokes_per_frame_set(int v) {
    if (v < 1) v = 1;
    if (v > 8) v = 8;
    s_debug_invokes_per_frame = v;
}

// Multi-invoke fine-grained sync strategy (matters only when
// invokes_per_frame > 1).  Choose how InferTask coordinates with
// PrepTask across the N invoke calls on the SAME .tpu_input buffer:
//
//   0 = LEGACY (default, race-prone for N>1):
//       arm sema once at top.  First SendInputs of invoke #1 atomic-
//       exchanges + gives sem_free → PrepTask starts writing the next
//       frame.  Invokes 2..N race against PrepTask's overwrite.
//       Identical to N=1 behaviour; for N=1 there is NO race because
//       only one invoke happens.
//
//   1 = DEFER  (variant A, safe-no-race):
//       Run invokes 1..N-1 with sema = nullptr (PrepTask stays
//       blocked).  Arm sema right before invoke N.  Invoke N's first
//       SendInputs releases PrepTask, but no further reads of
//       .tpu_input happen.  PrepTask gets the buffer back exactly
//       when it would have under N=1.  Cost: PrepTask waits the full
//       multi-invoke duration.
//
//   2 = REARM  (variant B, race-window-per-invoke):
//       Re-arm sema before each invoke.  PrepTask wakes up at the
//       end of each invoke's first SendInputs, may start writing the
//       next frame, but the very next invoke immediately re-reads
//       .tpu_input and races.  Empirically interesting only as a
//       race-tolerance bound: how often does corruption manifest?
//
// Toggle via sentai.diag.multi_invoke_mode([n]).
static volatile int s_multi_invoke_sync_mode = 0;
extern "C" int  sentai_pipeline_multi_invoke_mode_get(void) {
    return s_multi_invoke_sync_mode;
}
extern "C" void sentai_pipeline_multi_invoke_mode_set(int v) {
    if (v < 0) v = 0;
    if (v > 2) v = 2;
    s_multi_invoke_sync_mode = v;
}


// PrepTask staged mock — builds up the pipeline piece by piece to
// find which SDRAM/bus activity wedges the TPU.  Runtime-tunable via
// sentai.pipeline.debug_prep_mode(n):
//   0 = FULL  (default — cam_grab + PXP + quant)
//   1 = MOCK  (vTaskDelay only, no HW activity)
//   2 = CAM   (cam_grab + return; no PXP, no quant; zero dst)
//   3 = PXP   (cam_grab + PXP; no quant)
//   4 = FULL  (same as 0)
static volatile int      s_debug_prep_mode = 0;
extern "C" int  sentai_pipeline_debug_prep_mode_get(void) {
    return s_debug_prep_mode;
}
extern "C" void sentai_pipeline_debug_prep_mode_set(int v) {
    if (v < 0) v = 0;
    if (v > 4) v = 4;
    s_debug_prep_mode = v;
}
// Per-invoke outcome counters observable from REPL while the pipeline
// runs.  sentai.pipeline.infer_stats() returns a dict with these +
// cumulative invoke_ms_sum + last_invoke_rc for diagnosing WHEN
// InferTask starts failing during a sustained run.
static volatile uint32_t s_infer_ok_count    = 0;
static volatile uint32_t s_infer_fail_count  = 0;
static volatile uint32_t s_infer_ms_sum      = 0;
static volatile int32_t  s_infer_last_rc     = 0;
extern "C" int  sentai_pipeline_debug_no_invoke_get(void) {
    return s_debug_no_invoke;
}
extern "C" void sentai_pipeline_debug_no_invoke_set(int v) {
    s_debug_no_invoke = v ? 1 : 0;
}
extern "C" void sentai_pipeline_infer_stats(uint32_t* ok, uint32_t* fail,
                                            uint32_t* ms_sum,
                                            int32_t*  last_rc) {
    if (ok)      *ok      = s_infer_ok_count;
    if (fail)    *fail    = s_infer_fail_count;
    if (ms_sum)  *ms_sum  = s_infer_ms_sum;
    if (last_rc) *last_rc = s_infer_last_rc;
}
extern "C" void sentai_pipeline_infer_reset(void) {
    s_infer_ok_count   = 0;
    s_infer_fail_count = 0;
    s_infer_ms_sum     = 0;
    s_infer_last_rc    = 0;
}

// Synchronization
static SemaphoreHandle_t s_sem_staging_free = nullptr;  // staging available for PrepTask
static SemaphoreHandle_t s_sem_prep_done    = nullptr;  // staging has new prepped frame
static QueueHandle_t     s_det_queue        = nullptr;  // detection results → consumer

// Task handles
static TaskHandle_t s_prep_task  = nullptr;
static TaskHandle_t s_infer_task = nullptr;

// sentai: preallocated TCB + stack for PrepTask and InferTask.
// User hypothesis 2026-04-22: xTaskCreate (dynamic heap allocation
// from FreeRTOS heap) may fragment a region that is subsequently
// touched by USB.  Static allocation eliminates heap churn at
// pipeline.start and puts task structures in predictable memory.
// Task stacks live in SDRAM (.sdram_bss) to keep DTCM free for
// the USB staging buffer and other hot-path DMA targets.  TCBs are
// small (~100 bytes) and can live in default BSS (DTCM).
static StaticTask_t s_prep_tcb;
static StaticTask_t s_infer_tcb;
static constexpr unsigned kPrepStackWords  = configMINIMAL_STACK_SIZE * 8;
static constexpr unsigned kInferStackWords = configMINIMAL_STACK_SIZE * 12;
static StackType_t s_prep_stack [kPrepStackWords]
    __attribute__((aligned(8), section(".sdram_bss")));
static StackType_t s_infer_stack[kInferStackWords]
    __attribute__((aligned(8), section(".sdram_bss")));

// Control
static volatile bool s_running = false;

// Statistics (P3 FIX: use atomic operations for thread safety)
static uint32_t s_frames_processed = 0;  // accessed via __atomic
static uint32_t s_frames_dropped   = 0;  // accessed via __atomic
static TickType_t s_start_tick      = 0;

// Per-task liveness timestamps — tick when each stage last completed a frame.
// PrepTask updates after xSemaphoreGive(s_sem_prep_done).
// InferTask updates after sentai_health_success().
// Zero means the stage has not yet completed a single frame since start.
// Allows external callers to detect which pipeline stage is stuck.
static volatile TickType_t s_last_prep_frame_tick  = 0;
static volatile TickType_t s_last_infer_frame_tick = 0;

// Staging metadata (written by PrepTask, read by InferTask after semaphore)
// The semaphore handoff pattern ensures correct ordering:
//   PrepTask: write metadata → give(prep_done)
//   InferTask: take(prep_done) → read metadata
static int      s_stg_cam_id = -1;

// Force-parity flag.  When ON (default 0), PrepTask enforces strict
// alternation of the per-buffer cam_id tag: if the freshly-grabbed
// buffer carries the SAME cam_id as the previous accepted frame, the
// buffer is returned and a new grab is attempted (bounded retries).
// The "parity" is whatever cam_id was first observed -- subsequent
// frames are required to flip on every accepted handoff to InferTask.
// Useful for the calibrate path where the user wants strict 50/50
// distribution and any duplicated camera frame is a propagation bug.
//
// Skip is best-effort: if no fresh frame with the expected parity
// arrives within the retry budget, we accept whatever we have and
// bump s_force_parity_timeout (the pipeline keeps making progress).
static volatile int s_force_parity = 0;
static int          s_last_grabbed_cam_id = -1;
extern "C" int  sentai_pipeline_force_parity_get(void) { return s_force_parity; }
extern "C" void sentai_pipeline_force_parity_set(int v) { s_force_parity = v ? 1 : 0; }

// ---- Phase 2: per-camera slot dispatch ----
//
// `s_slot_for_cam[cam_id]` tells InferTask which TPU slot to invoke for
// a frame tagged with that camera.  Default {0, 0} = both cameras
// invoke slot 0 (legacy single-slot behaviour).  REPL API:
//   sentai.pipeline.set_slot_for_cam(cam_id, slot)
//
// NMS / detection_task output handling stays on slot 0 in Phase 2a —
// per-slot detection result publication is Phase 2b.  Until then, a
// camera mapped to slot != 0 will still TRIGGER its model (visible in
// per-slot invoke counters), but its detections won't surface in the
// normal `pipeline.calibrate` / `detection_get_latest` flow.
static volatile int8_t s_slot_for_cam[2] = { 0, 0 };
static volatile uint32_t s_slot_invokes[3] = { 0, 0, 0 };
extern "C" int  sentai_pipeline_get_slot_for_cam(int cam_id) {
    if (cam_id < 0 || cam_id >= 2) return -1;
    return (int)s_slot_for_cam[cam_id];
}
extern "C" int sentai_tpu_slot_ready(int slot);
extern "C" int  sentai_pipeline_set_slot_for_cam(int cam_id, int slot) {
    if (cam_id < 0 || cam_id >= 2) return -1;
    if (slot < 0 || slot >= 3) return -2;
    // Reject routing to an unloaded slot — InferTask would otherwise
    // fire `invoke_slot` repeatedly on a null interpreter and spam
    // SERR_TPU_NOT_READY for every frame.  Caller must call
    // `tpu.load_slot(slot, path)` BEFORE binding it to a camera.
    // Slot 0 is allowed unconditionally (legacy back-compat path,
    // many callers set cam→slot 0 before any model is loaded).
    if (slot != 0 && !sentai_tpu_slot_ready(slot)) {
        SERR_LOG(SERR_TPU_SLOT_NOT_READY,
                 (uint32_t)((cam_id << 4) | slot));
        return -3;
    }
    s_slot_for_cam[cam_id] = (int8_t)slot;
    return 0;
}
extern "C" void sentai_pipeline_slot_stats(uint32_t* per_slot, int n) {
    for (int i = 0; i < n && i < 3; i++) per_slot[i] = s_slot_invokes[i];
}
extern "C" void sentai_pipeline_slot_stats_reset(void) {
    s_slot_invokes[0] = s_slot_invokes[1] = s_slot_invokes[2] = 0;
}
static volatile uint32_t s_force_parity_skipped = 0;
static volatile uint32_t s_force_parity_timeout = 0;
extern "C" void sentai_pipeline_force_parity_stats(uint32_t* skipped, uint32_t* timeout) {
    if (skipped) *skipped = s_force_parity_skipped;
    if (timeout) *timeout = s_force_parity_timeout;
}
extern "C" void sentai_pipeline_force_parity_reset(void) {
    s_force_parity_skipped = 0;
    s_force_parity_timeout = 0;
    s_last_grabbed_cam_id = -1;
}

/* Schedule-aware grab considered (2026-04-27) and DROPPED.
 *
 * The VGA60+2:1 aliasing (cam1 always discarded by grab_latest because
 * its cycle is exactly the pipeline-iteration period) is a real
 * artefact, but the consumer-side workarounds (schedule-tracking grab,
 * FIFO grab) either duplicate the producer's schedule into a second
 * source of truth or trade significant latency.  Decision: keep the
 * single-source-of-truth (ISR scheduler is authoritative) and avoid
 * ratio cycles that alias with the pipeline iteration period.
 * Practical guidance for the diag drivers:
 *   - 1:1 always works (cycle = 2 sensor periods).
 *   - 3:1 always works (cycle = 4 sensor periods, cam1 distinct).
 *   - 2:1 at VGA60 aliases (cycle 50 ms ≈ 2× pipeline iter 22 ms);
 *     skip it from the test matrix and use 3:1 instead.
 * Documented in agent/experiment.md.  No code added.
 */

static int      s_stg_w  = 0;
static int      s_stg_h  = 0;
static int      s_stg_ch = 0;
static int      s_stg_total = 0;
static uint32_t s_stg_frame_seq = 0;

// ---------------------------------------------------------------------------
// PrepTask: Camera → PXP → int8 quant → staging buffer
// ---------------------------------------------------------------------------
static void prep_task_fn(void* /*param*/) {
    // PrepTask started

    // Rate-limit camera-miss health reports: report every 10th consecutive miss.
    // Normal transient misses (camera busy) do not pollute the health record.
    int cam_miss_streak = 0;

    while (s_running) {
        // -------------------------------------------------------------
        // Pick the destination buffer based on the active path:
        //   legacy  → single s_staging_buf, gated by s_sem_staging_free
        //   direct  → one of s_tpu_input_buf[0..1], gated by
        //             s_sem_bufs_free (counting, max 2).  Index is
        //             (s_prep_count_dt & 1) so the two tasks stay in
        //             lockstep without sharing a mutable "current" var.
        // The choice is latched per-iteration (not re-checked mid-frame)
        // so toggling the flag only affects the NEXT frame, avoiding a
        // half-direct / half-legacy frame that would confuse InferTask.
        // -------------------------------------------------------------
        const int direct = s_direct_tensor_enabled;
        SemaphoreHandle_t sem_input_free =
            direct ? s_sem_bufs_free : s_sem_staging_free;
        if (!sem_input_free) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        TickType_t t_iter_start = xTaskGetTickCount();
        TickType_t t_sem_start  = t_iter_start;
        if (xSemaphoreTake(sem_input_free, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (direct) s_dt_prep_buf_timeout++;
            continue;  // timeout — recheck s_running (normal when pipeline just started)
        }
        TickType_t t_sem_end = xTaskGetTickCount();
        if (!s_running) break;

        uint8_t* dst_buf = direct
            ? s_tpu_input_buf[s_prep_count_dt & 1u]
            : s_staging_buf;

        // MOCK mode — simulate prep cadence with zero bus traffic.
        // Sleep ~20 ms (approx full-pipeline iteration), fill metadata,
        // signal InferTask.  Lets us test InferTask in isolation with
        // no camera/PXP/quant load on SDRAM.
        const int prep_mode = s_debug_prep_mode;
        if (prep_mode == 1) {
            int mw, mh, mch, mtype, mzp;
            uint8_t* mtbuf;
            if (sentai_get_tensor_info(&mw, &mh, &mch, &mtbuf, &mtype, &mzp) != 0) {
                xSemaphoreGive(sem_input_free);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            int mtotal = mw * mh * mch;
            memset(dst_buf, 0, mtotal);
            vTaskDelay(pdMS_TO_TICKS(20));
            s_prep_stage_frames++;
            s_prep_stage_total_ms += 20;
            s_stg_w = mw; s_stg_h = mh; s_stg_ch = mch;
            s_stg_total = mtotal;
            s_stg_frame_seq++;
            s_stg_cam_id = -1;  // mock mode: no real camera
            s_last_prep_frame_tick = xTaskGetTickCount();
            if (direct) { s_prep_count_dt++; xSemaphoreGive(s_sem_prep_done_c); }
            else        { xSemaphoreGive(s_sem_prep_done); }
            continue;
        }

        // Get model tensor dimensions (for PXP target size)
        int w = 0, h = 0, ch = 0, type = 0, zp = 0;
        uint8_t* tensor_buf = nullptr;
        if (sentai_get_tensor_info(&w, &h, &ch, &tensor_buf, &type, &zp) != 0) {
            sentai_health_fail(SUBSYS_DETECT);
            xSemaphoreGive(sem_input_free);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int total = w * h * ch;
        if (total > kMaxStagingSize) {
            SERR_LOG(SERR_DET_TENSOR_SIZE, total);
            sentai_health_fail(SUBSYS_DETECT);
            xSemaphoreGive(sem_input_free);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Get latest camera frame (drains stale ones).
        TickType_t t_cam_start = xTaskGetTickCount();
        uint8_t* raw = nullptr;
        int idx = sentai_cam_grab_latest(&raw);
        // Snapshot the per-buffer cam_id tag set by CSI ISR.  Reflects
        // which physical camera filled the buffer that grab returned.
        int prep_cam_id = sentai_cam_grabbed_id();

        // force_parity: if this buffer matches the last accepted cam_id,
        // discard and retry (bounded).  Best-effort -- on timeout we
        // accept whatever we have so the pipeline keeps making progress.
        //
        // RATIO OVERRIDE (build #98x, 2026-04-27): when sentai.camera.ratio(a,b)
        // is active (g_cam_ratio_packed != 0) the CSI ISR is already
        // running an authoritative auto-alternate schedule.  force_parity
        // would then DISCARD frames the ISR scheduled on purpose — at
        // VGA60 with ratio(2,1) the 22 ms force_parity sleep is longer
        // than the 16.7 ms sensor period, so every "second cam0" the
        // schedule emits gets thrown away → captured distribution
        // collapses to 50:50 and the user-requested 2:1 parity is lost.
        // The ratio is the source of truth; force_parity is meaningful
        // ONLY when the ISR is NOT alternating.  Bypass when ratio is
        // packed-active.  Symbol declared at file scope above (extern "C").
        const bool ratio_active = (g_cam_ratio_packed != 0u);
        if (s_force_parity && !ratio_active && idx >= 0 && raw &&
            prep_cam_id == s_last_grabbed_cam_id && prep_cam_id >= 0) {
            // Hard wall-clock cap so a wedged camera at force_parity
            // ON cannot stretch a single PrepTask iteration past 200 ms
            // (embeded.md §B, §F).  Same effect as kMaxParityRetries
            // but enforced in time domain — defends against the case
            // where vTaskDelay returns immediately due to scheduler
            // anomalies and the loop iterates faster than expected.
            const TickType_t t_par_start = xTaskGetTickCount();
            const TickType_t kMaxParityWallMs = pdMS_TO_TICKS(200);
            const int kMaxParityRetries = 6;
            int retries = 0;
            while (retries < kMaxParityRetries &&
                   prep_cam_id == s_last_grabbed_cam_id &&
                   (xTaskGetTickCount() - t_par_start) < kMaxParityWallMs) {
                sentai_cam_return_raw(idx);
                s_force_parity_skipped++;
                // Tick-aligned to ~1 sensor frame at VGA45 (22 ms) so the
                // CSI ISR has had a chance to fill a fresh slot with the
                // OTHER camera's data before we re-grab.
                vTaskDelay(pdMS_TO_TICKS(22));
                idx = sentai_cam_grab_latest(&raw);
                if (idx < 0 || !raw) break;
                prep_cam_id = sentai_cam_grabbed_id();
                retries++;
            }
            if (idx >= 0 && raw &&
                prep_cam_id == s_last_grabbed_cam_id) {
                s_force_parity_timeout++;
            }
        }
        TickType_t t_cam_end = xTaskGetTickCount();
        if (idx < 0 || !raw) {
            // Rate-limit: only report to health after 10 consecutive camera misses.
            // A single miss is normal (no frame available yet); sustained misses
            // indicate a camera hardware or driver failure.
            if (++cam_miss_streak >= 10) {
                sentai_health_fail(SUBSYS_DETECT);
                cam_miss_streak = 0;
            }
            xSemaphoreGive(sem_input_free);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        cam_miss_streak = 0;  // reset on successful frame grab

        // ── sentai_prep Phase 1b: aux slot fan-out ──────────────────
        // After cam_grab succeeded + force_parity passed, publish
        // enabled aux slots from the same `raw` XRGB8888 frame.
        //
        // NON-BLOCKING by design (no semaphore on this path) — slot
        // consumers tolerate last-frame-wins per [[no-heavy-data-
        // through-mp]] / continuous-publish discipline.  ISR-ready
        // (no FreeRTOS blocking primitives invoked).
        //
        // Cost: only fires for slots whose refcount > 0 AND whose
        // frame_div counter aligned this frame (today always 1).
        // Disabled slots are a single mask test — zero PXP work when
        // no consumer is active.
        {
            const uint32_t fire_mask = sentai_prep_tick_frame();
            if (fire_mask & (1u << SENTAI_PREP_SLOT_GRAY_NATIVE)) {
                int sw = 0, sh = 0;
                uint8_t* sbuf = sentai_prep_slot_begin_write(
                    SENTAI_PREP_SLOT_GRAY_NATIVE, &sw, &sh);
                if (sbuf) {
                    // PXP XRGB→Y8 luma (kPXP_OutputPixelFormatY8) —
                    // BT.601 luma computed internally by PXP HW.
                    sentai_pxp_xrgb_to_y8(raw, DEMO_CAMERA_WIDTH,
                                           DEMO_CAMERA_HEIGHT,
                                           sbuf, sw, sh);
                    sentai_prep_slot_commit(SENTAI_PREP_SLOT_GRAY_NATIVE);
                }
            }
            // SLOT_RGB_64 and SLOT_GRAY_64 — Phase 1c (added when
            // SlamTask lands).
        }

        // MODE 2 (CAM): skip PXP + quant.  Return raw buffer immediately
        // and zero the tensor destination.  Tests if camera traffic alone
        // (CSI DMA into SDRAM) wedges the TPU.
        if (prep_mode == 2) {
            sentai_flow_m4_publish_frame(raw, DEMO_CAMERA_WIDTH,
                                         DEMO_CAMERA_HEIGHT,
                                         sentai_cam_current_id());
            sentai_cam_return_raw(idx);
            memset(dst_buf, 0, total);
            s_prep_stage_frames++;
            s_prep_stage_cam_grab_ms += (uint32_t)(t_cam_end - t_cam_start);
            s_prep_stage_total_ms    += (uint32_t)(xTaskGetTickCount() - t_iter_start);
            s_stg_w = w; s_stg_h = h; s_stg_ch = ch;
            s_stg_total = total;
            s_stg_frame_seq = sentai_cam_get_frame_seq();
            s_stg_cam_id = prep_cam_id;  // CAM mode: source from per-buffer tag
            s_last_grabbed_cam_id = prep_cam_id;  // for force_parity tracking
            s_last_prep_frame_tick = xTaskGetTickCount();
            if (direct) { s_prep_count_dt++; xSemaphoreGive(s_sem_prep_done_c); }
            else        { xSemaphoreGive(s_sem_prep_done); }
            continue;
        }

        // PXP hardware: XRGB8888 → RGB888P, scaled to model input size
        // In direct mode, dst_buf IS one of the tensor ping-pong buffers so
        // InferTask can read directly with a pointer swap (no memcpy).
        TickType_t t_pxp_start = xTaskGetTickCount();
        int rc = sentai_pxp_scale(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  dst_buf, w, h);
        TickType_t t_pxp_end = xTaskGetTickCount();
        // Publish a downsampled frame to the M4 flow task (no-op when
        // sentai_flow.m4_start() hasn't been called).  Must happen
        // BEFORE sentai_cam_return_raw — `raw` goes invalid after.
        sentai_flow_m4_publish_frame(raw, DEMO_CAMERA_WIDTH,
                                     DEMO_CAMERA_HEIGHT,
                                     sentai_cam_current_id());
        sentai_cam_return_raw(idx);

        if (rc != 0) {
            // PXP failure is always reportable — hardware error, not a transient miss.
            SERR_LOG(SERR_DET_INVOKE_FAIL, (uint32_t)rc);  // reuse invoke-fail code for PXP
            sentai_health_fail(SUBSYS_DETECT);
            xSemaphoreGive(sem_input_free);
            continue;
        }

        // Int8 quantization in-place on the same buffer we just wrote.
        // Skipped in MODE 3 (PXP-only isolation).
        TickType_t t_quant_start = xTaskGetTickCount();
        if (prep_mode != 3 && type == 9 /* kTfLiteInt8 */) {
            sentai_quant_uint8_to_int8(dst_buf, total, zp);
        }
        TickType_t t_quant_end = xTaskGetTickCount();

        // Accumulate per-stage timings.  Ticks are portTICK_PERIOD_MS = 1 ms on
        // this target so subtraction == milliseconds directly.
        s_prep_stage_frames++;
        s_prep_stage_sem_wait_ms += (uint32_t)(t_sem_end   - t_sem_start);
        s_prep_stage_cam_grab_ms += (uint32_t)(t_cam_end   - t_cam_start);
        s_prep_stage_pxp_ms      += (uint32_t)(t_pxp_end   - t_pxp_start);
        s_prep_stage_quant_ms    += (uint32_t)(t_quant_end - t_quant_start);
        s_prep_stage_total_ms    += (uint32_t)(t_quant_end - t_iter_start);

        // Publish metadata (safe: InferTask won't read until we signal)
        s_stg_w  = w;
        s_stg_h  = h;
        s_stg_ch = ch;
        s_stg_total = total;
        s_stg_frame_seq = sentai_cam_get_frame_seq();
        // Propagate per-buffer camera tag (full FULL/PXP path).
        s_stg_cam_id = prep_cam_id;
        s_last_grabbed_cam_id = prep_cam_id;  // for force_parity tracking

        // Signal next stage: a buffer has a new prepared frame.
        //   direct → frame is in OCRAM .tpu_input directly; InferTask invokes
        //   legacy → frame is in staging; InferTask memcpys to arena
        s_last_prep_frame_tick = xTaskGetTickCount();
        s_prep_count_dt++;
        if (direct) {
            xSemaphoreGive(s_sem_prep_done_c);
        } else {
            // Legacy path doesn't increment s_prep_count_dt; revert to keep
            // direct/legacy bookkeeping unchanged.
            s_prep_count_dt--;
            xSemaphoreGive(s_sem_prep_done);
        }

        // PrepTask FPS throttle — match user-set cadence.  Runs after
        // the give() so the next iteration's sem_free take starts
        // counting from the throttle boundary, not from cam_grab.
        if (s_prep_target_fps > 0) {
            const TickType_t min_period =
                pdMS_TO_TICKS(1000 / s_prep_target_fps);
            TickType_t elapsed = xTaskGetTickCount() - t_iter_start;
            if (elapsed < min_period) {
                vTaskDelay(min_period - elapsed);
            }
        }
        // Raw post-iteration sleep (sweet-spot exploration).  Applied
        // ON TOP of any prep_fps throttle.
        if (s_pipeline_loop_delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(s_pipeline_loop_delay_ms));
        }
    }

    // PrepTask exiting
    s_prep_task = nullptr;
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// eDMA-accelerated memcpy (SDRAM → SDRAM)
// ---------------------------------------------------------------------------
// The CPU memcpy for staging_buf → TFLite input tensor takes ~24 ms for
// 786 KB because every 32-byte line read+write goes through the SEMC bus
// and the D-cache's write-allocate policy double-traffics the destination.
// The RT1176 eDMA can stream the same buffer at back-to-back SDRAM bursts
// while the CPU is free to run NMS or yield to PrepTask.  Expected ~3-5 ms.
//
// Channel choice: DMA0 ch31 (audio uses ch0; camera/other drivers don't
// use eDMA).  Polled completion — no ISR, no FreeRTOS semaphore needed —
// so the helper is usable from any task without scheduler coupling.
static constexpr uint32_t kSentaiDmaChannel = 31;
static edma_handle_t s_dma_memcpy_handle;
static bool          s_dma_memcpy_inited = false;

// Runtime toggle so benchmarks can A/B the same firmware image.  Default ON
// (optimised path).  Exposed to Python as sentai.pipeline.dma_memcpy([flag]).
static volatile int s_dma_memcpy_enabled = 1;
extern "C" int  sentai_dma_memcpy_get(void) { return s_dma_memcpy_enabled; }
extern "C" void sentai_dma_memcpy_set(int v) { s_dma_memcpy_enabled = v ? 1 : 0; }

static void sentai_dma_init_once() {
    if (s_dma_memcpy_inited) return;
    edma_config_t cfg;
    EDMA_GetDefaultConfig(&cfg);
    // Don't re-init DMA0 if audio already did; EDMA_Init only touches global
    // control regs which are idempotent, but create handle after.
    EDMA_Init(DMA0, &cfg);
    EDMA_CreateHandle(&s_dma_memcpy_handle, DMA0, kSentaiDmaChannel);
    s_dma_memcpy_inited = true;
}

// Bounded, polled DMA memcpy. Returns true on success.
// Both pointers must be 4-byte aligned and `size` must be a multiple of 4.
// Cache management: clean src (flush any pending CPU writes) before DMA read;
// invalidate dst after DMA write (so subsequent CPU reads see DMA output).
static bool sentai_dma_memcpy(void* dst, const void* src, uint32_t size) {
    if (!s_dma_memcpy_inited) sentai_dma_init_once();

    // Cache strategy: staging_buf and the TFLite input tensor are both written
    // exclusively by DMA masters (PXP writes staging_buf; this eDMA writes the
    // tensor) — neither buffer has *dirty* cache lines from CPU writes, so a
    // CleanInvalidate before the transfer is unnecessary and expensive (~20 ms
    // for 786 KB due to SDRAM writeback traffic).  We only invalidate the dst
    // *after* the transfer so subsequent CPU reads don't return stale data.

    edma_transfer_config_t tcfg;
    // Use 32-byte transfer width so each eDMA request becomes an 8-beat
    // AXI burst to SEMC (SDRAM's efficient access size per AN12437).
    // Falls back to 4-byte width if buffers aren't 32-byte aligned or size
    // isn't a multiple of 32.  Word-by-word transfers defeat the burst logic
    // and deliver only ~10 MB/s — we need ~200 MB/s.
    uint32_t width = 4;
    if ((((uintptr_t)src | (uintptr_t)dst | size) & 0x1Fu) == 0u) {
        width = 32;
    } else if ((((uintptr_t)src | (uintptr_t)dst | size) & 0x7u) == 0u) {
        width = 8;
    }
    EDMA_PrepareTransfer(&tcfg,
                         const_cast<void*>(src), width,
                         dst,                    width,
                         /*bytesEachRequest=*/size,
                         /*transferBytes=*/size,
                         kEDMA_MemoryToMemory);

    if (EDMA_SubmitTransfer(&s_dma_memcpy_handle, &tcfg) != kStatus_Success) {
        return false;
    }
    EDMA_StartTransfer(&s_dma_memcpy_handle);
    // Mem-to-mem transfers have no peripheral request line to trigger the
    // minor loop — EDMA_StartTransfer only sets SERQ (the request-enable
    // bit).  We must also software-fire the first (and only) minor loop
    // via SSRT, otherwise the channel sits armed but idle and we time out.
    EDMA_TriggerChannelStart(DMA0, kSentaiDmaChannel);

    // Poll for completion — bounded: 786 KB / 200 MB/s ~= 4 ms, cap at 50 ms.
    // We intentionally poll rather than sleep; the only other task that
    // could run is PrepTask (prio 2) and its forward progress is gated on
    // the staging_free semaphore we haven't given yet, so yielding would
    // waste time and add scheduling jitter.
    // Poll the channel DONE flag — interrupts aren't enabled on this channel.
    // kEDMA_DoneFlag is set when the major loop (1 iteration for our config)
    // completes.  Bounded to 50 ms — 786 KB at 32-byte SDRAM bursts is ~15 ms
    // in steady state, so 50 ms leaves large headroom for SEMC contention.
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(50);
    while ((EDMA_GetChannelStatusFlags(DMA0, kSentaiDmaChannel)
            & kEDMA_DoneFlag) == 0) {
        if (xTaskGetTickCount() > deadline) {
            return false;  // timeout — caller falls back to CPU memcpy
        }
    }
    EDMA_ClearChannelStatusFlags(DMA0, kSentaiDmaChannel,
                                 kEDMA_InterruptFlag | kEDMA_DoneFlag);

#if (__CORTEX_M == 7)
    DCACHE_InvalidateByRange((uint32_t)dst, size);
#endif
    return true;
}

// ---------------------------------------------------------------------------
// InferTask: memcpy staging→tensor → Invoke → NMS → queue
// ---------------------------------------------------------------------------
static void infer_task_fn(void* /*param*/) {
    // InferTask started

    while (s_running) {
        // Latch the path for this iteration — same reasoning as PrepTask
        // (direct/legacy never interleaves mid-frame).  The flag is
        // guaranteed stable because direct_tensor_set() refuses to mutate
        // it while the pipeline is running.
        const int direct = s_direct_tensor_enabled;
        SemaphoreHandle_t sem_prep = direct ? s_sem_prep_done_c : s_sem_prep_done;
        SemaphoreHandle_t sem_free = direct ? s_sem_bufs_free   : s_sem_staging_free;
        if (!sem_prep || !sem_free) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        // Wait for PrepTask to deliver a new frame
        if (xSemaphoreTake(sem_prep, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (direct) s_dt_infer_wait_timeout++;
            continue;  // timeout — recheck s_running
        }
        if (!s_running) break;

        TickType_t t0 = xTaskGetTickCount();

        // Read staging metadata
        int total = s_stg_total;
        uint32_t frame_seq = s_stg_frame_seq;
        int frame_cam_id = s_stg_cam_id;  // source camera tag (-1 if unknown)

        // Get tensor buffer pointer (legacy path memcpy target; direct path
        // only uses it for dimension validation).
        int w = 0, h = 0, ch = 0, type = 0, zp = 0;
        uint8_t* tensor_buf = nullptr;
        if (sentai_get_tensor_info(&w, &h, &ch, &tensor_buf, &type, &zp) != 0) {
            if (direct) s_dt_pointer_swap_fail++;
            xSemaphoreGive(sem_free);
            continue;
        }

        TickType_t t_memcpy_start = xTaskGetTickCount();
        TickType_t t_memcpy_end;
        int invoke_ms;

        if (direct) {
            // Direct mode: PrepTask wrote into s_tpu_input_buf[index] in
            // OCRAM; InferTask hands the same pointer to invoke.  Zero-copy.
            uint8_t* buf = s_tpu_input_buf[s_infer_count_dt & 1u];
            s_infer_count_dt++;
            t_memcpy_end = t_memcpy_start;
            // Fine-grained pipeline sync (2026-04-25): instead of
            // giving sem_free BEFORE invoke (which lets PrepTask
            // overwrite .tpu_input concurrent with USB read), we arm
            // the driver to give sem_free at the END of SendInputs --
            // i.e. the exact moment USB no longer needs the buffer.
            // PrepTask gets unblocked at that point and can write the
            // NEXT frame while InferTask continues with compute +
            // output (which don't touch .tpu_input).
            // invokes_per_frame: run tpu_invoke_with_input N times on
            // the SAME input buffer per PrepTask frame.  Sync strategy
            // selected by s_multi_invoke_sync_mode (see decl).
            const int n_calls = (s_debug_invokes_per_frame > 0)
                                ? s_debug_invokes_per_frame : 1;
            const int sync_mode = s_multi_invoke_sync_mode;

            // Phase 2: pick TPU slot from cam_id mapping.  Slot 0 is
            // the legacy interpreter (`g_interpreter`) so the slot==0
            // fast path uses `_with_input` directly to keep the V22
            // OCRAM pointer-patch optimization byte-for-byte
            // identical.  Slots 1+ go through the general `_slot_with_input`.
            int active_slot = 0;
            if (frame_cam_id >= 0 && frame_cam_id < 2) {
                active_slot = (int)s_slot_for_cam[frame_cam_id];
            }
            if (active_slot < 0 || active_slot >= 3) active_slot = 0;
            s_slot_invokes[active_slot]++;
            #define INVOKE_DT(buf_ptr) ((active_slot == 0) \
                ? sentai_tpu_invoke_with_input(buf_ptr) \
                : sentai_tpu_invoke_slot_with_input(active_slot, buf_ptr))

            if (n_calls == 1 || sync_mode == 0) {
                // Legacy / single-invoke: arm once at the top; first
                // SendInputs releases PrepTask.  For n=1 this is the
                // race-free fast path.
                sentai_tpu_set_input_done_sema(sem_free);
                invoke_ms = s_debug_no_invoke ? 0 : INVOKE_DT(buf);
                for (int k = 1; k < n_calls && invoke_ms >= 0; k++) {
                    int extra_ms = INVOKE_DT(buf);
                    if (extra_ms < 0) { invoke_ms = extra_ms; break; }
                    s_infer_ok_count++;
                    s_infer_ms_sum += (uint32_t)extra_ms;
                }
            } else if (sync_mode == 1) {
                // DEFER: invokes 1..N-1 run with sema disarmed
                // (PrepTask stays blocked, no buffer overwrite).
                // Arm before invoke N so PrepTask is released exactly
                // at the same point as in the N=1 case.
                invoke_ms = s_debug_no_invoke ? 0 : INVOKE_DT(buf);
                for (int k = 1; k < n_calls - 1 && invoke_ms >= 0; k++) {
                    int extra_ms = INVOKE_DT(buf);
                    if (extra_ms < 0) { invoke_ms = extra_ms; break; }
                    s_infer_ok_count++;
                    s_infer_ms_sum += (uint32_t)extra_ms;
                }
                if (invoke_ms >= 0 && n_calls > 1) {
                    sentai_tpu_set_input_done_sema(sem_free);
                    int last_ms = INVOKE_DT(buf);
                    if (last_ms < 0) {
                        invoke_ms = last_ms;
                    } else {
                        s_infer_ok_count++;
                        s_infer_ms_sum += (uint32_t)last_ms;
                    }
                }
            } else {
                // REARM (sync_mode==2): arm before each invoke.
                // PrepTask wakes after each invoke's first SendInputs,
                // races with the next invoke's reread of .tpu_input.
                sentai_tpu_set_input_done_sema(sem_free);
                invoke_ms = s_debug_no_invoke ? 0 : INVOKE_DT(buf);
                for (int k = 1; k < n_calls && invoke_ms >= 0; k++) {
                    sentai_tpu_set_input_done_sema(sem_free);
                    int extra_ms = INVOKE_DT(buf);
                    if (extra_ms < 0) { invoke_ms = extra_ms; break; }
                    s_infer_ok_count++;
                    s_infer_ms_sum += (uint32_t)extra_ms;
                }
            }
            // One-shot consume: if SendInputs fired, driver already
            // cleared the pointer and gave sem_free.  If invoke failed
            // BEFORE SendInputs completed, the pointer is still live --
            // force clear + give sem_free manually so PrepTask doesn't
            // stall.  xSemaphoreGive on a max-1 counting sem caps, so
            // the "double give" risk when the driver also succeeded
            // doesn't corrupt the count.
            sentai_tpu_set_input_done_sema(nullptr);
            if (invoke_ms < 0) {
                xSemaphoreGive(sem_free);
            }
            #undef INVOKE_DT
        } else {
            bool dma_ok = false;
            if (s_dma_memcpy_enabled
                && ((((uintptr_t)tensor_buf | (uintptr_t)s_staging_buf
                      | (uint32_t)total) & 0x3u) == 0u)) {
                dma_ok = sentai_dma_memcpy(tensor_buf, s_staging_buf, total);
            }
            if (!dma_ok) {
                memcpy(tensor_buf, s_staging_buf, total);
            }
            t_memcpy_end = xTaskGetTickCount();
            xSemaphoreGive(sem_free);
            invoke_ms = s_debug_no_invoke ? 0 : sentai_tpu_invoke_internal();
        }
        s_infer_last_rc = invoke_ms;
        if (invoke_ms >= 0) {
            s_infer_ok_count++;
            s_infer_ms_sum += (uint32_t)invoke_ms;
        } else {
            s_infer_fail_count++;
        }

        if (invoke_ms < 0) {
            // Rate-limited SERR: log first failure, then every 100th.
            // Each SERR_LOG is a printf → USB CDC-ACM → competes with
            // USB host task for CPU.  Under sustained failure that
            // printf cascade amplifies the underlying timeout (user-
            // identified feedback loop 2026-04-22).  `s_frames_dropped`
            // is the authoritative counter; async_stats exposes full
            // per-URB telemetry for root-cause work.
            static uint32_t s_fail_seen = 0;
            if ((s_fail_seen % 100) == 0) {
                SERR_LOG(SERR_DET_INVOKE_FAIL, (uint32_t)(-invoke_ms));
            }
            s_fail_seen++;
            sentai_health_fail(SUBSYS_DETECT);
            continue;
        }
        if (direct) s_dt_frames++;

        // NMS post-processing on output tensors
        TickType_t t_nms_start = xTaskGetTickCount();
        int16_t det_buf[DETECTION_MAX_DETS * 6];
        int det_count = 0;
        sentai_tpu_detect(s_conf_permil, s_iou_permil, s_max_dets,
                          det_buf, &det_count);
        TickType_t t_nms_end = xTaskGetTickCount();

        TickType_t t_end = xTaskGetTickCount();

        // Build result frame
        DetectionFrame result;
        result.count        = det_count;
        result.frame_seq    = frame_seq;
        result.cam_id       = (int8_t)frame_cam_id;
        result.inference_ms = static_cast<uint32_t>(invoke_ms);
        result.total_ms     = static_cast<uint32_t>((t_end - t0) * portTICK_PERIOD_MS);
        result.memcpy_ms    = static_cast<uint32_t>((t_memcpy_end - t_memcpy_start) * portTICK_PERIOD_MS);
        result.nms_ms       = static_cast<uint32_t>((t_nms_end - t_nms_start) * portTICK_PERIOD_MS);

        for (int i = 0; i < det_count && i < DETECTION_MAX_DETS; i++) {
            result.dets[i].x1          = det_buf[i * 6 + 0];
            result.dets[i].y1          = det_buf[i * 6 + 1];
            result.dets[i].x2          = det_buf[i * 6 + 2];
            result.dets[i].y2          = det_buf[i * 6 + 3];
            result.dets[i].conf_permil = det_buf[i * 6 + 4];
            result.dets[i].class_id    = det_buf[i * 6 + 5];
        }

        // --- Tracker update (if enabled) ---
        if (sentai_tracker_is_enabled()) {
            // Read IMU for camera motion compensation
            float xm, ym, zm, tc;
            if (sentai_imu_read_accel(&xm, &ym, &zm, &tc) == 0) {
                float pitch = atan2f(xm, sqrtf(ym * ym + zm * zm)) * 57.2957795f;
                float roll  = atan2f(ym, sqrtf(xm * xm + zm * zm)) * 57.2957795f;
                sentai_tracker_set_imu(pitch, roll);
            }
            sentai_tracker_update(result.dets, result.count,
                                  tensor_buf, w, h, ch, zp, frame_seq);
        }

        // Push to consumer queue (overwrite oldest if full)
        if (xQueueSend(s_det_queue, &result, 0) != pdTRUE) {
            DetectionFrame discard;
            if (xQueueReceive(s_det_queue, &discard, 0) == pdTRUE) {
                if (xQueueSend(s_det_queue, &result, 0) != pdTRUE) {
                    SERR_LOG(SERR_DET_QUEUE_FULL, s_frames_dropped);
                }
            } else {
                SERR_LOG(SERR_DET_QUEUE_CORRUPT, s_frames_processed);
            }
            __atomic_fetch_add(&s_frames_dropped, 1, __ATOMIC_RELAXED);
        }

        __atomic_fetch_add(&s_frames_processed, 1, __ATOMIC_RELAXED);
        sentai_health_success(SUBSYS_DETECT);
        // Record liveness timestamp: when InferTask last completed a full inference.
        s_last_infer_frame_tick = xTaskGetTickCount();

        // FPS throttle — run TPU no faster than `s_target_fps` frames
        // per second.  User theory: unthrottled 75 FPS draws too much
        // current → TPU brown-outs during sustained pipeline runs,
        // corrupting its USB state machine.  Camera is capped at
        // 45 fps anyway, so pacing TPU to match avoids wasted work
        // and lowers peak power draw.  Runtime-tunable via
        // sentai.pipeline.target_fps(n); 0 disables (run flat out).
        if (s_target_fps > 0) {
            const TickType_t min_period_ticks =
                pdMS_TO_TICKS(1000 / s_target_fps);
            TickType_t elapsed = xTaskGetTickCount() - t0;
            if (elapsed < min_period_ticks) {
                vTaskDelay(min_period_ticks - elapsed);
            }
        }
    }

    // InferTask exiting -- clear the fine-grained sync hook so stray
    // SendInputs calls from REPL tpu.invoke() don't touch a stale
    // semaphore that will no longer be attended to.
    sentai_tpu_set_input_done_sema(nullptr);
    s_infer_task = nullptr;
    vTaskDelete(nullptr);
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

extern "C" int sentai_detection_start(int conf_permil, int iou_permil, int max_dets) {
    if (s_running) {
        SERR_LOG(SERR_DET_ALREADY_RUN, 0);
        return -1;
    }
    if (!sentai_tpu_is_ready()) {
        SERR_LOG(SERR_DET_MODEL_NONE, 0);
        return -2;
    }
    if (!sentai_cam_is_initialized()) {
        SERR_LOG(SERR_DET_CAM_NONE, 0);
        return -5;
    }

    // Validate tensor fits staging buffer
    {
        int w, h, ch, type, zp;
        uint8_t* buf;
        if (sentai_get_tensor_info(&w, &h, &ch, &buf, &type, &zp) != 0) {
            SERR_LOG(SERR_DET_TENSOR_INFO, 0);
            return -3;
        }
        if (w * h * ch > kMaxStagingSize) {
            SERR_LOG(SERR_DET_TENSOR_SIZE, w * h * ch);
            return -4;
        }
    }

    // Store config
    s_conf_permil = conf_permil;
    s_iou_permil  = iou_permil;
    s_max_dets    = (max_dets > DETECTION_MAX_DETS) ? DETECTION_MAX_DETS :
                    (max_dets < 1) ? 1 : max_dets;

    // Create sync primitives (once, reusable across start/stop cycles)
    if (!s_sem_staging_free)
        s_sem_staging_free = xSemaphoreCreateBinary();
    if (!s_sem_prep_done)
        s_sem_prep_done = xSemaphoreCreateBinary();
    // Single OCRAM tensor → counting sem max=1 for strict serial.
    //   s_sem_bufs_free   : counting, max 1, init 1 — buffer is free
    //   s_sem_prep_done_c : counting, max 1, init 0 — no frame ready yet
    if (!s_sem_bufs_free)
        s_sem_bufs_free   = xSemaphoreCreateCounting(1, 1);
    if (!s_sem_prep_done_c)
        s_sem_prep_done_c = xSemaphoreCreateCounting(1, 0);
    if (!s_det_queue)
        s_det_queue = xQueueCreate(4, sizeof(DetectionFrame));

    if (!s_sem_staging_free || !s_sem_prep_done ||
        !s_sem_bufs_free   || !s_sem_prep_done_c ||
        !s_det_queue) {
        SERR_LOG(SERR_DET_SYNC_FAIL, 0);
        return -6;
    }

    // Reset state
    xQueueReset(s_det_queue);
    // Drain stale semaphore state from previous stop() (binary sem max=1)
    xSemaphoreTake(s_sem_staging_free, 0);
    xSemaphoreTake(s_sem_prep_done, 0);
    xSemaphoreGive(s_sem_staging_free);  // staging starts as "free"
    // s_sem_prep_done starts empty (not given) → InferTask waits for first frame

    // Drain + re-prime the counting sems so a second start() has the
    // same initial-state invariant (bufs_free=1, prep_done_c=0).
    while (xSemaphoreTake(s_sem_bufs_free,   0) == pdTRUE) { }
    while (xSemaphoreTake(s_sem_prep_done_c, 0) == pdTRUE) { }
    xSemaphoreGive(s_sem_bufs_free);  // single slot = free
    s_prep_count_dt  = 0;
    s_infer_count_dt = 0;

    s_frames_processed = 0;
    s_frames_dropped   = 0;
    s_start_tick       = xTaskGetTickCount();
    // force_parity bookkeeping: clear "last accepted cam_id" so the first
    // grab of the new run can never trigger a same-id skip.  Counters are
    // left intact (REPL can call force_parity_reset to clear them).
    s_last_grabbed_cam_id = -1;
    s_running          = true;

    // Create tasks using STATIC allocation.  Both at prio 2 —
    // above REPL (1) so they run when REPL blocks, but equal to
    // each other so InferTask can't preempt PrepTask.  User
    // hypothesis 2026-04-22: original InferTask prio 3 preempting
    // PrepTask prio 2 was disturbing something in the USB/TPU path.
    // (configUSE_TIME_SLICING=0, so equal prio means they yield
    // only via explicit blocks — both tasks block naturally on
    // sems, so this is deadlock-free.)
    s_prep_task = xTaskCreateStatic(prep_task_fn, "det_prep",
                                    kPrepStackWords, nullptr,
                                    tskIDLE_PRIORITY + 2,
                                    s_prep_stack, &s_prep_tcb);
    s_infer_task = nullptr;
    if (s_prep_task != nullptr) {
        s_infer_task = xTaskCreateStatic(infer_task_fn, "det_infer",
                                         kInferStackWords, nullptr,
                                         tskIDLE_PRIORITY + 2,  // was +3
                                         s_infer_stack, &s_infer_tcb);
    }

    if (s_prep_task == nullptr || s_infer_task == nullptr) {
        SERR_LOG(SERR_DET_TASK_FAIL,
                 ((s_prep_task  ? 1u : 0u) << 16) |
                 ((s_infer_task ? 1u : 0u) <<  8));
        s_running = false;
        // If PrepTask was created but InferTask failed, stop it cleanly
        if (s_prep_task) {
            xSemaphoreGive(s_sem_staging_free);  // unblock prep
            for (int i = 0; i < 50 && s_prep_task; i++)
                vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (s_infer_task) {
            xSemaphoreGive(s_sem_prep_done);
            for (int i = 0; i < 50 && s_infer_task; i++)
                vTaskDelay(pdMS_TO_TICKS(20));
        }
        return -7;
    }

    SERR_LOG(SERR_DET_STARTED, conf_permil);
    return 0;
}

extern "C" int sentai_detection_stop(void) {
    if (!s_running) return 0;

    s_running = false;

    // Unblock tasks that may be waiting on semaphores (both paths).
    if (s_sem_staging_free) xSemaphoreGive(s_sem_staging_free);
    if (s_sem_prep_done)    xSemaphoreGive(s_sem_prep_done);
    if (s_sem_bufs_free)    xSemaphoreGive(s_sem_bufs_free);
    if (s_sem_prep_done_c)  xSemaphoreGive(s_sem_prep_done_c);

    // Wait for both tasks to self-delete (up to 2 seconds)
    for (int i = 0; i < 100 &&
         (s_prep_task || s_infer_task); i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (s_prep_task || s_infer_task) {
        SERR_LOG(SERR_DET_EXIT_DIRTY, 0);
    }

    SERR_LOG(SERR_DET_STOPPED, s_frames_processed);
    return 0;
}

extern "C" int sentai_detection_get(DetectionFrame* frame, int timeout_ms) {
    if (!s_det_queue) return -2;
    if (!s_running && uxQueueMessagesWaiting(s_det_queue) == 0) return -2;

    TickType_t ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (xQueueReceive(s_det_queue, frame, ticks) == pdTRUE) {
        return frame->count;
    }
    return -1;  // timeout
}

extern "C" int sentai_detection_is_running(void) {
    return s_running ? 1 : 0;
}

extern "C" void sentai_detection_task_stall_ms(uint32_t* prep_stall_ms,
                                               uint32_t* infer_stall_ms) {
    // Return elapsed ms since PrepTask / InferTask last completed a frame.
    // Returns 0xFFFFFFFFUL when not running or before the first frame.
    if (!s_running) {
        if (prep_stall_ms)  *prep_stall_ms  = 0xFFFFFFFFUL;
        if (infer_stall_ms) *infer_stall_ms = 0xFFFFFFFFUL;
        return;
    }
    TickType_t now = xTaskGetTickCount();
    if (prep_stall_ms) {
        TickType_t last = s_last_prep_frame_tick;
        *prep_stall_ms = last ? (uint32_t)((now - last) * portTICK_PERIOD_MS)
                              : 0xFFFFFFFFUL;
    }
    if (infer_stall_ms) {
        TickType_t last = s_last_infer_frame_tick;
        *infer_stall_ms = last ? (uint32_t)((now - last) * portTICK_PERIOD_MS)
                               : 0xFFFFFFFFUL;
    }
}

extern "C" void sentai_detection_stats(uint32_t* frames_processed,
                                       uint32_t* frames_dropped,
                                       uint32_t* avg_fps_x10) {
    // P3 FIX: Use atomic loads for thread-safe reads
    uint32_t processed = __atomic_load_n(&s_frames_processed, __ATOMIC_RELAXED);
    uint32_t dropped   = __atomic_load_n(&s_frames_dropped, __ATOMIC_RELAXED);
    
    if (frames_processed) *frames_processed = processed;
    if (frames_dropped)   *frames_dropped   = dropped;
    if (avg_fps_x10) {
        uint32_t elapsed_ms = (xTaskGetTickCount() - s_start_tick) * portTICK_PERIOD_MS;
        if (elapsed_ms > 0 && processed > 0)
            *avg_fps_x10 = static_cast<uint32_t>(
                static_cast<uint64_t>(processed) * 10000 / elapsed_ms);
        else
            *avg_fps_x10 = 0;
    }
}

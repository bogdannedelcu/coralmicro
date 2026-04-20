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
    int sentai_tpu_detect(int conf_permil, int iou_permil, int max_dets,
                          int16_t* out_buf, int* out_count);
    void sentai_quant_uint8_to_int8(uint8_t* buf, int count, int zp);
    // Wrappers added for pipeline access:
    int  sentai_pxp_scale(const uint8_t* src, int sw, int sh,
                          uint8_t* dst, int dw, int dh);
    int  sentai_get_tensor_info(int* w, int* h, int* ch,
                                uint8_t** buf, int* type, int* zp);
    int  sentai_cam_grab_latest(uint8_t** raw);
    void sentai_cam_return_raw(int idx);
    uint32_t sentai_cam_get_frame_seq(void);
    int  sentai_cam_is_initialized(void);
    int  sentai_imu_read_accel(float* x_mg, float* y_mg, float* z_mg, float* temp_c);
}

// ---------------------------------------------------------------------------
// Pipeline state
// ---------------------------------------------------------------------------
namespace {

// Staging buffer — PXP output goes here while TPU processes previous frame.
// 64-byte aligned for optimal DMA/cache behavior (no boundary issues).
// Max model input size: 640×640×3 = 1,228,800 bytes.
static constexpr int kMaxStagingSize = 640 * 640 * 3;
static uint8_t s_staging_buf[kMaxStagingSize]
    __attribute__((aligned(64), section(".sdram_bss")));

// Pipeline configuration (set by start, read by tasks)
static int s_conf_permil = 500;
static int s_iou_permil  = 450;
static int s_max_dets    = DETECTION_MAX_DETS;

// Synchronization
static SemaphoreHandle_t s_sem_staging_free = nullptr;  // staging available for PrepTask
static SemaphoreHandle_t s_sem_prep_done    = nullptr;  // staging has new prepped frame
static QueueHandle_t     s_det_queue        = nullptr;  // detection results → consumer

// Task handles
static TaskHandle_t s_prep_task  = nullptr;
static TaskHandle_t s_infer_task = nullptr;

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
        // Wait until staging buffer is free
        if (xSemaphoreTake(s_sem_staging_free, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;  // timeout — recheck s_running (normal when pipeline just started)
        }
        if (!s_running) break;

        // Get model tensor dimensions (for PXP target size)
        int w = 0, h = 0, ch = 0, type = 0, zp = 0;
        uint8_t* tensor_buf = nullptr;
        if (sentai_get_tensor_info(&w, &h, &ch, &tensor_buf, &type, &zp) != 0) {
            sentai_health_fail(SUBSYS_DETECT);
            xSemaphoreGive(s_sem_staging_free);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int total = w * h * ch;
        if (total > kMaxStagingSize) {
            SERR_LOG(SERR_DET_TENSOR_SIZE, total);
            sentai_health_fail(SUBSYS_DETECT);
            xSemaphoreGive(s_sem_staging_free);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Get latest camera frame (drains stale ones)
        uint8_t* raw = nullptr;
        int idx = sentai_cam_grab_latest(&raw);
        if (idx < 0 || !raw) {
            // Rate-limit: only report to health after 10 consecutive camera misses.
            // A single miss is normal (no frame available yet); sustained misses
            // indicate a camera hardware or driver failure.
            if (++cam_miss_streak >= 10) {
                sentai_health_fail(SUBSYS_DETECT);
                cam_miss_streak = 0;
            }
            xSemaphoreGive(s_sem_staging_free);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        cam_miss_streak = 0;  // reset on successful frame grab

        // PXP hardware: XRGB8888 → RGB888P, scaled to model input size
        // Writes to staging buffer (NOT the TFLite tensor).
        int rc = sentai_pxp_scale(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  s_staging_buf, w, h);
        sentai_cam_return_raw(idx);

        if (rc != 0) {
            // PXP failure is always reportable — hardware error, not a transient miss.
            SERR_LOG(SERR_DET_INVOKE_FAIL, (uint32_t)rc);  // reuse invoke-fail code for PXP
            sentai_health_fail(SUBSYS_DETECT);
            xSemaphoreGive(s_sem_staging_free);
            continue;
        }

        // Int8 quantization in-place on staging buffer
        if (type == 9 /* kTfLiteInt8 */) {
            sentai_quant_uint8_to_int8(s_staging_buf, total, zp);
        }

        // Publish metadata (safe: InferTask won't read until we signal)
        s_stg_w  = w;
        s_stg_h  = h;
        s_stg_ch = ch;
        s_stg_total = total;
        s_stg_frame_seq = sentai_cam_get_frame_seq();

        // Signal InferTask: staging buffer has a new prepared frame.
        // Record liveness timestamp *before* signalling so InferTask's pick-up
        // can never race with an uninitialized tick.
        s_last_prep_frame_tick = xTaskGetTickCount();
        xSemaphoreGive(s_sem_prep_done);
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
        // Wait for PrepTask to deliver a new frame
        if (xSemaphoreTake(s_sem_prep_done, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;  // timeout — recheck s_running
        }
        if (!s_running) break;

        TickType_t t0 = xTaskGetTickCount();

        // Read staging metadata
        int total = s_stg_total;
        uint32_t frame_seq = s_stg_frame_seq;

        // Get tensor buffer pointer
        int w = 0, h = 0, ch = 0, type = 0, zp = 0;
        uint8_t* tensor_buf = nullptr;
        if (sentai_get_tensor_info(&w, &h, &ch, &tensor_buf, &type, &zp) != 0) {
            xSemaphoreGive(s_sem_staging_free);
            continue;
        }

        // Copy staging → TFLite input tensor.  Both buffers live in SDRAM, so
        // this transfer shares bandwidth with the TPU USB input upload that
        // follows.  When the runtime flag is ON we use eDMA (back-to-back
        // SDRAM bursts, ~5× faster than CPU memcpy due to SEMC write-allocate
        // overhead).  When OFF, or if DMA submission fails, we fall back to
        // plain memcpy — lets us A/B the optimisation against baseline in
        // a single firmware build by toggling sentai.pipeline.dma_memcpy(0/1).
        TickType_t t_memcpy_start = xTaskGetTickCount();
        bool dma_ok = false;
        if (s_dma_memcpy_enabled
            && ((((uintptr_t)tensor_buf | (uintptr_t)s_staging_buf
                  | (uint32_t)total) & 0x3u) == 0u)) {
            dma_ok = sentai_dma_memcpy(tensor_buf, s_staging_buf, total);
        }
        if (!dma_ok) {
            memcpy(tensor_buf, s_staging_buf, total);
        }
        TickType_t t_memcpy_end = xTaskGetTickCount();

        // FREE staging immediately — PrepTask can start next frame NOW
        xSemaphoreGive(s_sem_staging_free);

        // --- From here, PrepTask runs in parallel (camera + PXP) ---

        // TPU inference (~20-50ms, blocks on USB → PrepTask gets CPU)
        int invoke_ms = sentai_tpu_invoke_internal();
        if (invoke_ms < 0) {
            SERR_LOG(SERR_DET_INVOKE_FAIL, (uint32_t)(-invoke_ms));
            sentai_health_fail(SUBSYS_DETECT);
            continue;
        }

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

        // Periodic stats logged every 30 frames
        // Note: stats available via sentai.det.stats() - no verbose print
    }

    // InferTask exiting
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
    if (!s_det_queue)
        s_det_queue = xQueueCreate(4, sizeof(DetectionFrame));

    if (!s_sem_staging_free || !s_sem_prep_done || !s_det_queue) {
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

    s_frames_processed = 0;
    s_frames_dropped   = 0;
    s_start_tick       = xTaskGetTickCount();
    s_running          = true;

    // Create tasks
    //   PrepTask  prio 2: above mp_repl (1), below InferTask (3)
    //   InferTask prio 3: same as app_main — when it blocks on USB, PrepTask runs
    BaseType_t r1 = xTaskCreate(prep_task_fn, "det_prep",
                                configMINIMAL_STACK_SIZE * 8,
                                nullptr, tskIDLE_PRIORITY + 2, &s_prep_task);
    BaseType_t r2 = pdFAIL;
    if (r1 == pdPASS) {
        r2 = xTaskCreate(infer_task_fn, "det_infer",
                         configMINIMAL_STACK_SIZE * 12,
                         nullptr, tskIDLE_PRIORITY + 3, &s_infer_task);
    }

    if (r1 != pdPASS || r2 != pdPASS) {
        SERR_LOG(SERR_DET_TASK_FAIL, (r1 << 8) | r2);
        s_running = false;
        // If PrepTask was created but InferTask failed, stop it cleanly
        if (r1 == pdPASS && s_prep_task) {
            xSemaphoreGive(s_sem_staging_free);  // unblock prep
            for (int i = 0; i < 50 && s_prep_task; i++)
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

    // Unblock tasks that may be waiting on semaphores
    if (s_sem_staging_free) xSemaphoreGive(s_sem_staging_free);
    if (s_sem_prep_done)    xSemaphoreGive(s_sem_prep_done);

    // Wait for both tasks to self-delete (up to 2 seconds)
    for (int i = 0; i < 100 && (s_prep_task || s_infer_task); i++) {
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

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

// Statistics
static volatile uint32_t s_frames_processed = 0;
static volatile uint32_t s_frames_dropped   = 0;
static TickType_t        s_start_tick        = 0;

// Staging metadata (written by PrepTask, read by InferTask after semaphore)
static int      s_stg_w  = 0;
static int      s_stg_h  = 0;
static int      s_stg_ch = 0;
static int      s_stg_total = 0;
static uint32_t s_stg_frame_seq = 0;

// ---------------------------------------------------------------------------
// PrepTask: Camera → PXP → int8 quant → staging buffer
// ---------------------------------------------------------------------------
static void prep_task_fn(void* /*param*/) {
    printf("[DetPipe] PrepTask started\r\n");

    while (s_running) {
        // Wait until staging buffer is free
        if (xSemaphoreTake(s_sem_staging_free, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;  // timeout — recheck s_running
        }
        if (!s_running) break;

        // Get model tensor dimensions (for PXP target size)
        int w = 0, h = 0, ch = 0, type = 0, zp = 0;
        uint8_t* tensor_buf = nullptr;
        if (sentai_get_tensor_info(&w, &h, &ch, &tensor_buf, &type, &zp) != 0) {
            xSemaphoreGive(s_sem_staging_free);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int total = w * h * ch;
        if (total > kMaxStagingSize) {
            printf("[DetPipe] ERROR: tensor %dx%dx%d = %d > staging %d\r\n",
                   w, h, ch, total, kMaxStagingSize);
            xSemaphoreGive(s_sem_staging_free);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Get latest camera frame (drains stale ones)
        uint8_t* raw = nullptr;
        int idx = sentai_cam_grab_latest(&raw);
        if (idx < 0 || !raw) {
            xSemaphoreGive(s_sem_staging_free);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // PXP hardware: XRGB8888 → RGB888P, scaled to model input size
        // Writes to staging buffer (NOT the TFLite tensor).
        int rc = sentai_pxp_scale(raw, DEMO_CAMERA_WIDTH, DEMO_CAMERA_HEIGHT,
                                  s_staging_buf, w, h);
        sentai_cam_return_raw(idx);

        if (rc != 0) {
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

        // Signal InferTask: staging buffer has a new prepared frame
        xSemaphoreGive(s_sem_prep_done);
    }

    printf("[DetPipe] PrepTask exiting\r\n");
    s_prep_task = nullptr;
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// InferTask: memcpy staging→tensor → Invoke → NMS → queue
// ---------------------------------------------------------------------------
static void infer_task_fn(void* /*param*/) {
    printf("[DetPipe] InferTask started\r\n");

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

        // Copy staging → TFLite input tensor (~0.5ms for 1.2MB)
        memcpy(tensor_buf, s_staging_buf, total);

        // FREE staging immediately — PrepTask can start next frame NOW
        xSemaphoreGive(s_sem_staging_free);

        // --- From here, PrepTask runs in parallel (camera + PXP) ---

        // TPU inference (~20-50ms, blocks on USB → PrepTask gets CPU)
        int invoke_ms = sentai_tpu_invoke_internal();
        if (invoke_ms < 0) {
            printf("[DetPipe] Invoke failed: %d\r\n", invoke_ms);
            continue;
        }

        // NMS post-processing on output tensors
        int16_t det_buf[DETECTION_MAX_DETS * 6];
        int det_count = 0;
        sentai_tpu_detect(s_conf_permil, s_iou_permil, s_max_dets,
                          det_buf, &det_count);

        TickType_t t_end = xTaskGetTickCount();

        // Build result frame
        DetectionFrame result;
        result.count        = det_count;
        result.frame_seq    = frame_seq;
        result.inference_ms = static_cast<uint32_t>(invoke_ms);
        result.total_ms     = static_cast<uint32_t>((t_end - t0) * portTICK_PERIOD_MS);

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
            xQueueReceive(s_det_queue, &discard, 0);
            xQueueSend(s_det_queue, &result, 0);
            s_frames_dropped++;
        }

        s_frames_processed++;

        // Periodic stats (every 30 frames ≈ once per second at ~30fps)
        if ((s_frames_processed % 30) == 0) {
            uint32_t elapsed_ms = (t_end - s_start_tick) * portTICK_PERIOD_MS;
            uint32_t fps_x10 = 0;
            if (elapsed_ms > 0)
                fps_x10 = static_cast<uint32_t>(
                    static_cast<uint64_t>(s_frames_processed) * 10000 / elapsed_ms);
            printf("[DetPipe] #%lu  %d dets  invoke=%dms  total=%lums  "
                   "%lu.%lu fps  (dropped %lu)\r\n",
                   (unsigned long)s_frames_processed, det_count, invoke_ms,
                   (unsigned long)result.total_ms,
                   (unsigned long)(fps_x10 / 10), (unsigned long)(fps_x10 % 10),
                   (unsigned long)s_frames_dropped);
        }
    }

    printf("[DetPipe] InferTask exiting\r\n");
    s_infer_task = nullptr;
    vTaskDelete(nullptr);
}

}  // anonymous namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

extern "C" int sentai_detection_start(int conf_permil, int iou_permil, int max_dets) {
    if (s_running) {
        printf("[DetPipe] Already running — stop first\r\n");
        return -1;
    }
    if (!sentai_tpu_is_ready()) {
        printf("[DetPipe] ERROR: model not loaded\r\n");
        return -2;
    }
    if (!sentai_cam_is_initialized()) {
        printf("[DetPipe] ERROR: camera not initialized\r\n");
        return -5;
    }

    // Validate tensor fits staging buffer
    {
        int w, h, ch, type, zp;
        uint8_t* buf;
        if (sentai_get_tensor_info(&w, &h, &ch, &buf, &type, &zp) != 0) {
            printf("[DetPipe] ERROR: cannot read tensor info\r\n");
            return -3;
        }
        if (w * h * ch > kMaxStagingSize) {
            printf("[DetPipe] ERROR: tensor %dx%dx%d too large\r\n", w, h, ch);
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
        printf("[DetPipe] ERROR: failed to create sync primitives\r\n");
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
        printf("[DetPipe] ERROR: task creation failed (r1=%d r2=%d)\r\n",
               (int)r1, (int)r2);
        s_running = false;
        // If PrepTask was created but InferTask failed, stop it cleanly
        if (r1 == pdPASS && s_prep_task) {
            xSemaphoreGive(s_sem_staging_free);  // unblock prep
            for (int i = 0; i < 50 && s_prep_task; i++)
                vTaskDelay(pdMS_TO_TICKS(20));
        }
        return -7;
    }

    printf("[DetPipe] Started  conf>%d.%d%%  iou>%d.%d%%  max=%d\r\n",
           conf_permil / 10, conf_permil % 10,
           iou_permil / 10, iou_permil % 10,
           s_max_dets);
    return 0;
}

extern "C" int sentai_detection_stop(void) {
    if (!s_running) return 0;

    printf("[DetPipe] Stopping...\r\n");
    s_running = false;

    // Unblock tasks that may be waiting on semaphores
    if (s_sem_staging_free) xSemaphoreGive(s_sem_staging_free);
    if (s_sem_prep_done)    xSemaphoreGive(s_sem_prep_done);

    // Wait for both tasks to self-delete (up to 2 seconds)
    for (int i = 0; i < 100 && (s_prep_task || s_infer_task); i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (s_prep_task || s_infer_task) {
        printf("[DetPipe] WARNING: tasks did not exit cleanly\r\n");
    }

    uint32_t elapsed_ms = (xTaskGetTickCount() - s_start_tick) * portTICK_PERIOD_MS;
    uint32_t fps_x10 = 0;
    if (elapsed_ms > 0 && s_frames_processed > 0)
        fps_x10 = static_cast<uint32_t>(
            static_cast<uint64_t>(s_frames_processed) * 10000 / elapsed_ms);

    printf("[DetPipe] Stopped  %lu frames  %lu dropped  %lu.%lu fps avg\r\n",
           (unsigned long)s_frames_processed, (unsigned long)s_frames_dropped,
           (unsigned long)(fps_x10 / 10), (unsigned long)(fps_x10 % 10));
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

extern "C" void sentai_detection_stats(uint32_t* frames_processed,
                                       uint32_t* frames_dropped,
                                       uint32_t* avg_fps_x10) {
    if (frames_processed) *frames_processed = s_frames_processed;
    if (frames_dropped)   *frames_dropped   = s_frames_dropped;
    if (avg_fps_x10) {
        uint32_t elapsed_ms = (xTaskGetTickCount() - s_start_tick) * portTICK_PERIOD_MS;
        if (elapsed_ms > 0 && s_frames_processed > 0)
            *avg_fps_x10 = static_cast<uint32_t>(
                static_cast<uint64_t>(s_frames_processed) * 10000 / elapsed_ms);
        else
            *avg_fps_x10 = 0;
    }
}

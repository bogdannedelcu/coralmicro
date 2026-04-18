// detection_task.h — Pipelined camera→PXP→TPU detection task
// Two FreeRTOS tasks run in parallel:
//   PrepTask  (prio 2): camera capture → PXP scale → int8 quant → staging buffer
//   InferTask (prio 3): memcpy staging→tensor → TPU Invoke → NMS → queue
// While TPU processes frame N, PrepTask prepares frame N+1 in the staging buffer.
// Results are pushed to a FreeRTOS queue consumed from Python.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum detections per frame
#define DETECTION_MAX_DETS 50

// One detection result (same layout as sentai_tpu_detect output)
typedef struct Detection {
    int16_t x1, y1, x2, y2;    // bounding box in model-input pixel space
    int16_t conf_permil;         // confidence × 1000  (e.g. 850 = 85.0%)
    int16_t class_id;            // class index (0-based)
} Detection;

// A complete frame of detection results
typedef struct DetectionFrame {
    Detection dets[DETECTION_MAX_DETS];
    int       count;             // valid detections (0..DETECTION_MAX_DETS)
    uint32_t  frame_seq;         // monotonic camera frame sequence number
    uint32_t  inference_ms;      // TPU invoke duration (ms)
    uint32_t  total_ms;          // full pipeline: cam + PXP + memcpy + invoke + NMS
} DetectionFrame;

// ---------------------------------------------------------------------------
// Public API (callable from any task, including MicroPython bridge)
// ---------------------------------------------------------------------------

// Start the detection pipeline.
// Requires: model loaded (sentai_load_model) + camera initialized (sentai_cam_init).
//   conf_permil : confidence threshold × 1000  (e.g. 500 = 50%)
//   iou_permil  : NMS IoU threshold × 1000     (e.g. 450 = 45%)
//   max_dets    : max detections per frame      (1..DETECTION_MAX_DETS)
// Returns 0 on success, negative on error.
int sentai_detection_start(int conf_permil, int iou_permil, int max_dets);

// Stop the detection pipeline.  Waits for both tasks to exit (up to 2 s).
// Returns 0 on success.
int sentai_detection_stop(void);

// Get the next detection result.
// Blocks up to timeout_ms milliseconds.
// Fills *frame with detection data.
// Returns detection count (>=0) on success, -1 on timeout, -2 if not running.
int sentai_detection_get(DetectionFrame* frame, int timeout_ms);

// 1 if pipeline is running, 0 otherwise.
int sentai_detection_is_running(void);

// Per-task liveness: elapsed ms since PrepTask / InferTask last completed a frame.
// Returns 0xFFFFFFFF when pipeline not running or before the first frame completes.
// Use to distinguish a stuck PrepTask (camera/PXP hang) from a stuck InferTask (TPU hang).
void sentai_detection_task_stall_ms(uint32_t* prep_stall_ms, uint32_t* infer_stall_ms);

// Pipeline statistics since last start.
void sentai_detection_stats(uint32_t* frames_processed,
                            uint32_t* frames_dropped,
                            uint32_t* avg_fps_x10);

#ifdef __cplusplus
}
#endif

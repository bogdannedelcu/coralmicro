// Emulator backend for targets that expose the shared sentai.pipeline binding
// but do not link the real camera/PrepTask/InferTask runtime.
//
// Keep this file as a C ABI backend only.  Python semantics live in the shared
// examples/sentai_runtime/bindings/modsentai_pipeline.c binding.

#include <stdint.h>
#include <string.h>

#include "examples/sentai_runtime/detection_task.h"
#include "examples/sentai_runtime/sentai_tracker.h"

static int g_dma_memcpy_enabled = 0;
static int g_direct_tensor_enabled = 0;
static int g_target_fps = 0;
static int g_prep_fps = 0;
static int g_debug_prep_mode = 0;
static int g_debug_no_invoke = 1;
static int g_invokes_per_frame = 1;
static int g_multi_invoke_mode = 0;
static int g_force_parity = 0;
static int g_loop_delay_ms = 0;
static int g_slot_for_cam[TRACKER_MAX_CAMERAS] = {0, 0};
static TrackerConfig g_tracker_cfg = {
    .min_hits = 3,
    .max_misses = 5,
    .max_lost = 1,
    .enable_histogram = 0,
    .enable_imu_cmc = 0,
    .iou_thresh = 300,
    .hist_weight = 0,
    .high_thresh = 500,
    .low_thresh = 100,
};
static CameraConfig g_camera_cfg[TRACKER_MAX_CAMERAS] = {
    {70.8f, 43.4f, 0.0f, 0.0f, 0.0f, 0},
    {70.8f, 43.4f, 0.0f, 0.0f, 0.0f, 0},
};
static int g_active_camera = 0;
static int g_tracker_enabled = 0;
static int g_altitude_cm = 0;
static int g_heading_deg = -1;
static float g_lat = 0.0f;
static float g_lon = 0.0f;
static float g_pitch_deg = 0.0f;
static float g_roll_deg = 0.0f;

int sentai_detection_start(int conf_permil, int iou_permil, int max_dets) {
    (void)conf_permil;
    (void)iou_permil;
    (void)max_dets;
    return -2;
}

int sentai_detection_start_one_shot(int conf_permil,
                                    int iou_permil,
                                    int max_dets) {
    (void)conf_permil;
    (void)iou_permil;
    (void)max_dets;
    return -2;
}

int sentai_prep_task_start_only(void) { return -2; }
int sentai_prep_task_stop_only(void) { return 0; }
int sentai_detection_stop(void) { return 0; }

int sentai_detection_get(DetectionFrame* frame, int timeout_ms) {
    (void)timeout_ms;
    if (frame) memset(frame, 0, sizeof(*frame));
    return -2;
}

int sentai_detection_get_after(DetectionFrame* frame,
                               int timeout_ms,
                               uint32_t after_frame_seq) {
    (void)after_frame_seq;
    return sentai_detection_get(frame, timeout_ms);
}

uint32_t sentai_detection_event_count(void) { return 0; }

int sentai_detection_wait_event(uint32_t after_count, int timeout_ms) {
    (void)after_count;
    (void)timeout_ms;
    return -2;
}

int sentai_detection_is_running(void) { return 0; }

void sentai_detection_stats(uint32_t* frames_processed,
                            uint32_t* frames_dropped,
                            uint32_t* avg_fps_x10) {
    if (frames_processed) *frames_processed = 0;
    if (frames_dropped) *frames_dropped = 0;
    if (avg_fps_x10) *avg_fps_x10 = 0;
}

void sentai_detection_task_stall_ms(uint32_t* prep_stall_ms,
                                    uint32_t* infer_stall_ms) {
    if (prep_stall_ms) *prep_stall_ms = 0xFFFFFFFFu;
    if (infer_stall_ms) *infer_stall_ms = 0xFFFFFFFFu;
}

int sentai_dma_memcpy_get(void) { return g_dma_memcpy_enabled; }
void sentai_dma_memcpy_set(int v) { g_dma_memcpy_enabled = v ? 1 : 0; }

int sentai_pipeline_direct_tensor_get(void) { return g_direct_tensor_enabled; }
int sentai_pipeline_direct_tensor_set(int v) {
    g_direct_tensor_enabled = v ? 1 : 0;
    return 0;
}
void sentai_pipeline_direct_tensor_stats(uint32_t* frames,
                                         uint32_t* prep_to,
                                         uint32_t* infer_to,
                                         uint32_t* swap_fail) {
    if (frames) *frames = 0;
    if (prep_to) *prep_to = 0;
    if (infer_to) *infer_to = 0;
    if (swap_fail) *swap_fail = 0;
}

int sentai_pipeline_target_fps_get(void) { return g_target_fps; }
void sentai_pipeline_target_fps_set(int v) { g_target_fps = v < 0 ? 0 : v; }
int sentai_pipeline_prep_fps_get(void) { return g_prep_fps; }
void sentai_pipeline_prep_fps_set(int v) { g_prep_fps = v < 0 ? 0 : v; }

void sentai_prep_stage_stats(uint32_t* frames, uint32_t* sem_wait,
                             uint32_t* cam_grab, uint32_t* pxp,
                             uint32_t* quant, uint32_t* total) {
    if (frames) *frames = 0;
    if (sem_wait) *sem_wait = 0;
    if (cam_grab) *cam_grab = 0;
    if (pxp) *pxp = 0;
    if (quant) *quant = 0;
    if (total) *total = 0;
}

void sentai_prep_stage_reset(void) {}

int sentai_pipeline_debug_prep_mode_get(void) { return g_debug_prep_mode; }
void sentai_pipeline_debug_prep_mode_set(int v) { g_debug_prep_mode = v; }
int sentai_pipeline_debug_no_invoke_get(void) { return g_debug_no_invoke; }
void sentai_pipeline_debug_no_invoke_set(int v) { g_debug_no_invoke = v ? 1 : 0; }

void sentai_pipeline_infer_stats(uint32_t* ok, uint32_t* fail,
                                 uint32_t* ms_sum, int32_t* last_rc) {
    if (ok) *ok = 0;
    if (fail) *fail = 0;
    if (ms_sum) *ms_sum = 0;
    if (last_rc) *last_rc = -2;
}

void sentai_pipeline_infer_reset(void) {}

int sentai_pipeline_invokes_per_frame_get(void) { return g_invokes_per_frame; }
void sentai_pipeline_invokes_per_frame_set(int v) {
    g_invokes_per_frame = v < 1 ? 1 : v;
}
int sentai_pipeline_multi_invoke_mode_get(void) { return g_multi_invoke_mode; }
void sentai_pipeline_multi_invoke_mode_set(int v) { g_multi_invoke_mode = v; }

int sentai_pipeline_set_slot_for_cam(int cam_id, int slot) {
    if (cam_id < 0 || cam_id >= TRACKER_MAX_CAMERAS) return -1;
    g_slot_for_cam[cam_id] = slot;
    return 0;
}

int sentai_pipeline_get_slot_for_cam(int cam_id) {
    if (cam_id < 0 || cam_id >= TRACKER_MAX_CAMERAS) return -1;
    return g_slot_for_cam[cam_id];
}

void sentai_pipeline_slot_stats(uint32_t* per_slot, int n) {
    for (int i = 0; per_slot && i < n; ++i) per_slot[i] = 0;
}

void sentai_pipeline_slot_stats_reset(void) {}

int sentai_pipeline_force_parity_get(void) { return g_force_parity; }
void sentai_pipeline_force_parity_set(int v) { g_force_parity = v ? 1 : 0; }
void sentai_pipeline_force_parity_stats(uint32_t* skipped, uint32_t* timeout) {
    if (skipped) *skipped = 0;
    if (timeout) *timeout = 0;
}
void sentai_pipeline_force_parity_reset(void) {}

int sentai_pipeline_loop_delay_get(void) { return g_loop_delay_ms; }
void sentai_pipeline_loop_delay_set(int v) { g_loop_delay_ms = v < 0 ? 0 : v; }

int sentai_cam_is_initialized(void) { return 0; }
int sentai_cam_test_pattern(int cam_id, int mode) {
    (void)cam_id;
    (void)mode;
    return -1;
}
int sentai_cam_ratio_set(uint32_t a, uint32_t b) {
    (void)a;
    (void)b;
    return -1;
}
void sentai_cam_ratio_get(uint32_t* a, uint32_t* b) {
    if (a) *a = 1;
    if (b) *b = 1;
}

void sentai_tracker_init(void) {}
void sentai_tracker_reset(void) {}
int sentai_tracker_update(const struct Detection* dets, int n_dets,
                          const uint8_t* tensor_buf, int tw, int th,
                          int tch, int zp, uint32_t frame_seq) {
    (void)dets;
    (void)n_dets;
    (void)tensor_buf;
    (void)tw;
    (void)th;
    (void)tch;
    (void)zp;
    (void)frame_seq;
    return 0;
}
int sentai_tracker_get_tracks(TrackedObject* out, int max) {
    (void)out;
    (void)max;
    return 0;
}
int sentai_tracker_get_event(TrackEvent* event, int timeout_ms) {
    (void)event;
    (void)timeout_ms;
    return 0;
}
int sentai_tracker_num_active(void) { return 0; }
void sentai_tracker_set_config(const TrackerConfig* cfg) {
    if (cfg) g_tracker_cfg = *cfg;
}
void sentai_tracker_get_config(TrackerConfig* cfg) {
    if (cfg) *cfg = g_tracker_cfg;
}
void sentai_tracker_set_imu(float pitch_deg, float roll_deg) {
    g_pitch_deg = pitch_deg;
    g_roll_deg = roll_deg;
}
void sentai_tracker_set_camera(int cam_id, const CameraConfig* cfg) {
    if (cfg && cam_id >= 0 && cam_id < TRACKER_MAX_CAMERAS) {
        g_camera_cfg[cam_id] = *cfg;
    }
}
void sentai_tracker_get_camera(int cam_id, CameraConfig* cfg) {
    if (!cfg) return;
    if (cam_id < 0 || cam_id >= TRACKER_MAX_CAMERAS) cam_id = 0;
    *cfg = g_camera_cfg[cam_id];
}
void sentai_tracker_set_active_camera(int cam_id) {
    if (cam_id >= 0 && cam_id < TRACKER_MAX_CAMERAS) g_active_camera = cam_id;
}
int sentai_tracker_get_active_camera(void) { return g_active_camera; }
void sentai_tracker_set_pose(int altitude_cm, int heading_deg,
                             float lat, float lon) {
    g_altitude_cm = altitude_cm;
    g_heading_deg = heading_deg;
    g_lat = lat;
    g_lon = lon;
}
void sentai_tracker_get_pose(int* altitude_cm, int* heading_deg,
                             float* lat, float* lon,
                             float* pitch_deg, float* roll_deg) {
    if (altitude_cm) *altitude_cm = g_altitude_cm;
    if (heading_deg) *heading_deg = g_heading_deg;
    if (lat) *lat = g_lat;
    if (lon) *lon = g_lon;
    if (pitch_deg) *pitch_deg = g_pitch_deg;
    if (roll_deg) *roll_deg = g_roll_deg;
}
void sentai_tracker_set_enabled(int enable) { g_tracker_enabled = enable ? 1 : 0; }
int sentai_tracker_is_enabled(void) { return g_tracker_enabled; }
int sentai_tracker_get_footprint(float* out_lat, float* out_lon) {
    (void)out_lat;
    (void)out_lon;
    return 0;
}

// Shared VisionMessage construction helpers for mesh and link transports.
//
// Both sentai_mesh.cc and sentai_link.cc build identical VisionMessage
// protobuf payloads — only the transport layer differs (Meshtastic serial
// framing vs MAVLink base64 STATUSTEXT chunks).  This header provides:
//
//   - vision_build_pose()         — SensorPose from tracker
//   - vision_build_config()       — SensorConfig from active camera
//   - VisionTxState               — per-transport change-detection state
//   - vision_attach_pose_config() — attach pose/config with dedup + periodic
//   - vision_fill_header/new_detection/update/delete() — body builders
//
// All functions are static inline — each translation unit gets its own copy
// (no linker issues, minimal code-size overhead on Cortex-M7).

#ifndef SENTAI_VISION_COMMON_H_
#define SENTAI_VISION_COMMON_H_

#include "generated/visionmesh.pb.h"
#include "sentai_tracker.h"
#include <cstring>

#define VISION_APP_VERSION      3
#define VISION_MODEL_NAME_MAX   32   // max model_name length on wire

// ===================== Model name (set by load_model) =====================

// Global model name — set by sentai_load_model(), read by attach logic.
// Protected: written only from Python thread (load_model), read from TX path.
static char  g_vision_model_name[VISION_MODEL_NAME_MAX + 1] = {0};
static bool  g_vision_model_dirty = false;  // true after load_model

// Called by sentai_load_model() after successful load.
static inline void vision_set_model_name(const char* path) {
    // Extract filename without path and extension:
    //   "/models/yolov8n_coco.tflite" → "yolov8n_coco"
    const char* base = path;
    for (const char* p = path; *p; p++) {
        if (*p == '/') base = p + 1;
    }
    // Copy up to max, strip .tflite extension
    int len = 0;
    for (const char* p = base; *p && *p != '.' && len < VISION_MODEL_NAME_MAX; p++, len++) {
        g_vision_model_name[len] = *p;
    }
    g_vision_model_name[len] = '\0';
    g_vision_model_dirty = true;
}

// Returns current model name (empty string if no model loaded).
static inline const char* vision_get_model_name(void) {
    return g_vision_model_name;
}

// ===================== Pose / Config builders =====================

// Build SensorPose from current tracker state (IMU + altitude + heading + GPS).
// Returns true if valid pose available (altitude > 0), false otherwise.
static inline bool vision_build_pose(visionmesh_SensorPose* pose) {
    *pose = visionmesh_SensorPose_init_zero;
    int alt = 0, hdg = -1;
    float lat = 0, lon = 0, pitch = 0, roll = 0;
    sentai_tracker_get_pose(&alt, &hdg, &lat, &lon, &pitch, &roll);
    if (alt <= 0) return false;

    pose->pitch_deg   = (int32_t)pitch;
    pose->roll_deg    = (int32_t)roll;
    pose->altitude_cm = (uint32_t)alt;
    pose->heading_deg = (uint32_t)(hdg >= 0 ? hdg : 0);

    if (lat != 0.0f || lon != 0.0f) {
        pose->has_camera_gps = true;
        pose->camera_gps.lat_e7 = (int32_t)(lat * 1e7f);
        pose->camera_gps.lon_e7 = (int32_t)(lon * 1e7f);
    }
    return true;
}

// Build SensorConfig from the currently active camera's mount geometry.
static inline void vision_build_config(visionmesh_SensorConfig* cfg) {
    *cfg = visionmesh_SensorConfig_init_zero;
    int cam_id = sentai_tracker_get_active_camera();
    CameraConfig ccfg;
    sentai_tracker_get_camera(cam_id, &ccfg);
    cfg->camera_id      = (uint32_t)cam_id;
    cfg->fov_h_e1       = (uint32_t)(ccfg.fov_h_deg * 10.0f + 0.5f);
    cfg->fov_v_e1       = (uint32_t)(ccfg.fov_v_deg * 10.0f + 0.5f);
    cfg->mount_pitch_e1 = (int32_t)(ccfg.mount_pitch_deg * 10.0f);
    cfg->mount_roll_e1  = (int32_t)(ccfg.mount_roll_deg  * 10.0f);
    cfg->mount_yaw_e1   = (int32_t)(ccfg.mount_yaw_deg   * 10.0f);
    cfg->ground_ref     = (uint32_t)ccfg.ground_ref;
}

// ===================== Per-transport state =====================

// Tracks what was last sent on a given transport, for change detection.
typedef struct {
    visionmesh_SensorPose   last_sent_pose;
    visionmesh_SensorConfig last_sent_config;
    bool     pose_ever_sent;
    bool     config_ever_sent;
    bool     model_sent;       // true after model_name was sent on this transport
} VisionTxState;

#define VISION_TX_STATE_INIT { \
    visionmesh_SensorPose_init_zero, \
    visionmesh_SensorConfig_init_zero, \
    false, false, false }

// ===================== Attach metadata (pose + config + model) =====================

// Attach SensorPose (only when changed), SensorConfig (only when changed),
// and model_name (once after load_model) to a VisionMessage.
static inline void vision_attach_metadata(visionmesh_VisionMessage* vision,
                                          VisionTxState* state) {
    // Pose: build from tracker, include only if different from last sent
    visionmesh_SensorPose pose;
    if (vision_build_pose(&pose)) {
        if (!state->pose_ever_sent ||
            memcmp(&pose, &state->last_sent_pose, sizeof(pose)) != 0) {
            vision->has_pose = true;
            vision->pose = pose;
            state->last_sent_pose = pose;
            state->pose_ever_sent = true;
        }
    }

    // Config: build from active camera, include only if changed
    visionmesh_SensorConfig cfg;
    vision_build_config(&cfg);
    if (!state->config_ever_sent ||
        memcmp(&cfg, &state->last_sent_config, sizeof(cfg)) != 0) {
        vision->has_config = true;
        vision->config = cfg;
        state->last_sent_config = cfg;
        state->config_ever_sent = true;
    }

    // Model name: send once after load_model, then clear dirty
    if (g_vision_model_dirty && !state->model_sent && g_vision_model_name[0]) {
        strncpy(vision->model_name, g_vision_model_name, sizeof(vision->model_name) - 1);
        state->model_sent = true;
    }
}

// ===================== Message body builders =====================

// Fill common VisionMessage header fields.
// hdr packs: ver(4) | sensor_id(4) | alarm_type(8).
// timestamp_utc: 0 = not sent (drone), non-zero = sent (pole).
static inline void vision_fill_header(visionmesh_VisionMessage* vision,
                                      uint32_t sensor_id, uint32_t track_id,
                                      uint32_t alarm_type, uint32_t timestamp_utc,
                                      uint32_t seq) {
    vision->hdr = (VISION_APP_VERSION & 0xF)
                | ((sensor_id & 0xF) << 4)
                | ((alarm_type & 0xFF) << 8);
    vision->track_id = track_id;
    if (timestamp_utc != 0) {
        vision->has_timestamp_utc = true;
        vision->timestamp_utc = timestamp_utc;
    }
    vision->seq = seq;
}

// Fill NewDetection body.
// det_meta packs: conf(8) | class_id(8).
static inline void vision_fill_new_detection(visionmesh_VisionMessage* vision,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t class_id,
    int32_t gx_cm, int32_t gy_cm, int16_t width_cm) {
    vision->which_body = visionmesh_VisionMessage_new_detection_tag;
    visionmesh_NewDetection* det = &vision->body.new_detection;
    det->xywh_packed = ((uint32_t)x) | ((uint32_t)y << 8) |
                       ((uint32_t)w << 16) | ((uint32_t)h << 24);
    det->det_meta = (conf & 0xFF)
                  | ((class_id & 0xFF) << 8);
    det->gx_cm = gx_cm;
    det->gy_cm = gy_cm;
    det->width_cm = (uint32_t)(width_cm > 0 ? width_cm : 0);
}

// Fill UpdateDetection body.
// conf_age packs: conf(8) | age(24).
static inline void vision_fill_update(visionmesh_VisionMessage* vision,
    uint8_t x, uint8_t y, uint8_t w, uint8_t h,
    uint32_t conf, uint32_t age,
    int32_t gx_cm, int32_t gy_cm) {
    vision->which_body = visionmesh_VisionMessage_update_detection_tag;
    visionmesh_UpdateDetection* upd = &vision->body.update_detection;
    upd->xywh_packed = ((uint32_t)x) | ((uint32_t)y << 8) |
                       ((uint32_t)w << 16) | ((uint32_t)h << 24);
    upd->conf_age = (conf & 0xFF) | ((age & 0xFFFFFF) << 8);
    upd->gx_cm = gx_cm;
    upd->gy_cm = gy_cm;
}

// Fill DeleteDetection body.
static inline void vision_fill_delete(visionmesh_VisionMessage* vision,
    uint32_t reason, uint32_t age, uint32_t total_hits,
    int32_t last_gx_cm, int32_t last_gy_cm) {
    vision->which_body = visionmesh_VisionMessage_delete_detection_tag;
    visionmesh_DeleteDetection* del = &vision->body.delete_detection;
    del->reason     = reason;
    del->age        = age;
    del->total_hits = total_hits;
    del->last_gx_cm = last_gx_cm;
    del->last_gy_cm = last_gy_cm;
}

#endif  // SENTAI_VISION_COMMON_H_

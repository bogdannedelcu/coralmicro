// ============== sentai.pipeline — Continuous detection pipeline =============
// This file is #include'd from modsentai.c — do NOT compile separately.

// Extern declarations for detection_task.cc functions
extern int sentai_detection_start(int conf_permil, int iou_permil, int max_dets);
extern int sentai_detection_stop(void);
extern int sentai_detection_get(DetectionFrame* frame, int timeout_ms);
extern int sentai_detection_is_running(void);
extern void sentai_detection_stats(uint32_t* frames_processed,
                                   uint32_t* frames_dropped,
                                   uint32_t* avg_fps_x10);

// sentai.pipeline.start([conf[, iou[, max[, track]]]]) -> int
// Start continuous detection pipeline.
// conf/iou: float 0.0-1.0 (default 0.5 / 0.45).  max: int (default 50).
// track: bool — enable SentAI-SORT tracker (default False).
// Returns 0 on success, negative on error.
static mp_obj_t mod_sentai_pipeline_start(size_t n_args, const mp_obj_t *args) {
    int conf  = (n_args >= 1) ? (int)(mp_obj_get_float(args[0]) * 1000.0f) : 500;
    int iou   = (n_args >= 2) ? (int)(mp_obj_get_float(args[1]) * 1000.0f) : 450;
    int maxd  = (n_args >= 3) ? mp_obj_get_int(args[2]) : 50;
    int track = (n_args >= 4) ? mp_obj_is_true(args[3]) : 0;

    if (track) {
        sentai_tracker_reset();
        sentai_tracker_set_enabled(1);
    }

    int rc = sentai_detection_start(conf, iou, maxd);
    if (rc < 0) {
        sentai_tracker_set_enabled(0);
        mp_raise_msg_varg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("pipeline start failed (%d)"), rc);
    }
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_start_obj,
                                            0, 4, mod_sentai_pipeline_start);

// sentai.pipeline.stop() -> int
static mp_obj_t mod_sentai_pipeline_stop(void) {
    sentai_tracker_set_enabled(0);
    return mp_obj_new_int(sentai_detection_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_stop_obj,
                                  mod_sentai_pipeline_stop);

// sentai.pipeline.get([timeout_ms]) -> list of (x1,y1,x2,y2,conf,class_id) or None
// Blocks up to timeout_ms (default 1000).
// Returns list of detection tuples, or None on timeout.
static mp_obj_t mod_sentai_pipeline_get(size_t n_args, const mp_obj_t *args) {
    int timeout = (n_args >= 1) ? mp_obj_get_int(args[0]) : 1000;

    DetectionFrame frame;
    int rc = sentai_detection_get(&frame, timeout);
    if (rc < 0) {
        return mp_const_none;  // timeout or not running
    }

    // Build list of (x1, y1, x2, y2, conf_float, class_id) tuples
    mp_obj_list_t *list = MP_OBJ_TO_PTR(mp_obj_new_list(frame.count, NULL));
    for (int i = 0; i < frame.count; i++) {
        mp_obj_t items[6] = {
            mp_obj_new_int(frame.dets[i].x1),
            mp_obj_new_int(frame.dets[i].y1),
            mp_obj_new_int(frame.dets[i].x2),
            mp_obj_new_int(frame.dets[i].y2),
            mp_obj_new_float(frame.dets[i].conf_permil / 1000.0f),
            mp_obj_new_int(frame.dets[i].class_id),
        };
        list->items[i] = mp_obj_new_tuple(6, items);
    }
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_get_obj,
                                            0, 1, mod_sentai_pipeline_get);

// sentai.pipeline.running() -> bool
static mp_obj_t mod_sentai_pipeline_running(void) {
    return mp_obj_new_bool(sentai_detection_is_running());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_running_obj,
                                  mod_sentai_pipeline_running);

// sentai.pipeline.stats() -> (frames_processed, frames_dropped, fps_float)
static mp_obj_t mod_sentai_pipeline_stats(void) {
    uint32_t processed = 0, dropped = 0, fps_x10 = 0;
    sentai_detection_stats(&processed, &dropped, &fps_x10);
    mp_obj_t items[3] = {
        mp_obj_new_int(processed),
        mp_obj_new_int(dropped),
        mp_obj_new_float(fps_x10 / 10.0f),
    };
    return mp_obj_new_tuple(3, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_stats_obj,
                                  mod_sentai_pipeline_stats);

// sentai.pipeline.tracks() -> list of tuples
// Returns snapshot of active (confirmed + lost) tracks.
// Each tuple: (id, x1, y1, x2, y2, conf, class_id, state, hits, age,
//              gx_cm, gy_cm, dist_cm, width_cm, lat, lon)
static mp_obj_t mod_sentai_pipeline_tracks(void) {
    TrackedObject buf[TRACKER_MAX_TRACKS];
    int n = sentai_tracker_get_tracks(buf, TRACKER_MAX_TRACKS);
    mp_obj_list_t *list = MP_OBJ_TO_PTR(mp_obj_new_list(n, NULL));
    for (int i = 0; i < n; i++) {
        mp_obj_t items[16] = {
            mp_obj_new_int(buf[i].id),
            mp_obj_new_int(buf[i].x1),
            mp_obj_new_int(buf[i].y1),
            mp_obj_new_int(buf[i].x2),
            mp_obj_new_int(buf[i].y2),
            mp_obj_new_float(buf[i].conf_permil / 1000.0f),
            mp_obj_new_int(buf[i].class_id),
            mp_obj_new_int(buf[i].state),
            mp_obj_new_int(buf[i].hits),
            mp_obj_new_int(buf[i].age_frames),
            mp_obj_new_int(buf[i].gx_cm),
            mp_obj_new_int(buf[i].gy_cm),
            mp_obj_new_int(buf[i].dist_cm),
            mp_obj_new_int(buf[i].width_cm),
            mp_obj_new_float(buf[i].lat),
            mp_obj_new_float(buf[i].lon),
        };
        list->items[i] = mp_obj_new_tuple(16, items);
    }
    return MP_OBJ_FROM_PTR(list);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_tracks_obj,
                                  mod_sentai_pipeline_tracks);

// sentai.pipeline.event([timeout_ms]) -> tuple or None
// Returns next track event: (type, id, x1, y1, x2, y2, conf, class_id, frame_seq,
//                            gx_cm, gy_cm, dist_cm, lat, lon)
// type: 1=new, 2=update, 3=lost, 4=removed
static mp_obj_t mod_sentai_pipeline_event(size_t n_args, const mp_obj_t *args) {
    int timeout = (n_args >= 1) ? mp_obj_get_int(args[0]) : 1000;
    TrackEvent evt;
    if (!sentai_tracker_get_event(&evt, timeout)) {
        return mp_const_none;
    }
    mp_obj_t items[14] = {
        mp_obj_new_int(evt.type),
        mp_obj_new_int(evt.id),
        mp_obj_new_int(evt.x1),
        mp_obj_new_int(evt.y1),
        mp_obj_new_int(evt.x2),
        mp_obj_new_int(evt.y2),
        mp_obj_new_float(evt.conf_permil / 1000.0f),
        mp_obj_new_int(evt.class_id),
        mp_obj_new_int(evt.frame_seq),
        mp_obj_new_int(evt.gx_cm),
        mp_obj_new_int(evt.gy_cm),
        mp_obj_new_int(evt.dist_cm),
        mp_obj_new_float(evt.lat),
        mp_obj_new_float(evt.lon),
    };
    return mp_obj_new_tuple(14, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_event_obj,
                                            0, 1, mod_sentai_pipeline_event);

// sentai.pipeline.track_config([min_hits[, max_misses[, iou[, hist[, high[, low]]]]]]) -> tuple
// Get/set tracker configuration. Returns (min_hits, max_misses, iou, hist_weight, high, low).
static mp_obj_t mod_sentai_pipeline_track_config(size_t n_args, const mp_obj_t *args) {
    TrackerConfig cfg;
    sentai_tracker_get_config(&cfg);
    if (n_args >= 1) cfg.min_hits    = (uint8_t)mp_obj_get_int(args[0]);
    if (n_args >= 2) cfg.max_misses  = (uint8_t)mp_obj_get_int(args[1]);
    if (n_args >= 3) cfg.iou_thresh  = (int16_t)(mp_obj_get_float(args[2]) * 1000.0f);
    if (n_args >= 4) cfg.hist_weight = (int16_t)(mp_obj_get_float(args[3]) * 1000.0f);
    if (n_args >= 5) cfg.high_thresh = (int16_t)(mp_obj_get_float(args[4]) * 1000.0f);
    if (n_args >= 6) cfg.low_thresh  = (int16_t)(mp_obj_get_float(args[5]) * 1000.0f);
    if (n_args >= 1) sentai_tracker_set_config(&cfg);
    mp_obj_t items[6] = {
        mp_obj_new_int(cfg.min_hits),
        mp_obj_new_int(cfg.max_misses),
        mp_obj_new_float(cfg.iou_thresh / 1000.0f),
        mp_obj_new_float(cfg.hist_weight / 1000.0f),
        mp_obj_new_float(cfg.high_thresh / 1000.0f),
        mp_obj_new_float(cfg.low_thresh / 1000.0f),
    };
    return mp_obj_new_tuple(6, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_track_config_obj,
                                            0, 6, mod_sentai_pipeline_track_config);

// sentai.pipeline.set_pose(altitude_cm[, heading_deg[, lat, lon]]) -> None
// Set sensor pose for ground-plane projection.
//   altitude_cm: camera height above ground in cm (0 = disable)
//   heading_deg: compass heading 0-359, -1 = no compass (default -1)
//   lat, lon: camera GPS in degrees (default 0.0 = no GPS)
static mp_obj_t mod_sentai_pipeline_set_pose(size_t n_args, const mp_obj_t *args) {
    int alt     = mp_obj_get_int(args[0]);
    int heading = (n_args >= 2) ? mp_obj_get_int(args[1]) : -1;
    float lat   = (n_args >= 3) ? mp_obj_get_float(args[2]) : 0.0f;
    float lon   = (n_args >= 4) ? mp_obj_get_float(args[3]) : 0.0f;
    sentai_tracker_set_pose(alt, heading, lat, lon);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_set_pose_obj,
                                            1, 4, mod_sentai_pipeline_set_pose);

// sentai.pipeline.camera_config(cam_id[, fov_h, fov_v, mount_pitch, mount_roll, mount_yaw]) -> tuple
// Get/set per-camera geometry. cam_id: 0 or 1.
// fov_h/fov_v: degrees. mount_pitch: 0=down, 25=tilted 25° from vertical.
// mount_roll: 0=landscape, 90=portrait. mount_yaw: 0=forward, 180=backward.
// With only cam_id: returns current config. With extra args: sets and returns.
// Returns (fov_h, fov_v, mount_pitch, mount_roll, mount_yaw).
static mp_obj_t mod_sentai_pipeline_camera_config(size_t n_args, const mp_obj_t *args) {
    int cam_id = mp_obj_get_int(args[0]);
    if (cam_id < 0 || cam_id >= TRACKER_MAX_CAMERAS) {
        mp_raise_ValueError(MP_ERROR_TEXT("cam_id must be 0 or 1"));
    }
    CameraConfig cfg;
    sentai_tracker_get_camera(cam_id, &cfg);
    if (n_args >= 2) cfg.fov_h_deg       = mp_obj_get_float(args[1]);
    if (n_args >= 3) cfg.fov_v_deg       = mp_obj_get_float(args[2]);
    if (n_args >= 4) cfg.mount_pitch_deg = mp_obj_get_float(args[3]);
    if (n_args >= 5) cfg.mount_roll_deg  = mp_obj_get_float(args[4]);
    if (n_args >= 6) cfg.mount_yaw_deg   = mp_obj_get_float(args[5]);
    if (n_args >= 2) sentai_tracker_set_camera(cam_id, &cfg);
    mp_obj_t items[5] = {
        mp_obj_new_float(cfg.fov_h_deg),
        mp_obj_new_float(cfg.fov_v_deg),
        mp_obj_new_float(cfg.mount_pitch_deg),
        mp_obj_new_float(cfg.mount_roll_deg),
        mp_obj_new_float(cfg.mount_yaw_deg),
    };
    return mp_obj_new_tuple(5, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_camera_config_obj,
                                            1, 6, mod_sentai_pipeline_camera_config);

// ---- module table ----
static const mp_rom_map_elem_t sentai_pipeline_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR_pipeline) },
    { MP_ROM_QSTR(MP_QSTR_start),         MP_ROM_PTR(&mod_sentai_pipeline_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),          MP_ROM_PTR(&mod_sentai_pipeline_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_get),           MP_ROM_PTR(&mod_sentai_pipeline_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_running),       MP_ROM_PTR(&mod_sentai_pipeline_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),         MP_ROM_PTR(&mod_sentai_pipeline_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracks),        MP_ROM_PTR(&mod_sentai_pipeline_tracks_obj) },
    { MP_ROM_QSTR(MP_QSTR_event),         MP_ROM_PTR(&mod_sentai_pipeline_event_obj) },
    { MP_ROM_QSTR(MP_QSTR_track_config),  MP_ROM_PTR(&mod_sentai_pipeline_track_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pose),       MP_ROM_PTR(&mod_sentai_pipeline_set_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera_config),  MP_ROM_PTR(&mod_sentai_pipeline_camera_config_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_pipeline_globals, sentai_pipeline_globals_table);
static const mp_obj_module_t sentai_pipeline_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_pipeline_globals,
};

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
extern void sentai_detection_task_stall_ms(uint32_t* prep_stall_ms,
                                           uint32_t* infer_stall_ms);

// sentai.pipeline.dma_memcpy([flag]) -> int (previous value)
// Toggle the eDMA-accelerated staging->tensor memcpy in InferTask on/off at
// runtime.  No args: return current state.  1: use eDMA (fast, ~14 ms on a
// 786 KB copy).  0: use plain CPU memcpy (~24 ms).  Intended for A/B
// benchmarking in a single firmware image — documented in paper/memcpy.md.
extern int  sentai_dma_memcpy_get(void);
extern void sentai_dma_memcpy_set(int v);
static mp_obj_t mod_sentai_pipeline_dma_memcpy(size_t n_args,
                                               const mp_obj_t *args) {
    int prev = sentai_dma_memcpy_get();
    if (n_args >= 1) {
        sentai_dma_memcpy_set(mp_obj_is_true(args[0]) ? 1 : 0);
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_dma_memcpy_obj,
                                            0, 1, mod_sentai_pipeline_dma_memcpy);

// sentai.pipeline.direct_tensor([flag]) -> int (previous value)
// Step 4 ping-pong path: PrepTask PXP writes directly into one of two
// aligned tensor-sized buffers; InferTask swaps the TFLite input tensor's
// data pointer to that buffer before each Invoke, eliminating the 16-17 ms
// staging->tensor memcpy.  Cannot be toggled while the pipeline is running
// (setter returns -1 -> ValueError here, per embeded.md §C).
extern int  sentai_pipeline_direct_tensor_get(void);
extern int  sentai_pipeline_direct_tensor_set(int v);
extern void sentai_pipeline_direct_tensor_stats(uint32_t* frames,
                                                uint32_t* prep_to,
                                                uint32_t* infer_to,
                                                uint32_t* swap_fail);
static mp_obj_t mod_sentai_pipeline_direct_tensor(size_t n_args,
                                                   const mp_obj_t *args) {
    int prev = sentai_pipeline_direct_tensor_get();
    if (n_args >= 1) {
        int v = mp_obj_is_true(args[0]) ? 1 : 0;
        if (sentai_pipeline_direct_tensor_set(v) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("stop pipeline before toggling direct_tensor"));
        }
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_direct_tensor_obj,
                                            0, 1, mod_sentai_pipeline_direct_tensor);

// sentai.pipeline.direct_stats() -> dict of supervision counters for the
// direct-tensor path.  Zeros when feature has never been enabled since boot.
static mp_obj_t mod_sentai_pipeline_direct_stats(void) {
    uint32_t f=0, pto=0, ito=0, sf=0;
    sentai_pipeline_direct_tensor_stats(&f, &pto, &ito, &sf);
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames),
                      mp_obj_new_int_from_uint(f));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_prep_buf_timeout),
                      mp_obj_new_int_from_uint(pto));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_infer_wait_timeout),
                      mp_obj_new_int_from_uint(ito));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_swap_fail),
                      mp_obj_new_int_from_uint(sf));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_direct_stats_obj,
                                  mod_sentai_pipeline_direct_stats);

// sentai.pipeline.desc_cache([flag]) -> int (previous value)
// Enable host-side caching of parameters + instructions in the EdgeTPU
// descriptor hint loop (edgetpu_executable.cc).  Matches a model's
// parameter_caching_token so a change of model or a reload automatically
// invalidates the cache (sentai_tpu_desc_cache_invalidate in
// sentai_load_model).  Safe to toggle at any time — first Invoke after
// enabling does a full upload and primes the cache, subsequent ones
// skip the 30+ MB of static wire traffic per frame.
extern volatile int      g_sentai_tpu_desc_cache_enabled;
extern volatile uint32_t g_sentai_tpu_desc_cache_sent_params;
extern volatile uint32_t g_sentai_tpu_desc_cache_sent_ins;
extern volatile uint32_t g_sentai_tpu_desc_cache_skip_params;
extern volatile uint32_t g_sentai_tpu_desc_cache_skip_ins;
extern void sentai_tpu_desc_cache_invalidate(void);
static mp_obj_t mod_sentai_pipeline_desc_cache(size_t n_args,
                                                const mp_obj_t *args) {
    int prev = g_sentai_tpu_desc_cache_enabled;
    if (n_args >= 1) {
        g_sentai_tpu_desc_cache_enabled = mp_obj_is_true(args[0]) ? 1 : 0;
        // Always invalidate on state change so a flip-flop test starts
        // from a clean cache and the first Invoke in the new mode does a
        // predictable full upload.
        sentai_tpu_desc_cache_invalidate();
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_desc_cache_obj,
                                            0, 1, mod_sentai_pipeline_desc_cache);

// sentai.pipeline.desc_cache_stats() -> dict with sent/skip counters.
static mp_obj_t mod_sentai_pipeline_desc_cache_stats(void) {
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sent_params),
                      mp_obj_new_int_from_uint(g_sentai_tpu_desc_cache_sent_params));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sent_ins),
                      mp_obj_new_int_from_uint(g_sentai_tpu_desc_cache_sent_ins));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_skip_params),
                      mp_obj_new_int_from_uint(g_sentai_tpu_desc_cache_skip_params));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_skip_ins),
                      mp_obj_new_int_from_uint(g_sentai_tpu_desc_cache_skip_ins));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_desc_cache_stats_obj,
                                  mod_sentai_pipeline_desc_cache_stats);

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

// sentai.pipeline.get_ex([timeout_ms]) -> (dets, inference_ms, total_ms, frame_seq) or None
// Same as get() but also returns the firmware-measured InferTask timing:
//   inference_ms  — time spent inside tpu.invoke()
//   total_ms      — wall time of the full InferTask loop iteration
//                   (memcpy staging->tensor + invoke + NMS + queue send)
//   frame_seq     — monotonic camera frame sequence number of the captured frame
// Use this to diagnose where time is spent between Python's pipeline.get() calls:
//   if total_ms ≈ frame_interval → InferTask is the bottleneck
//   if total_ms << frame_interval → Python/IPC overhead dominates
static mp_obj_t mod_sentai_pipeline_get_ex(size_t n_args, const mp_obj_t *args) {
    int timeout = (n_args >= 1) ? mp_obj_get_int(args[0]) : 1000;
    DetectionFrame frame;
    int rc = sentai_detection_get(&frame, timeout);
    if (rc < 0) return mp_const_none;

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

    // Tuple: (dets, invoke_ms, total_ms, frame_seq, memcpy_ms, nms_ms)
    // Extra fields expose the InferTask sub-stage timings so experiments can
    // distinguish TPU bus time, SDRAM memcpy time and NMS time individually.
    mp_obj_t tup[6] = {
        MP_OBJ_FROM_PTR(list),
        mp_obj_new_int_from_uint(frame.inference_ms),
        mp_obj_new_int_from_uint(frame.total_ms),
        mp_obj_new_int_from_uint(frame.frame_seq),
        mp_obj_new_int_from_uint(frame.memcpy_ms),
        mp_obj_new_int_from_uint(frame.nms_ms),
    };
    return mp_obj_new_tuple(6, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_get_ex_obj,
                                            0, 1, mod_sentai_pipeline_get_ex);

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

// sentai.pipeline.camera_config(cam_id[, fov_h, fov_v, mount_pitch, mount_roll, mount_yaw, ground_ref]) -> tuple
// Get/set per-camera geometry. cam_id: 0 or 1.
// fov_h/fov_v: degrees. mount_pitch: 0=down, 25=tilted 25° from vertical.
// mount_roll: 0=landscape, 90=portrait. mount_yaw: 0=forward, 180=backward.
// ground_ref: 0=centroid (drone/overhead), 1=bottom-center (pole mount).
// With only cam_id: returns current config. With extra args: sets and returns.
// Returns (fov_h, fov_v, mount_pitch, mount_roll, mount_yaw, ground_ref).
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
    if (n_args >= 7) cfg.ground_ref      = mp_obj_get_int(args[6]);
    if (n_args >= 2) sentai_tracker_set_camera(cam_id, &cfg);
    mp_obj_t items[6] = {
        mp_obj_new_float(cfg.fov_h_deg),
        mp_obj_new_float(cfg.fov_v_deg),
        mp_obj_new_float(cfg.mount_pitch_deg),
        mp_obj_new_float(cfg.mount_roll_deg),
        mp_obj_new_float(cfg.mount_yaw_deg),
        mp_obj_new_int(cfg.ground_ref),
    };
    return mp_obj_new_tuple(6, items);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_camera_config_obj,
                                            1, 7, mod_sentai_pipeline_camera_config);

// sentai.pipeline.task_health() -> (prep_stall_ms, infer_stall_ms)
// Milliseconds since PrepTask / InferTask last completed a frame.
// A large value (>5000) while running indicates a stuck pipeline stage.
// Returns (0xFFFFFFFF, 0xFFFFFFFF) when pipeline is not running.
static mp_obj_t mod_sentai_pipeline_task_health(void) {
    uint32_t prep = 0, infer = 0;
    sentai_detection_task_stall_ms(&prep, &infer);
    mp_obj_t items[2] = {
        mp_obj_new_int_from_uint(prep),
        mp_obj_new_int_from_uint(infer),
    };
    return mp_obj_new_tuple(2, items);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_task_health_obj,
                                  mod_sentai_pipeline_task_health);

// ---- module table ----
static const mp_rom_map_elem_t sentai_pipeline_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR_pipeline) },
    { MP_ROM_QSTR(MP_QSTR_start),         MP_ROM_PTR(&mod_sentai_pipeline_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),          MP_ROM_PTR(&mod_sentai_pipeline_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_get),           MP_ROM_PTR(&mod_sentai_pipeline_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ex),        MP_ROM_PTR(&mod_sentai_pipeline_get_ex_obj) },
    { MP_ROM_QSTR(MP_QSTR_dma_memcpy),    MP_ROM_PTR(&mod_sentai_pipeline_dma_memcpy_obj) },
    { MP_ROM_QSTR(MP_QSTR_direct_tensor), MP_ROM_PTR(&mod_sentai_pipeline_direct_tensor_obj) },
    { MP_ROM_QSTR(MP_QSTR_direct_stats),  MP_ROM_PTR(&mod_sentai_pipeline_direct_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_desc_cache),    MP_ROM_PTR(&mod_sentai_pipeline_desc_cache_obj) },
    { MP_ROM_QSTR(MP_QSTR_desc_cache_stats), MP_ROM_PTR(&mod_sentai_pipeline_desc_cache_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_running),       MP_ROM_PTR(&mod_sentai_pipeline_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),         MP_ROM_PTR(&mod_sentai_pipeline_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracks),        MP_ROM_PTR(&mod_sentai_pipeline_tracks_obj) },
    { MP_ROM_QSTR(MP_QSTR_event),         MP_ROM_PTR(&mod_sentai_pipeline_event_obj) },
    { MP_ROM_QSTR(MP_QSTR_track_config),  MP_ROM_PTR(&mod_sentai_pipeline_track_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pose),       MP_ROM_PTR(&mod_sentai_pipeline_set_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera_config),  MP_ROM_PTR(&mod_sentai_pipeline_camera_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_health),    MP_ROM_PTR(&mod_sentai_pipeline_task_health_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_pipeline_globals, sentai_pipeline_globals_table);
static const mp_obj_module_t sentai_pipeline_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_pipeline_globals,
};

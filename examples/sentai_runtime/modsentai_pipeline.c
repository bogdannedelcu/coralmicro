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

// sentai.pipeline.target_fps([n]) -> int
// Caps the InferTask loop at `n` frames/sec.  0 disables (run flat
// out, ~75 FPS on our YOLO 512).  Default 45 to match camera and
// avoid sustained peak-current draw that may be destabilising the
// TPU silicon.  Clamp [0, 120].
extern int  sentai_pipeline_target_fps_get(void);
extern void sentai_pipeline_target_fps_set(int v);
static mp_obj_t mod_sentai_pipeline_target_fps(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 1) sentai_pipeline_target_fps_set(mp_obj_get_int(args[0]));
    return mp_obj_new_int(sentai_pipeline_target_fps_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_target_fps_obj,
                                            0, 1, mod_sentai_pipeline_target_fps);

// sentai.pipeline.prep_fps([n]) -> int
// Caps PrepTask (cam_grab + PXP + quant) at `n` frames/sec.  0 = free
// run.  Convention (2026-04-22): pair with target_fps at 1:2 ratio
// (e.g. prep=30, tpu=60) so the tasks don't over-subscribe SDRAM bus
// bandwidth simultaneously.  Clamp [0, 120].
extern int  sentai_pipeline_prep_fps_get(void);
extern void sentai_pipeline_prep_fps_set(int v);
static mp_obj_t mod_sentai_pipeline_prep_fps(size_t n_args, const mp_obj_t *args) {
    if (n_args >= 1) sentai_pipeline_prep_fps_set(mp_obj_get_int(args[0]));
    return mp_obj_new_int(sentai_pipeline_prep_fps_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_prep_fps_obj,
                                            0, 1, mod_sentai_pipeline_prep_fps);

// desc_cache / multi_ep_routing Python bindings removed 2026-04-21.
// The underlying C infrastructure (g_sentai_tpu_desc_cache_enabled,
// g_sentai_tpu_multi_ep_routing, and the hot-path branches in
// edgetpu_driver.cc / edgetpu_executable.cc) remains in place but is
// permanently OFF — exposing it via MicroPython created a foot-gun
// where a user could enable un-validated optimisations on live TPU
// traffic.  Either fully validate and enable the path by default, or
// rip out the C branches too; do NOT re-expose the toggle as-is.
// TODO(tpu-dead-code-cleanup): remove g_sentai_tpu_{desc_cache,multi_ep_*}
// variables and their hot-path branches after the validation test plan
// for EP-routing is in place.  Tracked in the next sprint.

// sentai.pipeline.prep_stats() -> dict with per-stage averages of PrepTask.
// Each field is the CUMULATIVE ms spent in that stage divided by the
// number of completed PrepTask iterations.  Useful for root-causing where
// the pipeline spends its ~58 ms per frame (cam grab? PXP? semaphore
// wait?).  Reset via sentai.pipeline.prep_reset().
extern void sentai_prep_stage_stats(uint32_t* frames, uint32_t* sem_wait,
                                    uint32_t* cam_grab, uint32_t* pxp,
                                    uint32_t* quant, uint32_t* total);
extern void sentai_prep_stage_reset(void);
static mp_obj_t mod_sentai_pipeline_prep_stats(void) {
    uint32_t frames=0, sw=0, cg=0, pxp=0, qt=0, tot=0;
    sentai_prep_stage_stats(&frames, &sw, &cg, &pxp, &qt, &tot);
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames),
                      mp_obj_new_int_from_uint(frames));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sem_wait_ms_sum),
                      mp_obj_new_int_from_uint(sw));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cam_grab_ms_sum),
                      mp_obj_new_int_from_uint(cg));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_pxp_ms_sum),
                      mp_obj_new_int_from_uint(pxp));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_quant_ms_sum),
                      mp_obj_new_int_from_uint(qt));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_total_ms_sum),
                      mp_obj_new_int_from_uint(tot));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_prep_stats_obj,
                                  mod_sentai_pipeline_prep_stats);

static mp_obj_t mod_sentai_pipeline_prep_reset(void) {
    sentai_prep_stage_reset();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_prep_reset_obj,
                                  mod_sentai_pipeline_prep_reset);

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

    // Tuple: (dets, invoke_ms, total_ms, frame_seq, memcpy_ms, nms_ms, cam_id)
    // cam_id (added 2026-04-25) is the source camera tag from per-buffer
    // ISR tagging — 0/1 for cam0/cam1, or -1 if unknown.
    mp_obj_t tup[7] = {
        MP_OBJ_FROM_PTR(list),
        mp_obj_new_int_from_uint(frame.inference_ms),
        mp_obj_new_int_from_uint(frame.total_ms),
        mp_obj_new_int_from_uint(frame.frame_seq),
        mp_obj_new_int_from_uint(frame.memcpy_ms),
        mp_obj_new_int_from_uint(frame.nms_ms),
        mp_obj_new_int(frame.cam_id),
    };
    return mp_obj_new_tuple(7, tup);
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

// ---- Incremental-isolation diagnostics (2026-04-22) ----
// Sets PrepTask stage: 0=full, 1=mock (no HW), 2=cam only, 3=cam+PXP,
// 4=full.  Use to find which piece of PrepTask wedges the TPU.
extern int  sentai_pipeline_debug_prep_mode_get(void);
extern void sentai_pipeline_debug_prep_mode_set(int v);
static mp_obj_t mod_sentai_pipeline_debug_prep_mode(size_t n_args,
                                                    const mp_obj_t *args) {
    if (n_args >= 1) {
        sentai_pipeline_debug_prep_mode_set(mp_obj_get_int(args[0]));
    }
    return mp_obj_new_int(sentai_pipeline_debug_prep_mode_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_sentai_pipeline_debug_prep_mode_obj, 0, 1,
    mod_sentai_pipeline_debug_prep_mode);

// Toggles whether InferTask calls tpu_invoke at all.  When 1, InferTask
// does memcpy + gives sem_free but skips the TPU transfer, letting us
// test whether PrepTask activity alone wedges the TPU silicon.
extern int  sentai_pipeline_debug_no_invoke_get(void);
extern void sentai_pipeline_debug_no_invoke_set(int v);
static mp_obj_t mod_sentai_pipeline_debug_no_invoke(size_t n_args,
                                                    const mp_obj_t *args) {
    if (n_args >= 1) {
        sentai_pipeline_debug_no_invoke_set(mp_obj_is_true(args[0]));
    }
    return mp_obj_new_int(sentai_pipeline_debug_no_invoke_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_sentai_pipeline_debug_no_invoke_obj, 0, 1,
    mod_sentai_pipeline_debug_no_invoke);

// Returns per-InferTask-iteration counters: ok, fail, ms_sum, last_rc.
// Observable live during a sustained pipeline run so we can see WHEN
// InferTask starts failing (vs having to wait for pipeline.stop).
extern void sentai_pipeline_infer_stats(uint32_t* ok, uint32_t* fail,
                                        uint32_t* ms_sum,
                                        int32_t*  last_rc);
extern void sentai_pipeline_infer_reset(void);
static mp_obj_t mod_sentai_pipeline_infer_stats(void) {
    uint32_t ok=0, fail=0, ms=0;
    int32_t  last_rc=0;
    sentai_pipeline_infer_stats(&ok, &fail, &ms, &last_rc);
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, mp_obj_new_str("ok", 2),
                      mp_obj_new_int_from_uint(ok));
    mp_obj_dict_store(d, mp_obj_new_str("fail", 4),
                      mp_obj_new_int_from_uint(fail));
    mp_obj_dict_store(d, mp_obj_new_str("ms_sum", 6),
                      mp_obj_new_int_from_uint(ms));
    mp_obj_dict_store(d, mp_obj_new_str("last_rc", 7),
                      mp_obj_new_int(last_rc));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_infer_stats_obj,
                                  mod_sentai_pipeline_infer_stats);
static mp_obj_t mod_sentai_pipeline_infer_reset(void) {
    sentai_pipeline_infer_reset();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_infer_reset_obj,
                                  mod_sentai_pipeline_infer_reset);

// sentai.pipeline.invokes_per_frame([n]) — simulate multi-patch
// workloads.  Default 1.  Each PrepTask frame triggers n TPU invokes
// (same input buffer) so you can measure whether the MCU sustains
// "K patches per camera frame" at a given camera FPS.
extern int  sentai_pipeline_invokes_per_frame_get(void);
extern void sentai_pipeline_invokes_per_frame_set(int v);
static mp_obj_t mod_sentai_pipeline_invokes_per_frame(size_t n_args,
                                                     const mp_obj_t *args) {
    if (n_args >= 1) sentai_pipeline_invokes_per_frame_set(mp_obj_get_int(args[0]));
    return mp_obj_new_int(sentai_pipeline_invokes_per_frame_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_sentai_pipeline_invokes_per_frame_obj, 0, 1,
    mod_sentai_pipeline_invokes_per_frame);

// sentai.pipeline.multi_invoke_mode([n]) — sync strategy when
// invokes_per_frame > 1.  0 = LEGACY (race), 1 = DEFER (safe),
// 2 = REARM (race-window).  Has no effect for invokes_per_frame == 1.
extern int  sentai_pipeline_multi_invoke_mode_get(void);
extern void sentai_pipeline_multi_invoke_mode_set(int v);
static mp_obj_t mod_sentai_pipeline_multi_invoke_mode(size_t n_args,
                                                     const mp_obj_t *args) {
    if (n_args >= 1) sentai_pipeline_multi_invoke_mode_set(mp_obj_get_int(args[0]));
    return mp_obj_new_int(sentai_pipeline_multi_invoke_mode_get());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    mod_sentai_pipeline_multi_invoke_mode_obj, 0, 1,
    mod_sentai_pipeline_multi_invoke_mode);

// sentai.pipeline.force_parity([flag]) -> int (previous value)
// When 1, PrepTask discards a freshly-grabbed buffer if its per-buffer
// cam_id tag matches the LAST accepted frame, retrying up to a bounded
// number of times so the InferTask sees strict cam0/cam1 alternation.
// On retry exhaustion the buffer is accepted anyway (counter bumped) so
// the pipeline keeps making progress.
extern int  sentai_pipeline_force_parity_get(void);
extern void sentai_pipeline_force_parity_set(int v);
static mp_obj_t mod_sentai_pipeline_force_parity(size_t n_args,
                                                 const mp_obj_t *args) {
    int prev = sentai_pipeline_force_parity_get();
    if (n_args >= 1) {
        sentai_pipeline_force_parity_set(mp_obj_is_true(args[0]) ? 1 : 0);
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_force_parity_obj,
                                            0, 1, mod_sentai_pipeline_force_parity);

// sentai.pipeline.force_parity_stats() -> dict {skipped, timeout}
//   skipped : total buffers discarded because cam_id matched previous
//   timeout : retries that ran out -> accepted-anyway frames
extern void sentai_pipeline_force_parity_stats(uint32_t* skipped, uint32_t* timeout);
extern void sentai_pipeline_force_parity_reset(void);
static mp_obj_t mod_sentai_pipeline_force_parity_stats(void) {
    uint32_t sk = 0, to = 0;
    sentai_pipeline_force_parity_stats(&sk, &to);
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_skipped),
                      mp_obj_new_int_from_uint(sk));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_timeout),
                      mp_obj_new_int_from_uint(to));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_force_parity_stats_obj,
                                  mod_sentai_pipeline_force_parity_stats);

static mp_obj_t mod_sentai_pipeline_force_parity_reset(void) {
    sentai_pipeline_force_parity_reset();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_pipeline_force_parity_reset_obj,
                                  mod_sentai_pipeline_force_parity_reset);


// sentai.pipeline.calibrate(model=None, frames=100, timeout_ms=2000,
//                           delay_ms=-1, conf=0.25, iou=0.45) -> dict
//
// One-shot "how does this model behave on the current camera config":
//   - If `model` (str path) is given AND no model is currently loaded,
//     calibrate loads it via sentai.tpu.load(model).  If a model is
//     already loaded, the caller is trusted (no reload).
//   - If the pipeline is not running, calibrate starts it with the
//     given conf/iou/max=50 and STOPS it before returning.  If the
//     pipeline was already running, calibrate leaves it running.
//   - delay_ms (>=0) installs a per-iteration sleep in PrepTask for
//     the duration of the run; on exit the previous loop_delay is
//     restored.  Use to sweep the camera-alternation sweet-spot
//     (1..30 ms at VGA45 alt 1:1 is the typical exploration range).
//     Pass -1 (default) to leave loop_delay untouched.
//
// Caller is responsible for camera setup (init + ratio + switch_drain
// + select) before calling.  Returned dict reports cam0/cam1/unknown
// counts and timing statistics.
//
// Returned dict (all ints unless noted):
//   frames           : frames actually received (frames - timeouts)
//   timeouts         : get_ex calls that returned None
//   cam0, cam1       : per-camera frame counts (from per-buffer ISR tag)
//   unknown          : frames with cam_id < 0 (no tag)
//   invoke_ms_sum    : sum of TPU invoke times
//   invoke_ms_min    : 0 if no frames
//   invoke_ms_max    : 0 if no frames
//   total_ms_sum     : sum of full InferTask iteration time
//   total_ms_min     : ditto
//   total_ms_max     : ditto
//   wall_ms          : wall clock from first frame to last frame
//   fps_x100         : (frames * 100000) / wall_ms  -- two decimals
//   detections_total : sum of detection counts across all frames
//   parity_skipped   : sentai.pipeline.force_parity_stats().skipped delta
//   parity_timeout   : ditto for timeout
//
// Reads force_parity stats deltas around the loop so the same pipeline
// can be calibrated repeatedly without manual reset.
extern uint32_t sentai_ticks_ms(void);
extern void     sentai_sleep_ms(uint32_t ms);
extern int      sentai_pipeline_loop_delay_get(void);
extern void     sentai_pipeline_loop_delay_set(int v);
extern int      sentai_load_model(const char* path);
extern int      sentai_cam_is_initialized(void);
extern int      sentai_cam_test_pattern(int cam_id, int mode);
static mp_obj_t mod_sentai_pipeline_calibrate(size_t n_args,
                                              const mp_obj_t *args) {
    const char* model_path = NULL;
    if (n_args >= 1 && args[0] != mp_const_none) {
        model_path = mp_obj_str_get_str(args[0]);
    }
    int n_frames    = (n_args >= 2) ? mp_obj_get_int(args[1]) : 100;
    int timeout_ms  = (n_args >= 3) ? mp_obj_get_int(args[2]) : 2000;
    int delay_ms    = (n_args >= 4) ? mp_obj_get_int(args[3]) : -1;
    int conf_permil = (n_args >= 5) ? (int)(mp_obj_get_float(args[4]) * 1000.0f)
                                    : 250;
    int iou_permil  = (n_args >= 6) ? (int)(mp_obj_get_float(args[5]) * 1000.0f)
                                    : 450;
    int synthetic   = (n_args >= 7) ? mp_obj_is_true(args[6]) : 0;
    if (n_frames < 1)    n_frames   = 1;
    if (n_frames > 5000) n_frames   = 5000;
    if (timeout_ms < 50) timeout_ms = 50;
    if (delay_ms > 200)  delay_ms   = 200;

    // Auto-load model if asked AND none currently loaded.
    if (model_path != NULL && !sentai_tpu_is_ready()) {
        int rc = sentai_load_model(model_path);
        if (rc != 0) {
            mp_raise_msg_varg(&mp_type_RuntimeError,
                MP_ERROR_TEXT("calibrate: load model failed (%d)"), rc);
        }
    }
    if (!sentai_tpu_is_ready()) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("calibrate: no model loaded"));
    }
    if (!sentai_cam_is_initialized()) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("calibrate: camera not initialized"));
    }

    // Auto-start pipeline if not running.  Track whether WE started it
    // so we can stop on the way out (caller-started pipelines are
    // left running, matching their lifecycle expectation).
    int started_here = 0;
    if (!sentai_detection_is_running()) {
        int rc = sentai_detection_start(conf_permil, iou_permil, 50);
        if (rc != 0) {
            mp_raise_msg_varg(&mp_type_RuntimeError,
                MP_ERROR_TEXT("calibrate: pipeline start failed (%d)"), rc);
        }
        started_here = 1;
        // Brief settle so the first PrepTask iteration completes
        // before we start consuming.
        sentai_sleep_ms(80);
    }

    // Apply requested loop delay (save & restore around the run).
    int prev_loop_delay = sentai_pipeline_loop_delay_get();
    int restore_loop_delay = 0;
    if (delay_ms >= 0) {
        sentai_pipeline_loop_delay_set(delay_ms);
        restore_loop_delay = 1;
    }

    // Synthetic test pattern: cam0 → BLACK, cam1 → WHITE.  Used by
    // calibrate to validate per-buffer cam_id tagging end-to-end.
    // Pixel content of each captured frame becomes deterministic
    // (near-zero or near-saturated) regardless of physical scene,
    // so the consumer can compare cam_id tag vs. expected pixel
    // value to detect mis-attribution unambiguously.  Restored at
    // exit even if calibrate raises mid-loop.
    if (synthetic) {
        sentai_cam_test_pattern(0, 1);  // cam0 BLACK
        sentai_cam_test_pattern(1, 2);  // cam1 WHITE
        sentai_sleep_ms(150);           // let AEC manual settle
    }

    // Drain any stale frames that landed in the queue BEFORE the user
    // settled the camera state (select / ratio / switch_drain).  Without
    // this, the cam0 count gets polluted by frames captured during
    // pipeline warm-up, which biases the histogram.
    {
        DetectionFrame discard;
        while (sentai_detection_get(&discard, 0) >= 0) { /* drop */ }
    }

    uint32_t pf_sk0 = 0, pf_to0 = 0;
    sentai_pipeline_force_parity_stats(&pf_sk0, &pf_to0);

    uint32_t cam0 = 0, cam1 = 0, unk = 0, timeouts = 0;
    uint32_t inv_sum = 0, inv_min = 0xFFFFFFFFUL, inv_max = 0;
    uint32_t tot_sum = 0, tot_min = 0xFFFFFFFFUL, tot_max = 0;
    uint32_t det_total = 0;
    uint32_t t_first = 0, t_last = 0;
    uint32_t got = 0;

    // Bounded outer deadline (embeded.md §B): even if every get() takes
    // exactly timeout_ms (worst case = wedged pipeline before our auto-
    // started detect-task notices), we exit before the WDOG dead-
    // threshold (120 s).  +5 s slack covers per-iteration housekeeping.
    const uint32_t t_call_start = sentai_ticks_ms();
    const uint32_t kCalibrateMaxWallMs = 110000U;
    DetectionFrame frame;
    for (int i = 0; i < n_frames; i++) {
        if ((sentai_ticks_ms() - t_call_start) > kCalibrateMaxWallMs) {
            timeouts += (uint32_t)(n_frames - i);
            break;
        }
        int rc = sentai_detection_get(&frame, timeout_ms);
        if (rc < 0) { timeouts++; continue; }
        if (got == 0) t_first = sentai_ticks_ms();
        t_last = sentai_ticks_ms();
        got++;
        if      (frame.cam_id == 0) cam0++;
        else if (frame.cam_id == 1) cam1++;
        else                         unk++;
        det_total += (uint32_t)frame.count;
        inv_sum   += frame.inference_ms;
        if (frame.inference_ms < inv_min) inv_min = frame.inference_ms;
        if (frame.inference_ms > inv_max) inv_max = frame.inference_ms;
        tot_sum   += frame.total_ms;
        if (frame.total_ms < tot_min) tot_min = frame.total_ms;
        if (frame.total_ms > tot_max) tot_max = frame.total_ms;
    }

    if (got == 0) { inv_min = 0; tot_min = 0; }

    uint32_t pf_sk1 = 0, pf_to1 = 0;
    sentai_pipeline_force_parity_stats(&pf_sk1, &pf_to1);

    // Restore loop_delay before any blocking, in case the user used
    // calibrate to *probe* a delay and the delay was hurtful enough
    // that they want it gone immediately on return.
    if (restore_loop_delay) {
        sentai_pipeline_loop_delay_set(prev_loop_delay);
    }

    // Restore real-scene capture (clear OV5640 test pattern reg).
    if (synthetic) {
        sentai_cam_test_pattern(0, 0);
        sentai_cam_test_pattern(1, 0);
    }

    // If we started the pipeline ourselves, stop it on the way out.
    if (started_here) {
        sentai_detection_stop();
    }

    uint32_t wall_ms = (got > 1) ? (t_last - t_first) : 0;
    uint32_t fps_x100 = (wall_ms > 0)
        ? (uint32_t)((uint64_t)got * 100000ULL / wall_ms)
        : 0;

    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames),       mp_obj_new_int_from_uint(got));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_timeouts),     mp_obj_new_int_from_uint(timeouts));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cam0),         mp_obj_new_int_from_uint(cam0));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cam1),         mp_obj_new_int_from_uint(cam1));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_unknown),      mp_obj_new_int_from_uint(unk));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_invoke_ms_sum),mp_obj_new_int_from_uint(inv_sum));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_invoke_ms_min),mp_obj_new_int_from_uint(inv_min));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_invoke_ms_max),mp_obj_new_int_from_uint(inv_max));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_total_ms_sum), mp_obj_new_int_from_uint(tot_sum));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_total_ms_min), mp_obj_new_int_from_uint(tot_min));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_total_ms_max), mp_obj_new_int_from_uint(tot_max));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_wall_ms),      mp_obj_new_int_from_uint(wall_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_fps_x100),     mp_obj_new_int_from_uint(fps_x100));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_detections_total), mp_obj_new_int_from_uint(det_total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_parity_skipped),
                      mp_obj_new_int_from_uint(pf_sk1 - pf_sk0));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_parity_timeout),
                      mp_obj_new_int_from_uint(pf_to1 - pf_to0));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_calibrate_obj,
                                            0, 7, mod_sentai_pipeline_calibrate);

// sentai.pipeline.loop_delay([n]) -> int (previous value)
//
// Per-iteration sleep (ms) injected at the END of PrepTask, in addition
// to any prep_fps throttle.  Use to find the camera-alternation
// sweet-spot — small values (1..30 ms at VGA45 alt 1:1) let the CSI
// ISR cycle catch up before PrepTask grabs again.  Clamped [0, 200].
//
// Same knob is used internally by sentai.pipeline.calibrate(delay_ms=N)
// during the run; calibrate restores it on exit.
static mp_obj_t mod_sentai_pipeline_loop_delay(size_t n_args,
                                              const mp_obj_t *args) {
    int prev = sentai_pipeline_loop_delay_get();
    if (n_args >= 1) {
        sentai_pipeline_loop_delay_set(mp_obj_get_int(args[0]));
    }
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_pipeline_loop_delay_obj,
                                            0, 1, mod_sentai_pipeline_loop_delay);


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
    { MP_ROM_QSTR(MP_QSTR_target_fps),    MP_ROM_PTR(&mod_sentai_pipeline_target_fps_obj) },
    { MP_ROM_QSTR(MP_QSTR_prep_fps),      MP_ROM_PTR(&mod_sentai_pipeline_prep_fps_obj) },
    { MP_ROM_QSTR(MP_QSTR_prep_stats),    MP_ROM_PTR(&mod_sentai_pipeline_prep_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_prep_reset),    MP_ROM_PTR(&mod_sentai_pipeline_prep_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_running),       MP_ROM_PTR(&mod_sentai_pipeline_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),         MP_ROM_PTR(&mod_sentai_pipeline_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracks),        MP_ROM_PTR(&mod_sentai_pipeline_tracks_obj) },
    { MP_ROM_QSTR(MP_QSTR_event),         MP_ROM_PTR(&mod_sentai_pipeline_event_obj) },
    { MP_ROM_QSTR(MP_QSTR_track_config),  MP_ROM_PTR(&mod_sentai_pipeline_track_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pose),       MP_ROM_PTR(&mod_sentai_pipeline_set_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_camera_config),  MP_ROM_PTR(&mod_sentai_pipeline_camera_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_health),    MP_ROM_PTR(&mod_sentai_pipeline_task_health_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug_prep_mode), MP_ROM_PTR(&mod_sentai_pipeline_debug_prep_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_debug_no_invoke), MP_ROM_PTR(&mod_sentai_pipeline_debug_no_invoke_obj) },
    { MP_ROM_QSTR(MP_QSTR_infer_stats),     MP_ROM_PTR(&mod_sentai_pipeline_infer_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_infer_reset),     MP_ROM_PTR(&mod_sentai_pipeline_infer_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_invokes_per_frame), MP_ROM_PTR(&mod_sentai_pipeline_invokes_per_frame_obj) },
    { MP_ROM_QSTR(MP_QSTR_multi_invoke_mode), MP_ROM_PTR(&mod_sentai_pipeline_multi_invoke_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_force_parity),       MP_ROM_PTR(&mod_sentai_pipeline_force_parity_obj) },
    { MP_ROM_QSTR(MP_QSTR_force_parity_stats), MP_ROM_PTR(&mod_sentai_pipeline_force_parity_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_force_parity_reset), MP_ROM_PTR(&mod_sentai_pipeline_force_parity_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_calibrate),          MP_ROM_PTR(&mod_sentai_pipeline_calibrate_obj) },
    { MP_ROM_QSTR(MP_QSTR_loop_delay),         MP_ROM_PTR(&mod_sentai_pipeline_loop_delay_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_pipeline_globals, sentai_pipeline_globals_table);
static const mp_obj_module_t sentai_pipeline_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_pipeline_globals,
};

/* modsentai_sim_pipeline.c — extracted from modsentai_sim.c (refactor T0, 2026-05-16).
 * Part of the SIM sentai MP bindings, #include'd from modsentai_sim.c
 * inside the single translation unit.  See Sim.md §10y "SIM file
 * organisation" for the layout rules. */


/* ===== sentai.pipeline — Phase 5.6 minimal SIM ============================
 *
 * ARM has 35+ pipeline functions covering tracking, events, camera config,
 * health, etc.  For SIM we ship the essentials needed to demonstrate the
 * camera→TPU loop:
 *
 *   pipeline.tick()             — sync: latest cam frame → CPU resize →
 *                                  TPU set_input → invoke.  Returns
 *                                  invoke time in ms (-1 on failure).
 *   pipeline.start()            — spawn a FreeRTOS task running tick()
 *                                  in a loop at target_fps_hz.
 *   pipeline.stop()             — clear the task's run flag.
 *   pipeline.running()          — bool.
 *   pipeline.stats()            — dict {frames, last_invoke_ms,
 *                                  total_invoke_ms, last_err}.
 *
 * Camera frame must come from the Gazebo bridge (camera_bridge_recv has
 * a copy in s_rgb_full_pub).  TPU helper must be running.
 */
#include "FreeRTOS.h"
#include "task.h"

extern size_t sim_camera_latest_rgb(uint8_t* dst, size_t max_bytes,
                                     int* out_w, int* out_h, uint32_t* out_seq);

#define PIPE_CAM_W   640
#define PIPE_CAM_H   480
#define PIPE_CAM_SZ  (PIPE_CAM_W * PIPE_CAM_H * 3)

static uint8_t  s_pipe_cam_buf[PIPE_CAM_SZ];
static uint8_t  s_pipe_resized[1024 * 1024];   // up to ~1 MB resized tensor
static volatile int     s_pipe_running = 0;
static TaskHandle_t     s_pipe_task = NULL;
static volatile uint32_t s_pipe_frames = 0;
static volatile uint32_t s_pipe_last_ms = 0;
static volatile uint32_t s_pipe_total_ms = 0;
static volatile int     s_pipe_last_err = 0;
static volatile uint32_t s_pipe_target_fps = 10;  // safe default; helper ~16ms invoke

/* Bilinear-ish CPU resize (nearest-neighbour for speed; works for the
 * "let me see something running" smoke level.  Production should use
 * area-resampling for accuracy — same pattern as sentai_pxp_shim_sim.c. */
static int sim_resize_rgb888_nearest(const uint8_t* src, int sw, int sh,
                                      uint8_t* dst, int dw, int dh) {
    if (!src || !dst) return -1;
    for (int y = 0; y < dh; ++y) {
        int sy = (y * sh) / dh;
        if (sy >= sh) sy = sh - 1;
        const uint8_t* srow = src + sy * sw * 3;
        uint8_t* drow = dst + y * dw * 3;
        for (int x = 0; x < dw; ++x) {
            int sx = (x * sw) / dw;
            if (sx >= sw) sx = sw - 1;
            drow[x*3+0] = srow[sx*3+0];
            drow[x*3+1] = srow[sx*3+1];
            drow[x*3+2] = srow[sx*3+2];
        }
    }
    return 0;
}

/* Core single-tick: cam → resize → set_input → invoke.  Returns inference
 * latency in ms, or negative on error. */
static int pipeline_tick_once(void) {
    if (!sentai_tpu_is_ready()) return -10;

    /* Get latest 640x480 RGB frame. */
    int cw, ch; uint32_t seq;
    size_t got = sim_camera_latest_rgb(s_pipe_cam_buf, sizeof(s_pipe_cam_buf),
                                        &cw, &ch, &seq);
    if (got == 0) return -11;   /* no frame yet (Gazebo bridge not running?) */

    /* Query TPU input dimensions. */
    int iw=0, ih=0, ic=0, itype=0, izp=0;
    uint8_t* dummy = NULL;
    if (sentai_get_tensor_info(&iw, &ih, &ic, &dummy, &itype, &izp) != 0) {
        return -12;
    }
    if (ic != 3) return -13;   /* SIM resize path is RGB888 only */
    int resized_bytes = iw * ih * 3;
    if ((size_t)resized_bytes > sizeof(s_pipe_resized)) return -14;

    if (sim_resize_rgb888_nearest(s_pipe_cam_buf, cw, ch,
                                   s_pipe_resized, iw, ih) != 0) return -15;

    /* Push to TPU + invoke. */
    if (sentai_tpu_set_input_slot(0, s_pipe_resized, resized_bytes) != 0) return -16;

    TickType_t t0 = xTaskGetTickCount();
    int rc = sentai_tpu_invoke();
    if (rc != 0) return -17 - rc;
    uint32_t dt = (uint32_t)(xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;

    s_pipe_frames    += 1;
    s_pipe_last_ms    = dt;
    s_pipe_total_ms  += dt;
    s_pipe_last_err   = 0;

    /* Update tracker history (declared below; forward-decl). */
    extern void track_record(void);
    track_record();

    /* If the loaded model is SSD-style (4 outputs), auto-feed SentAI-SORT
     * with decoded detections.  Defined below — forward-decl + later
     * code calls it.  See ssd_decode_into() further down. */
    extern int  pipeline_autoupdate_tracker_if_ssd(void);
    pipeline_autoupdate_tracker_if_ssd();
    return (int)dt;
}

static mp_obj_t sentai_pipeline_tick_mp(void) {
    int ms = pipeline_tick_once();
    if (ms < 0) s_pipe_last_err = ms;
    return mp_obj_new_int(ms);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_tick_obj, sentai_pipeline_tick_mp);

static void pipeline_task(void* arg) {
    (void)arg;
    s_pipe_running = 1;
    while (s_pipe_running) {
        uint32_t period_ms = 1000 / (s_pipe_target_fps ? s_pipe_target_fps : 1);
        TickType_t t0 = xTaskGetTickCount();
        pipeline_tick_once();
        TickType_t spent = xTaskGetTickCount() - t0;
        TickType_t period = pdMS_TO_TICKS(period_ms);
        if (spent < period) vTaskDelay(period - spent);
        else                vTaskDelay(1);
    }
    s_pipe_task = NULL;
    vTaskDelete(NULL);
}

static mp_obj_t sentai_pipeline_start_mp(size_t n_args, const mp_obj_t* args) {
    if (s_pipe_running) return mp_obj_new_int(-1);
    if (n_args >= 1) s_pipe_target_fps = mp_obj_get_int(args[0]);
    BaseType_t ok = xTaskCreate(pipeline_task, "pipeline",
                                 configMINIMAL_STACK_SIZE * 16,
                                 NULL, tskIDLE_PRIORITY + 2, &s_pipe_task);
    if (ok != pdPASS) return mp_obj_new_int(-2);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_start_obj, 0, 1, sentai_pipeline_start_mp);

static mp_obj_t sentai_pipeline_stop_mp(void) {
    s_pipe_running = 0;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_stop_obj, sentai_pipeline_stop_mp);

static mp_obj_t sentai_pipeline_running_mp(void) {
    return mp_obj_new_bool(s_pipe_running);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_running_obj, sentai_pipeline_running_mp);

static mp_obj_t sentai_pipeline_stats_mp(void) {
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames),
                      mp_obj_new_int_from_uint(s_pipe_frames));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_invoke_ms_max),
                      mp_obj_new_int_from_uint(s_pipe_last_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_total_ms_sum),
                      mp_obj_new_int_from_uint(s_pipe_total_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_running),
                      mp_obj_new_bool(s_pipe_running));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_stats_obj, sentai_pipeline_stats_mp);

static mp_obj_t sentai_pipeline_target_fps_mp(size_t n_args, const mp_obj_t* args) {
    if (n_args >= 1) s_pipe_target_fps = mp_obj_get_int(args[0]);
    return mp_obj_new_int(s_pipe_target_fps);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_target_fps_obj, 0, 1, sentai_pipeline_target_fps_mp);

/* sentai.pipeline.detect(top_n=5) — returns top-N (idx, score_byte) tuples
 * from the FIRST output tensor of the loaded model.  Works for any
 * classification head (mobilenet 1001 classes, etc.).  For YOLO/heatmap
 * outputs the caller has to interpret bytes themselves.
 *
 * Returns: tuple of N tuples, each (class_idx_int, score_byte_int).
 * If no model loaded or no output: empty tuple. */
static mp_obj_t sentai_pipeline_detect_mp(size_t n_args, const mp_obj_t* args) {
    int top_n = (n_args >= 1) ? mp_obj_get_int(args[0]) : 5;
    if (top_n < 1) top_n = 1;
    if (top_n > 16) top_n = 16;
    if (!sentai_tpu_is_ready() || sentai_tpu_num_outputs() < 1) {
        return mp_obj_new_tuple(0, NULL);
    }
    int sz = sentai_tpu_get_output_size(0);
    const uint8_t* d = (const uint8_t*)sentai_tpu_get_output_data(0);
    if (!d || sz <= 0) return mp_obj_new_tuple(0, NULL);

    /* Bounded heap-free top-N by repeated linear scan; sz<=1001 typical. */
    int best_idx[16];
    uint8_t best_val[16];
    int found = 0;
    for (int rank = 0; rank < top_n; ++rank) {
        int   bi = -1;
        int   bv = -1;
        for (int i = 0; i < sz; ++i) {
            int v = d[i];
            /* Skip indices already chosen at higher rank. */
            int already = 0;
            for (int k = 0; k < rank; ++k) if (best_idx[k] == i) { already = 1; break; }
            if (already) continue;
            if (v > bv) { bv = v; bi = i; }
        }
        if (bi < 0) break;
        best_idx[rank] = bi;
        best_val[rank] = (uint8_t)bv;
        ++found;
    }
    mp_obj_t out[16];
    for (int i = 0; i < found; ++i) {
        mp_obj_t pair[2] = {
            mp_obj_new_int(best_idx[i]),
            mp_obj_new_int(best_val[i]),
        };
        out[i] = mp_obj_new_tuple(2, pair);
    }
    return mp_obj_new_tuple(found, out);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_detect_obj, 0, 1, sentai_pipeline_detect_mp);

/* sentai.pipeline.tracks() — returns list of "stable" classifications
 * over a sliding window of recent step()s.  A class counts as "tracked"
 * if it appeared in top-3 of >= 3 of the last 5 frames.  Useful for
 * filtering noise from per-frame detect().
 *
 * Returns: tuple of (class_idx, hit_count_in_window) tuples.  Empty
 * if no model, no recent frames, or no class qualified.
 */
#define TRACK_WINDOW 5
#define TRACK_TOPN   3
#define TRACK_MIN_HITS 3
static int s_track_hist[TRACK_WINDOW][TRACK_TOPN];
static int s_track_idx = 0;
static int s_track_count = 0;

/* Internal: called from pipeline_tick_once after a successful invoke.
 * Records the top-3 class indices into the rolling window. */
void track_record(void) {
    if (!sentai_tpu_is_ready() || sentai_tpu_num_outputs() < 1) return;
    int sz = sentai_tpu_get_output_size(0);
    const uint8_t* d = (const uint8_t*)sentai_tpu_get_output_data(0);
    if (!d || sz <= 0) return;

    int slot = s_track_idx % TRACK_WINDOW;
    int chosen[TRACK_TOPN];
    for (int rank = 0; rank < TRACK_TOPN; ++rank) {
        int bi = -1, bv = -1;
        for (int i = 0; i < sz; ++i) {
            int already = 0;
            for (int k = 0; k < rank; ++k) if (chosen[k] == i) { already = 1; break; }
            if (already) continue;
            int v = d[i];
            if (v > bv) { bv = v; bi = i; }
        }
        chosen[rank] = bi;
        s_track_hist[slot][rank] = bi;
    }
    s_track_idx++;
    if (s_track_count < TRACK_WINDOW) s_track_count++;
}

static mp_obj_t sentai_pipeline_tracks_mp(void) {
    /* Count how often each class appears in the window. */
    int n_eff = s_track_count < TRACK_WINDOW ? s_track_count : TRACK_WINDOW;
    int classes[TRACK_WINDOW * TRACK_TOPN];
    int counts [TRACK_WINDOW * TRACK_TOPN];
    int n_unique = 0;
    for (int slot = 0; slot < n_eff; ++slot) {
        for (int r = 0; r < TRACK_TOPN; ++r) {
            int c = s_track_hist[slot][r];
            if (c < 0) continue;
            int found = -1;
            for (int u = 0; u < n_unique; ++u) if (classes[u] == c) { found = u; break; }
            if (found >= 0) counts[found]++;
            else { classes[n_unique] = c; counts[n_unique] = 1; n_unique++; }
        }
    }
    /* Emit qualifying entries unsorted; caller can sort by count. */
    mp_obj_t out[16];
    int n_out = 0;
    for (int i = 0; i < n_unique && n_out < 16; ++i) {
        if (counts[i] >= TRACK_MIN_HITS) {
            mp_obj_t pair[2] = {
                mp_obj_new_int(classes[i]),
                mp_obj_new_int(counts[i]),
            };
            out[n_out++] = mp_obj_new_tuple(2, pair);
        }
    }
    return mp_obj_new_tuple(n_out, out);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_tracks_obj, sentai_pipeline_tracks_mp);

/* Reset track history (e.g., after model swap). */
static mp_obj_t sentai_pipeline_track_reset_mp(void) {
    s_track_idx = 0;
    s_track_count = 0;
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_track_reset_obj, sentai_pipeline_track_reset_mp);

/* ============= SSD MobileNet V2 detections + SentAI-SORT integration =====
 *
 * SSD MobileNet V2 (tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite) has
 * postprocess built-in.  4 outputs:
 *   out[0]: scores       float32 [20]
 *   out[1]: boxes        float32 [20,4] (ymin,xmin,ymax,xmax normalized)
 *   out[2]: num_detect   float32 [1]
 *   out[3]: classes      float32 [20]
 *
 * We pack these into the shared sentai_runtime/detection_task.h `Detection`
 * struct, then feed them to the BoT-SORT-adapted SentAI-SORT tracker
 * (sentai_tracker_*) — same code as ARM.  REPL surface mirrors the ARM
 * sentai.pipeline.* names so user scripts port unchanged.
 */
#include "examples/sentai_runtime/sentai_tracker.h"
#include "examples/sentai_runtime/detection_task.h"

#define SIM_SSD_INPUT_W 300
#define SIM_SSD_INPUT_H 300
#define SIM_SSD_INPUT_C 3
#define SIM_MAX_DETS    32

static Detection s_dets[SIM_MAX_DETS];
static int       s_n_dets = 0;
static uint32_t  s_det_frame_seq = 0;
/* Last input tensor we sent to TPU — needed by tracker's histogram path. */
extern uint8_t  s_pipe_resized[];

static int ssd_decode_into(int conf_thresh_permil) {
    s_n_dets = 0;
    if (!sentai_tpu_is_ready()) return -1;
    if (sentai_tpu_num_outputs() < 4) return -2;

    int sz_scores = sentai_tpu_get_output_size(0);
    int sz_boxes  = sentai_tpu_get_output_size(1);
    int sz_num    = sentai_tpu_get_output_size(2);
    int sz_class  = sentai_tpu_get_output_size(3);
    const float* sc  = (const float*)sentai_tpu_get_output_data(0);
    const float* bx  = (const float*)sentai_tpu_get_output_data(1);
    const float* nm  = (const float*)sentai_tpu_get_output_data(2);
    const float* cls = (const float*)sentai_tpu_get_output_data(3);
    if (!sc || !bx || !nm || !cls) return -3;
    (void)sz_scores; (void)sz_boxes; (void)sz_num; (void)sz_class;

    int n_avail = (int)nm[0];
    if (n_avail > 20) n_avail = 20;
    int kept = 0;
    for (int i = 0; i < n_avail && kept < SIM_MAX_DETS; ++i) {
        int permil = (int)(sc[i] * 1000.0f + 0.5f);
        if (permil < conf_thresh_permil) continue;
        float ymin = bx[i*4+0], xmin = bx[i*4+1];
        float ymax = bx[i*4+2], xmax = bx[i*4+3];
        Detection* d = &s_dets[kept++];
        d->x1 = (int16_t)(xmin * SIM_SSD_INPUT_W);
        d->y1 = (int16_t)(ymin * SIM_SSD_INPUT_H);
        d->x2 = (int16_t)(xmax * SIM_SSD_INPUT_W);
        d->y2 = (int16_t)(ymax * SIM_SSD_INPUT_H);
        d->conf_permil = (int16_t)permil;
        d->class_id    = (int16_t)cls[i];
    }
    s_n_dets = kept;
    s_det_frame_seq++;
    return kept;
}

/* Called from pipeline_tick_once() in the start() task — auto-feeds
 * SentAI-SORT with the latest detections when an SSD-style model is
 * loaded (4 outputs).  No-op for classification models. */
int pipeline_autoupdate_tracker_if_ssd(void) {
    if (sentai_tpu_num_outputs() < 4) return 0;
    int n = ssd_decode_into(300);   /* 30% threshold default */
    if (n <= 0) return 0;
    return sentai_tracker_update(s_dets, n,
                                  s_pipe_resized,
                                  SIM_SSD_INPUT_W, SIM_SSD_INPUT_H, SIM_SSD_INPUT_C,
                                  0 /* zp */, s_det_frame_seq);
}

/* sentai.pipeline.detections(thresh_permil=300) — returns list of
 * (x1, y1, x2, y2, conf_permil, class_id).  Decodes from the CURRENT
 * cached TPU output (last invoke).  Caller is expected to have done
 * pipeline.step() (or pipeline.start) before reading. */
static mp_obj_t sentai_pipeline_detections_mp(size_t n_args, const mp_obj_t* args) {
    int thresh = (n_args >= 1) ? mp_obj_get_int(args[0]) : 300;
    int n = ssd_decode_into(thresh);
    if (n < 0) return mp_obj_new_tuple(0, NULL);
    mp_obj_t out[SIM_MAX_DETS];
    for (int i = 0; i < n; ++i) {
        mp_obj_t t[6] = {
            mp_obj_new_int(s_dets[i].x1),
            mp_obj_new_int(s_dets[i].y1),
            mp_obj_new_int(s_dets[i].x2),
            mp_obj_new_int(s_dets[i].y2),
            mp_obj_new_int(s_dets[i].conf_permil),
            mp_obj_new_int(s_dets[i].class_id),
        };
        out[i] = mp_obj_new_tuple(6, t);
    }
    return mp_obj_new_tuple(n, out);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_detections_obj, 0, 1, sentai_pipeline_detections_mp);

/* sentai.pipeline.tracker_update(thresh_permil=300) — decodes detections
 * from current TPU output then feeds them to SentAI-SORT.  Returns the
 * number of confirmed tracks after update. */
static mp_obj_t sentai_pipeline_tracker_update_mp(size_t n_args, const mp_obj_t* args) {
    int thresh = (n_args >= 1) ? mp_obj_get_int(args[0]) : 300;
    int n = ssd_decode_into(thresh);
    if (n < 0) return mp_obj_new_int(n);
    int confirmed = sentai_tracker_update(s_dets, n,
                                            s_pipe_resized,
                                            SIM_SSD_INPUT_W, SIM_SSD_INPUT_H, SIM_SSD_INPUT_C,
                                            0 /* zp */, s_det_frame_seq);
    return mp_obj_new_int(confirmed);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_tracker_update_obj, 0, 1, sentai_pipeline_tracker_update_mp);

/* sentai.pipeline.tracker_enable(bool) */
static mp_obj_t sentai_pipeline_tracker_enable_mp(mp_obj_t v) {
    int on = mp_obj_is_true(v) ? 1 : 0;
    if (on) { sentai_tracker_reset(); sentai_tracker_set_enabled(1); }
    else      sentai_tracker_set_enabled(0);
    return mp_obj_new_int(on);
}
static MP_DEFINE_CONST_FUN_OBJ_1(sentai_pipeline_tracker_enable_obj, sentai_pipeline_tracker_enable_mp);

/* sentai.pipeline.tracker_reset() */
static mp_obj_t sentai_pipeline_tracker_reset_mp(void) {
    sentai_tracker_reset();
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_tracker_reset_obj, sentai_pipeline_tracker_reset_mp);

/* sentai.pipeline.tracker_tracks() — list of (id, class_id, x1,y1,x2,y2,
 * conf_permil, state, hits, age) */
#define SIM_MAX_TRACKS 16
static mp_obj_t sentai_pipeline_tracker_tracks_mp(void) {
    TrackedObject buf[SIM_MAX_TRACKS];
    int n = sentai_tracker_get_tracks(buf, SIM_MAX_TRACKS);
    if (n < 0) n = 0;
    mp_obj_t out[SIM_MAX_TRACKS];
    for (int i = 0; i < n; ++i) {
        mp_obj_t t[10] = {
            mp_obj_new_int(buf[i].id),
            mp_obj_new_int(buf[i].class_id),
            mp_obj_new_int(buf[i].x1),
            mp_obj_new_int(buf[i].y1),
            mp_obj_new_int(buf[i].x2),
            mp_obj_new_int(buf[i].y2),
            mp_obj_new_int(buf[i].conf_permil),
            mp_obj_new_int(buf[i].state),
            mp_obj_new_int(buf[i].hits),
            mp_obj_new_int(buf[i].age_frames),
        };
        out[i] = mp_obj_new_tuple(10, t);
    }
    return mp_obj_new_tuple(n, out);
}
static MP_DEFINE_CONST_FUN_OBJ_0(sentai_pipeline_tracker_tracks_obj, sentai_pipeline_tracker_tracks_mp);

/* sentai.pipeline.tracker_camera(cam_id, fov_h_deg, fov_v_deg) — set
 * camera intrinsics for the projection path.  Mount angles default to
 * 0 (drone looking straight down). */
static mp_obj_t sentai_pipeline_tracker_camera_mp(size_t n_args, const mp_obj_t* args) {
    int cam = (n_args >= 1) ? mp_obj_get_int(args[0]) : 0;
    CameraConfig cfg = {0};
    cfg.fov_h_deg = (n_args >= 2) ? mp_obj_get_float(args[1]) : 58.0f;
    cfg.fov_v_deg = (n_args >= 3) ? mp_obj_get_float(args[2]) : 45.0f;
    cfg.mount_pitch_deg = 0; cfg.mount_roll_deg = 0; cfg.mount_yaw_deg = 0;
    cfg.ground_ref = 0;  // CENTROID — drone overhead
    sentai_tracker_set_camera(cam, &cfg);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_tracker_camera_obj, 0, 3, sentai_pipeline_tracker_camera_mp);

/* sentai.pipeline.tracker_pose(altitude_cm, heading_deg=-1) — set drone
 * altitude + compass so tracks get ground-plane coordinates. */
static mp_obj_t sentai_pipeline_tracker_pose_mp(size_t n_args, const mp_obj_t* args) {
    int alt_cm  = (n_args >= 1) ? mp_obj_get_int(args[0]) : 100;
    int heading = (n_args >= 2) ? mp_obj_get_int(args[1]) : -1;
    sentai_tracker_set_pose(alt_cm, heading, 0.0f, 0.0f);
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_tracker_pose_obj, 0, 2, sentai_pipeline_tracker_pose_mp);

/* sentai.pipeline.tracker_event(timeout_ms=0) — non-blocking event poll.
 * Returns None if no event, otherwise tuple
 * (type, id, class_id, x1,y1,x2,y2, conf_permil, frame_seq). */
static mp_obj_t sentai_pipeline_tracker_event_mp(size_t n_args, const mp_obj_t* args) {
    int timeout_ms = (n_args >= 1) ? mp_obj_get_int(args[0]) : 0;
    TrackEvent evt;
    if (!sentai_tracker_get_event(&evt, timeout_ms)) return mp_const_none;
    mp_obj_t t[9] = {
        mp_obj_new_int(evt.type),
        mp_obj_new_int(evt.id),
        mp_obj_new_int(evt.class_id),
        mp_obj_new_int(evt.x1),
        mp_obj_new_int(evt.y1),
        mp_obj_new_int(evt.x2),
        mp_obj_new_int(evt.y2),
        mp_obj_new_int(evt.conf_permil),
        mp_obj_new_int_from_uint(evt.frame_seq),
    };
    return mp_obj_new_tuple(9, t);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(sentai_pipeline_tracker_event_obj, 0, 1, sentai_pipeline_tracker_event_mp);

static const mp_rom_map_elem_t sentai_pipeline_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_pipeline) },
    { MP_ROM_QSTR(MP_QSTR_step),        MP_ROM_PTR(&sentai_pipeline_tick_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),       MP_ROM_PTR(&sentai_pipeline_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&sentai_pipeline_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_running),     MP_ROM_PTR(&sentai_pipeline_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_stats),       MP_ROM_PTR(&sentai_pipeline_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_target_fps),  MP_ROM_PTR(&sentai_pipeline_target_fps_obj) },
    { MP_ROM_QSTR(MP_QSTR_predict),      MP_ROM_PTR(&sentai_pipeline_detect_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracks),      MP_ROM_PTR(&sentai_pipeline_tracks_obj) },
    { MP_ROM_QSTR(MP_QSTR_infer_reset), MP_ROM_PTR(&sentai_pipeline_track_reset_obj) },
    /* SSD + SORT bindings */
    { MP_ROM_QSTR(MP_QSTR_detections),     MP_ROM_PTR(&sentai_pipeline_detections_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_update), MP_ROM_PTR(&sentai_pipeline_tracker_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_enable), MP_ROM_PTR(&sentai_pipeline_tracker_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_reset),  MP_ROM_PTR(&sentai_pipeline_tracker_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_tracks), MP_ROM_PTR(&sentai_pipeline_tracker_tracks_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_camera), MP_ROM_PTR(&sentai_pipeline_tracker_camera_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_pose),   MP_ROM_PTR(&sentai_pipeline_tracker_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_tracker_event),  MP_ROM_PTR(&sentai_pipeline_tracker_event_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_pipeline_globals, sentai_pipeline_globals_table);
static const mp_obj_module_t sentai_pipeline_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *) &sentai_pipeline_globals,
};

// modsentai_flow.c -- MicroPython bindings for sentai.flow.
//
// Single architecture (build #1130+): everything on M7, in C++,
// in flow_task.cc.  M4 is no longer used (see flow_task.cc header
// for the rationale).  Bindings dropped the "m4_" prefix to reflect
// the new home.  Old `m4_*` names kept as aliases for one release.

#include <string.h>

#include "flow_shared.h"
#include "sentai_aruco_shim.h"

// Mode flag — see sentai.flow.mode().  0 = normal (flow only),
// 1 = anchor (flow + ArUco VPE publish).  Polled by flow_task
// (ARM) and camera_bridge_recv (SIM).
//
// Single writer (MicroPython REPL via mode()), multiple readers.
// `volatile` is enough — atomicity of a single uint32 store is
// guaranteed on both ARM Cortex-M7 and x86_64.
volatile uint32_t g_flow_anchor_mode = 0;

extern int      sentai_flow_enable(void);
extern int      sentai_flow_start(int cam_id);
extern int      sentai_flow_stop(void);
extern uint32_t sentai_flow_detail_score(void);
extern int      sentai_flow_gray_stretch_set(int enable);
extern int      sentai_flow_gray_stretch_get(uint8_t* vmin, uint8_t* vmax);
extern void     sentai_flow_pub_stats(uint32_t* frames_published,
                                       uint32_t* grab_fail_total,
                                       uint32_t* grab_fail_streak,
                                       int* running);
extern void     sentai_flow_perf_cyc(uint32_t* pxp, uint32_t* rgb2y,
                                      uint32_t* stretch, uint32_t* sad,
                                      uint32_t* total, uint32_t* grab,
                                      uint32_t* loop);
extern void     sentai_flow_deadband_state(uint32_t* period_ms_x10,
                                            uint32_t* deadband_mgp,
                                            uint32_t* velocity_mgp_per_s);
extern int      sentai_fs_cache_write(const uint8_t* data, int size);

// sentai.flow.enable() -> int (0 on success, -1 if no compute backend)
static mp_obj_t mod_sentai_flow_enable(void) {
    return mp_obj_new_int(sentai_flow_enable());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_enable_obj,
                                  mod_sentai_flow_enable);

// sentai.flow.start([cam_id=0]) -> int
//
// Default cam_id=0 (FRONT camera per cam_mux.h convention: I2C bus 1,
// MUX low).  Override at init by passing cam_id=1 (back camera).  The
// camera selection is captured ONCE at start; switching cameras
// mid-run requires stop+start (or sentai.camera.select() with the
// 1.5 s settle the visual snapshot path uses).
static mp_obj_t mod_sentai_flow_start(size_t n_args, const mp_obj_t *args) {
    int cam_id = 0;  // FRONT (cam0) by default
    if (n_args >= 1) cam_id = mp_obj_get_int(args[0]);
    int rc = sentai_flow_start(cam_id);
    if (rc != 0) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
                          MP_ERROR_TEXT("flow.start failed (%d)"), rc);
    }
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_flow_start_obj,
                                            0, 1, mod_sentai_flow_start);

// sentai.flow.stop() -> int
static mp_obj_t mod_sentai_flow_stop(void) {
    return mp_obj_new_int(sentai_flow_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_stop_obj,
                                  mod_sentai_flow_stop);

// sentai.flow.read() -> dict
//
// Returns the latest SAD result.  All fields published by the M7
// publisher path; same shape across builds for driver compat.
static mp_obj_t mod_sentai_flow_read(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    int alive = (sh->magic == FLOW_SHARED_MAGIC);
    mp_obj_t d = mp_obj_new_dict(8);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_alive), mp_obj_new_bool(alive));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_state),
                      mp_obj_new_int_from_uint(sh->m4_state));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dx),
                      mp_obj_new_int(sh->last_dx));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dy),
                      mp_obj_new_int(sh->last_dy));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sad),
                      mp_obj_new_int_from_uint(sh->last_sad));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_confidence),
                      mp_obj_new_int(sh->last_confidence));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_seq),
                      mp_obj_new_int_from_uint(sh->last_frame_seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cam_id),
                      mp_obj_new_int(sh->frame_cam_id));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_read_obj,
                                  mod_sentai_flow_read);

// sentai.flow.body_read() -> dict with body-frame conversion.
// cam0:  body_fw = -dx,  body_left = +dy
// cam1:  body_fw = +dx,  body_left = -dy
static mp_obj_t mod_sentai_flow_body_read(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    int alive = (sh->magic == FLOW_SHARED_MAGIC);
    int32_t dx = sh->last_dx;
    int32_t dy = sh->last_dy;
    int cam_id = sh->frame_cam_id;
    int32_t body_fw, body_left;
    if (cam_id == 0) { body_fw = -dx; body_left = +dy; }
    else             { body_fw = +dx; body_left = -dy; }
    mp_obj_t d = mp_obj_new_dict(7);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_alive), mp_obj_new_bool(alive));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_body_fw),   mp_obj_new_int(body_fw));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_body_left), mp_obj_new_int(body_left));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dx), mp_obj_new_int(dx));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_dy), mp_obj_new_int(dy));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_seq),
                      mp_obj_new_int_from_uint(sh->last_frame_seq));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cam_id),
                      mp_obj_new_int(cam_id));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_body_read_obj,
                                  mod_sentai_flow_body_read);

// sentai.flow.gray_snap() -> bytes  (copy of the 80x60 published gray)
static mp_obj_t mod_sentai_flow_gray_snap(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    return mp_obj_new_bytes((const uint8_t*)sh->gray, FLOW_GRAY_PIXELS);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_gray_snap_obj,
                                  mod_sentai_flow_gray_snap);

// sentai.flow.gray_to_cache() -> int  (zero-alloc: copy gray straight
// into the diag write-cache; lets bulk-capture loops stay snappy
// without 4800 B per-iter MP heap churn).
static mp_obj_t mod_sentai_flow_gray_to_cache(void) {
    volatile flow_shared_t* sh = &FLOW_SHARED();
    return mp_obj_new_int(sentai_fs_cache_write(
        (const uint8_t*)sh->gray, FLOW_GRAY_PIXELS));
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_gray_to_cache_obj,
                                  mod_sentai_flow_gray_to_cache);

// sentai.flow.detail_score() -> int (gradient energy x100)
static mp_obj_t mod_sentai_flow_detail_score(void) {
    return mp_obj_new_int_from_uint(sentai_flow_detail_score());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_detail_score_obj,
                                  mod_sentai_flow_detail_score);

// sentai.flow.gray_stretch([enable]) -> dict
static mp_obj_t mod_sentai_flow_gray_stretch(size_t n_args,
                                              const mp_obj_t *args) {
    uint8_t vmin = 0, vmax = 0;
    int prev;
    if (n_args == 0) {
        prev = sentai_flow_gray_stretch_get(&vmin, &vmax);
    } else {
        int want = mp_obj_is_true(args[0]) ? 1 : 0;
        prev = sentai_flow_gray_stretch_set(want);
        sentai_flow_gray_stretch_get(&vmin, &vmax);
    }
    mp_obj_t d = mp_obj_new_dict(3);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_enabled),
                      mp_obj_new_bool(prev != 0));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_vmin),
                      mp_obj_new_int(vmin));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_vmax),
                      mp_obj_new_int(vmax));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_flow_gray_stretch_obj,
                                            0, 1, mod_sentai_flow_gray_stretch);

// sentai.flow.period_ms() -> float
//
// Current per-frame sample period in milliseconds, derived from the
// rate-aware deadband state machine (sliding-window over the camera
// notify cadence). Useful for the drone-flow injection path that
// needs an explicit `dt` per measurement: pass this into the
// flow_pkt_t.dt field to match the actual integration window.
//
// Returns the *most recently observed* period; on cold start (before
// the first deadband recompute window completes) returns the
// 30-fps bootstrap value (33.3 ms).
static mp_obj_t mod_sentai_flow_period_ms(void) {
    uint32_t per_x10 = 333;
    sentai_flow_deadband_state(&per_x10, NULL, NULL);
    return mp_obj_new_float((float)per_x10 / 10.0f);
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_period_ms_obj,
                                  mod_sentai_flow_period_ms);

// sentai.flow.pub_stats() -> dict (publisher diag counters)
static mp_obj_t mod_sentai_flow_pub_stats(void) {
    uint32_t frames = 0, grab_fail_total = 0, grab_fail_streak = 0;
    int running = 0;
    sentai_flow_pub_stats(&frames, &grab_fail_total,
                           &grab_fail_streak, &running);
    mp_obj_t d = mp_obj_new_dict(4);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames),
                      mp_obj_new_int_from_uint(frames));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_grab_fail_total),
                      mp_obj_new_int_from_uint(grab_fail_total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_grab_fail_streak),
                      mp_obj_new_int_from_uint(grab_fail_streak));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_running),
                      mp_obj_new_bool(running));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_pub_stats_obj,
                                  mod_sentai_flow_pub_stats);

// sentai.flow.perf() -> dict (DWT cycle counts per stage; 800 cyc = 1us)
static mp_obj_t mod_sentai_flow_perf(void) {
    uint32_t pxp = 0, rgb2y = 0, stretch = 0, sad = 0,
             total = 0, grab = 0, loop = 0;
    sentai_flow_perf_cyc(&pxp, &rgb2y, &stretch, &sad, &total, &grab, &loop);
    mp_obj_t d = mp_obj_new_dict(7);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_pxp_cyc),     mp_obj_new_int_from_uint(pxp));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rgb2y_cyc),   mp_obj_new_int_from_uint(rgb2y));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_stretch_cyc), mp_obj_new_int_from_uint(stretch));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_sad_cyc),     mp_obj_new_int_from_uint(sad));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_total_cyc),   mp_obj_new_int_from_uint(total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_grab_cyc),    mp_obj_new_int_from_uint(grab));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_loop_cyc),    mp_obj_new_int_from_uint(loop));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_perf_obj,
                                  mod_sentai_flow_perf);

// sentai.flow.mode(name) -> str
//
// Switches the optical-flow path between "normal" (flow only — the
// default) and "anchor" (flow + per-frame ArUco detection that
// publishes drone WORLD pose).  Returns the active mode after the
// call.  Architecture: see sentai_aruco_shim.h.
//
//   sentai.flow.mode()           -> read current mode
//   sentai.flow.mode("normal")   -> back to flow-only
//   sentai.flow.mode("anchor")   -> enable anchor + lazy-init shim
__attribute__((section(".sdram_text"), noinline))
static mp_obj_t mod_sentai_flow_mode(size_t n_args, const mp_obj_t* args) {
    if (n_args >= 1) {
        size_t len = 0;
        const char* s = mp_obj_str_get_data(args[0], &len);
        if (len == 6 && strncmp(s, "normal", 6) == 0) {
            g_flow_anchor_mode = 0;
        } else if (len == 6 && strncmp(s, "anchor", 6) == 0) {
            sentai_aruco_init();          // idempotent
            g_flow_anchor_mode = 1;
        } else {
            mp_raise_msg_varg(&mp_type_ValueError,
                              MP_ERROR_TEXT("flow.mode expects 'normal' or 'anchor'"));
        }
    }
    const char* name = (g_flow_anchor_mode == 1) ? "anchor" : "normal";
    return mp_obj_new_str(name, strlen(name));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_flow_mode_obj,
                                            0, 1, mod_sentai_flow_mode);

// sentai.flow.anchor_pose() -> dict
//
// Returns the latest snapshot from the ArUco anchor shim.  Available
// in both 'normal' and 'anchor' modes (in 'normal' the dict will
// report detected=0 unless a previous anchor session left state).
// Useful for diagnostics + as a feed for sentai.link.send_vpe(...).
__attribute__((section(".sdram_text"), noinline))
static mp_obj_t mod_sentai_flow_anchor_pose(void) {
    sentai_aruco_pose_t p = {0};
    sentai_aruco_get_latest(&p);
    mp_obj_t d = mp_obj_new_dict(9);
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_detected),    mp_obj_new_bool(p.detected));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_num_markers), mp_obj_new_int(p.num_markers));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_x),           mp_obj_new_float(p.x_m));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_y),           mp_obj_new_float(p.y_m));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_z),           mp_obj_new_float(p.z_m));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_yaw),         mp_obj_new_float(p.yaw_rad));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_seq),   mp_obj_new_int_from_uint(p.frame_seq));
    mp_obj_dict_store(d, mp_obj_new_str("detect_us", 9),   mp_obj_new_int_from_uint(p.detect_us));
    mp_obj_dict_store(d, mp_obj_new_str("src_ts_ms", 9),   mp_obj_new_int_from_uint(p.src_ts_ms));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_anchor_pose_obj,
                                  mod_sentai_flow_anchor_pose);

// sentai.flow.anchor_forward(rate_hz, target="auto") -> int
//
// Enables the continuous C++ forwarder task that pushes the latest
// anchor pose to the flight controller(s) at `rate_hz` Hz.  Stops
// the task if rate_hz == 0 or target == "off".
//
//   target:  "auto" | "px4" | "cf2" | "both" | "off"
//   rate_hz: 1..200 (capped); 0 stops
//
// Returns 0 on success, -1 if FreeRTOS could not spawn the task,
// -2 if `target` is not one of the accepted names.
extern int sentai_anchor_forward_start(uint32_t rate_hz,
                                        const char* target, int target_len);
extern int sentai_anchor_forward_stop(void);
extern void sentai_anchor_forward_stats(uint32_t* sent_px4, uint32_t* sent_cf2,
                                         uint32_t* skipped, uint32_t* last_seq,
                                         uint32_t* send_failed, uint32_t* running,
                                         uint32_t* rate_hz, uint32_t* target);
extern uint32_t sentai_anchor_forward_iters(void);
extern void sentai_anchor_forward_health(uint32_t* dropped_nonfinite,
                                          uint32_t* dropped_oob,
                                          uint32_t* dropped_stale,
                                          uint32_t* stack_hwm_words);

__attribute__((section(".sdram_text"), noinline))
static mp_obj_t mod_sentai_flow_anchor_forward(size_t n_args, const mp_obj_t* args) {
    int rate = (n_args >= 1) ? mp_obj_get_int(args[0]) : 10;
    const char* tgt = "auto";
    size_t tgt_len = 4;
    if (n_args >= 2) tgt = mp_obj_str_get_data(args[1], &tgt_len);
    int rc = sentai_anchor_forward_start((uint32_t)rate, tgt, (int)tgt_len);
    if (rc == -2) {
        mp_raise_msg_varg(&mp_type_ValueError,
                          MP_ERROR_TEXT("flow.anchor_forward: target must be 'auto'|'px4'|'cf2'|'both'|'off'"));
    }
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mod_sentai_flow_anchor_forward_obj,
                                            0, 2, mod_sentai_flow_anchor_forward);

// sentai.flow.anchor_forward_stats() -> dict
__attribute__((section(".sdram_text"), noinline))
static mp_obj_t mod_sentai_flow_anchor_forward_stats(void) {
    uint32_t sent_px4 = 0, sent_cf2 = 0, skipped = 0, last_seq = 0;
    uint32_t send_failed = 0, running = 0, rate_hz = 0, target = 0;
    sentai_anchor_forward_stats(&sent_px4, &sent_cf2, &skipped, &last_seq,
                                 &send_failed, &running, &rate_hz, &target);
    static const char* const TGT_NAMES[] = {"off","auto","px4","cf2","both"};
    const char* tname = (target < 5) ? TGT_NAMES[target] : "unknown";
    mp_obj_t d = mp_obj_new_dict(8);
    mp_obj_dict_store(d, mp_obj_new_str("sent_px4",    8), mp_obj_new_int_from_uint(sent_px4));
    mp_obj_dict_store(d, mp_obj_new_str("sent_cf2",    8), mp_obj_new_int_from_uint(sent_cf2));
    mp_obj_dict_store(d, mp_obj_new_str("skipped",     7), mp_obj_new_int_from_uint(skipped));
    mp_obj_dict_store(d, mp_obj_new_str("last_seq",    8), mp_obj_new_int_from_uint(last_seq));
    mp_obj_dict_store(d, mp_obj_new_str("send_failed",11), mp_obj_new_int_from_uint(send_failed));
    mp_obj_dict_store(d, mp_obj_new_str("running",     7), mp_obj_new_bool(running));
    mp_obj_dict_store(d, mp_obj_new_str("rate_hz",     7), mp_obj_new_int_from_uint(rate_hz));
    mp_obj_dict_store(d, mp_obj_new_str("target",      6), mp_obj_new_str(tname, strlen(tname)));
    mp_obj_dict_store(d, mp_obj_new_str("iters",       5), mp_obj_new_int_from_uint(sentai_anchor_forward_iters()));
    // embeded.md fault-model counters packed as a single tuple to
    // minimise m_text dict-store call sites.  Order:
    //   (dropped_nonfinite, dropped_oob, dropped_stale, stack_hwm_words)
    uint32_t dnf = 0, doob = 0, dst = 0, hwm = 0;
    sentai_anchor_forward_health(&dnf, &doob, &dst, &hwm);
    mp_obj_t htup[4] = {
        mp_obj_new_int_from_uint(dnf),
        mp_obj_new_int_from_uint(doob),
        mp_obj_new_int_from_uint(dst),
        mp_obj_new_int_from_uint(hwm),
    };
    mp_obj_dict_store(d, mp_obj_new_str("health", 6), mp_obj_new_tuple(4, htup));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_anchor_forward_stats_obj,
                                  mod_sentai_flow_anchor_forward_stats);

// ---- module table ----
static const mp_rom_map_elem_t sentai_flow_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),       MP_ROM_QSTR(MP_QSTR_flow) },
    { MP_ROM_QSTR(MP_QSTR_enable),         MP_ROM_PTR(&mod_sentai_flow_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_start),          MP_ROM_PTR(&mod_sentai_flow_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),           MP_ROM_PTR(&mod_sentai_flow_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),           MP_ROM_PTR(&mod_sentai_flow_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_body_read),      MP_ROM_PTR(&mod_sentai_flow_body_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_gray_snap),      MP_ROM_PTR(&mod_sentai_flow_gray_snap_obj) },
    { MP_ROM_QSTR(MP_QSTR_gray_to_cache),  MP_ROM_PTR(&mod_sentai_flow_gray_to_cache_obj) },
    { MP_ROM_QSTR(MP_QSTR_detail_score),   MP_ROM_PTR(&mod_sentai_flow_detail_score_obj) },
    { MP_ROM_QSTR(MP_QSTR_gray_stretch),   MP_ROM_PTR(&mod_sentai_flow_gray_stretch_obj) },
    { MP_ROM_QSTR(MP_QSTR_period_ms),      MP_ROM_PTR(&mod_sentai_flow_period_ms_obj) },
    { MP_ROM_QSTR(MP_QSTR_pub_stats),      MP_ROM_PTR(&mod_sentai_flow_pub_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_perf),           MP_ROM_PTR(&mod_sentai_flow_perf_obj) },
    { MP_ROM_QSTR(MP_QSTR_mode),           MP_ROM_PTR(&mod_sentai_flow_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_anchor_pose),    MP_ROM_PTR(&mod_sentai_flow_anchor_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_anchor_forward), MP_ROM_PTR(&mod_sentai_flow_anchor_forward_obj) },
    { MP_ROM_QSTR(MP_QSTR_anchor_forward_stats),
                                           MP_ROM_PTR(&mod_sentai_flow_anchor_forward_stats_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_flow_globals, sentai_flow_globals_table);
static const mp_obj_module_t sentai_flow_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_flow_globals,
};

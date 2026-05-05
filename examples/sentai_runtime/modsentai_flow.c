// modsentai_flow.c -- MicroPython bindings for sentai.flow.
//
// Single architecture (build #1130+): everything on M7, in C++,
// in flow_task.cc.  M4 is no longer used (see flow_task.cc header
// for the rationale).  Bindings dropped the "m4_" prefix to reflect
// the new home.  Old `m4_*` names kept as aliases for one release.

#include <string.h>

#include "flow_shared.h"

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
extern int      sentai_fs_cache_write(const uint8_t* data, int size);

// sentai.flow.enable() -> int (0 on success, -1 if no compute backend)
static mp_obj_t mod_sentai_flow_enable(void) {
    return mp_obj_new_int(sentai_flow_enable());
}
static MP_DEFINE_CONST_FUN_OBJ_0(mod_sentai_flow_enable_obj,
                                  mod_sentai_flow_enable);

// sentai.flow.start([cam_id=0]) -> int
static mp_obj_t mod_sentai_flow_start(size_t n_args, const mp_obj_t *args) {
    int cam_id = 0;
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
    { MP_ROM_QSTR(MP_QSTR_pub_stats),      MP_ROM_PTR(&mod_sentai_flow_pub_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_perf),           MP_ROM_PTR(&mod_sentai_flow_perf_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_flow_globals, sentai_flow_globals_table);
static const mp_obj_module_t sentai_flow_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&sentai_flow_globals,
};

// modsentai_whycon.c — MicroPython binding for sentai.whycon.* (timing
// bench API for OP-S10-W17-T1 WhyCon-lite circular-marker prototype).
//
// Surface (intentionally minimal; this is a perf-measurement module
// not a production navigation surface):
//
//   sentai.whycon._test_synth(n, radius=15)       -> int n_detected (lite-bench solid disks)
//   sentai.whycon._test_synth_krajnik(n, R=15)    -> int n_detected (W3 + PnP bench)
//   sentai.whycon._test_pgm(path)                 -> int n_detected (or <0 err)
//   sentai.whycon._detect_cyc()                   -> uint cycles for last call
//   sentai.whycon._set_concentric(on)             -> none
//   sentai.whycon._set_diameter(metres)           -> none  (W19-T2 PnP enable)
//   sentai.whycon._stage_cyc()                    -> (t_a, t_b, t_w) lite 3-tuple
//   sentai.whycon._stage_cyc5()                   -> (t_a, t_b, t_w, t_w3, t_pnp)
//   sentai.whycon._get_markers()                  -> list[dict{cx,cy,a,b,ang}]  (legacy lite)
//   sentai.whycon._get_marker_details(i, buf)     -> int (writes 56 B struct; see ABI below)
//
// Cycles → ms at 800 MHz M7: divide by 800.
//
// Per [[no-heavy-data-through-mp]] HARD-RULE: new APIs use caller-
// allocated bytearrays for struct emission.  The legacy
// `_get_markers()` dict path is preserved for back-compat with
// existing test scripts; it stays lite-only and is slated for
// removal in W19-T1.

#include <string.h>

#include "py/runtime.h"
#include "py/objarray.h"

extern int      sentai_whycon_test_synth(int n_circles, int radius);
extern int      sentai_whycon_test_synth_krajnik(int n_circles, int radius);
extern int      sentai_whycon_test_pgm(const char* path);
extern uint32_t sentai_whycon_detect_cyc_last(void);
extern void     sentai_whycon_set_concentric_check(int on);
extern void     sentai_whycon_set_diameter(float meters);
extern void     sentai_whycon_stage_cyc(uint32_t* t_a, uint32_t* t_b, uint32_t* t_w);
extern void     sentai_whycon_stage_cyc5(uint32_t* t_a, uint32_t* t_b,
                                            uint32_t* t_w, uint32_t* t_w3,
                                            uint32_t* t_pnp);

// Mirror of sentai_whycon_marker_t (sentai_aruco.cc).  Any layout
// change there must be reflected here; the static_assert below
// catches binary-size drift at compile time.
typedef struct {
    float    cx, cy, axis_a, axis_b, angle;
    int      comp_id;
    float    tvec_cam[3];
    float    rvec_cam[3];
    float    reproj_err_px;
    uint8_t  pose_valid;
    uint8_t  _pad[3];
} sentai_whycon_marker_pub_t;
_Static_assert(sizeof(sentai_whycon_marker_pub_t) == 56,
               "sentai_whycon_marker_pub_t must match sentai_whycon_marker_t (56 B)");
extern int sentai_whycon_get_markers(sentai_whycon_marker_pub_t* out, int cap);

#define WHYCON_MAX_DETS_BIND 16

static mp_obj_t whycon_test_synth(size_t n_args, const mp_obj_t* args) {
    const int n = mp_obj_get_int(args[0]);
    const int r = (n_args >= 2) ? mp_obj_get_int(args[1]) : 15;
    return mp_obj_new_int(sentai_whycon_test_synth(n, r));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(whycon_test_synth_obj, 1, 2,
                                             whycon_test_synth);

static mp_obj_t whycon_test_synth_krajnik(size_t n_args,
                                             const mp_obj_t* args) {
    const int n = mp_obj_get_int(args[0]);
    const int r = (n_args >= 2) ? mp_obj_get_int(args[1]) : 15;
    return mp_obj_new_int(sentai_whycon_test_synth_krajnik(n, r));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(whycon_test_synth_krajnik_obj,
                                             1, 2, whycon_test_synth_krajnik);

static mp_obj_t whycon_test_pgm(mp_obj_t path_obj) {
    return mp_obj_new_int(sentai_whycon_test_pgm(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(whycon_test_pgm_obj, whycon_test_pgm);

static mp_obj_t whycon_detect_cyc_(void) {
    return mp_obj_new_int_from_uint(sentai_whycon_detect_cyc_last());
}
static MP_DEFINE_CONST_FUN_OBJ_0(whycon_detect_cyc_obj, whycon_detect_cyc_);

static mp_obj_t whycon_stage_cyc_(void) {
    uint32_t a = 0, b = 0, w = 0;
    sentai_whycon_stage_cyc(&a, &b, &w);
    mp_obj_t t[3] = {
        mp_obj_new_int_from_uint(a),
        mp_obj_new_int_from_uint(b),
        mp_obj_new_int_from_uint(w),
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(whycon_stage_cyc_obj, whycon_stage_cyc_);

static mp_obj_t whycon_set_concentric(mp_obj_t on_obj) {
    sentai_whycon_set_concentric_check(mp_obj_get_int(on_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(whycon_set_concentric_obj, whycon_set_concentric);

static mp_obj_t whycon_set_diameter(mp_obj_t d_obj) {
    sentai_whycon_set_diameter((float)mp_obj_get_float(d_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(whycon_set_diameter_obj, whycon_set_diameter);

static mp_obj_t whycon_stage_cyc5_(void) {
    uint32_t a = 0, b = 0, w = 0, w3 = 0, pnp = 0;
    sentai_whycon_stage_cyc5(&a, &b, &w, &w3, &pnp);
    mp_obj_t t[5] = {
        mp_obj_new_int_from_uint(a),
        mp_obj_new_int_from_uint(b),
        mp_obj_new_int_from_uint(w),
        mp_obj_new_int_from_uint(w3),
        mp_obj_new_int_from_uint(pnp),
    };
    return mp_obj_new_tuple(5, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(whycon_stage_cyc5_obj, whycon_stage_cyc5_);

// _get_marker_details(idx, out_buf) — writes a single 56-byte
// sentai_whycon_marker_pub_t into out_buf.  Returns 1 if a marker
// at `idx` is available, 0 otherwise (out_buf untouched).  Caller
// must allocate out_buf as bytearray(56).
static mp_obj_t whycon_get_marker_details(mp_obj_t idx_obj,
                                             mp_obj_t buf_obj) {
    const int idx = mp_obj_get_int(idx_obj);
    if (idx < 0 || idx >= WHYCON_MAX_DETS_BIND) return mp_obj_new_int(0);
    mp_buffer_info_t bi;
    if (!mp_get_buffer(buf_obj, &bi, MP_BUFFER_WRITE)) {
        mp_raise_TypeError(MP_ERROR_TEXT("out_buf must be writable bytearray"));
    }
    if (bi.len < (mp_int_t)sizeof(sentai_whycon_marker_pub_t)) {
        mp_raise_ValueError(MP_ERROR_TEXT("out_buf too small (need 56 B)"));
    }
    sentai_whycon_marker_pub_t mk[WHYCON_MAX_DETS_BIND];
    const int n = sentai_whycon_get_markers(mk, WHYCON_MAX_DETS_BIND);
    if (idx >= n) return mp_obj_new_int(0);
    memcpy(bi.buf, &mk[idx], sizeof(sentai_whycon_marker_pub_t));
    return mp_obj_new_int(1);
}
static MP_DEFINE_CONST_FUN_OBJ_2(whycon_get_marker_details_obj,
                                   whycon_get_marker_details);

static mp_obj_t whycon_get_markers(void) {
    sentai_whycon_marker_pub_t mk[WHYCON_MAX_DETS_BIND];
    const int n = sentai_whycon_get_markers(mk, WHYCON_MAX_DETS_BIND);
    mp_obj_t lst = mp_obj_new_list(0, NULL);
    for (int i = 0; i < n; ++i) {
        mp_obj_t d = mp_obj_new_dict(0);
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cx),
                           mp_obj_new_float(mk[i].cx));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_cy),
                           mp_obj_new_float(mk[i].cy));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_a),
                           mp_obj_new_float(mk[i].axis_a));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_b),
                           mp_obj_new_float(mk[i].axis_b));
        mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_ang),
                           mp_obj_new_float(mk[i].angle));
        mp_obj_list_append(lst, d);
    }
    return lst;
}
static MP_DEFINE_CONST_FUN_OBJ_0(whycon_get_markers_obj, whycon_get_markers);

static const mp_rom_map_elem_t sentai_whycon_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),             MP_ROM_QSTR(MP_QSTR_whycon) },
    { MP_ROM_QSTR(MP_QSTR__test_synth),          MP_ROM_PTR(&whycon_test_synth_obj) },
    { MP_ROM_QSTR(MP_QSTR__test_synth_krajnik),  MP_ROM_PTR(&whycon_test_synth_krajnik_obj) },
    { MP_ROM_QSTR(MP_QSTR__test_pgm),            MP_ROM_PTR(&whycon_test_pgm_obj) },
    { MP_ROM_QSTR(MP_QSTR__detect_cyc),          MP_ROM_PTR(&whycon_detect_cyc_obj) },
    { MP_ROM_QSTR(MP_QSTR__stage_cyc),           MP_ROM_PTR(&whycon_stage_cyc_obj) },
    { MP_ROM_QSTR(MP_QSTR__stage_cyc5),          MP_ROM_PTR(&whycon_stage_cyc5_obj) },
    { MP_ROM_QSTR(MP_QSTR__set_concentric),      MP_ROM_PTR(&whycon_set_concentric_obj) },
    { MP_ROM_QSTR(MP_QSTR__set_diameter),        MP_ROM_PTR(&whycon_set_diameter_obj) },
    { MP_ROM_QSTR(MP_QSTR__get_markers),         MP_ROM_PTR(&whycon_get_markers_obj) },
    { MP_ROM_QSTR(MP_QSTR__get_marker_details),  MP_ROM_PTR(&whycon_get_marker_details_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_whycon_globals, sentai_whycon_globals_table);

const mp_obj_module_t sentai_whycon_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_whycon_globals,
};

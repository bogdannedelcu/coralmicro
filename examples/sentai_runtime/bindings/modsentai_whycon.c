// modsentai_whycon.c — MicroPython binding for sentai.whycon.* (timing
// bench API for OP-S10-W17-T1 WhyCon-lite circular-marker prototype).
//
// Surface (intentionally minimal; this is a perf-measurement module
// not a production navigation surface):
//
//   sentai.whycon._test_synth(n, radius=15) -> int n_detected
//   sentai.whycon._test_pgm(path)           -> int n_detected (or <0 err)
//   sentai.whycon._detect_cyc()             -> uint cycles for last call
//   sentai.whycon._set_concentric(on)       -> int previous flag
//   sentai.whycon._get_markers()            -> list[dict{cx,cy,a,b,ang}]
//
// Cycles → ms at 800 MHz M7: divide by 800.

#include "py/runtime.h"

extern int      sentai_whycon_test_synth(int n_circles, int radius);
extern int      sentai_whycon_test_pgm(const char* path);
extern uint32_t sentai_whycon_detect_cyc_last(void);
extern void     sentai_whycon_set_concentric_check(int on);

typedef struct {
    float cx, cy, axis_a, axis_b, angle;
    int   comp_id;
} sentai_whycon_marker_pub_t;
extern int sentai_whycon_get_markers(sentai_whycon_marker_pub_t* out, int cap);

#define WHYCON_MAX_DETS_BIND 16

static mp_obj_t whycon_test_synth(size_t n_args, const mp_obj_t* args) {
    const int n = mp_obj_get_int(args[0]);
    const int r = (n_args >= 2) ? mp_obj_get_int(args[1]) : 15;
    return mp_obj_new_int(sentai_whycon_test_synth(n, r));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(whycon_test_synth_obj, 1, 2,
                                             whycon_test_synth);

static mp_obj_t whycon_test_pgm(mp_obj_t path_obj) {
    return mp_obj_new_int(sentai_whycon_test_pgm(mp_obj_str_get_str(path_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(whycon_test_pgm_obj, whycon_test_pgm);

static mp_obj_t whycon_detect_cyc_(void) {
    return mp_obj_new_int_from_uint(sentai_whycon_detect_cyc_last());
}
static MP_DEFINE_CONST_FUN_OBJ_0(whycon_detect_cyc_obj, whycon_detect_cyc_);

static mp_obj_t whycon_set_concentric(mp_obj_t on_obj) {
    sentai_whycon_set_concentric_check(mp_obj_get_int(on_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(whycon_set_concentric_obj, whycon_set_concentric);

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
    { MP_ROM_QSTR(MP_QSTR___name__),        MP_ROM_QSTR(MP_QSTR_whycon) },
    { MP_ROM_QSTR(MP_QSTR__test_synth),     MP_ROM_PTR(&whycon_test_synth_obj) },
    { MP_ROM_QSTR(MP_QSTR__test_pgm),       MP_ROM_PTR(&whycon_test_pgm_obj) },
    { MP_ROM_QSTR(MP_QSTR__detect_cyc),     MP_ROM_PTR(&whycon_detect_cyc_obj) },
    { MP_ROM_QSTR(MP_QSTR__set_concentric), MP_ROM_PTR(&whycon_set_concentric_obj) },
    { MP_ROM_QSTR(MP_QSTR__get_markers),    MP_ROM_PTR(&whycon_get_markers_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_whycon_globals, sentai_whycon_globals_table);

const mp_obj_module_t sentai_whycon_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_whycon_globals,
};

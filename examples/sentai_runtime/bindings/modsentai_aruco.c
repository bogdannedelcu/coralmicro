// modsentai_aruco.c — sentai.aruco MicroPython binding.
// OP-S6-W3-T5.  Per CLAUDE.md compute-in-C principle: this binding
// returns SMALL SCALARS only (marker_id, tvec, rvec, corners_px), never
// crosses an image buffer through the MP boundary.
//
// Surface:
//   sentai.aruco.init()                                    -> int rc
//   sentai.aruco.clear()                                   -> None
//   sentai.aruco.set_intrinsics(fx, fy, cx, cy)            -> None
//   sentai.aruco.set_marker_size(size_m)                   -> int rc
//   sentai.aruco.detect_from_camera()                      -> list[dict]
//                                                              (uses
//                                                              sentai.camera.grab_gray())
//   sentai.aruco.get_latest()                              -> list[dict]
//   sentai.aruco.get_stats()                               -> dict
//   sentai.aruco.rvec_to_R(rvec)                           -> 9-tuple
//   sentai.aruco.R_to_rvec(R)                              -> 3-tuple
//
// Each marker dict has fields: marker_id, hamming, tvec_cam (3-tuple),
// rvec_cam (3-tuple), corners_px (8-tuple), reproj_err_px, detect_us,
// src_ts_ms, frame_seq.
//
// `detect_from_camera()` is preferred to avoid passing the gray buffer
// through MP at all: it asks the camera_bridge_recv task (SIM) /
// camera ISR pipeline (ARM) for the latest frame pointer + dims and
// hands them directly to the C detector, never crossing the binding.

#include "sentai_aruco.h"

#include <stdint.h>
#include <string.h>

// Camera frame retrieval differs ARM vs SIM; both expose the same
// "grab gray pointer + dims" hook to other C subsystems.  We use the
// existing sentai_camera_grab_gray_zerocopy contract — if present —
// to read the frame buffer in C without an MP copy.  If the symbol
// is absent (early bring-up) we degrade to "no frame" gracefully.
extern int sentai_camera_grab_gray_zerocopy(const uint8_t** out_buf,
                                             int* out_w, int* out_h,
                                             uint32_t* out_seq,
                                             uint32_t* out_ts_ms)
    __attribute__((weak));

// =======================================================================
// Helpers
// =======================================================================

static mp_obj_t aruco_tvec_to_tuple(const float v[3]) {
    mp_obj_t items[3] = {
        mp_obj_new_float(v[0]),
        mp_obj_new_float(v[1]),
        mp_obj_new_float(v[2]),
    };
    return mp_obj_new_tuple(3, items);
}

static mp_obj_t aruco_corners_to_tuple(const float c[8]) {
    mp_obj_t items[8];
    for (int i = 0; i < 8; ++i) items[i] = mp_obj_new_float(c[i]);
    return mp_obj_new_tuple(8, items);
}

static mp_obj_t aruco_R_to_tuple(const float R[9]) {
    mp_obj_t items[9];
    for (int i = 0; i < 9; ++i) items[i] = mp_obj_new_float(R[i]);
    return mp_obj_new_tuple(9, items);
}

static int aruco_parse_R9(mp_obj_t obj, float out9[9]) {
    size_t len;
    mp_obj_t* items;
    mp_obj_get_array(obj, &len, &items);
    if (len == 9) {
        for (int i = 0; i < 9; ++i) out9[i] = mp_obj_get_float(items[i]);
        return 0;
    }
    if (len == 3) {
        for (int i = 0; i < 3; ++i) {
            size_t inner_len;
            mp_obj_t* inner;
            mp_obj_get_array(items[i], &inner_len, &inner);
            if (inner_len < 3) return -1;
            for (int j = 0; j < 3; ++j) {
                out9[i*3 + j] = mp_obj_get_float(inner[j]);
            }
        }
        return 0;
    }
    return -1;
}

static int aruco_parse_vec3(mp_obj_t obj, float* out3) {
    size_t len;
    mp_obj_t* items;
    mp_obj_get_array(obj, &len, &items);
    if (len < 3) return -1;
    for (int i = 0; i < 3; ++i) out3[i] = mp_obj_get_float(items[i]);
    return 0;
}

static mp_obj_t aruco_marker_to_dict(const sentai_aruco_marker_t* m) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(9));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_marker_id),
                       mp_obj_new_int(m->marker_id));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_hamming),
                       mp_obj_new_int(m->hamming));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_tvec_cam),
                       aruco_tvec_to_tuple(m->tvec_cam));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rvec_cam),
                       aruco_tvec_to_tuple(m->rvec_cam));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_corners_px),
                       aruco_corners_to_tuple(m->corners_px));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_reproj_err_px),
                       mp_obj_new_float(m->reproj_err_px));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_detect_us),
                       mp_obj_new_int(m->detect_us));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_src_ts_ms),
                       mp_obj_new_int(m->src_ts_ms));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frame_seq),
                       mp_obj_new_int(m->frame_seq));
    return MP_OBJ_FROM_PTR(d);
}

static mp_obj_t aruco_markers_to_list(const sentai_aruco_marker_t* arr, int n) {
    mp_obj_t list = mp_obj_new_list(0, NULL);
    for (int i = 0; i < n; ++i) {
        mp_obj_list_append(list, aruco_marker_to_dict(&arr[i]));
    }
    return list;
}

// =======================================================================
// API
// =======================================================================

static mp_obj_t aruco_init(void) {
    return mp_obj_new_int(sentai_aruco_init());
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_init_obj, aruco_init);

static mp_obj_t aruco_clear(void) {
    sentai_aruco_clear();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_clear_obj, aruco_clear);

static mp_obj_t aruco_set_intrinsics(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    sentai_aruco_set_intrinsics(
        mp_obj_get_float(args[0]),
        mp_obj_get_float(args[1]),
        mp_obj_get_float(args[2]),
        mp_obj_get_float(args[3]));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aruco_set_intrinsics_obj, 4, 4,
                                            aruco_set_intrinsics);

static mp_obj_t aruco_set_marker_size(mp_obj_t size_obj) {
    return mp_obj_new_int(sentai_aruco_set_marker_size(
                            mp_obj_get_float(size_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aruco_set_marker_size_obj, aruco_set_marker_size);

static mp_obj_t aruco_detect_from_camera(void) {
    if (!sentai_camera_grab_gray_zerocopy) {
        return mp_obj_new_list(0, NULL);
    }
    const uint8_t* buf = NULL;
    int w = 0, h = 0;
    uint32_t seq = 0, ts = 0;
    if (sentai_camera_grab_gray_zerocopy(&buf, &w, &h, &seq, &ts) != 0) {
        return mp_obj_new_list(0, NULL);
    }
    sentai_aruco_marker_t arr[SENTAI_ARUCO_MAX_MARKERS];
    int n = sentai_aruco_detect(buf, w, h, seq, ts, arr,
                                  SENTAI_ARUCO_MAX_MARKERS);
    if (n < 0) n = 0;
    return aruco_markers_to_list(arr, n);
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_detect_from_camera_obj,
                                   aruco_detect_from_camera);

static mp_obj_t aruco_get_latest(void) {
    sentai_aruco_marker_t arr[SENTAI_ARUCO_MAX_MARKERS];
    int n = sentai_aruco_get_latest(arr, SENTAI_ARUCO_MAX_MARKERS);
    return aruco_markers_to_list(arr, n);
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_get_latest_obj, aruco_get_latest);

static mp_obj_t aruco_get_stats(void) {
    sentai_aruco_stats_t s;
    sentai_aruco_get_stats(&s);
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(7));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_total),
                       mp_obj_new_int(s.frames_total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_frames_with_detect),
                       mp_obj_new_int(s.frames_with_detect));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_markers_total),
                       mp_obj_new_int(s.markers_total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_overflow_total),
                       mp_obj_new_int(s.overflow_total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rejected_dict_total),
                       mp_obj_new_int(s.rejected_dict_total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_rejected_reproj_total),
                       mp_obj_new_int(s.rejected_reproj_total));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_last_detect_us),
                       mp_obj_new_int(s.last_detect_us));
    return MP_OBJ_FROM_PTR(d);
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_get_stats_obj, aruco_get_stats);

static mp_obj_t aruco_rvec_to_R_(mp_obj_t rvec_obj) {
    float r[3];
    if (aruco_parse_vec3(rvec_obj, r) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("rvec must be 3 floats"));
    }
    float R[9];
    sentai_aruco_rvec_to_R(r, R);
    return aruco_R_to_tuple(R);
}
static MP_DEFINE_CONST_FUN_OBJ_1(aruco_rvec_to_R_obj, aruco_rvec_to_R_);

static mp_obj_t aruco_R_to_rvec_(mp_obj_t R_obj) {
    float R[9];
    if (aruco_parse_R9(R_obj, R) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("R must be 9 floats or 3x3"));
    }
    float r[3];
    sentai_aruco_R_to_rvec(R, r);
    return aruco_tvec_to_tuple(r);
}
static MP_DEFINE_CONST_FUN_OBJ_1(aruco_R_to_rvec_obj, aruco_R_to_rvec_);

// Test helper used by EXP-s159 smoke: synthesize a 320x240 frame with
// one marker + run detection, all in C (no MP image buffer).  Returns
// the number of markers detected; caller reads details via get_latest.
extern int sentai_aruco_test_synth_and_detect(int marker_id,
                                               int side_px,
                                               int rotation_cw);

static mp_obj_t aruco_test_synth_and_detect_(size_t n_args,
                                              const mp_obj_t* args) {
    const int marker_id  = mp_obj_get_int(args[0]);
    const int side_px    = (n_args >= 2) ? mp_obj_get_int(args[1]) : 96;
    const int rotation   = (n_args >= 3) ? mp_obj_get_int(args[2]) : 0;
    const int n = sentai_aruco_test_synth_and_detect(marker_id, side_px,
                                                      rotation);
    return mp_obj_new_int(n);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(aruco_test_synth_obj, 1, 3,
                                            aruco_test_synth_and_detect_);

// s175 PnP rotation invariance test — load PGM file from disk + detect.
extern int sentai_aruco_detect_pgm_file(const char* path);
static mp_obj_t aruco_test_pgm_(mp_obj_t path_obj) {
    const char* path = mp_obj_str_get_str(path_obj);
    return mp_obj_new_int(sentai_aruco_detect_pgm_file(path));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aruco_test_pgm_obj, aruco_test_pgm_);

// OP-S10-W14-T18-T verify: runs scalar reference + optimized adaptive
// threshold on a deterministic synth gray frame, returns the count of
// differing s_binary bytes.  0 == math-identical (expected).
extern int sentai_aruco_adaptive_threshold_verify(int block);
static mp_obj_t aruco_verify_threshold_(mp_obj_t block_obj) {
    const int block = mp_obj_get_int(block_obj);
    return mp_obj_new_int(sentai_aruco_adaptive_threshold_verify(block));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aruco_verify_threshold_obj,
                                  aruco_verify_threshold_);

extern uint32_t sentai_aruco_thresh_old_cyc(void);
extern uint32_t sentai_aruco_thresh_new_cyc(void);
static mp_obj_t aruco_thresh_cycles_(void) {
    mp_obj_t tup[2] = {
        mp_obj_new_int_from_uint(sentai_aruco_thresh_old_cyc()),
        mp_obj_new_int_from_uint(sentai_aruco_thresh_new_cyc()),
    };
    return mp_obj_new_tuple(2, tup);
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_thresh_cycles_obj, aruco_thresh_cycles_);

// OP-S10-W16 ablation: run scalar threshold on M7 with D-cache OFF.
// Validates that cache thrash on 309 KB integral image is the root
// cause of M7's per-pixel inefficiency vs M4.
extern uint32_t sentai_aruco_thresh_nocache(int block);
static mp_obj_t aruco_thresh_nocache_(mp_obj_t block_obj) {
    return mp_obj_new_int_from_uint(sentai_aruco_thresh_nocache(mp_obj_get_int(block_obj)));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aruco_thresh_nocache_obj, aruco_thresh_nocache_);

// OP-S10-W16-T3.8 — end-to-end detect() timing + rolling-path toggle.
// Lets us measure the savings from rolling-integral threshold against
// the full sentai_aruco_detect() pipeline (not just the threshold stage).
extern void sentai_aruco_set_use_rolling(int on);
extern int  sentai_aruco_get_use_rolling(void);
extern uint32_t sentai_aruco_detect_cyc_last(void);
static mp_obj_t aruco_use_rolling_(mp_obj_t on_obj) {
    const int prev = sentai_aruco_get_use_rolling();
    sentai_aruco_set_use_rolling(mp_obj_get_int(on_obj));
    return mp_obj_new_int(prev);
}
static MP_DEFINE_CONST_FUN_OBJ_1(aruco_use_rolling_obj, aruco_use_rolling_);
static mp_obj_t aruco_detect_cyc_(void) {
    return mp_obj_new_int_from_uint(sentai_aruco_detect_cyc_last());
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_detect_cyc_obj, aruco_detect_cyc_);

// OP-S10-W16-T3.8 — rolling-integral threshold verify + bench.
// Eliminates the 309 KB integral image (uses 2.6 KB OCRAM scratch instead).
// Returns byte-mismatch count vs scalar reference; cycle count for the
// rolling kernel readable via _thresh_rolling_cyc().
extern int sentai_aruco_thresh_rolling_verify(int block);
extern uint32_t sentai_aruco_thresh_rolling_cyc(void);
static mp_obj_t aruco_thresh_rolling_verify_(mp_obj_t block_obj) {
    const int block = mp_obj_get_int(block_obj);
    return mp_obj_new_int(sentai_aruco_thresh_rolling_verify(block));
}
static MP_DEFINE_CONST_FUN_OBJ_1(aruco_thresh_rolling_verify_obj,
                                  aruco_thresh_rolling_verify_);
static mp_obj_t aruco_thresh_rolling_cyc_(void) {
    return mp_obj_new_int_from_uint(sentai_aruco_thresh_rolling_cyc());
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_thresh_rolling_cyc_obj,
                                  aruco_thresh_rolling_cyc_);

// OP-S10-W17-T6 — per-stage cycle breakdown for apples-to-apples
// comparison vs WhyCon `_stage_cyc5()`.  Returns 5-tuple of cycle
// counts accumulated during the last sentai_aruco_detect() call:
//   (t_thresh, t_flood, t_quad, t_decode, t_pnp).
extern void sentai_aruco_stage_cyc(uint32_t* t_thresh, uint32_t* t_flood,
                                     uint32_t* t_quad, uint32_t* t_decode,
                                     uint32_t* t_pnp);
static mp_obj_t aruco_stage_cyc_(void) {
    uint32_t t_thr = 0, t_fl = 0, t_q = 0, t_dec = 0, t_pnp = 0;
    sentai_aruco_stage_cyc(&t_thr, &t_fl, &t_q, &t_dec, &t_pnp);
    mp_obj_t t[5] = {
        mp_obj_new_int_from_uint(t_thr),
        mp_obj_new_int_from_uint(t_fl),
        mp_obj_new_int_from_uint(t_q),
        mp_obj_new_int_from_uint(t_dec),
        mp_obj_new_int_from_uint(t_pnp),
    };
    return mp_obj_new_tuple(5, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(aruco_stage_cyc_obj, aruco_stage_cyc_);

// =======================================================================
// Module table
// =======================================================================

static const mp_rom_map_elem_t sentai_aruco_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),         MP_ROM_QSTR(MP_QSTR_aruco) },
    { MP_ROM_QSTR(MP_QSTR_init),             MP_ROM_PTR(&aruco_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),            MP_ROM_PTR(&aruco_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_intrinsics),   MP_ROM_PTR(&aruco_set_intrinsics_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_marker_size),  MP_ROM_PTR(&aruco_set_marker_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_detect_from_camera),
                                              MP_ROM_PTR(&aruco_detect_from_camera_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_latest),       MP_ROM_PTR(&aruco_get_latest_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_stats),        MP_ROM_PTR(&aruco_get_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_rvec_to_R),        MP_ROM_PTR(&aruco_rvec_to_R_obj) },
    { MP_ROM_QSTR(MP_QSTR_R_to_rvec),        MP_ROM_PTR(&aruco_R_to_rvec_obj) },
    { MP_ROM_QSTR(MP_QSTR__test_synth_and_detect),
                                              MP_ROM_PTR(&aruco_test_synth_obj) },
    { MP_ROM_QSTR(MP_QSTR__test_pgm),         MP_ROM_PTR(&aruco_test_pgm_obj) },
    { MP_ROM_QSTR(MP_QSTR__verify_threshold), MP_ROM_PTR(&aruco_verify_threshold_obj) },
    { MP_ROM_QSTR(MP_QSTR__thresh_cycles),    MP_ROM_PTR(&aruco_thresh_cycles_obj) },
    { MP_ROM_QSTR(MP_QSTR__thresh_nocache),   MP_ROM_PTR(&aruco_thresh_nocache_obj) },
    { MP_ROM_QSTR(MP_QSTR__thresh_rolling_verify),
                                              MP_ROM_PTR(&aruco_thresh_rolling_verify_obj) },
    { MP_ROM_QSTR(MP_QSTR__thresh_rolling_cyc),
                                              MP_ROM_PTR(&aruco_thresh_rolling_cyc_obj) },
    { MP_ROM_QSTR(MP_QSTR__use_rolling),      MP_ROM_PTR(&aruco_use_rolling_obj) },
    { MP_ROM_QSTR(MP_QSTR__detect_cyc),       MP_ROM_PTR(&aruco_detect_cyc_obj) },
    { MP_ROM_QSTR(MP_QSTR__stage_cyc),        MP_ROM_PTR(&aruco_stage_cyc_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_aruco_globals, sentai_aruco_globals_table);

const mp_obj_module_t sentai_aruco_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_aruco_globals,
};

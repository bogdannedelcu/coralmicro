// modsentai_calib.c — sentai.calib MicroPython binding.
//
// ObjectsPlan OP-S6-W1.  Wraps sentai_calib.{h,cc} — Kabsch 3D
// Procrustes camera-to-body extrinsic auto-calibration.
//
// Surface:
//   sentai.calib.init()                       -> None  (load persisted)
//   sentai.calib.clear()                      -> None  (reset to default)
//   sentai.calib.run_kabsch(samples,
//                            persisted_R=None) -> (R 9-tuple, quality dict)
//   sentai.calib.commit_R(R, cam_offset_B=None) -> int (0 ok, -1 invalid)
//   sentai.calib.save()                       -> bool
//   sentai.calib.load()                       -> bool
//   sentai.calib.get_R_cam_to_body()          -> 9-tuple
//   sentai.calib.get_cam_offset_B()           -> 3-tuple
//   sentai.calib.is_calibrated()              -> bool
//   sentai.calib.rotation_angle_deg(R1, R2)   -> float
//
// `samples` is a list of (tvec_cam[3], marker_W[3], drone_W[3], yaw_rad).
//
// #include'd from modsentai.c (ARM) AND sim/modsentai_sim.c (SIM); the
// underlying C++ module compiles identically on both targets thanks to
// the SENTAI_HAVE_FXUSER ifdef shim in sentai_calib.cc.

#include "sentai_calib.h"
#include "sentai_calib_orientation_task.h"
#include "sentai_calib_bringup.h"

#include <stdint.h>
#include <string.h>

// ---- Helpers -----------------------------------------------------------

static int calib_parse_vec3(mp_obj_t obj, float* out3) {
    size_t len;
    mp_obj_t* items;
    mp_obj_get_array(obj, &len, &items);
    if (len < 3) return -1;
    for (int i = 0; i < 3; ++i) out3[i] = mp_obj_get_float(items[i]);
    return 0;
}

static int calib_parse_R9(mp_obj_t obj, float out9[9]) {
    size_t len;
    mp_obj_t* items;
    mp_obj_get_array(obj, &len, &items);
    if (len == 9) {
        for (int i = 0; i < 9; ++i) out9[i] = mp_obj_get_float(items[i]);
        return 0;
    }
    if (len == 3) {
        // Accept list of 3 rows (each a 3-tuple) — matches the Python
        // host-side convention.
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

static mp_obj_t calib_R_to_tuple(const float R[9]) {
    mp_obj_t items[9];
    for (int i = 0; i < 9; ++i) items[i] = mp_obj_new_float(R[i]);
    return mp_obj_new_tuple(9, items);
}

static mp_obj_t calib_vec3_to_tuple(const float v[3]) {
    mp_obj_t items[3] = {
        mp_obj_new_float(v[0]),
        mp_obj_new_float(v[1]),
        mp_obj_new_float(v[2]),
    };
    return mp_obj_new_tuple(3, items);
}

static mp_obj_t calib_quality_to_dict(const sentai_calib_quality_t* q) {
    mp_obj_dict_t* d = MP_OBJ_TO_PTR(mp_obj_new_dict(7));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_n_samples),
                       mp_obj_new_int(q->n_samples));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_det_R),
                       mp_obj_new_float(q->det_R));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_mean_residual_deg),
                       mp_obj_new_float(q->mean_residual_deg));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_max_residual_deg),
                       mp_obj_new_float(q->max_residual_deg));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_drift_from_persisted_deg),
                       mp_obj_new_float(q->drift_from_persisted_deg));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_accepted),
                       mp_obj_new_bool(q->accepted));
    mp_obj_dict_store(d, MP_ROM_QSTR(MP_QSTR_reject_code),
                       mp_obj_new_int(q->reject_code));
    return MP_OBJ_FROM_PTR(d);
}

// ===================== init / clear =====================

static mp_obj_t calib_init(void) {
    sentai_calib_init();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_init_obj, calib_init);

static mp_obj_t calib_clear(void) {
    sentai_calib_clear();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_clear_obj, calib_clear);

// ===================== run_kabsch(samples, persisted_R=None) =====================

static mp_obj_t calib_run_kabsch(size_t n_args, const mp_obj_t* args) {
    // args[0]: samples list
    // args[1]: optional persisted_R (None or 9-tuple/3x3)
    size_t   n_samples;
    mp_obj_t* sample_items;
    mp_obj_get_array(args[0], &n_samples, &sample_items);
    if (n_samples > SENTAI_CALIB_SAMPLES_MAX) {
        mp_raise_ValueError(MP_ERROR_TEXT("too many samples"));
    }

    sentai_calib_sample_t samples[SENTAI_CALIB_SAMPLES_MAX];
    for (size_t i = 0; i < n_samples; ++i) {
        size_t   tlen;
        mp_obj_t* tup;
        mp_obj_get_array(sample_items[i], &tlen, &tup);
        if (tlen < 4) {
            mp_raise_ValueError(
                MP_ERROR_TEXT("sample must be (tvec, marker, drone, yaw)"));
        }
        if (calib_parse_vec3(tup[0], samples[i].tvec_cam) != 0 ||
            calib_parse_vec3(tup[1], samples[i].marker_W) != 0 ||
            calib_parse_vec3(tup[2], samples[i].drone_W ) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("sample vec3 must have 3 floats"));
        }
        samples[i].yaw_rad = mp_obj_get_float(tup[3]);
    }

    float persisted_R[9];
    const float* persisted_ptr = NULL;
    if (n_args >= 2 && args[1] != mp_const_none) {
        if (calib_parse_R9(args[1], persisted_R) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("persisted_R must be 9 floats"));
        }
        persisted_ptr = persisted_R;
    }

    float R_out[9];
    sentai_calib_quality_t q_out;
    int rc = sentai_calib_run_kabsch(samples, (int)n_samples,
                                      persisted_ptr, R_out, &q_out);
    (void)rc;  // negative rc still returns a defined R + quality with reject_code set

    mp_obj_t result[2] = {
        calib_R_to_tuple(R_out),
        calib_quality_to_dict(&q_out),
    };
    return mp_obj_new_tuple(2, result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calib_run_kabsch_obj, 1, 2,
                                            calib_run_kabsch);

// ===================== commit_R(R, cam_offset_B=None) =====================

static mp_obj_t calib_commit_R(size_t n_args, const mp_obj_t* args) {
    float R[9];
    if (calib_parse_R9(args[0], R) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("R must be 9 floats"));
    }
    float off[3];
    const float* off_ptr = NULL;
    if (n_args >= 2 && args[1] != mp_const_none) {
        if (calib_parse_vec3(args[1], off) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("cam_offset_B must be 3 floats"));
        }
        off_ptr = off;
    }
    int rc = sentai_calib_commit_R(R, off_ptr);
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calib_commit_R_obj, 1, 2,
                                            calib_commit_R);

// ===================== save / load =====================

static mp_obj_t calib_save(void) {
    return mp_obj_new_bool(sentai_calib_save());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_save_obj, calib_save);

static mp_obj_t calib_save_contract(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    size_t status_len = 0;
    const char* status = mp_obj_str_get_data(args[0], &status_len);
    if (status_len == 0 || status_len >= 32) {
        mp_raise_ValueError(MP_ERROR_TEXT("bad status"));
    }
    char status_buf[32];
    memcpy(status_buf, status, status_len);
    status_buf[status_len] = '\0';
    const int accepted = mp_obj_is_true(args[1]) ? 1 : 0;
    const float roll_sign = (float)mp_obj_get_float(args[2]);
    const float pitch_sign = (float)mp_obj_get_float(args[4]);

    size_t rv_len = 0, pv_len = 0;
    mp_obj_t* rv_items = NULL;
    mp_obj_t* pv_items = NULL;
    mp_obj_get_array(args[3], &rv_len, &rv_items);
    mp_obj_get_array(args[5], &pv_len, &pv_items);
    if (rv_len != 2 || pv_len != 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("axis vectors must have 2 floats"));
    }
    float rv[2] = {
        (float)mp_obj_get_float(rv_items[0]),
        (float)mp_obj_get_float(rv_items[1]),
    };
    float pv[2] = {
        (float)mp_obj_get_float(pv_items[0]),
        (float)mp_obj_get_float(pv_items[1]),
    };
    return mp_obj_new_bool(sentai_calib_save_contract(
        status_buf, accepted, roll_sign, rv, pitch_sign, pv));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calib_save_contract_obj,
                                            6, 6, calib_save_contract);

static mp_obj_t calib_load(void) {
    return mp_obj_new_bool(sentai_calib_load());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_load_obj, calib_load);

// ===================== getters =====================

static mp_obj_t calib_get_R(void) {
    return calib_R_to_tuple(sentai_calib_get_R_cam_to_body());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_get_R_obj, calib_get_R);

static mp_obj_t calib_get_off(void) {
    return calib_vec3_to_tuple(sentai_calib_get_cam_offset_B());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_get_off_obj, calib_get_off);

static mp_obj_t calib_is_calib(void) {
    return mp_obj_new_bool(sentai_calib_is_calibrated());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_is_calib_obj, calib_is_calib);

static mp_obj_t calib_ini_status_tuple(void) {
    sentai_calib_ini_status_t st;
    if (!sentai_calib_get_ini_status(&st)) {
        return mp_const_none;
    }
    mp_obj_t t[11] = {
        mp_obj_new_int(st.present),
        mp_obj_new_int(st.schema_ok),
        mp_obj_new_int(st.accepted_ok),
        mp_obj_new_int(st.status_ok),
        mp_obj_new_int(st.task_ok),
        mp_obj_new_int(st.layout_ok),
        mp_obj_new_int(st.required_ok),
        mp_obj_new_int(st.strict_lines_ok),
        mp_obj_new_int_from_uint(st.missing_mask),
        mp_obj_new_str(st.status, strlen(st.status)),
        mp_obj_new_str(st.layout_id, strlen(st.layout_id)),
    };
    return mp_obj_new_tuple(11, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_ini_status_tuple_obj,
                                  calib_ini_status_tuple);

static mp_obj_t calib_get_extpos_signs(void) {
    const float* s = sentai_calib_get_extpos_signs();
    mp_obj_t t[3] = {
        mp_obj_new_float(s[0]),
        mp_obj_new_float(s[1]),
        mp_obj_new_float(s[2]),
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_get_extpos_signs_obj,
                                  calib_get_extpos_signs);

static mp_obj_t calib_get_axis_seed_tuple(void) {
    float rv[2], pv[2], rs = 0.0f, ps = 0.0f;
    const int ok = sentai_calib_get_axis_seed(rv, &rs, pv, &ps);
    mp_obj_t t[7] = {
        mp_obj_new_int(ok),
        mp_obj_new_float(rs),
        mp_obj_new_float(rv[0]),
        mp_obj_new_float(rv[1]),
        mp_obj_new_float(ps),
        mp_obj_new_float(pv[0]),
        mp_obj_new_float(pv[1]),
    };
    return mp_obj_new_tuple(7, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_get_axis_seed_tuple_obj,
                                  calib_get_axis_seed_tuple);

static mp_obj_t calib_defaults_tuple(void) {
    sentai_calib_defaults_t d;
    if (!sentai_calib_get_defaults(&d)) return mp_const_none;
    mp_obj_t t[9] = {
        mp_obj_new_int(d.img_w),
        mp_obj_new_int(d.img_h),
        mp_obj_new_float(d.fx),
        mp_obj_new_float(d.fy),
        mp_obj_new_float(d.cx),
        mp_obj_new_float(d.cy),
        mp_obj_new_float(d.marker_diameter_m),
        mp_obj_new_int(d.marker_world_count),
        mp_obj_new_float(d.full_vis_margin_px),
    };
    return mp_obj_new_tuple(9, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_defaults_tuple_obj,
                                  calib_defaults_tuple);

static mp_obj_t calib_limits_tuple(void) {
    sentai_calib_limits_t d;
    if (!sentai_calib_get_limits(&d)) return mp_const_none;
    mp_obj_t t[10] = {
        mp_obj_new_int(d.min_full_markers),
        mp_obj_new_int(d.acq_full_markers),
        mp_obj_new_float(d.marker_avg_full_lock),
        mp_obj_new_int(d.marker_count_avg_window),
        mp_obj_new_float(d.min_lock_radius_px),
        mp_obj_new_int(d.axis_hard_min_full_markers),
        mp_obj_new_float(d.axis_min_avg_full_markers),
        mp_obj_new_float(d.response_min_px),
        mp_obj_new_float(d.axis_dominance_ratio_min),
        mp_obj_new_float(d.axis_orthogonal_dot_max_norm),
    };
    return mp_obj_new_tuple(10, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_limits_tuple_obj,
                                  calib_limits_tuple);

static mp_obj_t calib_setup_defaults(void) {
    return mp_obj_new_int(sentai_calib_setup_defaults());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_setup_defaults_obj,
                                  calib_setup_defaults);

// LEGACY MP calibration helper family.
//
// These tuple-style primitives were used while s205 still orchestrated the
// calibration controller from MicroPython.  The active s205 path now uses the
// orientation_* task APIs below; keep this block only as temporary compatibility
// surface for older experiments while we delete callers deliberately.
static mp_obj_t calib_marker_avg_lock_ok_tuple(size_t n_args,
                                               const mp_obj_t* args) {
    const int ready = mp_obj_is_true(args[0]) ? 1 : 0;
    const float avg_full = (float)mp_obj_get_float(args[1]);
    const float threshold = (n_args >= 3) ?
        (float)mp_obj_get_float(args[2]) : -1.0f;
    return mp_obj_new_bool(sentai_calib_marker_avg_lock_ok(
        ready, avg_full, threshold));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_marker_avg_lock_ok_tuple_obj, 2, 3,
    calib_marker_avg_lock_ok_tuple);

static mp_obj_t calib_marker_avg_unsafe_tuple(size_t n_args,
                                              const mp_obj_t* args) {
    const int ready = mp_obj_is_true(args[0]) ? 1 : 0;
    const float avg_full = (float)mp_obj_get_float(args[1]);
    const float threshold = (n_args >= 3) ?
        (float)mp_obj_get_float(args[2]) : -1.0f;
    return mp_obj_new_bool(sentai_calib_marker_avg_unsafe(
        ready, avg_full, threshold));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_marker_avg_unsafe_tuple_obj, 2, 3,
    calib_marker_avg_unsafe_tuple);

static mp_obj_t calib_feature_has_lock_tuple(size_t n_args,
                                             const mp_obj_t* args) {
    (void)n_args;
    return mp_obj_new_bool(sentai_calib_feature_has_lock(
        mp_obj_get_int(args[0]),
        (float)mp_obj_get_float(args[1]),
        mp_obj_is_true(args[2]) ? 1 : 0,
        mp_obj_is_true(args[3]) ? 1 : 0,
        (float)mp_obj_get_float(args[4])));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_feature_has_lock_tuple_obj, 5, 5, calib_feature_has_lock_tuple);

static mp_obj_t calib_marker_lock_ok_tuple(size_t n_args,
                                           const mp_obj_t* args) {
    (void)n_args;
    return mp_obj_new_bool(sentai_calib_marker_lock_ok(
        mp_obj_get_int(args[0]), (float)mp_obj_get_float(args[1])));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_marker_lock_ok_tuple_obj, 2, 2, calib_marker_lock_ok_tuple);

static mp_obj_t calib_score_axis_candidate(size_t n_args,
                                           const mp_obj_t* args) {
    (void)n_args;
    size_t roll_axis_len = 0, pitch_axis_len = 0;
    const char* roll_axis = mp_obj_str_get_data(args[0], &roll_axis_len);
    const int roll_sign = mp_obj_get_int(args[1]);
    const char* pitch_axis = mp_obj_str_get_data(args[2], &pitch_axis_len);
    const int pitch_sign = mp_obj_get_int(args[3]);
    if (roll_axis_len == 0 || pitch_axis_len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("axis must be x or y"));
    }
    sentai_calib_axis_candidate_t c;
    if (!sentai_calib_score_axis_candidate(roll_axis, roll_sign,
                                           pitch_axis, pitch_sign, &c)) {
        return mp_const_none;
    }
    mp_obj_t r_items[9];
    for (int i = 0; i < 9; ++i) {
        r_items[i] = mp_obj_new_float(c.R_cam_to_body[i]);
    }
    mp_obj_t t[7] = {
        mp_obj_new_bool(c.ok),
        mp_obj_new_int(c.best_idx),
        mp_obj_new_float(c.best_score),
        mp_obj_new_float(c.second_score),
        mp_obj_new_float(c.margin),
        mp_obj_new_float(c.det),
        mp_obj_new_tuple(9, r_items),
    };
    return mp_obj_new_tuple(7, t);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calib_score_axis_candidate_obj,
                                            4, 4, calib_score_axis_candidate);

static const char* calib_axis_name_(int axis_code) {
    return axis_code == 0 ? "x" : "y";
}

static mp_obj_t calib_expected_from_row_tuple(size_t n_args,
                                              const mp_obj_t* args) {
    (void)n_args;
    int axis_code = 0;
    int sign = 0;
    if (!sentai_calib_expected_from_row(
            (float)mp_obj_get_float(args[0]),
            (float)mp_obj_get_float(args[1]),
            (float)mp_obj_get_float(args[2]),
            &axis_code, &sign)) {
        return mp_const_none;
    }
    mp_obj_t t[2] = {
        mp_obj_new_str(calib_axis_name_(axis_code), 1),
        mp_obj_new_int(sign),
    };
    return mp_obj_new_tuple(2, t);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_expected_from_row_tuple_obj, 3, 3, calib_expected_from_row_tuple);

static mp_obj_t calib_axis_observation_from_delta_tuple(size_t n_args,
                                                        const mp_obj_t* args) {
    (void)n_args;
    int axis_code = 0;
    int sign = 0;
    float dominance = 0.0f;
    float strength = 0.0f;
    if (!sentai_calib_axis_observation_from_delta(
            (float)mp_obj_get_float(args[0]),
            (float)mp_obj_get_float(args[1]),
            &axis_code, &sign, &dominance, &strength)) {
        return mp_const_none;
    }
    mp_obj_t t[4] = {
        mp_obj_new_str(calib_axis_name_(axis_code), 1),
        mp_obj_new_int(sign),
        mp_obj_new_float(dominance),
        mp_obj_new_float(strength),
    };
    return mp_obj_new_tuple(4, t);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_axis_observation_from_delta_tuple_obj, 2, 2,
    calib_axis_observation_from_delta_tuple);

static mp_obj_t calib_sample_calib_observation_tuple(size_t n_args,
                                                     const mp_obj_t* args) {
    sentai_calib_defaults_t d;
    (void)sentai_calib_get_defaults(&d);
    int img_w = d.img_w;
    int img_h = d.img_h;
    float margin = d.full_vis_margin_px;
    if (n_args == 3) {
        img_w = mp_obj_get_int(args[0]);
        img_h = mp_obj_get_int(args[1]);
        margin = (float)mp_obj_get_float(args[2]);
    }
    SentaiMarkersObservation o;
    const int rc = sentai_calib_sample_observation(
        img_w, img_h, margin, &o);
    if (rc < -1) {
        // Keep the existing calib MP convention: negative n_raw means "no
        // usable runtime observation", not an exception that kills cleanup.
        o.n_raw = rc;
    }
    mp_obj_t t[15] = {
        mp_obj_new_int(o.n_raw),
        mp_obj_new_int(o.n_full),
        mp_obj_new_int(o.n_pose_valid),
        mp_obj_new_float(o.centroid_x),
        mp_obj_new_float(o.centroid_y),
        mp_obj_new_float(o.radius_mean_px),
        mp_obj_new_float(o.z_cam_mean_m),
        mp_obj_new_float(o.bbox_min_x),
        mp_obj_new_float(o.bbox_min_y),
        mp_obj_new_float(o.bbox_max_x),
        mp_obj_new_float(o.bbox_max_y),
        mp_obj_new_int_from_uint(o.frame_seq),
        mp_obj_new_int_from_uint(o.src_ts_ms),
        mp_obj_new_int(o.valid),
        mp_obj_new_int(o.backend),
    };
    return mp_obj_new_tuple(15, t);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_sample_calib_observation_tuple_obj, 0, 3,
    calib_sample_calib_observation_tuple);

static mp_obj_t calib_vertical_rate_thrust_tuple(size_t n_args,
                                                 const mp_obj_t* args) {
    (void)n_args;
    int thrust = 0;
    float z_prev_out = 0.0f;
    float vz_filt_out = 0.0f;
    const int ok = sentai_calib_vertical_rate_thrust(
        (float)mp_obj_get_float(args[0]),
        (float)mp_obj_get_float(args[1]),
        (float)mp_obj_get_float(args[2]),
        mp_obj_get_int(args[3]),
        (float)mp_obj_get_float(args[4]),
        (float)mp_obj_get_float(args[5]),
        (float)mp_obj_get_float(args[6]),
        (float)mp_obj_get_float(args[7]),
        mp_obj_get_int(args[8]),
        mp_obj_get_int(args[9]),
        &thrust,
        &z_prev_out,
        &vz_filt_out);
    if (!ok) return mp_const_none;
    mp_obj_t t[3] = {
        mp_obj_new_int(thrust),
        mp_obj_new_float(z_prev_out),
        mp_obj_new_float(vz_filt_out),
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_vertical_rate_thrust_tuple_obj, 10, 10,
    calib_vertical_rate_thrust_tuple);

static mp_obj_t calib_z_hold_thrust_tuple(size_t n_args,
                                          const mp_obj_t* args) {
    (void)n_args;
    int thrust = 0;
    float z_prev_out = 0.0f;
    float vz_filt_out = 0.0f;
    const int ok = sentai_calib_z_hold_thrust(
        (float)mp_obj_get_float(args[0]),
        (float)mp_obj_get_float(args[1]),
        (float)mp_obj_get_float(args[2]),
        (float)mp_obj_get_float(args[3]),
        mp_obj_get_int(args[4]),
        (float)mp_obj_get_float(args[5]),
        (float)mp_obj_get_float(args[6]),
        (float)mp_obj_get_float(args[7]),
        (float)mp_obj_get_float(args[8]),
        mp_obj_get_int(args[9]),
        mp_obj_get_int(args[10]),
        &thrust,
        &z_prev_out,
        &vz_filt_out);
    if (!ok) return mp_const_none;
    mp_obj_t t[3] = {
        mp_obj_new_int(thrust),
        mp_obj_new_float(z_prev_out),
        mp_obj_new_float(vz_filt_out),
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_z_hold_thrust_tuple_obj, 11, 11, calib_z_hold_thrust_tuple);

static mp_obj_t calib_orientation_task_start(size_t n_args,
                                             const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_task_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_task_start_obj, 0, 0,
    calib_orientation_task_start);

static mp_obj_t calib_orientation_arm_zero_start(size_t n_args,
                                                 const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_arm_zero_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_arm_zero_start_obj, 0, 0,
    calib_orientation_arm_zero_start);

static mp_obj_t calib_orientation_marker_acquisition_start(
        size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_marker_acquisition_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_marker_acquisition_start_obj, 0, 0,
    calib_orientation_marker_acquisition_start);

static mp_obj_t calib_orientation_post_lock_brake_start(
        size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_post_lock_brake_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_post_lock_brake_start_obj, 0, 0,
    calib_orientation_post_lock_brake_start);

static mp_obj_t calib_orientation_visual_z_hold_start(
        size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_visual_z_hold_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_visual_z_hold_start_obj, 0, 0,
    calib_orientation_visual_z_hold_start);

static mp_obj_t calib_orientation_axis_response_start(
        size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_axis_response_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_axis_response_start_obj, 0, 0,
    calib_orientation_axis_response_start);

static mp_obj_t calib_orientation_centroid_validation_start(
        size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_centroid_validation_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_centroid_validation_start_obj, 0, 0,
    calib_orientation_centroid_validation_start);

static mp_obj_t calib_orientation_score_candidate_from_axis(void) {
    return mp_obj_new_bool(sentai_calib_orientation_score_candidate_from_axis());
}
static MP_DEFINE_CONST_FUN_OBJ_0(
    calib_orientation_score_candidate_from_axis_obj,
    calib_orientation_score_candidate_from_axis);

static mp_obj_t calib_orientation_optical_axis_validate(void) {
    return mp_obj_new_bool(sentai_calib_orientation_optical_axis_validate());
}
static MP_DEFINE_CONST_FUN_OBJ_0(
    calib_orientation_optical_axis_validate_obj,
    calib_orientation_optical_axis_validate);

static mp_obj_t calib_orientation_final_candidate_validation_start(
        size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_final_candidate_validation_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_final_candidate_validation_start_obj, 0, 0,
    calib_orientation_final_candidate_validation_start);

static mp_obj_t calib_orientation_final_recenter_start(
        size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_final_recenter_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_final_recenter_start_obj, 0, 0,
    calib_orientation_final_recenter_start);

static mp_obj_t calib_orientation_manual_descend_start(size_t n_args,
                                                       const mp_obj_t *args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_manual_descend_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_manual_descend_start_obj, 0, 0,
    calib_orientation_manual_descend_start);

static mp_obj_t calib_orientation_center_hold_descend_start(
        size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    (void)args;
    return mp_obj_new_int(sentai_calib_orientation_center_hold_descend_start());
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_center_hold_descend_start_obj, 0, 0,
    calib_orientation_center_hold_descend_start);

static mp_obj_t calib_orientation_save_contract(size_t n_args,
                                                const mp_obj_t* args) {
    (void)n_args;
    size_t status_len = 0;
    const char* status = mp_obj_str_get_data(args[0], &status_len);
    (void)status_len;
    return mp_obj_new_bool(sentai_calib_orientation_save_contract(
        status, mp_obj_is_true(args[1]) ? 1 : 0));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    calib_orientation_save_contract_obj, 2, 2,
    calib_orientation_save_contract);

static mp_obj_t calib_orientation_emergency_stop(void) {
    return mp_obj_new_int(sentai_calib_orientation_emergency_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_orientation_emergency_stop_obj,
                                  calib_orientation_emergency_stop);

static mp_obj_t calib_orientation_task_stop(void) {
    return mp_obj_new_int(sentai_calib_orientation_task_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_orientation_task_stop_obj,
                                  calib_orientation_task_stop);

static mp_obj_t calib_orientation_task_is_done(void) {
    return mp_obj_new_bool(sentai_calib_orientation_task_is_done());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_orientation_task_is_done_obj,
                                  calib_orientation_task_is_done);

static mp_obj_t calib_orientation_result_tuple(void) {
    int ok = 0;
    int thrust = 0;
    int disarmed = 0;
    if (!sentai_calib_orientation_task_result(&ok, &thrust, &disarmed)) {
        return mp_const_none;
    }
    mp_obj_t t[3] = {
        mp_obj_new_bool(ok),
        mp_obj_new_int(thrust),
        mp_obj_new_bool(disarmed),
    };
    return mp_obj_new_tuple(3, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_orientation_result_tuple_obj,
                                  calib_orientation_result_tuple);

static mp_obj_t calib_orientation_status_tuple(void) {
    sentai_calib_orientation_status_t st;
    if (!sentai_calib_orientation_task_get_status(&st)) {
        return mp_const_none;
    }
    mp_obj_t t[183] = {
        mp_obj_new_int(st.phase),
        mp_obj_new_int(st.status),
        mp_obj_new_bool(st.started),
        mp_obj_new_bool(st.done),
        mp_obj_new_int(st.ok_full_ticks),
        mp_obj_new_int(st.samples),
        mp_obj_new_int(st.n_full_max),
        mp_obj_new_float(st.radius_max_px),
        mp_obj_new_float(st.z_cam_max_m),
        mp_obj_new_bool(st.feature_lock),
        mp_obj_new_int(st.arm_rc),
        mp_obj_new_int(st.arm_retry_rc),
        mp_obj_new_int(st.zero_packets),
        mp_obj_new_bool(st.acq_locked),
        mp_obj_new_int(st.acq_last_thrust_u16),
        mp_obj_new_int(st.acq_lock_streak),
        mp_obj_new_int(st.acq_n_full_max),
        mp_obj_new_float(st.acq_radius_max_px),
        mp_obj_new_float(st.acq_z_cam_min_m),
        mp_obj_new_float(st.acq_z_cam_max_m),
        mp_obj_new_float(st.acq_z_cam_last_m),
        mp_obj_new_int(st.acq_first_seen_tick),
        mp_obj_new_bool(st.post_ok),
        mp_obj_new_int(st.post_abort_code),
        mp_obj_new_int(st.post_valid_ticks),
        mp_obj_new_float(st.post_z_cam_min_m),
        mp_obj_new_float(st.post_z_cam_max_m),
        mp_obj_new_float(st.post_z_cam_last_m),
        mp_obj_new_float(st.post_vz_filt_m_s),
        mp_obj_new_int(st.post_thrust_last_u16),
        mp_obj_new_int(st.post_ok_vz_ticks),
        mp_obj_new_bool(st.zhold_ok),
        mp_obj_new_int(st.zhold_abort_code),
        mp_obj_new_bool(st.zhold_target_reached),
        mp_obj_new_bool(st.zhold_target_reached_last),
        mp_obj_new_bool(st.zhold_target_reached_peak),
        mp_obj_new_int(st.zhold_valid_ticks),
        mp_obj_new_int(st.zhold_ticks_requested),
        mp_obj_new_float(st.zhold_z_cam_min_m),
        mp_obj_new_float(st.zhold_z_cam_max_m),
        mp_obj_new_float(st.zhold_z_cam_last_m),
        mp_obj_new_float(st.zhold_vz_filt_m_s),
        mp_obj_new_int(st.zhold_thrust_min_u16),
        mp_obj_new_int(st.zhold_thrust_max_u16),
        mp_obj_new_int(st.zhold_thrust_last_u16),
        mp_obj_new_bool(st.axis_ok),
        mp_obj_new_int(st.axis_abort_code),
        mp_obj_new_int(st.axis_results_count),
        mp_obj_new_float(st.axis_pitch_comp_dx),
        mp_obj_new_float(st.axis_pitch_comp_dy),
        mp_obj_new_float(st.axis_roll_comp_dx),
        mp_obj_new_float(st.axis_roll_comp_dy),
        mp_obj_new_int(st.axis_pitch_dominant_axis_code),
        mp_obj_new_int(st.axis_pitch_dominant_sign),
        mp_obj_new_float(st.axis_pitch_dominance_ratio),
        mp_obj_new_float(st.axis_pitch_response_strength_px),
        mp_obj_new_bool(st.axis_pitch_sign_ok),
        mp_obj_new_int(st.axis_pitch_min_full_markers),
        mp_obj_new_float(st.axis_pitch_avg_full_markers),
        mp_obj_new_bool(st.axis_pitch_marker_lock_ok),
        mp_obj_new_float(st.axis_pitch_return_err_px),
        mp_obj_new_bool(st.axis_pitch_return_ok),
        mp_obj_new_float(st.axis_pitch_z_last_m),
        mp_obj_new_int(st.axis_roll_dominant_axis_code),
        mp_obj_new_int(st.axis_roll_dominant_sign),
        mp_obj_new_float(st.axis_roll_dominance_ratio),
        mp_obj_new_float(st.axis_roll_response_strength_px),
        mp_obj_new_bool(st.axis_roll_sign_ok),
        mp_obj_new_int(st.axis_roll_min_full_markers),
        mp_obj_new_float(st.axis_roll_avg_full_markers),
        mp_obj_new_bool(st.axis_roll_marker_lock_ok),
        mp_obj_new_float(st.axis_roll_return_err_px),
        mp_obj_new_bool(st.axis_roll_return_ok),
        mp_obj_new_float(st.axis_roll_z_last_m),
        mp_obj_new_bool(st.axis_orthogonality_present),
        mp_obj_new_float(st.axis_orthogonality_dot_norm),
        mp_obj_new_bool(st.axis_orthogonality_ok),
        mp_obj_new_int(st.axis_thrust_last_u16),
        mp_obj_new_bool(st.centroid_ok),
        mp_obj_new_int(st.centroid_abort_code),
        mp_obj_new_int(st.centroid_ticks_done),
        mp_obj_new_int(st.centroid_n_full_min),
        mp_obj_new_float(st.centroid_avg_full_markers),
        mp_obj_new_bool(st.centroid_marker_lock_ok),
        mp_obj_new_float(st.centroid_initial_err_px),
        mp_obj_new_float(st.centroid_final_err_px),
        mp_obj_new_float(st.centroid_min_err_px),
        mp_obj_new_float(st.centroid_max_err_px),
        mp_obj_new_float(st.centroid_improvement_px),
        mp_obj_new_int(st.centroid_thrust_last_u16),
        mp_obj_new_float(st.centroid_first_roll_deg),
        mp_obj_new_float(st.centroid_first_pitch_deg),
        mp_obj_new_float(st.centroid_first_err_px),
        mp_obj_new_float(st.centroid_last_roll_deg),
        mp_obj_new_float(st.centroid_last_pitch_deg),
        mp_obj_new_float(st.centroid_last_err_px),
        mp_obj_new_bool(st.final_val_ok),
        mp_obj_new_int(st.final_val_abort_code),
        mp_obj_new_int(st.final_val_thrust_last_u16),
        mp_obj_new_bool(st.final_pitch_ok),
        mp_obj_new_int(st.final_pitch_attempts),
        mp_obj_new_float(st.final_pitch_pulse_deg),
        mp_obj_new_int(st.final_pitch_expected_axis_code),
        mp_obj_new_int(st.final_pitch_expected_sign),
        mp_obj_new_int(st.final_pitch_observed_axis_code),
        mp_obj_new_int(st.final_pitch_observed_sign),
        mp_obj_new_float(st.final_pitch_dominance_ratio),
        mp_obj_new_float(st.final_pitch_response_strength_px),
        mp_obj_new_float(st.final_pitch_noise_gate_px),
        mp_obj_new_float(st.final_pitch_comp_dx),
        mp_obj_new_float(st.final_pitch_comp_dy),
        mp_obj_new_float(st.final_pitch_return_err_px),
        mp_obj_new_int(st.final_pitch_min_full_markers),
        mp_obj_new_float(st.final_pitch_avg_full_markers),
        mp_obj_new_bool(st.final_pitch_marker_lock_ok),
        mp_obj_new_bool(st.final_pitch_consistent),
        mp_obj_new_bool(st.final_pitch_observable),
        mp_obj_new_bool(st.final_roll_ok),
        mp_obj_new_int(st.final_roll_attempts),
        mp_obj_new_float(st.final_roll_pulse_deg),
        mp_obj_new_int(st.final_roll_expected_axis_code),
        mp_obj_new_int(st.final_roll_expected_sign),
        mp_obj_new_int(st.final_roll_observed_axis_code),
        mp_obj_new_int(st.final_roll_observed_sign),
        mp_obj_new_float(st.final_roll_dominance_ratio),
        mp_obj_new_float(st.final_roll_response_strength_px),
        mp_obj_new_float(st.final_roll_noise_gate_px),
        mp_obj_new_float(st.final_roll_comp_dx),
        mp_obj_new_float(st.final_roll_comp_dy),
        mp_obj_new_float(st.final_roll_return_err_px),
        mp_obj_new_int(st.final_roll_min_full_markers),
        mp_obj_new_float(st.final_roll_avg_full_markers),
        mp_obj_new_bool(st.final_roll_marker_lock_ok),
        mp_obj_new_bool(st.final_roll_consistent),
        mp_obj_new_bool(st.final_roll_observable),
        mp_obj_new_bool(st.recenter_ok),
        mp_obj_new_int(st.recenter_abort_code),
        mp_obj_new_bool(st.recenter_recenter_ok),
        mp_obj_new_bool(st.recenter_hover_ok),
        mp_obj_new_int(st.recenter_ticks_done),
        mp_obj_new_int(st.recenter_hover_ticks),
        mp_obj_new_int(st.recenter_hover_stable_ticks),
        mp_obj_new_int(st.recenter_n_full_min),
        mp_obj_new_float(st.recenter_avg_full_markers),
        mp_obj_new_float(st.recenter_initial_err_px),
        mp_obj_new_float(st.recenter_final_err_px),
        mp_obj_new_float(st.recenter_min_err_px),
        mp_obj_new_float(st.recenter_max_err_px),
        mp_obj_new_float(st.recenter_improvement_px),
        mp_obj_new_float(st.recenter_hover_err_max_px),
        mp_obj_new_float(st.recenter_hover_err_min_px),
        mp_obj_new_float(st.recenter_hover_err_last_px),
        mp_obj_new_int(st.recenter_thrust_last_u16),
        mp_obj_new_float(st.recenter_first_roll_deg),
        mp_obj_new_float(st.recenter_first_pitch_deg),
        mp_obj_new_float(st.recenter_first_err_px),
        mp_obj_new_float(st.recenter_last_roll_deg),
        mp_obj_new_float(st.recenter_last_pitch_deg),
        mp_obj_new_float(st.recenter_last_err_px),
        mp_obj_new_bool(st.manual_disarmed),
        mp_obj_new_int(st.manual_disarm_rc),
        mp_obj_new_int(st.manual_ticks_done),
        mp_obj_new_int(st.manual_thrust_last_u16),
        mp_obj_new_bool(st.center_hold_ok),
        mp_obj_new_int(st.center_hold_trigger_code),
        mp_obj_new_int(st.center_hold_n_full_min),
        mp_obj_new_float(st.center_hold_avg_full_markers),
        mp_obj_new_int(st.center_hold_avg_window),
        mp_obj_new_float(st.center_hold_err_max_px),
        mp_obj_new_float(st.center_hold_err_last_px),
        mp_obj_new_int(st.center_hold_descent_pause_ticks),
        mp_obj_new_int(st.center_hold_ticks_done),
        mp_obj_new_float(st.center_hold_z_cam_min_m),
        mp_obj_new_float(st.center_hold_z_cam_max_m),
        mp_obj_new_float(st.center_hold_z_cam_last_m),
        mp_obj_new_float(st.center_hold_vz_filt_last_m_s),
        mp_obj_new_float(st.center_hold_target_vz_m_s),
        mp_obj_new_int(st.center_hold_thrust_last_u16),
        mp_obj_new_bool(st.center_hold_disarmed),
        mp_obj_new_int(st.center_hold_disarm_rc),
        mp_obj_new_float(st.center_hold_deadband_px),
        mp_obj_new_bool(st.center_hold_noise_ok),
        mp_obj_new_float(st.center_hold_noise_sigma_px),
    };
    return mp_obj_new_tuple(183, t);
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_orientation_status_tuple_obj,
                                  calib_orientation_status_tuple);

// OP-S10-W21-T6 — takeoff-refusal guard helper.  Mission code calls
// this before issuing hl_takeoff; raises RuntimeError when the drone
// has not been brought up.  Per [[sentai-calib-is-production-bringup]]:
// uncalibrated drones never auto-fly — the operator (or production
// technician) must trigger sentai.calib.run_bringup() first.
static mp_obj_t calib_assert_calibrated(void) {
    if (!sentai_calib_is_calibrated()) {
        mp_raise_msg(&mp_type_RuntimeError,
            MP_ERROR_TEXT("sentai.calib not ready — run sentai.calib.run_bringup() first"));
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_assert_calibrated_obj,
                                   calib_assert_calibrated);

// ===================== rotation_angle_deg(R1, R2) =====================

static mp_obj_t calib_rotation_angle(mp_obj_t a, mp_obj_t b) {
    float R1[9], R2[9];
    if (calib_parse_R9(a, R1) != 0 || calib_parse_R9(b, R2) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("R1 and R2 must be 9 floats"));
    }
    return mp_obj_new_float(sentai_calib_rotation_angle_deg(R1, R2));
}
static MP_DEFINE_CONST_FUN_OBJ_2(calib_rotation_angle_obj, calib_rotation_angle);

// ===================== LEGACY OP-S10-W14 autotuner =====================
// B5 delete candidate: kept only so old Flow-autotune experiments can still
// be replayed.  New B3/B4 image-frame calibration/validation must not add
// surface here.
// Minimal MP surface (operator 2026-05-18): set_context + task_start
// + task_stop + is_done + get_kp.  All math + state machine in C++.

static int calib_parse_axis_(mp_obj_t obj, sentai_calib_axis_t* out) {
    if (mp_obj_is_str(obj)) {
        size_t n;
        const char* s = mp_obj_str_get_data(obj, &n);
        if (n == 1 && (s[0] == 'x' || s[0] == 'X')) { *out = SENTAI_CALIB_AXIS_X; return 0; }
        if (n == 1 && (s[0] == 'y' || s[0] == 'Y')) { *out = SENTAI_CALIB_AXIS_Y; return 0; }
        if (n == 3 && (s[0] == 'y' || s[0] == 'Y') &&
                      (s[1] == 'a' || s[1] == 'A') &&
                      (s[2] == 'w' || s[2] == 'W')) {
            *out = SENTAI_CALIB_AXIS_YAW; return 0;
        }
        return -1;
    }
    // Integer alias 0/1/2 — convenient for the bringup orchestrator
    // which iterates over axes by index.
    const int v = mp_obj_get_int(obj);
    if (v == SENTAI_CALIB_AXIS_X || v == SENTAI_CALIB_AXIS_Y ||
        v == SENTAI_CALIB_AXIS_YAW) {
        *out = (sentai_calib_axis_t)v;
        return 0;
    }
    return -1;
}

static mp_obj_t calib_set_context(size_t n_args, const mp_obj_t* args) {
    float zh   = mp_obj_get_float(args[0]);
    float gdx  = mp_obj_get_float(args[1]);
    float gdy  = mp_obj_get_float(args[2]);
    float ms   = mp_obj_get_float(args[3]);
    return mp_obj_new_int(sentai_calib_set_context(zh, gdx, gdy, ms));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calib_set_context_obj,
                                            4, 4, calib_set_context);

static mp_obj_t calib_task_start(size_t n_args, const mp_obj_t* args) {
    sentai_calib_axis_t axis;
    if (calib_parse_axis_(args[0], &axis) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("axis must be 'x' or 'y'"));
    }
    float dur_s    = mp_obj_get_float(args[1]);
    float vmax_m_s = mp_obj_get_float(args[2]);
    return mp_obj_new_int(sentai_calib_task_start(axis, dur_s, vmax_m_s));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calib_task_start_obj,
                                            3, 3, calib_task_start);

static mp_obj_t calib_task_stop(void) {
    return mp_obj_new_int(sentai_calib_task_stop());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_task_stop_obj, calib_task_stop);

static mp_obj_t calib_is_done(void) {
    return mp_obj_new_bool(sentai_calib_task_is_done());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_is_done_obj, calib_is_done);

static mp_obj_t calib_get_kp(mp_obj_t axis_obj) {
    sentai_calib_axis_t axis;
    if (calib_parse_axis_(axis_obj, &axis) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("axis must be 'x', 'y', 'yaw' or int 0..2"));
    }
    return mp_obj_new_float(sentai_calib_get_kp(axis));
}
static MP_DEFINE_CONST_FUN_OBJ_1(calib_get_kp_obj, calib_get_kp);

// LEGACY OP-S10-W21-T3 — persisted Kp commit / read.
//   sentai.calib.commit_kp(axis, kp)        -> int (0 ok, -1 invalid)
//   sentai.calib.get_persisted_kp(axis)     -> float (-1.0 if not set)
static mp_obj_t calib_commit_kp(mp_obj_t axis_obj, mp_obj_t kp_obj) {
    sentai_calib_axis_t axis;
    if (calib_parse_axis_(axis_obj, &axis) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("axis must be 'x', 'y', 'yaw' or int 0..2"));
    }
    const float kp = (float)mp_obj_get_float(kp_obj);
    return mp_obj_new_int(sentai_calib_commit_kp(axis, kp));
}
static MP_DEFINE_CONST_FUN_OBJ_2(calib_commit_kp_obj, calib_commit_kp);

static mp_obj_t calib_get_persisted_kp(mp_obj_t axis_obj) {
    sentai_calib_axis_t axis;
    if (calib_parse_axis_(axis_obj, &axis) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("axis must be 'x', 'y', 'yaw' or int 0..2"));
    }
    return mp_obj_new_float(sentai_calib_get_persisted_kp(axis));
}
static MP_DEFINE_CONST_FUN_OBJ_1(calib_get_persisted_kp_obj,
                                  calib_get_persisted_kp);

static mp_obj_t calib_get_td_ms(void) {
    return mp_obj_new_int_from_uint(sentai_calib_get_td_ms());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_get_td_ms_obj, calib_get_td_ms);

// LEGACY OP-S10-W14-T12 — HOLD validation (closed-loop P with both Kp_x/y).
//   sentai.calib.hold_start(kp_x, kp_y, vmax_clip, dur_s) -> int
static mp_obj_t calib_hold_start(size_t n_args, const mp_obj_t* args) {
    float kp_x      = mp_obj_get_float(args[0]);
    float kp_y      = mp_obj_get_float(args[1]);
    float vmax_clip = mp_obj_get_float(args[2]);
    float dur_s     = mp_obj_get_float(args[3]);
    return mp_obj_new_int(sentai_calib_hold_start(kp_x, kp_y,
                                                    vmax_clip, dur_s));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calib_hold_start_obj, 4, 4,
                                            calib_hold_start);

// LEGACY OP-S10-W14-T16 — YawArucoBaseline rotating hold (5th arg = yaw_rate
// in deg/s).  Same as hold_start but commands rotation; auto-switches
// VPE format (ExtPos position-only while rotating).
static mp_obj_t calib_hold_yaw_start(size_t n_args, const mp_obj_t* args) {
    float kp_x       = mp_obj_get_float(args[0]);
    float kp_y       = mp_obj_get_float(args[1]);
    float vmax_clip  = mp_obj_get_float(args[2]);
    float dur_s      = mp_obj_get_float(args[3]);
    float yaw_rate   = mp_obj_get_float(args[4]);
    return mp_obj_new_int(sentai_calib_hold_yaw_start(kp_x, kp_y,
                                                       vmax_clip, dur_s,
                                                       yaw_rate));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calib_hold_yaw_start_obj, 5, 5,
                                            calib_hold_yaw_start);

static mp_obj_t calib_get_hold_max(void) {
    return mp_obj_new_float(sentai_calib_get_hold_max_drift_m());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_get_hold_max_obj, calib_get_hold_max);

static mp_obj_t calib_get_hold_rms(void) {
    return mp_obj_new_float(sentai_calib_get_hold_rms_drift_m());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_get_hold_rms_obj, calib_get_hold_rms);

// ===================== LEGACY OP-S10-W21-T4 bringup orchestrator =====================
// B5 delete candidate: this returns MP dicts and belongs to the old bringup
// path.  Keep for replay only; do not use as a model for the new ARM path.
//
//   sentai.calib.run_bringup(marker_world, marker_size_m=0.094,
//                              z_hold=0.78, sweep_radius=0.10,
//                              settle_s=2.0, vmax=0.10,
//                              dur_relay=30.0, dur_hold=10.0,
//                              hold_rms_max=0.030)
//      marker_world: list of (x, y, z) tuples (1..8), indexed by ID.
//      Returns int: 0 on spawn, -1 invalid, -2 busy, -3 task-create fail.
//
//   sentai.calib.bringup_is_done()       -> bool
//   sentai.calib.bringup_get_phase()     -> int (0..8)
//   sentai.calib.bringup_get_result()    -> dict (see below)
//   sentai.calib.bringup_abort()         -> 0

static mp_obj_t calib_run_bringup(size_t n_args, const mp_obj_t* pos_args,
                                    mp_map_t* kw_args) {
    if (n_args < 1) {
        mp_raise_TypeError(MP_ERROR_TEXT("marker_world required"));
    }
    sentai_calib_bringup_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));

    // marker_world: list of N (x, y, z) tuples.
    size_t mw_n; mp_obj_t* mw_items;
    mp_obj_get_array(pos_args[0], &mw_n, &mw_items);
    if ((int)mw_n < 1 || (int)mw_n > SENTAI_CALIB_BRINGUP_MAX_MARKERS) {
        mp_raise_ValueError(MP_ERROR_TEXT("marker_world len must be 1..8"));
    }
    ctx.marker_n = (int32_t)mw_n;
    for (size_t i = 0; i < mw_n; ++i) {
        float xyz[3];
        if (calib_parse_vec3(mw_items[i], xyz) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("marker_world[i] must be 3-tuple"));
        }
        ctx.marker_world_n3[3*i + 0] = xyz[0];
        ctx.marker_world_n3[3*i + 1] = xyz[1];
        ctx.marker_world_n3[3*i + 2] = xyz[2];
    }

    // Defaults — tuned per OP-S10-W14 + design doc target acceptance.
    ctx.marker_size_m   = 0.094f;
    ctx.z_hold_m        = 0.78f;
    ctx.sweep_radius_m  = 0.10f;
    ctx.settle_s        = 2.0f;
    ctx.vmax_m_s        = 0.10f;
    ctx.dur_relay_s     = 30.0f;
    ctx.dur_hold_s      = 10.0f;
    ctx.hold_rms_max_m  = 0.030f;

    // Override via kwargs.
    static const mp_obj_t key_marker_size = MP_OBJ_NEW_QSTR(MP_QSTR_marker_size_m);
    static const mp_obj_t key_z_hold      = MP_OBJ_NEW_QSTR(MP_QSTR_z_hold);
    static const mp_obj_t key_sweep_r     = MP_OBJ_NEW_QSTR(MP_QSTR_sweep_radius);
    static const mp_obj_t key_settle_s    = MP_OBJ_NEW_QSTR(MP_QSTR_settle_s);
    static const mp_obj_t key_vmax        = MP_OBJ_NEW_QSTR(MP_QSTR_vmax);
    static const mp_obj_t key_dur_relay   = MP_OBJ_NEW_QSTR(MP_QSTR_dur_relay);
    static const mp_obj_t key_dur_hold    = MP_OBJ_NEW_QSTR(MP_QSTR_dur_hold);
    static const mp_obj_t key_hold_rms    = MP_OBJ_NEW_QSTR(MP_QSTR_hold_rms_max);
    mp_map_elem_t* el;
    if ((el = mp_map_lookup(kw_args, (mp_obj_t)key_marker_size, MP_MAP_LOOKUP)))
        ctx.marker_size_m = mp_obj_get_float(el->value);
    if ((el = mp_map_lookup(kw_args, (mp_obj_t)key_z_hold, MP_MAP_LOOKUP)))
        ctx.z_hold_m = mp_obj_get_float(el->value);
    if ((el = mp_map_lookup(kw_args, (mp_obj_t)key_sweep_r, MP_MAP_LOOKUP)))
        ctx.sweep_radius_m = mp_obj_get_float(el->value);
    if ((el = mp_map_lookup(kw_args, (mp_obj_t)key_settle_s, MP_MAP_LOOKUP)))
        ctx.settle_s = mp_obj_get_float(el->value);
    if ((el = mp_map_lookup(kw_args, (mp_obj_t)key_vmax, MP_MAP_LOOKUP)))
        ctx.vmax_m_s = mp_obj_get_float(el->value);
    if ((el = mp_map_lookup(kw_args, (mp_obj_t)key_dur_relay, MP_MAP_LOOKUP)))
        ctx.dur_relay_s = mp_obj_get_float(el->value);
    if ((el = mp_map_lookup(kw_args, (mp_obj_t)key_dur_hold, MP_MAP_LOOKUP)))
        ctx.dur_hold_s = mp_obj_get_float(el->value);
    if ((el = mp_map_lookup(kw_args, (mp_obj_t)key_hold_rms, MP_MAP_LOOKUP)))
        ctx.hold_rms_max_m = mp_obj_get_float(el->value);

    return mp_obj_new_int(sentai_calib_bringup_start(&ctx));
}
static MP_DEFINE_CONST_FUN_OBJ_KW(calib_run_bringup_obj, 1, calib_run_bringup);

static mp_obj_t calib_bringup_is_done(void) {
    return mp_obj_new_bool(sentai_calib_bringup_is_done());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_bringup_is_done_obj, calib_bringup_is_done);

static mp_obj_t calib_bringup_get_phase(void) {
    return mp_obj_new_int((int)sentai_calib_bringup_get_phase());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_bringup_get_phase_obj, calib_bringup_get_phase);

static mp_obj_t calib_bringup_abort(void) {
    return mp_obj_new_int(sentai_calib_bringup_abort());
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_bringup_abort_obj, calib_bringup_abort);

static mp_obj_t calib_bringup_get_result(void) {
    sentai_calib_bringup_result_t r;
    if (sentai_calib_bringup_get_result(&r) != 0) {
        return mp_const_none;
    }
    mp_obj_t d = mp_obj_new_dict(0);
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_accepted),
                       mp_obj_new_bool(r.accepted));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_reject_code),
                       mp_obj_new_int(r.reject_code));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_last_phase),
                       mp_obj_new_int(r.last_phase));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_R_cam_to_body),
                       calib_R_to_tuple(r.R_cam_to_body));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_cam_offset_B),
                       calib_vec3_to_tuple(r.cam_offset_B));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_kp_x),
                       mp_obj_new_float(r.kp_x));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_kp_y),
                       mp_obj_new_float(r.kp_y));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_hold_max_drift_m),
                       mp_obj_new_float(r.hold_max_drift_m));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_hold_rms_drift_m),
                       mp_obj_new_float(r.hold_rms_drift_m));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_n_samples_used),
                       mp_obj_new_int(r.n_samples_used));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_total_duration_ms),
                       mp_obj_new_int(r.total_duration_ms));
    mp_obj_dict_store(d, MP_OBJ_NEW_QSTR(MP_QSTR_ext_quality),
                       calib_quality_to_dict(&r.ext_quality));
    return d;
}
static MP_DEFINE_CONST_FUN_OBJ_0(calib_bringup_get_result_obj,
                                  calib_bringup_get_result);

// ===================== Module table =====================

static const mp_rom_map_elem_t sentai_calib_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),         MP_ROM_QSTR(MP_QSTR_calib) },
    { MP_ROM_QSTR(MP_QSTR_init),              MP_ROM_PTR(&calib_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),             MP_ROM_PTR(&calib_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_run_kabsch),        MP_ROM_PTR(&calib_run_kabsch_obj) },
    { MP_ROM_QSTR(MP_QSTR_commit_R),          MP_ROM_PTR(&calib_commit_R_obj) },
    { MP_ROM_QSTR(MP_QSTR_save),              MP_ROM_PTR(&calib_save_obj) },
    { MP_ROM_QSTR(MP_QSTR_save_contract),     MP_ROM_PTR(&calib_save_contract_obj) },
    { MP_ROM_QSTR(MP_QSTR_load),              MP_ROM_PTR(&calib_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_R_cam_to_body), MP_ROM_PTR(&calib_get_R_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_cam_offset_B),  MP_ROM_PTR(&calib_get_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_calibrated),     MP_ROM_PTR(&calib_is_calib_obj) },
    { MP_ROM_QSTR(MP_QSTR_ini_status_tuple),  MP_ROM_PTR(&calib_ini_status_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_extpos_signs),  MP_ROM_PTR(&calib_get_extpos_signs_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_axis_seed_tuple),
                                              MP_ROM_PTR(&calib_get_axis_seed_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_defaults_tuple),
                                              MP_ROM_PTR(&calib_defaults_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_limits_tuple),
                                              MP_ROM_PTR(&calib_limits_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_setup_defaults),
                                              MP_ROM_PTR(&calib_setup_defaults_obj) },
    { MP_ROM_QSTR(MP_QSTR_marker_avg_lock_ok_tuple),
                                              MP_ROM_PTR(&calib_marker_avg_lock_ok_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_marker_avg_unsafe_tuple),
                                              MP_ROM_PTR(&calib_marker_avg_unsafe_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_feature_has_lock_tuple),
                                              MP_ROM_PTR(&calib_feature_has_lock_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_marker_lock_ok_tuple),
                                              MP_ROM_PTR(&calib_marker_lock_ok_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_score_axis_candidate),
                                              MP_ROM_PTR(&calib_score_axis_candidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_expected_from_row_tuple),
                                              MP_ROM_PTR(&calib_expected_from_row_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_axis_observation_from_delta_tuple),
                                              MP_ROM_PTR(&calib_axis_observation_from_delta_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_sample_calib_observation_tuple),
                                              MP_ROM_PTR(&calib_sample_calib_observation_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_vertical_rate_thrust_tuple),
                                              MP_ROM_PTR(&calib_vertical_rate_thrust_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_z_hold_thrust_tuple),
                                              MP_ROM_PTR(&calib_z_hold_thrust_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_task_start),
                                              MP_ROM_PTR(&calib_orientation_task_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_arm_zero_start),
                                              MP_ROM_PTR(&calib_orientation_arm_zero_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_marker_acquisition_start),
                                              MP_ROM_PTR(&calib_orientation_marker_acquisition_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_post_lock_brake_start),
                                              MP_ROM_PTR(&calib_orientation_post_lock_brake_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_visual_z_hold_start),
                                              MP_ROM_PTR(&calib_orientation_visual_z_hold_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_axis_response_start),
                                              MP_ROM_PTR(&calib_orientation_axis_response_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_centroid_validation_start),
                                              MP_ROM_PTR(&calib_orientation_centroid_validation_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_score_candidate_from_axis),
                                              MP_ROM_PTR(&calib_orientation_score_candidate_from_axis_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_optical_axis_validate),
                                              MP_ROM_PTR(&calib_orientation_optical_axis_validate_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_final_candidate_validation_start),
                                              MP_ROM_PTR(&calib_orientation_final_candidate_validation_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_final_recenter_start),
                                              MP_ROM_PTR(&calib_orientation_final_recenter_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_manual_descend_start),
                                              MP_ROM_PTR(&calib_orientation_manual_descend_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_center_hold_descend_start),
                                              MP_ROM_PTR(&calib_orientation_center_hold_descend_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_save_contract),
                                              MP_ROM_PTR(&calib_orientation_save_contract_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_emergency_stop),
                                              MP_ROM_PTR(&calib_orientation_emergency_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_task_stop),
                                              MP_ROM_PTR(&calib_orientation_task_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_task_is_done),
                                              MP_ROM_PTR(&calib_orientation_task_is_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_result_tuple),
                                              MP_ROM_PTR(&calib_orientation_result_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_orientation_status_tuple),
                                              MP_ROM_PTR(&calib_orientation_status_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_assert_calibrated), MP_ROM_PTR(&calib_assert_calibrated_obj) },
    { MP_ROM_QSTR(MP_QSTR_rotation_angle_deg),
                                              MP_ROM_PTR(&calib_rotation_angle_obj) },
    // LEGACY OP-S10-W14 autotuner surface — B5 delete candidate.
    { MP_ROM_QSTR(MP_QSTR_set_context),       MP_ROM_PTR(&calib_set_context_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_start),        MP_ROM_PTR(&calib_task_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_stop),         MP_ROM_PTR(&calib_task_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_done),           MP_ROM_PTR(&calib_is_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_kp),            MP_ROM_PTR(&calib_get_kp_obj) },
    { MP_ROM_QSTR(MP_QSTR_commit_kp),         MP_ROM_PTR(&calib_commit_kp_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_persisted_kp),  MP_ROM_PTR(&calib_get_persisted_kp_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_td_ms),         MP_ROM_PTR(&calib_get_td_ms_obj) },
    // LEGACY OP-S10-W14-T12 — HOLD validation.
    { MP_ROM_QSTR(MP_QSTR_hold_start),        MP_ROM_PTR(&calib_hold_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_hold_yaw_start),    MP_ROM_PTR(&calib_hold_yaw_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hold_max),      MP_ROM_PTR(&calib_get_hold_max_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hold_rms),      MP_ROM_PTR(&calib_get_hold_rms_obj) },
    // LEGACY OP-S10-W21-T4 bringup orchestrator — B5 delete candidate.
    { MP_ROM_QSTR(MP_QSTR_run_bringup),         MP_ROM_PTR(&calib_run_bringup_obj) },
    { MP_ROM_QSTR(MP_QSTR_bringup_is_done),     MP_ROM_PTR(&calib_bringup_is_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_bringup_get_phase),   MP_ROM_PTR(&calib_bringup_get_phase_obj) },
    { MP_ROM_QSTR(MP_QSTR_bringup_get_result),  MP_ROM_PTR(&calib_bringup_get_result_obj) },
    { MP_ROM_QSTR(MP_QSTR_bringup_abort),       MP_ROM_PTR(&calib_bringup_abort_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_calib_globals, sentai_calib_globals_table);

const mp_obj_module_t sentai_calib_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_calib_globals,
};

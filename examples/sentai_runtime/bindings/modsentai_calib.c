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

// ===================== rotation_angle_deg(R1, R2) =====================

static mp_obj_t calib_rotation_angle(mp_obj_t a, mp_obj_t b) {
    float R1[9], R2[9];
    if (calib_parse_R9(a, R1) != 0 || calib_parse_R9(b, R2) != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("R1 and R2 must be 9 floats"));
    }
    return mp_obj_new_float(sentai_calib_rotation_angle_deg(R1, R2));
}
static MP_DEFINE_CONST_FUN_OBJ_2(calib_rotation_angle_obj, calib_rotation_angle);

// ===================== OP-S10-W14 autotuner =====================
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

// OP-S10-W21-T3 — persisted Kp commit / read.
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

// OP-S10-W14-T12 — HOLD validation (closed-loop P with both Kp_x/y).
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

// OP-S10-W14-T16 — YawArucoBaseline rotating hold (5th arg = yaw_rate
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

// ===================== OP-S10-W21-T4 bringup orchestrator =====================
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
    { MP_ROM_QSTR(MP_QSTR_load),              MP_ROM_PTR(&calib_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_R_cam_to_body), MP_ROM_PTR(&calib_get_R_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_cam_offset_B),  MP_ROM_PTR(&calib_get_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_calibrated),     MP_ROM_PTR(&calib_is_calib_obj) },
    { MP_ROM_QSTR(MP_QSTR_rotation_angle_deg),
                                              MP_ROM_PTR(&calib_rotation_angle_obj) },
    // OP-S10-W14 autotuner surface
    { MP_ROM_QSTR(MP_QSTR_set_context),       MP_ROM_PTR(&calib_set_context_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_start),        MP_ROM_PTR(&calib_task_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_task_stop),         MP_ROM_PTR(&calib_task_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_done),           MP_ROM_PTR(&calib_is_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_kp),            MP_ROM_PTR(&calib_get_kp_obj) },
    { MP_ROM_QSTR(MP_QSTR_commit_kp),         MP_ROM_PTR(&calib_commit_kp_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_persisted_kp),  MP_ROM_PTR(&calib_get_persisted_kp_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_td_ms),         MP_ROM_PTR(&calib_get_td_ms_obj) },
    // OP-S10-W14-T12 — HOLD validation
    { MP_ROM_QSTR(MP_QSTR_hold_start),        MP_ROM_PTR(&calib_hold_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_hold_yaw_start),    MP_ROM_PTR(&calib_hold_yaw_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hold_max),      MP_ROM_PTR(&calib_get_hold_max_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hold_rms),      MP_ROM_PTR(&calib_get_hold_rms_obj) },
    // OP-S10-W21-T4 bringup orchestrator
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

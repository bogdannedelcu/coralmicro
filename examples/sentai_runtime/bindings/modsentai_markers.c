// modsentai_markers.c — MicroPython binding for sentai.markers.*
// (OP-S10-W19-T1 unified fiducial-marker namespace).
//
// Surface (minimal, struct-only):
//
//   sentai.markers.init(backend)            -> int (0 ok, <0 err)
//                                            backend ∈ {"aruco", "whycon"}
//                                            or int (1=ARUCO, 2=WHYCON).
//   sentai.markers.clear()                  -> None
//   sentai.markers.backend()                -> "none" | "aruco" | "whycon"
//   sentai.markers.set_intrinsics(fx,fy,cx,cy) -> None
//   sentai.markers.set_marker_size(metres)  -> None
//   sentai.markers.detect_from_camera()     -> int n_detected (or <0 err)
//   sentai.markers.get_count()              -> int
//   sentai.markers.get_pose(i, out_buf)     -> int (1 ok, 0 idx-out)
//                                              out_buf is bytearray(48)
//   sentai.markers.get_stats(out_buf)       -> int (1 ok)
//                                              out_buf is bytearray(20)
//   sentai.markers.detect_cyc_last()        -> uint
//
// SIM eval helper (W19-T4):
//   sentai.markers.synth_one_whycon(cx, cy, R) -> int n_detected
//
// Per HARD-RULE [[no-heavy-data-through-mp]]: callers ALLOCATE the
// bytearray; the binding writes the struct in place.  NO mp_obj_dict_*
// calls in this module.

#include <string.h>

#include "py/runtime.h"
#include "py/objarray.h"

#include "../sentai_markers.h"

// =====================================================================
// Forward decls — C surface from sentai_markers.cc.
// =====================================================================

extern int      sentai_markers_init(sentai_markers_backend_t backend);
extern void     sentai_markers_clear(void);
extern sentai_markers_backend_t sentai_markers_get_backend(void);
extern void     sentai_markers_set_intrinsics(float fx, float fy,
                                                float cx, float cy);
extern void     sentai_markers_set_marker_size(float meters);
extern void     sentai_markers_set_cam_extrinsics(float tx, float ty,
                                                     float tz, float roll,
                                                     float pitch, float yaw);
extern void     sentai_markers_clear_cam_extrinsics(void);
extern int      sentai_markers_detect_frame(const uint8_t* gray, int w, int h,
                                              uint32_t frame_seq,
                                              uint32_t src_ts_ms);
extern int      sentai_markers_get_count(void);
extern int      sentai_markers_get_pose(int i, SentaiMarkersPose* out);
extern int      sentai_markers_get_stats(SentaiMarkersStats* out);
extern int      sentai_markers_get_latest(int i, SentaiMarkersPose* out);
extern uint32_t sentai_markers_detect_cyc_last(void);
extern int      sentai_markers_synth_one_whycon(int cx_px, int cy_px,
                                                  int radius_px);

extern int      sentai_markers_set_marker_world(int n, const float* xyz_n3);
extern int      sentai_markers_get_marker_world_count(void);
extern int      sentai_markers_get_drone_pose(float cf2_yaw_rad,
                                                SentaiMarkersDronePose* out);
extern int      sentai_markers_test_inject_obs(int n, const float* tvec_n3);

extern int sentai_camera_grab_gray_zerocopy(const uint8_t** out_buf,
                                              int* out_w, int* out_h,
                                              uint32_t* out_seq,
                                              uint32_t* out_ts);

// =====================================================================
// Backend parsing.
// =====================================================================

static sentai_markers_backend_t parse_backend_(mp_obj_t obj) {
    if (mp_obj_is_str(obj)) {
        const char* s = mp_obj_str_get_str(obj);
        if (strcmp(s, "aruco")  == 0) return SENTAI_MARKERS_BACKEND_ARUCO;
        if (strcmp(s, "whycon") == 0) return SENTAI_MARKERS_BACKEND_WHYCON;
        if (strcmp(s, "none")   == 0) return SENTAI_MARKERS_BACKEND_NONE;
        mp_raise_ValueError(MP_ERROR_TEXT(
            "backend must be 'aruco' or 'whycon'"));
    }
    const int v = mp_obj_get_int(obj);
    if (v == (int)SENTAI_MARKERS_BACKEND_ARUCO ||
        v == (int)SENTAI_MARKERS_BACKEND_WHYCON ||
        v == (int)SENTAI_MARKERS_BACKEND_NONE) {
        return (sentai_markers_backend_t)v;
    }
    mp_raise_ValueError(MP_ERROR_TEXT(
        "backend int must be 0 (NONE), 1 (ARUCO), or 2 (WHYCON)"));
}

// =====================================================================
// MP bindings.
// =====================================================================

static mp_obj_t markers_init_(mp_obj_t backend_obj) {
    const int rc = sentai_markers_init(parse_backend_(backend_obj));
    return mp_obj_new_int(rc);
}
static MP_DEFINE_CONST_FUN_OBJ_1(markers_init_obj, markers_init_);

static mp_obj_t markers_clear_(void) {
    sentai_markers_clear();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(markers_clear_obj, markers_clear_);

static mp_obj_t markers_backend_(void) {
    switch (sentai_markers_get_backend()) {
    case SENTAI_MARKERS_BACKEND_ARUCO:
        return mp_obj_new_str("aruco", 5);
    case SENTAI_MARKERS_BACKEND_WHYCON:
        return mp_obj_new_str("whycon", 6);
    default:
        return mp_obj_new_str("none", 4);
    }
}
static MP_DEFINE_CONST_FUN_OBJ_0(markers_backend_obj, markers_backend_);

static mp_obj_t markers_set_intrinsics_(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    const float fx = (float)mp_obj_get_float(args[0]);
    const float fy = (float)mp_obj_get_float(args[1]);
    const float cx = (float)mp_obj_get_float(args[2]);
    const float cy = (float)mp_obj_get_float(args[3]);
    sentai_markers_set_intrinsics(fx, fy, cx, cy);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(markers_set_intrinsics_obj,
                                             4, 4, markers_set_intrinsics_);

static mp_obj_t markers_set_marker_size_(mp_obj_t m_obj) {
    sentai_markers_set_marker_size((float)mp_obj_get_float(m_obj));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(markers_set_marker_size_obj,
                                   markers_set_marker_size_);

// sentai.markers.set_cam_extrinsics(tx, ty, tz, roll, pitch, yaw)
//   All floats in metres / radians.  Configures the cam-mount offset
//   so that tvec_cam returned by get_pose is in BODY frame (drone
//   centre of mass), not camera optical frame.
static mp_obj_t markers_set_cam_extrinsics_(size_t n_args,
                                               const mp_obj_t* args) {
    (void)n_args;
    const float tx    = (float)mp_obj_get_float(args[0]);
    const float ty    = (float)mp_obj_get_float(args[1]);
    const float tz    = (float)mp_obj_get_float(args[2]);
    const float roll  = (float)mp_obj_get_float(args[3]);
    const float pitch = (float)mp_obj_get_float(args[4]);
    const float yaw   = (float)mp_obj_get_float(args[5]);
    sentai_markers_set_cam_extrinsics(tx, ty, tz, roll, pitch, yaw);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(markers_set_cam_extrinsics_obj,
                                             6, 6, markers_set_cam_extrinsics_);

static mp_obj_t markers_clear_cam_extrinsics_(void) {
    sentai_markers_clear_cam_extrinsics();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(markers_clear_cam_extrinsics_obj,
                                   markers_clear_cam_extrinsics_);

static mp_obj_t markers_detect_from_camera_(void) {
    const uint8_t* buf = NULL;
    int w = 0, h = 0;
    uint32_t seq = 0, ts = 0;
    if (sentai_camera_grab_gray_zerocopy(&buf, &w, &h, &seq, &ts) != 0) {
        return mp_obj_new_int(0);
    }
    return mp_obj_new_int(
        sentai_markers_detect_frame(buf, w, h, seq, ts));
}
static MP_DEFINE_CONST_FUN_OBJ_0(markers_detect_from_camera_obj,
                                   markers_detect_from_camera_);

// Bench-test entry point: run the active backend's detector on a
// caller-supplied grayscale buffer.  Useful for replay / unit tests
// without a live camera.
static mp_obj_t markers_detect_buffer_(size_t n_args, const mp_obj_t* args) {
    mp_buffer_info_t bi;
    if (!mp_get_buffer(args[0], &bi, MP_BUFFER_READ)) {
        mp_raise_TypeError(MP_ERROR_TEXT("gray must be bytes/bytearray"));
    }
    const int w = mp_obj_get_int(args[1]);
    const int h = mp_obj_get_int(args[2]);
    if ((size_t)w * (size_t)h > bi.len) {
        mp_raise_ValueError(MP_ERROR_TEXT("gray too small for w*h"));
    }
    return mp_obj_new_int(
        sentai_markers_detect_frame((const uint8_t*)bi.buf, w, h, 0, 0));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(markers_detect_buffer_obj, 3, 3,
                                            markers_detect_buffer_);

static mp_obj_t markers_get_count_(void) {
    return mp_obj_new_int(sentai_markers_get_count());
}
static MP_DEFINE_CONST_FUN_OBJ_0(markers_get_count_obj, markers_get_count_);

static mp_obj_t markers_get_pose_(mp_obj_t idx_obj, mp_obj_t buf_obj) {
    const int idx = mp_obj_get_int(idx_obj);
    mp_buffer_info_t bi;
    if (!mp_get_buffer(buf_obj, &bi, MP_BUFFER_WRITE)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "out_buf must be writable bytearray(48)"));
    }
    if (bi.len < (mp_int_t)sizeof(SentaiMarkersPose)) {
        mp_raise_ValueError(MP_ERROR_TEXT("out_buf too small"));
    }
    return mp_obj_new_int(
        sentai_markers_get_pose(idx, (SentaiMarkersPose*)bi.buf));
}
static MP_DEFINE_CONST_FUN_OBJ_2(markers_get_pose_obj, markers_get_pose_);

// SIM-convenience variant: returns a 12-tuple
//   (id, pixel_cx, pixel_cy, tx, ty, tz, rx, ry, rz, reproj_err,
//    backend, pose_valid)
// for callers that lack `bytearray` (the embed REPL config in SIM
// doesn't ship the bytearray builtin).  Equivalent payload to
// get_pose's struct write; ONLY for SIM eval scripts — production
// code should use get_pose + bytearray.
static mp_obj_t markers_get_pose_tuple_(mp_obj_t idx_obj) {
    const int idx = mp_obj_get_int(idx_obj);
    SentaiMarkersPose p;
    if (!sentai_markers_get_pose(idx, &p)) {
        return mp_const_none;
    }
    mp_obj_t t[12] = {
        mp_obj_new_int(p.id),
        mp_obj_new_float(p.pixel_cx),
        mp_obj_new_float(p.pixel_cy),
        mp_obj_new_float(p.tvec_cam[0]),
        mp_obj_new_float(p.tvec_cam[1]),
        mp_obj_new_float(p.tvec_cam[2]),
        mp_obj_new_float(p.rvec_cam[0]),
        mp_obj_new_float(p.rvec_cam[1]),
        mp_obj_new_float(p.rvec_cam[2]),
        mp_obj_new_float(p.reproj_err_px),
        mp_obj_new_int(p.backend),
        mp_obj_new_int(p.pose_valid),
    };
    return mp_obj_new_tuple(12, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(markers_get_pose_tuple_obj,
                                   markers_get_pose_tuple_);

static mp_obj_t markers_get_stats_(mp_obj_t buf_obj) {
    mp_buffer_info_t bi;
    if (!mp_get_buffer(buf_obj, &bi, MP_BUFFER_WRITE)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "out_buf must be writable bytearray(20)"));
    }
    if (bi.len < (mp_int_t)sizeof(SentaiMarkersStats)) {
        mp_raise_ValueError(MP_ERROR_TEXT("out_buf too small"));
    }
    return mp_obj_new_int(
        sentai_markers_get_stats((SentaiMarkersStats*)bi.buf));
}
static MP_DEFINE_CONST_FUN_OBJ_1(markers_get_stats_obj, markers_get_stats_);

static mp_obj_t markers_detect_cyc_last_(void) {
    return mp_obj_new_int_from_uint(sentai_markers_detect_cyc_last());
}
static MP_DEFINE_CONST_FUN_OBJ_0(markers_detect_cyc_last_obj,
                                   markers_detect_cyc_last_);

static mp_obj_t markers_synth_one_whycon_(size_t n_args, const mp_obj_t* args) {
    (void)n_args;
    const int cx = mp_obj_get_int(args[0]);
    const int cy = mp_obj_get_int(args[1]);
    const int r  = mp_obj_get_int(args[2]);
    return mp_obj_new_int(sentai_markers_synth_one_whycon(cx, cy, r));
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(markers_synth_one_whycon_obj,
                                             3, 3, markers_synth_one_whycon_);

// W19-T6b: drone-pose recovery (Kabsch + yaw-anchor).
// sentai.markers.set_marker_world(xyz_buf) where xyz_buf is a bytes /
// bytearray containing 3 * N float32 values (row-major, N markers).
// N is inferred from the buffer length; pass an empty buffer to clear.
static mp_obj_t markers_set_marker_world_(mp_obj_t buf_obj) {
    mp_buffer_info_t bi;
    if (!mp_get_buffer(buf_obj, &bi, MP_BUFFER_READ)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "xyz_buf must be bytes/bytearray of N*12 bytes"));
    }
    const size_t bytes_per_marker = 3 * sizeof(float);
    if (bi.len % bytes_per_marker != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "xyz_buf len must be multiple of 12 (3 float32 per marker)"));
    }
    const int n = (int)(bi.len / bytes_per_marker);
    return mp_obj_new_int(
        sentai_markers_set_marker_world(n, (const float*)bi.buf));
}
static MP_DEFINE_CONST_FUN_OBJ_1(markers_set_marker_world_obj,
                                   markers_set_marker_world_);

static mp_obj_t markers_get_marker_world_count_(void) {
    return mp_obj_new_int(sentai_markers_get_marker_world_count());
}
static MP_DEFINE_CONST_FUN_OBJ_0(markers_get_marker_world_count_obj,
                                   markers_get_marker_world_count_);

static mp_obj_t markers_get_drone_pose_(mp_obj_t yaw_obj, mp_obj_t buf_obj) {
    const float yaw = (float)mp_obj_get_float(yaw_obj);
    mp_buffer_info_t bi;
    if (!mp_get_buffer(buf_obj, &bi, MP_BUFFER_WRITE)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "out_buf must be writable bytearray(28)"));
    }
    if (bi.len < (mp_int_t)sizeof(SentaiMarkersDronePose)) {
        mp_raise_ValueError(MP_ERROR_TEXT("out_buf too small (need 28)"));
    }
    return mp_obj_new_int(
        sentai_markers_get_drone_pose(yaw, (SentaiMarkersDronePose*)bi.buf));
}
static MP_DEFINE_CONST_FUN_OBJ_2(markers_get_drone_pose_obj,
                                   markers_get_drone_pose_);

// SIM-convenience tuple variant for embed REPL (no bytearray builtin).
// Returns (ok, x, y, z, yaw_rad, res_max, n_used, flip_x, flip_y, flip_z)
// or None if no fit was produced.
static mp_obj_t markers_get_drone_pose_tuple_(mp_obj_t yaw_obj) {
    const float yaw = (float)mp_obj_get_float(yaw_obj);
    SentaiMarkersDronePose p;
    if (!sentai_markers_get_drone_pose(yaw, &p)) {
        return mp_const_none;
    }
    mp_obj_t t[9] = {
        mp_obj_new_float(p.x),
        mp_obj_new_float(p.y),
        mp_obj_new_float(p.z),
        mp_obj_new_float(p.yaw_rad),
        mp_obj_new_float(p.res_max),
        mp_obj_new_int(p.n_used),
        mp_obj_new_int(p.flip_x),
        mp_obj_new_int(p.flip_y),
        mp_obj_new_int(p.flip_z),
    };
    return mp_obj_new_tuple(9, t);
}
static MP_DEFINE_CONST_FUN_OBJ_1(markers_get_drone_pose_tuple_obj,
                                   markers_get_drone_pose_tuple_);

// Test-only: inject synthetic body-frame tvecs into the marker cache.
// Used by s184 to validate the Kabsch+yaw-anchor pipeline against
// analytic ground truth.  buf is bytes/bytearray of N*12 bytes (3 *
// float32 per marker).
static mp_obj_t markers_test_inject_obs_(mp_obj_t buf_obj) {
    mp_buffer_info_t bi;
    if (!mp_get_buffer(buf_obj, &bi, MP_BUFFER_READ)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "tvec_buf must be bytes/bytearray of N*12 bytes"));
    }
    const size_t bytes_per = 3 * sizeof(float);
    if (bi.len % bytes_per != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "tvec_buf len must be multiple of 12 (3 float32 per marker)"));
    }
    const int n = (int)(bi.len / bytes_per);
    return mp_obj_new_int(
        sentai_markers_test_inject_obs(n, (const float*)bi.buf));
}
static MP_DEFINE_CONST_FUN_OBJ_1(markers_test_inject_obs_obj,
                                   markers_test_inject_obs_);

// =====================================================================
// Module table.
// =====================================================================

static const mp_rom_map_elem_t sentai_markers_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),          MP_ROM_QSTR(MP_QSTR_markers) },
    { MP_ROM_QSTR(MP_QSTR_init),              MP_ROM_PTR(&markers_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear),             MP_ROM_PTR(&markers_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_backend),           MP_ROM_PTR(&markers_backend_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_intrinsics),    MP_ROM_PTR(&markers_set_intrinsics_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_cam_extrinsics),
                                                MP_ROM_PTR(&markers_set_cam_extrinsics_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_cam_extrinsics),
                                                MP_ROM_PTR(&markers_clear_cam_extrinsics_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_marker_size),   MP_ROM_PTR(&markers_set_marker_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_detect_from_camera),
                                                MP_ROM_PTR(&markers_detect_from_camera_obj) },
    { MP_ROM_QSTR(MP_QSTR_detect_buffer),     MP_ROM_PTR(&markers_detect_buffer_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_count),         MP_ROM_PTR(&markers_get_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pose),          MP_ROM_PTR(&markers_get_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pose_tuple),    MP_ROM_PTR(&markers_get_pose_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_stats),         MP_ROM_PTR(&markers_get_stats_obj) },
    { MP_ROM_QSTR(MP_QSTR_detect_cyc_last),   MP_ROM_PTR(&markers_detect_cyc_last_obj) },
    { MP_ROM_QSTR(MP_QSTR_synth_one_whycon),  MP_ROM_PTR(&markers_synth_one_whycon_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_marker_world),  MP_ROM_PTR(&markers_set_marker_world_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_marker_world_count),
                                                MP_ROM_PTR(&markers_get_marker_world_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drone_pose),     MP_ROM_PTR(&markers_get_drone_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drone_pose_tuple),
                                                MP_ROM_PTR(&markers_get_drone_pose_tuple_obj) },
    { MP_ROM_QSTR(MP_QSTR_test_inject_obs),    MP_ROM_PTR(&markers_test_inject_obs_obj) },
};
static MP_DEFINE_CONST_DICT(sentai_markers_globals, sentai_markers_globals_table);

const mp_obj_module_t sentai_markers_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sentai_markers_globals,
};

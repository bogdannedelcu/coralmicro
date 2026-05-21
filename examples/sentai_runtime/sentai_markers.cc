// sentai_markers.cc — unified fiducial-marker dispatcher.
//
// OP-S10-W19-T1.  Implements the public C surface from
// `sentai_markers.h`.  Routes init / set_* / get_* calls to the
// active backend's internal helpers (sentai_aruco_* /
// sentai_whycon_*) and translates the backend-specific output struct
// into the unified `SentaiMarkersPose`.
//
// Scope of T1 (this commit):
//   - State + cache for the unified API
//   - init / clear / set_intrinsics / set_marker_size
//   - get_count / get_pose / get_stats / get_latest
//   - detect-from-buffer (for the SIM eval; the binding pulls the
//     camera frame via sentai_camera_grab_gray_zerocopy and hands
//     us the buffer)
//   - synth_one helper for the SIM Krajník eval (s181) — draws ONE
//     Krajník marker at an arbitrary pixel position and runs the
//     active backend's detector on it.
//
// Out of scope (later W19 steps): refactor of sentai_calib*.cc
// internals from sentai_aruco_get_latest() to
// sentai_markers_get_latest(); deletion of the parallel
// sentai.aruco / sentai.whycon modules.

#include "sentai_markers.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "sentai_aruco.h"
#include "sentai_svd3.h"

// =====================================================================
// OP-S10-W19-T5: camera extrinsics (operator 2026-05-20).
// =====================================================================
//
// When set, transforms every detected marker's tvec from CAMERA
// OPTICAL frame (where the PnP solver produces it — OpenCV convention
// X-right, Y-down, Z-forward) into the BODY frame of the drone
// (X-forward, Y-left, Z-up — REP-103 / cf2 firmware convention).
//
// Composition:
//
//   R_opt_to_body = R_link_to_body · R_opt_to_link
//
//   R_opt_to_link (fixed, ROS REP 103):
//                  [ 0  0  1]    (cam Z = link X)
//                  [-1  0  0]    (cam X = -link Y)
//                  [ 0 -1  0]    (cam Y = -link Z)
//
//   R_link_to_body = R_z(yaw) · R_y(pitch) · R_x(roll)  (extrinsic
//                   XYZ, SDF / urdf convention)
//
//   t_link_in_body = (tx, ty, tz) from SDF <pose> position
//
// Default state: identity transform (R_opt_to_body = I, t = 0).  In
// this state tvec stays in cam optical frame — preserves backward
// compatibility with sentai_calib_task (whose job is to compute
// R_cam_to_body from raw cam-frame tvec, so it needs the untransformed
// input).
//
static float s_cam_t_body[3]    = { 0.0f, 0.0f, 0.0f };
static float s_R_opt_to_body[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};
static uint8_t s_extrinsics_set = 0;   // 0 = identity (passthrough)

static inline void apply_cam_extrinsics_(float tvec[3]) {
    if (!s_extrinsics_set) return;
    const float x = tvec[0], y = tvec[1], z = tvec[2];
    tvec[0] = s_R_opt_to_body[0]*x + s_R_opt_to_body[1]*y
            + s_R_opt_to_body[2]*z + s_cam_t_body[0];
    tvec[1] = s_R_opt_to_body[3]*x + s_R_opt_to_body[4]*y
            + s_R_opt_to_body[5]*z + s_cam_t_body[1];
    tvec[2] = s_R_opt_to_body[6]*x + s_R_opt_to_body[7]*y
            + s_R_opt_to_body[8]*z + s_cam_t_body[2];
}

extern "C" void sentai_markers_set_cam_extrinsics(float tx, float ty, float tz,
                                                     float roll, float pitch,
                                                     float yaw) {
    s_cam_t_body[0] = tx;
    s_cam_t_body[1] = ty;
    s_cam_t_body[2] = tz;

    const float cr = cosf(roll),  sr = sinf(roll);
    const float cp = cosf(pitch), sp = sinf(pitch);
    const float cy = cosf(yaw),   sy = sinf(yaw);

    // R_link_to_body = R_z(yaw) · R_y(pitch) · R_x(roll)  (row-major).
    const float R_l2b[9] = {
        cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
        sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
        -sp,    cp*sr,             cp*cr
    };

    // R_opt_to_link (ROS REP 103, fixed).
    const float R_o2l[9] = {
        0.0f, 0.0f, 1.0f,
       -1.0f, 0.0f, 0.0f,
        0.0f,-1.0f, 0.0f
    };

    // R_opt_to_body = R_l2b · R_o2l.
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum += R_l2b[i*3 + k] * R_o2l[k*3 + j];
            }
            s_R_opt_to_body[i*3 + j] = sum;
        }
    }
    s_extrinsics_set = 1;
}

extern "C" void sentai_markers_clear_cam_extrinsics(void) {
    for (int i = 0; i < 9; ++i) s_R_opt_to_body[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    s_cam_t_body[0] = s_cam_t_body[1] = s_cam_t_body[2] = 0.0f;
    s_extrinsics_set = 0;
}

// Internal WhyCon helpers — live in sentai_aruco.cc alongside the
// WhyCon detection pipeline.  Header-less by design: this dispatcher
// is the only consumer post-W19-T1, so we forward-declare locally.
extern "C" {

typedef struct {
    float    cx, cy, axis_a, axis_b, angle;
    int      comp_id;
    float    tvec_cam[3];
    float    rvec_cam[3];
    float    reproj_err_px;
    uint8_t  pose_valid;
    uint8_t  _pad[3];
} sentai_whycon_marker_internal_t;

int      sentai_whycon_test_synth_krajnik(int n_circles, int radius);
int      sentai_whycon_synth_one(int cx_px, int cy_px, int radius_px);
int      sentai_whycon_get_markers(sentai_whycon_marker_internal_t* out, int cap);
uint32_t sentai_whycon_detect_cyc_last(void);
void     sentai_whycon_set_concentric_check(int on);
void     sentai_whycon_set_diameter(float meters);

uint32_t sentai_aruco_detect_cyc_last(void);

}

// =====================================================================
// Module state.
// =====================================================================

#define SENTAI_MARKERS_MAX_DETS  16

static sentai_markers_backend_t s_backend = SENTAI_MARKERS_BACKEND_NONE;
static SentaiMarkersPose        s_cache[SENTAI_MARKERS_MAX_DETS];
static int                      s_cache_n = 0;
static SentaiMarkersStats       s_stats   = {};

// =====================================================================
// Backend → unified struct translation.
// =====================================================================

static inline void zero_pose_(SentaiMarkersPose* p) {
    memset(p, 0, sizeof(*p));
    p->id = -1;
}

static void aruco_to_unified_(const sentai_aruco_marker_t* in,
                                SentaiMarkersPose* out) {
    zero_pose_(out);
    out->id       = (int32_t)in->marker_id;
    float cx = 0.0f, cy = 0.0f;
    for (int k = 0; k < 4; ++k) {
        cx += in->corners_px[k * 2 + 0];
        cy += in->corners_px[k * 2 + 1];
    }
    out->pixel_cx = cx * 0.25f;
    out->pixel_cy = cy * 0.25f;
    memcpy(out->tvec_cam, in->tvec_cam, sizeof(out->tvec_cam));
    memcpy(out->rvec_cam, in->rvec_cam, sizeof(out->rvec_cam));
    apply_cam_extrinsics_(out->tvec_cam);   // T5: cam-optical → body
    out->reproj_err_px = in->reproj_err_px;
    out->backend       = (uint8_t)SENTAI_MARKERS_BACKEND_ARUCO;
    out->pose_valid    = 1;
}

static void whycon_to_unified_(const sentai_whycon_marker_internal_t* in,
                                 int seq, SentaiMarkersPose* out) {
    zero_pose_(out);
    out->id            = (int32_t)seq;
    out->pixel_cx      = in->cx;
    out->pixel_cy      = in->cy;
    memcpy(out->tvec_cam, in->tvec_cam, sizeof(out->tvec_cam));
    memcpy(out->rvec_cam, in->rvec_cam, sizeof(out->rvec_cam));
    apply_cam_extrinsics_(out->tvec_cam);   // T5: cam-optical → body
    out->reproj_err_px = in->reproj_err_px;
    out->backend       = (uint8_t)SENTAI_MARKERS_BACKEND_WHYCON;
    out->pose_valid    = in->pose_valid;
}

static void cache_from_whycon_(int n) {
    sentai_whycon_marker_internal_t arr[SENTAI_MARKERS_MAX_DETS];
    const int got = sentai_whycon_get_markers(arr, SENTAI_MARKERS_MAX_DETS);
    const int m = (got < n) ? got : n;
    for (int i = 0; i < m && i < SENTAI_MARKERS_MAX_DETS; ++i) {
        whycon_to_unified_(&arr[i], i, &s_cache[i]);
    }
    s_cache_n = (m > SENTAI_MARKERS_MAX_DETS) ? SENTAI_MARKERS_MAX_DETS : m;
}

// =====================================================================
// Public API — lifecycle.
// =====================================================================

extern "C" int sentai_markers_init(sentai_markers_backend_t backend) {
    if (backend != SENTAI_MARKERS_BACKEND_ARUCO &&
        backend != SENTAI_MARKERS_BACKEND_WHYCON) {
        return -1;
    }
    memset(&s_stats, 0, sizeof(s_stats));
    s_cache_n = 0;
    s_backend = backend;
    s_stats.backend = (uint8_t)backend;
    if (backend == SENTAI_MARKERS_BACKEND_ARUCO) {
        return sentai_aruco_init();
    }
    // OP-S10-W19-T4 iter-8b: enable Phase W3 concentric inner-disc
    // validation by default when WhyCon backend is selected.  Without
    // W3, the detector returns TWO blobs per marker (the outer black
    // annulus AND the centre black dot are both dark, both pass the
    // fill-ratio gate).  W3 samples at 0.55·R and 0.95·R from the
    // bbox centre — the centre-dot candidate fails because there's
    // no white inner ring inside it, while the outer-annulus candidate
    // passes (white at 0.55·R, black at 0.95·R = the annulus body).
    sentai_whycon_set_concentric_check(1);
    return 0;
}

extern "C" void sentai_markers_clear(void) {
    sentai_aruco_clear();
    memset(&s_stats, 0, sizeof(s_stats));
    s_cache_n = 0;
    s_stats.backend = (uint8_t)s_backend;
}

extern "C" sentai_markers_backend_t sentai_markers_get_backend(void) {
    return s_backend;
}

// =====================================================================
// Public API — configuration.
// =====================================================================

extern "C" void sentai_markers_set_intrinsics(float fx, float fy,
                                                float cx, float cy) {
    sentai_aruco_set_intrinsics(fx, fy, cx, cy);
}

extern "C" void sentai_markers_set_marker_size(float meters) {
    if (s_backend == SENTAI_MARKERS_BACKEND_WHYCON) {
        sentai_whycon_set_diameter(meters);
    } else {
        (void)sentai_aruco_set_marker_size(meters);
    }
}

// =====================================================================
// Public API — detection on a caller-supplied frame.
// =====================================================================

extern "C" int sentai_markers_detect_frame(const uint8_t* gray, int w, int h,
                                             uint32_t frame_seq,
                                             uint32_t src_ts_ms) {
    s_cache_n = 0;
    if (s_backend == SENTAI_MARKERS_BACKEND_ARUCO) {
        sentai_aruco_marker_t arr[SENTAI_MARKERS_MAX_DETS];
        const int n = sentai_aruco_detect(gray, w, h, frame_seq, src_ts_ms,
                                            arr, SENTAI_MARKERS_MAX_DETS);
        if (n < 0) return n;
        for (int i = 0; i < n && i < SENTAI_MARKERS_MAX_DETS; ++i) {
            aruco_to_unified_(&arr[i], &s_cache[i]);
        }
        s_cache_n = (n > SENTAI_MARKERS_MAX_DETS)
                      ? SENTAI_MARKERS_MAX_DETS : n;
    } else if (s_backend == SENTAI_MARKERS_BACKEND_WHYCON) {
        // W19-T1 / s182 camera-driven path: copy gray into the WhyCon
        // s_test_gray staging via sentai_whycon_detect_buffer, then
        // run the full detect pipeline.  Returns n_dets or negative.
        extern int sentai_whycon_detect_buffer(const uint8_t* g, int w_, int h_);
        const int n = sentai_whycon_detect_buffer(gray, w, h);
        if (n < 0) return n;
        cache_from_whycon_(n);
        (void)frame_seq; (void)src_ts_ms;
    } else {
        return 0;
    }
    s_stats.frames_total++;
    s_stats.markers_total += (uint32_t)s_cache_n;
    if (s_cache_n > 0) s_stats.frames_with_detect++;

    // OP-S10-W19-T4 iter-6: forward the grayscale frame + n_dets to
    // the Flight Recorder.  Silent no-op if the mission hasn't opened
    // the "frames" channel.  Mirrors sentai_safety_task.cc:219 pattern.
    extern int sentai_fr_push_frame(const uint8_t* gray, int w_, int h_,
                                      int n, uint32_t seq, uint32_t ts);
    (void)sentai_fr_push_frame(gray, w, h, s_cache_n, frame_seq, src_ts_ms);

    return s_cache_n;
}

extern "C" int sentai_markers_get_count(void) {
    return s_cache_n;
}

extern "C" int sentai_markers_get_pose(int i, SentaiMarkersPose* out) {
    if (!out || i < 0 || i >= s_cache_n) return 0;
    memcpy(out, &s_cache[i], sizeof(*out));
    return 1;
}

extern "C" int sentai_markers_get_stats(SentaiMarkersStats* out) {
    if (!out) return 0;
    memcpy(out, &s_stats, sizeof(*out));
    return 1;
}

extern "C" int sentai_markers_get_latest(int i, SentaiMarkersPose* out) {
    return sentai_markers_get_pose(i, out);
}

extern "C" uint32_t sentai_markers_detect_cyc_last(void) {
    if (s_backend == SENTAI_MARKERS_BACKEND_ARUCO) {
        return sentai_aruco_detect_cyc_last();
    } else if (s_backend == SENTAI_MARKERS_BACKEND_WHYCON) {
        return sentai_whycon_detect_cyc_last();
    }
    return 0;
}

// =====================================================================
// SIM-only synth + detect.  Draws ONE Krajník marker at an arbitrary
// pixel position with the given outer radius, runs the active
// backend's detector (only WhyCon makes sense for circular markers),
// and caches the result.  Returns n_dets, or -1 on inactive/wrong
// backend.  Used by s181 to sweep known world positions (X, Y, Z)
// through the closed-form PnP and recover an estimated pose for
// thesis-grade plots.
// =====================================================================

extern "C" int sentai_markers_synth_one_whycon(int cx_px, int cy_px,
                                                 int radius_px) {
    if (s_backend != SENTAI_MARKERS_BACKEND_WHYCON) return -1;
    s_cache_n = 0;
    const int n = sentai_whycon_synth_one(cx_px, cy_px, radius_px);
    if (n < 0) return n;
    cache_from_whycon_(n);
    s_stats.frames_total++;
    s_stats.markers_total += (uint32_t)s_cache_n;
    if (s_cache_n > 0) s_stats.frames_with_detect++;
    return s_cache_n;
}

// =====================================================================
// OP-S10-W19-T6b: drone-pose recovery from multi-marker Kabsch + yaw-anchor.
// Host-side reference: verdict_sota.py::kabsch_with_assignment (assignment
// search + Z-plane reflection) + the iter-5 yaw-anchor flip at lines
// 443-468.  See feedback_yaw_anchor_mirror_picker.md for the why.
// =====================================================================

static float s_world_xyz[SENTAI_MARKERS_MAX_WORLD][3];
static int   s_world_n = 0;

// Test-only: bypass the detector pipeline and inject synthetic
// observations (body-frame tvecs) directly into the cache.  Used by
// s184 (W19-T6b smoke) to validate the Kabsch+yaw-anchor pipeline
// against analytic ground truth without going through the WhyCon pixel
// renderer.  Production callers MUST NOT use this; if you find a call
// site outside experiments/sNNN_*/, that is a bug.
extern "C" int sentai_markers_test_inject_obs(int n, const float* tvec_n3) {
    if (n < 0 || n > SENTAI_MARKERS_MAX_DETS) return -1;
    if (n > 0 && !tvec_n3) return -1;
    s_cache_n = n;
    for (int i = 0; i < n; ++i) {
        zero_pose_(&s_cache[i]);
        s_cache[i].id            = i;
        s_cache[i].tvec_cam[0]   = tvec_n3[3*i + 0];
        s_cache[i].tvec_cam[1]   = tvec_n3[3*i + 1];
        s_cache[i].tvec_cam[2]   = tvec_n3[3*i + 2];
        s_cache[i].pose_valid    = 1;
        s_cache[i].backend       = (uint8_t)s_backend;
    }
    return 0;
}

extern "C" int sentai_markers_set_marker_world(int n, const float* xyz_n3) {
    if (n < 0 || n > SENTAI_MARKERS_MAX_WORLD) return -1;
    if (n > 0 && !xyz_n3) return -1;
    for (int i = 0; i < n; ++i) {
        s_world_xyz[i][0] = xyz_n3[3*i + 0];
        s_world_xyz[i][1] = xyz_n3[3*i + 1];
        s_world_xyz[i][2] = xyz_n3[3*i + 2];
    }
    s_world_n = n;
    return 0;
}

extern "C" int sentai_markers_get_marker_world_count(void) {
    return s_world_n;
}

namespace {

// Recursive permutation enumerator: fills perm[0..n_obs-1] with each
// ordered choice of n_obs distinct indices from {0..n_world-1}, runs
// the Kabsch evaluator on each one, tracks the min-residual fit.
struct PermCtx {
    int          n_world;
    int          n_obs;
    const float* p_cam_flat;     // [3 * n_obs], row-major
    int          perm[SENTAI_MARKERS_MAX_WORLD];
    int          used[SENTAI_MARKERS_MAX_WORLD];

    float best_R[9];
    float best_t[3];
    float best_res;
    int   best_perm[SENTAI_MARKERS_MAX_WORLD];
    int   have_best;
};

void recurse_pick(PermCtx& c, int depth) {
    if (depth == c.n_obs) {
        // Build p_world from current perm.
        float p_world[3 * SENTAI_MARKERS_MAX_WORLD];
        for (int i = 0; i < c.n_obs; ++i) {
            const int w = c.perm[i];
            p_world[3*i + 0] = s_world_xyz[w][0];
            p_world[3*i + 1] = s_world_xyz[w][1];
            p_world[3*i + 2] = s_world_xyz[w][2];
        }
        float R[9], t[3], res_max;
        if (sentai_kabsch_align(c.p_cam_flat, p_world, c.n_obs,
                                R, t, &res_max) != 0) {
            return;
        }
        if (!c.have_best || res_max < c.best_res) {
            memcpy(c.best_R, R, sizeof(R));
            memcpy(c.best_t, t, sizeof(t));
            c.best_res = res_max;
            memcpy(c.best_perm, c.perm, sizeof(c.perm));
            c.have_best = 1;
        }
        return;
    }
    for (int i = 0; i < c.n_world; ++i) {
        if (c.used[i]) continue;
        c.perm[depth]  = i;
        c.used[i]      = 1;
        recurse_pick(c, depth + 1);
        c.used[i]      = 0;
    }
}

}  // namespace

extern "C" int sentai_markers_get_drone_pose(float cf2_yaw_rad,
                                              SentaiMarkersDronePose* out) {
    if (!out) return 0;
    if (s_cache_n < 3 || s_world_n < s_cache_n) return 0;

    // Snapshot observations: tvecs are already in BODY frame post-T5
    // extrinsics, so Kabsch fits body -> world directly.
    float p_cam[3 * SENTAI_MARKERS_MAX_WORLD];
    const int n_obs = (s_cache_n > SENTAI_MARKERS_MAX_WORLD)
                          ? SENTAI_MARKERS_MAX_WORLD : s_cache_n;
    for (int i = 0; i < n_obs; ++i) {
        if (!s_cache[i].pose_valid) return 0;
        p_cam[3*i + 0] = s_cache[i].tvec_cam[0];
        p_cam[3*i + 1] = s_cache[i].tvec_cam[1];
        p_cam[3*i + 2] = s_cache[i].tvec_cam[2];
    }

    PermCtx c;
    memset(&c, 0, sizeof(c));
    c.n_world    = s_world_n;
    c.n_obs      = n_obs;
    c.p_cam_flat = p_cam;
    recurse_pick(c, 0);
    if (!c.have_best) return 0;

    float* R = c.best_R;
    float* t = c.best_t;

    // Z-plane reflection: if Kabsch put the drone below the marker
    // plane (unconstrained 3D fit has a 2-fold ambiguity for coplanar
    // points), reflect t around mean(world_z) of the winning perm.
    float z_mean = 0.0f;
    for (int i = 0; i < n_obs; ++i) {
        z_mean += s_world_xyz[c.best_perm[i]][2];
    }
    z_mean /= (float)n_obs;
    uint8_t flip_z = 0;
    if (t[2] < z_mean) {
        t[2]   = 2.0f * z_mean - t[2];
        flip_z = 1;
    }

    // Yaw-anchored X/Y mirror flip: symmetric pads (square, hex)
    // admit 4 valid det=+1 Kabsch rotations.  Use sign of cos(cf2_yaw)
    // against R diagonal to detect which mirror was picked; flip t
    // around the marker centroid on each mismatched axis.
    const float cos_yaw = cosf(cf2_yaw_rad);
    const uint8_t flip_x = (R[0] * cos_yaw < 0.0f) ? 1 : 0;
    const uint8_t flip_y = (R[4] * cos_yaw < 0.0f) ? 1 : 0;
    if (flip_x || flip_y) {
        float mx = 0.0f, my = 0.0f;
        for (int i = 0; i < n_obs; ++i) {
            mx += s_world_xyz[c.best_perm[i]][0];
            my += s_world_xyz[c.best_perm[i]][1];
        }
        mx /= (float)n_obs;
        my /= (float)n_obs;
        if (flip_x) t[0] = 2.0f * mx - t[0];
        if (flip_y) t[1] = 2.0f * my - t[1];
    }

    out->x       = t[0];
    out->y       = t[1];
    out->z       = t[2];
    // Yaw recovery accounting for the X/Y mirror disambiguation: if the
    // SVD landed on the 180-X or 180-Y or 180-Z branch, R itself encodes
    // that wrong rotation.  R_true = diag(s_x, s_y, s_z) @ R_wrong where
    // s_i = -1 iff flip_i (and s_z = s_x * s_y to keep det = +1, which
    // is already enforced by Kabsch).  For yaw extraction we only need
    // the top-left 2x2 corner: yaw_true = atan2(s_y*R[3], s_x*R[0]).
    const float r10 = flip_y ? -R[3] : R[3];
    const float r00 = flip_x ? -R[0] : R[0];
    out->yaw_rad = atan2f(r10, r00);
    out->res_max = c.best_res;
    out->n_used  = n_obs;
    out->flip_x  = flip_x;
    out->flip_y  = flip_y;
    out->flip_z  = flip_z;
    out->_pad    = 0;
    return 1;
}

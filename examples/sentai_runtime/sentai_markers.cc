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
    out->reproj_err_px = in->reproj_err_px;
    out->backend       = (uint8_t)SENTAI_MARKERS_BACKEND_WHYCON;
    out->pose_valid    = in->pose_valid;
}

static void cache_from_aruco_(int n) {
    sentai_aruco_marker_t arr[SENTAI_MARKERS_MAX_DETS];
    const int got = sentai_aruco_get_latest(arr, SENTAI_MARKERS_MAX_DETS);
    const int m = (got < n) ? got : n;
    for (int i = 0; i < m && i < SENTAI_MARKERS_MAX_DETS; ++i) {
        aruco_to_unified_(&arr[i], &s_cache[i]);
    }
    s_cache_n = (m > SENTAI_MARKERS_MAX_DETS) ? SENTAI_MARKERS_MAX_DETS : m;
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

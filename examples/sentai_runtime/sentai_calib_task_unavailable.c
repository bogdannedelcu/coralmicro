// sentai_calib_task_unavailable.c -- shared unavailable legacy calib worker.
//
// Link this in runtimes that expose sentai.calib without the legacy
// Flow-autotune worker from sentai_calib_task.cc.  Synchronous math,
// persistence, and helper functions remain in sentai_calib.cc.

#include "sentai_calib.h"

#define SENTAI_CALIB_TASK_UNAVAILABLE (-1)

int sentai_calib_set_context(float z_hold_m,
                             float marker_grid_dx_m,
                             float marker_grid_dy_m,
                             float marker_size_m) {
    (void)z_hold_m;
    (void)marker_grid_dx_m;
    (void)marker_grid_dy_m;
    (void)marker_size_m;
    return SENTAI_CALIB_TASK_UNAVAILABLE;
}

float sentai_calib_get_kp(sentai_calib_axis_t axis) {
    (void)axis;
    return -1.0f;
}

uint32_t sentai_calib_get_td_ms(void) {
    return 0;
}

int sentai_calib_task_start(sentai_calib_axis_t axis,
                            float dur_s,
                            float vmax_m_s) {
    (void)axis;
    (void)dur_s;
    (void)vmax_m_s;
    return SENTAI_CALIB_TASK_UNAVAILABLE;
}

int sentai_calib_task_stop(void) {
    return 0;
}

int sentai_calib_task_is_done(void) {
    return 1;
}

int sentai_calib_hold_start(float kp_x, float kp_y,
                            float vmax_clip, float dur_s) {
    (void)kp_x;
    (void)kp_y;
    (void)vmax_clip;
    (void)dur_s;
    return SENTAI_CALIB_TASK_UNAVAILABLE;
}

int sentai_calib_hold_yaw_start(float kp_x, float kp_y,
                                float vmax_clip, float dur_s,
                                float yaw_rate_deg_s) {
    (void)kp_x;
    (void)kp_y;
    (void)vmax_clip;
    (void)dur_s;
    (void)yaw_rate_deg_s;
    return SENTAI_CALIB_TASK_UNAVAILABLE;
}

float sentai_calib_get_hold_max_drift_m(void) {
    return 0.0f;
}

float sentai_calib_get_hold_rms_drift_m(void) {
    return 0.0f;
}

// sentai_servo_marker_task.h -- TD-S10-B4 image-frame marker-control task.
//
// B4 consumes the strict /system/calib.ini produced by the A3/B3 calibration
// task and then performs marker-centered bootstrap and later Generic Hover
// image-frame control.  MicroPython remains only the phase orchestrator.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SENTAI_SERVO_MARKER_TASK_PERIOD_MS
#define SENTAI_SERVO_MARKER_TASK_PERIOD_MS 33
#endif

typedef enum {
    SENTAI_SERVO_MARKER_PHASE_IDLE = 0,
    SENTAI_SERVO_MARKER_PHASE_SETUP = 1,
    SENTAI_SERVO_MARKER_PHASE_ACQUIRE = 2,
    SENTAI_SERVO_MARKER_PHASE_CENTER_HOLD = 3,
    SENTAI_SERVO_MARKER_PHASE_EXTPOS_WARMUP = 4,
    SENTAI_SERVO_MARKER_PHASE_HANDOFF_HOVER = 5,
    SENTAI_SERVO_MARKER_PHASE_IMAGE_AXIS_MOTION = 6,
    SENTAI_SERVO_MARKER_PHASE_LAND = 7,
    SENTAI_SERVO_MARKER_PHASE_DONE = 8,
    SENTAI_SERVO_MARKER_PHASE_STOPPED = 9,
    SENTAI_SERVO_MARKER_PHASE_FAULTED = 10,
} sentai_servo_marker_phase_t;

typedef struct {
    uint8_t phase;
    uint8_t started;
    uint8_t done;
    uint8_t ok;
    int rc;
    int reason_code;

    uint8_t setup_ok;
    uint8_t calib_loaded;
    uint8_t calib_strict_ok;
    uint8_t axis_seed_ok;
    int camera_rc;
    int crazy_rc;
    int flow_rc;
    int pose_subscribe_rc;
    float roll_vec_px[2];
    float pitch_vec_px[2];
    float extpos_signs[3];
    float extpos_origin_corr_m[3];
    float cam_offset_B[3];

    uint8_t acquire_ok;
    int acquire_ticks_done;
    int acquire_last_thrust_u16;
    int acquire_n_full_max;
    float acquire_avg_full_markers;
    float acquire_radius_max_px;
    float acquire_z_cam_max_m;
    float acquire_z_target_m;

    uint8_t center_ok;
    int center_ticks_done;
    int center_n_full_min;
    float center_avg_full_markers;
    float center_err_initial_px;
    float center_err_last_px;
    float center_err_max_px;
    int center_thrust_last_u16;

    uint8_t extpos_ok;
    uint8_t extpos_converged;
    uint8_t extpos_diverged;
    uint8_t extpos_marker_visibility_lost;
    int extpos_ticks_done;
    int extpos_send_ok;
    int extpos_pose_rejects;
    int extpos_flow_send_ok;
    int extpos_flow_read_errors;
    int extpos_kalman_reset_rc;
    int extpos_stddev_bootstrap_rc;
    int extpos_stddev_flow_rc;
    int extpos_n_full_min;
    float extpos_n_full_recent_avg;
    float extpos_last_pose_m[3];
    float extpos_last_sent_m[3];
    float extpos_last_est_m[3];
    float extpos_err_first_m;
    float extpos_err_last_m;
    float extpos_err_recent_mean_m;
    float extpos_err_min_m;
    float extpos_err_max_m;
    int extpos_reject_reason_code;

    uint8_t handoff_ok;
    int handoff_release_rc;
    int handoff_hover_rc_last;
    int handoff_ticks_done;
    int handoff_extpos_send_ok;
    int handoff_pose_rejects;
    int handoff_flow_send_ok;
    int handoff_flow_read_errors;
    int handoff_n_full_min;
    float handoff_n_full_recent_avg;
    float handoff_hover_z_m;
    float handoff_last_est_m[3];

    uint8_t axis_motion_ok;
    int axis_segments_done;
    int axis_ticks_done;
    int axis_extpos_send_ok;
    int axis_pose_rejects;
    int axis_flow_send_ok;
    int axis_flow_read_errors;
    int axis_n_full_min;
    float axis_n_full_recent_avg;
    float axis_last_err_px;
    float axis_envelope_px[4];

    uint8_t land_ok;
    int land_release_rc;
    int land_hover_rc_last;
    int land_disarm_rc;
    int land_ticks_done;
    int land_extpos_send_ok;
    int land_pose_rejects;
    int land_flow_send_ok;
    int land_flow_read_errors;
    int land_n_full_min;
    float land_n_full_recent_avg;
    float land_start_z_m;
    float land_last_z_cmd_m;
    float land_last_est_m[3];
} sentai_servo_marker_status_t;

int sentai_servo_marker_setup_start(void);
int sentai_servo_marker_acquire_start(void);
int sentai_servo_marker_center_hold_start(void);
int sentai_servo_marker_extpos_warmup_start(void);
int sentai_servo_marker_handoff_hover_start(void);
int sentai_servo_marker_axis_motion_start(void);
int sentai_servo_marker_land_start(void);
int sentai_servo_marker_task_stop(void);
int sentai_servo_marker_task_is_done(void);
int sentai_servo_marker_task_result(int* ok_out, int* phase_out);
int sentai_servo_marker_task_get_status(sentai_servo_marker_status_t* out);

#ifdef __cplusplus
}
#endif

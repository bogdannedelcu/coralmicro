// sentai_calib_orientation_task.h — image-frame orientation calibration worker.
//
// This is the new task home for the s205/TD-S10 calibration flow.  Keep
// blocking loops, camera sampling cadence, and repeated actuator phases here;
// leave sentai_calib.cc as synchronous math/config/persistence primitives.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS
#define SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS 33
#endif

typedef enum {
    SENTAI_CALIB_ORIENTATION_PHASE_IDLE = 0,
    SENTAI_CALIB_ORIENTATION_PHASE_PREFLIGHT_FEATURES = 1,
    SENTAI_CALIB_ORIENTATION_PHASE_ARM_ZERO_UNLOCK = 2,
    SENTAI_CALIB_ORIENTATION_PHASE_MARKER_ACQUISITION = 3,
    SENTAI_CALIB_ORIENTATION_PHASE_POST_LOCK_BRAKE = 4,
    SENTAI_CALIB_ORIENTATION_PHASE_VISUAL_Z_HOLD = 5,
    SENTAI_CALIB_ORIENTATION_PHASE_AXIS_RESPONSE_SMOKE = 6,
    SENTAI_CALIB_ORIENTATION_PHASE_CENTROID_VALIDATION = 7,
    SENTAI_CALIB_ORIENTATION_PHASE_FINAL_CANDIDATE_VALIDATION = 8,
    SENTAI_CALIB_ORIENTATION_PHASE_FINAL_RECENTER = 9,
    SENTAI_CALIB_ORIENTATION_PHASE_MANUAL_DESCEND_DISARM = 10,
    SENTAI_CALIB_ORIENTATION_PHASE_CENTER_HOLD_DESCEND_DISARM = 11,
    SENTAI_CALIB_ORIENTATION_PHASE_DONE = 12,
    SENTAI_CALIB_ORIENTATION_PHASE_STOPPED = 13,
    SENTAI_CALIB_ORIENTATION_PHASE_FAULTED = 14,
} sentai_calib_orientation_phase_t;

typedef enum {
    SENTAI_CALIB_ORIENTATION_STATUS_IDLE = 0,
    SENTAI_CALIB_ORIENTATION_STATUS_RUNNING = 1,
    SENTAI_CALIB_ORIENTATION_STATUS_OK = 2,
    SENTAI_CALIB_ORIENTATION_STATUS_STOPPED = 3,
    SENTAI_CALIB_ORIENTATION_STATUS_FAULTED = 4,
} sentai_calib_orientation_status_code_t;

typedef struct {
    uint8_t phase;
    uint8_t status;
    uint8_t started;
    uint8_t done;
    int ok_full_ticks;
    int samples;
    int n_full_max;
    float radius_max_px;
    float z_cam_max_m;
    uint8_t feature_lock;
    int arm_rc;
    int arm_retry_rc;
    int zero_packets;
    uint8_t acq_locked;
    int acq_last_thrust_u16;
    int acq_lock_streak;
    int acq_n_full_max;
    float acq_radius_max_px;
    float acq_z_cam_min_m;
    float acq_z_cam_max_m;
    float acq_z_cam_last_m;
    int acq_first_seen_tick;
    uint8_t post_ok;
    int post_abort_code;
    int post_valid_ticks;
    float post_z_cam_min_m;
    float post_z_cam_max_m;
    float post_z_cam_last_m;
    float post_vz_filt_m_s;
    int post_thrust_last_u16;
    int post_ok_vz_ticks;
    uint8_t zhold_ok;
    int zhold_abort_code;
    uint8_t zhold_target_reached;
    uint8_t zhold_target_reached_last;
    uint8_t zhold_target_reached_peak;
    int zhold_valid_ticks;
    int zhold_ticks_requested;
    float zhold_z_cam_min_m;
    float zhold_z_cam_max_m;
    float zhold_z_cam_last_m;
    float zhold_vz_filt_m_s;
    int zhold_thrust_min_u16;
    int zhold_thrust_max_u16;
    int zhold_thrust_last_u16;
    uint8_t axis_ok;
    int axis_abort_code;
    int axis_results_count;
    float axis_pitch_comp_dx;
    float axis_pitch_comp_dy;
    float axis_roll_comp_dx;
    float axis_roll_comp_dy;
    int axis_pitch_dominant_axis_code;
    int axis_pitch_dominant_sign;
    float axis_pitch_dominance_ratio;
    float axis_pitch_response_strength_px;
    uint8_t axis_pitch_sign_ok;
    int axis_pitch_min_full_markers;
    float axis_pitch_avg_full_markers;
    uint8_t axis_pitch_marker_lock_ok;
    float axis_pitch_return_err_px;
    uint8_t axis_pitch_return_ok;
    float axis_pitch_z_last_m;
    int axis_roll_dominant_axis_code;
    int axis_roll_dominant_sign;
    float axis_roll_dominance_ratio;
    float axis_roll_response_strength_px;
    uint8_t axis_roll_sign_ok;
    int axis_roll_min_full_markers;
    float axis_roll_avg_full_markers;
    uint8_t axis_roll_marker_lock_ok;
    float axis_roll_return_err_px;
    uint8_t axis_roll_return_ok;
    float axis_roll_z_last_m;
    uint8_t axis_orthogonality_present;
    float axis_orthogonality_dot_norm;
    uint8_t axis_orthogonality_ok;
    int axis_thrust_last_u16;
    uint8_t centroid_ok;
    int centroid_abort_code;
    int centroid_ticks_done;
    int centroid_n_full_min;
    float centroid_avg_full_markers;
    uint8_t centroid_marker_lock_ok;
    float centroid_initial_err_px;
    float centroid_final_err_px;
    float centroid_min_err_px;
    float centroid_max_err_px;
    float centroid_improvement_px;
    int centroid_thrust_last_u16;
    float centroid_first_roll_deg;
    float centroid_first_pitch_deg;
    float centroid_first_err_px;
    float centroid_last_roll_deg;
    float centroid_last_pitch_deg;
    float centroid_last_err_px;
    uint8_t final_val_ok;
    int final_val_abort_code;
    int final_val_thrust_last_u16;
    uint8_t final_pitch_ok;
    int final_pitch_attempts;
    float final_pitch_pulse_deg;
    int final_pitch_expected_axis_code;
    int final_pitch_expected_sign;
    int final_pitch_observed_axis_code;
    int final_pitch_observed_sign;
    float final_pitch_dominance_ratio;
    float final_pitch_response_strength_px;
    float final_pitch_noise_gate_px;
    float final_pitch_comp_dx;
    float final_pitch_comp_dy;
    float final_pitch_return_err_px;
    int final_pitch_min_full_markers;
    float final_pitch_avg_full_markers;
    uint8_t final_pitch_marker_lock_ok;
    uint8_t final_pitch_consistent;
    uint8_t final_pitch_observable;
    uint8_t final_roll_ok;
    int final_roll_attempts;
    float final_roll_pulse_deg;
    int final_roll_expected_axis_code;
    int final_roll_expected_sign;
    int final_roll_observed_axis_code;
    int final_roll_observed_sign;
    float final_roll_dominance_ratio;
    float final_roll_response_strength_px;
    float final_roll_noise_gate_px;
    float final_roll_comp_dx;
    float final_roll_comp_dy;
    float final_roll_return_err_px;
    int final_roll_min_full_markers;
    float final_roll_avg_full_markers;
    uint8_t final_roll_marker_lock_ok;
    uint8_t final_roll_consistent;
    uint8_t final_roll_observable;
    uint8_t recenter_ok;
    int recenter_abort_code;
    uint8_t recenter_recenter_ok;
    uint8_t recenter_hover_ok;
    int recenter_ticks_done;
    int recenter_hover_ticks;
    int recenter_hover_stable_ticks;
    int recenter_n_full_min;
    float recenter_avg_full_markers;
    float recenter_initial_err_px;
    float recenter_final_err_px;
    float recenter_min_err_px;
    float recenter_max_err_px;
    float recenter_improvement_px;
    float recenter_hover_err_max_px;
    float recenter_hover_err_min_px;
    float recenter_hover_err_last_px;
    int recenter_thrust_last_u16;
    float recenter_first_roll_deg;
    float recenter_first_pitch_deg;
    float recenter_first_err_px;
    float recenter_last_roll_deg;
    float recenter_last_pitch_deg;
    float recenter_last_err_px;
    uint8_t manual_disarmed;
    int manual_disarm_rc;
    int manual_ticks_done;
    int manual_thrust_last_u16;
    uint8_t center_hold_ok;
    int center_hold_trigger_code;
    int center_hold_n_full_min;
    float center_hold_avg_full_markers;
    int center_hold_avg_window;
    float center_hold_err_max_px;
    float center_hold_err_last_px;
    int center_hold_descent_pause_ticks;
    int center_hold_ticks_done;
    float center_hold_z_cam_min_m;
    float center_hold_z_cam_max_m;
    float center_hold_z_cam_last_m;
    float center_hold_vz_filt_last_m_s;
    float center_hold_target_vz_m_s;
    int center_hold_thrust_last_u16;
    uint8_t center_hold_disarmed;
    int center_hold_disarm_rc;
    float center_hold_deadband_px;
    uint8_t center_hold_noise_ok;
    float center_hold_noise_sigma_px;
} sentai_calib_orientation_status_t;

int sentai_calib_orientation_task_start(void);
int sentai_calib_orientation_arm_zero_start(void);
int sentai_calib_orientation_marker_acquisition_start(void);
int sentai_calib_orientation_post_lock_brake_start(void);
int sentai_calib_orientation_visual_z_hold_start(void);
int sentai_calib_orientation_axis_response_start(void);
int sentai_calib_orientation_centroid_validation_start(void);
int sentai_calib_orientation_score_candidate_from_axis(void);
int sentai_calib_orientation_optical_axis_validate(void);
int sentai_calib_orientation_final_candidate_validation_start(void);
int sentai_calib_orientation_final_recenter_start(void);
int sentai_calib_orientation_manual_descend_start(void);
int sentai_calib_orientation_center_hold_descend_start(void);
int sentai_calib_orientation_save_contract(const char* status,
                                           int accepted);
int sentai_calib_orientation_emergency_stop(void);
int sentai_calib_orientation_task_stop(void);
int sentai_calib_orientation_task_is_done(void);
int sentai_calib_orientation_task_result(int* ok_out,
                                         int* thrust_last_out,
                                         int* disarmed_out);
int sentai_calib_orientation_task_get_status(
        sentai_calib_orientation_status_t* out);

#ifdef __cplusplus
}
#endif

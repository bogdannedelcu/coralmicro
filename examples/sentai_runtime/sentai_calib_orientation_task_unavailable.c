// sentai_calib_orientation_task_unavailable.c -- shared unavailable backend.
//
// Link this in runtimes that expose sentai.calib without the image-frame
// orientation worker from sentai_calib_orientation_task.cc.

#include "sentai_calib_orientation_task.h"

#include <string.h>

#define SENTAI_CALIB_ORIENTATION_UNAVAILABLE (-1)

static sentai_calib_orientation_status_t s_unavailable_status(void) {
    sentai_calib_orientation_status_t st;
    memset(&st, 0, sizeof(st));
    st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
    st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
    st.started = 0;
    st.done = 1;
    return st;
}

int sentai_calib_orientation_task_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_arm_zero_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_marker_acquisition_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_post_lock_brake_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_visual_z_hold_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_axis_response_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_centroid_validation_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_score_candidate_from_axis(void) {
    return 0;
}

int sentai_calib_orientation_optical_axis_validate(void) {
    return 0;
}

int sentai_calib_orientation_final_candidate_validation_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_final_recenter_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_manual_descend_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_center_hold_descend_start(void) {
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_save_contract(const char* status, int accepted) {
    (void)status;
    (void)accepted;
    return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
}

int sentai_calib_orientation_emergency_stop(void) {
    return 0;
}

int sentai_calib_orientation_task_stop(void) {
    return 0;
}

int sentai_calib_orientation_task_is_done(void) {
    return 1;
}

int sentai_calib_orientation_task_result(int* ok_out,
                                         int* thrust_last_out,
                                         int* disarmed_out) {
    if (ok_out) *ok_out = 0;
    if (thrust_last_out) *thrust_last_out = 0;
    if (disarmed_out) *disarmed_out = 0;
    return 1;
}

int sentai_calib_orientation_current_thrust(void) {
    return 0;
}

float sentai_calib_orientation_runtime_z_target_m(void) {
    return 0.0f;
}

int sentai_calib_orientation_task_get_status(
        sentai_calib_orientation_status_t* out) {
    if (!out) return SENTAI_CALIB_ORIENTATION_UNAVAILABLE;
    *out = s_unavailable_status();
    return 1;
}

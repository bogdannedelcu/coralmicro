// sentai_servo_marker_task_unavailable.c -- shared unavailable marker backend.
//
// Link this in runtimes that expose sentai.servo without the camera/markers
// worker from sentai_servo_marker_task.cc.  The servo FSM stays fully shared;
// only the continuous marker-control task reports unavailable.

#include "sentai_servo_marker_task.h"

#include <string.h>

#define SENTAI_SERVO_MARKER_TASK_UNAVAILABLE (-1)

static sentai_servo_marker_status_t s_unavailable_status(void) {
    sentai_servo_marker_status_t st;
    memset(&st, 0, sizeof(st));
    st.phase = SENTAI_SERVO_MARKER_PHASE_FAULTED;
    st.done = 1;
    st.ok = 0;
    st.rc = SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
    st.reason_code = SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
    return st;
}

int sentai_servo_marker_setup_start(void) {
    return SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
}

int sentai_servo_marker_acquire_start(void) {
    return SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
}

int sentai_servo_marker_center_hold_start(void) {
    return SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
}

int sentai_servo_marker_extpos_warmup_start(void) {
    return SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
}

int sentai_servo_marker_handoff_hover_start(void) {
    return SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
}

int sentai_servo_marker_axis_motion_start(void) {
    return SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
}

int sentai_servo_marker_land_start(void) {
    return SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
}

int sentai_servo_marker_task_stop(void) {
    return 0;
}

int sentai_servo_marker_task_is_done(void) {
    return 1;
}

int sentai_servo_marker_task_result(int* ok_out, int* phase_out) {
    if (ok_out) *ok_out = 0;
    if (phase_out) *phase_out = SENTAI_SERVO_MARKER_PHASE_FAULTED;
    return 1;
}

int sentai_servo_marker_task_get_status(sentai_servo_marker_status_t* out) {
    if (!out) return SENTAI_SERVO_MARKER_TASK_UNAVAILABLE;
    *out = s_unavailable_status();
    return 0;
}

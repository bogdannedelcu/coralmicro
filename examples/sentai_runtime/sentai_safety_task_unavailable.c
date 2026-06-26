// sentai_safety_task_unavailable.c -- shared unavailable SafetyTask backend.
//
// Link this only in runtimes that expose sentai.safety but do not include the
// camera/markers worker from sentai_safety_task.cc.  The safety state machine
// remains fully available through sentai_safety.cc; only the continuous worker
// is reported unavailable.

#include "sentai_safety.h"
#include "sentai_safety_task.h"

#include <string.h>

int sentai_safety_task_start(void) {
    return SENTAI_SAFETY_ERR_STATE;
}

int sentai_safety_task_stop(void) {
    return SENTAI_SAFETY_OK;
}

int sentai_safety_task_get_stats(sentai_safety_task_stats_t* out) {
    if (!out) return SENTAI_SAFETY_ERR_PARAMS;
    memset(out, 0, sizeof(*out));
    out->health = SENTAI_SAFETY_TASK_UNAVAILABLE;
    return SENTAI_SAFETY_OK;
}

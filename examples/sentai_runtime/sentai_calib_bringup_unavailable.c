// sentai_calib_bringup_unavailable.c -- shared unavailable bringup backend.
//
// Link this in runtimes that expose sentai.calib without the legacy bringup
// orchestrator task from sentai_calib_bringup.cc.

#include "sentai_calib_bringup.h"

#include <string.h>

#define SENTAI_CALIB_BRINGUP_UNAVAILABLE (-1)

static sentai_calib_bringup_result_t s_unavailable_result(void) {
    sentai_calib_bringup_result_t r;
    memset(&r, 0, sizeof(r));
    r.accepted = 0;
    r.reject_code = SENTAI_CALIB_BRINGUP_REJ_INVALID_CTX;
    r.last_phase = SENTAI_CALIB_BRINGUP_PHASE_DONE_FAIL;
    return r;
}

int sentai_calib_bringup_start(const sentai_calib_bringup_ctx_t* ctx) {
    (void)ctx;
    return SENTAI_CALIB_BRINGUP_UNAVAILABLE;
}

int sentai_calib_bringup_is_done(void) {
    return 1;
}

sentai_calib_bringup_phase_t sentai_calib_bringup_get_phase(void) {
    return SENTAI_CALIB_BRINGUP_PHASE_DONE_FAIL;
}

int sentai_calib_bringup_get_result(sentai_calib_bringup_result_t* out) {
    if (!out) return SENTAI_CALIB_BRINGUP_UNAVAILABLE;
    *out = s_unavailable_result();
    return 0;
}

int sentai_calib_bringup_abort(void) {
    return 0;
}

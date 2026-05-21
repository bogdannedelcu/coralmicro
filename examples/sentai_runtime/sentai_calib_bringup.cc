// sentai_calib_bringup.cc — OP-S10-W21-T4 orchestrator worker.
//
// Header sentai_calib_bringup.h documents the contract + fault model.
// This file is plumbing: a single FreeRTOS task that walks the phase
// sequence and stitches together the existing primitives:
//
//   SAMPLE   : own loop — hover() pulses for sweep diversity,
//              sentai_markers_get_latest + sentai_crazy_pose readers,
//              build sentai_calib_sample_t[] up to SAMPLES_MAX.
//   KABSCH   : sentai_calib_run_kabsch, then per-sample mean residual
//              gives cam_offset_B.  No persistence here (RAM only).
//   AUTOTUNE : sentai_calib_task_start(axis, ...) + task_is_done poll;
//              the inner worker self-deletes when done.  task_stop()
//              between phases drains s_started so the next start works.
//   HOLD     : sentai_calib_hold_start(kp_x, kp_y, ...) + poll.
//   SAVE     : sentai_calib_commit_R + commit_kp + save (INI v2).
//
// Anti-cheat: only sentai_markers_get_latest (PnP) and sentai_crazy_pose
// (cf2 EKF telemetry) — no GT consumption.  See [[sentai-sim-air-gapped-
// from-truth]].
//
// Routed to .sentai_slow per linker map; cold-path orchestrator.

#include "sentai_calib_bringup.h"
#include "sentai_calib.h"
#include "sentai_calib_task.h"
#include "sentai_markers.h"
#include "sentai_crazy.h"
#include "sentai_crazy_log.h"
#include "sentai_safety.h"
#include "sentai_fr.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

extern "C" uint32_t sentai_now_ms(void) __attribute__((weak));

namespace {

// ── Static state ────────────────────────────────────────────────────────
static sentai_calib_bringup_ctx_t      s_ctx;
static sentai_calib_bringup_result_t   s_result;
static sentai_calib_sample_t           s_samples[SENTAI_CALIB_BRINGUP_SAMPLES_MAX];
static volatile sentai_calib_bringup_phase_t s_phase = SENTAI_CALIB_BRINGUP_PHASE_IDLE;
static volatile bool      s_done    = false;
static volatile bool      s_running = false;
static TaskHandle_t       s_handle  = nullptr;
static EventGroupHandle_t s_stop_evt = nullptr;
#define STOP_BIT 0x01

inline uint32_t now_ms_() {
    if (sentai_now_ms) return sentai_now_ms();
    return xTaskGetTickCount() * (1000U / configTICK_RATE_HZ);
}

inline bool aborted_() {
    return s_stop_evt &&
           ((xEventGroupGetBits(s_stop_evt) & STOP_BIT) != 0);
}

// ── Validation ─────────────────────────────────────────────────────────
int validate_ctx_(const sentai_calib_bringup_ctx_t* c) {
    if (!c) return -1;
    if (c->marker_n < 1 || c->marker_n > SENTAI_CALIB_BRINGUP_MAX_MARKERS) return -1;
    if (!(c->marker_size_m > 0.0f && c->marker_size_m < 0.5f))             return -1;
    if (!(c->z_hold_m > 0.10f && c->z_hold_m < 3.0f))                       return -1;
    if (!(c->sweep_radius_m > 0.02f && c->sweep_radius_m < 0.5f))           return -1;
    if (!(c->settle_s    > 0.20f && c->settle_s    < 30.0f))                return -1;
    if (!(c->vmax_m_s    > 0.01f && c->vmax_m_s    < 0.5f))                 return -1;
    if (!(c->dur_relay_s > 1.0f  && c->dur_relay_s < 120.0f))               return -1;
    if (!(c->dur_hold_s  > 1.0f  && c->dur_hold_s  < 120.0f))               return -1;
    if (!(c->hold_rms_max_m > 0.001f && c->hold_rms_max_m < 0.5f))          return -1;
    for (int i = 0; i < 3 * c->marker_n; ++i) {
        if (!isfinite(c->marker_world_n3[i])) return -1;
    }
    return 0;
}

// ── SAMPLE phase ────────────────────────────────────────────────────────
// 4 corner poses around (0,0,z_hold).  At each pose: 1s velocity nudge
// toward the corner, settle_s settling, then capture k markers × 1
// snapshot (mean of 5 reads) as samples.  Limited to SAMPLES_MAX overall.
int phase_sample_() {
    const float r = s_ctx.sweep_radius_m;
    const float vmove = 0.10f;              // 1-sec travel ≈ 10 cm
    // 4 corners — short-axis sweep keeps drone in marker FOV.
    const float corners[4][2] = {
        { +r, +r }, { -r, +r }, { -r, -r }, { +r, -r },
    };
    int n_samples = 0;

    for (int p = 0; p < SENTAI_CALIB_BRINGUP_SWEEP_POSES; ++p) {
        if (aborted_())                      return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
        if (sentai_safety_is_aborted())     return SENTAI_CALIB_BRINGUP_REJ_SAFETY;

        // Nudge toward the corner (1 s @ ±vmove m/s) — open-loop.
        const float vx = (corners[p][0] >= 0.0f) ? +vmove : -vmove;
        const float vy = (corners[p][1] >= 0.0f) ? +vmove : -vmove;
        for (int k = 0; k < 10; ++k) {       // 10 × 100 ms = 1 s
            if (aborted_()) return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
            (void)sentai_crazy_hover(vx, vy, 0.0f, s_ctx.z_hold_m);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        // Settle: hover in place.
        const int settle_ticks = (int)(s_ctx.settle_s * 10.0f + 0.5f);
        for (int k = 0; k < settle_ticks; ++k) {
            if (aborted_()) return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
            (void)sentai_crazy_hover(0.0f, 0.0f, 0.0f, s_ctx.z_hold_m);
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // Capture: 5 reads of (markers + EKF) over 500 ms; average.
        SentaiMarkersPose mk_acc[SENTAI_CALIB_BRINGUP_MAX_MARKERS];
        int               mk_n_acc[SENTAI_CALIB_BRINGUP_MAX_MARKERS];
        memset(mk_acc, 0, sizeof(mk_acc));
        memset(mk_n_acc, 0, sizeof(mk_n_acc));
        float px = 0.0f, py = 0.0f, pz = 0.0f, pyaw = 0.0f;
        int   pose_n = 0;
        for (int t = 0; t < 5; ++t) {
            (void)sentai_crazy_hover(0.0f, 0.0f, 0.0f, s_ctx.z_hold_m);
            // EKF pose snapshot.
            float x, y, z, yaw;
            if (sentai_crazy_pose(&x, &y, &z, &yaw) == 0 &&
                isfinite(x) && isfinite(y) && isfinite(z) && isfinite(yaw)) {
                px += x; py += y; pz += z; pyaw += yaw;
                ++pose_n;
            }
            // Marker snapshot.
            int n = sentai_markers_get_count();
            if (n > SENTAI_CALIB_BRINGUP_MAX_MARKERS) n = SENTAI_CALIB_BRINGUP_MAX_MARKERS;
            for (int i = 0; i < n; ++i) {
                SentaiMarkersPose mk;
                if (sentai_markers_get_latest(i, &mk) != 0) continue;
                if (!mk.pose_valid) continue;
                if (mk.id < 0 || mk.id >= s_ctx.marker_n) continue;
                mk_acc[mk.id].tvec_cam[0] += mk.tvec_cam[0];
                mk_acc[mk.id].tvec_cam[1] += mk.tvec_cam[1];
                mk_acc[mk.id].tvec_cam[2] += mk.tvec_cam[2];
                ++mk_n_acc[mk.id];
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (pose_n == 0) continue;            // EKF dropped — skip pose
        const float inv_pn = 1.0f / (float)pose_n;
        const float drone_x   = px * inv_pn;
        const float drone_y   = py * inv_pn;
        const float drone_z   = pz * inv_pn;
        const float drone_yaw = pyaw * inv_pn;

        // Emit one sample per marker seen in this pose.
        for (int id = 0; id < s_ctx.marker_n; ++id) {
            if (mk_n_acc[id] == 0)            continue;
            if (n_samples >= SENTAI_CALIB_BRINGUP_SAMPLES_MAX) break;
            const float inv_mn = 1.0f / (float)mk_n_acc[id];
            sentai_calib_sample_t* s = &s_samples[n_samples];
            s->tvec_cam[0] = mk_acc[id].tvec_cam[0] * inv_mn;
            s->tvec_cam[1] = mk_acc[id].tvec_cam[1] * inv_mn;
            s->tvec_cam[2] = mk_acc[id].tvec_cam[2] * inv_mn;
            s->marker_W[0] = s_ctx.marker_world_n3[3*id + 0];
            s->marker_W[1] = s_ctx.marker_world_n3[3*id + 1];
            s->marker_W[2] = s_ctx.marker_world_n3[3*id + 2];
            s->drone_W[0]  = drone_x;
            s->drone_W[1]  = drone_y;
            s->drone_W[2]  = drone_z;
            s->yaw_rad     = drone_yaw;
            ++n_samples;
        }
    }
    s_result.n_samples_used = n_samples;
    if (n_samples < SENTAI_CALIB_SAMPLES_MIN) return SENTAI_CALIB_BRINGUP_REJ_FEW_SAMPLES;
    return 0;
}

// ── KABSCH phase ────────────────────────────────────────────────────────
int phase_kabsch_() {
    // Use the currently-cached R as the drift reference (so the quality
    // gate compares to the existing default / persisted value).
    const float* R_now = sentai_calib_get_R_cam_to_body();
    int rc = sentai_calib_run_kabsch(s_samples, s_result.n_samples_used,
                                       R_now,
                                       s_result.R_cam_to_body,
                                       &s_result.ext_quality);
    if (rc != 0) return SENTAI_CALIB_BRINGUP_REJ_KABSCH_QUAL;
    if (!s_result.ext_quality.accepted) return SENTAI_CALIB_BRINGUP_REJ_KABSCH_QUAL;

    // cam_offset_B = mean over samples of:
    //   marker_W − R · tvec_cam − drone_W
    // (drone level → world ≈ body frame; calibrating in level hover so
    // ignoring the per-sample yaw rotation introduces a per-sample noise
    // averaged out across the sweep.  Refinement is W21-T5 territory.)
    const float* R = s_result.R_cam_to_body;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    for (int i = 0; i < s_result.n_samples_used; ++i) {
        const sentai_calib_sample_t* p = &s_samples[i];
        const float rx = R[0]*p->tvec_cam[0] + R[1]*p->tvec_cam[1] + R[2]*p->tvec_cam[2];
        const float ry = R[3]*p->tvec_cam[0] + R[4]*p->tvec_cam[1] + R[5]*p->tvec_cam[2];
        const float rz = R[6]*p->tvec_cam[0] + R[7]*p->tvec_cam[1] + R[8]*p->tvec_cam[2];
        sx += p->marker_W[0] - rx - p->drone_W[0];
        sy += p->marker_W[1] - ry - p->drone_W[1];
        sz += p->marker_W[2] - rz - p->drone_W[2];
    }
    const float inv = 1.0f / (float)s_result.n_samples_used;
    s_result.cam_offset_B[0] = sx * inv;
    s_result.cam_offset_B[1] = sy * inv;
    s_result.cam_offset_B[2] = sz * inv;
    return 0;
}

// ── AUTOTUNE phase (X or Y) ────────────────────────────────────────────
int phase_autotune_(sentai_calib_axis_t axis) {
    int rc = sentai_calib_task_start(axis, s_ctx.dur_relay_s, s_ctx.vmax_m_s);
    if (rc != 0) return -1;                  // task spawn failed

    // Poll until inner worker self-completes; cooperative abort observed.
    const uint32_t t0 = now_ms_();
    const uint32_t timeout_ms = (uint32_t)(s_ctx.dur_relay_s * 1000.0f) + 5000;
    while (!sentai_calib_task_is_done()) {
        if (aborted_()) {
            (void)sentai_calib_task_stop();
            return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
        }
        if (sentai_safety_is_aborted()) {
            (void)sentai_calib_task_stop();
            return SENTAI_CALIB_BRINGUP_REJ_SAFETY;
        }
        if ((now_ms_() - t0) > timeout_ms) {
            (void)sentai_calib_task_stop();
            return -2;                       // hard timeout
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // Reap the worker handle so the next task_start can re-arm.
    (void)sentai_calib_task_stop();

    const float kp = sentai_calib_get_kp(axis);
    if (!(kp > 0.0f)) return -3;             // autotune did not converge

    if (axis == SENTAI_CALIB_AXIS_X) s_result.kp_x = kp;
    else                              s_result.kp_y = kp;
    return 0;
}

// ── HOLD phase ─────────────────────────────────────────────────────────
int phase_hold_() {
    // vmax_clip = 3 × autotune vmax — give the closed loop headroom
    // beyond what the relay scanned (~0.30 m/s default).
    const float vmax_clip = 3.0f * s_ctx.vmax_m_s;
    int rc = sentai_calib_hold_start(s_result.kp_x, s_result.kp_y,
                                       vmax_clip, s_ctx.dur_hold_s);
    if (rc != 0) return -1;
    const uint32_t t0 = now_ms_();
    const uint32_t timeout_ms = (uint32_t)(s_ctx.dur_hold_s * 1000.0f) + 5000;
    while (!sentai_calib_task_is_done()) {
        if (aborted_()) {
            (void)sentai_calib_task_stop();
            return SENTAI_CALIB_BRINGUP_REJ_ABORTED;
        }
        if (sentai_safety_is_aborted()) {
            (void)sentai_calib_task_stop();
            return SENTAI_CALIB_BRINGUP_REJ_SAFETY;
        }
        if ((now_ms_() - t0) > timeout_ms) {
            (void)sentai_calib_task_stop();
            return -2;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    (void)sentai_calib_task_stop();

    s_result.hold_max_drift_m = sentai_calib_get_hold_max_drift_m();
    s_result.hold_rms_drift_m = sentai_calib_get_hold_rms_drift_m();
    if (s_result.hold_rms_drift_m > s_ctx.hold_rms_max_m) {
        return SENTAI_CALIB_BRINGUP_REJ_HOLD_DRIFT;
    }
    return 0;
}

// ── SAVE phase ─────────────────────────────────────────────────────────
int phase_save_() {
    if (sentai_calib_commit_R(s_result.R_cam_to_body,
                                s_result.cam_offset_B) != 0) {
        return SENTAI_CALIB_BRINGUP_REJ_SAVE;
    }
    if (sentai_calib_commit_kp(SENTAI_CALIB_AXIS_X, s_result.kp_x) != 0 ||
        sentai_calib_commit_kp(SENTAI_CALIB_AXIS_Y, s_result.kp_y) != 0) {
        return SENTAI_CALIB_BRINGUP_REJ_SAVE;
    }
    if (sentai_calib_save() != 1) return SENTAI_CALIB_BRINGUP_REJ_SAVE;
    return 0;
}

// ── Worker entry ───────────────────────────────────────────────────────
void worker_loop_() {
    const uint32_t t_start_ms = now_ms_();
    sentai_fr_push_event("bringup", "task_start");
    fprintf(stderr, "[bringup] START z_hold=%.2f sweep=±%.2f markers=%d\n",
            (double)s_ctx.z_hold_m, (double)s_ctx.sweep_radius_m,
            (int)s_ctx.marker_n);

    // Forward geometry to the autotune ctx (grid_dx/grid_dy used for FOV
    // cap).  Use sweep_radius as both grid axes — conservative.
    (void)sentai_calib_set_context(s_ctx.z_hold_m,
                                     s_ctx.sweep_radius_m,
                                     s_ctx.sweep_radius_m,
                                     s_ctx.marker_size_m);

    int rc;

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_SAMPLE;
    rc = phase_sample_();
    if (rc != 0) { s_result.reject_code = rc; goto fail; }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_KABSCH;
    rc = phase_kabsch_();
    if (rc != 0) { s_result.reject_code = rc; goto fail; }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_AUTOTUNE_X;
    rc = phase_autotune_(SENTAI_CALIB_AXIS_X);
    if (rc != 0) {
        s_result.reject_code = (rc == SENTAI_CALIB_BRINGUP_REJ_SAFETY)
            ? SENTAI_CALIB_BRINGUP_REJ_SAFETY
            : (rc == SENTAI_CALIB_BRINGUP_REJ_ABORTED)
                ? SENTAI_CALIB_BRINGUP_REJ_ABORTED
                : SENTAI_CALIB_BRINGUP_REJ_AUTOTUNE_X;
        goto fail;
    }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_AUTOTUNE_Y;
    rc = phase_autotune_(SENTAI_CALIB_AXIS_Y);
    if (rc != 0) {
        s_result.reject_code = (rc == SENTAI_CALIB_BRINGUP_REJ_SAFETY)
            ? SENTAI_CALIB_BRINGUP_REJ_SAFETY
            : (rc == SENTAI_CALIB_BRINGUP_REJ_ABORTED)
                ? SENTAI_CALIB_BRINGUP_REJ_ABORTED
                : SENTAI_CALIB_BRINGUP_REJ_AUTOTUNE_Y;
        goto fail;
    }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_HOLD;
    rc = phase_hold_();
    if (rc != 0) {
        s_result.reject_code = (rc == SENTAI_CALIB_BRINGUP_REJ_SAFETY ||
                                rc == SENTAI_CALIB_BRINGUP_REJ_ABORTED ||
                                rc == SENTAI_CALIB_BRINGUP_REJ_HOLD_DRIFT)
            ? rc
            : SENTAI_CALIB_BRINGUP_REJ_HOLD_DRIFT;
        goto fail;
    }

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_SAVE;
    rc = phase_save_();
    if (rc != 0) { s_result.reject_code = rc; goto fail; }

    // Success.
    s_result.accepted = 1;
    s_result.last_phase = SENTAI_CALIB_BRINGUP_PHASE_DONE_OK;
    s_result.total_duration_ms = now_ms_() - t_start_ms;
    s_phase = SENTAI_CALIB_BRINGUP_PHASE_DONE_OK;
    sentai_fr_push_event("bringup", "done_ok");
    fprintf(stderr, "[bringup] DONE_OK kp_x=%.3f kp_y=%.3f rms=%.4fm dur=%ums\n",
            (double)s_result.kp_x, (double)s_result.kp_y,
            (double)s_result.hold_rms_drift_m,
            (unsigned)s_result.total_duration_ms);
    goto exit_;

fail:
    s_result.accepted = 0;
    s_result.last_phase = (int32_t)s_phase;
    s_result.total_duration_ms = now_ms_() - t_start_ms;
    s_phase = SENTAI_CALIB_BRINGUP_PHASE_DONE_FAIL;
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "phase=%d reject=%d",
                 (int)s_result.last_phase, (int)s_result.reject_code);
        sentai_fr_push_event("bringup_fail", buf);
    }
    fprintf(stderr, "[bringup] DONE_FAIL phase=%d reject=%d dur=%ums\n",
            (int)s_result.last_phase, (int)s_result.reject_code,
            (unsigned)s_result.total_duration_ms);

exit_:
    s_done = true;
    // Park the drone at the last commanded setpoint — keep hovering
    // until the caller lands.
    (void)sentai_crazy_hover(0.0f, 0.0f, 0.0f, s_ctx.z_hold_m);
}

void worker_entry_(void*) {
    worker_loop_();
    s_running = false;
    s_handle  = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace

// =======================================================================
// Public API
// =======================================================================

extern "C" int sentai_calib_bringup_start(const sentai_calib_bringup_ctx_t* ctx) {
    if (s_running) return -2;
    if (validate_ctx_(ctx) != 0) return -1;

    // Snapshot ctx + reset result.
    memcpy(&s_ctx, ctx, sizeof(s_ctx));
    memset(&s_result, 0, sizeof(s_result));
    memcpy(s_result.R_cam_to_body,
           sentai_calib_get_R_cam_to_body(), 9 * sizeof(float));
    memcpy(s_result.cam_offset_B,
           sentai_calib_get_cam_offset_B(), 3 * sizeof(float));
    s_result.kp_x = -1.0f;
    s_result.kp_y = -1.0f;

    s_phase = SENTAI_CALIB_BRINGUP_PHASE_IDLE;
    s_done  = false;

    if (!s_stop_evt) s_stop_evt = xEventGroupCreate();
    if (!s_stop_evt) return -3;
    xEventGroupClearBits(s_stop_evt, STOP_BIT);

    BaseType_t ok = xTaskCreate(
        worker_entry_, "sentai_bring",
        configMINIMAL_STACK_SIZE * 4, nullptr,
        tskIDLE_PRIORITY + 2, &s_handle);
    if (ok != pdPASS || !s_handle) {
        fprintf(stderr, "[bringup] xTaskCreate FAIL ok=%ld\n", (long)ok);
        return -3;
    }
    s_running = true;
    return 0;
}

extern "C" int sentai_calib_bringup_is_done(void) {
    return s_done ? 1 : 0;
}

extern "C" sentai_calib_bringup_phase_t sentai_calib_bringup_get_phase(void) {
    return s_phase;
}

extern "C" int sentai_calib_bringup_get_result(sentai_calib_bringup_result_t* out) {
    if (!out) return -1;
    memcpy(out, &s_result, sizeof(*out));
    return 0;
}

extern "C" int sentai_calib_bringup_abort(void) {
    if (s_stop_evt) xEventGroupSetBits(s_stop_evt, STOP_BIT);
    return 0;
}

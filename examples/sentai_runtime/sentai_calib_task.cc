// sentai_calib_task.cc — OP-S10-W14 worker that drives the relay
// autotuner.  See sentai_calib_task.h + sentai_calib_autotune.h for
// the contract; ideas/objects_plan/16_sentai_calib_autotune.md for
// the algorithm derivation.
//
// REUSE ONLY (no duplicate compute):
//   - sentai_aruco_get_latest() : cached PnP markers (camera frame)
//   - sentai_calib_get_R_cam_to_body() : SIM bring-up identity-like
//   - sentai_crazy_hover()      : body-frame velocity command
//   - sentai_crazy_get_altitude() : current altitude (for z hold)
//   - sentai_safety_is_aborted() : abort autotune on safety latch
//   - sentai_fr_push_scalar/event : journaling
//
// All math lives in sentai_calib_autotune.cc.  This file is plumbing.

#include "sentai_calib.h"
#include "sentai_calib_task.h"
#include "sentai_calib_autotune.h"
#include "sentai_aruco.h"
#include "sentai_safety.h"
#include "sentai_fr.h"
#include "sentai_crazy.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

extern "C" uint32_t sentai_now_ms(void) __attribute__((weak));

namespace {

// ── Persistent context (set by sentai_calib_set_context) ──────────────
struct Context {
    float z_hold_m;
    float grid_dx_m;
    float grid_dy_m;
    float marker_size_m;
    int   set;
};
static Context s_ctx = { 0.60f, 0.20f, 0.15f, 0.0625f, 0 };

// ── Flow gains (the W14 deliverable) ──────────────────────────────────
struct FlowGains {
    float    Kp[2];        // [SENTAI_CALIB_AXIS_X, SENTAI_CALIB_AXIS_Y]
    uint32_t td_ms;        // phase 2; 0 until then
};
static FlowGains s_gains = { { -1.0f, -1.0f }, 0 };

// ── Task state ────────────────────────────────────────────────────────
static TaskHandle_t       s_task_handle  = nullptr;
static EventGroupHandle_t s_stop_evt     = nullptr;
#define STOP_BIT  0x01
static volatile bool      s_started      = false;
static volatile bool      s_done         = false;

// Anchor — the centroid of marker tvec_cam at first valid tick.
static int   s_anchored    = 0;
static float s_anchor_cx   = 0.0f;
static float s_anchor_cy   = 0.0f;
static int   s_cycle_axis  = 0;     // captured from arm()

// Stats (post-run summary push)
static uint32_t s_n_ticks      = 0;
static uint32_t s_n_valid_pnp  = 0;
static uint32_t s_n_no_pnp     = 0;

inline uint32_t now_ms_() {
    if (sentai_now_ms) return sentai_now_ms();
    // No SIM-local fallback here; SIM should always have a now_ms
    // weak symbol via modsentai_sim_camera.c.
    return xTaskGetTickCount() * (1000U / configTICK_RATE_HZ);
}

// Approximate body-frame drift along the tuned axis from camera-frame
// marker centroid.  Downward-facing cam convention: when drone moves
// +X_body, markers in tvec_cam shift -X_cam.  We compose with the
// cached R_cam_to_body (default identity for SIM bring-up) so we get
// a body-frame quantity even when the SIM coord setup is non-trivial.
//
// NOTE: a wrong sign here makes the relay DIVERGE.  In SIM phase 1
// (downward cam, identity R) the formulas below are correct; the
// safety task will latch abort if it does diverge (markers leave FOV
// in < 1 s), which is the correct fail-safe.
inline float drift_along_axis_(const sentai_aruco_marker_t* mk, int n,
                                 int axis) {
    if (n <= 0) return 0.0f;
    float cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < n; ++i) {
        cx += mk[i].tvec_cam[0];
        cy += mk[i].tvec_cam[1];
    }
    cx /= (float)n;
    cy /= (float)n;
    static int diag_calls = 0;
    if (diag_calls++ < 10) {
        fprintf(stderr, "[autotune diag] call=%d n=%d anchored=%d "
                        "cx=%.4f cy=%.4f mk0_tvec=(%.4f,%.4f,%.4f)\n",
                diag_calls, n, s_anchored, (double)cx, (double)cy,
                (double)mk[0].tvec_cam[0], (double)mk[0].tvec_cam[1],
                (double)mk[0].tvec_cam[2]);
    }
    if (!s_anchored) {
        s_anchor_cx = cx;
        s_anchor_cy = cy;
        s_anchored  = 1;
        return 0.0f;
    }
    // Camera-frame displacement of centroid since anchor.
    float dcx = cx - s_anchor_cx;
    float dcy = cy - s_anchor_cy;
    // Downward-cam → body-frame mapping.  Apply R_cam_to_body
    // (row-major 3x3) projected onto the body axis.  R[0..2] = body X
    // row, R[3..5] = body Y row.
    const float* R = sentai_calib_get_R_cam_to_body();
    float body_x = R[0] * dcx + R[1] * dcy;        // ignoring tvec.z drift
    float body_y = R[3] * dcx + R[4] * dcy;
    // Sign of relative motion: drone +X_body → centroid -X_cam, so
    // drift_body = -(R · dt_cam).  ZN math is invariant to overall
    // sign as long as the closed loop is negative feedback; the
    // task's relay (drive AGAINST drift) handles the sign correctly.
    if (axis == SENTAI_CALIB_AXIS_X) return -body_x;
    else                              return -body_y;
}

void worker_loop_() {
    fprintf(stderr, "[sentai_calib_task] START axis=%d z_hold=%.2f\n",
            (int)s_cycle_axis, (double)s_ctx.z_hold_m);
    sentai_fr_push_event("autotune", "task_start");

    while ((xEventGroupGetBits(s_stop_evt) & STOP_BIT) == 0) {
        ++s_n_ticks;

        // Pull latest PnP (cached; no detect re-run).
        sentai_aruco_marker_t mk[16];
        int n = sentai_aruco_get_latest(mk, 16);
        int pnp_valid = (n >= 4) ? 1 : 0;
        if (pnp_valid) ++s_n_valid_pnp; else ++s_n_no_pnp;

        // Compute drift along tuned axis (body frame approx).
        float drift_m = pnp_valid
            ? drift_along_axis_(mk, n, s_cycle_axis)
            : 0.0f;
        uint32_t ts = now_ms_();

        // Drive the state machine.
        float v_cmd = sentai_calib_autotune_tick(drift_m, ts, pnp_valid);

        // Issue body-frame velocity command via cf2 hover.
        // hover(vx, vy, yaw_rate, z_distance).  yaw_rate=0; z held.
        if (s_cycle_axis == SENTAI_CALIB_AXIS_X) {
            (void)sentai_crazy_hover(v_cmd, 0.0f, 0.0f, s_ctx.z_hold_m);
        } else {
            (void)sentai_crazy_hover(0.0f, v_cmd, 0.0f, s_ctx.z_hold_m);
        }

        // Push samples to FR (silent no-op if channel not open).
        sentai_fr_push_scalar("at_drift_m", drift_m, ts);
        sentai_fr_push_scalar("at_v_cmd",   v_cmd,   ts);

        // Safety check — if safety latched abort, end autotune.
        if (sentai_safety_is_aborted()) {
            sentai_fr_push_event("autotune", "safety_abort");
            sentai_calib_autotune_abort();
        }

        // Terminal state → publish results then exit.
        sentai_calib_autotune_state_t st = sentai_calib_autotune_get_state();
        if (st == SENTAI_CALIB_AT_DONE_OK ||
            st == SENTAI_CALIB_AT_DONE_FAIL ||
            st == SENTAI_CALIB_AT_ABORTED) {
            // Park at zero velocity.
            (void)sentai_crazy_hover(0.0f, 0.0f, 0.0f, s_ctx.z_hold_m);
            if (st == SENTAI_CALIB_AT_DONE_OK) {
                float kp = sentai_calib_autotune_get_last_kp();
                s_gains.Kp[s_cycle_axis] = kp;
                char buf[96];
                snprintf(buf, sizeof(buf),
                         "axis=%d kp=%.4f Tu_s=%.3f ay_m=%.4f cyc=%d",
                         (int)s_cycle_axis,
                         (double)kp,
                         (double)sentai_calib_autotune_get_last_Tu_s(),
                         (double)sentai_calib_autotune_get_last_ay_m(),
                         sentai_calib_autotune_get_last_cycles());
                sentai_fr_push_event("autotune_done_ok", buf);
            } else if (st == SENTAI_CALIB_AT_DONE_FAIL) {
                sentai_fr_push_event("autotune_done_fail", "timeout_or_unstable");
            } else {
                sentai_fr_push_event("autotune_done_abort", "safety");
            }
            s_done = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(SENTAI_CALIB_TASK_PERIOD_MS));
    }
    fprintf(stderr,
            "[sentai_calib_task] STOP ticks=%u pnp_ok=%u pnp_miss=%u state=%d\n",
            (unsigned)s_n_ticks, (unsigned)s_n_valid_pnp,
            (unsigned)s_n_no_pnp,
            (int)sentai_calib_autotune_get_state());
    sentai_fr_push_event("autotune", "task_stop");
}

void worker_entry_(void*) {
    worker_loop_();
    vTaskDelete(nullptr);
    s_task_handle = nullptr;
}

}  // namespace

// =======================================================================
// Public API — declared in sentai_calib.h
// =======================================================================

extern "C" int sentai_calib_set_context(float z_hold_m,
                                          float marker_grid_dx_m,
                                          float marker_grid_dy_m,
                                          float marker_size_m) {
    if (!(z_hold_m > 0.0f && z_hold_m < 5.0f))                return -1;
    if (!(marker_grid_dx_m > 0.0f && marker_grid_dx_m < 1.0f)) return -1;
    if (!(marker_grid_dy_m > 0.0f && marker_grid_dy_m < 1.0f)) return -1;
    if (!(marker_size_m   > 0.0f && marker_size_m   < 0.5f))   return -1;
    s_ctx.z_hold_m      = z_hold_m;
    s_ctx.grid_dx_m     = marker_grid_dx_m;
    s_ctx.grid_dy_m     = marker_grid_dy_m;
    s_ctx.marker_size_m = marker_size_m;
    s_ctx.set           = 1;
    return 0;
}

extern "C" float sentai_calib_get_kp(sentai_calib_axis_t axis) {
    if (axis != SENTAI_CALIB_AXIS_X && axis != SENTAI_CALIB_AXIS_Y) return -1.0f;
    return s_gains.Kp[(int)axis];
}

extern "C" uint32_t sentai_calib_get_td_ms(void) {
    return s_gains.td_ms;
}

extern "C" int sentai_calib_task_start(sentai_calib_axis_t axis,
                                         float dur_s,
                                         float vmax_m_s) {
    if (s_started) return 0;
    if (!s_ctx.set) {
        // Use defaults silently; operator can override via set_context.
    }
    // FOV-bound v_max (operator-suggested: keep markers in FOV).
    float half_period_max_s = 2.0f;   // conservative
    float displace_max = s_ctx.grid_dx_m;
    if (s_ctx.grid_dy_m < displace_max) displace_max = s_ctx.grid_dy_m;
    displace_max *= SENTAI_CALIB_TASK_FOV_FRACTION;
    float vmax_fov = displace_max / half_period_max_s;
    if (vmax_m_s > vmax_fov) vmax_m_s = vmax_fov;

    // Reset state.
    s_anchored      = 0;
    s_anchor_cx     = 0.0f;
    s_anchor_cy     = 0.0f;
    s_cycle_axis    = (int)axis;
    s_n_ticks       = 0;
    s_n_valid_pnp   = 0;
    s_n_no_pnp      = 0;
    s_done          = false;
    sentai_calib_autotune_init();
    int arm_rc = sentai_calib_autotune_arm(axis, dur_s, vmax_m_s);
    if (arm_rc != 0) return -2;

    if (!s_stop_evt) s_stop_evt = xEventGroupCreate();
    if (!s_stop_evt) return -3;
    xEventGroupClearBits(s_stop_evt, STOP_BIT);

    BaseType_t ok = xTaskCreate(
        worker_entry_, "sentai_calib",
        configMINIMAL_STACK_SIZE * 4, nullptr,
        tskIDLE_PRIORITY + 2, &s_task_handle);
    if (ok != pdPASS || !s_task_handle) {
        fprintf(stderr, "[sentai_calib_task] xTaskCreate FAIL ok=%ld\n",
                (long)ok);
        return -4;
    }
    s_started = true;
    return 0;
}

extern "C" int sentai_calib_task_stop(void) {
    if (!s_started) return 0;
    if (s_stop_evt) xEventGroupSetBits(s_stop_evt, STOP_BIT);
    for (int i = 0; i < SENTAI_CALIB_TASK_STOP_BUDGET_TICKS; ++i) {
        if (!s_task_handle) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    s_task_handle = nullptr;
    s_started     = false;
    return 0;
}

extern "C" int sentai_calib_task_is_done(void) {
    return s_done ? 1 : 0;
}

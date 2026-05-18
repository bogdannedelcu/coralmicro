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

// ── KNOWN_POSITIONS_M — marker world positions (must match the SDF
//    aruco_id0..3 + aruco_detector.py KNOWN_POSITIONS_M).
//    OP-S10-W14 iter #2 layout (2026-05-18): doubled to 12×12 cm
//    markers at ±0.12, ±0.20.  Z = top-of-box (0.005 + 0.005 = 0.010).
static const float KNOWN_POS_M[4][3] = {
    /* id 0 */ { +0.12f, +0.20f, 0.010f },
    /* id 1 */ { -0.12f, +0.20f, 0.010f },
    /* id 2 */ { -0.12f, -0.20f, 0.010f },
    /* id 3 */ { +0.12f, -0.20f, 0.010f },
};

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
    // sign-probe inside the state machine handles the unknown
    // sign of the cmd→drift mapping via `sign_flip`.
    if (axis == SENTAI_CALIB_AXIS_X) return -body_x;
    else                              return -body_y;
}

void worker_loop_() {
    fprintf(stderr, "[sentai_calib_task] START axis=%d z_hold=%.2f\n",
            (int)s_cycle_axis, (double)s_ctx.z_hold_m);
    sentai_fr_push_event("autotune", "task_start");

    // Relay step size (HL go_to(relative=1) magnitude per sign-flip).
    // Tuned 2026-05-18 iter #4: hover() doesn't override HL Commander
    // after takeoff (empirically confirmed — drift stayed +ve even with
    // v_cmd=-v_max for 1.5 s).  Use go_to(relative=1) instead — stays
    // within HL Commander mode and is proven to work (s170, s145).
    // Step magnitude = vmax × half_period_target (rough).  vmax=0.10,
    // T_u_target ≈ 1.5 s → step = 0.10 × 0.75 = 0.075 m.  Bounded by
    // FOV (capped further inside).
    // Iter #11+: relay drives via hover() velocity, no per-step
    // position trajectory.  step_size kept for future go_to fallback.
    (void)0;       /* placeholder */
    int last_relay_sgn = 0;          // tracks last v_cmd sign

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

        // Operator-observed 2026-05-18 ("drona driftuieste in sus
        // si ne afecteaza testul autotune; ce vezi tu cu semne
        // schimbate e de fapt un efect al driftului pe verticala
        // cand obiectele de la sol par ca se deplaseaza inspre
        // centru").  Z drift contaminates lateral PnP geometry, so
        // keep z_hold via a minimal passive correction (NOT T10 full
        // active loop — just a per-relay-tick z bump in the go_to
        // call when sign changes).  Compute z error from PnP only
        // when valid; hold last value otherwise to avoid noise.
        static float last_z_correction = 0.0f;
        if (pnp_valid) {
            // tvec_cam[2] = depth from cam to marker ≈ drone altitude
            // for the downward-facing setup at z=z_hold above flat
            // markers.  Mean over visible markers smooths per-marker
            // PnP noise.
            float z_pnp = 0.0f;
            for (int i = 0; i < n; ++i) z_pnp += mk[i].tvec_cam[2];
            z_pnp /= (float)n;
            // z_error > 0 when drone is TOO LOW (drone < z_hold).
            // cf2 body z = +Z up (NWU), so dz_command should follow
            // sign of z_error.  Gain 1.0 since we send only on sign
            // change (~ 2 Hz), well-damped.
            // Gain 0.1 (not 1.0): with relative=1 go_to applied on
            // every sign-flip (~2 Hz), commanded z deltas accumulate.
            // 1.0 gain × ±10 cm clip × 30 ticks during autotune = ±3 m
            // total commanded — way too aggressive (iter #6 trial:
            // drone climbed uncontrolled to z_hold + ε).  0.1 gain
            // with ±2 cm clip gives ±60 cm cumulative max, well-damped.
            last_z_correction = 0.1f * (s_ctx.z_hold_m - z_pnp);
            if (last_z_correction > +0.02f) last_z_correction = +0.02f;
            if (last_z_correction < -0.02f) last_z_correction = -0.02f;
        }
        sentai_fr_push_scalar("at_z_corr", last_z_correction, ts);

        // ── VPE forwarder (OP-S10-W14 iter #14): close the loop
        //    on cf2's EKF altitude.  Iter #13 with 5 Hz reduced
        //    z overshoot 35 % but didn't eliminate it — cf2 fusion
        //    weight on sparse VPE was too low.  Iter #14: send
        //    EVERY tick (30 Hz) so the EKF gets continuous
        //    correction, on par with cf2's baro update rate.
        //    Anti-cheat compliant: PnP-derived, not GT-injected.
        static uint32_t s_vpe_last_ms = 0;
        if (pnp_valid && (ts - s_vpe_last_ms) >= 33) {  // 30 Hz
            // drone_world[i] = marker_world[i] - R_cam_to_body * tvec_cam[i]
            // Average over visible markers.  R_cam_to_body comes
            // from sentai_calib (SIM default identity-like).
            const float* R = sentai_calib_get_R_cam_to_body();
            float dx_sum = 0.0f, dy_sum = 0.0f, dz_sum = 0.0f;
            int   dn_used = 0;
            for (int i = 0; i < n; ++i) {
                uint8_t mid = mk[i].marker_id;
                if (mid >= 4) continue;                 // only id 0..3 known
                const float* mw = KNOWN_POS_M[mid];
                // R * tvec_cam (row-major)
                float tx = mk[i].tvec_cam[0];
                float ty = mk[i].tvec_cam[1];
                float tz = mk[i].tvec_cam[2];
                float rx = R[0]*tx + R[1]*ty + R[2]*tz;
                float ry = R[3]*tx + R[4]*ty + R[5]*tz;
                float rz = R[6]*tx + R[7]*ty + R[8]*tz;
                dx_sum += mw[0] - rx;
                dy_sum += mw[1] - ry;
                dz_sum += mw[2] - rz;
                ++dn_used;
            }
            if (dn_used > 0) {
                float dx = dx_sum / (float)dn_used;
                float dy = dy_sum / (float)dn_used;
                float dz = dz_sum / (float)dn_used;
                (void)sentai_crazy_send_extpos(dx, dy, dz);
                sentai_fr_push_scalar("at_vpe_x", dx, ts);
                sentai_fr_push_scalar("at_vpe_y", dy, ts);
                sentai_fr_push_scalar("at_vpe_z", dz, ts);
                s_vpe_last_ms = ts;
            }
        }

        // Drive the state machine.
        float v_cmd = sentai_calib_autotune_tick(drift_m, ts, pnp_valid);

        // Iter #11: switch to TRUE velocity relay via hover() +
        // absolute z_hold.  Prereq: caller has issued
        // sentai.crazy.hl_stop() after takeoff so Generic Setpoints
        // win over HL Commander.  Two killer benefits vs go_to:
        //   1. hover.z_distance is ABSOLUTE altitude (m above
        //      takeoff) — no accumulation.  cf2 holds the altitude
        //      via its altitude PID + baro; z_hold stays at our
        //      setpoint regardless of how many ticks pass.
        //   2. hover commanded velocity is INSTANTANEOUS (not a
        //      trajectory) — no overshoot, no ringing.  The relay
        //      flips happen exactly at drift sign changes.
        //
        // Note: we send hover EVERY tick (30 Hz), not just on sign
        // change, because the cf2 Generic Commander watchdog cuts
        // motors after ~1 s without setpoints.
        float vx = (s_cycle_axis == SENTAI_CALIB_AXIS_X) ? v_cmd : 0.0f;
        float vy = (s_cycle_axis == SENTAI_CALIB_AXIS_Y) ? v_cmd : 0.0f;
        (void)sentai_crazy_hover(vx, vy, /*yaw_rate=*/0.0f,
                                    /*z_absolute=*/s_ctx.z_hold_m);
        // Track sign-flips for diagnostics; last_relay_sgn no longer
        // gates the call (every tick now), but useful for FR events.
        int sgn_cmd = (v_cmd > 0.001f) ? +1 : (v_cmd < -0.001f ? -1 : 0);
        if (sgn_cmd != 0 && sgn_cmd != last_relay_sgn) {
            char buf[48];
            snprintf(buf, sizeof(buf), "sgn=%d v_cmd=%.3f", sgn_cmd, (double)v_cmd);
            sentai_fr_push_event("autotune_relay_flip", buf);
            last_relay_sgn = sgn_cmd;
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
            // Park: HL Commander already holds at last commanded
            // trajectory endpoint when we stop sending go_to.  No
            // explicit park call needed (used to call hover(0,0,...)
            // but hover doesn't override HL — see comment above).
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

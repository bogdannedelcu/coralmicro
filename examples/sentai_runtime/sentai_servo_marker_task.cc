// sentai_servo_marker_task.cc -- B4 image-frame marker-control worker.

#include "sentai_servo_marker_task.h"

#include "flow_shared.h"
#include "sentai_calib.h"
#include "sentai_calib_orientation_task.h"
#include "sentai_crazy.h"
#include "sentai_crazy_log.h"
#include "sentai_fr.h"
#include "sentai_markers.h"
#include "sentai_safety.h"
#include "sentai_servo.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

extern "C" int sentai_cam_init_full(int pixfmt, int fps, int mirror, int flip)
    __attribute__((weak));
extern "C" int sentai_flow_start(int cam_id) __attribute__((weak));
extern "C" void sentai_flow_deadband_state(uint32_t* period_ms_x10,
                                           uint32_t* deadband_mgp,
                                           uint32_t* velocity_mgp_per_s)
    __attribute__((weak));
#if defined(SENTAI_PLATFORM_SIM)
struct SimFlowSnapshot {
    volatile uint32_t seq;
    volatile int32_t dx_q1000;
    volatile int32_t dy_q1000;
    volatile uint32_t conf;
    volatile uint64_t latency_us;
    volatile int32_t dz_q1000;
    volatile uint32_t dz_conf;
};
extern "C" const SimFlowSnapshot* sim_camera_flow_snapshot(void)
    __attribute__((weak));
#endif

namespace {

#define STOP_BIT 0x01

static const int kImgW = 320;
static const int kImgH = 240;
static const float kFx = 288.3f;
static const float kCx = 160.0f;
static const float kCy = 120.0f;
static const float kMarkerDiameterM = 0.0544f;
static const float kFullVisMarginPx = 2.0f;
static const int kLockAvgWindow = 10;
static const int kLockFullMarkers = 7;
static const int kMinFullMarkers = 4;
static const int kZeroUnlockMs = 1500;
static const int kRampMaxMs = 12000;
static const int kBaseThrustU16 = 30000;
static const int kMaxThrustU16 = 34500;
static const float kTakeoffZTargetScale = 1.5f;
static const float kAcquireCentroidAvgMinMarkers = 2.0f;
static const float kAcquireCentroidFilterAlpha = 0.30f;
static const float kAcquireCentroidGain = 0.025f;
static const float kAcquireCentroidMaxDeg = 0.20f;
static const float kAcquireCentroidDeadbandPx = 3.0f;
static const int kHoverVisualThrustU16 = 32700;
static const int kHoldMinThrustU16 = 30000;
static const int kHoldMaxThrustU16 = 34500;
static const float kKpThrustPerM = 6500.0f;
static const float kKdThrustPerMS = 5200.0f;
static const float kVZLpfAlpha = 0.25f;

static const int kCenterHoldMs = 2000;
static const float kCenterHoldGain = 0.08f;
static const float kCenterHoldMaxDeg = 0.45f;
static const float kIbvsDampingPxPerDeg = 1.5f;
static const float kIbvsSustainedResponseSign = -1.0f;
static const float kIbvsZRefM = 0.64f;
static const float kIbvsZGainMin = 0.65f;
static const float kIbvsZGainMax = 1.35f;
static const int kExtposWarmupMinMs = 2500;
static const int kExtposWarmupMaxMs = 12000;
static const float kExtposPnpZRatioMin = 0.50f;
static const float kExtposPnpZRatioMax = 1.50f;
static const float kExtposConvergedErrFraction = 0.20f;
static const float kExtposDivergedErrFraction = 1.00f;
static const int kHandoffHoverMs = 4000;
static const float kGenericImageEdgeMarginPx = 10.0f;
static const int kGenericAxisMaxMs = 8000;
static const int kGenericCenterMs = 6000;
static const int kGenericSettleMs = 600;
static const int kGenericLandDurMs = 3000;
static const int kGenericLandTimeoutMs = 3800;
static const float kGenericLandFinalZM = 0.05f;
static const float kGenericImageEnvelopeAmpXCapPx = 76.0f;
static const float kGenericImageEnvelopeAmpYCapPx = 44.0f;
static const float kGenericKpVelPerPx = 0.0010f;
static const float kGenericKdVelPerPx = 0.00035f;
static const float kGenericVmaxMS = 0.08f;
static const float kGenericTargetTolFrac = 0.05f;
static const float kExtposBootstrapStddevM = 0.04f;
static const float kExtposFlowAssistedStddevM = 0.12f;
static const float kFlowScaleFw = 6.179050948650294f;
static const float kFlowScaleLeft = 6.392121671017544f;
static const float kFlowDtMinS = 0.001f;
static const float kFlowDtMaxS = 0.2f;
static const uint8_t kCrptPortSetpointSim = 0x09;
static const uint8_t kSensorFlowSim = 6;
static const uint8_t kCrptPortSetpointGeneric = 0x07;

enum Mode {
    MODE_NONE = 0,
    MODE_ACQUIRE = 1,
    MODE_CENTER_HOLD = 2,
    MODE_EXTPOS_WARMUP = 3,
    MODE_HANDOFF_HOVER = 4,
    MODE_AXIS_MOTION = 5,
    MODE_LAND = 6,
};

struct State {
    sentai_servo_marker_status_t st;
    TaskHandle_t task_handle;
    EventGroupHandle_t stop_evt;
    int mode;
    int current_thrust_u16;
    float z_target_m;
    float roll_vec[2];
    float pitch_vec[2];
    volatile int prep_hold_stop;
    volatile int prep_hold_ticks;
    TaskHandle_t prep_hold_handle;
    uint32_t flow_last_seq;
    float image_envelope[4];  // min_x_cx, max_x_cx, min_y_cy, max_y_cy
};

static State s;

static float clip_(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int send_rpyt_(float roll_deg, float pitch_deg,
                      float yaw_rate_deg_s, int thrust_u16) {
    uint8_t p[14];
    memcpy(p + 0, &roll_deg, 4);
    memcpy(p + 4, &pitch_deg, 4);
    memcpy(p + 8, &yaw_rate_deg_s, 4);
    const uint16_t th = (uint16_t)clip_((float)thrust_u16, 0.0f, 65535.0f);
    memcpy(p + 12, &th, 2);
    return sentai_crazy_send_crtp(3, 0, p, sizeof(p));
}

static int stopped_() {
    return (s.stop_evt &&
            (xEventGroupGetBits(s.stop_evt) & STOP_BIT)) ? 1 : 0;
}

static void finish_(int ok, int reason_code) {
    s.st.ok = ok ? 1 : 0;
    s.st.reason_code = reason_code;
    s.st.done = 1;
    s.st.phase = ok ? SENTAI_SERVO_MARKER_PHASE_DONE
                    : SENTAI_SERVO_MARKER_PHASE_FAULTED;
    s.task_handle = nullptr;
}

static int sample_observation_(SentaiMarkersObservation* o) {
    return sentai_calib_sample_observation(
        kImgW, kImgH, kFullVisMarginPx, o);
}

static void marker_window_reset_(int slot, int size) {
    sentai_markers_window_reset(slot, size);
}

static float marker_window_push_avg_(int slot, int n_full,
                                     SentaiMarkersWindowStats* out) {
    memset(out, 0, sizeof(*out));
    sentai_markers_window_push(slot, n_full, out);
    return out->avg_full;
}

static float visual_z_(const SentaiMarkersObservation& o) {
    // s203/B4 used apparent marker diameter as the visual-Z control signal.
    // Keep this independent of calibrated PnP/extrinsics; PnP Z is only for
    // ExtPos validation.
    if (o.radius_mean_px <= 0.0f) return 0.0f;
    return kFx * kMarkerDiameterM / (2.0f * o.radius_mean_px);
}

static float flow_std_from_conf_(int conf) {
    if (conf >= 200) return 3.0f;
    if (conf >= 128) return 4.0f;
    if (conf >= 64) return 6.0f;
    return 10.0f;
}

static int send_flow_from_snapshot_() {
#if defined(SENTAI_PLATFORM_SIM)
    if (!sim_camera_flow_snapshot) return -20;
    const SimFlowSnapshot* sh = sim_camera_flow_snapshot();
    if (!sh) return -20;
    uint32_t seq0 = sh->seq;
    int32_t dx = sh->dx_q1000;
    int32_t dy = sh->dy_q1000;
    uint32_t conf = sh->conf;
    uint32_t seq1 = sh->seq;
    if (seq1 != seq0) {
        dx = sh->dx_q1000;
        dy = sh->dy_q1000;
        conf = sh->conf;
        seq0 = seq1;
    }
    if (seq0 == 0) return -21;
    if (seq0 == s.flow_last_seq) return 0;
    s.flow_last_seq = seq0;

    float t_body[3];
    float R[9];
    int extrinsics_set = 0;
    sentai_markers_get_cam_extrinsics_matrix(t_body, R, &extrinsics_set);
    (void)t_body;
    float body_fw = -(float)dx;
    float body_left = (float)dy;
    if (extrinsics_set) {
        body_fw = R[0] * (float)dx + R[1] * (float)dy;
        body_left = R[3] * (float)dx + R[4] * (float)dy;
    }

    float dt = (float)SENTAI_SERVO_MARKER_TASK_PERIOD_MS / 1000.0f;
    if (sentai_flow_deadband_state) {
        uint32_t period_ms_x10 = 0;
        sentai_flow_deadband_state(&period_ms_x10, nullptr, nullptr);
        if (period_ms_x10 > 0) dt = (float)period_ms_x10 / 10000.0f;
    }
    dt = clip_(dt, kFlowDtMinS, kFlowDtMaxS);
    const float dpx = (body_fw / 1000.0f) * kFlowScaleFw;
    const float dpy = (body_left / 1000.0f) * kFlowScaleLeft;
    const float std = flow_std_from_conf_((int)conf);
    uint8_t p[17];
    p[0] = kSensorFlowSim;
    memcpy(p + 1, &dpx, 4);
    memcpy(p + 5, &dpy, 4);
    memcpy(p + 9, &dt, 4);
    memcpy(p + 13, &std, 4);
    const int rc = sentai_crazy_send_crtp(kCrptPortSetpointSim, 0, p, sizeof(p));
    return rc == 0 ? 1 : -22;
#else
    volatile flow_shared_t* sh = &FLOW_SHARED();
    if (sh->magic != FLOW_SHARED_MAGIC || sh->last_frame_seq == 0) return 0;
    const uint32_t seq = sh->last_frame_seq;
    if (seq == s.flow_last_seq) return 0;
    s.flow_last_seq = seq;

    int32_t dx = sh->last_dx;
    int32_t dy = sh->last_dy;
    float t_body[3];
    float R[9];
    int extrinsics_set = 0;
    sentai_markers_get_cam_extrinsics_matrix(t_body, R, &extrinsics_set);
    (void)t_body;
    float body_fw = 0.0f;
    float body_left = 0.0f;
    if (extrinsics_set) {
        body_fw = R[0] * (float)dx + R[1] * (float)dy;
        body_left = R[3] * (float)dx + R[4] * (float)dy;
    } else if (sh->frame_cam_id == 0) {
        body_fw = -(float)dx;
        body_left = (float)dy;
    } else {
        body_fw = (float)dx;
        body_left = -(float)dy;
    }

    float dt = (float)SENTAI_SERVO_MARKER_TASK_PERIOD_MS / 1000.0f;
    if (sentai_flow_deadband_state) {
        uint32_t period_ms_x10 = 0;
        sentai_flow_deadband_state(&period_ms_x10, nullptr, nullptr);
        if (period_ms_x10 > 0) dt = (float)period_ms_x10 / 10000.0f;
    }
    dt = clip_(dt, kFlowDtMinS, kFlowDtMaxS);
    const float dpx = (body_fw / 1000.0f) * kFlowScaleFw;
    const float dpy = (body_left / 1000.0f) * kFlowScaleLeft;
    const float std = flow_std_from_conf_(sh->last_confidence);
    uint8_t p[17];
    p[0] = kSensorFlowSim;
    memcpy(p + 1, &dpx, 4);
    memcpy(p + 5, &dpy, 4);
    memcpy(p + 9, &dt, 4);
    memcpy(p + 13, &std, 4);
    const int rc = sentai_crazy_send_crtp(kCrptPortSetpointSim, 0, p, sizeof(p));
    return rc == 0 ? 1 : -22;
#endif
}

static int release_rpyt_without_disarm_() {
#if defined(SENTAI_PLATFORM_SIM)
    const uint8_t notify_stop[5] = {0, 0, 0, 0, 0};
    return sentai_crazy_send_crtp(kCrptPortSetpointGeneric, 1,
                                  notify_stop, sizeof(notify_stop));
#else
    return sentai_crazy_attitude_release_no_disarm();
#endif
}

static int z_thrust_(float z, float* z_prev, float* vz_filt) {
    if (z > 0.0f && *z_prev > 0.0f) {
        const float vz = (z - *z_prev) *
            (1000.0f / (float)SENTAI_SERVO_MARKER_TASK_PERIOD_MS);
        *vz_filt = (1.0f - kVZLpfAlpha) * (*vz_filt) + kVZLpfAlpha * vz;
    } else if (z > 0.0f) {
        *vz_filt = 0.0f;
    }
    if (z > 0.0f) *z_prev = z;
    float cmd = (float)kHoverVisualThrustU16;
    if (z > 0.0f) {
        cmd += kKpThrustPerM * (s.z_target_m - z);
        cmd -= kKdThrustPerMS * (*vz_filt);
    }
    return (int)clip_(cmd, (float)kHoldMinThrustU16,
                      (float)kHoldMaxThrustU16);
}

static void centroid_command_to_(const SentaiMarkersObservation& o,
                                 float z_visual,
                                 float target_x,
                                 float target_y,
                                 float gain,
                                 float max_deg,
                                 float deadband_px,
                                 float* roll,
                                 float* pitch,
                                 float* err) {
    float ex = 0.0f, ey = 0.0f;
    float target[2], rr[2], pp[2], z_gain = 1.0f, det = 0.0f;
    sentai_servo_ibvs_centroid_command(
        o.centroid_x, o.centroid_y, z_visual, target_x, target_y,
        s.roll_vec, s.pitch_vec, gain, max_deg,
        deadband_px, kIbvsDampingPxPerDeg, kIbvsSustainedResponseSign,
        kIbvsZRefM, kIbvsZGainMin, kIbvsZGainMax,
        roll, pitch, &ex, &ey, err, target, rr, pp, &z_gain, &det);
}

static void center_command_(const SentaiMarkersObservation& o, float z_visual,
                            float* roll, float* pitch, float* err) {
    centroid_command_to_(o, z_visual, kCx, kCy, kCenterHoldGain,
                         kCenterHoldMaxDeg, 0.0f, roll, pitch, err);
}

static void prep_hold_worker_(void*) {
    float z_prev = 0.0f;
    float vz_filt = 0.0f;
    int thrust = s.current_thrust_u16 > 0 ?
        s.current_thrust_u16 : kHoverVisualThrustU16;
    s.prep_hold_ticks = 0;
    while (!s.prep_hold_stop) {
        SentaiMarkersObservation o;
        sample_observation_(&o);
        const float z_visual = visual_z_(o);
        if (o.n_full >= kMinFullMarkers && z_visual > 0.0f) {
            float roll = 0.0f, pitch = 0.0f, err = 0.0f;
            center_command_(o, z_visual, &roll, &pitch, &err);
            thrust = z_thrust_(z_visual, &z_prev, &vz_filt);
            send_rpyt_(roll, pitch, 0.0f, thrust);
        } else {
            send_rpyt_(0.0f, 0.0f, 0.0f, kHoverVisualThrustU16);
        }
        s.current_thrust_u16 = thrust;
        s.prep_hold_ticks++;
        vTaskDelay(pdMS_TO_TICKS(SENTAI_SERVO_MARKER_TASK_PERIOD_MS));
    }
    s.prep_hold_handle = nullptr;
    vTaskDelete(nullptr);
}

static int prep_hold_start_() {
    if (s.prep_hold_handle != nullptr) return 0;
    s.prep_hold_stop = 0;
    s.prep_hold_ticks = 0;
    const BaseType_t ok = xTaskCreate(
        prep_hold_worker_, "servo_prep_hold", configMINIMAL_STACK_SIZE * 3,
        nullptr, tskIDLE_PRIORITY + 2, &s.prep_hold_handle);
    return ok == pdPASS ? 0 : -1;
}

static void prep_hold_stop_() {
    s.prep_hold_stop = 1;
    for (int i = 0; i < 20 && s.prep_hold_handle != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void run_acquire_() {
    sentai_fr_push_event("marker_acquire", "start=1");
    (void)sentai_crazy_arm();
    const int zero_ticks = kZeroUnlockMs / SENTAI_SERVO_MARKER_TASK_PERIOD_MS;
    for (int k = 0; k < zero_ticks && !stopped_(); ++k) {
        send_rpyt_(0.0f, 0.0f, 0.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(SENTAI_SERVO_MARKER_TASK_PERIOD_MS));
    }

    marker_window_reset_(1, kLockAvgWindow);
    SentaiMarkersWindowStats win;
    memset(&win, 0, sizeof(win));
    int lock_avg_ticks = 0;
    int n_full_max = 0;
    float avg_best = 0.0f;
    float radius_max = 0.0f;
    float z_peak = 0.0f;
    float z_first_full = 0.0f;
    float centroid_filt_x = 0.0f;
    float centroid_filt_y = 0.0f;
    float centroid_err_last = 0.0f;
    float centroid_err_max = 0.0f;
    int centroid_ref_valid = 0;
    int centroid_seen_ticks = 0;
    int centroid_cmd_ticks = 0;
    int first_seen_tick = -1;
    int first_full_tick = -1;
    int thrust = kBaseThrustU16;
    int ticks_done = 0;
    const int ticks = kRampMaxMs / SENTAI_SERVO_MARKER_TASK_PERIOD_MS;

    for (int k = 0; k < ticks && !stopped_(); ++k) {
        const float alpha = (float)k / (float)((ticks > 1) ? (ticks - 1) : 1);
        thrust = kBaseThrustU16 +
            (int)(alpha * (float)(kMaxThrustU16 - kBaseThrustU16));

        SentaiMarkersObservation o;
        sample_observation_(&o);
        const float z_visual = visual_z_(o);
        const float avg = marker_window_push_avg_(1, o.n_full, &win);
        float roll_cmd = 0.0f;
        float pitch_cmd = 0.0f;
        if (o.n_full > 0 && first_seen_tick < 0) first_seen_tick = k;
        if (avg > kAcquireCentroidAvgMinMarkers && z_visual > 0.0f) {
            centroid_seen_ticks++;
            if (!centroid_ref_valid) {
                centroid_filt_x = o.centroid_x;
                centroid_filt_y = o.centroid_y;
                centroid_ref_valid = 1;
            } else {
                centroid_filt_x =
                    (1.0f - kAcquireCentroidFilterAlpha) * centroid_filt_x +
                    kAcquireCentroidFilterAlpha * o.centroid_x;
                centroid_filt_y =
                    (1.0f - kAcquireCentroidFilterAlpha) * centroid_filt_y +
                    kAcquireCentroidFilterAlpha * o.centroid_y;
            }
            SentaiMarkersObservation cmd_obs = o;
            cmd_obs.centroid_x = centroid_filt_x;
            cmd_obs.centroid_y = centroid_filt_y;
            centroid_command_to_(cmd_obs, z_visual, kCx, kCy,
                                 kAcquireCentroidGain,
                                 kAcquireCentroidMaxDeg,
                                 kAcquireCentroidDeadbandPx,
                                 &roll_cmd, &pitch_cmd, &centroid_err_last);
            if (centroid_err_last > centroid_err_max) {
                centroid_err_max = centroid_err_last;
            }
            centroid_cmd_ticks++;
        }
        send_rpyt_(roll_cmd, pitch_cmd, 0.0f, thrust);

        if (avg > avg_best) avg_best = avg;
        if (o.n_full > n_full_max) n_full_max = o.n_full;
        if (o.radius_mean_px > radius_max) radius_max = o.radius_mean_px;
        if (z_visual > z_peak) z_peak = z_visual;
        if (z_first_full <= 0.0f &&
                o.n_full >= kLockFullMarkers && z_visual > 0.0f) {
            z_first_full = z_visual;
            first_full_tick = k;
        }

        if (avg > (float)(kLockFullMarkers - 1)) lock_avg_ticks++;
        else lock_avg_ticks = 0;
        ticks_done = k + 1;

        if ((k % 10) == 0) {
            char ev[260];
            snprintf(ev, sizeof(ev),
                     "k=%d thrust=%d n_avg=%.2f n_full=%d radius=%.2f z=%.3f hold=%d err=%.2f filt_x=%.2f filt_y=%.2f roll=%.3f pitch=%.3f",
                     k, thrust, (double)avg, o.n_full,
                     (double)o.radius_mean_px, (double)z_visual,
                     centroid_ref_valid, (double)centroid_err_last,
                     (double)centroid_filt_x, (double)centroid_filt_y,
                     (double)roll_cmd, (double)pitch_cmd);
            sentai_fr_push_event("marker_acquire_tick", ev);
        }

        if (lock_avg_ticks >= kLockAvgWindow) {
            s.st.acquire_ok = 1;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(SENTAI_SERVO_MARKER_TASK_PERIOD_MS));
    }

    s.current_thrust_u16 = thrust;
    const float z_lock = z_peak;
    s.z_target_m = z_lock * kTakeoffZTargetScale;
    if (s.z_target_m <= 0.0f) s.z_target_m = kIbvsZRefM;
    s.st.acquire_ticks_done = ticks_done;
    s.st.acquire_last_thrust_u16 = thrust;
    s.st.acquire_n_full_max = n_full_max;
    s.st.acquire_avg_full_markers = avg_best;
    s.st.acquire_radius_max_px = radius_max;
    s.st.acquire_z_cam_max_m = z_peak;
    s.st.acquire_z_target_m = s.z_target_m;

    char ev[320];
    snprintf(ev, sizeof(ev),
             "ok=%d ticks=%d thrust_last=%d n_full_max=%d n_avg_best=%.2f radius_max=%.2f z_lock=%.3f z_peak=%.3f z_target=%.3f centroid_avg_min=%.2f centroid_seen=%d centroid_cmd=%d centroid_err_last=%.2f centroid_err_max=%.2f first_seen_tick=%d first_full_tick=%d",
             s.st.acquire_ok, ticks_done, thrust, n_full_max,
             (double)avg_best, (double)radius_max, (double)z_lock,
             (double)z_peak, (double)s.z_target_m,
             (double)kAcquireCentroidAvgMinMarkers,
             centroid_seen_ticks, centroid_cmd_ticks,
             (double)centroid_err_last, (double)centroid_err_max,
             first_seen_tick, first_full_tick);
    sentai_fr_push_event("marker_acquire", ev);
    finish_(s.st.acquire_ok, s.st.acquire_ok ? 0 : 1);
}

static void run_center_hold_() {
    sentai_fr_push_event("marker_center_hold", "start=1");
    sentai_markers_window_reset(1, kLockAvgWindow);
    SentaiMarkersWindowStats win;
    memset(&win, 0, sizeof(win));

    float z_prev = 0.0f;
    float vz_filt = 0.0f;
    int n_full_min = 99;
    int thrust = s.current_thrust_u16 > 0 ?
        s.current_thrust_u16 : kHoverVisualThrustU16;
    float err_initial = -1.0f;
    float err_last = 999.0f;
    float err_max = 0.0f;

    const int ticks = kCenterHoldMs / SENTAI_SERVO_MARKER_TASK_PERIOD_MS;
    int ticks_done = 0;
    for (int k = 0; k < ticks && !stopped_(); ++k) {
        SentaiMarkersObservation o;
        sample_observation_(&o);
        sentai_markers_window_push(1, o.n_full, &win);
        if (o.n_full < n_full_min) n_full_min = o.n_full;
        const float z_visual = visual_z_(o);
        if (o.n_full < kMinFullMarkers || z_visual <= 0.0f) {
            send_rpyt_(0.0f, 0.0f, 0.0f, kHoverVisualThrustU16);
            finish_(0, 2);
            return;
        }

        float roll = 0.0f, pitch = 0.0f, err = 0.0f;
        center_command_(o, z_visual, &roll, &pitch, &err);
        if (err_initial < 0.0f) err_initial = err;
        err_last = err;
        if (err > err_max) err_max = err;
        thrust = z_thrust_(z_visual, &z_prev, &vz_filt);
        send_rpyt_(roll, pitch, 0.0f, thrust);
        ticks_done = k + 1;
        if (k % 10 == 0) {
            char ev[180];
            snprintf(ev, sizeof(ev),
                     "k=%d roll=%.3f pitch=%.3f thrust=%d err=%.2f avg=%.2f z=%.3f",
                     k, (double)roll, (double)pitch, thrust, (double)err,
                     (double)win.avg_full, (double)z_visual);
            sentai_fr_push_event("marker_center_hold_tick", ev);
        }
        vTaskDelay(pdMS_TO_TICKS(SENTAI_SERVO_MARKER_TASK_PERIOD_MS));
    }

    s.current_thrust_u16 = thrust;
    s.st.center_ticks_done = ticks_done;
    s.st.center_n_full_min = n_full_min == 99 ? 0 : n_full_min;
    s.st.center_avg_full_markers = win.avg_full;
    s.st.center_err_initial_px = err_initial < 0.0f ? 999.0f : err_initial;
    s.st.center_err_last_px = err_last;
    s.st.center_err_max_px = err_max;
    s.st.center_thrust_last_u16 = thrust;
    s.st.center_ok = (win.ready && win.avg_full >= (float)kMinFullMarkers &&
                      err_last < 80.0f) ? 1 : 0;
    char ev[180];
    snprintf(ev, sizeof(ev),
             "ok=%d ticks=%d n_full_min=%d avg=%.2f err_initial=%.2f err_last=%.2f err_max=%.2f thrust=%d",
             s.st.center_ok, ticks_done, s.st.center_n_full_min,
             (double)win.avg_full, (double)s.st.center_err_initial_px,
             (double)err_last, (double)err_max, thrust);
    sentai_fr_push_event("marker_center_hold", ev);
    finish_(s.st.center_ok, s.st.center_ok ? 0 : 3);
}

static int pose_to_extpos_(const SentaiMarkersDronePose& pose,
                           float extpos[3]) {
    if (!(pose.z > 0.0f)) return 0;
    extpos[0] = s.st.extpos_signs[0] * pose.x + s.st.extpos_origin_corr_m[0];
    extpos[1] = s.st.extpos_signs[1] * pose.y + s.st.extpos_origin_corr_m[1];
    extpos[2] = s.st.extpos_signs[2] * pose.z + s.st.extpos_origin_corr_m[2];
    return 1;
}

static float pose_error_(const float a[3], const float b[3]) {
    const float dx = b[0] - a[0];
    const float dy = b[1] - a[1];
    const float dz = b[2] - a[2];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static void log_estimator_pose_(const float est[3]) {
    sentai_fr_push_scalar("marker_est_x", est[0], 0);
    sentai_fr_push_scalar("marker_est_y", est[1], 0);
    sentai_fr_push_scalar("marker_est_z", est[2], 0);
}

static int pose_plausible_(const SentaiMarkersDronePose& pose,
                           const SentaiMarkersObservation& o,
                           float z_visual, int* reason) {
    if (pose.n_used <= 0) {
        if (reason) *reason = 1;
        return 0;
    }
    if (o.n_full < kMinFullMarkers) {
        if (reason) *reason = 2;
        return 0;
    }
    if (z_visual <= 0.0f || pose.z <= 0.0f) {
        if (reason) *reason = 3;
        return 0;
    }
    const float ratio = pose.z / z_visual;
    if (ratio < kExtposPnpZRatioMin || ratio > kExtposPnpZRatioMax) {
        if (reason) *reason = 4;
        return 0;
    }
    if (reason) *reason = 0;
    return 1;
}

static int send_extpos_from_current_markers_(const SentaiMarkersObservation& o,
                                             float z_visual,
                                             float last_pose[3],
                                             float last_extpos[3],
                                             float last_est[3],
                                             int* have_pose,
                                             int* have_extpos,
                                             int* have_est,
                                             int* reject_reason) {
    float est_x = 0.0f, est_y = 0.0f, est_z = 0.0f, cf_yaw = 0.0f;
    if (sentai_crazy_pose(&est_x, &est_y, &est_z, &cf_yaw) == 0) {
        last_est[0] = est_x;
        last_est[1] = est_y;
        last_est[2] = est_z;
        log_estimator_pose_(last_est);
        if (have_est) *have_est = 1;
    }

    SentaiMarkersDronePose pose;
    memset(&pose, 0, sizeof(pose));
    if (!sentai_markers_get_drone_pose(cf_yaw, &pose)) {
        if (reject_reason) *reject_reason = 1;
        return 0;
    }
    last_pose[0] = pose.x;
    last_pose[1] = pose.y;
    last_pose[2] = pose.z;
    if (have_pose) *have_pose = 1;
    int reason = 0;
    if (!pose_plausible_(pose, o, z_visual, &reason)) {
        if (reject_reason) *reject_reason = reason;
        return 0;
    }

    float extpos[3] = {0.0f, 0.0f, 0.0f};
    if (!pose_to_extpos_(pose, extpos)) {
        if (reject_reason) *reject_reason = 3;
        return 0;
    }
    if (sentai_crazy_send_extpos(extpos[0], extpos[1], extpos[2]) != 0) {
        if (reject_reason) *reject_reason = 5;
        return 0;
    }
    memcpy(last_extpos, extpos, sizeof(float) * 3);
    if (have_extpos) *have_extpos = 1;
    if (reject_reason) *reject_reason = 0;
    return 1;
}

static void run_extpos_warmup_() {
    sentai_fr_push_event("marker_extpos_warmup", "start=1");
    const int prep_hold_rc = prep_hold_start_();
    s.st.extpos_kalman_reset_rc = sentai_crazy_kalman_reset_before_extpos();
    s.st.extpos_stddev_bootstrap_rc =
        sentai_crazy_set_extpos_stddev(kExtposBootstrapStddevM);
    prep_hold_stop_();
    {
        char ev[160];
        snprintf(ev, sizeof(ev),
                 "hold_rc=%d hold_ticks=%d kalman_reset_rc=%d bootstrap_stddev_rc=%d bootstrap_stddev=%.3f",
                 prep_hold_rc, (int)s.prep_hold_ticks,
                 s.st.extpos_kalman_reset_rc,
                 s.st.extpos_stddev_bootstrap_rc,
                 (double)kExtposBootstrapStddevM);
        sentai_fr_push_event("marker_estimator_prep", ev);
    }

    sentai_markers_window_reset(1, kLockAvgWindow);
    SentaiMarkersWindowStats win;
    memset(&win, 0, sizeof(win));

    float z_prev = 0.0f;
    float vz_filt = 0.0f;
    int thrust = s.current_thrust_u16 > 0 ?
        s.current_thrust_u16 : kHoverVisualThrustU16;
    int n_full_min = 99;
    int send_ok = 0;
    int pose_rejects = 0;
    int flow_send_ok = 0;
    int flow_read_errors = 0;
    int last_reject_reason = 0;
    float last_pose[3] = {0.0f, 0.0f, 0.0f};
    float last_extpos[3] = {0.0f, 0.0f, 0.0f};
    float last_est[3] = {0.0f, 0.0f, 0.0f};
    int have_pose = 0;
    int have_extpos = 0;
    int have_est = 0;
    float err_first = -1.0f;
    float err_last = 999.0f;
    float err_min = 999.0f;
    float err_max = 0.0f;
    float recent[10];
    int recent_n = 0;
    int recent_i = 0;
    int converged = 0;
    int diverged = 0;
    int marker_lost = 0;
    s.flow_last_seq = 0;

    const int min_ticks =
        kExtposWarmupMinMs / SENTAI_SERVO_MARKER_TASK_PERIOD_MS;
    const int max_ticks =
        kExtposWarmupMaxMs / SENTAI_SERVO_MARKER_TASK_PERIOD_MS;
    int ticks_done = 0;
    for (int k = 0; k < max_ticks && !stopped_(); ++k) {
        SentaiMarkersObservation o;
        sample_observation_(&o);
        sentai_markers_window_push(1, o.n_full, &win);
        if (o.n_full < n_full_min) n_full_min = o.n_full;
        const float z_visual = visual_z_(o);

        if (send_extpos_from_current_markers_(
                o, z_visual, last_pose, last_extpos, last_est,
                &have_pose, &have_extpos, &have_est,
                &last_reject_reason)) {
            send_ok++;
        } else {
            pose_rejects++;
        }

        if (have_extpos && have_est) {
            const float e = pose_error_(last_extpos, last_est);
            if (err_first < 0.0f) err_first = e;
            err_last = e;
            if (e < err_min) err_min = e;
            if (e > err_max) err_max = e;
            recent[recent_i] = e;
            recent_i = (recent_i + 1) % 10;
            if (recent_n < 10) recent_n++;
            if (k >= min_ticks && recent_n >= kLockAvgWindow &&
                    s.z_target_m > 0.0f) {
                float sum = 0.0f;
                for (int i = 0; i < recent_n; ++i) sum += recent[i];
                const float mean = sum / (float)recent_n;
                if (mean <= kExtposConvergedErrFraction * s.z_target_m) {
                    converged = 1;
                } else if (mean >= kExtposDivergedErrFraction * s.z_target_m) {
                    diverged = 1;
                }
            }
        }

        float roll = 0.0f, pitch = 0.0f, err_px = 0.0f;
        if (o.n_full >= kMinFullMarkers && z_visual > 0.0f) {
            center_command_(o, z_visual, &roll, &pitch, &err_px);
            thrust = z_thrust_(z_visual, &z_prev, &vz_filt);
            send_rpyt_(roll, pitch, 0.0f, thrust);
        } else {
            send_rpyt_(0.0f, 0.0f, 0.0f, kHoverVisualThrustU16);
        }

        const int flow_rc = send_flow_from_snapshot_();
        if (flow_rc > 0) {
            flow_send_ok++;
        } else if (flow_rc < 0) {
            flow_read_errors++;
        }

        ticks_done = k + 1;
        if (k % 10 == 0) {
            char ev[220];
            snprintf(ev, sizeof(ev),
                     "k=%d n=%d avg=%.2f send_ok=%d pose_rej=%d est_ok=%d err=%.3f conv=%d div=%d flow=%d",
                     k, o.n_full, (double)win.avg_full, send_ok,
                     pose_rejects, have_est, (double)err_last,
                     converged, diverged, flow_send_ok);
            sentai_fr_push_event("marker_extpos_warmup_tick", ev);
        }
        if (win.ready && win.avg_full < (float)kMinFullMarkers) {
            marker_lost = 1;
            break;
        }
        if (converged || diverged) break;
        vTaskDelay(pdMS_TO_TICKS(SENTAI_SERVO_MARKER_TASK_PERIOD_MS));
    }

    float recent_mean = 0.0f;
    if (recent_n > 0) {
        for (int i = 0; i < recent_n; ++i) recent_mean += recent[i];
        recent_mean /= (float)recent_n;
    }
    s.current_thrust_u16 = thrust;
    s.st.extpos_ticks_done = ticks_done;
    s.st.extpos_send_ok = send_ok;
    s.st.extpos_pose_rejects = pose_rejects;
    s.st.extpos_flow_send_ok = flow_send_ok;
    s.st.extpos_flow_read_errors = flow_read_errors;
    s.st.extpos_n_full_min = n_full_min == 99 ? 0 : n_full_min;
    s.st.extpos_n_full_recent_avg = win.avg_full;
    s.st.extpos_err_first_m = err_first < 0.0f ? 0.0f : err_first;
    s.st.extpos_err_last_m = err_last;
    s.st.extpos_err_recent_mean_m = recent_mean;
    s.st.extpos_err_min_m = err_min == 999.0f ? 0.0f : err_min;
    s.st.extpos_err_max_m = err_max;
    s.st.extpos_reject_reason_code = last_reject_reason;
    s.st.extpos_converged = converged ? 1 : 0;
    s.st.extpos_diverged = diverged ? 1 : 0;
    s.st.extpos_marker_visibility_lost = marker_lost ? 1 : 0;
    if (have_pose) memcpy(s.st.extpos_last_pose_m, last_pose, sizeof(last_pose));
    if (have_extpos) memcpy(s.st.extpos_last_sent_m, last_extpos, sizeof(last_extpos));
    if (have_est) memcpy(s.st.extpos_last_est_m, last_est, sizeof(last_est));
    if (converged && !diverged && !marker_lost) {
        s.st.extpos_stddev_flow_rc =
            sentai_crazy_set_extpos_stddev(kExtposFlowAssistedStddevM);
    } else {
        s.st.extpos_stddev_flow_rc = -99;
    }
    s.st.extpos_ok = (have_pose && send_ok > 5 && converged && !diverged &&
                      !marker_lost && win.ready &&
                      win.avg_full > (float)(kMinFullMarkers - 1)) ? 1 : 0;
    char ev[360];
    snprintf(ev, sizeof(ev),
             "ok=%d ticks=%d send_ok=%d pose_rej=%d n_full_min=%d avg=%.2f err_first=%.3f err_last=%.3f err_recent=%.3f err_min=%.3f err_max=%.3f converged=%d diverged=%d marker_lost=%d flow=%d flow_err=%d reject_reason=%d kreset=%d std0=%d std1=%d",
             s.st.extpos_ok, ticks_done, send_ok, pose_rejects,
             s.st.extpos_n_full_min, (double)win.avg_full,
             (double)s.st.extpos_err_first_m, (double)err_last,
             (double)recent_mean, (double)s.st.extpos_err_min_m,
             (double)s.st.extpos_err_max_m, converged, diverged,
             marker_lost, flow_send_ok, flow_read_errors, last_reject_reason,
             s.st.extpos_kalman_reset_rc,
             s.st.extpos_stddev_bootstrap_rc, s.st.extpos_stddev_flow_rc);
    sentai_fr_push_event("marker_extpos_warmup", ev);
    finish_(s.st.extpos_ok, s.st.extpos_ok ? 0 : 4);
}

static void run_handoff_hover_() {
    sentai_fr_push_event("marker_handoff_hover", "start=1");
    sentai_markers_window_reset(1, kLockAvgWindow);
    SentaiMarkersWindowStats win;
    memset(&win, 0, sizeof(win));

    float hover_z = s.z_target_m;
    if (s.st.extpos_last_est_m[2] > 0.2f) {
        hover_z = s.st.extpos_last_est_m[2];
    } else if (s.st.extpos_last_pose_m[2] > 0.2f) {
        hover_z = s.st.extpos_last_pose_m[2];
    }
    s.st.handoff_hover_z_m = hover_z;
    const int release_rc = release_rpyt_without_disarm_();
    s.st.handoff_release_rc = release_rc;

    int n_full_min = 99;
    int send_ok = 0;
    int pose_rejects = 0;
    int flow_send_ok = 0;
    int flow_read_errors = 0;
    int hover_rc_last = -99;
    float last_pose[3] = {0.0f, 0.0f, 0.0f};
    float last_extpos[3] = {0.0f, 0.0f, 0.0f};
    float last_est[3] = {0.0f, 0.0f, 0.0f};
    int have_pose = 0;
    int have_extpos = 0;
    int have_est = 0;
    int reject_reason = 0;
    const int ticks = kHandoffHoverMs / SENTAI_SERVO_MARKER_TASK_PERIOD_MS;
    int ticks_done = 0;

    vTaskDelay(pdMS_TO_TICKS(15));
    for (int k = 0; k < ticks && !stopped_(); ++k) {
        SentaiMarkersObservation o;
        sample_observation_(&o);
        sentai_markers_window_push(1, o.n_full, &win);
        if (o.n_full < n_full_min) n_full_min = o.n_full;
        const float z_visual = visual_z_(o);

        if (send_extpos_from_current_markers_(
                o, z_visual, last_pose, last_extpos, last_est,
                &have_pose, &have_extpos, &have_est, &reject_reason)) {
            send_ok++;
        } else {
            pose_rejects++;
        }
        const int flow_rc = send_flow_from_snapshot_();
        if (flow_rc > 0) flow_send_ok++;
        else if (flow_rc < 0) flow_read_errors++;
        hover_rc_last = sentai_crazy_hover(0.0f, 0.0f, 0.0f, hover_z);
        ticks_done = k + 1;

        if (k % 10 == 0) {
            char ev[200];
            snprintf(ev, sizeof(ev),
                     "k=%d z=%.3f hover_rc=%d n=%d avg=%.2f send_ok=%d pose_rej=%d flow=%d flow_err=%d",
                     k, (double)hover_z, hover_rc_last, o.n_full,
                     (double)win.avg_full, send_ok, pose_rejects,
                     flow_send_ok, flow_read_errors);
            sentai_fr_push_event("marker_handoff_hover_tick", ev);
        }
        if (win.ready && win.avg_full < (float)kMinFullMarkers) break;
        vTaskDelay(pdMS_TO_TICKS(SENTAI_SERVO_MARKER_TASK_PERIOD_MS));
    }

    s.st.handoff_ticks_done = ticks_done;
    s.st.handoff_hover_rc_last = hover_rc_last;
    s.st.handoff_extpos_send_ok = send_ok;
    s.st.handoff_pose_rejects = pose_rejects;
    s.st.handoff_flow_send_ok = flow_send_ok;
    s.st.handoff_flow_read_errors = flow_read_errors;
    s.st.handoff_n_full_min = n_full_min == 99 ? 0 : n_full_min;
    s.st.handoff_n_full_recent_avg = win.avg_full;
    if (have_est) memcpy(s.st.handoff_last_est_m, last_est, sizeof(last_est));
    s.st.handoff_ok = (release_rc == 0 && hover_rc_last == 0 &&
                       win.ready && win.avg_full >= (float)kMinFullMarkers)
                          ? 1 : 0;
    char ev[180];
    snprintf(ev, sizeof(ev),
             "ok=%d release_rc=%d hover_rc=%d ticks=%d n_full_min=%d avg=%.2f hover_z=%.3f send_ok=%d pose_rej=%d flow=%d flow_err=%d",
             s.st.handoff_ok, release_rc, hover_rc_last, ticks_done,
             s.st.handoff_n_full_min, (double)win.avg_full,
             (double)hover_z, send_ok, pose_rejects, flow_send_ok,
             flow_read_errors);
    sentai_fr_push_event("marker_handoff_hover", ev);
    finish_(s.st.handoff_ok, s.st.handoff_ok ? 0 : 5);
}

enum ImageTarget {
    IMG_TARGET_CENTER = 0,
    IMG_TARGET_MIN_X = 1,
    IMG_TARGET_MAX_X = 2,
    IMG_TARGET_MIN_Y = 3,
    IMG_TARGET_MAX_Y = 4,
};

static void envelope_from_observation_(const SentaiMarkersObservation& o) {
    const float left_span = o.centroid_x - o.bbox_min_x;
    const float right_span = o.bbox_max_x - o.centroid_x;
    const float top_span = o.centroid_y - o.bbox_min_y;
    const float bottom_span = o.bbox_max_y - o.centroid_y;
    float amp_x = fminf(kCx - kGenericImageEdgeMarginPx - left_span,
                        (float)kImgW - kGenericImageEdgeMarginPx - kCx - right_span);
    float amp_y = fminf(kCy - kGenericImageEdgeMarginPx - top_span,
                        (float)kImgH - kGenericImageEdgeMarginPx - kCy - bottom_span);
    if (amp_x < 0.0f) amp_x = 0.0f;
    if (amp_y < 0.0f) amp_y = 0.0f;
    if (amp_x > kGenericImageEnvelopeAmpXCapPx) {
        amp_x = kGenericImageEnvelopeAmpXCapPx;
    }
    if (amp_y > kGenericImageEnvelopeAmpYCapPx) {
        amp_y = kGenericImageEnvelopeAmpYCapPx;
    }
    s.image_envelope[0] = kCx - amp_x;
    s.image_envelope[1] = kCx + amp_x;
    s.image_envelope[2] = kCy - amp_y;
    s.image_envelope[3] = kCy + amp_y;
    memcpy(s.st.axis_envelope_px, s.image_envelope, sizeof(s.image_envelope));
}

static int image_error_(const SentaiMarkersObservation& o, int target,
                        float* ex, float* ey) {
    if (!o.valid || o.n_full < kMinFullMarkers) return 0;
    float tx = kCx;
    float ty = kCy;
    if (target == IMG_TARGET_MIN_X) tx = s.image_envelope[0];
    else if (target == IMG_TARGET_MAX_X) tx = s.image_envelope[1];
    else if (target == IMG_TARGET_MIN_Y) ty = s.image_envelope[2];
    else if (target == IMG_TARGET_MAX_Y) ty = s.image_envelope[3];
    *ex = o.centroid_x - tx;
    *ey = o.centroid_y - ty;
    return 1;
}

static void guarded_axis_error_(int target, float ex, float ey,
                                float* cmd_ex, float* cmd_ey) {
    const float tol_x = (float)kImgW * kGenericTargetTolFrac;
    const float tol_y = (float)kImgH * kGenericTargetTolFrac;
    *cmd_ex = ex;
    *cmd_ey = ey;
    if ((target == IMG_TARGET_MIN_X || target == IMG_TARGET_MAX_X) &&
            fabsf(ey) > tol_y) {
        *cmd_ex = 0.0f;
    } else if ((target == IMG_TARGET_MIN_Y || target == IMG_TARGET_MAX_Y) &&
            fabsf(ex) > tol_x) {
        *cmd_ey = 0.0f;
    }
}

static int target_reached_(float ex, float ey) {
    const float tol_x = (float)kImgW * kGenericTargetTolFrac;
    const float tol_y = (float)kImgH * kGenericTargetTolFrac;
    return fabsf(ex) <= tol_x && fabsf(ey) <= tol_y;
}

static void hover_cmd_from_error_(float ex, float ey, float prev_ex,
                                  float prev_ey, int have_prev,
                                  float* vx, float* vy) {
    const float dex = have_prev ? ex - prev_ex : 0.0f;
    const float dey = have_prev ? ey - prev_ey : 0.0f;
    *vx = kGenericKpVelPerPx * ey + kGenericKdVelPerPx * dey;
    *vy = kGenericKpVelPerPx * ex + kGenericKdVelPerPx * dex;
    *vx = clip_(*vx, -kGenericVmaxMS, kGenericVmaxMS);
    *vy = clip_(*vy, -kGenericVmaxMS, kGenericVmaxMS);
}

static void run_axis_motion_() {
    sentai_fr_push_event("marker_axis_motion", "start=1");
    sentai_markers_window_reset(1, kLockAvgWindow);
    SentaiMarkersWindowStats win;
    memset(&win, 0, sizeof(win));

    const int targets[] = {
        IMG_TARGET_CENTER, IMG_TARGET_MIN_X, IMG_TARGET_MAX_X,
        IMG_TARGET_CENTER, IMG_TARGET_MIN_Y, IMG_TARGET_MAX_Y,
        IMG_TARGET_CENTER,
    };
    const char* labels[] = {
        "pre_axis_center", "image_min_x", "image_max_x",
        "center_after_x", "image_min_y", "image_max_y",
        "center_after_y",
    };
    const int count = (int)(sizeof(targets) / sizeof(targets[0]));
    float hover_z = s.st.handoff_hover_z_m > 0.2f ?
        s.st.handoff_hover_z_m : s.z_target_m;
    int n_full_min = 99;
    int total_ticks = 0;
    int send_ok = 0;
    int pose_rejects = 0;
    int flow_send_ok = 0;
    int flow_read_errors = 0;
    int segments_done = 0;
    int abort = 0;
    float last_err = 999.0f;

    for (int si = 0; si < count && !abort && !stopped_(); ++si) {
        const int target = targets[si];
        const int max_ms = (target == IMG_TARGET_CENTER) ?
            kGenericCenterMs : kGenericAxisMaxMs;
        const int max_ticks =
            (max_ms + kGenericSettleMs) / SENTAI_SERVO_MARKER_TASK_PERIOD_MS;
        int reached_k = -1;
        int hover_rc_last = -99;
        float prev_ex = 0.0f;
        float prev_ey = 0.0f;
        int have_prev = 0;
        int seg_reached = 0;
        SentaiMarkersObservation seg_last_obs;
        memset(&seg_last_obs, 0, sizeof(seg_last_obs));

        for (int k = 0; k < max_ticks && !stopped_(); ++k) {
            SentaiMarkersObservation o;
            sample_observation_(&o);
            seg_last_obs = o;
            sentai_markers_window_push(1, o.n_full, &win);
            if (o.n_full < n_full_min) n_full_min = o.n_full;
            const float z_visual = visual_z_(o);
            float last_pose[3] = {0.0f, 0.0f, 0.0f};
            float last_extpos[3] = {0.0f, 0.0f, 0.0f};
            float last_est[3] = {0.0f, 0.0f, 0.0f};
            int have_pose = 0, have_extpos = 0, have_est = 0, reject_reason = 0;
            if (send_extpos_from_current_markers_(
                    o, z_visual, last_pose, last_extpos, last_est,
                    &have_pose, &have_extpos, &have_est, &reject_reason)) {
                send_ok++;
            } else {
                pose_rejects++;
            }
            const int flow_rc = send_flow_from_snapshot_();
            if (flow_rc > 0) flow_send_ok++;
            else if (flow_rc < 0) flow_read_errors++;

            float ex = 0.0f, ey = 0.0f;
            float vx = 0.0f, vy = 0.0f;
            if (image_error_(o, target, &ex, &ey)) {
                float cmd_ex = 0.0f, cmd_ey = 0.0f;
                guarded_axis_error_(target, ex, ey, &cmd_ex, &cmd_ey);
                hover_cmd_from_error_(cmd_ex, cmd_ey, prev_ex, prev_ey,
                                      have_prev, &vx, &vy);
                prev_ex = cmd_ex;
                prev_ey = cmd_ey;
                have_prev = 1;
                last_err = sqrtf(ex * ex + ey * ey);
                if (target_reached_(ex, ey)) {
                    seg_reached = 1;
                    if (reached_k < 0) reached_k = k;
                    vx = 0.0f;
                    vy = 0.0f;
                }
            }
            hover_rc_last = sentai_crazy_hover(vx, vy, 0.0f, hover_z);
            total_ticks++;
            if (k % 10 == 0) {
                char ev[300];
                snprintf(ev, sizeof(ev),
                         "seg=%s k=%d target=%d vx=%.3f vy=%.3f err=%.2f cx=%.1f cy=%.1f bx0=%.1f by0=%.1f bx1=%.1f by1=%.1f n=%d avg=%.2f reached=%d rc=%d flow=%d flow_err=%d",
                         labels[si], k, target, (double)vx, (double)vy,
                         (double)last_err, (double)o.centroid_x,
                         (double)o.centroid_y, (double)o.bbox_min_x,
                         (double)o.bbox_min_y, (double)o.bbox_max_x,
                         (double)o.bbox_max_y, o.n_full,
                         (double)win.avg_full, seg_reached, hover_rc_last,
                         flow_send_ok, flow_read_errors);
                sentai_fr_push_event("marker_axis_motion_tick", ev);
            }
            if (win.ready && win.avg_full < (float)kMinFullMarkers) {
                abort = 1;
                break;
            }
            if (reached_k >= 0 &&
                    (k - reached_k) >=
                    (kGenericSettleMs / SENTAI_SERVO_MARKER_TASK_PERIOD_MS)) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(SENTAI_SERVO_MARKER_TASK_PERIOD_MS));
        }
        if (!seg_reached || hover_rc_last != 0) abort = 1;
        else {
            segments_done++;
            if (si == 0 && seg_last_obs.valid &&
                    seg_last_obs.n_full >= kMinFullMarkers) {
                envelope_from_observation_(seg_last_obs);
                char env_ev[160];
                snprintf(env_ev, sizeof(env_ev),
                         "ok=1 min_x=%.2f max_x=%.2f min_y=%.2f max_y=%.2f cap_x=%.2f cap_y=%.2f",
                         (double)s.image_envelope[0],
                         (double)s.image_envelope[1],
                         (double)s.image_envelope[2],
                         (double)s.image_envelope[3],
                         (double)kGenericImageEnvelopeAmpXCapPx,
                         (double)kGenericImageEnvelopeAmpYCapPx);
                sentai_fr_push_event("marker_axis_envelope", env_ev);
            }
        }
        char ev[320];
        snprintf(ev, sizeof(ev),
                 "label=%s target=%d max_ms=%d reached=%d reached_k=%d segments=%d abort=%d err_last=%.2f n_full_min=%d avg=%.2f hover_rc=%d send_ok=%d pose_rej=%d flow=%d flow_err=%d",
                 labels[si], target, max_ms, seg_reached, reached_k,
                 segments_done, abort, (double)last_err,
                 n_full_min == 99 ? 0 : n_full_min, (double)win.avg_full,
                 hover_rc_last, send_ok, pose_rejects, flow_send_ok,
                 flow_read_errors);
        sentai_fr_push_event("marker_axis_motion_segment", ev);
    }

    s.st.axis_segments_done = segments_done;
    s.st.axis_ticks_done = total_ticks;
    s.st.axis_extpos_send_ok = send_ok;
    s.st.axis_pose_rejects = pose_rejects;
    s.st.axis_flow_send_ok = flow_send_ok;
    s.st.axis_flow_read_errors = flow_read_errors;
    s.st.axis_n_full_min = n_full_min == 99 ? 0 : n_full_min;
    s.st.axis_n_full_recent_avg = win.avg_full;
    s.st.axis_last_err_px = last_err;
    s.st.axis_motion_ok = (!abort && segments_done == count &&
                           win.avg_full >= (float)kMinFullMarkers) ? 1 : 0;
    char ev[180];
    snprintf(ev, sizeof(ev),
             "ok=%d segments=%d ticks=%d n_full_min=%d avg=%.2f send_ok=%d pose_rej=%d err=%.2f flow=%d flow_err=%d",
             s.st.axis_motion_ok, segments_done, total_ticks,
             s.st.axis_n_full_min, (double)win.avg_full, send_ok,
             pose_rejects, (double)last_err,
             flow_send_ok, flow_read_errors);
    sentai_fr_push_event("marker_axis_motion", ev);
    finish_(s.st.axis_motion_ok, s.st.axis_motion_ok ? 0 : 6);
}

static void run_land_() {
    sentai_fr_push_event("marker_land", "start=1");
    sentai_markers_window_reset(1, kLockAvgWindow);
    SentaiMarkersWindowStats win;
    memset(&win, 0, sizeof(win));

    float start_z = s.st.handoff_hover_z_m > 0.2f ?
        s.st.handoff_hover_z_m : s.z_target_m;
    if (s.st.handoff_last_est_m[2] > 0.2f) {
        start_z = s.st.handoff_last_est_m[2];
    }
    s.st.land_start_z_m = start_z;
    const int release_rc = release_rpyt_without_disarm_();
    s.st.land_release_rc = release_rc;

    const int ticks =
        kGenericLandTimeoutMs / SENTAI_SERVO_MARKER_TASK_PERIOD_MS;
    int ticks_done = 0;
    int send_ok = 0;
    int pose_rejects = 0;
    int flow_send_ok = 0;
    int flow_read_errors = 0;
    int n_full_min = 99;
    int hover_rc_last = -99;
    float last_pose[3] = {0.0f, 0.0f, 0.0f};
    float last_extpos[3] = {0.0f, 0.0f, 0.0f};
    float last_est[3] = {0.0f, 0.0f, 0.0f};
    int have_pose = 0;
    int have_extpos = 0;
    int have_est = 0;
    int reject_reason = 0;
    float z_cmd = start_z;

    for (int k = 0; k < ticks && !stopped_(); ++k) {
        const float t_ms =
            (float)(k * SENTAI_SERVO_MARKER_TASK_PERIOD_MS);
        float frac = t_ms / (float)kGenericLandDurMs;
        frac = clip_(frac, 0.0f, 1.0f);
        z_cmd = start_z + frac * (kGenericLandFinalZM - start_z);

        SentaiMarkersObservation o;
        sample_observation_(&o);
        sentai_markers_window_push(1, o.n_full, &win);
        if (o.n_full < n_full_min) n_full_min = o.n_full;
        const float z_visual = visual_z_(o);
        if (send_extpos_from_current_markers_(
                o, z_visual, last_pose, last_extpos, last_est,
                &have_pose, &have_extpos, &have_est, &reject_reason)) {
            send_ok++;
        } else {
            pose_rejects++;
        }
        const int flow_rc = send_flow_from_snapshot_();
        if (flow_rc > 0) flow_send_ok++;
        else if (flow_rc < 0) flow_read_errors++;
        hover_rc_last = sentai_crazy_hover(0.0f, 0.0f, 0.0f, z_cmd);
        ticks_done = k + 1;
        if (k % 10 == 0) {
            char ev[220];
            snprintf(ev, sizeof(ev),
                     "k=%d z=%.3f hover_rc=%d n=%d avg=%.2f send_ok=%d pose_rej=%d flow=%d flow_err=%d",
                     k, (double)z_cmd, hover_rc_last, o.n_full,
                     (double)win.avg_full, send_ok, pose_rejects,
                     flow_send_ok, flow_read_errors);
            sentai_fr_push_event("marker_land_tick", ev);
        }
        vTaskDelay(pdMS_TO_TICKS(SENTAI_SERVO_MARKER_TASK_PERIOD_MS));
    }

    const int disarm_rc = sentai_crazy_disarm();
    s.st.land_hover_rc_last = hover_rc_last;
    s.st.land_disarm_rc = disarm_rc;
    s.st.land_ticks_done = ticks_done;
    s.st.land_extpos_send_ok = send_ok;
    s.st.land_pose_rejects = pose_rejects;
    s.st.land_flow_send_ok = flow_send_ok;
    s.st.land_flow_read_errors = flow_read_errors;
    s.st.land_n_full_min = n_full_min == 99 ? 0 : n_full_min;
    s.st.land_n_full_recent_avg = win.avg_full;
    s.st.land_last_z_cmd_m = z_cmd;
    if (have_est) memcpy(s.st.land_last_est_m, last_est, sizeof(last_est));
    s.st.land_ok = (release_rc == 0 && hover_rc_last == 0 &&
                    disarm_rc == 0) ? 1 : 0;
    char ev[220];
    snprintf(ev, sizeof(ev),
             "ok=%d release_rc=%d hover_rc=%d disarm_rc=%d ticks=%d n_full_min=%d avg=%.2f send_ok=%d pose_rej=%d z_start=%.3f z_last=%.3f flow=%d flow_err=%d",
             s.st.land_ok, release_rc, hover_rc_last, disarm_rc,
             ticks_done, s.st.land_n_full_min, (double)win.avg_full,
             send_ok, pose_rejects, (double)start_z, (double)z_cmd,
             flow_send_ok, flow_read_errors);
    sentai_fr_push_event("marker_land", ev);
    finish_(s.st.land_ok, s.st.land_ok ? 0 : 7);
}

static void worker_(void*) {
    if (s.mode == MODE_ACQUIRE) run_acquire_();
    else if (s.mode == MODE_CENTER_HOLD) run_center_hold_();
    else if (s.mode == MODE_EXTPOS_WARMUP) run_extpos_warmup_();
    else if (s.mode == MODE_HANDOFF_HOVER) run_handoff_hover_();
    else if (s.mode == MODE_AXIS_MOTION) run_axis_motion_();
    else if (s.mode == MODE_LAND) run_land_();
    else finish_(0, -99);
    vTaskDelete(nullptr);
}

static int start_worker_(int mode, uint8_t phase) {
    if (s.task_handle != nullptr && !s.st.done) return 0;
    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (!s.stop_evt) return -1;
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    s.mode = mode;
    s.st.phase = phase;
    s.st.started = 1;
    s.st.done = 0;
    s.st.ok = 0;
    s.st.rc = 0;
    s.st.reason_code = 0;
    const BaseType_t ok = xTaskCreate(
        worker_, "servo_marker", configMINIMAL_STACK_SIZE * 4,
        nullptr, tskIDLE_PRIORITY + 2, &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_SERVO_MARKER_PHASE_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

}  // namespace

extern "C" int sentai_servo_marker_setup_start(void) {
    memset(&s.st, 0, sizeof(s.st));
    s.st.phase = SENTAI_SERVO_MARKER_PHASE_SETUP;
    s.st.started = 1;
    s.st.done = 1;

    s.st.camera_rc = sentai_calib_setup_defaults();
    if (s.st.camera_rc != 0) {
        s.st.rc = s.st.camera_rc;
        sentai_fr_push_event("marker_setup", "ok=0 reason=setup_defaults");
        return 0;
    }

    s.st.calib_loaded = sentai_calib_load() ? 1 : 0;
    sentai_calib_ini_status_t ini;
    memset(&ini, 0, sizeof(ini));
    (void)sentai_calib_get_ini_status(&ini);
    s.st.calib_strict_ok =
        (ini.present && ini.schema_ok && ini.accepted_ok && ini.status_ok &&
         ini.layout_ok && ini.required_ok && ini.strict_lines_ok) ? 1 : 0;
    if (!s.st.calib_loaded || !s.st.calib_strict_ok) {
        s.st.rc = -20;
        sentai_fr_push_event("marker_setup", "ok=0 reason=calib_ini_strict");
        return 0;
    }

    const float* R = sentai_calib_get_R_cam_to_body();
    const float* off = sentai_calib_get_cam_offset_B();
    const float* signs = sentai_calib_get_extpos_signs();
    sentai_markers_set_cam_extrinsics_matrix(off[0], off[1], off[2], R);
    memcpy(s.st.cam_offset_B, off, sizeof(s.st.cam_offset_B));
    memcpy(s.st.extpos_signs, signs, sizeof(s.st.extpos_signs));
    s.st.extpos_origin_corr_m[0] = -(1.0f - signs[0]) * off[0];
    s.st.extpos_origin_corr_m[1] = -(1.0f - signs[1]) * off[1];
    s.st.extpos_origin_corr_m[2] = 0.0f;

    float roll_sign = 0.0f;
    float pitch_sign = 0.0f;
    s.st.axis_seed_ok = sentai_calib_get_axis_seed(
        s.roll_vec, &roll_sign, s.pitch_vec, &pitch_sign) ? 1 : 0;
    memcpy(s.st.roll_vec_px, s.roll_vec, sizeof(s.roll_vec));
    memcpy(s.st.pitch_vec_px, s.pitch_vec, sizeof(s.pitch_vec));
    if (!s.st.axis_seed_ok) {
        s.st.rc = -30;
        sentai_fr_push_event("marker_setup", "ok=0 reason=missing_axis_seed");
        return 0;
    }

    (void)sentai_safety_init();
    s.st.crazy_rc = sentai_crazy_init(576000);
    s.st.flow_rc = sentai_flow_start ? sentai_flow_start(0) : -999;
    s.st.pose_subscribe_rc = sentai_crazy_pose_subscribe(33);
    s.st.setup_ok = (s.st.crazy_rc == 0) ? 1 : 0;
    s.st.ok = s.st.setup_ok;
    s.st.rc = s.st.setup_ok ? 0 : -40;
    sentai_fr_push_event("marker_setup",
                         s.st.setup_ok ? "ok=1" : "ok=0 reason=runtime_init");
    return 0;
}

extern "C" int sentai_servo_marker_acquire_start(void) {
    if (!s.st.setup_ok) return -1;
    return start_worker_(MODE_ACQUIRE,
                         SENTAI_SERVO_MARKER_PHASE_ACQUIRE);
}

extern "C" int sentai_servo_marker_center_hold_start(void) {
    if (!s.st.axis_seed_ok) return -3;
    if (!s.st.acquire_ok) return -2;
    s.current_thrust_u16 = s.st.acquire_last_thrust_u16;
    s.z_target_m = s.st.acquire_z_target_m;
    return start_worker_(MODE_CENTER_HOLD,
                         SENTAI_SERVO_MARKER_PHASE_CENTER_HOLD);
}

extern "C" int sentai_servo_marker_extpos_warmup_start(void) {
    if (!s.st.axis_seed_ok) return -3;
    s.current_thrust_u16 = s.st.center_thrust_last_u16 > 0 ?
        s.st.center_thrust_last_u16 : s.st.acquire_last_thrust_u16;
    s.z_target_m = s.st.acquire_z_target_m;
    return start_worker_(MODE_EXTPOS_WARMUP,
                         SENTAI_SERVO_MARKER_PHASE_EXTPOS_WARMUP);
}

extern "C" int sentai_servo_marker_handoff_hover_start(void) {
    if (!s.st.extpos_ok) return -4;
    s.z_target_m = s.st.acquire_z_target_m;
    return start_worker_(MODE_HANDOFF_HOVER,
                         SENTAI_SERVO_MARKER_PHASE_HANDOFF_HOVER);
}

extern "C" int sentai_servo_marker_axis_motion_start(void) {
    if (!s.st.handoff_ok) return -5;
    s.z_target_m = s.st.acquire_z_target_m;
    return start_worker_(MODE_AXIS_MOTION,
                         SENTAI_SERVO_MARKER_PHASE_IMAGE_AXIS_MOTION);
}

extern "C" int sentai_servo_marker_land_start(void) {
    if (!s.st.axis_motion_ok) return -6;
    s.z_target_m = s.st.acquire_z_target_m;
    return start_worker_(MODE_LAND, SENTAI_SERVO_MARKER_PHASE_LAND);
}

extern "C" int sentai_servo_marker_task_stop(void) {
    if (s.stop_evt) xEventGroupSetBits(s.stop_evt, STOP_BIT);
    return 0;
}

extern "C" int sentai_servo_marker_task_is_done(void) {
    return s.st.done ? 1 : 0;
}

extern "C" int sentai_servo_marker_task_result(int* ok_out, int* phase_out) {
    if (!ok_out || !phase_out) return 0;
    *ok_out = s.st.ok ? 1 : 0;
    *phase_out = s.st.phase;
    return 1;
}

extern "C" int sentai_servo_marker_task_get_status(
        sentai_servo_marker_status_t* out) {
    if (!out) return 0;
    *out = s.st;
    return 1;
}

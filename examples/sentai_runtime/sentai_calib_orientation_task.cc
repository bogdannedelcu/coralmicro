// sentai_calib_orientation_task.cc — s205 image-frame calibration task.
//
// The first migrated phase is preflight feature sampling.  Later s205 phases
// that run over time (arm-zero streaming, thrust acquisition, Z-hold, axis
// pulses, final validation, and descent) should grow into this worker instead
// of becoming blocking MicroPython or synchronous sentai.calib calls.

#include "sentai_calib_orientation_task.h"

#include "sentai_calib.h"
#include "sentai_crazy.h"
#include "sentai_fr.h"
#include "sentai_markers.h"
#include "sentai_servo.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

namespace {

#define STOP_BIT 0x01

static const int kManualDescendMs = 2600;
static const int kManualZeroMs = 300;

static const int kPreflightSamples = 20;
static const int kArmPreZeroMs = 400;
static const int kZeroUnlockMs = 1500;
static const int kArmRetryZeroMs = 500;
static const int kAcqMaxRampMs = 14000;
static const int kAcqRampMs = 8000;
static const int kAcqBaseThrustU16 = 30000;
static const int kAcqMaxThrustU16 = 34000;
static const int kAcqLockBrakeThrustU16 = 30500;
static const int kAcqLockConsecTicks = 10;
static const int kPostLockBrakeMaxMs = 1200;
static const int kPostLockBrakeMinTicks = 8;
static const int kPostLockBrakeThrustU16 = 30500;
static const int kPostLockSettleThrustU16 = 32000;
static const float kPostLockVzOkMS = 0.035f;
static const int kPostLockVzOkTicks = 4;
static const int kOpticalAxisExpectedBodyZCameraZSign = -1;
static const int kOpticalAxisMinPoseValid = 7;
static const float kOpticalAxisMinMeanTzM = 0.20f;
static const float kZCalibAltitudeGain = 1.30f;
static const float kZHoldTargetMaxM = 1.05f;
static const int kZHoldDurationMs = 4200;
static const float kZHoldTargetTolM = 0.04f;
static const int kZHoldContinueIfTargetSeen = 1;
static const int kZHoldLostMaxTicks = 15;

static const int kCenterHoldDurationMs = 14000;
static const int kCenterHoldDisarmFullMarkers = 4;
static const int kCenterHoldThrustFloorU16 = 29500;
static const float kCenterHoldKp = 0.10f;
static const float kCenterHoldMaxDeg = 0.55f;
static const float kCenterHoldDeadbandSigmaMult = 2.0f;
static const float kCenterHoldDeadbandFloorPx = 0.75f;
static const float kCenterHoldTargetVzMS = -0.09f;
static const float kCenterHoldKdThrustPerMS = 5200.0f;
static const int kCenterHoldLostMaxTicks = 5;
static const int kCenterHoldZeroMs = 200;

static const int kFinalValidateNoiseSamples = 12;
static const float kFinalValidateNoiseSigmaMult = 3.0f;
static const float kFinalValidateNoiseFloorPx = 0.6f;
static const int kHoverVisualThrustU16 = 32800;
static const int kHoldMinThrustU16 = 30000;
static const int kHoldMaxThrustU16 = 34500;
static const float kZHoldTargetM = 0.64f;
static const float kKpThrustPerM = 6500.0f;
static const float kKdThrustPerMS = 5200.0f;
static const float kVZLpfAlpha = 0.25f;
static const float kIbvsDampingPxPerDeg = 1.5f;
static const float kIbvsSustainedResponseSign = -1.0f;
static const float kIbvsZRefM = 0.64f;
static const float kIbvsZGainMin = 0.65f;
static const float kIbvsZGainMax = 1.35f;

static const int kFinalRecenterDurationMs = 12000;
static const int kFinalRecenterHoverMs = 2000;
static const int kFinalRecenterHoverMaxMs = 8000;
static const float kFinalRecenterKp = 0.075f;
static const float kFinalRecenterMaxDeg = 0.70f;
static const float kFinalRecenterDeadbandPx = 5.0f;
static const float kFinalRecenterTolPx = 15.0f;
static const float kFinalRecenterMinImprovePx = 8.0f;
static const float kFinalRecenterWorseMaxPx = 10.0f;
static const float kFinalRecenterHoverTolPx = 18.0f;

static const float kFinalValidatePulseDeg = 0.7f;
static const float kFinalValidateRetryPulse1Deg = 1.0f;
static const float kFinalValidateRetryPulse2Deg = 1.3f;
static const int kFinalValidatePulseMs = 200;
static const int kFinalValidateSettleMs = 250;
static const float kFinalValidateReturnMaxPx = 14.0f;
static const float kFinalValidateMaxPulseDeg = 1.3f;

static const float kAxisPulseDeg = 1.0f;
static const int kAxisPulseMs = 250;
static const int kAxisSettleMs = 350;
static const int kAxisMaxAxes = 2;
static const float kAxisReturnMaxPx = 12.0f;

static const int kCentroidValidationDurationMs = 2000;
static const float kCentroidValidationKp = 0.08f;
static const float kCentroidValidationMaxDeg = 0.45f;
static const float kCentroidValidationDeadbandPx = 6.0f;
static const float kCentroidValidationTolPx = 12.0f;
static const float kCentroidValidationMinImprovePx = 3.0f;
static const float kCentroidValidationWorseMaxPx = 8.0f;

enum TaskMode {
    MODE_PREFLIGHT = 1,
    MODE_ARM_ZERO_UNLOCK = 2,
    MODE_MARKER_ACQUISITION = 3,
    MODE_POST_LOCK_BRAKE = 4,
    MODE_VISUAL_Z_HOLD = 5,
    MODE_AXIS_RESPONSE_SMOKE = 6,
    MODE_CENTROID_VALIDATION = 7,
    MODE_FINAL_CANDIDATE_VALIDATION = 8,
    MODE_FINAL_RECENTER = 9,
    MODE_MANUAL_DESCEND_DISARM = 10,
    MODE_CENTER_HOLD_DESCEND_DISARM = 11,
};

struct State {
    sentai_calib_orientation_status_t st;
    TaskHandle_t task_handle;
    EventGroupHandle_t stop_evt;
    int tick_ms;
    int mode;
    int pre_zero_ms;
    int zero_unlock_ms;
    int retry_zero_ms;
    int max_ramp_ms;
    int ramp_ms;
    int base_thrust_u16;
    int max_thrust_u16;
    int lock_brake_thrust_u16;
    int lock_consec_ticks;
    int post_max_ms;
    int post_min_ticks;
    int post_brake_thrust_u16;
    int post_settle_thrust_u16;
    float post_vz_ok_m_s;
    int post_vz_ok_ticks;
    int post_lost_max_ticks;
    float post_lpf_alpha;
    float zhold_target_z_m;
    int zhold_duration_ms;
    float zhold_target_tol_m;
    int zhold_continue_if_target_seen;
    int zhold_hover_thrust_u16;
    int zhold_min_thrust_u16;
    int zhold_max_thrust_u16;
    float zhold_kp_thrust_per_m;
    float zhold_kd_thrust_per_m_s;
    int zhold_lost_max_ticks;
    float zhold_lpf_alpha;
    float axis_pulse_deg;
    int axis_pulse_ms;
    int axis_settle_ms;
    int axis_max_axes;
    int axis_hard_min_full_markers;
    float axis_min_avg_full_markers;
    float axis_response_min_px;
    float axis_dominance_ratio_min;
    float axis_orthogonal_dot_max_norm;
    float axis_return_max_px;
    int axis_hover_thrust_u16;
    int axis_min_thrust_u16;
    int axis_max_thrust_u16;
    float axis_runtime_z_target_m;
    float axis_kp_thrust_per_m;
    float axis_kd_thrust_per_m_s;
    float axis_lpf_alpha;
    int centroid_duration_ms;
    float centroid_roll_vec[2];
    float centroid_pitch_vec[2];
    float centroid_gain;
    float centroid_max_deg;
    float centroid_deadband_px;
    float centroid_tol_px;
    float centroid_min_improve_px;
    float centroid_worse_max_px;
    int centroid_hard_min_full_markers;
    int centroid_hover_thrust_u16;
    int centroid_min_thrust_u16;
    int centroid_max_thrust_u16;
    float centroid_runtime_z_target_m;
    float centroid_kp_thrust_per_m;
    float centroid_kd_thrust_per_m_s;
    float centroid_damping_px_per_deg;
    float centroid_sustained_response_sign;
    float centroid_z_ref_m;
    float centroid_z_gain_min;
    float centroid_z_gain_max;
    float centroid_lpf_alpha;
    float final_R[9];
    float final_pulse_deg;
    float final_retry_pulse_1_deg;
    float final_retry_pulse_2_deg;
    int final_pulse_ms;
    int final_settle_ms;
    float final_return_max_px;
    float final_max_pulse_deg;
    int final_noise_samples;
    float final_noise_sigma_mult;
    float final_noise_floor_px;
    float final_dominance_ratio_min;
    int final_hard_min_full_markers;
    int recenter_duration_ms;
    int recenter_hover_ms;
    int recenter_hover_max_ms;
    float recenter_tol_px;
    float recenter_min_improve_px;
    float recenter_worse_max_px;
    float recenter_hover_tol_px;
    int recenter_hard_min_full_markers;
    float recenter_avg_full_threshold;
    int manual_duration_ms;
    int manual_start_thrust_u16;
    int manual_zero_ms;
    int center_duration_ms;
    int center_start_thrust_u16;
    int center_disarm_full_markers;
    int center_avg_window;
    float center_target_vz_m_s;
    int center_thrust_floor_u16;
    float center_kd_thrust_per_m_s;
    int center_lost_max_ticks;
    float center_deadband_sigma_mult;
    float center_deadband_floor_px;
    int center_zero_ms;
    float last_axis_roll_vec[2];
    float last_axis_pitch_vec[2];
    int last_axis_roll_axis_code;
    int last_axis_pitch_axis_code;
    int last_axis_roll_sign;
    int last_axis_pitch_sign;
    int last_axis_roll_valid;
    int last_axis_pitch_valid;
    float last_acq_z_cam_max_m;
    float last_post_z_cam_max_m;
    int final_R_valid;
    int candidate_ok;
    int candidate_best_idx;
    float candidate_best_score;
    float candidate_second_score;
    float candidate_margin;
    float candidate_det;
    int current_thrust_u16;
};

static State s = {};

inline bool stop_requested_() {
    return s.stop_evt &&
           (xEventGroupGetBits(s.stop_evt) & STOP_BIT) != 0;
}

void reset_status_(uint8_t phase, int tick_ms) {
    memset(&s.st, 0, sizeof(s.st));
    s.st.phase = phase;
    s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_RUNNING;
    s.st.started = 1;
    s.st.done = 0;
    s.st.arm_rc = -999;
    s.st.arm_retry_rc = -999;
    s.st.acq_first_seen_tick = -1;
    s.st.zhold_thrust_min_u16 = 999999;
    s.st.axis_pitch_min_full_markers = 99;
    s.st.axis_roll_min_full_markers = 99;
    s.st.axis_thrust_last_u16 = 0;
    s.st.centroid_n_full_min = 99;
    s.tick_ms = tick_ms;
}

const char* axis_code_name_(int code) {
    return code == 1 ? "y" : "x";
}

int sample_observation_(SentaiMarkersObservation* o) {
    sentai_calib_defaults_t d;
    (void)sentai_calib_get_defaults(&d);
    const int rc = sentai_calib_sample_observation(
        d.img_w, d.img_h, d.full_vis_margin_px, o);
    if (rc < -1) {
        memset(o, 0, sizeof(*o));
        o->n_raw = rc;
    }
    return rc;
}

struct MarkerWindow {
    int vals[64];
    int size;
    int count;
    int pos;
    int sum;
};

void marker_window_init_(MarkerWindow* w, int size) {
    memset(w, 0, sizeof(*w));
    if (size < 1) size = 1;
    if (size > 64) size = 64;
    w->size = size;
}

void marker_window_push_(MarkerWindow* w, int n_full) {
    if (w->count >= w->size) {
        w->sum -= w->vals[w->pos];
    } else {
        w->count += 1;
    }
    w->vals[w->pos] = n_full;
    w->sum += n_full;
    w->pos = (w->pos + 1) % w->size;
}

float marker_window_avg_(const MarkerWindow* w) {
    return w->count > 0 ? ((float)w->sum / (float)w->count) : 0.0f;
}

int marker_window_ready_(const MarkerWindow* w) {
    return w->count >= w->size ? 1 : 0;
}

void sample_preflight_() {
    sentai_calib_defaults_t d;
    sentai_calib_limits_t lim;
    (void)sentai_calib_get_defaults(&d);
    (void)sentai_calib_get_limits(&lim);

    sentai_fr_push_event("phase_preflight_features", "start");
    for (int k = 0; k < s.st.samples; ++k) {
        if (stop_requested_()) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
            s.st.done = 1;
            return;
        }

        SentaiMarkersObservation o;
        const int rc = sentai_calib_sample_observation(
            d.img_w, d.img_h, d.full_vis_margin_px, &o);
        if (rc < -1) {
            memset(&o, 0, sizeof(o));
            o.n_raw = rc;
        }

        if (o.n_full >= lim.min_full_markers) {
            s.st.ok_full_ticks += 1;
        }
        if (o.n_full > s.st.n_full_max) {
            s.st.n_full_max = o.n_full;
        }
        if (o.radius_mean_px > s.st.radius_max_px) {
            s.st.radius_max_px = o.radius_mean_px;
        }
        if (o.z_cam_mean_m > s.st.z_cam_max_m) {
            s.st.z_cam_max_m = o.z_cam_mean_m;
        }

        char ev[160];
        snprintf(ev, sizeof(ev),
                 "k=%d n_raw=%d n_full=%d cx=%.2f cy=%.2f radius=%.2f z=%.3f",
                 k, o.n_raw, o.n_full,
                 o.valid ? (double)o.centroid_x : 0.0,
                 o.valid ? (double)o.centroid_y : 0.0,
                 (double)o.radius_mean_px, (double)o.z_cam_mean_m);
        sentai_fr_push_event("feature_tick", ev);

        if (s.tick_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
        }
    }

    s.st.feature_lock =
        (s.st.ok_full_ticks >= (s.st.samples / 2)) ? 1 : 0;
    s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
    s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    s.st.done = 1;

    char ev[96];
    snprintf(ev, sizeof(ev), "ok_ticks=%d samples=%d lock=%d",
             s.st.ok_full_ticks, s.st.samples, s.st.feature_lock ? 1 : 0);
    sentai_fr_push_event("phase_preflight_features", ev);
}

int send_rpyt_(float roll, float pitch, float yaw_rate, int thrust_u16) {
    if (thrust_u16 < 0) thrust_u16 = 0;
    if (thrust_u16 > 65535) thrust_u16 = 65535;
    uint16_t thrust = (uint16_t)thrust_u16;
    uint8_t data[14];
    int idx = 0;
    memcpy(data + idx, &roll, 4); idx += 4;
    memcpy(data + idx, &pitch, 4); idx += 4;
    memcpy(data + idx, &yaw_rate, 4); idx += 4;
    data[idx++] = (uint8_t)(thrust & 0xFF);
    data[idx++] = (uint8_t)((thrust >> 8) & 0xFF);
    return sentai_crazy_send_crtp(3, 0, data, idx);
}

int send_zero_rpyt_() {
    return send_rpyt_(0.0f, 0.0f, 0.0f, 0);
}

int stream_zero_for_ms_(int duration_ms) {
    if (duration_ms < 0) duration_ms = 0;
    int n_ticks = duration_ms / s.tick_ms;
    if (duration_ms > 0 && n_ticks < 1) n_ticks = 1;
    int last_rc = 0;
    for (int i = 0; i < n_ticks; ++i) {
        if (stop_requested_()) return last_rc;
        last_rc = send_zero_rpyt_();
        s.st.zero_packets += 1;
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
    }
    return last_rc;
}

void arm_zero_unlock_() {
    sentai_fr_push_event("phase_arm_zero_unlock", "start");
    (void)stream_zero_for_ms_(s.pre_zero_ms);
    if (stop_requested_()) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
        s.st.done = 1;
        return;
    }

    s.st.arm_rc = sentai_crazy_arm();
    if (s.st.arm_rc != 0) {
        char ev[64];
        snprintf(ev, sizeof(ev), "arm_rc=%d", s.st.arm_rc);
        sentai_fr_push_event("arm_error", ev);
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return;
    }

    (void)stream_zero_for_ms_(s.zero_unlock_ms);
    if (!stop_requested_()) {
        s.st.arm_retry_rc = sentai_crazy_arm();
    }
    (void)stream_zero_for_ms_(s.retry_zero_ms);

    if (stop_requested_()) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
    } else {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    }
    s.st.done = 1;

    char ev[128];
    snprintf(ev, sizeof(ev),
             "pre_zero_ms=%d zero_unlock_ms=%d retry_zero_ms=%d arm_retry=%d",
             s.pre_zero_ms, s.zero_unlock_ms, s.retry_zero_ms,
             s.st.arm_retry_rc == 0 ? 1 : 0);
    sentai_fr_push_event("phase_arm_zero_unlock", ev);
}

void marker_acquisition_() {
    sentai_calib_defaults_t d;
    (void)sentai_calib_get_defaults(&d);
    sentai_fr_push_event("phase_thrust_only_marker_acquisition", "start");

    MarkerWindow win;
    marker_window_init_(&win, s.lock_consec_ticks);
    int ticks = s.max_ramp_ms / s.tick_ms;
    if (ticks < 1) ticks = 1;
    s.st.acq_last_thrust_u16 = s.base_thrust_u16;

    for (int k = 0; k < ticks; ++k) {
        if (stop_requested_()) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
            s.st.done = 1;
            return;
        }

        const float t_s = ((float)k * (float)s.tick_ms) / 1000.0f;
        int thrust = s.lock_brake_thrust_u16;
        if (s.st.acq_lock_streak <= 0) {
            float alpha = 1.0f;
            if (s.ramp_ms > 0) {
                alpha = ((float)(k * s.tick_ms)) / (float)s.ramp_ms;
                if (alpha > 1.0f) alpha = 1.0f;
                if (alpha < 0.0f) alpha = 0.0f;
            }
            thrust = s.base_thrust_u16 +
                (int)(alpha * (float)(s.max_thrust_u16 - s.base_thrust_u16));
        }
        s.st.acq_last_thrust_u16 = thrust;
        (void)send_rpyt_(0.0f, 0.0f, 0.0f, thrust);

        SentaiMarkersObservation o;
        const int rc = sentai_calib_sample_observation(
            d.img_w, d.img_h, d.full_vis_margin_px, &o);
        if (rc < -1) {
            memset(&o, 0, sizeof(o));
            o.n_raw = rc;
        }

        marker_window_push_(&win, o.n_full);
        const int avg_ready = marker_window_ready_(&win);
        const float avg_full = marker_window_avg_(&win);

        const int has_lock = sentai_calib_feature_has_lock(
            o.n_full, o.radius_mean_px, o.valid, avg_ready, avg_full);
        if (has_lock) s.st.acq_lock_streak += 1;
        else s.st.acq_lock_streak = 0;

        if (o.n_full > s.st.acq_n_full_max) {
            s.st.acq_n_full_max = o.n_full;
        }
        if (o.radius_mean_px > s.st.acq_radius_max_px) {
            s.st.acq_radius_max_px = o.radius_mean_px;
        }
        if (o.z_cam_mean_m > 0.0f) {
            s.st.acq_z_cam_last_m = o.z_cam_mean_m;
            if (s.st.acq_z_cam_min_m <= 0.0f ||
                    o.z_cam_mean_m < s.st.acq_z_cam_min_m) {
                s.st.acq_z_cam_min_m = o.z_cam_mean_m;
            }
            if (o.z_cam_mean_m > s.st.acq_z_cam_max_m) {
                s.st.acq_z_cam_max_m = o.z_cam_mean_m;
            }
        }
        if (s.st.acq_first_seen_tick < 0 && o.n_full > 0) {
            s.st.acq_first_seen_tick = k;
        }

        if ((k % 10) == 0 || s.st.acq_lock_streak > 0) {
            char ev[192];
            snprintf(ev, sizeof(ev),
                     "k=%d t_s=%.3f thrust=%d n_raw=%d n_full=%d avg=%.2f lock=%d radius=%.2f z=%.3f",
                     k, (double)t_s, thrust, o.n_raw, o.n_full,
                     (double)avg_full, s.st.acq_lock_streak,
                     (double)o.radius_mean_px, (double)o.z_cam_mean_m);
            sentai_fr_push_event("acq_tick", ev);
        }

        if (s.st.acq_lock_streak >= s.lock_consec_ticks) {
            s.st.acq_locked = 1;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
    }

    s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
    s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    s.st.done = 1;
    s.last_acq_z_cam_max_m = s.st.acq_z_cam_max_m;

    char ev[160];
    snprintf(ev, sizeof(ev),
             "locked=%d last_thrust=%d n_full_max=%d radius_max=%.2f z_last=%.3f first_seen=%d",
             s.st.acq_locked ? 1 : 0, s.st.acq_last_thrust_u16,
             s.st.acq_n_full_max, (double)s.st.acq_radius_max_px,
             (double)s.st.acq_z_cam_last_m, s.st.acq_first_seen_tick);
    sentai_fr_push_event("phase_thrust_only_marker_acquisition", ev);
}

void post_lock_brake_() {
    sentai_calib_defaults_t d;
    sentai_calib_limits_t lim;
    (void)sentai_calib_get_defaults(&d);
    (void)sentai_calib_get_limits(&lim);
    sentai_fr_push_event("phase_post_lock_brake", "start");

    MarkerWindow win;
    marker_window_init_(&win, lim.marker_count_avg_window);
    int ticks = s.post_max_ms / s.tick_ms;
    if (ticks < 1) ticks = 1;
    int lost_ticks = 0;
    float z_prev = 0.0f;
    s.st.post_thrust_last_u16 = s.post_brake_thrust_u16;

    for (int k = 0; k < ticks; ++k) {
        if (stop_requested_()) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
            s.st.done = 1;
            return;
        }

        SentaiMarkersObservation o;
        const int rc = sentai_calib_sample_observation(
            d.img_w, d.img_h, d.full_vis_margin_px, &o);
        if (rc < -1) {
            memset(&o, 0, sizeof(o));
            o.n_raw = rc;
        }

        marker_window_push_(&win, o.n_full);
        const int ready = marker_window_ready_(&win);
        const float avg_full = marker_window_avg_(&win);
        const int valid = sentai_calib_marker_avg_lock_ok(
            ready, avg_full, -1.0f) && o.z_cam_mean_m > 0.0f;
        if (valid) {
            s.st.post_valid_ticks += 1;
            lost_ticks = 0;
            const float z = o.z_cam_mean_m;
            if (z_prev > 0.0f) {
                const float vz = (z - z_prev) * (1000.0f / (float)s.tick_ms);
                s.st.post_vz_filt_m_s =
                    (1.0f - s.post_lpf_alpha) * s.st.post_vz_filt_m_s +
                    s.post_lpf_alpha * vz;
            }
            z_prev = z;
            s.st.post_z_cam_last_m = z;
            if (s.st.post_z_cam_min_m <= 0.0f ||
                    z < s.st.post_z_cam_min_m) {
                s.st.post_z_cam_min_m = z;
            }
            if (z > s.st.post_z_cam_max_m) {
                s.st.post_z_cam_max_m = z;
            }

            if (s.st.post_vz_filt_m_s > s.post_vz_ok_m_s) {
                s.st.post_thrust_last_u16 = s.post_brake_thrust_u16;
                s.st.post_ok_vz_ticks = 0;
            } else {
                s.st.post_thrust_last_u16 = s.post_settle_thrust_u16;
                s.st.post_ok_vz_ticks += 1;
            }
        } else {
            lost_ticks += 1;
            s.st.post_thrust_last_u16 = s.post_settle_thrust_u16;
            if (sentai_calib_marker_avg_unsafe(ready, avg_full, -1.0f) ||
                    lost_ticks >= s.post_lost_max_ticks) {
                s.st.post_abort_code = 1;
            }
        }

        (void)send_rpyt_(0.0f, 0.0f, 0.0f, s.st.post_thrust_last_u16);
        if ((k % 3) == 0 || lost_ticks > 0) {
            char ev[160];
            snprintf(ev, sizeof(ev),
                     "k=%d thrust=%d n_full=%d avg=%.2f z=%.3f vz=%.3f ok_vz=%d lost=%d",
                     k, s.st.post_thrust_last_u16, o.n_full,
                     (double)avg_full, (double)o.z_cam_mean_m,
                     (double)s.st.post_vz_filt_m_s,
                     s.st.post_ok_vz_ticks, lost_ticks);
            sentai_fr_push_event("post_lock_brake_tick", ev);
        }

        if (s.st.post_abort_code != 0) break;
        if (k >= s.post_min_ticks &&
                s.st.post_ok_vz_ticks >= s.post_vz_ok_ticks) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
    }

    s.st.post_ok = (s.st.post_abort_code == 0 &&
                    s.st.post_valid_ticks > 0 &&
                    s.st.post_z_cam_last_m > 0.0f) ? 1 : 0;
    s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
    s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    s.st.done = 1;
    s.last_post_z_cam_max_m = s.st.post_z_cam_max_m;

    char ev[128];
    snprintf(ev, sizeof(ev),
             "ok=%d abort=%d valid_ticks=%d z_last=%.3f vz=%.3f thrust=%d ok_vz=%d",
             s.st.post_ok ? 1 : 0, s.st.post_abort_code,
             s.st.post_valid_ticks, (double)s.st.post_z_cam_last_m,
             (double)s.st.post_vz_filt_m_s, s.st.post_thrust_last_u16,
             s.st.post_ok_vz_ticks);
    sentai_fr_push_event("phase_post_lock_brake", ev);
}

void visual_z_hold_() {
    sentai_calib_defaults_t d;
    sentai_calib_limits_t lim;
    (void)sentai_calib_get_defaults(&d);
    (void)sentai_calib_get_limits(&lim);
    sentai_fr_push_event("phase_visual_z_hold", "start");

    MarkerWindow win;
    marker_window_init_(&win, lim.marker_count_avg_window);
    int ticks = s.zhold_duration_ms / s.tick_ms;
    if (ticks < 1) ticks = 1;
    s.st.zhold_ticks_requested = ticks;
    s.st.zhold_thrust_last_u16 = s.zhold_hover_thrust_u16;
    s.st.zhold_thrust_min_u16 = s.zhold_hover_thrust_u16;
    s.st.zhold_thrust_max_u16 = s.zhold_hover_thrust_u16;
    int lost_ticks = 0;
    float z_prev = 0.0f;

    for (int k = 0; k < ticks; ++k) {
        if (stop_requested_()) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
            s.st.done = 1;
            return;
        }

        SentaiMarkersObservation o;
        const int rc = sentai_calib_sample_observation(
            d.img_w, d.img_h, d.full_vis_margin_px, &o);
        if (rc < -1) {
            memset(&o, 0, sizeof(o));
            o.n_raw = rc;
        }

        marker_window_push_(&win, o.n_full);
        const int ready = marker_window_ready_(&win);
        const float avg_full = marker_window_avg_(&win);
        const int valid = sentai_calib_marker_avg_lock_ok(
            ready, avg_full, -1.0f) && o.z_cam_mean_m > 0.0f;
        if (valid) {
            s.st.zhold_valid_ticks += 1;
            lost_ticks = 0;
            const float z = o.z_cam_mean_m;
            s.st.zhold_z_cam_last_m = z;
            if (s.st.zhold_z_cam_min_m <= 0.0f ||
                    z < s.st.zhold_z_cam_min_m) {
                s.st.zhold_z_cam_min_m = z;
            }
            if (z > s.st.zhold_z_cam_max_m) {
                s.st.zhold_z_cam_max_m = z;
            }

            int thrust = 0;
            float z_prev_out = 0.0f;
            float vz_out = 0.0f;
            const int ok = sentai_calib_z_hold_thrust(
                z, z_prev, s.st.zhold_vz_filt_m_s, s.zhold_target_z_m,
                s.zhold_hover_thrust_u16, s.zhold_kp_thrust_per_m,
                s.zhold_kd_thrust_per_m_s,
                ((float)s.tick_ms) / 1000.0f, s.zhold_lpf_alpha,
                s.zhold_min_thrust_u16, s.zhold_max_thrust_u16,
                &thrust, &z_prev_out, &vz_out);
            if (ok) {
                s.st.zhold_thrust_last_u16 = thrust;
                z_prev = z_prev_out;
                s.st.zhold_vz_filt_m_s = vz_out;
            } else {
                s.st.zhold_thrust_last_u16 = s.zhold_hover_thrust_u16;
            }
        } else {
            lost_ticks += 1;
            s.st.zhold_thrust_last_u16 = s.zhold_hover_thrust_u16;
            if (s.st.zhold_thrust_last_u16 < s.zhold_min_thrust_u16) {
                s.st.zhold_thrust_last_u16 = s.zhold_min_thrust_u16;
            }
            if (sentai_calib_marker_avg_unsafe(ready, avg_full, -1.0f) ||
                    lost_ticks >= s.zhold_lost_max_ticks) {
                s.st.zhold_abort_code = 1;
            }
        }

        if (s.st.zhold_thrust_last_u16 < s.st.zhold_thrust_min_u16) {
            s.st.zhold_thrust_min_u16 = s.st.zhold_thrust_last_u16;
        }
        if (s.st.zhold_thrust_last_u16 > s.st.zhold_thrust_max_u16) {
            s.st.zhold_thrust_max_u16 = s.st.zhold_thrust_last_u16;
        }
        (void)send_rpyt_(0.0f, 0.0f, 0.0f,
                         s.st.zhold_thrust_last_u16);

        if ((k % 5) == 0 || lost_ticks > 0) {
            char ev[160];
            snprintf(ev, sizeof(ev),
                     "k=%d thrust=%d n_full=%d avg=%.2f z=%.3f vz=%.3f lost=%d",
                     k, s.st.zhold_thrust_last_u16, o.n_full,
                     (double)avg_full, (double)o.z_cam_mean_m,
                     (double)s.st.zhold_vz_filt_m_s, lost_ticks);
            sentai_fr_push_event("zhold_tick", ev);
        }

        if (s.st.zhold_abort_code != 0) break;
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
    }

    s.st.zhold_target_reached_last =
        (s.st.zhold_z_cam_last_m >=
         (s.zhold_target_z_m - s.zhold_target_tol_m)) ? 1 : 0;
    s.st.zhold_target_reached_peak =
        (s.st.zhold_z_cam_max_m >=
         (s.zhold_target_z_m - s.zhold_target_tol_m)) ? 1 : 0;
    s.st.zhold_target_reached =
        (s.st.zhold_target_reached_last ||
         (s.zhold_continue_if_target_seen &&
          s.st.zhold_target_reached_peak)) ? 1 : 0;
    s.st.zhold_ok = (s.st.zhold_abort_code == 0 &&
                     s.st.zhold_valid_ticks >= (ticks / 2) &&
                     s.st.zhold_z_cam_last_m > 0.0f &&
                     s.st.zhold_target_reached) ? 1 : 0;
    s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
    s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    s.st.done = 1;

    char ev[160];
    snprintf(ev, sizeof(ev),
             "ok=%d abort=%d target=%.3f reached=%d valid=%d z_last=%.3f thrust=%d",
             s.st.zhold_ok ? 1 : 0, s.st.zhold_abort_code,
             (double)s.zhold_target_z_m,
             s.st.zhold_target_reached ? 1 : 0,
             s.st.zhold_valid_ticks,
             (double)s.st.zhold_z_cam_last_m,
             s.st.zhold_thrust_last_u16);
    sentai_fr_push_event("phase_visual_z_hold", ev);
}

int axis_z_hold_thrust_(const SentaiMarkersObservation& o,
                        float* z_prev,
                        float* vz_filt) {
    int thrust = s.axis_hover_thrust_u16;
    float z_prev_out = *z_prev;
    float vz_out = *vz_filt;
    const int ok = sentai_calib_z_hold_thrust(
        o.z_cam_mean_m, *z_prev, *vz_filt, s.axis_runtime_z_target_m,
        s.axis_hover_thrust_u16, s.axis_kp_thrust_per_m,
        s.axis_kd_thrust_per_m_s, ((float)s.tick_ms) / 1000.0f,
        s.axis_lpf_alpha, s.axis_min_thrust_u16, s.axis_max_thrust_u16,
        &thrust, &z_prev_out, &vz_out);
    if (ok) {
        *z_prev = z_prev_out;
        *vz_filt = vz_out;
    }
    return thrust;
}

struct AxisSegment {
    SentaiMarkersObservation first;
    SentaiMarkersObservation last;
    int ticks;
    int min_full;
    float avg_full;
};

AxisSegment stream_axis_segment_(const char* label,
                                 float roll_deg,
                                 float pitch_deg,
                                 int duration_ms,
                                 float* z_prev,
                                 float* vz_filt) {
    AxisSegment seg;
    memset(&seg, 0, sizeof(seg));
    seg.min_full = 99;
    int ticks = duration_ms / s.tick_ms;
    if (duration_ms > 0 && ticks < 1) ticks = 1;
    if (ticks < 1) ticks = 1;
    seg.ticks = ticks;
    float n_full_sum = 0.0f;

    for (int k = 0; k < ticks; ++k) {
        SentaiMarkersObservation o;
        (void)sample_observation_(&o);
        const int thrust = axis_z_hold_thrust_(o, z_prev, vz_filt);
        s.st.axis_thrust_last_u16 = thrust;
        (void)send_rpyt_(roll_deg, pitch_deg, 0.0f, thrust);
        if (k == 0) seg.first = o;
        seg.last = o;
        if (o.n_full < seg.min_full) seg.min_full = o.n_full;
        n_full_sum += (float)o.n_full;
        if (k == 0 || k == ticks - 1) {
            char ev[180];
            snprintf(ev, sizeof(ev),
                     "label=%s k=%d roll=%.2f pitch=%.2f thrust=%d n_full=%d cx=%.2f cy=%.2f z=%.3f vz=%.3f",
                     label, k, (double)roll_deg, (double)pitch_deg,
                     thrust, o.n_full, (double)o.centroid_x,
                     (double)o.centroid_y, (double)o.z_cam_mean_m,
                     (double)*vz_filt);
            sentai_fr_push_event("axis_segment_tick", ev);
        }
        if (s.tick_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
        }
        if (stop_requested_()) break;
    }
    if (seg.min_full == 99) seg.min_full = 0;
    seg.avg_full = n_full_sum / (float)seg.ticks;

    char ev[128];
    snprintf(ev, sizeof(ev),
             "label=%s ticks=%d min_full=%d avg_full=%.2f",
             label, seg.ticks, seg.min_full, (double)seg.avg_full);
    sentai_fr_push_event("axis_segment_summary", ev);
    return seg;
}

void store_axis_result_(int is_roll,
                        float comp_dx,
                        float comp_dy,
                        int dominant_axis,
                        int dominant_sign,
                        float dominance,
                        float strength,
                        int axis_sign_ok,
                        int min_full,
                        float avg_full,
                        int marker_lock_ok,
                        float return_err,
                        int return_ok,
                        float z_last) {
    if (is_roll) {
        s.last_axis_roll_vec[0] = comp_dx;
        s.last_axis_roll_vec[1] = comp_dy;
        s.last_axis_roll_axis_code = dominant_axis;
        s.last_axis_roll_sign = dominant_sign;
        s.last_axis_roll_valid = axis_sign_ok ? 1 : 0;
        s.st.axis_roll_comp_dx = comp_dx;
        s.st.axis_roll_comp_dy = comp_dy;
        s.st.axis_roll_dominant_axis_code = dominant_axis;
        s.st.axis_roll_dominant_sign = dominant_sign;
        s.st.axis_roll_dominance_ratio = dominance;
        s.st.axis_roll_response_strength_px = strength;
        s.st.axis_roll_sign_ok = axis_sign_ok ? 1 : 0;
        s.st.axis_roll_min_full_markers = min_full;
        s.st.axis_roll_avg_full_markers = avg_full;
        s.st.axis_roll_marker_lock_ok = marker_lock_ok ? 1 : 0;
        s.st.axis_roll_return_err_px = return_err;
        s.st.axis_roll_return_ok = return_ok ? 1 : 0;
        s.st.axis_roll_z_last_m = z_last;
    } else {
        s.last_axis_pitch_vec[0] = comp_dx;
        s.last_axis_pitch_vec[1] = comp_dy;
        s.last_axis_pitch_axis_code = dominant_axis;
        s.last_axis_pitch_sign = dominant_sign;
        s.last_axis_pitch_valid = axis_sign_ok ? 1 : 0;
        s.st.axis_pitch_comp_dx = comp_dx;
        s.st.axis_pitch_comp_dy = comp_dy;
        s.st.axis_pitch_dominant_axis_code = dominant_axis;
        s.st.axis_pitch_dominant_sign = dominant_sign;
        s.st.axis_pitch_dominance_ratio = dominance;
        s.st.axis_pitch_response_strength_px = strength;
        s.st.axis_pitch_sign_ok = axis_sign_ok ? 1 : 0;
        s.st.axis_pitch_min_full_markers = min_full;
        s.st.axis_pitch_avg_full_markers = avg_full;
        s.st.axis_pitch_marker_lock_ok = marker_lock_ok ? 1 : 0;
        s.st.axis_pitch_return_err_px = return_err;
        s.st.axis_pitch_return_ok = return_ok ? 1 : 0;
        s.st.axis_pitch_z_last_m = z_last;
    }
}

void axis_response_smoke_() {
    sentai_fr_push_event("phase_axis_response_smoke", "start");
    SentaiMarkersObservation baseline;
    (void)sample_observation_(&baseline);
    float z_prev = baseline.z_cam_mean_m;
    float vz_filt = 0.0f;
    int ok = 1;

    for (int axis_idx = 0; axis_idx < 2; ++axis_idx) {
        if (axis_idx >= s.axis_max_axes) break;
        const int is_roll = axis_idx == 1;
        const char* name = is_roll ? "roll" : "pitch";
        const float roll_cmd = is_roll ? s.axis_pulse_deg : 0.0f;
        const float pitch_cmd = is_roll ? 0.0f : s.axis_pulse_deg;

        SentaiMarkersObservation axis_base;
        (void)sample_observation_(&axis_base);
        if (axis_base.n_full < s.axis_hard_min_full_markers) {
            s.st.axis_abort_code = 1;
            ok = 0;
            char ev[128];
            snprintf(ev, sizeof(ev),
                     "axis=%s reason=weak_marker_lock_before_axis_pulse n_full=%d",
                     name, axis_base.n_full);
            sentai_fr_push_event("axis_response_abort", ev);
            break;
        }

        char pos_label[32];
        char neg_label[32];
        char settle_label[32];
        snprintf(pos_label, sizeof(pos_label), "%s_pos", name);
        snprintf(neg_label, sizeof(neg_label), "%s_neg", name);
        snprintf(settle_label, sizeof(settle_label), "%s_settle", name);
        AxisSegment pos = stream_axis_segment_(
            pos_label, roll_cmd, pitch_cmd, s.axis_pulse_ms,
            &z_prev, &vz_filt);
        if (stop_requested_()) break;
        AxisSegment neg = stream_axis_segment_(
            neg_label, -roll_cmd, -pitch_cmd, s.axis_pulse_ms,
            &z_prev, &vz_filt);
        if (stop_requested_()) break;
        AxisSegment settle = stream_axis_segment_(
            settle_label, 0.0f, 0.0f, s.axis_settle_ms,
            &z_prev, &vz_filt);
        if (stop_requested_()) break;

        int min_full = pos.min_full;
        if (neg.min_full < min_full) min_full = neg.min_full;
        if (settle.min_full < min_full) min_full = settle.min_full;
        const int total_ticks = pos.ticks + neg.ticks + settle.ticks;
        const float avg_full =
            (pos.avg_full * (float)pos.ticks +
             neg.avg_full * (float)neg.ticks +
             settle.avg_full * (float)settle.ticks) /
            (float)total_ticks;
        const int marker_lock_ok =
            sentai_calib_marker_lock_ok(min_full, avg_full);
        if (!marker_lock_ok) ok = 0;

        const float pos_seg_dx = pos.last.centroid_x - pos.first.centroid_x;
        const float pos_seg_dy = pos.last.centroid_y - pos.first.centroid_y;
        const float neg_seg_dx = neg.last.centroid_x - neg.first.centroid_x;
        const float neg_seg_dy = neg.last.centroid_y - neg.first.centroid_y;
        const float comp_dx = 0.5f * (pos_seg_dx - neg_seg_dx);
        const float comp_dy = 0.5f * (pos_seg_dy - neg_seg_dy);
        int dominant_axis = 0;
        int dominant_sign = 0;
        float dominance = 0.0f;
        float strength = 0.0f;
        (void)sentai_calib_axis_observation_from_delta(
            comp_dx, comp_dy, &dominant_axis, &dominant_sign,
            &dominance, &strength);
        const int axis_sign_ok =
            (strength >= s.axis_response_min_px &&
             dominance >= s.axis_dominance_ratio_min) ? 1 : 0;
        if (!axis_sign_ok) ok = 0;

        SentaiMarkersObservation cur;
        (void)sample_observation_(&cur);
        if (cur.n_full < min_full) min_full = cur.n_full;
        const float return_dx = cur.centroid_x - axis_base.centroid_x;
        const float return_dy = cur.centroid_y - axis_base.centroid_y;
        const float return_err = sqrtf(return_dx * return_dx +
                                       return_dy * return_dy);
        const int return_ok =
            (return_err <= s.axis_return_max_px) ? 1 : 0;

        store_axis_result_(is_roll, comp_dx, comp_dy, dominant_axis,
                           dominant_sign, dominance, strength,
                           axis_sign_ok, min_full, avg_full,
                           marker_lock_ok, return_err, return_ok,
                           cur.z_cam_mean_m);
        s.st.axis_results_count += 1;

        char ev[192];
        snprintf(ev, sizeof(ev),
                 "axis=%s comp=%.2f,%.2f dom_axis=%d sign=%d dominance=%.2f strength=%.2f min_full=%d avg_full=%.2f return_err=%.2f ok=%d",
                 name, (double)comp_dx, (double)comp_dy, dominant_axis,
                 dominant_sign, (double)dominance, (double)strength,
                 min_full, (double)avg_full, (double)return_err,
                 axis_sign_ok && marker_lock_ok);
        sentai_fr_push_event("axis_response", ev);
        if (!axis_sign_ok) break;
    }

    if (s.st.axis_results_count >= 2) {
        const float ps = sqrtf(s.st.axis_pitch_comp_dx *
                               s.st.axis_pitch_comp_dx +
                               s.st.axis_pitch_comp_dy *
                               s.st.axis_pitch_comp_dy);
        const float rs = sqrtf(s.st.axis_roll_comp_dx *
                               s.st.axis_roll_comp_dx +
                               s.st.axis_roll_comp_dy *
                               s.st.axis_roll_comp_dy);
        float dot_norm = 1.0f;
        if (ps > 0.0001f && rs > 0.0001f) {
            dot_norm = ((s.st.axis_pitch_comp_dx * s.st.axis_roll_comp_dx +
                         s.st.axis_pitch_comp_dy * s.st.axis_roll_comp_dy) /
                        (ps * rs));
        }
        s.st.axis_orthogonality_present = 1;
        s.st.axis_orthogonality_dot_norm = dot_norm;
        s.st.axis_orthogonality_ok =
            (fabsf(dot_norm) <= s.axis_orthogonal_dot_max_norm) ? 1 : 0;
        if (!s.st.axis_orthogonality_ok) ok = 0;
    }

    if (stop_requested_()) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
    } else {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    }
    s.st.axis_ok = (ok && s.st.axis_results_count >= s.axis_max_axes) ? 1 : 0;
    s.st.done = 1;

    char ev[128];
    snprintf(ev, sizeof(ev),
             "ok=%d results=%d abort=%d thrust=%d",
             s.st.axis_ok ? 1 : 0, s.st.axis_results_count,
             s.st.axis_abort_code, s.st.axis_thrust_last_u16);
    sentai_fr_push_event("phase_axis_response_smoke", ev);
}

int centroid_z_hold_thrust_(const SentaiMarkersObservation& o,
                            float* z_prev,
                            float* vz_filt) {
    int thrust = s.centroid_hover_thrust_u16;
    float z_prev_out = *z_prev;
    float vz_out = *vz_filt;
    const int ok = sentai_calib_z_hold_thrust(
        o.z_cam_mean_m, *z_prev, *vz_filt,
        s.centroid_runtime_z_target_m, s.centroid_hover_thrust_u16,
        s.centroid_kp_thrust_per_m, s.centroid_kd_thrust_per_m_s,
        ((float)s.tick_ms) / 1000.0f, s.centroid_lpf_alpha,
        s.centroid_min_thrust_u16, s.centroid_max_thrust_u16,
        &thrust, &z_prev_out, &vz_out);
    if (ok) {
        *z_prev = z_prev_out;
        *vz_filt = vz_out;
    }
    return thrust;
}

void centroid_validation_() {
    sentai_calib_defaults_t d;
    sentai_calib_limits_t lim;
    (void)sentai_calib_get_defaults(&d);
    (void)sentai_calib_get_limits(&lim);
    sentai_fr_push_event("phase_centroid_pd_validation", "start");

    SentaiMarkersObservation initial;
    (void)sample_observation_(&initial);
    if (initial.n_full < s.centroid_hard_min_full_markers ||
            initial.z_cam_mean_m <= 0.0f) {
        s.st.centroid_abort_code = 2;
        s.st.centroid_thrust_last_u16 = s.centroid_hover_thrust_u16;
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
        s.st.done = 1;
        sentai_fr_push_event("phase_centroid_pd_validation",
                             "ok=0 reason=no_marker_lock");
        return;
    }

    float err_x = d.cx - initial.centroid_x;
    float err_y = d.cy - initial.centroid_y;
    float err = sqrtf(err_x * err_x + err_y * err_y);
    s.st.centroid_initial_err_px = err;
    s.st.centroid_min_err_px = err;
    s.st.centroid_max_err_px = err;
    s.st.centroid_n_full_min = initial.n_full;

    MarkerWindow win;
    marker_window_init_(&win, lim.marker_count_avg_window);
    marker_window_push_(&win, initial.n_full);
    float z_prev = initial.z_cam_mean_m;
    float vz_filt = 0.0f;
    int ticks = s.centroid_duration_ms / s.tick_ms;
    if (ticks < 1) ticks = 1;

    for (int k = 0; k < ticks; ++k) {
        if (stop_requested_()) break;

        SentaiMarkersObservation o;
        (void)sample_observation_(&o);
        marker_window_push_(&win, o.n_full);
        if (o.n_full < s.st.centroid_n_full_min) {
            s.st.centroid_n_full_min = o.n_full;
        }
        const int ready = marker_window_ready_(&win);
        const float avg_full = marker_window_avg_(&win);
        if (sentai_calib_marker_avg_unsafe(ready, avg_full, -1.0f) ||
                o.z_cam_mean_m <= 0.0f) {
            s.st.centroid_abort_code = 1;
            s.st.centroid_thrust_last_u16 = s.centroid_hover_thrust_u16;
            (void)send_rpyt_(0.0f, 0.0f, 0.0f,
                             s.st.centroid_thrust_last_u16);
            break;
        }

        float roll_cmd = 0.0f;
        float pitch_cmd = 0.0f;
        float target_delta[2] = {0.0f, 0.0f};
        float response_roll[2] = {0.0f, 0.0f};
        float response_pitch[2] = {0.0f, 0.0f};
        float z_gain = 1.0f;
        float dls_det = 0.0f;
        const int ctrl_ok = sentai_servo_ibvs_centroid_command(
            o.centroid_x, o.centroid_y, o.z_cam_mean_m, d.cx, d.cy,
            s.centroid_roll_vec, s.centroid_pitch_vec, s.centroid_gain,
            s.centroid_max_deg, s.centroid_deadband_px,
            s.centroid_damping_px_per_deg,
            s.centroid_sustained_response_sign, s.centroid_z_ref_m,
            s.centroid_z_gain_min, s.centroid_z_gain_max,
            &roll_cmd, &pitch_cmd, &err_x, &err_y, &err,
            target_delta, response_roll, response_pitch, &z_gain, &dls_det);
        if (!ctrl_ok) {
            roll_cmd = 0.0f;
            pitch_cmd = 0.0f;
            err_x = d.cx - o.centroid_x;
            err_y = d.cy - o.centroid_y;
            err = sqrtf(err_x * err_x + err_y * err_y);
        }

        s.st.centroid_thrust_last_u16 =
            centroid_z_hold_thrust_(o, &z_prev, &vz_filt);
        (void)send_rpyt_(roll_cmd, pitch_cmd, 0.0f,
                         s.st.centroid_thrust_last_u16);
        if (k == 0) {
            s.st.centroid_first_roll_deg = roll_cmd;
            s.st.centroid_first_pitch_deg = pitch_cmd;
            s.st.centroid_first_err_px = err;
        }
        s.st.centroid_last_roll_deg = roll_cmd;
        s.st.centroid_last_pitch_deg = pitch_cmd;
        s.st.centroid_last_err_px = err;
        s.st.centroid_ticks_done = k + 1;

        if (err < s.st.centroid_min_err_px) s.st.centroid_min_err_px = err;
        if (err > s.st.centroid_max_err_px) s.st.centroid_max_err_px = err;
        if ((k % 5) == 0 ||
                err > s.st.centroid_initial_err_px +
                      s.centroid_worse_max_px) {
            char ev[160];
            snprintf(ev, sizeof(ev),
                     "k=%d roll=%.3f pitch=%.3f err=%.2f n_full=%d thrust=%d",
                     k, (double)roll_cmd, (double)pitch_cmd, (double)err,
                     o.n_full, s.st.centroid_thrust_last_u16);
            sentai_fr_push_event("centroid_pd_tick", ev);
        }
        if (err > s.st.centroid_initial_err_px + s.centroid_worse_max_px) {
            s.st.centroid_abort_code = 3;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
    }

    (void)send_rpyt_(0.0f, 0.0f, 0.0f, s.st.centroid_thrust_last_u16);
    SentaiMarkersObservation last;
    (void)sample_observation_(&last);
    if (last.n_full < s.st.centroid_n_full_min) {
        s.st.centroid_n_full_min = last.n_full;
    }
    const float last_err_x = d.cx - last.centroid_x;
    const float last_err_y = d.cy - last.centroid_y;
    s.st.centroid_final_err_px =
        sqrtf(last_err_x * last_err_x + last_err_y * last_err_y);
    if (s.st.centroid_final_err_px < s.st.centroid_min_err_px) {
        s.st.centroid_min_err_px = s.st.centroid_final_err_px;
    }
    if (s.st.centroid_final_err_px > s.st.centroid_max_err_px) {
        s.st.centroid_max_err_px = s.st.centroid_final_err_px;
    }
    s.st.centroid_improvement_px =
        s.st.centroid_initial_err_px - s.st.centroid_final_err_px;
    s.st.centroid_avg_full_markers = marker_window_avg_(&win);
    s.st.centroid_marker_lock_ok = sentai_calib_marker_lock_ok(
        s.st.centroid_n_full_min, s.st.centroid_avg_full_markers) ? 1 : 0;
    s.st.centroid_ok =
        (s.st.centroid_abort_code == 0 &&
         s.st.centroid_marker_lock_ok &&
         (s.st.centroid_final_err_px <= s.centroid_tol_px ||
          s.st.centroid_improvement_px >= s.centroid_min_improve_px) &&
         s.st.centroid_max_err_px <=
             s.st.centroid_initial_err_px + s.centroid_worse_max_px) ? 1 : 0;

    if (stop_requested_()) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
    } else {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    }
    s.st.done = 1;

    char ev[160];
    snprintf(ev, sizeof(ev),
             "ok=%d abort=%d initial=%.2f final=%.2f improve=%.2f n_full_min=%d avg=%.2f thrust=%d",
             s.st.centroid_ok ? 1 : 0, s.st.centroid_abort_code,
             (double)s.st.centroid_initial_err_px,
             (double)s.st.centroid_final_err_px,
             (double)s.st.centroid_improvement_px,
             s.st.centroid_n_full_min,
             (double)s.st.centroid_avg_full_markers,
             s.st.centroid_thrust_last_u16);
    sentai_fr_push_event("phase_centroid_pd_validation", ev);
}

struct NoiseEstimate {
    int ok;
    int samples;
    int n_full_min;
    float avg_full;
    float sigma_px;
    float gate_px;
};

NoiseEstimate measure_noise_(const char* label,
                             float* z_prev,
                             float* vz_filt) {
    NoiseEstimate n;
    memset(&n, 0, sizeof(n));
    n.n_full_min = 99;
    n.sigma_px = 999.0f;
    n.gate_px = 999.0f;
    const int ticks = s.final_noise_samples < 2 ? 2 : s.final_noise_samples;
    float xs[32];
    float ys[32];
    int sample_count = 0;
    float n_full_sum = 0.0f;
    for (int k = 0; k < ticks; ++k) {
        SentaiMarkersObservation o;
        (void)sample_observation_(&o);
        const int thrust = axis_z_hold_thrust_(o, z_prev, vz_filt);
        s.st.final_val_thrust_last_u16 = thrust;
        (void)send_rpyt_(0.0f, 0.0f, 0.0f, thrust);
        if (o.n_full < n.n_full_min) n.n_full_min = o.n_full;
        n_full_sum += (float)o.n_full;
        if (o.n_full >= s.final_hard_min_full_markers &&
                sample_count < 32) {
            xs[sample_count] = o.centroid_x;
            ys[sample_count] = o.centroid_y;
            sample_count += 1;
        }
        if (k == 0 || k == ticks - 1) {
            char ev[160];
            snprintf(ev, sizeof(ev),
                     "label=%s k=%d n_full=%d cx=%.2f cy=%.2f thrust=%d vz=%.3f",
                     label, k, o.n_full, (double)o.centroid_x,
                     (double)o.centroid_y, thrust, (double)*vz_filt);
            sentai_fr_push_event("final_validation_noise_tick", ev);
        }
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
        if (stop_requested_()) break;
    }
    n.samples = sample_count;
    n.avg_full = n_full_sum / (float)ticks;
    if (sample_count < 2) {
        sentai_fr_push_event("final_validation_noise",
                             "ok=0 reason=insufficient_marker_lock_for_noise_estimate");
        return n;
    }

    const int last = sample_count - 1;
    const float drift_x_per_tick = (xs[last] - xs[0]) / (float)last;
    const float drift_y_per_tick = (ys[last] - ys[0]) / (float)last;
    float var = 0.0f;
    for (int i = 0; i < sample_count; ++i) {
        const float pred_x = xs[0] + drift_x_per_tick * (float)i;
        const float pred_y = ys[0] + drift_y_per_tick * (float)i;
        const float dx = xs[i] - pred_x;
        const float dy = ys[i] - pred_y;
        var += dx * dx + dy * dy;
    }
    var /= (float)(sample_count - 1);
    n.sigma_px = sqrtf(var);
    n.gate_px = s.final_noise_sigma_mult * n.sigma_px;
    if (n.gate_px < s.final_noise_floor_px) {
        n.gate_px = s.final_noise_floor_px;
    }
    n.ok = sentai_calib_marker_lock_ok(n.n_full_min, n.avg_full);

    char ev[160];
    snprintf(ev, sizeof(ev),
             "ok=%d samples=%d n_full_min=%d avg=%.2f sigma=%.3f gate=%.3f",
             n.ok, n.samples, n.n_full_min, (double)n.avg_full,
             (double)n.sigma_px, (double)n.gate_px);
    sentai_fr_push_event("final_validation_noise", ev);
    return n;
}

struct FinalAttempt {
    int ok;
    int consistent;
    int observable;
    float pulse_deg;
    int exp_axis;
    int exp_sign;
    int obs_axis;
    int obs_sign;
    float dominance;
    float strength;
    float noise_gate;
    float comp_dx;
    float comp_dy;
    float return_err;
    int min_full;
    float avg_full;
    int marker_lock_ok;
};

FinalAttempt final_validate_attempt_(const char* name,
                                     float pulse_deg,
                                     int row_idx,
                                     float noise_gate_px,
                                     float* z_prev,
                                     float* vz_filt) {
    FinalAttempt a;
    memset(&a, 0, sizeof(a));
    a.pulse_deg = pulse_deg;
    a.noise_gate = noise_gate_px;
    const int is_roll = strcmp(name, "roll") == 0;
    const float roll_cmd = is_roll ? pulse_deg : 0.0f;
    const float pitch_cmd = is_roll ? 0.0f : pulse_deg;

    SentaiMarkersObservation axis_base;
    (void)sample_observation_(&axis_base);
    if (axis_base.n_full < s.final_hard_min_full_markers) {
        a.min_full = axis_base.n_full;
        return a;
    }
    if (axis_base.z_cam_mean_m > 0.0f) {
        *z_prev = axis_base.z_cam_mean_m;
    }

    char label[40];
    snprintf(label, sizeof(label), "final_%s_pos", name);
    AxisSegment pos = stream_axis_segment_(
        label, roll_cmd, pitch_cmd, s.final_pulse_ms, z_prev, vz_filt);
    snprintf(label, sizeof(label), "final_%s_neg", name);
    AxisSegment neg = stream_axis_segment_(
        label, -roll_cmd, -pitch_cmd, s.final_pulse_ms, z_prev, vz_filt);
    snprintf(label, sizeof(label), "final_%s_settle", name);
    AxisSegment settle = stream_axis_segment_(
        label, 0.0f, 0.0f, s.final_settle_ms, z_prev, vz_filt);
    s.st.final_val_thrust_last_u16 = s.st.axis_thrust_last_u16;

    const float pos_seg_dx = pos.last.centroid_x - pos.first.centroid_x;
    const float pos_seg_dy = pos.last.centroid_y - pos.first.centroid_y;
    const float neg_seg_dx = neg.last.centroid_x - neg.first.centroid_x;
    const float neg_seg_dy = neg.last.centroid_y - neg.first.centroid_y;
    a.comp_dx = 0.5f * (pos_seg_dx - neg_seg_dx);
    a.comp_dy = 0.5f * (pos_seg_dy - neg_seg_dy);
    (void)sentai_calib_axis_observation_from_delta(
        a.comp_dx, a.comp_dy, &a.obs_axis, &a.obs_sign,
        &a.dominance, &a.strength);
    (void)sentai_calib_expected_from_row(
        s.final_R[row_idx * 3], s.final_R[row_idx * 3 + 1],
        s.final_R[row_idx * 3 + 2], &a.exp_axis, &a.exp_sign);

    const float return_dx = settle.last.centroid_x - axis_base.centroid_x;
    const float return_dy = settle.last.centroid_y - axis_base.centroid_y;
    a.return_err = sqrtf(return_dx * return_dx + return_dy * return_dy);
    a.min_full = pos.min_full;
    if (neg.min_full < a.min_full) a.min_full = neg.min_full;
    if (settle.min_full < a.min_full) a.min_full = settle.min_full;
    const int total_ticks = pos.ticks + neg.ticks + settle.ticks;
    a.avg_full = ((pos.avg_full * (float)pos.ticks +
                   neg.avg_full * (float)neg.ticks +
                   settle.avg_full * (float)settle.ticks) /
                  (float)total_ticks);
    a.marker_lock_ok = sentai_calib_marker_lock_ok(a.min_full, a.avg_full);
    a.consistent = (a.obs_axis == a.exp_axis &&
                    a.obs_sign == a.exp_sign &&
                    a.dominance >= s.final_dominance_ratio_min &&
                    a.marker_lock_ok &&
                    a.return_err <= s.final_return_max_px) ? 1 : 0;
    a.observable = (a.strength >= noise_gate_px) ? 1 : 0;
    a.ok = (a.consistent && a.observable) ? 1 : 0;
    return a;
}

void store_final_attempt_(int is_roll, const FinalAttempt& a, int attempts) {
    if (is_roll) {
        s.st.final_roll_ok = a.ok ? 1 : 0;
        s.st.final_roll_attempts = attempts;
        s.st.final_roll_pulse_deg = a.pulse_deg;
        s.st.final_roll_expected_axis_code = a.exp_axis;
        s.st.final_roll_expected_sign = a.exp_sign;
        s.st.final_roll_observed_axis_code = a.obs_axis;
        s.st.final_roll_observed_sign = a.obs_sign;
        s.st.final_roll_dominance_ratio = a.dominance;
        s.st.final_roll_response_strength_px = a.strength;
        s.st.final_roll_noise_gate_px = a.noise_gate;
        s.st.final_roll_comp_dx = a.comp_dx;
        s.st.final_roll_comp_dy = a.comp_dy;
        s.st.final_roll_return_err_px = a.return_err;
        s.st.final_roll_min_full_markers = a.min_full;
        s.st.final_roll_avg_full_markers = a.avg_full;
        s.st.final_roll_marker_lock_ok = a.marker_lock_ok ? 1 : 0;
        s.st.final_roll_consistent = a.consistent ? 1 : 0;
        s.st.final_roll_observable = a.observable ? 1 : 0;
    } else {
        s.st.final_pitch_ok = a.ok ? 1 : 0;
        s.st.final_pitch_attempts = attempts;
        s.st.final_pitch_pulse_deg = a.pulse_deg;
        s.st.final_pitch_expected_axis_code = a.exp_axis;
        s.st.final_pitch_expected_sign = a.exp_sign;
        s.st.final_pitch_observed_axis_code = a.obs_axis;
        s.st.final_pitch_observed_sign = a.obs_sign;
        s.st.final_pitch_dominance_ratio = a.dominance;
        s.st.final_pitch_response_strength_px = a.strength;
        s.st.final_pitch_noise_gate_px = a.noise_gate;
        s.st.final_pitch_comp_dx = a.comp_dx;
        s.st.final_pitch_comp_dy = a.comp_dy;
        s.st.final_pitch_return_err_px = a.return_err;
        s.st.final_pitch_min_full_markers = a.min_full;
        s.st.final_pitch_avg_full_markers = a.avg_full;
        s.st.final_pitch_marker_lock_ok = a.marker_lock_ok ? 1 : 0;
        s.st.final_pitch_consistent = a.consistent ? 1 : 0;
        s.st.final_pitch_observable = a.observable ? 1 : 0;
    }
}

void final_candidate_validation_() {
    sentai_fr_push_event("phase_final_candidate_validation", "start");
    int ok = 1;
    float z_prev = 0.0f;
    float vz_filt = 0.0f;
    const char* names[2] = {"pitch", "roll"};
    const int rows[2] = {0, 1};
    const float pulses[3] = {
        s.final_pulse_deg,
        s.final_retry_pulse_1_deg,
        s.final_retry_pulse_2_deg,
    };

    for (int axis_idx = 0; axis_idx < 2; ++axis_idx) {
        const int is_roll = axis_idx == 1;
        NoiseEstimate noise = measure_noise_(names[axis_idx], &z_prev, &vz_filt);
        if (!noise.ok) {
            s.st.final_val_abort_code = 1;
            ok = 0;
            break;
        }
        int axis_ok = 0;
        FinalAttempt prev;
        memset(&prev, 0, sizeof(prev));
        FinalAttempt last;
        memset(&last, 0, sizeof(last));
        int attempts = 0;
        for (int pi = 0; pi < 3; ++pi) {
            if (pulses[pi] > s.final_max_pulse_deg) continue;
            last = final_validate_attempt_(
                names[axis_idx], pulses[pi], rows[axis_idx],
                noise.gate_px, &z_prev, &vz_filt);
            attempts += 1;
            if (last.ok) {
                axis_ok = 1;
                break;
            }
            if (last.consistent && attempts >= 2) {
                const int repeat_consistent =
                    (prev.consistent &&
                     prev.obs_axis == last.obs_axis &&
                     prev.obs_sign == last.obs_sign &&
                     last.strength >= 1.10f * prev.strength &&
                     last.strength >= 0.75f * last.noise_gate);
                if (repeat_consistent) {
                    last.ok = 1;
                    last.observable = 1;
                    axis_ok = 1;
                    break;
                }
            }
            if (!last.consistent && last.observable) break;
            prev = last;
        }
        store_final_attempt_(is_roll, last, attempts);
        char ev[180];
        snprintf(ev, sizeof(ev),
                 "axis=%s ok=%d attempts=%d pulse=%.2f exp=%d/%d obs=%d/%d strength=%.2f gate=%.2f",
                 names[axis_idx], axis_ok, attempts, (double)last.pulse_deg,
                 last.exp_axis, last.exp_sign, last.obs_axis, last.obs_sign,
                 (double)last.strength, (double)last.noise_gate);
        sentai_fr_push_event("final_candidate_axis", ev);
        if (!axis_ok) {
            s.st.final_val_abort_code = 2;
            ok = 0;
            break;
        }
    }

    s.st.final_val_ok = ok ? 1 : 0;
    if (stop_requested_()) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
    } else {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    }
    s.st.done = 1;

    char ev[96];
    snprintf(ev, sizeof(ev), "ok=%d abort=%d thrust=%d",
             s.st.final_val_ok ? 1 : 0, s.st.final_val_abort_code,
             s.st.final_val_thrust_last_u16);
    sentai_fr_push_event("phase_final_candidate_validation", ev);
}

int run_ibvs_tick_(const SentaiMarkersObservation& o,
                   float* z_prev,
                   float* vz_filt,
                   float* roll_cmd,
                   float* pitch_cmd,
                   float* err_px) {
    float err_x = 0.0f;
    float err_y = 0.0f;
    float target_delta[2] = {0.0f, 0.0f};
    float response_roll[2] = {0.0f, 0.0f};
    float response_pitch[2] = {0.0f, 0.0f};
    float z_gain = 1.0f;
    float dls_det = 0.0f;
    sentai_calib_defaults_t d;
    (void)sentai_calib_get_defaults(&d);
    const int ctrl_ok = sentai_servo_ibvs_centroid_command(
        o.centroid_x, o.centroid_y, o.z_cam_mean_m, d.cx, d.cy,
        s.centroid_roll_vec, s.centroid_pitch_vec, s.centroid_gain,
        s.centroid_max_deg, s.centroid_deadband_px,
        s.centroid_damping_px_per_deg,
        s.centroid_sustained_response_sign, s.centroid_z_ref_m,
        s.centroid_z_gain_min, s.centroid_z_gain_max,
        roll_cmd, pitch_cmd, &err_x, &err_y, err_px,
        target_delta, response_roll, response_pitch, &z_gain, &dls_det);
    if (!ctrl_ok) {
        *roll_cmd = 0.0f;
        *pitch_cmd = 0.0f;
        err_x = d.cx - o.centroid_x;
        err_y = d.cy - o.centroid_y;
        *err_px = sqrtf(err_x * err_x + err_y * err_y);
    }
    const int thrust = centroid_z_hold_thrust_(o, z_prev, vz_filt);
    (void)send_rpyt_(*roll_cmd, *pitch_cmd, 0.0f, thrust);
    return thrust;
}

void final_recenter_() {
    sentai_calib_defaults_t d;
    sentai_calib_limits_t lim;
    (void)sentai_calib_get_defaults(&d);
    (void)sentai_calib_get_limits(&lim);
    sentai_fr_push_event("phase_final_centroid_recenter", "start");

    SentaiMarkersObservation initial;
    (void)sample_observation_(&initial);
    if (initial.n_full < s.recenter_hard_min_full_markers ||
            initial.z_cam_mean_m <= 0.0f) {
        s.st.recenter_abort_code = 2;
        s.st.recenter_thrust_last_u16 = s.centroid_hover_thrust_u16;
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
        s.st.done = 1;
        sentai_fr_push_event("phase_final_centroid_recenter",
                             "ok=0 reason=no_marker_lock");
        return;
    }

    float err_x = d.cx - initial.centroid_x;
    float err_y = d.cy - initial.centroid_y;
    float err = sqrtf(err_x * err_x + err_y * err_y);
    s.st.recenter_initial_err_px = err;
    s.st.recenter_min_err_px = err;
    s.st.recenter_max_err_px = err;
    s.st.recenter_n_full_min = initial.n_full;

    MarkerWindow win;
    marker_window_init_(&win, lim.marker_count_avg_window);
    marker_window_push_(&win, initial.n_full);
    float z_prev = initial.z_cam_mean_m;
    float vz_filt = 0.0f;
    int ticks = s.recenter_duration_ms / s.tick_ms;
    if (ticks < 1) ticks = 1;

    for (int k = 0; k < ticks; ++k) {
        if (stop_requested_()) break;
        SentaiMarkersObservation o;
        (void)sample_observation_(&o);
        marker_window_push_(&win, o.n_full);
        if (o.n_full < s.st.recenter_n_full_min) {
            s.st.recenter_n_full_min = o.n_full;
        }
        const int ready = marker_window_ready_(&win);
        const float avg_full = marker_window_avg_(&win);
        if (sentai_calib_marker_avg_unsafe(ready, avg_full, -1.0f) ||
                o.z_cam_mean_m <= 0.0f) {
            s.st.recenter_abort_code = 1;
            s.st.recenter_thrust_last_u16 = s.centroid_hover_thrust_u16;
            (void)send_rpyt_(0.0f, 0.0f, 0.0f,
                             s.st.recenter_thrust_last_u16);
            break;
        }

        float roll_cmd = 0.0f;
        float pitch_cmd = 0.0f;
        s.st.recenter_thrust_last_u16 = run_ibvs_tick_(
            o, &z_prev, &vz_filt, &roll_cmd, &pitch_cmd, &err);
        if (k == 0) {
            s.st.recenter_first_roll_deg = roll_cmd;
            s.st.recenter_first_pitch_deg = pitch_cmd;
            s.st.recenter_first_err_px = err;
        }
        s.st.recenter_last_roll_deg = roll_cmd;
        s.st.recenter_last_pitch_deg = pitch_cmd;
        s.st.recenter_last_err_px = err;
        s.st.recenter_ticks_done = k + 1;
        if (err < s.st.recenter_min_err_px) s.st.recenter_min_err_px = err;
        if (err > s.st.recenter_max_err_px) s.st.recenter_max_err_px = err;
        if ((k % 5) == 0 || err <= s.recenter_tol_px) {
            char ev[160];
            snprintf(ev, sizeof(ev),
                     "k=%d roll=%.3f pitch=%.3f err=%.2f n_full=%d thrust=%d",
                     k, (double)roll_cmd, (double)pitch_cmd, (double)err,
                     o.n_full, s.st.recenter_thrust_last_u16);
            sentai_fr_push_event("final_recenter_tick", ev);
        }
        if (err <= s.recenter_tol_px) break;
        if (err > s.st.recenter_initial_err_px + s.recenter_worse_max_px) {
            s.st.recenter_abort_code = 3;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
    }

    (void)send_rpyt_(0.0f, 0.0f, 0.0f, s.st.recenter_thrust_last_u16);
    SentaiMarkersObservation last;
    (void)sample_observation_(&last);
    if (last.n_full < s.st.recenter_n_full_min) {
        s.st.recenter_n_full_min = last.n_full;
    }
    if (last.n_full > 0) {
        err_x = d.cx - last.centroid_x;
        err_y = d.cy - last.centroid_y;
        s.st.recenter_final_err_px = sqrtf(err_x * err_x + err_y * err_y);
    } else {
        s.st.recenter_final_err_px = 999.0f;
    }
    if (s.st.recenter_final_err_px < s.st.recenter_min_err_px) {
        s.st.recenter_min_err_px = s.st.recenter_final_err_px;
    }
    if (s.st.recenter_final_err_px > s.st.recenter_max_err_px) {
        s.st.recenter_max_err_px = s.st.recenter_final_err_px;
    }
    s.st.recenter_improvement_px =
        s.st.recenter_initial_err_px - s.st.recenter_final_err_px;
    s.st.recenter_avg_full_markers = marker_window_avg_(&win);
    s.st.recenter_recenter_ok =
        (s.st.recenter_abort_code == 0 &&
         sentai_calib_marker_lock_ok(s.st.recenter_n_full_min,
                                     s.st.recenter_avg_full_markers) &&
         (s.st.recenter_final_err_px <= s.recenter_tol_px ||
          s.st.recenter_improvement_px >= s.recenter_min_improve_px) &&
         s.st.recenter_max_err_px <=
             s.st.recenter_initial_err_px + s.recenter_worse_max_px) ? 1 : 0;

    s.st.recenter_hover_err_min_px = s.st.recenter_final_err_px;
    s.st.recenter_hover_err_last_px = s.st.recenter_final_err_px;
    if (s.st.recenter_recenter_ok) {
        int hover_req = s.recenter_hover_ms / s.tick_ms;
        if (hover_req < 1) hover_req = 1;
        int hover_max = s.recenter_hover_max_ms / s.tick_ms;
        if (hover_max < hover_req) hover_max = hover_req;
        for (int hk = 0; hk < hover_max; ++hk) {
            if (stop_requested_()) break;
            SentaiMarkersObservation o;
            (void)sample_observation_(&o);
            marker_window_push_(&win, o.n_full);
            if (o.n_full < s.st.recenter_n_full_min) {
                s.st.recenter_n_full_min = o.n_full;
            }
            const int ready = marker_window_ready_(&win);
            const float avg_full = marker_window_avg_(&win);
            if (sentai_calib_marker_avg_unsafe(ready, avg_full, -1.0f) ||
                    o.z_cam_mean_m <= 0.0f) {
                s.st.recenter_abort_code = 4;
                s.st.recenter_thrust_last_u16 = s.centroid_hover_thrust_u16;
                (void)send_rpyt_(0.0f, 0.0f, 0.0f,
                                 s.st.recenter_thrust_last_u16);
                break;
            }
            float roll_cmd = 0.0f;
            float pitch_cmd = 0.0f;
            s.st.recenter_thrust_last_u16 = run_ibvs_tick_(
                o, &z_prev, &vz_filt, &roll_cmd, &pitch_cmd, &err);
            s.st.recenter_hover_err_last_px = err;
            if (err > s.st.recenter_hover_err_max_px) {
                s.st.recenter_hover_err_max_px = err;
            }
            if (err < s.st.recenter_hover_err_min_px) {
                s.st.recenter_hover_err_min_px = err;
            }
            s.st.recenter_hover_ticks = hk + 1;
            if ((hk % 5) == 0) {
                char ev[160];
                snprintf(ev, sizeof(ev),
                         "k=%d roll=%.3f pitch=%.3f err=%.2f stable=%d req=%d",
                         hk, (double)roll_cmd, (double)pitch_cmd,
                         (double)err, s.st.recenter_hover_stable_ticks,
                         hover_req);
                sentai_fr_push_event("final_center_hover_tick", ev);
            }
            if (err <= s.recenter_hover_tol_px) {
                s.st.recenter_hover_stable_ticks += 1;
                if (s.st.recenter_hover_stable_ticks >= hover_req) break;
            } else {
                s.st.recenter_hover_stable_ticks = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
        }
        (void)send_rpyt_(0.0f, 0.0f, 0.0f, s.st.recenter_thrust_last_u16);
        s.st.recenter_avg_full_markers = marker_window_avg_(&win);
        s.st.recenter_hover_ok =
            (s.st.recenter_abort_code == 0 &&
             s.st.recenter_hover_stable_ticks >= hover_req &&
             sentai_calib_marker_avg_lock_ok(
                 marker_window_ready_(&win), s.st.recenter_avg_full_markers,
                 s.recenter_avg_full_threshold)) ? 1 : 0;
        if (!s.st.recenter_hover_ok && s.st.recenter_abort_code == 0) {
            s.st.recenter_abort_code = 5;
        }
    }

    s.st.recenter_ok =
        (s.st.recenter_recenter_ok && s.st.recenter_hover_ok) ? 1 : 0;
    if (stop_requested_()) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
    } else {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    }
    s.st.done = 1;

    char ev[500];
    snprintf(ev, sizeof(ev),
             "ok=%d abort=%d recenter_ok=%d hover_ok=%d "
             "ticks=%d hover_ticks=%d hover_stable=%d "
             "n_full_min=%d avg=%.2f initial=%.2f final=%.2f "
             "min=%.2f max=%.2f improve=%.2f "
             "hover_err_max=%.2f hover_err_min=%.2f hover_err_last=%.2f "
             "first_roll=%.3f first_pitch=%.3f first_err=%.2f "
             "last_roll=%.3f last_pitch=%.3f last_err=%.2f thrust=%d",
             s.st.recenter_ok ? 1 : 0, s.st.recenter_abort_code,
             s.st.recenter_recenter_ok ? 1 : 0,
             s.st.recenter_hover_ok ? 1 : 0,
             s.st.recenter_ticks_done,
             s.st.recenter_hover_ticks,
             s.st.recenter_hover_stable_ticks,
             s.st.recenter_n_full_min,
             (double)s.st.recenter_avg_full_markers,
             (double)s.st.recenter_initial_err_px,
             (double)s.st.recenter_final_err_px,
             (double)s.st.recenter_min_err_px,
             (double)s.st.recenter_max_err_px,
             (double)s.st.recenter_improvement_px,
             (double)s.st.recenter_hover_err_max_px,
             (double)s.st.recenter_hover_err_min_px,
             (double)s.st.recenter_hover_err_last_px,
             (double)s.st.recenter_first_roll_deg,
             (double)s.st.recenter_first_pitch_deg,
             (double)s.st.recenter_first_err_px,
             (double)s.st.recenter_last_roll_deg,
             (double)s.st.recenter_last_pitch_deg,
             (double)s.st.recenter_last_err_px,
             s.st.recenter_thrust_last_u16);
    sentai_fr_push_event("phase_final_centroid_recenter", ev);
}

void manual_descend_disarm_() {
    sentai_fr_push_event("phase_manual_descend_disarm", "start");
    int ticks = s.manual_duration_ms / s.tick_ms;
    if (s.manual_duration_ms > 0 && ticks < 1) ticks = 1;
    s.st.manual_thrust_last_u16 = s.manual_start_thrust_u16;

    for (int k = 0; k < ticks; ++k) {
        if (stop_requested_()) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
            s.st.done = 1;
            return;
        }
        const float alpha = 1.0f - ((float)(k + 1) / (float)ticks);
        int thrust = (int)((float)s.manual_start_thrust_u16 * alpha);
        if (thrust < 0) thrust = 0;
        s.st.manual_thrust_last_u16 = thrust;
        s.st.manual_ticks_done = k + 1;
        (void)send_rpyt_(0.0f, 0.0f, 0.0f, thrust);
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
    }

    (void)stream_zero_for_ms_(s.manual_zero_ms);
    if (stop_requested_()) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
        s.st.done = 1;
        return;
    }

    s.st.manual_disarm_rc = sentai_crazy_disarm();
    s.st.manual_disarmed = (s.st.manual_disarm_rc == 0) ? 1 : 0;
    s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
    s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    s.st.done = 1;

    char ev[120];
    snprintf(ev, sizeof(ev), "ticks=%d thrust_last=%d disarm_rc=%d",
             s.st.manual_ticks_done, s.st.manual_thrust_last_u16,
             s.st.manual_disarm_rc);
    sentai_fr_push_event("phase_manual_descend_disarm", ev);
}

int vertical_rate_thrust_(const SentaiMarkersObservation& o,
                          float* z_prev,
                          float* vz_filt,
                          int base_thrust_u16,
                          float target_vz_m_s) {
    int thrust = s.center_thrust_floor_u16;
    float z_prev_out = *z_prev;
    float vz_filt_out = *vz_filt;
    const int ok = sentai_calib_vertical_rate_thrust(
        o.z_cam_mean_m, *z_prev, *vz_filt, base_thrust_u16,
        target_vz_m_s, ((float)s.tick_ms) / 1000.0f,
        s.centroid_lpf_alpha, s.center_kd_thrust_per_m_s,
        s.center_thrust_floor_u16, s.centroid_max_thrust_u16,
        &thrust, &z_prev_out, &vz_filt_out);
    if (!ok) {
        thrust = s.center_thrust_floor_u16;
    }
    *z_prev = z_prev_out;
    *vz_filt = vz_filt_out;
    return thrust;
}

void center_hold_descend_disarm_() {
    sentai_fr_push_event("phase_center_hold_descend_disarm", "start");
    float z_prev = 0.0f;
    float vz_filt = 0.0f;
    NoiseEstimate noise = measure_noise_(
        "center_hold_descend", &z_prev, &vz_filt);
    s.st.center_hold_noise_ok = noise.ok ? 1 : 0;
    s.st.center_hold_noise_sigma_px = noise.sigma_px;
    s.st.center_hold_deadband_px = s.center_deadband_floor_px;
    if (noise.ok) {
        const float measured =
            s.center_deadband_sigma_mult * noise.sigma_px;
        if (measured > s.st.center_hold_deadband_px) {
            s.st.center_hold_deadband_px = measured;
        }
    }
    s.centroid_deadband_px = s.st.center_hold_deadband_px;
    s.st.center_hold_target_vz_m_s = s.center_target_vz_m_s;
    s.st.center_hold_avg_window = s.center_avg_window;
    s.st.center_hold_n_full_min = 99;
    s.st.center_hold_z_cam_min_m = 999.0f;
    s.st.center_hold_thrust_last_u16 = s.center_start_thrust_u16;
    s.st.center_hold_disarm_rc = -999;

    MarkerWindow marker_win;
    marker_window_init_(&marker_win, s.center_avg_window);
    float err_win[64];
    int err_count = 0;
    int err_pos = 0;
    int lost_ticks = 0;
    const int ticks = s.center_duration_ms / s.tick_ms < 1 ?
        1 : s.center_duration_ms / s.tick_ms;

    for (int k = 0; k < ticks; ++k) {
        if (stop_requested_()) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_STOPPED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_STOPPED;
            s.st.done = 1;
            return;
        }

        SentaiMarkersObservation o;
        (void)sample_observation_(&o);
        marker_window_push_(&marker_win, o.n_full);
        const float avg_full = marker_window_avg_(&marker_win);
        const int avg_ready = marker_window_ready_(&marker_win);
        s.st.center_hold_avg_full_markers = avg_full;
        if (o.n_full < s.st.center_hold_n_full_min) {
            s.st.center_hold_n_full_min = o.n_full;
        }
        if (o.z_cam_mean_m > 0.0f) {
            s.st.center_hold_z_cam_last_m = o.z_cam_mean_m;
            if (o.z_cam_mean_m < s.st.center_hold_z_cam_min_m) {
                s.st.center_hold_z_cam_min_m = o.z_cam_mean_m;
            }
            if (o.z_cam_mean_m > s.st.center_hold_z_cam_max_m) {
                s.st.center_hold_z_cam_max_m = o.z_cam_mean_m;
            }
        }

        if (avg_ready && avg_full <= (float)s.center_disarm_full_markers) {
            s.st.center_hold_trigger_code = 1;
            break;
        }
        if (o.n_full <= 0) {
            lost_ticks += 1;
            if (lost_ticks >= s.center_lost_max_ticks) {
                s.st.center_hold_trigger_code = 2;
                break;
            }
        } else {
            lost_ticks = 0;
        }

        float roll_cmd = 0.0f;
        float pitch_cmd = 0.0f;
        float err_px = 0.0f;
        sentai_calib_defaults_t d;
        (void)sentai_calib_get_defaults(&d);
        float err_x = 0.0f;
        float err_y = 0.0f;
        float target_delta[2] = {0.0f, 0.0f};
        float response_roll[2] = {0.0f, 0.0f};
        float response_pitch[2] = {0.0f, 0.0f};
        float z_gain = 1.0f;
        float dls_det = 0.0f;
        (void)sentai_servo_ibvs_centroid_command(
            o.centroid_x, o.centroid_y, o.z_cam_mean_m, d.cx, d.cy,
            s.centroid_roll_vec, s.centroid_pitch_vec, s.centroid_gain,
            s.centroid_max_deg, s.centroid_deadband_px,
            s.centroid_damping_px_per_deg,
            s.centroid_sustained_response_sign, s.centroid_z_ref_m,
            s.centroid_z_gain_min, s.centroid_z_gain_max,
            &roll_cmd, &pitch_cmd, &err_x, &err_y, &err_px,
            target_delta, response_roll, response_pitch, &z_gain, &dls_det);
        if (err_px > s.st.center_hold_err_max_px) {
            s.st.center_hold_err_max_px = err_px;
        }
        s.st.center_hold_err_last_px = err_px;
        err_win[err_pos] = err_px;
        err_pos = (err_pos + 1) % s.center_avg_window;
        if (err_count < s.center_avg_window) err_count += 1;

        const int descend_thrust = vertical_rate_thrust_(
            o, &z_prev, &vz_filt, s.center_start_thrust_u16,
            s.center_target_vz_m_s);
        const int hold_thrust = vertical_rate_thrust_(
            o, &z_prev, &vz_filt, s.center_start_thrust_u16, 0.0f);
        const int roll_saturated =
            fabsf(fabsf(roll_cmd) - s.centroid_max_deg) < 0.0001f;
        const int pitch_saturated =
            fabsf(fabsf(pitch_cmd) - s.centroid_max_deg) < 0.0001f;
        float error_trend = 0.0f;
        const int trend_ready = err_count >= s.center_avg_window;
        if (trend_ready) {
            const int first_idx = err_pos;
            const int last_idx =
                (err_pos + s.center_avg_window - 1) % s.center_avg_window;
            error_trend = err_win[last_idx] - err_win[first_idx];
        }
        const int authority_limited =
            (roll_saturated || pitch_saturated) &&
            trend_ready && error_trend > 0.0f;
        const int descent_paused =
            avg_ready && avg_full > (float)s.center_disarm_full_markers &&
            authority_limited;
        int thrust = descent_paused ? hold_thrust : descend_thrust;
        if (descent_paused) {
            s.st.center_hold_descent_pause_ticks += 1;
        }
        s.st.center_hold_thrust_last_u16 = thrust;
        s.st.center_hold_vz_filt_last_m_s = vz_filt;
        s.st.center_hold_ticks_done = k + 1;
        (void)send_rpyt_(roll_cmd, pitch_cmd, 0.0f, thrust);

        if (k % 5 == 0) {
            char ev[200];
            snprintf(ev, sizeof(ev),
                     "k=%d thrust=%d paused=%d avg=%.2f err=%.2f vz=%.3f roll=%.2f pitch=%.2f",
                     k, thrust, descent_paused, (double)avg_full,
                     (double)err_px, (double)vz_filt, (double)roll_cmd,
                     (double)pitch_cmd);
            sentai_fr_push_event("center_hold_descend_tick", ev);
        }
        vTaskDelay(pdMS_TO_TICKS(s.tick_ms));
    }

    if (s.st.center_hold_trigger_code == 0) {
        s.st.center_hold_trigger_code = 3;
    }
    if (s.st.center_hold_z_cam_min_m == 999.0f) {
        s.st.center_hold_z_cam_min_m = 0.0f;
    }
    (void)stream_zero_for_ms_(s.center_zero_ms);
    s.st.center_hold_disarm_rc = sentai_crazy_disarm();
    s.st.center_hold_disarmed =
        (s.st.center_hold_disarm_rc == 0) ? 1 : 0;
    s.st.center_hold_ok =
        (s.st.center_hold_trigger_code == 1 &&
         s.st.center_hold_disarmed) ? 1 : 0;
    s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_DONE;
    s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_OK;
    s.st.done = 1;

    char ev[520];
    snprintf(ev, sizeof(ev),
             "ok=%d trigger=%d disarmed=%d disarm_rc=%d "
             "threshold=%d n_full_min=%d avg=%.2f avg_window=%d "
             "err_max=%.2f err_last=%.2f pause_ticks=%d ticks=%d "
             "z_min=%.3f z_max=%.3f z_last=%.3f vz=%.3f target_vz=%.3f "
             "thrust=%d deadband=%.2f noise_ok=%d noise_sigma=%.2f",
             s.st.center_hold_ok ? 1 : 0, s.st.center_hold_trigger_code,
             s.st.center_hold_disarmed ? 1 : 0,
             s.st.center_hold_disarm_rc,
             s.center_disarm_full_markers,
             s.st.center_hold_n_full_min,
             (double)s.st.center_hold_avg_full_markers,
             s.st.center_hold_avg_window,
             (double)s.st.center_hold_err_max_px,
             (double)s.st.center_hold_err_last_px,
             s.st.center_hold_descent_pause_ticks,
             s.st.center_hold_ticks_done,
             (double)s.st.center_hold_z_cam_min_m,
             (double)s.st.center_hold_z_cam_max_m,
             (double)s.st.center_hold_z_cam_last_m,
             (double)s.st.center_hold_vz_filt_last_m_s,
             (double)s.st.center_hold_target_vz_m_s,
             s.st.center_hold_thrust_last_u16,
             (double)s.st.center_hold_deadband_px,
             s.st.center_hold_noise_ok ? 1 : 0,
             (double)s.st.center_hold_noise_sigma_px);
    sentai_fr_push_event("phase_center_hold_descend_disarm", ev);
}

void worker_task_entry_(void* /*arg*/) {
    if (s.mode == MODE_CENTER_HOLD_DESCEND_DISARM) {
        center_hold_descend_disarm_();
    } else if (s.mode == MODE_MANUAL_DESCEND_DISARM) {
        manual_descend_disarm_();
    } else if (s.mode == MODE_FINAL_RECENTER) {
        final_recenter_();
    } else if (s.mode == MODE_FINAL_CANDIDATE_VALIDATION) {
        final_candidate_validation_();
    } else if (s.mode == MODE_CENTROID_VALIDATION) {
        centroid_validation_();
    } else if (s.mode == MODE_AXIS_RESPONSE_SMOKE) {
        axis_response_smoke_();
    } else if (s.mode == MODE_VISUAL_Z_HOLD) {
        visual_z_hold_();
    } else if (s.mode == MODE_POST_LOCK_BRAKE) {
        post_lock_brake_();
    } else if (s.mode == MODE_MARKER_ACQUISITION) {
        marker_acquisition_();
    } else if (s.mode == MODE_ARM_ZERO_UNLOCK) {
        arm_zero_unlock_();
    } else {
        sample_preflight_();
    }
    s.task_handle = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace

extern "C" int sentai_calib_orientation_task_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_PREFLIGHT_FEATURES,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_PREFLIGHT;
    s.st.samples = kPreflightSamples;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_visual_z_hold_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;

    float target_z_m = kZHoldTargetM * kZCalibAltitudeGain;
    if (s.last_acq_z_cam_max_m * kZCalibAltitudeGain > target_z_m) {
        target_z_m = s.last_acq_z_cam_max_m * kZCalibAltitudeGain;
    }
    if (s.last_post_z_cam_max_m * kZCalibAltitudeGain > target_z_m) {
        target_z_m = s.last_post_z_cam_max_m * kZCalibAltitudeGain;
    }
    if (target_z_m > kZHoldTargetMaxM) {
        target_z_m = kZHoldTargetMaxM;
    }

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_VISUAL_Z_HOLD,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_VISUAL_Z_HOLD;
    s.zhold_target_z_m = target_z_m;
    s.zhold_duration_ms = kZHoldDurationMs;
    s.zhold_target_tol_m = kZHoldTargetTolM;
    s.zhold_continue_if_target_seen = kZHoldContinueIfTargetSeen;
    s.zhold_hover_thrust_u16 = kHoverVisualThrustU16;
    s.zhold_min_thrust_u16 = kHoldMinThrustU16;
    s.zhold_max_thrust_u16 = kHoldMaxThrustU16;
    s.zhold_kp_thrust_per_m = kKpThrustPerM;
    s.zhold_kd_thrust_per_m_s = kKdThrustPerMS;
    s.zhold_lost_max_ticks = kZHoldLostMaxTicks;
    s.zhold_lpf_alpha = kVZLpfAlpha;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_axis_response_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;

    sentai_calib_limits_t lim;
    (void)sentai_calib_get_limits(&lim);
    const float runtime_z_target_m =
        s.zhold_target_z_m > 0.0f ? s.zhold_target_z_m : kZHoldTargetM;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_AXIS_RESPONSE_SMOKE,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_AXIS_RESPONSE_SMOKE;
    s.axis_pulse_deg = kAxisPulseDeg;
    s.axis_pulse_ms = kAxisPulseMs;
    s.axis_settle_ms = kAxisSettleMs;
    s.axis_max_axes = kAxisMaxAxes;
    s.axis_hard_min_full_markers = lim.axis_hard_min_full_markers;
    s.axis_min_avg_full_markers = lim.axis_min_avg_full_markers;
    s.axis_response_min_px = lim.response_min_px;
    s.axis_dominance_ratio_min = lim.axis_dominance_ratio_min;
    s.axis_orthogonal_dot_max_norm = lim.axis_orthogonal_dot_max_norm;
    s.axis_return_max_px = kAxisReturnMaxPx;
    s.axis_hover_thrust_u16 = kHoverVisualThrustU16;
    s.axis_min_thrust_u16 = kHoldMinThrustU16;
    s.axis_max_thrust_u16 = kHoldMaxThrustU16;
    s.axis_runtime_z_target_m = runtime_z_target_m;
    s.axis_kp_thrust_per_m = kKpThrustPerM;
    s.axis_kd_thrust_per_m_s = kKdThrustPerMS;
    s.axis_lpf_alpha = kVZLpfAlpha;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_centroid_validation_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;
    if (!s.last_axis_roll_valid || !s.last_axis_pitch_valid) return -3;

    sentai_calib_limits_t lim;
    (void)sentai_calib_get_limits(&lim);
    const float runtime_z_target_m =
        s.zhold_target_z_m > 0.0f ? s.zhold_target_z_m : kZHoldTargetM;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_CENTROID_VALIDATION,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_CENTROID_VALIDATION;
    s.centroid_duration_ms = kCentroidValidationDurationMs;
    s.centroid_roll_vec[0] = s.last_axis_roll_vec[0];
    s.centroid_roll_vec[1] = s.last_axis_roll_vec[1];
    s.centroid_pitch_vec[0] = s.last_axis_pitch_vec[0];
    s.centroid_pitch_vec[1] = s.last_axis_pitch_vec[1];
    s.centroid_gain = kCentroidValidationKp;
    s.centroid_max_deg = kCentroidValidationMaxDeg;
    s.centroid_deadband_px = kCentroidValidationDeadbandPx;
    s.centroid_tol_px = kCentroidValidationTolPx;
    s.centroid_min_improve_px = kCentroidValidationMinImprovePx;
    s.centroid_worse_max_px = kCentroidValidationWorseMaxPx;
    s.centroid_hard_min_full_markers = lim.axis_hard_min_full_markers;
    s.centroid_hover_thrust_u16 = kHoverVisualThrustU16;
    s.centroid_min_thrust_u16 = kHoldMinThrustU16;
    s.centroid_max_thrust_u16 = kHoldMaxThrustU16;
    s.centroid_runtime_z_target_m = runtime_z_target_m;
    s.centroid_kp_thrust_per_m = kKpThrustPerM;
    s.centroid_kd_thrust_per_m_s = kKdThrustPerMS;
    s.centroid_damping_px_per_deg = kIbvsDampingPxPerDeg;
    s.centroid_sustained_response_sign = kIbvsSustainedResponseSign;
    s.centroid_z_ref_m = kIbvsZRefM;
    s.centroid_z_gain_min = kIbvsZGainMin;
    s.centroid_z_gain_max = kIbvsZGainMax;
    s.centroid_lpf_alpha = kVZLpfAlpha;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_score_candidate_from_axis(void) {
    if (!s.last_axis_roll_valid || !s.last_axis_pitch_valid) {
        sentai_fr_push_event("phase_candidate_scoring",
                             "ok=0 reason=missing_axis_response");
        return 0;
    }

    sentai_calib_axis_candidate_t c;
    const char* roll_axis = axis_code_name_(s.last_axis_roll_axis_code);
    const char* pitch_axis = axis_code_name_(s.last_axis_pitch_axis_code);
    if (!sentai_calib_score_axis_candidate(
            roll_axis, s.last_axis_roll_sign,
            pitch_axis, s.last_axis_pitch_sign, &c)) {
        sentai_fr_push_event("phase_candidate_scoring",
                             "ok=0 reason=cpp_candidate_scoring_failed");
        return 0;
    }

    s.candidate_ok = c.ok ? 1 : 0;
    s.candidate_best_idx = c.best_idx;
    s.candidate_best_score = c.best_score;
    s.candidate_second_score = c.second_score;
    s.candidate_margin = c.margin;
    s.candidate_det = c.det;
    memcpy(s.final_R, c.R_cam_to_body, sizeof(s.final_R));
    s.final_R_valid = c.ok ? 1 : 0;

    char ev[240];
    snprintf(ev, sizeof(ev),
             "ok=%d best_idx=%d best_score=%.3f second_score=%.3f "
             "margin=%.3f det=%.3f roll_axis=%s roll_sign=%d "
             "pitch_axis=%s pitch_sign=%d",
             s.candidate_ok, s.candidate_best_idx,
             (double)s.candidate_best_score,
             (double)s.candidate_second_score,
             (double)s.candidate_margin,
             (double)s.candidate_det,
             roll_axis, s.last_axis_roll_sign,
             pitch_axis, s.last_axis_pitch_sign);
    sentai_fr_push_event("phase_candidate_scoring", ev);
    return s.candidate_ok;
}

extern "C" int sentai_calib_orientation_optical_axis_validate(void) {
    if (!s.final_R_valid) {
        sentai_fr_push_event("phase_optical_axis_validation",
                             "ok=0 reason=missing_candidate_R");
        return 0;
    }

    SentaiMarkersObservation o;
    (void)sample_observation_(&o);
    const int pose_valid = o.n_pose_valid;
    const float tz_mean = o.z_cam_mean_m;
    const int body_z_camera_z_sign = (s.final_R[8] >= 0.0f) ? 1 : -1;
    const int axis_ok =
        (body_z_camera_z_sign ==
         kOpticalAxisExpectedBodyZCameraZSign) ? 1 : 0;
    const int pose_ok =
        (pose_valid >= kOpticalAxisMinPoseValid &&
         tz_mean >= kOpticalAxisMinMeanTzM) ? 1 : 0;
    const int ok = (axis_ok && pose_ok) ? 1 : 0;

    char ev[220];
    snprintf(ev, sizeof(ev),
             "ok=%d axis_ok=%d pose_ok=%d body_z_camera_z_sign=%d "
             "expected_sign=%d pose_valid=%d pose_min=%d tz=%.3f tz_min=%.3f",
             ok, axis_ok, pose_ok, body_z_camera_z_sign,
             kOpticalAxisExpectedBodyZCameraZSign,
             pose_valid, kOpticalAxisMinPoseValid,
             (double)tz_mean, (double)kOpticalAxisMinMeanTzM);
    sentai_fr_push_event("phase_optical_axis_validation", ev);
    return ok;
}

extern "C" int sentai_calib_orientation_save_contract(const char* status,
                                                       int accepted) {
    if (!s.final_R_valid) return 0;
    if (!s.last_axis_roll_valid || !s.last_axis_pitch_valid) return 0;
    const float cam_off[3] = {-0.04f, 0.0f, -0.02f};
    if (sentai_calib_commit_R(s.final_R, cam_off) != 0) return 0;
    const int ok = sentai_calib_save_contract(
        status ? status : "", accepted,
        (float)s.last_axis_roll_sign,
        s.last_axis_roll_vec,
        (float)s.last_axis_pitch_sign,
        s.last_axis_pitch_vec);
    char ev[160];
    snprintf(ev, sizeof(ev),
             "status=%s accepted=%d writer=sentai.calib.save_contract ok=%d",
             status ? status : "", accepted ? 1 : 0, ok ? 1 : 0);
    sentai_fr_push_event("calib_contract_persist", ev);
    return ok ? 1 : 0;
}

extern "C" int sentai_calib_orientation_emergency_stop(void) {
    if (s.stop_evt != nullptr) {
        xEventGroupSetBits(s.stop_evt, STOP_BIT);
    }
    for (int i = 0; i < 10; ++i) {
        (void)send_rpyt_(0.0f, 0.0f, 0.0f, 0);
        vTaskDelay(pdMS_TO_TICKS(SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS));
    }
    return sentai_crazy_disarm();
}

extern "C" int sentai_calib_orientation_final_candidate_validation_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;
    if (!s.final_R_valid) return -3;

    sentai_calib_limits_t lim;
    (void)sentai_calib_get_limits(&lim);
    const float runtime_z_target_m =
        s.zhold_target_z_m > 0.0f ? s.zhold_target_z_m : kZHoldTargetM;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(
        SENTAI_CALIB_ORIENTATION_PHASE_FINAL_CANDIDATE_VALIDATION,
        SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_FINAL_CANDIDATE_VALIDATION;
    s.final_pulse_deg = kFinalValidatePulseDeg;
    s.final_retry_pulse_1_deg = kFinalValidateRetryPulse1Deg;
    s.final_retry_pulse_2_deg = kFinalValidateRetryPulse2Deg;
    s.final_pulse_ms = kFinalValidatePulseMs;
    s.final_settle_ms = kFinalValidateSettleMs;
    s.final_return_max_px = kFinalValidateReturnMaxPx;
    s.final_max_pulse_deg = kFinalValidateMaxPulseDeg;
    s.final_noise_samples = kFinalValidateNoiseSamples;
    s.final_noise_sigma_mult = kFinalValidateNoiseSigmaMult;
    s.final_noise_floor_px = kFinalValidateNoiseFloorPx;
    s.final_dominance_ratio_min = lim.axis_dominance_ratio_min;
    s.final_hard_min_full_markers = lim.axis_hard_min_full_markers;
    s.axis_hover_thrust_u16 = kHoverVisualThrustU16;
    s.axis_min_thrust_u16 = kHoldMinThrustU16;
    s.axis_max_thrust_u16 = kHoldMaxThrustU16;
    s.axis_runtime_z_target_m = runtime_z_target_m;
    s.axis_kp_thrust_per_m = kKpThrustPerM;
    s.axis_kd_thrust_per_m_s = kKdThrustPerMS;
    s.axis_lpf_alpha = kVZLpfAlpha;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_final_recenter_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;
    if (!s.last_axis_roll_valid || !s.last_axis_pitch_valid) return -3;

    sentai_calib_limits_t lim;
    (void)sentai_calib_get_limits(&lim);
    const float runtime_z_target_m =
        s.zhold_target_z_m > 0.0f ? s.zhold_target_z_m : kZHoldTargetM;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_FINAL_RECENTER,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_FINAL_RECENTER;
    s.recenter_duration_ms = kFinalRecenterDurationMs;
    s.recenter_hover_ms = kFinalRecenterHoverMs;
    s.recenter_hover_max_ms = kFinalRecenterHoverMaxMs;
    s.centroid_roll_vec[0] = s.last_axis_roll_vec[0];
    s.centroid_roll_vec[1] = s.last_axis_roll_vec[1];
    s.centroid_pitch_vec[0] = s.last_axis_pitch_vec[0];
    s.centroid_pitch_vec[1] = s.last_axis_pitch_vec[1];
    s.centroid_gain = kFinalRecenterKp;
    s.centroid_max_deg = kFinalRecenterMaxDeg;
    s.centroid_deadband_px = kFinalRecenterDeadbandPx;
    s.recenter_tol_px = kFinalRecenterTolPx;
    s.recenter_min_improve_px = kFinalRecenterMinImprovePx;
    s.recenter_worse_max_px = kFinalRecenterWorseMaxPx;
    s.recenter_hover_tol_px = kFinalRecenterHoverTolPx;
    s.recenter_hard_min_full_markers = lim.axis_hard_min_full_markers;
    s.recenter_avg_full_threshold = lim.axis_min_avg_full_markers;
    s.centroid_hover_thrust_u16 = kHoverVisualThrustU16;
    s.centroid_min_thrust_u16 = kHoldMinThrustU16;
    s.centroid_max_thrust_u16 = kHoldMaxThrustU16;
    s.centroid_runtime_z_target_m = runtime_z_target_m;
    s.centroid_kp_thrust_per_m = kKpThrustPerM;
    s.centroid_kd_thrust_per_m_s = kKdThrustPerMS;
    s.centroid_damping_px_per_deg = kIbvsDampingPxPerDeg;
    s.centroid_sustained_response_sign = kIbvsSustainedResponseSign;
    s.centroid_z_ref_m = kIbvsZRefM;
    s.centroid_z_gain_min = kIbvsZGainMin;
    s.centroid_z_gain_max = kIbvsZGainMax;
    s.centroid_lpf_alpha = kVZLpfAlpha;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_post_lock_brake_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_POST_LOCK_BRAKE,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_POST_LOCK_BRAKE;
    s.post_max_ms = kPostLockBrakeMaxMs;
    s.post_min_ticks = kPostLockBrakeMinTicks;
    s.post_brake_thrust_u16 = kPostLockBrakeThrustU16;
    s.post_settle_thrust_u16 = kPostLockSettleThrustU16;
    s.post_vz_ok_m_s = kPostLockVzOkMS;
    s.post_vz_ok_ticks = kPostLockVzOkTicks;
    s.post_lost_max_ticks = kZHoldLostMaxTicks;
    s.post_lpf_alpha = kVZLpfAlpha;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_marker_acquisition_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_MARKER_ACQUISITION,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_MARKER_ACQUISITION;
    s.max_ramp_ms = kAcqMaxRampMs;
    s.ramp_ms = kAcqRampMs;
    s.base_thrust_u16 = kAcqBaseThrustU16;
    s.max_thrust_u16 = kAcqMaxThrustU16;
    s.lock_brake_thrust_u16 = kAcqLockBrakeThrustU16;
    s.lock_consec_ticks = kAcqLockConsecTicks;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_arm_zero_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_ARM_ZERO_UNLOCK,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_ARM_ZERO_UNLOCK;
    s.pre_zero_ms = kArmPreZeroMs;
    s.zero_unlock_ms = kZeroUnlockMs;
    s.retry_zero_ms = kArmRetryZeroMs;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_manual_descend_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;
    int start_thrust_u16 = s.current_thrust_u16;
    if (start_thrust_u16 < 0) start_thrust_u16 = 0;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_MANUAL_DESCEND_DISARM,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_MANUAL_DESCEND_DISARM;
    s.manual_duration_ms = kManualDescendMs;
    s.manual_start_thrust_u16 = start_thrust_u16;
    s.manual_zero_ms = kManualZeroMs;
    s.st.manual_disarm_rc = -999;
    s.st.manual_thrust_last_u16 = start_thrust_u16;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_center_hold_descend_start(void) {
    if (s.task_handle != nullptr && !s.st.done) return 0;
    int start_thrust_u16 = s.current_thrust_u16;
    if (start_thrust_u16 < 0) start_thrust_u16 = 0;
    if (!s.last_axis_roll_valid || !s.last_axis_pitch_valid) return -3;

    sentai_calib_limits_t lim;
    (void)sentai_calib_get_limits(&lim);
    int avg_window = lim.marker_count_avg_window;
    if (avg_window < 1) avg_window = 1;
    if (avg_window > 64) avg_window = 64;
    const float runtime_z_target_m =
        s.zhold_target_z_m > 0.0f ? s.zhold_target_z_m : kZHoldTargetM;

    if (s.stop_evt == nullptr) {
        s.stop_evt = xEventGroupCreate();
        if (s.stop_evt == nullptr) {
            s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
            s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
            return -1;
        }
    }
    xEventGroupClearBits(s.stop_evt, STOP_BIT);
    reset_status_(SENTAI_CALIB_ORIENTATION_PHASE_CENTER_HOLD_DESCEND_DISARM,
                  SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS);
    s.mode = MODE_CENTER_HOLD_DESCEND_DISARM;
    s.center_duration_ms = kCenterHoldDurationMs;
    s.center_start_thrust_u16 = start_thrust_u16;
    s.center_disarm_full_markers = kCenterHoldDisarmFullMarkers;
    s.center_avg_window = avg_window;
    s.center_target_vz_m_s = kCenterHoldTargetVzMS;
    s.center_thrust_floor_u16 = kCenterHoldThrustFloorU16;
    s.center_kd_thrust_per_m_s = kCenterHoldKdThrustPerMS;
    s.center_lost_max_ticks = kCenterHoldLostMaxTicks;
    s.center_deadband_sigma_mult = kCenterHoldDeadbandSigmaMult;
    s.center_deadband_floor_px = kCenterHoldDeadbandFloorPx;
    s.center_zero_ms = kCenterHoldZeroMs;
    s.final_noise_samples = kFinalValidateNoiseSamples;
    s.final_noise_sigma_mult = kFinalValidateNoiseSigmaMult;
    s.final_noise_floor_px = kFinalValidateNoiseFloorPx;
    s.final_hard_min_full_markers = lim.axis_hard_min_full_markers;
    s.axis_hover_thrust_u16 = kHoverVisualThrustU16;
    s.axis_min_thrust_u16 = kHoldMinThrustU16;
    s.axis_max_thrust_u16 = kHoldMaxThrustU16;
    s.axis_runtime_z_target_m = runtime_z_target_m;
    s.axis_kp_thrust_per_m = kKpThrustPerM;
    s.axis_kd_thrust_per_m_s = kKdThrustPerMS;
    s.axis_lpf_alpha = kVZLpfAlpha;
    s.centroid_roll_vec[0] = s.last_axis_roll_vec[0];
    s.centroid_roll_vec[1] = s.last_axis_roll_vec[1];
    s.centroid_pitch_vec[0] = s.last_axis_pitch_vec[0];
    s.centroid_pitch_vec[1] = s.last_axis_pitch_vec[1];
    s.centroid_gain = kCenterHoldKp;
    s.centroid_max_deg = kCenterHoldMaxDeg;
    s.centroid_hover_thrust_u16 = kHoverVisualThrustU16;
    s.centroid_min_thrust_u16 = kHoldMinThrustU16;
    s.centroid_max_thrust_u16 = kHoldMaxThrustU16;
    s.centroid_runtime_z_target_m = runtime_z_target_m;
    s.centroid_kp_thrust_per_m = kKpThrustPerM;
    s.centroid_kd_thrust_per_m_s = kKdThrustPerMS;
    s.centroid_damping_px_per_deg = kIbvsDampingPxPerDeg;
    s.centroid_sustained_response_sign = kIbvsSustainedResponseSign;
    s.centroid_z_ref_m = kIbvsZRefM;
    s.centroid_z_gain_min = kIbvsZGainMin;
    s.centroid_z_gain_max = kIbvsZGainMax;
    s.centroid_lpf_alpha = kVZLpfAlpha;

    const BaseType_t ok = xTaskCreate(
        worker_task_entry_,
        "calib_orient",
        configMINIMAL_STACK_SIZE * 4,
        nullptr,
        tskIDLE_PRIORITY + 2,
        &s.task_handle);
    if (ok != pdPASS || s.task_handle == nullptr) {
        s.st.phase = SENTAI_CALIB_ORIENTATION_PHASE_FAULTED;
        s.st.status = SENTAI_CALIB_ORIENTATION_STATUS_FAULTED;
        s.st.done = 1;
        return -2;
    }
    return 0;
}

extern "C" int sentai_calib_orientation_task_stop(void) {
    if (s.stop_evt) {
        xEventGroupSetBits(s.stop_evt, STOP_BIT);
    }
    for (int i = 0; i < 20; ++i) {
        if (s.task_handle == nullptr || s.st.done) break;
        vTaskDelay(pdMS_TO_TICKS(SENTAI_CALIB_ORIENTATION_TASK_PERIOD_MS));
    }
    return 0;
}

extern "C" int sentai_calib_orientation_task_is_done(void) {
    return s.st.done ? 1 : 0;
}

extern "C" int sentai_calib_orientation_task_result(int* ok_out,
                                                    int* thrust_last_out,
                                                    int* disarmed_out) {
    if (!ok_out || !thrust_last_out || !disarmed_out) return 0;
    int ok = 0;
    int thrust = 0;
    int disarmed = 0;
    switch (s.mode) {
        case MODE_PREFLIGHT:
            ok = s.st.feature_lock ? 1 : 0;
            break;
        case MODE_ARM_ZERO_UNLOCK:
            ok = (s.st.arm_rc == 0) ? 1 : 0;
            break;
        case MODE_MARKER_ACQUISITION:
            ok = s.st.acq_locked ? 1 : 0;
            thrust = s.st.acq_last_thrust_u16;
            break;
        case MODE_POST_LOCK_BRAKE:
            ok = s.st.post_ok ? 1 : 0;
            thrust = s.st.post_thrust_last_u16;
            break;
        case MODE_VISUAL_Z_HOLD:
            ok = s.st.zhold_ok ? 1 : 0;
            thrust = s.st.zhold_thrust_last_u16;
            break;
        case MODE_AXIS_RESPONSE_SMOKE:
            ok = s.st.axis_ok ? 1 : 0;
            thrust = s.st.axis_thrust_last_u16;
            break;
        case MODE_CENTROID_VALIDATION:
            ok = s.st.centroid_ok ? 1 : 0;
            thrust = s.st.centroid_thrust_last_u16;
            break;
        case MODE_FINAL_CANDIDATE_VALIDATION:
            ok = s.st.final_val_ok ? 1 : 0;
            thrust = s.st.final_val_thrust_last_u16;
            break;
        case MODE_FINAL_RECENTER:
            ok = s.st.recenter_ok ? 1 : 0;
            thrust = s.st.recenter_thrust_last_u16;
            break;
        case MODE_MANUAL_DESCEND_DISARM:
            ok = s.st.manual_disarmed ? 1 : 0;
            thrust = s.st.manual_thrust_last_u16;
            disarmed = s.st.manual_disarmed ? 1 : 0;
            break;
        case MODE_CENTER_HOLD_DESCEND_DISARM:
            ok = s.st.center_hold_ok ? 1 : 0;
            thrust = s.st.center_hold_thrust_last_u16;
            disarmed = s.st.center_hold_disarmed ? 1 : 0;
            break;
        default:
            break;
    }
    *ok_out = ok;
    *thrust_last_out = thrust;
    *disarmed_out = disarmed;
    s.current_thrust_u16 = thrust;
    return 1;
}

extern "C" int sentai_calib_orientation_current_thrust(void) {
    return s.current_thrust_u16;
}

extern "C" float sentai_calib_orientation_runtime_z_target_m(void) {
    return s.zhold_target_z_m > 0.0f ? s.zhold_target_z_m : kZHoldTargetM;
}

extern "C" int sentai_calib_orientation_task_get_status(
        sentai_calib_orientation_status_t* out) {
    if (!out) return 0;
    *out = s.st;
    return 1;
}

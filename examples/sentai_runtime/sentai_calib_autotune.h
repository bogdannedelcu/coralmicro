// sentai_calib_autotune.h — internal-ish API for the Flow autotuner
// state machine (OP-S10-W14-T3).
//
// Pure compute — no FreeRTOS, no I/O.  The worker in
// sentai_calib_task.cc drives this each tick by pushing the latest
// drift sample and receiving back a desired body-frame velocity
// command.  This separation matches sentai.safety / sentai_safety_task:
// the state machine is testable standalone and the task is the
// only thing that touches the camera + cf2 telemetry.

#pragma once

#include <stdint.h>
#include "sentai_calib.h"   // sentai_calib_axis_t + AT_* enums + context

#ifdef __cplusplus
extern "C" {
#endif

// Tunables — exposed as constants for future override via -D.
#ifndef SENTAI_CALIB_AT_MIN_CYCLES
#define SENTAI_CALIB_AT_MIN_CYCLES        6     // half-periods detected
#endif
#ifndef SENTAI_CALIB_AT_MAX_CYCLES
#define SENTAI_CALIB_AT_MAX_CYCLES        24    // → DONE_FAIL if exceeded
#endif
#ifndef SENTAI_CALIB_AT_AMP_STABLE_TOL
// 2026-05-18 iter #8: relaxed from 0.20 → 0.40.  Real oscillation
// trace shows peaks varying ±50 % around the median (cf2's HL
// trajectory dynamics + PnP measurement noise + finite step relay).
// Tighter than ±20 % required > 30 s of clean data which exceeds
// our practical safety budget.  ZN with 40 % amp-stable still
// produces a useful Kp estimate (median-of-recent-peaks is the
// describing-function input — robust to outliers).
#define SENTAI_CALIB_AT_AMP_STABLE_TOL    0.40f
#endif
#ifndef SENTAI_CALIB_AT_AMP_STABLE_N
#define SENTAI_CALIB_AT_AMP_STABLE_N      4     // window size for stability
#endif
#ifndef SENTAI_CALIB_AT_DEAD_BAND_M
// 2026-05-18 iter #3 (post-marker-doubling): peak amplitude observed
// in s172 trial was ~12 mm, so a 5 mm dead band was eating half the
// cycle and stalling peak detection.  Drop to 3 mm (above PnP noise
// floor ≈ 1-2 mm, well below typical relay amplitude).
#define SENTAI_CALIB_AT_DEAD_BAND_M       0.003f
#endif

// Initialise the state machine.  Idempotent.  Clears history; does
// NOT change persisted Kp values.
void sentai_calib_autotune_init(void);

// Enter the AUTOTUNE_RELAY mode.  axis selects which body-axis
// velocity is driven; dur_s_max bounds total runtime; vmax_m_s is
// the relay magnitude (capped internally to half the smaller marker
// grid spacing × 0.5 / max_half_period, see implementation).
// Returns 0 on success, -1 on already-running, -2 on bad params.
int sentai_calib_autotune_arm(sentai_calib_axis_t axis,
                               float dur_s_max,
                               float vmax_m_s);

// Force a transition to ABORTED (idempotent).  Used by the worker
// when sentai.safety.aborted() trips during EXCITING.
void sentai_calib_autotune_abort(void);

// Single tick — call once per camera frame (≈ every 33 ms).  The
// worker passes:
//   drift_m   : (pnp world position - anchor position) along the
//               currently-tuned axis (m).  PnP frame; sign matters.
//   ts_ms     : monotonic timestamp from sentai_now_ms / clock_gettime.
//   pnp_valid : 1 if PnP this tick had >= 4 detections, 0 otherwise.
// Returns the relay velocity command for the active axis (m/s).
// When pnp_valid==0 the relay holds its last command (graceful).
// Inactive axis returns 0.
float sentai_calib_autotune_tick(float drift_m,
                                  uint32_t ts_ms,
                                  int pnp_valid);

// Read-side.
sentai_calib_autotune_state_t sentai_calib_autotune_get_state(void);

// On the latest DONE_OK transition the implementation populates
// the K_p / T_u / a_y results.  Stale across re-arms; check
// get_state() == DONE_OK before relying on these.
float sentai_calib_autotune_get_last_kp(void);     // ZN P-only
float sentai_calib_autotune_get_last_Tu_s(void);   // measured period
float sentai_calib_autotune_get_last_ay_m(void);   // measured amplitude
int   sentai_calib_autotune_get_last_cycles(void); // half-periods used

#ifdef __cplusplus
}
#endif

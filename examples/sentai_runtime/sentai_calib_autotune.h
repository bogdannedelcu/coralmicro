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
// 2026-05-18 iter #22: reduced 6 → 4.  With hysteresis 20 mm the
// half-period is ~3-4 s (vs ~1 s with tight dead-band), so over a
// 30 s autotune trial we get only 7-10 half-periods total.  Iter
// #21 trial produced 5 CLEAN peaks (+74, -77, +92, -89, +89 mm,
// ~10 % variance) but timed out before MIN_CYCLES=6 was reached.
// Lowering to 4 lets us latch onto the converged oscillation
// sooner; AMP_STABLE_N=4 stability check still requires 4 peaks
// to be in tolerance, so the convergence guarantee is preserved.
#define SENTAI_CALIB_AT_MIN_CYCLES        4
#endif
#ifndef SENTAI_CALIB_AT_MAX_CYCLES
#define SENTAI_CALIB_AT_MAX_CYCLES        24    // → DONE_FAIL if exceeded
#endif
#ifndef SENTAI_CALIB_AT_AMP_STABLE_TOL
// 2026-05-18 iter #19: relaxed 0.40 → 0.60 after empirical noise-OFF
// trials.  Without IMU noise, peaks were 17-50 mm (±50 % around
// mean 33 mm).  cf2 dynamics are inherently nonlinear (velocity
// loop saturation, lateral coupling) so even noise-free trials show
// >40 % amplitude variance.  ZN with 60 % tolerance still produces
// a usable Kp; the alternative is no convergence at all.
#define SENTAI_CALIB_AT_AMP_STABLE_TOL    0.60f
#endif
#ifndef SENTAI_CALIB_AT_AMP_STABLE_N
#define SENTAI_CALIB_AT_AMP_STABLE_N      4     // window size for stability
#endif
#ifndef SENTAI_CALIB_AT_DEAD_BAND_M
// 2026-05-18 iter #20: dead-band acts as relay HYSTERESIS (classical
// Åström-Hägglund extension).  3 mm dead-band caused resonance —
// relay flipped at zero-crossing, drone had no time to settle before
// the next command reversed direction → amplitude amplified each
// cycle (operator: "pare asa ca intr-o rezonanta care tot amplifica").
// 20 mm hysteresis forces the drone to ACTUALLY DISPLACE before the
// relay flips, breaking the resonance.  ZN math with hysteresis:
//   Ku = 4·vmax / (π·sqrt(a²−ε²))  where ε = hysteresis = dead_band.
// Approximation: when ε << a we get back to plain ZN.  Trial: ε=20mm
// expects a ≈ 40-60 mm → sqrt(a²−ε²) ≈ 35-55 mm vs a=40-60 mm = 15 %
// gain underestimate, acceptable for first-iter ID.
#define SENTAI_CALIB_AT_DEAD_BAND_M       0.020f
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

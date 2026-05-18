// sentai_calib_autotune.cc — OP-S10-W14-T3 state machine implementation.
//
// Pure compute.  See sentai_calib_autotune.h for the API contract.
// Algorithm: Åström-Hägglund relay autotune driving the velocity-
// feedback loop, Ziegler-Nichols P-only formula on the measured
// (T_u, a_y) pair.  References + math derivation in
// ideas/objects_plan/16_sentai_calib_autotune.md §2 / §5.

#include "sentai_calib_autotune.h"
#include "sentai_calib.h"
#include "sentai_fr.h"            // for sentai_fr_push_scalar (peak logging)

#include <math.h>
#include <string.h>
#include <stdint.h>

namespace {

// ── Peak ring ─────────────────────────────────────────────────────────
struct PeakSample {
    uint32_t ts_ms;
    float    drift_m;          // signed
};
static constexpr int PEAK_RING_CAP = 32;

struct State {
    // Configured at arm()
    sentai_calib_axis_t axis;
    float    vmax_m_s;
    float    dur_s_max;

    // Runtime
    sentai_calib_autotune_state_t state;
    float    last_v_cmd;
    float    prev_drift_m;
    uint32_t t_arm_ms;
    uint32_t t_last_tick_ms;
    int      cycle_count;       // half-periods detected
    PeakSample peaks[PEAK_RING_CAP];
    int      peak_head;          // next slot

    // Iter #21 hysteresis-safe extremum tracking (see push_peak_
    // call site in tick()): track running max-|drift| signed since
    // last sign flip, plus last NON-ZERO sgn (sticky across the
    // dead-band silent zone).
    int      last_active_sgn;    // +1 / -1 / 0 (only 0 at start)
    float    peak_extremum;       // signed max |drift| this half-cycle
    int      peak_in_progress;   // 0/1 — set on first non-zero tick

    // Live sign discovery (operator 2026-05-18 — "nu cumva sign-ul
    // poate fi detectat live").  In ARMING we apply +v_max for
    // SIGN_PROBE_MS and track the PEAK signed drift response (not
    // end-of-probe — cf2's velocity loop has overshoot, the
    // end-value can have any sign).  Sign of peak |drift| during
    // probe determines `sign_flip`:
    //   peak > 0  → "+v_max produced positive drift" → relay
    //               uses sign_flip = +1 (drive opposite of sign(drift))
    //   peak < 0  → "+v_max produced negative drift" → relay
    //               uses sign_flip = -1 (drive same as sign(drift))
    float    sign_drift_at_probe_start;
    float    sign_probe_peak_signed;   // max |drift| during probe, signed
    int      sign_flip;                 // +1 or -1 after probe
    uint32_t t_probe_started_ms;

    // DONE_OK results (valid only when state == DONE_OK)
    float    last_kp;
    float    last_Tu_s;
    float    last_ay_m;
    int      last_cycles;
};
static State s = {};

#ifndef SENTAI_CALIB_AT_SIGN_PROBE_MS
// 2026-05-18 iter #3: 1 s → 1.5 s.  cf2's velocity loop response
// has ~500 ms time constant in SIM; probe needed > 1 RC period to
// capture the steady-state direction not just the initial overshoot.
#define SENTAI_CALIB_AT_SIGN_PROBE_MS  1500
#endif
#ifndef SENTAI_CALIB_AT_SIGN_MIN_DRIFT_M
// PEAK |drift| must exceed this magnitude during probe for sign
// detection to be conclusive.  6 mm = 3 × dead-band (was 10 mm,
// but trial #2 saw peak −9.7 mm get rejected as "too close to
// threshold" while end-of-probe was +12 mm → wrong sign chosen).
#define SENTAI_CALIB_AT_SIGN_MIN_DRIFT_M 0.006f
#endif

inline int sign_of_(float x, float dead) {
    if (x >  dead) return +1;
    if (x < -dead) return -1;
    return 0;
}

inline void push_peak_(uint32_t ts_ms, float drift_m) {
    s.peaks[s.peak_head] = (PeakSample){ts_ms, drift_m};
    s.peak_head = (s.peak_head + 1) % PEAK_RING_CAP;
    if (s.cycle_count < PEAK_RING_CAP * 1000) s.cycle_count++;
}

// Returns true if the last N peaks have stable |amplitude| within tol.
// Also computes mean half-period (s) and mean |amplitude| (m) into out_*.
bool peaks_stable_(int N, float tol, float* out_Tu_s, float* out_ay_m) {
    if (s.cycle_count < N + 1) return false;
    int head = s.peak_head;
    // Walk back N+1 peaks (indices N..0 from most recent backwards).
    int idx[PEAK_RING_CAP];
    for (int i = 0; i < N + 1; ++i) {
        int j = (head - 1 - i + PEAK_RING_CAP) % PEAK_RING_CAP;
        idx[i] = j;
    }
    // Amplitudes (last N)
    float amps[PEAK_RING_CAP];
    float amp_mean = 0.0f;
    for (int i = 0; i < N; ++i) {
        amps[i] = fabsf(s.peaks[idx[i]].drift_m);
        amp_mean += amps[i];
    }
    amp_mean /= (float)N;
    if (amp_mean <= 1e-6f) return false;
    for (int i = 0; i < N; ++i) {
        float rel = fabsf(amps[i] - amp_mean) / amp_mean;
        if (rel > tol) return false;
    }
    // Half-period: mean inter-peak time over last N
    float dt_sum_s = 0.0f;
    for (int i = 0; i < N; ++i) {
        uint32_t t1 = s.peaks[idx[i]].ts_ms;
        uint32_t t0 = s.peaks[idx[i + 1]].ts_ms;
        if (t1 <= t0) return false;
        dt_sum_s += (float)(t1 - t0) * 1e-3f;
    }
    float half_period_s = dt_sum_s / (float)N;
    if (half_period_s <= 1e-3f) return false;
    *out_Tu_s = 2.0f * half_period_s;       // full period
    *out_ay_m = amp_mean;
    return true;
}

}  // namespace

extern "C" void sentai_calib_autotune_init(void) {
    memset(&s, 0, sizeof(s));
    s.state = SENTAI_CALIB_AT_IDLE;
}

extern "C" int sentai_calib_autotune_arm(sentai_calib_axis_t axis,
                                          float dur_s_max,
                                          float vmax_m_s) {
    if (s.state == SENTAI_CALIB_AT_ARMING ||
        s.state == SENTAI_CALIB_AT_EXCITING ||
        s.state == SENTAI_CALIB_AT_SETTLING) return -1;
    if (!(dur_s_max > 0.0f) || !(vmax_m_s > 0.0f && vmax_m_s < 1.0f))
        return -2;
    if (axis != SENTAI_CALIB_AXIS_X && axis != SENTAI_CALIB_AXIS_Y)
        return -2;
    memset(&s.peaks, 0, sizeof(s.peaks));
    s.peak_head      = 0;
    s.cycle_count    = 0;
    s.axis           = axis;
    s.dur_s_max      = dur_s_max;
    s.vmax_m_s       = vmax_m_s;
    s.last_v_cmd     = 0.0f;
    s.prev_drift_m   = 0.0f;
    s.t_arm_ms       = 0;
    s.t_last_tick_ms = 0;
    s.last_kp        = -1.0f;
    s.last_Tu_s      = 0.0f;
    s.last_ay_m      = 0.0f;
    s.last_cycles    = 0;
    s.sign_flip                 = +1;
    s.sign_drift_at_probe_start = 0.0f;
    s.sign_probe_peak_signed    = 0.0f;
    s.t_probe_started_ms        = 0;
    s.last_active_sgn           = 0;
    s.peak_extremum             = 0.0f;
    s.peak_in_progress          = 0;
    s.state          = SENTAI_CALIB_AT_ARMING;
    return 0;
}

extern "C" void sentai_calib_autotune_abort(void) {
    if (s.state == SENTAI_CALIB_AT_DONE_OK   ||
        s.state == SENTAI_CALIB_AT_DONE_FAIL ||
        s.state == SENTAI_CALIB_AT_ABORTED   ||
        s.state == SENTAI_CALIB_AT_IDLE) return;
    s.state      = SENTAI_CALIB_AT_ABORTED;
    s.last_v_cmd = 0.0f;
}

extern "C" float sentai_calib_autotune_tick(float drift_m,
                                              uint32_t ts_ms,
                                              int pnp_valid) {
    switch (s.state) {
    case SENTAI_CALIB_AT_IDLE:
    case SENTAI_CALIB_AT_DONE_OK:
    case SENTAI_CALIB_AT_DONE_FAIL:
    case SENTAI_CALIB_AT_ABORTED:
        return 0.0f;
    default: break;
    }

    if (s.t_arm_ms == 0) {
        s.t_arm_ms       = ts_ms;
        s.t_last_tick_ms = ts_ms;
    }

    // Deadline → DONE_FAIL.
    uint32_t elapsed_ms = ts_ms - s.t_arm_ms;
    if ((float)elapsed_ms * 1e-3f >= s.dur_s_max) {
        s.state      = SENTAI_CALIB_AT_DONE_FAIL;
        s.last_v_cmd = 0.0f;
        return 0.0f;
    }

    // If this tick has no valid PnP, HOLD last command (graceful).
    // We don't grow the streak / detect peaks — peak detection is
    // driven by drift derivative; missing samples are skipped.
    if (!pnp_valid) {
        return s.last_v_cmd;
    }

    // ── ARMING: sign-probe phase ─────────────────────────────────
    // Apply +v_max for SIGN_PROBE_MS and track the PEAK signed drift
    // excursion.  cf2's velocity loop has overshoot, so end-of-probe
    // value can be wrong-signed (trial #2: peak -9.7 mm but end
    // +12 mm — wrong sign chosen).  Using peak |drift| is robust
    // because the largest deviation reflects which way the loop is
    // actually being pushed, not where damping eventually settles.
    if (s.state == SENTAI_CALIB_AT_ARMING) {
        if (s.t_probe_started_ms == 0) {
            s.t_probe_started_ms        = ts_ms;
            s.sign_drift_at_probe_start = drift_m;
            s.sign_probe_peak_signed    = 0.0f;
            s.last_v_cmd                = +s.vmax_m_s;
            s.prev_drift_m              = drift_m;
            return s.last_v_cmd;
        }
        // Track peak |drift| signed (excursion from anchor).
        float excursion = drift_m - s.sign_drift_at_probe_start;
        if (fabsf(excursion) > fabsf(s.sign_probe_peak_signed)) {
            s.sign_probe_peak_signed = excursion;
        }
        uint32_t probe_elapsed = ts_ms - s.t_probe_started_ms;
        if (probe_elapsed < SENTAI_CALIB_AT_SIGN_PROBE_MS) {
            // Hold +v_max command throughout probe.
            s.last_v_cmd   = +s.vmax_m_s;
            s.prev_drift_m = drift_m;
            return s.last_v_cmd;
        }
        // Probe complete — analyse PEAK signed excursion.
        float peak = s.sign_probe_peak_signed;
        if (peak > +SENTAI_CALIB_AT_SIGN_MIN_DRIFT_M) {
            // +v_max peak in +drift direction → drift sign matches
            // our negative-feedback assumption → sign_flip = +1.
            s.sign_flip = +1;
        } else if (peak < -SENTAI_CALIB_AT_SIGN_MIN_DRIFT_M) {
            // +v_max peak in -drift direction → convention inverted.
            s.sign_flip = -1;
        } else {
            // Inconclusive (drone barely moved beyond noise).
            // Default sign_flip = +1; if wrong, drone drifts away
            // until safety latches abort — operator visible failure
            // mode, not silent error.
            s.sign_flip = +1;
        }
        // Transition.
        s.state        = SENTAI_CALIB_AT_EXCITING;
        s.prev_drift_m = drift_m;
        // Initial command from EXCITING fall-through below.
    }

    // EXCITING: relay = drive velocity AGAINST drift sign × sign_flip.
    int sgn = sign_of_(drift_m, SENTAI_CALIB_AT_DEAD_BAND_M);

    // ── Peak detection with HYSTERESIS-safe extremum tracking ────
    //    Iter #21 fix.  Previous (iter #20) code required the prev
    //    tick's sgn != 0 to detect a flip — broke with dead_band
    //    20 mm because drift slips through the dead-band silently
    //    between camera ticks, leaving prev_sgn=0 and missing the
    //    flip event.  Now we track:
    //      - last_active_sgn: most recent NON-ZERO sign (sticky)
    //      - peak_extremum:   running max-|drift| since last flip,
    //                         signed (preserves which side of zero)
    //    On a true sign flip (sgn != 0 && sgn != last_active_sgn),
    //    we record peak_extremum (the actual extremum we just
    //    passed, not just last tick) and reset.
    if (sgn != 0) {
        // Update extremum tracker while we're outside dead-band.
        if (s.peak_in_progress &&
            (fabsf(drift_m) > fabsf(s.peak_extremum))) {
            s.peak_extremum = drift_m;
        } else if (!s.peak_in_progress) {
            // First entry into a non-zero zone after a flip — start
            // tracking the new extremum from this sample.
            s.peak_extremum    = drift_m;
            s.peak_in_progress = 1;
        }
        // Sign flip detection — relative to last NON-ZERO sgn.
        if (s.last_active_sgn != 0 && sgn != s.last_active_sgn) {
            push_peak_(ts_ms, s.peak_extremum);
            sentai_fr_push_scalar("at_peak", s.peak_extremum, ts_ms);
            // Reset extremum tracker for the new half-cycle.
            s.peak_extremum    = drift_m;
        }
        s.last_active_sgn = sgn;
    }
    // (When sgn == 0, leave last_active_sgn and peak_extremum
    // untouched — the next non-zero tick continues the half-cycle.)

    // Command sign decision (no hysteresis — dead band already
    // guards against jitter).  sign_flip swaps direction if the
    // probe-phase analysis showed our convention was inverted.
    if (sgn > 0)      s.last_v_cmd = -s.vmax_m_s * (float)s.sign_flip;
    else if (sgn < 0) s.last_v_cmd = +s.vmax_m_s * (float)s.sign_flip;
    // else hold previous (in dead band)

    s.prev_drift_m   = drift_m;
    s.t_last_tick_ms = ts_ms;

    // Convergence check
    if (s.cycle_count >= SENTAI_CALIB_AT_MIN_CYCLES) {
        float Tu_s, ay_m;
        if (peaks_stable_(SENTAI_CALIB_AT_AMP_STABLE_N,
                           SENTAI_CALIB_AT_AMP_STABLE_TOL,
                           &Tu_s, &ay_m)) {
            // ZN P-only:  Ku = 4*v_max / (pi*a_y); Kp = 0.5 * Ku
            float Ku = (4.0f * s.vmax_m_s) / (3.14159265f * ay_m);
            float Kp = 0.5f * Ku;
            s.last_kp     = Kp;
            s.last_Tu_s   = Tu_s;
            s.last_ay_m   = ay_m;
            s.last_cycles = s.cycle_count;
            s.state       = SENTAI_CALIB_AT_DONE_OK;
            s.last_v_cmd  = 0.0f;
            return 0.0f;
        }
    }
    if (s.cycle_count >= SENTAI_CALIB_AT_MAX_CYCLES) {
        s.state      = SENTAI_CALIB_AT_DONE_FAIL;
        s.last_v_cmd = 0.0f;
        return 0.0f;
    }
    return s.last_v_cmd;
}

extern "C" sentai_calib_autotune_state_t sentai_calib_autotune_get_state(void) {
    return s.state;
}

extern "C" float sentai_calib_autotune_get_last_kp(void)    { return s.last_kp; }
extern "C" float sentai_calib_autotune_get_last_Tu_s(void)  { return s.last_Tu_s; }
extern "C" float sentai_calib_autotune_get_last_ay_m(void)  { return s.last_ay_m; }
extern "C" int   sentai_calib_autotune_get_last_cycles(void){ return s.last_cycles; }

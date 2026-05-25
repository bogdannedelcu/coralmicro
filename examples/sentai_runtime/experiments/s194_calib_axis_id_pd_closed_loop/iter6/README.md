# s194 iter6 — Simplified open-loop axis ID (no XY hold)

**Hypothesis**: iter5 XY hold introduced more instability than it
prevented because (1) PnP velocity jitter ±1.5 m/s caused saturated
oscillation, (2) naive identity mapping for 90° mount pushed drone in
wrong direction.

iter6 drops the complex closed-loop machinery entirely and uses the
same simple open-loop approach as s193 iter17 (which DID reach
`axis_id_R_committed` event), but with the iter3 PID Z stabilization
in background (which iter17 didn't have, hence iter17's downstream
failures).

## Algorithm

For each body axis (pitch then roll), 4 steps:

1. **Pulse** — `_stream(roll=0, pitch=+PULSE_DEG, dur=150ms)` — drone tilts
2. **Brake** — `_stream(roll=0, pitch=−PULSE_DEG, dur=100ms)` — partial cancel
3. **Settle** — `_stream(0, 0, dur=800ms)` — drone coasts, cf2 attitude PID levels
4. **Capture** — PnP median over 500 ms

Then:
- `dx_p = p_pitch − p0`, `dy_p = …`  → pitch displacement vector in pad
- `dx_r = p_roll − p0`, `dy_r = …`   → roll displacement vector
- Gram-Schmidt: `by_perp = roll_disp − (roll_disp · pitch_unit) × pitch_unit`
- `R_body_to_pad = column-stack(bx_pad, by_pad, body_z)`
- `sentai.calib.commit_R(R)`

Z PID continues in background via inner `_do_tick()` — drone stays at
target z = 0.30 m throughout pulses + captures.

## Change vs iter5

| Aspect | iter5 | iter6 |
|---|---|---|
| XY hold in Phase 3b/4 | Naive identity PD (unstable) | **REMOVED** |
| Phase 3b timeout | 5 s | **4 s** |
| Phase 4.5 axis ID | Closed-loop PD-park (complex, oscillated) | **Open-loop pulse+brake+settle+capture** |
| Discovery method | per-axis exploratory + PD-park ±δ | single pulse per axis (s193 iter17 style) |
| Drift cancellation | symmetric ±δ captures | **none — single-sided** |
| Total axis-ID duration | ~6-10 s (PD-park timeouts) | **~3.5 s** (fixed timing) |

## Success criteria (narrowed)

- `status == AXIS_ID_OK` in summary
- `axis_id_R_committed` event emitted, rc_commit == 0
- mag_p, mag_r both ≥ 1 cm (single-sided so smaller signal)
- `|theta_delta_deg|` in [60°, 120°] (single-sided is noisier, wider tol)
- |abs_dot| < 0.8 (axes not collinear)
- Drone lands safely (no crash, z descends smoothly to ground)
- Land XY ≤ 25 cm (accepting drift from no XY hold)

## Trade-off accepted

Single-sided capture means the displacement vector includes accumulated
drift (no symmetry cancellation). This is OK for ROUGH R discovery —
later experiments can refine via the C orchestrator's SAMPLE+KABSCH
phase which has its own drift cancellation via 4 cross poses.

## Date

2026-05-24

## Files

(Filled after run.)

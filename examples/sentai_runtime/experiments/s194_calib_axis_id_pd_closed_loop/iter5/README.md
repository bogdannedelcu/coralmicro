# s194 iter5 — A+B+C: XY hold + Z PID retune + fixed Z target

**Hypothesis**: iter4 failed because drone drifted -25 cm on pad-X during
the 17 s combined Phase 3 + 3b + 4 + Z-gate window with no horizontal
position control. Three coordinated fixes:

**Fix A — XY hold with naive identity R**
- Save `(x0, y0)` from first PnP at handoff
- In Phase 3b + Phase 4 loops, compute `err_x = x0 - px`, `err_y = y0 - py`
- Naive: `pitch_cmd = Kp·err_x − Kd·vx_filt`, `roll_cmd = Kp·err_y − Kd·vy_filt`
- For 90° mount rotation this maps wrong axis BUT:
  - `KP_XY = 2.0 °/m` (very small)
  - `KD_XY = 8.0 °/(m/s)` (heavy damping)
  - `ATT_XY_CMD_MAX = 1.5°` (hard saturation)
  - Wrong-direction command at 1.5° produces accel 0.26 m/s² — bounded
- Even if direction is wrong, damping bounds drift; axis ID (Phase 4.5)
  later discovers correct R for proper hold

**Fix B — Z PID retune**
- KP_THRUST: 10000 → **6000** (less aggressive, reduces overshoot)
- KD_THRUST: 8000 → **12000** (more damping)
- iter3+iter4 saw 25 cm Z oscillation with Kp=10k; softer gains should
  converge below 5 cm without oscillation

**Fix C — Fixed Z target in Phase 3b**
- iter1-iter4: `z_target = last_pnp[2]` (whatever z drone happens to be at)
- iter5: `z_target = RAMP_HANDOFF_Z_TARGET = 0.30 m` (fixed hover band)
- Phase 3b PID drives drone to canonical hover altitude regardless of
  ramp overshoot (iter4 saw ramp handoff at z=0.30 PnP / z=0.54 EKF — PID
  thought drone was at 0.30, ekf knew it was at 0.54, confused everything)

**Also** (carryover from iter3/iter4):
- Z PID with I term + anti-windup (Ki=5000, I_max=±2.0 m·s)
- Phase 3b stable check tightened: `|err_z|<5 cm AND |vz|<5 cm/s` for
  12 ticks
- Phase 3b timeout: 8 → **5 s** (less drift accumulation)
- Phase 3b streams `_extpos_force_z` (prevents EKF blowup)
- PD-park unchanged from iter4 (KP_ATT=4, KD_ATT=28, VXY_LPF=0.3)
- Orchestrator stops after axis ID (no phase5_handoff / phase6_bringup)

**Date**: 2026-05-24

## Success criteria

- `status == AXIS_ID_OK` in summary
- `phase3b_pid_hold stable=True` (no timeout)
- **XY drift in Phase 3b+4** ≤ 5 cm (was 6+10 = 16 cm in iter4 — half-life)
- `axis_id_R_committed` event emitted, rc_commit==0
- `|theta_delta_deg|` in [70°, 110°]
- `mag_p, mag_r ≥ 8 mm`
- GT Z stays ±5 cm during axis ID
- Land XY ≤ 15 cm

## Risks

1. **Naive XY hold drifts wrong direction for 90° mount** — saturation
   should bound, but if drone goes wrong way faster than estimated,
   could oscillate or drift further than no-hold. Mitigation: tiny gains,
   ±1.5° saturation, journal observability via `pitch_xy`/`roll_xy` in
   pid_tick events.
2. **Fixed Z target 0.30 m** — if ramp overshoots much past 0.30 m,
   Phase 3b PID has bigger initial error to fight. iter4 ramped to 0.54 m
   ekf, so 0.24 m error to recover. With Kd=12000 + Ki=5000 should converge
   in 3-5 s.

## Files

(Filled after run.)

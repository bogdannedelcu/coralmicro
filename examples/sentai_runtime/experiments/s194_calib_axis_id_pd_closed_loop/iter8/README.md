# s194 iter8 — T_HOVER recalibrated 36000 → 32500

**Hypothesis**: iter7 confirmed operator's observation "drona sta destul
de stabila pe X/Y dar pe Z coboara constant". Root cause traced to
`T_HOVER_NOMINAL=36000` being **4000 units above real cf2 SITL hover**
(~32500). With this bias:
- P term alone can never balance (err_z stays ~0.4 m to compensate)
- I term winds up to -0.6 m·s (or worse) just to maintain hover
- When drone transitions through target z (descending), I lags → drone
  keeps falling
- After Phase 3b, axis ID happens during continuous descent → pitch/roll
  pulse displacements are dominated by Z fall rather than body excitation
- Collinearity check rejects R (both pulse vectors point in same fall
  direction in pad XY)

**Fix**: recalibrate `T_HOVER_NOMINAL` to empirical real hover.

## Change vs iter7

| Param | iter7 | iter8 | Why |
|---|---|---|---|
| `T_HOVER_NOMINAL` | 36000 | **32500** | iter7 showed thrust=32400 → drone fell slowly → real hover < 32500 |
| `KP_THRUST_PER_M` | 6000 | **10000** | T_HOVER now correct, can be more aggressive without oscillating |
| `KD_THRUST_PER_M_PER_S` | 12000 | **10000** | Less over-damped now that bias is gone |
| `KI_THRUST_PER_M_S` | 5000 | **2000** | Need less I when P alone is near-correct |
| `I_INTEGRAL_MAX` | 2.0 | **0.5** | Limit windup (was wound to −0.6 in iter7) |

## Theoretical prediction

With T_HOVER=32500, Kp=10000, drone at err_z=−0.05 (drone 5cm above target):
- thrust = 32500 + 10000·(−0.05) + 0 − 0 = 32000
- At 32000 < 32500 → drone descends ~few cm/s → brings drone toward target

Steady-state at exactly target: thrust = 32500 = real hover ✓

If real hover is slightly different (32400 actual), then steady-state
err_z = (32500 − 32400)/10000 = 0.01 m = 1 cm (within tolerance).

## Algorithm

Phase 4.5 axis ID unchanged from iter7 — simple open-loop pulses:
1. Capture p0 median (0.5s)
2. +pitch 2°/150ms + brake −pitch 100ms + settle 800ms + capture p_pitch
3. +roll 2°/150ms + brake −roll 100ms + settle 800ms + capture p_roll
4. R from displacement vectors with Gram-Schmidt, commit_R

**Critical**: Z PID in background continues to hold target during ALL pulses.

## Success criteria

- Phase 3b PID converges (`stable=True`, not timeout)
- During axis ID, GT Z stays within ±5 cm of 0.30 m target
- mag_p ≥ 1 cm, mag_r ≥ 1 cm
- |abs_dot_unit_pre_gs| < 0.5 (signals decoupled, not collinear from Z fall)
- |theta_delta_deg| in [60°, 120°]
- R committed (rc_commit == 0)
- Land XY ≤ 25 cm

## Date

2026-05-24

## Files

(Filled after run.)

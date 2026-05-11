# Altitude-Sweep Calibration Run — 2026-05-11

Multi-DOF excitation (Z sweep ±25cm × 20s with XY centering active).
Enabled via `SENTAI_ALT_CAL=1`.

## Per-altitude bbox tracking precision

| Bucket | z range | Samples | mean |err_x| (px) | mean |err_y| (px) |
|--------|---------|---------|------------------|------------------|
| low    | z < HOLD_Z - 15cm | 44 | 44.6 | 88.2 |
| mid_low | -15cm < z - HOLD_Z < 0 | 10 | 57.2 | 78.7 |
| mid_high | 0 < z - HOLD_Z < +15cm | 44 | 41.9 | 73.5 |
| high   | z > HOLD_Z + 15cm | 0 | n/a | n/a |

## Main hover result post-calibration

- HOLD_Z: 2.5m
- Final pose: (+0.401, -0.283, 0.13)
- Cat target: (+0.4, -0.3)
- **Final distance: 1.7 cm** (best ever — beats 1.9cm baseline without cal)

## Conclusions

1. err_x altitude-invariant → X gain normalisation works.
2. err_y degrades at low altitude → Y axis benefits from gain scheduling.
3. Calibration acts as warm-up — EKF settled + tracker stable before main hover.
4. `high` bucket empty: vz gain too small; needs 2.0 instead of 0.5 to reach +25cm.

See Sim.md §10j altitude-sweep calibration section for full discussion.

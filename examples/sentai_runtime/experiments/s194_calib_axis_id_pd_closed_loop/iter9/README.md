# s194 iter9 — relax PnP gates + lower T_MAX

**Hypothesis**: iter8 fixed Z stability (T_HOVER recalibrated). Two
secondary blockers remain:
1. **Ramp overshoots**: T_MAX=37000 vs T_HOVER=32500 = 4500 margin → ramp
   pushes drone to 0.45 m before PID brakes. Burns 3 s of timeout budget.
2. **PnP intermittent**: at z=0.30 m, PnP momentarily drops to n=3
   markers. Both Phase 3b stable check (`n>=4`) and `_capture_median`
   (`min 3 samples`) reset.

## Fixes

| Param | iter8 | iter9 | Why |
|---|---|---|---|
| `T_MAX_U16` | 37000 | **34000** | Margin to T_HOVER=32500 reduced to 1500 → less ramp overshoot |
| Phase 3b stable check | `err+vz+n>=4+pz>0` | **`err+vz` only** | Drone physically stable counts; tolerates short PnP drops |
| `_capture_median` min samples | 3 | **2** | More forgiving at low z where PnP is jittery |
| `PHASE3B_TIMEOUT_S` | 4 | **6** | Safety margin (rarely needed now that T_HOVER correct) |

## Risk of dropped n>=4 gate in stable check

If PnP dies for many seconds, `z_now = z_prev` (stale) → vz_filt → 0 →
err_z stays at last value → stable check could fire on stale data.

Mitigated by:
- PnP at z=0.30 m typically has n=4-7 (only momentary n=3 drops)
- Phase 3b only 6 s total, can't drift far on stale data
- N_STABLE_REQ=12 ticks (400 ms) — drone has to be stable for nearly
  half a second consecutively

## Success criteria

- **Phase 3b stable=True** (not timeout) — finally!
- GT Z within ±5 cm of 0.30 m during axis ID (iter8 already at ±1mm)
- mag_p, mag_r ≥ 1 cm — pulses produce detectable XY signal
- |abs_dot| < 0.5 — pulses are decoupled (not collinear from Z fall)
- |theta_delta| in [60°, 120°] — orthogonal-ish axes
- R committed (rc_commit == 0)
- Land XY ≤ 15 cm

## Date

2026-05-24

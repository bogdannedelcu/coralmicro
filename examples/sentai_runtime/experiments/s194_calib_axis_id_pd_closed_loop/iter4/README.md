# s194 iter4 — PD-park velocity LPF + relaxed Z tol + ExtPos in Phase 3b

**Hypothesis**: iter3 made real progress on Z stability (Δz=4.9 cm
during axis ID, vs 25 cm iter2) but failed downstream:
- Phase 3b PID timed out at err_z=−4.93 cm (just outside 3 cm tol)
- PD-park oscillated wildly (50 cm range vs 6 cm target) due to noisy
  PnP-derived velocity feedback
- cf2 EKF blew up (ekf_z → −6.25 m) because Phase 3b didn't stream
  ExtPos to anchor it

**Three fixes**:

1. **Phase 3b tol relaxed 3 cm → 5 cm** — PID converges to ~5 cm steady-
   state with current gains; matches Z-gate tol which DID pass in iter3
2. **Stream ExtPos in Phase 3b** — `_extpos_force_z(pz)` per tick so cf2
   Kalman tracks real z instead of integrating IMU into a -6 m hallucination
3. **PD-park gentle gains + velocity LPF**:
   - `KP_ATT_PER_M`: 6 → **4** (more conservative)
   - `KD_ATT_PER_M_S`: 18 → **28** (more damping)
   - `VXY_LPF_ALPHA = 0.3` NEW: low-pass filter on `vel_along` (~92 ms
     time constant), smooths PnP-jitter (33 ms per-sample, ±5 cm jitter
     → spurious 1.5 m/s velocity spikes that drove PD wild in iter3)
   - `PARK_TIMEOUT_S`: 4 → **6** s (gentler PD needs more time)
   - Added `axis_id_park_tick` event every 8 ticks (~265 ms) for visibility

**Change vs iter3**:

| Param | iter3 | iter4 |
|---|---|---|
| Z_STABLE_TOL_M | 0.03 | **0.05** |
| Phase 3b ExtPos | not sent | **sent every tick** |
| KP_ATT_PER_M | 6 | **4** |
| KD_ATT_PER_M_S | 18 | **28** |
| VXY_LPF_ALPHA | n/a (raw vel) | **0.3** (filtered) |
| PARK_TIMEOUT_S | 4 | **6** |

**Date**: 2026-05-24

## Success criteria

Same as iter3 README, plus:
- `phase3b_pid_hold stable=True` (not timeout)
- During Phase 3b, `pid_tick` events show `ekf_z` stays within ±5 cm of
  `z_now` (PnP-derived) — ExtPos anchoring works
- PD-park `vel_along_filt` magnitude < 0.5 m/s peak (not the 1+ m/s
  jitter from iter3 raw vel)
- `axis_id_park_done` events emitted for all 4 legs (pitch+, pitch-,
  roll+, roll-)

## Files

(Filled after run.)

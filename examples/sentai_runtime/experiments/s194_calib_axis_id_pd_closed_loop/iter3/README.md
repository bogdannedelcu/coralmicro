# s194 iter3 — Z PID + tighter stable + stop-after-axis-ID

**Hypothesis**: Three coordinated fixes resolve the iter1+iter2 failure
mode (drone climbing during axis ID due to Z PD steady-state bias):

1. **Z PD → PID**: add I term (Ki=5000, anti-windup at ±2.0 m·s) to
   eliminate steady-state Z error caused by T_HOVER_NOMINAL bias.
2. **Phase 3b stable check tightens**: `|err_z| < 3 cm` AND `|vz| < 3 cm/s`
   for 15 ticks (≈ 500 ms), timeout 8 s.  Was velocity-only with 4 ticks
   in iter1/iter2 (inherited from s193 iter-20).
3. **Stop after axis ID** (operator request 2026-05-24): skip
   `_phase5_handoff_to_hl` and `_phase6_bringup` entirely; go directly
   to `_phase7_land` after axis ID.  Success criterion = axes identified
   correctly + drone lands safely near origin.

**Change vs iter2**:

| Aspect | iter2 | iter3 |
|---|---|---|
| Z controller | P+D (steady-state bias 30+ cm) | **PID** (zero steady-state) |
| Phase 3b stable check | `vz<10cm/s` for 4 ticks (velocity-only) | `err<3cm AND vz<3cm/s` for 15 ticks |
| Phase 3b timeout | 2 s (inherited SAMPLE_MAX_S) | **8 s** (PID needs time to integrate) |
| Z gate (axis ID) | 4cm/3cm/s tol, 10 ticks (unchanged) | 4cm/3cm/s tol, 10 ticks (same) |
| Pulse magnitude | 2°/120ms, δ=3cm (unchanged) | 2°/120ms, δ=3cm (same) |
| Post-axis-ID flow | phase5_handoff → phase6_bringup → land | **direct phase7_land** (no handoff) |
| Status enum | PASS/FAIL/etc | AXIS_ID_OK / AXIS_ID_FAIL |

**Why operator requested stop-after-axis-ID**:

Per the scope spec (`OP-S10-W21_calib_unified_bringup.md`):
> "fara sa prabusim drona, incet incet sa ii descoperim caracteristicile"

iter1+iter2 both tumbled in phase5_handoff after axis ID failed.  Per
the staged identification principle ("stabilizarea critica initial pe Z
apoi pe masura ce descoperim axele pe fiecare din axe"), we should
NOT proceed to phase5 until Z + per-axis identification is rock-solid.

Once axis ID consistently passes + drone lands cleanly, iter4+ can
re-enable phase5_handoff with confidence.

**Date**: 2026-05-24

## Success criteria (iter3-specific, narrower than full bringup)

- `status == "AXIS_ID_OK"` in summary.json
- `axis_id_R_committed` event emitted (`rc_commit == 0`)
- `theta_delta_deg` in [70°, 110°] (90° apart ± noise)
- `|abs_dot_unit_pre_gs|` < 0.5 (axes not collinear)
- `mag_p` AND `mag_r` ≥ 8 mm (signal strong enough to be reliable)
- Phase 3b PID converges (no timeout) — `phase3b_pid_hold` event has
  `stable: True`
- During axis ID, drone Z (from GT) stays within ±10 cm of stable
  value (was +25 cm climb during iter2)
- Land XY ≤ 15 cm from takeoff origin (drone goes home safely)

## Files captured in this folder

- `mission_s194_journal.txt`, `mission_s194_summary.json`
- `cf2_gt.jsonl` (host GT recorder, post-mortem only)
- `gz_to_uds_bridge.log`, `launch_hybrid.log`, `launch_sim.pids`
- `fr_current/scalars.csv` — includes new `i_integral`, `i_term` scalars
- `sentai_repl.log`, `verdict.log`

## Result

(Filled after running.)

# s194 iter2 — smaller pulses + Z-stability gate

**Hypothesis**: Two orthogonal fixes resolve iter1's `fov_loss` cascade:
- (A, operator-requested) Movements were too aggressive — reduce
  exploratory pulse + PD-park amplitudes + attitude saturation
- (B, GT-confirmed root cause) Z was out-of-control before axis ID
  started — drone climbing at 0.13 m/s while PD reported `stable=True`.
  Add a hard Z-stability pre-gate before any axis-ID excitation.

**Change vs iter1**:

A — smaller movements:
- `EXPLORE_DEG`: 4° → 2° (peak v ≈ 4 cm/s vs 25 cm/s)
- `EXPLORE_DUR_S`: 0.25 → 0.12 s
- `DISP_M`: 6 cm → 3 cm (PD target half-amplitude)
- `KP_ATT_PER_M`: 10 → 6 (gentler)
- `KD_ATT_PER_M_S`: 14 → 18 (more damping)
- `ATT_CMD_MAX_DEG`: 7° → 3° (hard saturation)
- `PARK_TOL_M`: 12 → 10 mm; `PARK_VEL_TOL`: 5 → 4 cm/s
- `MIN_DISP_M`: 15 → 8 mm

B — Z-stability gate:
- New `_z_stability_gate()` runs at start of each `_id_axis()` call
- Requires `|PnP_z − z_target| < 4 cm` AND `|vz_filt| < 3 cm/s` for
  10 consecutive ticks (≈ 333 ms)
- Timeout 4 s; abort axis ID with `FAIL_z_not_stable` if not reached

**Date**: 2026-05-24

## iter1 GT evidence motivating fix B

| event | GT z | PnP-claimed z | Δ vs PD target (0.235m) |
|---|---|---|---|
| handoff | 0.284 m | 0.235 | +5 cm |
| PD "stable=True" | 0.539 m | 0.509 | **+30 cm — PD lied** |
| axis_id start (p0) | 0.669 m | 0.617 | +43 cm |
| explore pulse done | 0.810 m | 0.770 | +57 cm, X drifted −7.5 cm |
| PD-park fov_loss | 0.921 m | n/a | +69 cm, X drift −43 cm |

The drone was effectively flying upward at 0.13 m/s during the entire
axis-ID phase.  Any lateral excitation amplifies because PD-park
cannot brake against the existing momentum.

## Files captured in this folder

- `mission_s194_journal.txt` — mission FSM events + per-tick PnP/EKF
- `mission_s194_summary.json` — phase_reached + R + axis_id_ok + status
- `cf2_gt.jsonl` — host-side GT recorder (post-mortem only)
- `gz_to_uds_bridge.log`, `launch_hybrid.log`, `launch_sim.pids` — runtime
- `fr_current/scalars.csv`, `fr_current/events.csv` — FR per-tick logs

## Result

(Filled after running iter2.)

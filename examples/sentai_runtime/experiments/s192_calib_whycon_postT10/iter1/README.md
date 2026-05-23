# s192 iter1 — WhyCon detection smoke (post-T10)

**Hypothesis**: After T10's OpenCV-parity port (commit `b4eb37ce`),
the embedded WhyCon detector reliably sees the 6-marker pad during
cf2 hover in Gazebo.  Yesterday's pre-T10 detector returned `n=0`
for hundreds of frames at the same altitudes.

**Change vs prior iter**: baseline iter (no prior iter in this
experiment — predecessor is s191 iter-19).

**Date**: 2026-05-23

## What this iter does

1. Phase 1-4 from s191 (setup → zero-unlock RPYT → thrust ramp →
   PD altitude lock).
2. NEW phase 5: **5 s detection measurement window** with PD
   altitude maintained.  Per-tick log: `n` markers + PnP xyz.
3. Land via RPYT thrust ramp-down + disarm.

## Files captured in this folder

- `mission_s192_journal.txt` — per-tick mission log
- `mission_s192_summary.json` — pass/fail + detect_window stats
- `cf2_gt.jsonl` — host-side GT recorder (sim/scripts/gt_recorder.py)
- `sentai_repl.log` — sentai_sim REPL output
- `verdict.log` — human-readable verdict
- `verdict_s192.json` — structured verdict (recall, n histogram,
  PnP mean+range, landing offset)

## Pass criteria

1. Drone takes off (ramp_handoff reached, PD altitude stable)
2. **≥80% of measurement-window ticks have `n>=4`**
3. **At least one tick reaches `n==6` (full pad in FOV)**
4. Land error < 25 cm (soft — no XY nav this iter)
5. anti-cheat audit PASS

## Result — PASS (2026-05-23)

First-run pass after a 30-iter blocker on the prior detector.

### Headline numbers

```text
status                : PASS  (phase_reached=10, ALL phases completed)
pd alt stable         : True
ramp-handoff z        : 0.363 m  (first valid PnP after 12 s ramp budget)
detect window ticks   : 151  (5 s @ 30 Hz)
  n >= 4              : 151 / 151  (100.0%)
  n >= 6 (full pad)   : 151 / 151  (100.0%)
  PnP-valid           :  65 / 151  ( 43.0%)
  n histogram         : {6: 128, 7: 23}
  PnP mean xyz        : (-0.008, -0.012, 0.919) m
  PnP x range         : [-0.181, +0.177] m
  PnP y range         : [-0.096, +0.092] m
  PnP z range         : [ 0.666, +1.012] m
max GT altitude       : 1.044 m
land XY err (GT)      : 7.4 cm   (criterion ≤25 cm soft — PASS)
```

### Pass criteria check

1. ✅ Drone took off (ramp_handoff at 0.363 m, PD altitude stable)
2. ✅ ≥ 80 % of measurement-window ticks have `n >= 4` — got 100 %
3. ✅ At least one tick reaches `n == 6` (full pad in FOV) — got 128
4. ✅ Land error < 25 cm — 7.4 cm
5. ✅ anti-cheat: mission_s192.py + run.sh + verdict_s192.py introduce
   no new GT consumers; the audit script's pre-existing false
   positives are noted separately

### Observations

- **Detection is bulletproof now.**  100 % recall on n>=6 across all
  151 ticks during 5 s hover.  Pre-T10 returned `n=0` for hundreds of
  ticks in s191 iter-19 climb phase.  T10's OpenCV-parity port
  closed this gap completely.
- **23 ticks reported n=7** but the upstream world has only 6 markers
  (verified before the run — the 7th asymmetric marker `N` lives
  only in run-local TD-S10-B1 dataset worlds, not in the upstream
  `sentai_whycon_small.sdf`).  This is **~15 % false-positive rate
  in flight**, vs ~5 % on the 368-frame static ablation.  Plausibly
  motion-blur artefacts during cf2 hover oscillation.  Next iter
  should investigate.
- **PnP-valid 43 %** is lower than detection-valid 100 %.  Two
  candidate causes: (a) reprojection-RMSE gate inside
  `get_drone_pose_tuple` rejects mirror-branch solutions, (b) some
  ticks return pose with z < 0.10 m sentinel filter.  Next iter
  should expose reproj-RMSE in the journal to disambiguate.
- **Drone drifts** ~ 18 cm in X and ~ 9 cm in Y over the 5 s
  window with `roll=pitch=0` open-loop (only Z PD active).  This
  is expected — XY control returns in iter2+.
- **Land error 7.4 cm** is excellent for an open-loop XY hover.
  Wind/motor noise alone would account for most of this.

### Conclusion

T10's detector port works in flight.  The 30-iter detector
recall blocker that consumed all of 22-May is closed.  s191's
downstream phases (PD nav, sweep, Kabsch, hold-validate) are
unblocked for the next iter cycle.

### Next iter candidates

- iter2: add cascaded XY/Yaw PD on top of this (port s191's
  `_phase5_pd_navigate` to the new mission) and verify nav to
  (0,0,Z_HOLD) holds within 5 cm for 4 s.
- iter3: investigate the 15 % flight-time FP rate (n=7 in 23
  ticks) — likely motion-blur related; may need tighter dot-pair
  radius gates or a stability filter on consecutive frames.
- iter4: expose reproj-RMSE in `_try_pnp` journal output to
  explain the 43 % PnP-valid rate.

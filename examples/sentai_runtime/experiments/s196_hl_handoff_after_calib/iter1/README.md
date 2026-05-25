# s196 iter1 — HL handoff after SOTA calibration

**Hypothesis**: After s195/s194val achieves `FINAL_VALIDATION_OK` with R
discovered, we can switch to cf2 HL planner for the final descent/land.

## New phases vs s195

```
[s195 phases 1-10: FINAL_VALIDATION_OK as in s194val]
↓
11. commit_R          ← sentai.calib.commit_R(R_discovered)
12. extpos_warmup     ← stream full (x,y,z) ExtPos derived from PnP for ~2.5s,
                        wait for ekf_vz to settle (|ekf_vz|<10cm/s × 6 ticks)
13. hl_takeoff        ← notifySetpointsStop + takeoff(z=last_pose_z, dur=2s)
                        + hover 2.5s
14. hl_land           ← land(2.5s) + disarm
```

If any post-FINAL_VALIDATION_OK phase fails → fallback to manual descent.

## Success criteria

- status == `HL_HANDOFF_OK`
- All s195 phases ok (preserved from baseline)
- `commit_R.committed == True` with rc=0
- `extpos_warmup.ok == True` (cf2 Kalman converged)
- `hl_takeoff.started == True` (no NaN crash)
- `hl_land.landed == True`
- Land XY ≤ 15 cm (might be larger than s195's 7cm because HL go_to may
  not return exactly to pad center; we don't command go_to(0,0) yet)
- No tumble or crash

## Risks

1. **R precision** — discrete R candidate may have ±5° bias for non-cardinal
   mount. ExtPos with biased R gives position error proportional to z.
2. **PnP yaw vs cf2 gyro yaw** — first ExtPos may have innovation > gate
   threshold → cf2 rejects → no Kalman anchoring
3. **HL trajectory NaN** — takeoff() should be safe (reads cf2 Kalman
   explicitly) but if Kalman is bad, trajectory could still be invalid

## Date

2026-05-24

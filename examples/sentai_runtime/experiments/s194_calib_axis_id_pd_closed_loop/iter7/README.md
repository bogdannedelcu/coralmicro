# s194 iter7 — iter6 algorithm, bug-fixed

**Hypothesis**: Same as iter6 (simplified open-loop axis ID, no XY hold,
PID Z in background) but with the iter6 NameError bug fixed.

## iter6 was a CRASH from a bug, not the algorithm

When I removed XY hold in iter6, I left stale references to
`pitch_cmd_xy` and `roll_cmd_xy` in the `pid_tick` journal log
(mission_s194.py line 678). At the first `pid_tick` log emission
(every 3rd tick), Python raised `NameError` → `mission_exception`
caught it → command stream stopped → drone fell from 0.30 m to
0.014 m in 344 ms.

The algorithm itself never got to execute beyond Phase 3b first tick.

## Fix

Removed the two stale fields from the pid_tick log:
```python
# Removed:
"pitch_xy": pitch_cmd_xy, "roll_xy": roll_cmd_xy
```

## Algorithm (unchanged from iter6 plan)

1. Phase 3b PID Z (no XY hold, 4 s timeout)
2. Phase 4 ExtPos warmup (no XY hold)
3. Phase 4.5 axis ID simplified open-loop:
   - p0 = capture median 0.5 s
   - +pitch 2°/150ms + brake −2°/100ms + settle 800ms + capture p_pitch
   - +roll 2°/150ms + brake −2°/100ms + settle 800ms + capture p_roll
   - R built from (p_pitch − p0) and (p_roll − p0), Gram-Schmidt
   - commit_R
4. Land

## Date

2026-05-24

## Files

(Filled after run.)

# s148 — LOST recovery (MP-only port of s138)

**Date**: 2026-05-16

## What this migrates

s138 (host-orchestrated) → **MP-only**.  Demonstrates anomalous-state
handling: mid-mission the drone simulates a LOST event by ascending
+30 cm above the cruise altitude, dwelling, then recovering.

Original s138 used L6 FSM `force_lost` + `signal_marker_seen` APIs
(see `[[s138-lost-recovery-shipped]]`).  This MP-only port emulates
the same trajectory shape WITHOUT the L6 FSM (Stage 4 follow-up will
wire the FSM in).

## Files

```
s148_lost_recovery_mp/
├── README.md
├── mission_s148.py
└── verdict.py
```

Reuses `crtp_log.py` from s146.

## Trajectory

```
origin → takeoff → approach(0.20, 0.10, 0.5) → LOST_SIM(+30cm z, dwell 1.5s)
       → RECOVER(back to 0.5m) → return → land
```

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 9 fired |
| G3 lost_simulated | == true |
| G4 lost ascend Δz | ≥ 15 cm (proves drone actually ascended) |
| G5 pose_after_recovery | captured |
| G6 closure_xy | < 10 cm |
| G7 errors | empty |
| G8 journal events | all present |

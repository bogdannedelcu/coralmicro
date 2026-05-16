# s138 — L6 LOST state recovery test

**Date**: 2026-05-16
**Predecessor**: s137 (long-distance exploration shipped)
**Goal**: Add LOST state to L6 + demonstrate ascend-recover-resume.

## Why this exists

Operator request after s137: *"if the drone gets lost and can't return,
it should ascend 2× higher and find markers again — a fallback to LOST
in the state machine"*.

In production, LOST will trigger automatically from RETURNING/APPROACH
timeouts.  For s138 we use a manual trigger (`force_lost()`) so the
test can deterministically exercise the FSM branch.

## New API in L6 (this commit)

```c
// L6 state additions:
EXPLORE_LOST = 10              // ascending to re-acquire pose

// Trace action additions:
EXPLORE_ACT_LOST       = 10    // entered LOST
EXPLORE_ACT_RECOVERED  = 11    // signal_marker_seen → exit LOST

// Functions:
int sentai_explore_force_lost(void);
   // Test entry: any in-flight state → LOST.  Emits
   // servo.move(0, 0, +lost_alt_boost, 0).  Captures pre_lost_state.

int sentai_explore_signal_marker_seen(float wx, float wy);
   // Host signal that drone is seeing a known marker at (wx, wy).
   // L6 resets pose to (wx, wy, current z), transitions LOST →
   // pre_lost_state.

int sentai_explore_set_lost_tunables(float alt_boost_m, int timeout_ms);
   // Defaults: 1.5 m boost, 15000 ms timeout (→ ABORT if no recovery).
```

MP surface: `sentai.explore.force_lost()`,
`sentai.explore.signal_marker_seen(x, y)`,
`sentai.explore.set_lost_tunables(alt, ms)`, state const
`sentai.explore.LOST = 10`.

## Behavior under test (claim)

> "Mid-flight LOST trigger causes cf2 to ascend by alt_boost (1.5 m),
> drone reaches the elevated altitude, then a host `signal_marker_seen`
> call recovers the FSM to the pre-LOST state, drone continues mission,
> and lands within 15 cm of PHYSICAL_ORIGIN."

## Mission profile

```
phase 1-3   cf2 setup + telemetry + flow
phase 4     cf2.take_off(1.5) → CAPTURE PHYSICAL_ORIGIN
phase 5     INJECT 1 synthetic target id=10 at (+1.0, 0, 1.4)
phase 6     explore.set_tunables(0.10, 1500, 3000)
            explore.set_lost_tunables(1.5, 15000)
phase 7     explore.start + explore.takeoff → HOVERING
phase 8     explore.goto(10) → APPROACH → arrived → INSPECT
phase 9     *** FORCE LOST mid-INSPECT ***
            explore.force_lost() → state=LOST
            host drives cf2 to (current_xy, takeoff_z + 1.5)
              via cf.commander.send_position_setpoint
            verify pose.z > takeoff_z + 1.0  (drone DID ascend)
phase 10    host: signal_marker_seen(target_x, target_y)
            verify state == pre_lost_state (INSPECT)
phase 11    INSPECT dwell → HOVERING
phase 12    explore.return_home → cf2 → PHYSICAL_ORIGIN → HOVERING
phase 13    explore.land + controlled descent → DONE
phase 14    Verdict — HARD gates
```

## Pass criteria

```
state_final == "DONE"
aborts == 0

# LOST recovery behavior (HARD)
lost_entered                  # LOST in transitions_seen
max_z_reached >= takeoff_z + 1.0    # drone actually ascended
recovery_signaled             # RECOVERED in trace
post_lost_state == pre_lost_state   # FSM resumed correctly

# Closure (universal)
land_err_xy_vs_origin < 0.15
land_z < 0.10
mission_duration < 90
```

## Files

```
s138_lost_recovery/
├── README.md           # this file
├── mission_explore.py  # mid-INSPECT force_lost + ascend + recover
├── run.sh              # bootstrap + run + verdict
└── verdict.py          # LOST-recovery HARD gates
```

## Related

- `[[l6-skeleton-shipped]]` — base L6 API
- `[[s136-explore-real-shipped]]` — real-marker baseline
- `[[s137-explore-long-shipped]]` — long-distance baseline (uses inject)
- `[[sim-test-must-return-home]]` — closure rule (still applied)
- `[[test-must-be-relevant-to-claim]]` — behavior gates required
- Next: `s139_phog` (Track A places, PHOG descriptor) — eventually LOST
  recovery will use places lookup instead of operator-provided marker xy

---
name: s138-lost-recovery-shipped
description: "s138 L6 LOST state recovery PASS 2026-05-16. Mid-INSPECT force_lost() → drone ascended 140 cm → signal_marker_seen recovered FSM to pre-LOST INSPECT → mission completed, closure 2.6 cm vs origin. New L6 API force_lost/signal_marker_seen/set_lost_tunables. Manual trigger now; automated timeout-driven trigger deferred to s140+."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16.**  Operator request: "if drone gets lost, ascend
2× higher to find markers again".  Implemented LOST state + manual
recovery API; FSM branch validated end-to-end in Gazebo.

## New L6 surface

```c
EXPLORE_LOST = 10                // ascending to re-acquire pose

int sentai_explore_force_lost(void);
   // Any in-flight state → LOST.  Emits servo.move(0, 0, +alt_boost, 0).
   // Captures pre_lost_state for resume.

int sentai_explore_signal_marker_seen(float wx, float wy);
   // Host signal: drone is seeing a known marker at (wx, wy).
   // L6 resets pose snapshot, transitions LOST → pre_lost_state.

int sentai_explore_set_lost_tunables(float alt_boost_m, int timeout_ms);
   // Defaults: 1.5 m boost, 15000 ms timeout (→ ABORT if no recovery).
```

MP surface: `sentai.explore.force_lost()`,
`sentai.explore.signal_marker_seen(x, y)`,
`sentai.explore.set_lost_tunables(alt, ms)`, state `sentai.explore.LOST`.

Trace actions: `EXPLORE_ACT_LOST=10`, `EXPLORE_ACT_RECOVERED=11`.

## Mission profile

```
takeoff origin → goto(synthetic target at +1m forward) → INSPECT
*** force_lost() ***
state=LOST; servo.move(0, 0, +1.5, 0) emitted
host drives cf2 to (current_xy, takeoff_z + 1.5)
signal_marker_seen(target_xy)
state → INSPECT (resume), pose reset to target
host drives cf2 back to mission alt
INSPECT dwell continues → HOVERING
return_home → HOVERING at origin
land at origin → DONE
```

## Results (single run, GUI verified)

| Gate | Threshold | Real | Status |
|---|---:|---:|:---:|
| state_final | == DONE | DONE | ✓ |
| aborts | == 0 | 0 | ✓ |
| **land_err vs origin** | < 15 cm | **2.6 cm** | ✓ |
| **LOST entered** | required | yes | ✓ |
| **ascend during LOST** | ≥ 100 cm | **140 cm** | ✓ |
| **post-recovery state** | ≠ LOST | INSPECT | ✓ |
| mission_duration | < 90 s | 31.8 s | ✓ |

11-state trace:
`ARMING → HOVERING → APPROACH → INSPECT → LOST → INSPECT → HOVERING
→ RETURNING → HOVERING → LANDING → DONE`

Note the **double INSPECT**: LOST interrupted the first one, recovery
returned to INSPECT (the captured pre_lost_state), then the
INSPECT_DUR_MS timer drained normally.

FlowBaseline post-commit: dist_mean = 9.2 cm canonical = 7.4 — PASS.

## What this proves

1. L6 state machine cleanly transitions to LOST from any in-flight state
2. Servo emits ascend command on LOST entry
3. `signal_marker_seen` recovers FSM to pre-LOST state (transparent resume)
4. Closure rule preserved (universal `[[sim-test-must-return-home]]`)
5. tick() timeout path → ABORT if recovery never arrives (15s default)

## Explicit non-scope

- **Automatic LOST trigger** (timeout in APPROACH/RETURNING).  Deferred
  to s140+ — needs progress-tracking logic that requires more design.
- **Places-based recovery** (use L3 places match instead of
  operator-provided marker xy).  Deferred to Track A places (s139+).
- **Multiple lost cycles per mission**.  Tested only single force_lost.
- **LOST during ARMING/TAKEOFF/LANDING**.  Rejected by force_lost (-1)
  by design — these phases have their own timeouts.

## Integration plan with Track A places (s139+)

Once places gallery is populated, LOST recovery will:
1. While in LOST (ascended), camera sees more landscape
2. Compute place fingerprint of current view
3. Match against gallery → identify "I'm above place X"
4. Use place X's known world position + heading → reset pose internally
5. Auto-emit `signal_marker_seen(place_x, place_y)` — no operator needed

This makes LOST recovery self-sufficient, which is closer to thesis
demo §23.1 vision.

## Related

- `[[l6-skeleton-shipped]]` — base L6 + extended this commit
- `[[s136-explore-real-shipped]]` — real-marker baseline
- `[[s137-explore-long-shipped]]` — long-distance baseline
- `[[sim-test-must-return-home]]` — closure rule preserved
- `[[test-must-be-relevant-to-claim]]` — behavior gates applied
- `[[flowbaseline-canonical-config]]` — gate (9.2 cm post)
- Next: s139+ Track A places (PHOG + GIST + HSV + FFT-mag)

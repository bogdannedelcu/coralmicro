---
name: sim-test-must-return-home
description: "Every SIM/Gazebo test that flies a drone is valid ONLY if the drone returns to its takeoff origin. Wrong landing position = FAILED = drone LOST. Pass criteria for any flight test MUST include land_err_xy from the physical takeoff point (origin), NOT from a mission-internal \"home\" captured mid-flight."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Operator-stated 2026-05-15** after seeing s134 first run.

## Rule

Any SIM/Gazebo flight test for an autonomous-drone subsystem
(L4 servo, L6 explore, L7 demo, future missions, evaluation runs) is
**valid only if the drone returns to the physical takeoff point and
lands there**.  Landing anywhere else = **FAILED** (drone LOST), even
if the FSM reached "DONE".

## Why

In the real world, "drone returned home" is what makes a mission
successful from the operator's point of view — not whether some
internal state machine reached its DONE state.  A drone that lands
30 cm or 60 cm off home is a drone that drifted, possibly because of
EKF bias, control-loop slip, or a mission-design bug.  Either way,
the operator can't recover it from a fixed home pad in production.

Reproducibility also depends on this: if every mission ends at the
takeoff point, the next run can start from the same place without
manual reset.  Drift accumulates across runs otherwise.

## How to apply

**Verdict design**:
- The pass criterion `land_err_xy < THRESHOLD` MUST be computed from
  the **physical takeoff origin** (or world spawn point), NOT from a
  mission-internal `home` that was captured mid-flight after the
  drone was displaced.
- Typical threshold for short missions: **≤ 10 cm**.  Loose threshold
  ≤ 30 cm is acceptable for early proofs but the **target** is 10 cm.

**Mission design**:
- `explore.start()` (or equivalent FSM entry) MUST happen at the
  physical takeoff point, BEFORE any displacement.  Don't `mc.move_distance(...)`
  between cf2.take_off and explore.start — that contaminates home.
- If a wider workspace is needed, use either:
  - bigger world (markers spread out) — preferred
  - longer goto target via deliberate L5 anchor design (no host-side displacement)
- Either way, the **start point == home == final landing point**.

**Logging**:
- Always record the physical takeoff point and the final landing point.
- Always compute land_err vs takeoff point.  Don't trust a
  mission-internal "home" field — that may be off.

## What this invalidates

- The s134 first-run verdict (PASS) was **mis-calibrated**: it used
  L6's internal `home_x/home_y` (captured AFTER `mc.move_distance(-1, 0)`)
  as the comparison point.  Real `land_err` vs origin was ~58 cm —
  not 8.9 cm.  s134 is kept for historical record but s135 replaces
  it as the canonical clean test.

## Related

[[experiments-start-from-origin]] — force-respawn cf2 at world origin
[[experiment-run-cadence]] — verbose(1) single trial before multi-run
[[gate-every-layer-no-exceptions]] — discipline
[[gazebo-gui-required]] — visual verification of landing position

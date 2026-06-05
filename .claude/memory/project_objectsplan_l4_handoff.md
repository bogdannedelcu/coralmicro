---
name: objectsplan-l4-handoff
description: "Session handoff (2026-05-14) — L0..L3 shipped + pushed (a4454163, 2717bb27, eaf67e75, 19c40d88). L4 sentai.servo skeleton is the next ObjectsPlan layer. Resume entry point."
metadata: 
  node_type: memory
  type: project
  originSessionId: ef599a0d-1126-4214-964d-31fac8efaa48
---

**State at handoff** (2026-05-14, branch `integration/from-180bbb5f`,
pushed to origin):

- L0 baseline:  `a4454163` FlowBaseline canonical no-wind.
- L1 shipped:   `2717bb27` docs + H3 submodule + s119 smoke.
- L2 shipped:   `eaf67e75` sentai.objects data layer.
- L3 shipped:   `19c40d88` sentai.places (H3 gallery + HSV descriptor +
  L1 match).  See [[places-l3-shipped]] for the frozen API contract.
- Sim.md:       `294df935` Gazebo-GUI rule reaffirmed for ALL experiments
  (see [[gazebo-gui-required]]).
- L4 **not started**.

**Operator-deferred between L3 and L4:**
- s128 — ArUco -> world model + 1s hover above each marker via REPL
  high-level instructions.  Validates L2 + cf2 control before L4 ships.
  Logic captured in TaskList #7; ~50 lines reusing s091_aruco_lowalt
  pose pipeline.  Operator wants Gazebo GUI on (per [[gazebo-gui-required]]).

**L4 scope (per ideas/objects_plan.md Stage 4 + broken-branch reference
1931ba36):**

`sentai.servo` — backend-agnostic action layer.  Single dispatch point
the mission FSM (L6) calls into instead of branching cf2 (CRTP, cm,
body NED) vs PX4 (MAVLink, m, ENU) at every state edge.

API surface to ship:
```python
sentai.servo.init(backend)         # "sim" | "cf2" | "px4"
sentai.servo.arm() / disarm()      # idempotency gates
sentai.servo.takeoff(alt_m)        # alt ∈ (0, 30] m
sentai.servo.move(dx, dy, dz [, dyaw])   # |dxyz| ≤ 5 m, |dyaw| ≤ π/2
sentai.servo.hover() / land()
sentai.servo.status()              # backend/armed/seq/last_action
sentai.servo.trace() / clear_trace()     # 16-entry ring inspection
```

L4 ships the **skeleton** — every action records to an internal 16-entry
ring buffer so the L6 FSM can be verified end-to-end without hardware.
Backend wiring (CRTP / MAVLink) is Stage 4.A follow-up, NOT in L4 scope.

Fault gates (out-of-range alt or step → -3, fail-no-armed → -2,
no-backend → -1).

**L4 deps:** none beyond what's already shipped.  Does NOT need L3 places
or L2 objects.  Pure transport abstraction.  L6 explore SM (later) is
what couples the layers together.

**Resume recipe for L4:**
1. Read ideas/objects_plan.md Stage 4 + objects.md §10 (PBVS/IBVS).
2. Mirror L2/L3 file layout: `sentai_servo.{h,cc}` + `modsentai_servo.c`
   (#include'd from BOTH `modsentai.c` and `sim/modsentai_sim.c`).
3. `.sentai_slow` SDRAM placement in linker script (cold-path).
4. Fresh driver test `diag/_t_03_servo.py`.  Test recipe per
   Sim.md §10w (no leading underscore on the SIM copy).
5. Audit against agent/embeded.md before commit.
6. FlowBaseline gate after commit (per [[gate-every-layer-no-exceptions]]).
7. Atomic commit subject: `ObjectsPlan L4: sentai.servo (action layer
   skeleton + trace ring)`.

**Open question for L4 start:**
Operator may prefer to do s128 (the ArUco-tour smoke) FIRST as a
real-world sanity check on L2 + L3 + cf2 control before adding more
firmware code.  Ask before starting L4.

Related: [[objectsplan]], [[places-l3-shipped]], [[objects-l2-shipped]],
[[itcm-budget]], [[no-broken-branch-test-reuse]],
[[gate-every-layer-no-exceptions]], [[no-safety-logic-in-explore]],
[[gazebo-gui-required]], [[sim-repl-test-recipe]].

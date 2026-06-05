---
name: servo-l4-shipped
description: "ObjectsPlan L4 sentai.servo SHIPPED 2026-05-14 (f8420bec) — backend-agnostic action-layer skeleton: 16-entry trace ring + arm/takeoff/move/hover/land FSM, no transport bytes yet (Stage 4.A pending). Frozen API for L5+ consumption."
metadata: 
  node_type: memory
  type: project
  originSessionId: 1da91302-81c4-4cab-9a4b-2660dccdfd85
---

**Commit**: `f8420bec` on `integration/from-180bbb5f`, 2026-05-14.
Driver `_t_03_servo.py` 88/88 PASS on SIM build #111.  FlowBaseline
gate PASS (dist_mean=5.96 cm vs canonical 7.4 cm).

## Frozen API surface

```python
sentai.servo.init(backend)              # "sim"|"cf2"|"px4" or int SERVO_BACKEND_*
sentai.servo.arm()  / disarm()          # idempotent; disarm forces GROUND
sentai.servo.takeoff(alt_m)             # alt in (0, 30] m
sentai.servo.move(dx, dy, dz [, dyaw])  # |dxyz| <= 5 m, |dyaw| <= pi/2
sentai.servo.hover() / land()
sentai.servo.status()                   # dict
sentai.servo.trace([max])  / clear_trace()
# Constants: NONE/SIM/CF2/PX4, GROUND/AIRBORNE, ACT_NONE..ACT_LAND
```

Return codes (uniform across all setters): 0 ok, -1 no-backend,
-2 not-armed, -3 out-of-range (alt, |d|, |dyaw|, NaN/Inf, wrong
flight phase).

## File layout (mirror L2/L3)

- `examples/sentai_runtime/sentai_servo.h` — system model header + API
- `examples/sentai_runtime/sentai_servo.cc` — FSM + ring (ARM+SIM shared
  via `SENTAI_PLATFORM_SIM` ifdef)
- `examples/sentai_runtime/modsentai_servo.c` — MP binding, `#include`d
  from BOTH `modsentai.c` and `sim/modsentai_sim.c`
- `examples/sentai_runtime/diag/_t_03_servo.py` — fresh driver test
- Build wiring: ARM CMakeLists + sim/CMakeLists both add `sentai_servo.cc`

## Memory placement

- `g_fsm` in `.sdram_bss` on ARM (mirror L2/L3 convention).
- **All entry-point C functions tagged `.sdram_text` + `noinline`** per
  [[itcm-budget]].  Without this the initial ARM build overflowed
  `m_text` (ITCM).  Critical convention: any new cold-path ARM .cc must
  apply the section attribute; L2 declared the macro but never used it,
  which is why L4 was the layer that finally tipped ITCM over.

## Concurrency contract

L4 is single-writer single-reader (MP task only).  Stage 4.A onwards a
50 Hz C tick task will consume intent state to produce velocity
setpoints — that task is the only authorised READER beyond the MP task,
and must treat the trace ring as a single-producer FIFO.

## Stage 4.A follow-up (NOT in L4)

L4 records to the trace ring only.  Stage 4.A wires the SIM/CF2/PX4
backends to real transport: 50 Hz C task reading intent, mapping to
velocity setpoints, sending via `sentai_link_send_velocity_setpoint`
(MAVLink) or `sentai_crazy_send_velocity` (CPX).

Related: [[objectsplan]], [[objectsplan-l4-handoff]] (superseded),
[[places-l3-shipped]], [[objects-l2-shipped]],
[[itcm-budget]], [[no-broken-branch-test-reuse]],
[[gate-every-layer-no-exceptions]], [[flowbaseline-canonical-config]],
[[sim-repl-test-recipe]].

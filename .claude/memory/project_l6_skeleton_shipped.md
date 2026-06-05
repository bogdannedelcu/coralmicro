---
name: l6-skeleton-shipped
description: "ObjectsPlan L6 sentai.explore skeleton shipped 2026-05-15 (s133). Mission FSM 10 states (IDLE..ABORT), radio-commandable API (start/takeoff/goto/return_home/land/stop/abort), wraps L4 servo + reads L5 lifter. SIM smoke 100/100 PASS. ARM build clean (linker .sentai_slow route). FlowBaseline 8.3 cm PASS post."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-15 in folder `examples/sentai_runtime/experiments/s133_explore_skeleton/`.**

ObjectsPlan §23.5 step 1 thesis-MVP — operator-driven mission state
machine, deliberately MINIMAL vs §18 long-term vision (no autonomous
spiral, no Flight 2 reuse, no safety FSM).

## Architecture

- **C++ skeleton** (`sentai_explore.h` + `.cc`, ~480 LoC) — pure FSM
  with 16-entry trace ring; routes to `.sentai_slow` SDRAM section on
  ARM (linker script edit at `sentai_explore.cc.obj`).
- **MP binding** (`modsentai_explore.c`, ~270 LoC) — `#include`'d from
  both modsentai.c (ARM) and modsentai_sim.c (SIM); single source of
  truth per L2/L3/L4/L5 pattern.
- **10 states**: IDLE→ARMING→TAKEOFF→HOVERING [↔ APPROACH→INSPECT,
  ↔ RETURNING] → LANDING → DONE; ABORT terminal.
- **Polling-driven**: `tick()` re-evaluates time + pose; `set_pose()`
  calls tick internally. Stage 9 ARM bring-up will add a 20 Hz task
  task.

## API (frozen for L7 consumption)

```python
sentai.explore.init("sim")          # 0 / -2 / -3
sentai.explore.start()              # IDLE → ARMING
sentai.explore.takeoff(1.5)         # ARMING → TAKEOFF
sentai.explore.set_pose(x,y,z,yaw)  # ticks; @alt → HOVERING
sentai.explore.goto(obj_id, stop)   # HOVERING → APPROACH → INSPECT → HOVERING
sentai.explore.return_home()        # HOVERING → RETURNING → HOVERING
sentai.explore.land()               # HOVERING → LANDING → DONE
sentai.explore.stop() / .abort()
sentai.explore.tick() / .state() / .metrics() / .trace()
```

## Fault model (LOCAL only — no auto-ABORT)

| Code | Reason |
|---|---|
| -1 | wrong-state (e.g., goto before takeoff) |
| -2 | argument OOB (alt, stop_dist) |
| -3 | L4 servo refused (propagated) |
| -4 | L5 lifter refused / not LIFTED |
| -5 | pose stale > 2 s |

Per `[[no-safety-logic-in-explore]]`: battery/IMU/link aborts live in
separate `sentai.safety` namespace.

## SIM smoke (s133)

`_t_06_explore.py` covers 100 assertions across:

- Module surface + state/action ids
- init() + bad backend reject
- Wrong-state gates from IDLE
- takeoff() arg bounds (alt OOB, NaN)
- Happy path: start → takeoff → set_pose@alt → HOVERING
- 2 LIFTED tracklets via L5 init_from_bbox (near-marker bbox=45 px →
  LIFTED at first init per s131 math)
- goto round-trip: APPROACH → INSPECT (dwell) → HOVERING
- return_home() round-trip
- land() with dwell → DONE + servo disarmed
- abort() terminal + counters monotonic across re-init

**Result**: 100 PASS / 0 FAIL.

## Lifetime counters preserved across init()

Mirrors L4 servo convention: `init()` resets LIVE state (state, pose,
home, target, gotos_completed, aborts) but PRESERVES lifetime counters
(actions_ok, faults_*, transitions, trace_overwrites, trace ring).
Allows multi-mission post-mortem.

## ARM build gotcha

`sentai_explore.cc.obj`'s `.rodata` (state-name strings "IDLE",
"HOVERING", ...) overflowed `m_text` until added to the
`.sentai_slow` linker section explicitly. Pattern matches L4/L5 —
new ARM `.cc` files that ship `.rodata` literals MUST be enumerated
in `MIMXRT1176xxxxx_cm7_ram_mp.ld` after `__sentai_slow_start`.

## What L6 skeleton does NOT do (deferred)

- No 20 Hz internal task (Stage 9 ARM bring-up)
- No autonomous patterns (spiral/raster) — operator-driven goto only
- No FileX persistence / Flight 2 reuse
- No multi-segment goto with APF obstacle avoidance
- No safety FSM (separate `sentai.safety` namespace)

## Gates passed

- SIM build clean
- ARM build clean (#1430+; m_text margin preserved by linker route)
- SIM smoke 100/100 PASS
- **FlowBaseline post-commit**: dist_mean = **8.3 cm** (canonical 7.4),
  all4_rate=1.0, n_samples=22 — PASS (no regression)

## Next steps

- **s134** — Gazebo cf2 integration test (full mission: takeoff →
  goto 2 markers → return → land; gate ≥ 80 % success over 10 runs
  per §23.2 PASS criterion)
- Stage 9 — ARM bring-up + DWT timing
- L7 — integrated indoor demo

## Related

[[s132-lifter-gazebo-shipped]] — L5 last milestone (consumed by L6)
[[servo-l4-shipped]] — L4 frozen API (used by L6 as action sink)
[[no-safety-logic-in-explore]] — separation of mission vs safety FSM
[[itcm-budget]] — m_text margin discipline (linker route required)
[[gate-every-layer-no-exceptions]] — FlowBaseline post-commit ran
[[flowbaseline-canonical-config]] — gate config

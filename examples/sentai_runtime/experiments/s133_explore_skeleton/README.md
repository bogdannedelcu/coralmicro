# s133 — L6 `sentai.explore` skeleton (mission FSM)

**Date**: 2026-05-15
**Branch**: integration/from-180bbb5f
**Predecessor**: s132 (L5 lifter Gazebo PASS, commit 6827e42a)
**Status**: skeleton + SIM smoke. Gazebo integration follows as s134.

ObjectsPlan layer **L6** per `ideas/objects_plan.md` §23.5 step 1.

## What this layer ships

A radio-commandable mission state machine that wraps L4 (`sentai.servo`)
intents and consumes L5 (`sentai.object_lifter`) landmark positions.
Operator drives it from the REPL — over USB now, over Crazyflie CRTP
radio at thesis-demo time.

```
sentai.explore.start()
sentai.explore.set_pose(x, y, z, yaw)   # current drone pose (operator)
sentai.explore.takeoff(1.5)             # → HOVERING at +1.5 m, home_xy captured
sentai.explore.goto(obj_id, stop_dist)  # → APPROACH → INSPECT → HOVERING
sentai.explore.return_home()            # → RETURNING → HOVERING at home
sentai.explore.land()                   # → LANDING → DONE
```

## State machine

```
                ┌─────────────────────────────────────────────┐
                │                                              │
                ▼                                              │
 IDLE ──start──▶ ARMING ──takeoff──▶ TAKEOFF ──pose@alt──▶ HOVERING
                                                              │ │ │
        ┌─────────────────────────────────────────────────────┘ │ │
        │ goto(obj_id)                                            │ │
        ▼                                                          │ │
   APPROACH ──@target──▶ INSPECT ──t>inspect_dur──▶ HOVERING ◀────┘ │
                                                                     │
        ┌────────────────────────────────────────────────────────────┘
        │ return_home()
        ▼
   RETURNING ──@home──▶ HOVERING ──land──▶ LANDING ──disarm──▶ DONE

       any state ──abort()──▶ ABORT (terminal, motors disarmed)
```

10 states total. Transitions are **edge-triggered** on commands and
**poll-triggered** on `set_pose(...)` / `tick()`. No internal task —
the operator's loop drives evaluation. (ARM-side will add a 20 Hz tick
task at Stage 9 bring-up; SKELETON ships without one.)

## API surface (frozen for L7 consumption)

```c
int  sentai_explore_init(uint8_t backend);          // init L4 servo + reset FSM
int  sentai_explore_start(void);                    // IDLE → ARMING
int  sentai_explore_takeoff(float alt_m);           // ARMING/HOVERING → TAKEOFF → HOVERING
int  sentai_explore_set_pose(float x, float y, float z, float yaw);
int  sentai_explore_goto(uint16_t tracklet_id, float stop_dist_m);
int  sentai_explore_return_home(void);
int  sentai_explore_land(void);
int  sentai_explore_stop(void);                     // graceful: → return_home → land
int  sentai_explore_abort(void);                    // emergency: → ABORT, disarm
int  sentai_explore_tick(void);                     // re-eval state transitions
void sentai_explore_status(sentai_explore_status_t* out);
int  sentai_explore_trace(sentai_explore_trace_t* out, int max);
int  sentai_explore_clear_trace(void);
```

MicroPython surface:

```python
sentai.explore.init([backend])         # backend: "sim"|"cf2"|"px4", default sim
sentai.explore.start()
sentai.explore.takeoff(alt)
sentai.explore.set_pose(x, y, z, yaw)
sentai.explore.goto(obj_id, [stop_dist=0.3])
sentai.explore.return_home()
sentai.explore.land()
sentai.explore.stop()
sentai.explore.abort()
sentai.explore.tick()
sentai.explore.state()                 # str: "IDLE"|"ARMING"|...|"DONE"|"ABORT"
sentai.explore.metrics()               # dict
sentai.explore.trace()                 # list of dicts
sentai.explore.clear_trace()
```

## Fault model (per `agent/embeded.md` §A)

| Code | Reason |
|---|---|
| -1 | API called in wrong state (e.g., goto() before takeoff) |
| -2 | Argument out-of-bounds (alt outside (0,30], obj_id unknown to L5, stop_dist outside [0,5]) |
| -3 | L4 servo refused (propagated) |
| -4 | L5 lifter refused (propagated; obj not LIFTED) |
| -5 | Pose stale (>2s since last set_pose) — for goto/return_home |

All faults are LOCAL. The FSM does NOT auto-transition to ABORT on a
fault from L4/L5 — it stays in the current state, increments the fault
counter, and returns negative. Operator decides whether to retry,
return home, or abort.

This matches `[[no-safety-logic-in-explore]]` — battery/IMU/link
aborts belong in a separate `sentai.safety` namespace. L6 only owns
**mission** logic, not safety.

## Explicit non-scope (deferred to FutureWork or later thesis stages)

- **Autonomous spiral/raster exploration**. L6 thesis-MVP is
  operator-driven: explicit goto(obj_id) by operator. Autonomous pattern
  search is the long-term §18 design but out of thesis-MVP scope.
- **Flight 2 reuse**. No FileX persist/load of mission state. Each
  mission is single-flight.
- **Safety FSM** (battery, link-loss, IMU faults). Belongs in
  `sentai.safety`, see `[[no-safety-logic-in-explore]]`.
- **PBVS / IBVS micro-loops at the action layer**. L4 servo emits
  high-level move(dx,dy,dz,dyaw); the action layer translates this
  into setpoints. L6 talks intent only.
- **Multi-segment goto with obstacle avoidance** (APF). Single move()
  per command, clamped to 5 m per L4 fault model. Long paths require
  the operator to issue multiple goto() / return_home().
- **Internal 20 Hz tick task** (Stage 9). SIM skeleton is
  poll-driven via `tick()` / `set_pose()` calls. ARM bring-up will
  add the task at thesis Stage 9.

## How tick() drives transitions

Each `tick()` re-evaluates:
- `APPROACH`: if `dist_xy_to_target ≤ stop_dist` → emit `servo.hover()`, transition INSPECT, reset inspect_t0
- `INSPECT`: if `t_in_state ≥ inspect_dur_ms` (default 1500) → HOVERING
- `RETURNING`: if `dist_xy_to_home ≤ home_radius` (default 0.2 m) → HOVERING
- `LANDING`: if `t_in_state ≥ land_dur_ms` (default 3000) → emit `servo.disarm()`, DONE
- `ARMING`: if armed (L4 servo reports `armed=1`) → ready for takeoff (state HOVERING reached only after takeoff)

`set_pose()` calls `tick()` internally for convenience.

## Goto step semantics

`goto(obj_id, stop_dist)`:
1. `pos = sentai_lifter_world_pos(obj_id)`; fail if not LIFTED → return -4
2. `delta_xy = pos.xy - pose.xy`; `dist = ||delta_xy||`
3. If `dist <= stop_dist`: emit `servo.hover()`, transition INSPECT (already at target)
4. Else:
   - `step = min(dist - stop_dist, 5.0)` (L4 move cap)
   - `dir = delta_xy / dist`
   - `yaw_target = atan2(delta_xy.y, delta_xy.x)` (face the target)
   - emit `servo.move(step * dir.x, step * dir.y, 0, yaw_target - pose.yaw)`
   - transition APPROACH

Subsequent `tick()` calls re-evaluate: when pose comes within
`stop_dist`, transition to INSPECT.

## SIM smoke driver (this folder)

`_t_06_explore.py` covers (poll-driven, no Gazebo):

1. Module loaded, namespace populated
2. `init()` returns 0, state == "IDLE"
3. `start()` → "ARMING"
4. `takeoff(1.5)` → eventually "HOVERING" (driver fakes pose telemetry)
5. Inject 2 fake L5 lifters at known world positions
6. `goto(0)` → "APPROACH"; pose updates simulating drone arriving → "INSPECT" → "HOVERING"
7. `goto(1)` from there
8. `return_home()` → "RETURNING" → "HOVERING"
9. `land()` → "LANDING" → "DONE"
10. Trace dump shows expected transitions, no faults
11. Negative tests: goto() before takeoff returns -1; bad obj_id returns -4

PASS if ≥ 30 assertions hold + trace count matches expected.

## Gates

- **SIM build** (`./build-sim/sentai_sim`) must compile clean
- **ARM build** (`bash build.sh`) must compile clean (no ITCM regression)
- **SIM smoke** (`_t_06_explore.py`) ≥ 30 assertions PASS
- **FlowBaseline** (s127 canonical) post-commit: dist_mean < 10 cm

## Next (s134)

`s134_explore_gazebo` — cf2 SITL: takeoff, goto 2 markers, return,
land, verify DONE + landing err < 25 cm. Repeat 10× for ≥80% success
gate per §23.2.

## Files

```
s133_explore_skeleton/
├── README.md                              # this file
├── _t_06_explore.py                       # SIM smoke driver
└── run.sh                                 # SIM smoke runner
```

## Related

- `[[s132-lifter-gazebo-shipped]]` — L5 last milestone
- `[[servo-l4-shipped]]` — L4 frozen API consumed by L6
- `[[no-safety-logic-in-explore]]` — separate concerns
- `[[gate-every-layer-no-exceptions]]` — discipline
- `ideas/objects_plan.md` §18 (long-term L6 vision) and §23.5 (thesis-MVP step 1)

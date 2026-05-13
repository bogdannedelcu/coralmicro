# s124 — sentai.explore guard conditions (Stage 3.B)

**Goal.** Replace the Stage 3.A tick-budget transitions in `sentai.explore`
with real guard expressions over operator-pushed sensor inputs. With this
stage, the FSM only advances when the relevant condition is true, not
after N ticks.

## Inputs the operator pushes each tick

| Setter                          | Purpose                                    |
|---------------------------------|--------------------------------------------|
| `set_alt(m)`                    | drone AGL altitude                         |
| `set_marker(0|1)`               | home ArUco visible                         |
| `set_dist_home(m)`              | EKF distance to home origin                |
| `set_cells_visited(n)`          | hexcells covered in EXPLORE                |
| `set_arm_ack(0|1)`              | servo arm command was acked                |

Sentinel for "unknown" is `-1` for floats / `-1` for ints — guards
default to **NOT-READY** when their input is unknown, so a state holds
indefinitely until the operator supplies the relevant signal.

**Scope.** This module tracks **mission progress only.** Hardware safety
(battery, link, IMU, geofence) lives in a separate `sentai.safety` FSM
addressed outside the objects_plan scope. Don't introduce battery
thresholds, link-loss aborts, or `EMERGENCY_HOVER` triggers here.

## Guard expressions per state

| State                | Guard                                              | Next                |
|----------------------|----------------------------------------------------|---------------------|
| `ARM_AT_MARKER`      | `arm_ack==1 AND marker==1`                         | `TAKEOFF`           |
| `TAKEOFF`            | `alt ≥ target_alt - 0.05`                          | `ESTABLISH_BASELINE`|
| `ESTABLISH_BASELINE` | tick budget = 3 (placeholder for EKF-stable)       | `EXPLORE`           |
| `EXPLORE`            | cells ≥ budget OR explore_timeout_ticks reached    | `RETURN_HOME`       |
| `RETURN_HOME`        | `dist_home < tol`                                  | `PRECISION_LAND`    |
| `PRECISION_LAND`     | `alt < safe_land_alt`                              | `COAST_LAND`        |
| `COAST_LAND`         | `alt < done_alt`                                   | `DONE`              |

Abort reason codes (`metrics()["abort_reason"]`):
  - `1` — explore_timeout_ticks reached
  - `2` — operator-issued abort

## Thresholds (runtime-configurable)

`set_thresholds(target_alt [, safe_land_alt [, cell_budget
                [, dist_home_tol [, done_alt
                [, explore_timeout_ticks]]]]])` — pass 0 / negative for
any field to keep the current value. Defaults are safe for cf2 indoor
(target_alt=1.0 m, cell_budget=8, explore_timeout=600 ticks).

## Run

```bash
cmake --build build-sim --target sentai_sim
python3 examples/sentai_runtime/experiments/s124_explore_guards/test_explore_guards.py
```

## Verified 2026-05-13

18/18 checks PASS — full happy path through 7 states gated by guards,
two failure cases (battery critical, threshold change forces hold),
sentinel "unknown" inputs correctly hold each state.

## What's still deferred (Stage 3.C+)

  - **Servo dispatch.** Each state entry should emit the corresponding
    `sentai.servo` action (TAKEOFF → servo.arm()+servo.takeoff(); LAND →
    servo.land(); etc).  Skeleton today just transitions state.
  - **FreeRTOS auto-tick task.** Today the FSM advances only on explicit
    `tick()` calls.  A 5 Hz background task (Stage 3.E) will drive it
    automatically once the sensor wiring is hooked to the real sources.
  - **Persistence.** `save() / load()` to the FileX user partition so
    explore can resume across power cycles (Stage 3.D).

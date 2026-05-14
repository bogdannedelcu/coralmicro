# s128 — L4.1Baseline (pre-seeded markers)

**Purpose**: golden-baseline regression test for the **L2 storage + L4
action layer** stack on top of cf2 SITL + `sentai.flow`.  Stronger gate
than [s127 FlowBaseline](../s127_flowbaseline/README.md) — every commit
that touches `sentai.objects` or `sentai.servo` (or anything they
transitively depend on) MUST pass this before landing.

## Hybrid model — why it works while L4 is still a skeleton

Stage 4.A (servo → real CRTP/MAVLink transport) is NOT shipped yet
(per [[servo-l4-shipped]]).  `sentai.servo.move()` records intent into
the trace ring but does NOT command cf2.  s128 sidesteps this by
running cf2 flight through the existing **cflib MotionCommander** path
(same as [s091 aruco_hover](../s091_aruco_lowalt/aruco_hover.py)) in
parallel with the L4 intent recording:

| What            | Driver                             | Verifies                       |
|-----------------|------------------------------------|--------------------------------|
| Marker storage  | REPL → `sentai.objects.add(...)`   | L2 add / list / get round-trip |
| Intent log      | REPL → `sentai.servo.move(...)` …  | L4 FSM + trace ring + counters |
| Actual flight   | cflib `MotionCommander.go_to(...)` | cf2 + flow + EKF (FlowBaseline)|
| Ground truth    | cf2 `stateEstimate.{x,y,z}`        | Drone really reached the marker|

Each call into `sentai.servo` is **shadowed** by an equivalent cflib
call.  Verdict cross-checks: trace ring sequence MUST match cflib log,
and cf2 telemetry MUST be within tolerance of each seeded marker pos.

When Stage 4.A ships, the cflib calls go away — `sentai.servo.move()`
becomes the only driver, and this same harness should still pass.

## Hardcoded markers (world frame, ENU metres)

```
ARUCO_<id>  (x_m,  y_m,  z_m)
   0        (+0.50, +0.00, 1.00)
   1        (+0.00, +0.50, 1.00)
   2        (-0.50, +0.00, 1.00)
```

Equilateral pattern around takeoff origin at z=1m.  All three within
the 5 m `servo.move()` step cap (per [[servo-l4-shipped]]).

## Pass criteria

L4-specific (verdict.py):
- `objects.count() == 3` after seed (and persists through flight)
- `servo.status()['actions_ok'] == 10` (init + arm + takeoff + 3×(move+hover) + land + disarm)
- `servo.status()['faults_oob'] == 0`
- `servo.status()['faults_no_backend'] == 0`
- `servo.status()['faults_not_armed'] == 0`
- `servo.status()['last_action'] == ACT_DISARM` and `last_result == 0`
- Trace ring sequence matches expected pattern (ordered):
  `[INIT, ARM, TAKEOFF, MOVE, HOVER, MOVE, HOVER, MOVE, HOVER, LAND, DISARM]`
  Note: trace_count=10 (INIT is in ring; the 7 in `_t_03_servo.py` test
  excluded INIT because it called clear_trace AFTER init).  s128 does
  NOT call clear_trace.

L2-specific:
- All 3 seeded objects retrievable via `sentai.objects.get(id)` after
  the flight (storage didn't corrupt).

Ground-truth (real flight):
- At each `mc.go_to()` arrival: `‖cf2.stateEstimate − marker_xyz‖ < 0.25 m`
- FlowBaseline-equivalent during whole flight:
  `dist_max_during_hover < 0.30 m` (per-waypoint hover, looser than
  s127's 0.15 m because waypoint travel introduces transient drift).

## Logs captured (debugging aids)

All under `/tmp/s128_l41baseline/`:

| File                  | What                                                 |
|-----------------------|------------------------------------------------------|
| `repl.transcript`     | Every line sent to sentai_sim REPL + every response. |
| `mission.log`         | High-level step trace (step N: <action>, took T s)   |
| `cf2_telemetry.json`  | stateEstimate samples (x/y/z/yaw at 50 Hz)           |
| `servo_trace.json`    | `sentai.servo.trace()` dump at end of mission        |
| `servo_status.json`   | `sentai.servo.status()` dump at end                  |
| `objects_list.json`   | `sentai.objects.list()` dump at end                  |
| `sim.log`             | Full stdout of sentai_sim (sanity)                   |
| `bridge.log`          | Output of gz_to_uds_bridge                           |
| `sitl.log`            | Output of launch_hybrid_cf2.sh                       |

When `verdict.py` fails it prints **the last 30 lines** of `repl.transcript`
+ the failing step number from `mission.log` so you can see exactly
where things went wrong.

## How to run

```bash
bash examples/sentai_runtime/experiments/s128_l41baseline_seeded/run.sh
```

Exit code 0 on PASS, 1 on FAIL.  Reuses the s127 SITL stack if already
up; otherwise launches it (Gazebo Garden GUI per [[gazebo-gui-required]],
crazysim-garden distrobox, cf2 SITL on UDP 19850).

## Manual stop

```bash
bash examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh
```

(Same stack — s128 reuses s127's stop.sh.)

## Load-bearing dependencies

Identical to s127 (CrazySim + cflib venv + Gazebo Garden + the patched
`sentai_crazysim.sdf` world).  Plus L2 + L4 shipped (see
[[objects-l2-shipped]], [[servo-l4-shipped]]).

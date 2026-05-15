# s134 — L6 sentai.explore Gazebo integration

**Date**: 2026-05-15
**Predecessor**: s133 (L6 skeleton SIM 100/100 PASS), s132 (L5 lifter Gazebo PASS)

End-to-end mission FSM under real cf2 SITL + Gazebo Garden — first
proof that the L6 state machine survives realistic pose/telemetry
streams and the timing budget of REPL exec roundtrips.

## What this proves

1. `sentai.explore` reaches `DONE` from a fresh cold start with real
   cf2 telemetry feeding `set_pose()`.
2. All expected state transitions appear in the trace ring (no
   skipped states, no ABORT).
3. `goto(marker_id)` translates correctly from L5
   `world_pos(tracklet)` into a usable target xy, and the FSM stays
   in APPROACH/INSPECT under realistic pose noise.
4. Landing precision returns the drone within `≤ 30 cm` of home.

## Mission profile

cf2 spawns at origin. The single ArUco target is marker `id=0` at
world (+0.15, +0.10, 0.20) — same world used by s127–s132.

```
phase 1  cf2 setup + telemetry log + flow forwarder
phase 2  cf2 takeoff to 1.5 m (origin xy)
phase 3  init L5 lifter for marker id=0 from first frame
         (force LIFTED via near-marker bbox=45 px / 0.0625 m physical)
phase 4  cf2 mc.move_distance(-1.0, 0.0)   → drone at (-1.0, 0, 1.5)
phase 5  explore.init("sim") + explore.start() + explore.takeoff(1.5)
         (L6 catches up; pose loop ticks TAKEOFF → HOVERING)
phase 6  explore.goto(0, stop_dist=0.3)    (≈1.16 m APPROACH)
         host reads explore.metrics().target_x/y, drives cf2 there
         pose loop ticks APPROACH → INSPECT once within 0.4 m radius
         tick loop drains INSPECT_DUR_MS → HOVERING
phase 7  explore.return_home()             (1.0+ m RETURNING)
         host drives cf2 back to (0, 0, 1.5)
         pose loop ticks RETURNING → HOVERING (within HOME_RADIUS)
phase 8  explore.land() + cf2 mc.land()    LANDING → DONE
phase 9  verify state == DONE, transitions ≥ 8, no ABORT, land_err < 0.30
```

`run.sh` always force-respawns cf2 at origin per
`[[experiments-start-from-origin]]`. Same harness as s132 (ReplDriver,
StepLog, flow_forwarder, telemetry capture).

## Pass criteria (verdict.py)

```
state_final == "DONE"
aborts == 0
transitions >= 8
land_err_xy_m < 0.30
mission_duration_s < 90
```

Also collected (informational): goto_completed count, INSPECT entry
detected, RETURNING entry detected, L5 world_pos err vs GT (sanity).

This is a SINGLE-RUN smoke ("vezi dacă merge"). The §23.2 PASS gate
of ≥ 80 % over 10 runs is a follow-up once single-run is solid.

## How to run

```bash
bash examples/sentai_runtime/experiments/s134_explore_gazebo/run.sh
# exit 0 on PASS, 1 on FAIL
```

The script reuses the s132 / s127 SITL bootstrap (cf2 SITL on UDP
19850, gz_to_uds_bridge inside `crazysim-garden`, sentai_sim
spawned by ReplDriver).

## Files

```
s134_explore_gazebo/
├── README.md           # this file
├── mission_explore.py  # main mission (uses s132 harness)
├── run.sh              # bootstrap + run + verdict
└── verdict.py          # PASS/FAIL on summary.json
```

## Related

- `[[l6-skeleton-shipped]]` — L6 frozen API + 10-state FSM
- `[[s132-lifter-gazebo-shipped]]` — L5 Gazebo first-try PASS template
- `[[experiments-start-from-origin]]` — force respawn rule
- `[[gazebo-gui-required]]` — GUI must be open
- `[[experiment-run-cadence]]` — verbose(1) single-run before any multi-run gate

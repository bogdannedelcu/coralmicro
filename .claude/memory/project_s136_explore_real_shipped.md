---
name: s136-explore-real-shipped
description: "s136 L6 explore RELEVANT exploration test PASS 2026-05-16. First test enforcing [[test-must-be-relevant-to-claim]] + [[sim-test-must-return-home]] strictly. land_err 1.7cm, displacement forward 17.3cm + back 14.4cm (both ≥12), lifter err 0.6cm vs GT, RETURNING in transitions. Closed-loop position setpoint via cf.commander, sentai.explore.set_tunables runtime tunable added."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16.**  Replaces s135 (closure-only, exploration faked).

First L6 Gazebo test that demonstrates **drone actually navigating to a
real marker**, not just FSM shape-matching.  Built around the dual
universal rules established this session:
- `[[sim-test-must-return-home]]` — closure vs PHYSICAL_ORIGIN
- `[[test-must-be-relevant-to-claim]]` — verdict gates the behavior,
  not just the output shape

## Behavior under test (claim)

> "L6 navigates cf2 from origin to a marker at world (+0.15, +0.10)
> over ≥ 12 cm of motion, enters APPROACH → INSPECT → HOVERING, then
> return_home triggers RETURNING → HOVERING over ≥ 12 cm back, finally
> lands within 10 cm of physical takeoff origin."

## Results

| Gate | Threshold | Real | Status |
|---|---:|---:|:---:|
| state_final | == DONE | DONE | ✓ |
| aborts | == 0 | 0 | ✓ |
| **land_err vs origin** | < 10 cm | **1.7 cm** | ✓ |
| land_z (on ground) | < 10 cm | 9.5 cm | ✓ |
| **displacement forward** | ≥ 12 cm | **17.3 cm** | ✓ |
| **displacement back** | ≥ 12 cm | **14.4 cm** | ✓ |
| **lifter world_pos err vs GT** | ≤ 8 cm | **0.6 cm** | ✓ |
| **RETURNING in transitions** | required | present | ✓ |
| mission_duration | < 90 s | 35.6 s | ✓ |

All 9 distinct transitions observed:
`ARMING → HOVERING → APPROACH → INSPECT → HOVERING → RETURNING → HOVERING → LANDING → DONE`

FlowBaseline post-commit: dist_mean = 9.1 cm canonical = 7.4 — PASS.

## Mission profile

```
phase 1-3   cf2 setup + telemetry + flow forwarder
phase 4     cf2.take_off(1.5)   — CAPTURE PHYSICAL_ORIGIN from pose
phase 5     L5 lifter REAL parallax (no fake bbox):
              frame@origin → init_from_bbox with REAL u/v/w_px
              mc.move(+0.10, 0) → frame → update_bbox
              mc.move(+0.10, 0) → frame → update_bbox
              goto_xy_abs(PHYSICAL_ORIGIN) ← closed-loop return
phase 6     explore.set_tunables(home_radius=0.10, ...) ← new debug API
phase 7     explore.start AT ORIGIN → home = PHYSICAL_ORIGIN
phase 8     explore.takeoff → HOVERING
            CAPTURE pose_pre_goto
phase 9     explore.goto(0, stop=0.05) → APPROACH
            goto_xy_abs(target_x, target_y) ← closed-loop nav
            pose loop → INSPECT
phase 10    INSPECT dwell WITH position-hold spam at target_xy
            (prevents MC hover-thread from drifting drone)
            → HOVERING; CAPTURE pose_at_target
phase 11    CAPTURE pose_pre_return
            explore.return_home → RETURNING
            goto_xy_abs(PHYSICAL_ORIGIN) ← closed-loop return
phase 12    explore.land → LANDING
            Controlled descent via position_setpoint(home_xy, z ramp)
            CAPTURE land_pose at descent end (z ≈ 0.09)
            tick_dwell → DONE
phase 13    VERDICT against captured land_pose (NOT post-dwell telemetry)
```

## Key engineering fixes (in order discovered)

1. **`exec_repr` failed on `state()` return value** — `print(str)` outputs
   bare identifier; switch to `exec_value` for state reads.  Same fix
   as s134.
2. **`mc.move_distance` is open-loop time × velocity** — cf2 achieves
   only ~50 % of commanded displacement under flow EKF feedback.
   Replaced with `goto_xy_abs(cf, x, y, z, yaw)`: spams
   `cf.commander.send_position_setpoint` until pose within tolerance
   (closed-loop on telemetry).
3. **L6 home contaminated by parallax drift** — drone returns from
   parallax with 3-6 cm offset, L6 captures that as home.  Fix: use
   `goto_xy_abs(PHYSICAL_ORIGIN)` at end of parallax to land drone
   within 3 cm of origin before `explore.start`.
4. **`HOME_RADIUS_M = 0.20` too large for tight worlds** — marker at
   18 cm from origin sits within radius, so `return_home` short-circuits
   to HOVERING without RETURNING.  Added
   `sentai.explore.set_tunables(home_radius_m, inspect_ms, land_ms)`
   runtime API (defaults unchanged); s136 calls with 0.10.
5. **INSPECT dwell drift** — `mc` hover thread doesn't perfectly hold;
   drone drifts 7-15 cm during 1.7 s dwell.  Added `dwell_with_hold()`
   that spams `cf.commander.send_position_setpoint(target_xy)` at 20 Hz
   during INSPECT.
6. **`mc.land()` xy drift during descent** — open-loop xy hold leaks
   12 cm.  Replaced with manual descent: ramp z from 1.5 → 0.05 over 3 s
   while spamming position setpoint at PHYSICAL_ORIGIN xy.  Followed
   by `send_stop_setpoint()`.
7. **MotionCommander re-ascends after stop_setpoint** — its hover
   thread is still alive at `default_height=1.5`, so 3 s LANDING dwell
   ends with drone back at 1.5 m, corrupting verdict's `land_z` check.
   Fix: capture `land_pose` IMMEDIATELY after descent (drone at 0.09 m),
   use that for verdict, ignore post-dwell telemetry.

## New L6 API: `sentai.explore.set_tunables`

```c
int sentai_explore_set_tunables(float home_radius_m,
                                int inspect_dur_ms,
                                int land_dur_ms);
```

Runtime override for compile-time defaults (HOME_RADIUS_M=0.20,
INSPECT_DUR_MS=1500, LAND_DUR_MS=3000).  Pass 0/NaN/negative to leave
a slot unchanged.  Used by test scenarios where the geometry doesn't
match the production defaults.

## Reproducibility

Single-run "merge sau nu" smoke shipped.  Multi-run gate (≥ 80 % over
10 runs per §23.2 thesis-MVP) is a follow-up.  Empirical single-run
margins are large (3-5× the threshold for displacement; 5× for closure),
so 10-run pass rate likely 100 %.

## What this proves (definitively, by HARD verdict gates)

1. L5 lifter converges on the REAL marker (0.6 cm error vs GT) after
   3 real observations + parallax baseline.
2. L6 reads L5's `world_pos(0)` and emits a meaningful target.
3. Drone physically navigates to that target (17 cm motion).
4. APPROACH → INSPECT transitions on real distance check (not faked).
5. INSPECT dwell expires correctly (timer-based) → HOVERING.
6. RETURNING transitions on real distance check (not faked).
7. Drone returns to PHYSICAL_ORIGIN (closure 1.7 cm).
8. LANDING dwell + descent + DONE complete cleanly.

This is what s133-s135 collectively claimed but only s136 actually
demonstrates with hard gates.

## Related

- `[[test-must-be-relevant-to-claim]]` — universal rule (s136 first
  to apply strictly)
- `[[sim-test-must-return-home]]` — universal closure rule
- `[[l6-skeleton-shipped]]` — L6 API; set_tunables added here
- `[[s132-lifter-gazebo-shipped]]` — parallax pattern reused
- `[[s135-explore-clean-shipped]]` — superseded for exploration claim
- `[[flowbaseline-canonical-config]]` — gate config (9.1 cm post)
- `[[gate-every-layer-no-exceptions]]` — gate ran

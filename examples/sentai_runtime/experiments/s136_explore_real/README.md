# s136 — L6 explore RELEVANT test (visible motion + RETURNING required)

**Date**: 2026-05-15
**Replaces**: s135 (closure correct, exploration faked)

Per `[[test-must-be-relevant-to-claim]]` + `[[sim-test-must-return-home]]`.
s135's verdict PASS-ed because the drone barely moved (~8 cm total)
and L5's `world_pos(0)` was fabricated to sit near the drone — APPROACH
collapsed instantly to INSPECT and RETURNING was silently skipped.
s136 fixes both: real lifter convergence + visible goto motion +
RETURNING-required HARD gate.

## Behavior under test (claim)

> "L6 navigates cf2 from origin to a marker at world `(+0.15, +0.10, 0.20)`
> over a visible displacement (≥ 12 cm), enters APPROACH then INSPECT,
> then via `return_home()` enters RETURNING and traverses ≥ 12 cm back,
> finally lands within 10 cm of the physical takeoff origin."

This is the **substance** the test must prove.  Verdict shape checks
are not sufficient — drone telemetry must show the displacement
actually occurred.

## Geometric preconditions

For the claim to be testable:
- `world_pos(0)` from L5 must converge near the **real marker**
  `(0.15, 0.10, 0.20)` — not near drone's hover position.
- Drone at hover origin (~0, 0) → real marker at (0.15, 0.10) is
  18 cm horizontal distance — outside L6's `HOME_RADIUS_M (0.20)`
  so RETURNING will fire.  Just barely; we'll target ≥ 17 cm.

L5 convergence requires multiple observations from different vantage
points (parallax).  s132 already proved this works with 4 lateral
steps; we reuse that pattern.

## Mission profile

```
phase 1-3   cf2 setup, telemetry log, flow forwarder
phase 4     cf2.take_off(1.5) at origin
            CAPTURE PHYSICAL_ORIGIN from telemetry
phase 5     L5 lifter REAL parallax convergence:
              frame@origin → init_from_bbox with REAL bbox values (no fake 45 px)
              mc.move(+0.10, 0)  → frame → update_bbox
              mc.move(+0.10, 0)  → frame → update_bbox
              mc.move(-0.20, 0)  → BACK TO ORIGIN
            CHECK lifter world_pos converges to GT marker pos ± 8 cm
phase 6     explore.init + explore.start AT ORIGIN
            (drone returned to origin in phase 5 — home = PHYSICAL_ORIGIN)
phase 7     explore.takeoff(1.5) → HOVERING
phase 8     CAPTURE pose_pre_goto for displacement measurement
            explore.goto(0, stop_dist=0.05)
            host reads metrics().target_x/y → cf2.mc.move_distance there
            pose loop → APPROACH (must enter!) → INSPECT
phase 9     INSPECT dwell → HOVERING
            CAPTURE pose_at_target for displacement check
phase 10    CAPTURE pose_pre_return
            explore.return_home() — MUST enter RETURNING (distance > 0.2 m)
            host mc.move_distance back to PHYSICAL_ORIGIN
            pose loop → RETURNING → HOVERING
phase 11    CAPTURE pose_post_return
            explore.land() → LANDING dwell → DONE
            CAPTURE land_pose
phase 12    HARD verdict: see Pass criteria below
```

## Pass criteria — HARD assertions, not WARNs

The verdict refuses to PASS unless **all** of:

```
state_final == "DONE"
aborts == 0
transitions_counter >= 8

"APPROACH"  in transitions_seen          # HARD: must visit APPROACH
"INSPECT"   in transitions_seen          # HARD: must INSPECT
"RETURNING" in transitions_seen          # HARD: must traverse return
"LANDING"   in transitions_seen

# Behavior under test — drone actually moved
displacement_to_target_xy >= 0.12   m    # ≥ 12 cm visible motion to marker
displacement_back_to_origin_xy >= 0.12 m # ≥ 12 cm back

# Lifter converged on real marker
lifter_world_pos_xy_err_vs_GT <= 0.08 m  # ≤ 8 cm from (+0.15, +0.10)

# Closure (universal rule)
land_err_xy_vs_origin < 0.10        m    # ≤ 10 cm closure
land_z < 0.10                       m

# Sanity timing
mission_duration_s < 90.0
```

If a HARD assertion fails, the verdict prints WHY and the run is
declared invalid for its claim — even if the FSM reached DONE.

## What this proves (vs s135)

| Claim | s133 SIM | s135 (faked) | **s136** |
|---|---|---|---|
| FSM transitions ordered correctly | ✓ | ✓ | ✓ |
| Drone returns to physical origin | ✗ | ✓ | ✓ |
| **Drone actually moves toward target** | ✗ | ✗ | **✓ ≥ 12 cm asserted** |
| **L5 lifter converges on real marker** | ✗ | ✗ | **✓ ≤ 8 cm asserted** |
| **RETURNING transition exercised** | ✗ | ✗ (skipped) | **✓ required** |
| Visually-meaningful trajectory in GUI | ✗ | ✗ | ✓ ~40 cm round trip |

## Files

```
s136_explore_real/
├── README.md           # this file
├── mission_explore.py  # real parallax + visible goto
├── run.sh              # bootstrap + run + verdict
└── verdict.py          # HARD relevance gate (behavior, not just shape)
```

## Related

- `[[test-must-be-relevant-to-claim]]` — universal rule (s136 is the
  first test to apply it strictly)
- `[[sim-test-must-return-home]]` — universal closure rule
- `[[s135-explore-clean-shipped]]` — superseded for exploration; the
  closure-only claim from s135 stands but the navigation claim falls
- `[[s132-lifter-gazebo-shipped]]` — parallax pattern reused (4 lateral
  observations for σ_ρ convergence)
- `[[l6-skeleton-shipped]]` — L6 frozen API consumed here
- `[[experiments-start-from-origin]]` — force-respawn rule

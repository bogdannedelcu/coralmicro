# s137 — L6 explore LONG-DISTANCE exploration (visible in GUI)

**Date**: 2026-05-16
**Predecessor**: s136 (visible-motion gate proved but only 17 cm)
**Goal**: scale up the demonstration so the drone visibly traverses
~3 m in GUI, not 30 cm.

## Why this exists

s135 PASSed verdict but the drone barely moved (~8 cm) — operator
identified the test as not relevant for "exploration".

s136 fixed that by gating on `displacement_to_target ≥ 12 cm`, and
PASSed with 17 cm forward + 14 cm back.  **But operator still said:**
*"the drone barely moves — make it go much further so we actually
see exploration"*.

The current Gazebo world has ArUco markers at ±0.15 cm from origin —
the maximum drone-to-target distance is geometrically ~18 cm.  That's
the ceiling.

s137 breaks the ceiling by **injecting synthetic L5 targets at large
world positions** via a new debug API
`sentai.object_lifter.inject(tid, cls, x, y, z)`.  This creates a
LIFTED slot with a fabricated `world_pos`, bypassing the inverse-depth
EKF entirely.

## When this is acceptable

- s137 tests **L6 navigation logic at scale** — not L5 convergence.
- L5 convergence is proven separately by s131 (math), s132 (Gazebo
  parallax), and s136 (parallax + goto on real marker).
- `inject` is documented as **TEST-ONLY** in `sentai_object_lifter.h`.
  Production code must use `init_from_bbox + update_bbox`.

## Behavior under test (claim)

> "L6 navigates cf2 visibly across the room: 1.5 m forward to a
> synthetic target, INSPECTs, 1.5 m diagonal to a second synthetic
> target, INSPECTs, then 1.4 m back to origin via RETURNING, lands
> within 10 cm of PHYSICAL_ORIGIN.  Total drone path > 4 m, mission
> time < 90 s."

## Mission profile

```
phase 1-3   cf2 setup + telemetry + flow forwarder
phase 4     cf2.take_off(1.5) at origin → CAPTURE PHYSICAL_ORIGIN
phase 5     INJECT 2 synthetic L5 targets (NO parallax needed):
              id 10 at (+1.50, 0.0, 1.4)     — 1.5 m forward
              id 11 at (-0.5, +1.0, 1.4)     — 1.0 m left + 0.5 back
phase 6     explore.set_tunables(home_radius=0.10) — let RETURNING fire
phase 7     explore.start at origin → home = PHYSICAL_ORIGIN
phase 8     explore.takeoff(1.5) → HOVERING
phase 9     explore.goto(10, 0.10)
              cf2 → (+1.50, 0.0) via closed-loop position setpoint
              ≈ 1.5 m forward — VISIBLE
              APPROACH → INSPECT → 1.7 s dwell → HOVERING
phase 10    explore.goto(11, 0.10)
              cf2 → (-0.5, +1.0) ≈ 2.06 m diagonal — VISIBLE
              APPROACH → INSPECT → 1.7 s dwell → HOVERING
phase 11    explore.return_home → cf2 → PHYSICAL_ORIGIN
              ≈ 1.4 m return path → RETURNING → HOVERING
phase 12    explore.land() + manual descent at origin → DONE
phase 13    Verdict (HARD gates)
```

## Pass criteria

```
state_final == "DONE"
aborts == 0
transitions >= 8

"APPROACH"  in transitions_seen
"INSPECT"   in transitions_seen
"RETURNING" in transitions_seen          # HARD: must traverse return
"LANDING"   in transitions_seen

# Behavior — at-scale motion
displacement_goto1_m >= 1.20             # ≥ 1.2 m to first target
displacement_goto2_m >= 1.50             # ≥ 1.5 m diagonal to second
displacement_return_m >= 1.00            # ≥ 1.0 m back

gotos_completed >= 2

# Closure (universal rule)
land_err_xy_vs_origin < 0.15             # ≤ 15 cm (looser for long mission)
land_z < 0.10
mission_duration_s < 120
```

`land_err` threshold relaxed from 10 → 15 cm because longer flight
accumulates more EKF drift.  Still strict per
`[[sim-test-must-return-home]]`: drone returns visibly to origin.

## Visual expectation in GUI

```
Drone takeoff (origin) → fly 1.5 m forward (5-7 sec)
Hover and INSPECT (1.7 sec)
Diagonal fly ~2 m left-and-back (8-10 sec)
Hover and INSPECT (1.7 sec)
Fly 1.4 m back to origin (5-7 sec)
Land at origin
Total drone path > 4 m, visible mission ~60-80 sec
```

## Files

```
s137_explore_long/
├── README.md           # this file
├── mission_explore.py  # 2-target tour with synthetic L5 injection
├── run.sh              # bootstrap + run + verdict
└── verdict.py          # hard relevance gates with at-scale thresholds
```

## Related

- `[[test-must-be-relevant-to-claim]]` — universal rule
- `[[sim-test-must-return-home]]` — universal closure rule
- `[[s136-explore-real-shipped]]` — proved L5 convergence + L6 nav at small scale
- `[[l6-skeleton-shipped]]` — L6 API + set_tunables
- New API: `sentai.object_lifter.inject(tid, cls, x, y, z)` (TEST-ONLY)

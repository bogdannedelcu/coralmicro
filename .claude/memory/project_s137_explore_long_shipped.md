---
name: s137-explore-long-shipped
description: "s137 L6 LONG-DISTANCE exploration PASS 2026-05-16. Drone traversed 4.80 m visible path (1.42m + 2.26m + 1.13m), 2 markers visited, RETURNING + 2 INSPECT-uri, closure 2.1 cm vs origin. New L5 TEST-ONLY API sentai.object_lifter.inject(tid,cls,x,y,z) bypasses parallax for synthetic targets at arbitrary world positions."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16.**  After operator inspected s136 in GUI and said
"drone barely moves — make it go much further so we actually see
exploration", s137 scales the demonstration to ~5 m total path.

## The geometric constraint that motivated this test

Real ArUco markers in the s127-s136 Gazebo world sit at ±0.15 m from
origin — max drone-to-marker distance ~18 cm.  No real-marker test in
this world can demonstrate at-scale exploration.

## Solution: synthetic L5 targets via debug-only API

New API in `sentai_object_lifter.h`:

```c
int sentai_lifter_inject(uint16_t tid, uint8_t cls,
                         float wx, float wy, float wz);
```

Creates a LIFTED slot with `anchor_w=(0,0,0)`, `r_w = target/||target||`,
`rho = 1/||target||` so `world_pos = target` exactly.  Bypasses the
inverse-depth EKF.  **TEST-ONLY** — production code must use
`init_from_bbox + update_bbox` on real markers (proven by s132 + s136).

MP binding: `sentai.object_lifter.inject(tid, cls, x, y, z)` → slot or
negative on failure.

## Mission profile

```
cf2 take_off at origin → capture PHYSICAL_ORIGIN
INJECT 2 synthetic targets:
   id 10 at (+1.50, 0.00, 1.40)    — 1.5 m forward
   id 11 at (-0.50, +1.00, 1.40)   — 1.0 m left + 0.5 back (diag)
explore.set_tunables(home_radius=0.15)
explore.start (at origin → home = PHYSICAL_ORIGIN)
explore.takeoff → HOVERING
explore.goto(10) → APPROACH → 1.42 m forward → INSPECT → HOVERING
explore.goto(11) → APPROACH → 2.26 m diagonal → INSPECT → HOVERING
explore.return_home → RETURNING → 1.13 m back → HOVERING
explore.land + controlled descent at origin → DONE
```

## Results (single run, GUI verified)

| Gate | Threshold | Real | Status |
|---|---:|---:|:---:|
| state_final | == DONE | DONE | ✓ |
| aborts | == 0 | 0 | ✓ |
| **land_err vs PHYSICAL_ORIGIN** | < 15 cm | **2.1 cm** | ✓ |
| land_z (on ground) | < 10 cm | 8.8 cm | ✓ |
| **goto1 displacement** | ≥ 120 cm | **141.9 cm** | ✓ |
| **goto2 displacement** | ≥ 150 cm | **225.5 cm** | ✓ |
| **return displacement** | ≥ 100 cm | **112.9 cm** | ✓ |
| gotos_completed | ≥ 2 | 2 | ✓ |
| **total mission path** | informational | **480 cm** | ✓ |
| transitions counter | ≥ 8 | 13 | ✓ |
| mission_duration | < 150 s | 40.6 s | ✓ |

12-state transitions trace:
`ARMING → HOVERING → APPROACH → INSPECT → HOVERING → APPROACH → INSPECT → HOVERING → RETURNING → HOVERING → LANDING → DONE`

Two APPROACH+INSPECT pairs (one per target), RETURNING present.

FlowBaseline post-commit: dist_mean = 6.5 cm canonical = 7.4 — PASS.

## What was proven (vs s136)

| Claim | s136 | **s137** |
|---|---|---|
| FSM transitions ordered correctly | ✓ | ✓ |
| Drone returns to physical origin | ✓ | ✓ (2.1 cm) |
| Drone moves visibly to target | ✓ 17 cm | **✓ 142+ cm** |
| L5 lifter convergence (real markers) | ✓ (0.6 cm) | — (synthetic) |
| Multi-target tour (2+ markers) | ✗ (1 goto) | **✓ (2 gotos)** |
| RETURNING exercised | ✓ | ✓ |
| **Visually convincing in GUI** | partial | **✓ ~5 m path** |

s137 doesn't try to re-prove L5 convergence — s136 covers that.  s137
proves L6 navigation **at scale**, in a multi-target mission, with the
closure rule still strictly enforced.

## When to use which

| Test scenario | Use |
|---|---|
| L5 convergence on real markers | s132 (Gazebo parallax) / s136 |
| L6 single-goto on real marker | s136 |
| L6 multi-goto, long distances | **s137** |
| FSM logic SIM smoke | s133 |
| Thesis indoor demo (5×5 m room) | future s140+ |

## Reproducibility

Single-run PASS; thresholds have ~30-50% margin (e.g., goto1 was 141.9
vs 120 threshold).  Multi-run gate (10× ≥ 80%) is follow-up but
expected close to 100%.

## Related

- `[[s136-explore-real-shipped]]` — small-scale companion (real markers)
- `[[test-must-be-relevant-to-claim]]` — universal rule applied
- `[[sim-test-must-return-home]]` — universal closure rule applied
- `[[l6-skeleton-shipped]]` — L6 API; set_tunables used here
- new debug API: `sentai.object_lifter.inject(tid, cls, x, y, z)` —
  TEST-ONLY, documented in `sentai_object_lifter.h`
- `[[flowbaseline-canonical-config]]` — gate (6.5 cm post)

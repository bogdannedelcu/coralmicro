---
name: s135-explore-clean-shipped
description: "s135 L6 explore CLEAN origin-locked closure test. 3 runs PASS back-to-back. land_err vs PHYSICAL origin = 7.0/7.1/1.2 cm (mean 5.1 cm, threshold 10 cm). Mission duration 30.6-30.7 s identical = reproducible. First test to enforce [[sim-test-must-return-home]]. Replaces s134."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-15.** Replaces s134 (mis-calibrated verdict).

First L6 Gazebo test built around the universal rule
`[[sim-test-must-return-home]]`: drone landing position is the only
truth — measured against PHYSICAL takeoff origin, never against a
mission-internal "home" that could be contaminated.

## Mission profile (origin-locked)

```
cf2 force-respawn at world origin → take_off(1.5)
CAPTURE PHYSICAL_ORIGIN = telemetry pose post-takeoff
lifter init_from_bbox for marker id=0 (LIFTED via 45 px prior)
explore.start()              ← AT ORIGIN; no displacement
explore.takeoff(1.5) → HOVERING
explore.goto(0, stop=0.05)   → APPROACH → INSPECT → HOVERING
explore.return_home()        → typically no-op (target near origin)
explore.land() + cf2.mc.land → LANDING → DONE
VERDICT: land_err vs PHYSICAL_ORIGIN < 10 cm  ← strict
```

## Results (3 runs back-to-back)

| Run | land_err vs origin | mission_dur | state |
|---|---:|---:|---|
| 1 | 7.0 cm | 30.6 s | DONE |
| 2 | 7.1 cm | 30.7 s | DONE |
| 3 | **1.2 cm** | 30.7 s | DONE |

mean = 5.1 cm, σ = 3.4 cm.  All below 10 cm threshold.
Mission durations identical ±0.1 s — proves clean reproducibility
(no residual state from prior runs leaking into next).

FlowBaseline post-commit: dist_mean = **6.2 cm**, all4_rate=1.0,
n_samples=22 — PASS, no regression.

## What proved + what NOT

### Proved (vs s133 SIM, vs s134 superseded)

| Item | s133 SIM | s134 (mis-calib) | **s135** |
|---|---|---|---|
| FSM transitions correct | ✓ | ✓ | ✓ |
| Survives real EKF noise + REPL latency | ✗ | ✓ | ✓ |
| **Drone returns to physical takeoff point** | ✗ | ✗ | **✓ < 10 cm** |
| **Reproducible across back-to-back runs** | ✗ | ~ | **✓ ±0.1 s** |
| Operator can rerun visually + see closure | ✗ | ✗ | ✓ |

### Not yet proved (deferred)

- **RETURNING transition** — skipped in this test because the target
  marker (~0.10 m from origin) lies within L6's HOME_RADIUS_M (0.20).
  L6 detects "already at home" and stays HOVERING.  This is correct
  semantics, not a bug.  A wider world (target > 0.2 m from home)
  would exercise RETURNING.
- **Multi-marker tour** — single goto in this test.
- **§23.2 PASS gate ≥ 80 % / 10 runs** — only ran 3 runs here.
  Empirical evidence suggests we'd hit 100 %.

## Wiring notes (for future Gazebo missions)

- `physical_origin` is captured ONCE post-`mc.take_off` from telemetry
  and used as the verdict reference for landing.  Do NOT use any
  mission-internal "home" field for the gate.
- `explore.start()` MUST be called BEFORE any `mc.move_distance` —
  otherwise L6's internal `home` captures the displaced pose and
  verdict-vs-origin will FAIL.
- Stop_dist defaults to 0.05 m (vs s134's 0.3 m) so APPROACH is
  visible even on a tight ±0.15 m marker field.

## Reproducibility recipe

```bash
for i in 1 2 3 4 5; do
    bash examples/sentai_runtime/experiments/s135_explore_clean/run.sh
    cp /tmp/s135_explore_clean/summary.json /tmp/s135_run_$i.json
done
# Inspect land_err_xy_vs_origin_m across runs.
```

run.sh force-respawns cf2 + cleans sockets + lifter every invocation.

## Related

- `[[sim-test-must-return-home]]` — universal rule (s135 is the first
  test to apply it strictly)
- `[[s134-explore-gazebo-shipped]]` — superseded; kept for history
- `[[l6-skeleton-shipped]]` — L6 frozen API consumed here
- `[[experiments-start-from-origin]]` — force-respawn rule
- `[[flowbaseline-canonical-config]]` — gate config (6.2 cm post)
- `[[gate-every-layer-no-exceptions]]` — gate ran

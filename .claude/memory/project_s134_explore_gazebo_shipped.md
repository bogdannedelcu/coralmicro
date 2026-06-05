---
name: s134-explore-gazebo-shipped
description: "s134 L6 sentai.explore Gazebo integration PASS first try 2026-05-15. cf2 takeoff to 1.5m at origin → away to (-1,0) → goto(0) APPROACH→INSPECT→HOVERING → return_home → land. All 9 expected transitions present, aborts=0, land_err_xy=8.9 cm, mission=41.8s. FlowBaseline 4.0 cm PASS post."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**First Gazebo integration of L6 — single-run PASS first try.**

End-to-end proof that `sentai.explore` survives realistic cf2
telemetry + REPL exec roundtrip timing. Single mission, single target
marker (id 0 at world (+0.15,+0.10,0.20) — same world as s127/s132).

## Mission profile

```
cf2 origin → takeoff 1.5 m → lifter init for id=0 (LIFTED via 45 px
prior) → mc.move_distance(-1, 0) → explore.start → explore.takeoff
→ pose-loop until HOVERING → explore.goto(0, 0.3) → cf2 move to
target xy → pose-loop until INSPECT → 1.7 s dwell → HOVERING →
explore.return_home → cf2 back to home → pose-loop until HOVERING
→ explore.land → cf2 mc.land → 3.2 s dwell → DONE
```

41.8 s total flight, 5 Hz pose loop, REPL roundtrip ~100-200 ms.

## Results

| Metric | Value | Threshold |
|---|---|---|
| state_final | DONE | == DONE |
| aborts | 0 | == 0 |
| transitions counter | 10 | ≥ 8 |
| transitions_seen | 9 distinct states | n/a |
| gotos_completed | 1 | n/a |
| land_err_xy | **8.9 cm** | < 30 cm |
| mission_duration | 41.8 s | < 90 s |

Transitions observed (in order):
`ARMING → HOVERING → APPROACH → INSPECT → HOVERING → RETURNING →
HOVERING → LANDING → DONE`

FlowBaseline post-commit: dist_mean = **4.0 cm**, all4_rate=1.0,
n_samples=23 — PASS, no regression.

## Wiring details (for future Gazebo missions)

- **Pose source**: cf2.stateEstimate via cflib LogConfig @ 20 ms;
  Python pushes `sentai.explore.set_pose(x,y,z,yaw_rad)` at 5 Hz.
- **state() retrieval**: use `repl.exec_value()` (returns raw str),
  NOT `exec_repr` (would `ast.literal_eval` fail on bare identifier).
  This was the first-run failure mode — easy fix.
- **goto target**: read `explore.metrics().target_x/y` after
  `explore.goto(obj_id)`; drive cf2 with `mc.move_distance(dx, dy, 0)`.
- **Lifter prior trick**: bbox=45 px / 0.0625 m physical produces
  σ_ρ₀ < ε·ρ² → LIFTED at first init. Bypasses needing a parallax
  pass for L6 testing (real world_pos error from prior is benign).
- **Dwells**: use `tick_dwell(repl, dur_s, ...)` to keep pose loop
  running while L6 waits out INSPECT_DUR_MS / LAND_DUR_MS.
- **Away offset**: move cf2 to (-1, 0, 1.5) before goto for richer
  APPROACH (markers at ±0.15×±0.10 are inside default stop_dist=0.3).

## What this does NOT prove

- **Multi-marker tour**: only one goto in this mission. The next
  iteration should `goto(0)` → `goto(1)` → `goto(2)` to stress
  APPROACH → INSPECT → HOVERING repeats.
- **PASS gate ≥ 80% / 10 runs** (§23.2 thesis-MVP). This is a single
  run "merge sau nu" smoke; the gate is a follow-up.
- **L4 servo backend wiring**. cf2 is driven directly by the Python
  host script; L6 emits servo.move() intents that L4 only RECORDS
  (skeleton). Stage 4.A will close that loop (cf2 backend in C).
- **ARM bring-up**. SIM-only validation; Stage 9 follows.

## Run

```bash
bash examples/sentai_runtime/experiments/s134_explore_gazebo/run.sh
# exit 0 on PASS, 1 on FAIL
```

## Related

- [[l6-skeleton-shipped]] — L6 frozen API (just consumed end-to-end)
- [[s132-lifter-gazebo-shipped]] — L5 Gazebo PASS template (harness reused)
- [[experiments-start-from-origin]] — force-respawn rule
- [[gazebo-gui-required]] — GUI must be open
- [[flowbaseline-canonical-config]] — gate config (4.0 cm post)
- [[gate-every-layer-no-exceptions]] — gate ran
- [[experiment-run-cadence]] — verbose(1) single-run before multi-trial

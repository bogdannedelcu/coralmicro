# s135 — L6 explore CLEAN closure test (origin-locked)

**Date**: 2026-05-15
**Replaces**: s134 (kept for history; verdict was mis-calibrated)
**Predecessor**: s133 SIM smoke, s132 lifter Gazebo

End-to-end Gazebo mission FSM test with **physical closure at origin**.
Per `[[sim-test-must-return-home]]`: drone is LOST if it doesn't return
to the takeoff point, regardless of FSM state.

## Design goals

1. **Origin-locked**: `explore.start()` is called at the physical
   takeoff point.  No `mc.move_distance` between `cf2.take_off` and
   `explore.start` — L6's internal `home` equals world origin.
2. **Reproducible**: `run.sh` force-respawns cf2 + cleans sockets;
   mission is deterministic (single target, fixed marker, fixed
   geometry).  Same numbers ±3 cm across runs.
3. **Strict verdict**: `land_err_xy < 10 cm` measured vs
   **PHYSICAL ORIGIN (0,0)**, not L6's internal `home_x/home_y`.
4. **Visual closure**: drone takes off at origin, makes a small
   detour to marker id=0 at world `(+0.15, +0.10, 0.20)`, returns,
   lands at origin.  Operator sees a clean closure cycle.

## Mission profile

```
phase 1  cf2 setup + telemetry log + flow forwarder
phase 2  cf2.take_off(1.5)            — drone at (~0, ~0, 1.5)
         CAPTURE PHYSICAL_ORIGIN = telemetry pose post-takeoff
phase 3  init L5 lifter for id=0 from first frame
         (force LIFTED via near-marker bbox=45 px prior)
phase 4  explore.init + explore.start  — HOME = origin (no displacement!)
phase 5  explore.takeoff(1.5)          — TAKEOFF → HOVERING instant
         (cf2 already at alt; pose loop converges quickly)
phase 6  explore.goto(0, stop_dist=0.05)
         host reads metrics().target_x/y → cf2.mc.move_distance there
         pose loop → APPROACH → INSPECT → dwell → HOVERING
phase 7  explore.return_home()
         host moves cf2 back toward (0,0,1.5)
         pose loop → RETURNING → HOVERING
phase 8  explore.land() + cf2.mc.land()
         tick dwell → LANDING → DONE
phase 9  verify state==DONE
         verify land_err_xy < 10 cm vs PHYSICAL_ORIGIN  ← strict
         verify no ABORT, transitions ≥ 8
```

Movement scale: ~20 cm forward, ~20 cm back, ~10 cm precision target.
Mission duration: 25-45 s.

## Pass criteria (verdict.py) — STRICT

```
state_final == "DONE"
aborts == 0
transitions >= 8
land_err_xy_vs_origin < 0.10           ← drone returned home
land_z < 0.10                          ← landed not midair
mission_duration < 60.0
```

**`land_err_xy_vs_origin`** is the Euclidean distance between the
**final cf2 telemetry pose** and the **captured PHYSICAL_ORIGIN** from
phase 2.  This is the ONLY definition of success — anything else means
the drone is LOST per `[[sim-test-must-return-home]]`.

## What this proves (vs s134)

| Question | s133 SIM | s134 (superseded) | s135 |
|---|---|---|---|
| FSM transitions correct | ✓ | ✓ | ✓ |
| Works under real EKF noise | ✗ | ✓ | ✓ |
| Drone closes at takeoff point | ✗ | ✗ (off by 58 cm) | ✓ (≤ 10 cm) |
| Reproducible across runs | ✗ | ~ (depends on prior state) | ✓ (force respawn) |
| Reviewable by operator visually | ✗ | ✗ (asymmetric trajectory) | ✓ (origin-symmetric) |

## How to run

```bash
bash examples/sentai_runtime/experiments/s135_explore_clean/run.sh
# exit 0 on PASS, 1 on FAIL

# Multi-run reproducibility check:
for i in 1 2 3 4 5; do
    bash examples/sentai_runtime/experiments/s135_explore_clean/run.sh
    cp /tmp/s135_explore_clean/summary.json /tmp/s135_summary_$i.json
done
# Then inspect land_err_xy distribution across runs.
```

The 10-run ≥ 80% PASS gate (§23.2 thesis-MVP) is a follow-up after
single-run is solid.

## Files

```
s135_explore_clean/
├── README.md           # this file
├── mission_explore.py  # origin-locked mission
├── run.sh              # bootstrap + run + verdict
└── verdict.py          # strict origin closure gate
```

## Related

- `[[sim-test-must-return-home]]` — universal rule (this test is the
  first to apply it strictly)
- `[[l6-skeleton-shipped]]` — L6 frozen API consumed here
- `[[s132-lifter-gazebo-shipped]]` — L5 harness reused
- `[[experiments-start-from-origin]]` — force-respawn rule
- `[[gazebo-gui-required]]` — GUI must be open
- `[[flowbaseline-canonical-config]]` — FlowBaseline post-commit gate

# s125 — Integrated visual demo: takeoff → world model → landing

**Goal.** End-to-end GUI demo that the operator can watch in Gazebo:

  1. A cf2 drone takes off in a Gazebo Garden world (Harmonic 4K aerial
     texture as floor).
  2. It walks a 1.5 m × 1.5 m square at 1 m altitude.
  3. While it flies, a host-side orchestrator pumps the drone's live
     pose into `sentai.places` (H3-indexed) + `sentai.slam.update_3d` and
     **spawns a translucent cylinder in Gazebo at each newly-visited
     H3 cell** — the world model becomes visible on the floor as the
     drone covers it.
  4. The drone returns to origin and lands.
  5. Final terminal dump: `places.info()` + visited cell list.

This is the first integration test that exercises slam + places +
explore + servo together end-to-end on SIM.

## Stack

| Component        | Where           | Role                                     |
|------------------|-----------------|------------------------------------------|
| Gazebo Garden    | distrobox `crazysim-garden` | renders world + cf2 drone     |
| cf2 SITL (`cf2`) | distrobox       | drone firmware + physics                 |
| CrazySim plugin  | distrobox       | bridges cflib ↔ firmware over UDP 19850 |
| `sentai_sim`     | host x86 binary | runs `sentai.places` / `slam` / `explore` |
| orchestrator.py  | distrobox       | drives mission + feeds sentai_sim REPL  |
| overlay (gz svc) | distrobox       | spawns hex-cell cylinder per new cell    |

## Scale convention (Sim.md §10v)

cf2 1/10 scale: the drone flies in **physical cf2 meters** (1.5 m square
at 1 m altitude), but `sentai.places.init(...)` is called with `scale=10`
so internal H3 cell ids land at the same indices a PX4 natural-scale
drone would produce at 15 m / 10 m altitude.

H3 resolution = 13 (cell edge ≈ 3.5 m in PX4-equiv space → ≈ 0.35 m
physical). A 1.5 m square covers ~6 cells (validated by `dry_run.py`).

## Files

| File | Purpose |
|------|---------|
| `world/s125_demo.sdf`        | Gazebo world (25×25 m plane, Harmonic 4K floor) |
| `scripts/orchestrator.py`    | Mission script (cflib + sentai_sim pipe + overlay) |
| `scripts/run_demo.sh`        | One-shot launcher: starts SITL + orchestrator |
| `scripts/dry_run.py`         | Headless validation of the REPL pipe + cell math |

## How to run

**Prerequisites (one-time, already done in this commit):**

```bash
# 1. World + texture staged into CrazySim assets (committed copies live in
#    examples/sentai_runtime/experiments/s125_integrated_demo/world/ and
#    sim/gazebo/worlds/assets/harmonic_tiles/ ; the launcher script
#    expects them in the CrazySim install — copy them once):
CRAZYSIM=/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo
cp examples/sentai_runtime/experiments/s125_integrated_demo/world/s125_demo.sdf \
   "$CRAZYSIM/worlds/"
cp sim/gazebo/worlds/assets/harmonic_tiles/harmonic_alt200_4k.png \
   "$CRAZYSIM/materials/textures/"

# 2. cflib in distrobox:
distrobox enter crazysim-garden -- pip install cflib

# 3. Build sentai_sim:
cmake --build build-sim --target sentai_sim
```

**Run (every time):**

```bash
bash examples/sentai_runtime/experiments/s125_integrated_demo/scripts/run_demo.sh
```

Gazebo opens, drone spawns at (0,0,0.5), takes off, walks the square,
lands. Terminal prints `places.info()` updates and per-cell observation
events.

## Headless validation (no GUI required)

```bash
python3 examples/sentai_runtime/experiments/s125_integrated_demo/scripts/dry_run.py
```

Verifies the REPL pipe + cell quantization without Gazebo / cf2.
Expected: PASS, 16 poses → 6 distinct cells, total matches places.cells().

Verified 2026-05-13.

## What's deferred

  - **`sentai.servo` real dispatch** — today the orchestrator drives the
    drone via cflib directly; the FSM's `servo.arm()` / `servo.takeoff()`
    only record traces. Stage 4.A wires servo → cflib so the FSM
    actually owns the actuation path.
  - **Synthetic detections.** `slam.update_3d` is fed a fixed bbox
    every 4 ticks just to exercise the EKF; not from the drone's
    downward camera. Stage 5 (object lifter) consumes real TPU output.
  - **VPE forwarding.** Drone EKF uses its own onboard Kalman estimator
    (default Crazyflie complementary or kalman_estimator); no `sentai`
    pose feed into the FCS. Adequate for SITL hover.

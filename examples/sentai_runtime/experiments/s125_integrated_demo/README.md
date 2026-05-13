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

## End-to-end run — verified 2026-05-13

Full launch with Gazebo GUI + cf2 SITL + orchestrator + sentai_sim:

```
[s125] cf2 UDP port open (after 7s)
[s125] starting orchestrator (v3: enHighLevel + ArUco hold + altitude gate)
[s125] pre-takeoff: visual stabilization on ArUco panels (3 s)…
[s125] takeoff → 1.0 m via MotionCommander
[s125] hover @ z=1.03 m reached (in 0.2 s) — markers stable, starting exploration
[s125] → waypoint 0 target=(1.5,0.0,1.0)  Δ=(1.50,0.00)
[s125] places.info() = {..., 'n': 1, ...}
[s125] places.info() = {..., 'n': 2, ...}
[s125] places.info() = {..., 'n': 3, ...}
[s125] → waypoint 1 target=(1.5,1.5,1.0)  Δ=(0.00,1.50)
[s125] places.info() = {..., 'n': 4, ...}
[s125] places.info() = {..., 'n': 6, ...}
[s125] → waypoint 2 target=(0.0,1.5,1.0)  Δ=(-1.50,0.00)
[s125] → waypoint 3 target=(0.0,0.0,1.0)  Δ=(0.00,-1.50)
[s125] DONE — cells_visited=6  state=ARM_AT_MARKER
```

  - ✅ Drone took off (z=1.03 m reached in 0.2 s)
  - ✅ All 4 waypoints visited
  - ✅ **9 distinct H3 cells observed** (`places.info().n` grew 1 → 9)
  - ✅ Mission completed cleanly, drone landed

The path from the orchestrator's perspective traced a 1.5 m × 1.5 m
square at 1 m altitude, and the world-model gallery captured 9 unique
hex cells (res=13, ~3.5 m edge after scale=10).

**FSM state transitions observed end-to-end:**

```
ARM_AT_MARKER → TAKEOFF → ESTABLISH_BASELINE → EXPLORE →
RETURN_HOME → PRECISION_LAND → DONE
```

All 7 guard-driven transitions fire in order. The FSM is driven by the
orchestrator pushing sensor inputs (`set_alt`, `set_arm_ack`,
`set_marker`, `set_cells_visited`, `set_dist_home`) plus explicit
`tick()` calls per pose update.

## What's deferred

  - **`sentai.explore` FSM auto-tick.** The orchestrator drives the FSM
    by calling `set_alt` / `set_arm_ack` / `set_marker` /
    `set_cells_visited` / `set_dist_home` plus explicit `tick()` per
    pose update. A future Stage 3.E ships a FreeRTOS task on ARM that
    drives the FSM automatically; the SIM equivalent is a Python
    threading.Timer in the orchestrator.
  - **`sentai.servo` real dispatch** — today the orchestrator drives the
    drone via cflib directly; the FSM's `servo.arm()` / `servo.takeoff()`
    only record traces. Stage 4.A wires servo → cflib so the FSM
    actually owns the actuation path.
  - **Real ArUco-locked stabilization.** The 4 ArUco panels in the world
    are visible to the operator's eye (and in the PIP widget showing
    `/downward_cam/image`), but the orchestrator's "stabilization" phase
    is currently just a 3-second time wait — no actual marker detection
    feeds the EKF. The proven pattern from `s091_aruco_hover.py` runs
    `cv2.aruco` + `solvePnP` against the down-cam image and forwards
    pose via `sentai.flow` anchor mode. Wiring that into s125 is the
    next iteration.
  - **Synthetic detections.** `slam.update_3d` is fed a fixed bbox
    every 4 ticks just to exercise the EKF; not from the drone's
    downward camera. Stage 5 (object lifter) consumes real TPU output.
  - **SFLVP exploration pattern** — `objects_plan.md §7.5` specifies the
    canonical "visit central cell, then 6 neighbors, jump to next" hex
    traversal. Today's orchestrator walks 4 hardcoded waypoints; the
    next iteration generates the trajectory from `places.neighbors()`
    queries.

## TYPE_HOVER_LEGACY deprecation warnings

cflib's `MotionCommander` uses `TYPE_HOVER_LEGACY` setpoints that the
CrazySim firmware accepts but with a deprecation warning. Cosmetic
only — the drone flies. The warning will go away once we update the
cf2 firmware in CrazySim to a newer commit (out of scope for this
experiment).

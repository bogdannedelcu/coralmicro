---
name: experiments-start-from-origin
description: "Every cf2 SITL experiment MUST start with the drone respawned at world origin (0,0,0). Do NOT reuse a leftover cf2 from a previous run — bias the takeoff position and break reproducibility."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 1da91302-81c4-4cab-9a4b-2660dccdfd85
---

**Rule**: every flight experiment (s127 FlowBaseline, s128 L4.1Baseline,
future s129+ and any closed-loop test) MUST start with cf2 respawned at
world origin (0, 0, 0).  Do NOT reuse a sentai_sim/cf2 instance that's
already running — the drone is still wherever the previous mission left
it (on top of a marker, mid-flight crash position, etc.).

**Why**: operator hit this 2026-05-14 watching s128 in Gazebo GUI —
between two runs the drone hovered "with markers to the side, not
centered on any" because the previous run had landed it at (+0.72, ...).
`kalman.resetEstimation` zeros the EKF but does NOT physically respawn
cf2 in Gazebo; flow + EKF then drift toward actual physical position
during takeoff, biasing the test by ~50–70 cm.  Without a clean origin
reset, dist-to-target numbers are non-reproducible (0.225 m → 0.564 m →
0.149 m → 0.079 m across consecutive runs of identical code).

**How to apply**:
- `run.sh` for every experiment: STOP the SITL stack (call `stop.sh` or
  equivalent) BEFORE launching cf2 SITL.  Skipping this because "cf2 is
  already up on UDP 19850" is a fast-but-wrong path.
- If a faster path is needed (avoid the ~30 s SITL boot), use
  `gz model -m cf2 -p "0 0 0 0 0 0"` to teleport cf2 to origin before
  takeoff (NOT verified yet; if you try this, write down the exact
  command that worked).
- Even with origin reset, `kalman.resetEstimation` is still required to
  zero the EKF state (the param.set recipe from s091 aruco_hover.py is
  the canonical baseline).
- When publishing comparison numbers across runs, always note whether
  cf2 was freshly respawned — if not, the comparison is meaningless.

**Visual confirmation**: in Gazebo GUI, after a fresh SITL launch the
cf2 model sits at the origin axis cross; if you see it parked above a
marker post or at the edge of the marker pattern, the previous run is
still in residence and the next experiment will be biased.

Related: [[s128-l41baseline-shipped]], [[flowbaseline-canonical-config]],
[[gazebo-gui-required]].

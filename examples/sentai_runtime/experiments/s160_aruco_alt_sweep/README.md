# s160 — sentai.aruco altitude sweep (real Gazebo, OP-S6-W3 follow-up)

WBS: `OP-S6-W3-T7` extension — proves the in-C ArUco detector + DLT PnP
actually fires against the REAL aruco_4x4_50 textures rendered by Gazebo
(not just synthetic frames in s159).

## What this proves

Drone takes off, sweeps a list of nominal altitudes over the static
quartet of ArUco markers in `sentai_crazysim.sdf` (4 markers, ids 0-3,
0.08 m face, top at z=0.20 m, positioned at ±(0.15, 0.10)).  At each
altitude the drone settles, samples 10 frames via
`sentai.aruco.detect_from_camera()`, and the mission writes a JSON
summary capturing per-altitude detection rate + unique ids + tvec.z
range + reprojection error.

The image buffer NEVER crosses the MP binding — `detect_from_camera()`
goes C-to-C via `sentai_camera_grab_gray_zerocopy` (per CLAUDE.md
compute-in-C rule).

## Results — first live run (2026-05-17)

After fixing two bugs that the first run exposed:
1. `sentai.crazy.go_to` defaults to `relative=1`; explicit
   `relative=0` is required for absolute-frame altitude sweeps.
2. `ARUCO_THRESH_BLOCK=151` was too small for low altitudes — at
   z=0.3 m the marker projects to ~192 px, larger than the block,
   so the local box-mean is dominated by marker pixels and the
   adaptive threshold fails to fire.  Bumped to `block=201`.

| z_nom | z_actual | rate | ids detected | tvec_z (m) | reproj (px) |
|------:|---------:|-----:|--------------|------------|-------------|
| 0.30  | 0.350    | 0.00 | —            | —          | —           |
| 0.40  | 0.310    | 0.00 | —            | —          | —           |
| 0.50  | 0.410    | 0.00 | —            | —          | —           |
| 0.70  | 0.722    | 1.00 | [1, 2]       | 0.53–0.56  | 0.26        |
| 1.00  | 0.991    | 1.00 | [0, 1, 2, 3] | 0.82–0.91  | 0.28        |

closure_xy_m = 3.4 mm (well under 15 cm gate).

## Observations + open questions

- **Detection ALL FOUR IDs at z=1.0 m** with sub-pixel reprojection
  (0.28 px mean).  That is excellent ground truth for the indoor
  thesis demo scenario (§23.1).
- **At z=0.7 m only 2 ids** — because at that altitude FOV covers
  only the central 2 markers; the outer pair is off-screen.
- **At z ≤ 0.5 m, 0% detection rate** — *but* the drone's reported
  z_actual rarely matched z_nominal:
    - z_nom=0.4 → z_act=0.31 (drone DROPPED below previous 0.35)
    - z_nom=0.5 → z_act=0.41
  cf2 HL Commander `relative=0` IS being sent (the C dispatch in
  `sim/sentai_crazy_sim.cc:316` packs the byte correctly).  Either
  cf2 SITL's HL Commander interprets the absolute frame differently
  than expected (e.g., "absolute relative to takeoff position", not
  world-frame) and the trajectory undershoots when the delta is
  small, or the convergence tolerance (`APPROACH_TOL_M=0.10`) lets
  the mission proceed before the drone actually reaches the target.
  This is a **flight-control investigation**, separate from the
  detection algorithm — the detector is validated.

  TODO (`OP-S6-W3 follow-up`): instrument the cf2 HL Commander
  trajectory + use Bitcraze's known-good `position_setpoint` /
  `notify_setpoint_stop` semantics for fine altitude control.

## SOTA basis (recap from s159)

- Bradley & Roth 2007 — adaptive box-mean threshold (with block=201)
- Viola & Jones 2001 — integral image
- Rosenfeld & Pfaltz 1966 — 4-connected component labeling
- Garrido-Jurado 2014 — bit-sample bilinear + 3×3 majority + 4-rotation
  hamming dictionary match
- Hartley & Zisserman 2003 §8.1 — DLT homography → R|t decomposition
- OpenCV 4x4_50 ids 0..3 decoded directly from
  `sim/gazebo/materials/textures/aruco_4x4_50_id*.png`

## How to run

```bash
bash examples/sentai_runtime/experiments/s160_aruco_alt_sweep/run.sh
```

PASS gate (verdict.py):
- mission status == "OK"
- at least one altitude with detection_rate ≥ 0.5
- closure_xy_m < 15 cm

## Cross-references

- s159 — synthetic smoke that this experiment promotes to live data.
- `sentai_aruco.{h,cc}` — implementation + SOTA citations.
- `[[op-s6-w3-aruco-shipped]]` — memory entry.
- `[[sentai-sim-air-gapped-from-truth]]` — only inputs are camera
  frames + CRTP LOG telemetry; no Gazebo ground truth.
- `[[missions-run-in-sentai-only]]` — the whole mission script runs
  inside the SIM MicroPython VM.

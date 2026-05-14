# s130 — 4.5Baseline (image-only world-model navigation)

**Purpose**: validate that the drone can localize and navigate between
4 ArUco markers using **only image-derived pose** — multi-marker PnP
for position AND yaw, both image-derived. `cf2.stateEstimate` is captured
for verdict comparison only; never read by the control loop.

This is the **bridge** between L4 (servo skeleton) / s128/s129 (closed-
loop on cf2 EKF or PnP-tvec) and the future Stage 5/11 architecture
where localization comes from `sentai.places.query(scene_descriptor)`
plus `sentai_object_lifter` (no markers, no cf2 input).

ArUco itself remains permanent in the final architecture as the
**takeoff calibration primitive** (set origin from visible markers
before flow takes over) + Stage 6 loop-closure on yaw. s130 validates
that pipeline in isolation, even though all 4 markers happen to be
visible from the compact pattern at z=1 m.

## What changes vs s128 / s129

|                  | s128 L4.1           | s129 L4.2            | **s130 4.5Baseline**         |
|------------------|---------------------|----------------------|------------------------------|
| Inner-loop pose  | `cf2.stateEstimate` | hybrid (cf2 + tvec)  | **image_localize(dets)**     |
| Inner-loop yaw   | `cf2.stateEstimate` | `cf2.stateEstimate`  | **yaw_from_baselines(dets)** |
| Target lookup    | hardcoded host dict | hardcoded host dict  | **sentai.objects.get(id)**   |
| cf2 telemetry    | control + verdict   | control + verdict    | **verdict only**             |
| Marker layout    | ±0.15×±0.10 m       | ±0.15×±0.10 m        | ±0.15×±0.10 m (same)         |

## Pipeline

```
                      Gazebo /downward_cam/image
                                │
                       gz_to_uds_bridge
                                │
                         sentai_sim
                                │
                  SENTAI_DUMP_FRAMES_DIR (PPM every 15)
                                │
        ┌───────────────────────┴───────────────────────┐
        │  mission_l45.py outer loop (per target):       │
        │    1. read fresh PPM                           │
        │    2. detect ArUco (estimate_pose=True)        │
        │    3. filter to known markers (sentai.objects) │
        │    4. if target visible & pixel_err < HANDOFF: │
        │         → IBVS inner loop (s129)               │
        │       else if ≥2 known markers visible:        │
        │         a. yaw  = yaw_from_baselines(dets)     │
        │         b. pose = estimate_drone_world_pose(   │
        │                       dets, drone_yaw=yaw)     │
        │         c. world_delta = target_pose - pose    │
        │         d. body_delta = R_world_to_body(yaw)   │
        │                          @ world_delta         │
        │         e. mc.move + servo.move(intent)        │
        │       else: ABORT (<2 markers, can't localize) │
        │    5. log (image_pose, cf2_pose) for verdict   │
        └────────────────────────────────────────────────┘
                                │
                       4 markers IBVS-centred
```

## Control law

**Outer loop (image-only, between targets)**:
```python
yaw  = yaw_from_baselines_2d(dets, MARKER_WORLD_MAP)  # Procrustes 2D
pose = estimate_drone_world_pose(dets, drone_yaw=yaw) # multi-marker PnP avg
world_delta = (target_world.x - pose.x, target_world.y - pose.y)
body_dx = +cos(yaw) * world_delta.x + sin(yaw) * world_delta.y
body_dy = -sin(yaw) * world_delta.x + cos(yaw) * world_delta.y
mc.move_distance(body_dx * GAIN_K, body_dy * GAIN_K, 0)
```

**Inner loop (IBVS, when target in FOV)**: identical to s129. Reuses
`ibvs_center_on_marker` verbatim.

## Yaw from baselines (2D Procrustes)

For ≥2 markers with known world positions {wᵢ} and observed tvecs in
camera frame {tᵢ}:

1. Body-frame positions: `bᵢ = R_cam_to_body @ tᵢ` (R fixed, known)
2. Center both: `b̄ᵢ = bᵢ - mean(b)`, `w̄ᵢ = wᵢ - mean(w)`
3. Optimal yaw (closed form):
   ```
   yaw = atan2(Σᵢ b̄ᵢ.x·w̄ᵢ.y - b̄ᵢ.y·w̄ᵢ.x,
               Σᵢ b̄ᵢ.x·w̄ᵢ.x + b̄ᵢ.y·w̄ᵢ.y)
   ```

This is the standard 2D rotation-only Procrustes solution (equivalent
to the 2D variant of Kabsch's algorithm). Robust to per-marker tvec
noise via averaging; degrades gracefully with marker count.

## Pass criteria

L4-side FSM (same as s128/s129):
- `servo.status()['actions_ok'] ≥ 4*(≥1 outer-move + ≥1 IBVS-move) + 4 hover + 4 = 16`
- All fault counters == 0
- `last_action == DISARM`, `last_result == 0`
- Final `armed==0`, `flight==GROUND`

s130-specific (image-only validation):
- 4/4 markers IBVS-converged with `px_dist < 30 px` (same as s129 tight)
- **Image-vs-cf2 position agreement**: per-iter
  `||image_pose - cf2_pose||_xy` median < 5 cm, max < 10 cm
- **Image-vs-cf2 yaw agreement**: per-iter
  `|image_yaw - cf2_yaw|` median < 3°, max < 5°
- `n_localize_failures == 0` (every iter has ≥2 known markers visible)

## Seed phase (REPL, minimal)

```python
sentai.objects.add(0, +0.15, +0.10, 0.20)   # id0 NE
sentai.objects.add(1, -0.15, +0.10, 0.20)   # id1 NW
sentai.objects.add(2, -0.15, -0.10, 0.20)   # id2 SW
sentai.objects.add(3, +0.15, -0.10, 0.20)   # id3 SE
```

Inner-loop reads `sentai.objects.get(cid)` once per target to validate
the API; host caches the result locally (no per-iter REPL roundtrip).

## How to run

```bash
bash examples/sentai_runtime/experiments/s130_image_only_nav/run.sh
```

Exit 0 on PASS, 1 on FAIL. Always force-respawns cf2 at origin per
`[[experiments-start-from-origin]]`.

## Logs captured (under `/tmp/s130_image_only_nav/`)

Mirrors s129 + adds one new file:
- `summary.json` — verdict reads this; includes per-iter image-vs-cf2 log
- `journal.txt` — REPL-side servo status snapshots
- `mission.log` — host-side step trace
- `repl.transcript` — raw stdin/stdout dump
- `cf2_telemetry.json` — stateEstimate samples @ 20 ms
- `servo_status.json`, `servo_trace.json` — final servo dumps
- `ibvs_iter_log.json` — per-iter tvec/body_delta/pixel_err (inner loop)
- `image_vs_cf2.json` — **NEW** — per-outer-iter image vs cf2 pose+yaw

## Related

- s128 L4.1Baseline (`s128_l41baseline_seeded/`) — EKF closed-loop, 4 markers.
- s129 L4.2Baseline (`s129_l42baseline_centering/`) — IBVS via PnP-tvec.
- `[[servo-l4-shipped]]` — L4 servo skeleton API.
- `[[objects-l2-shipped]]` — sentai.objects API.
- `[[sentai-sim-journal]]` — structured logging.
- `[[experiments-start-from-origin]]` — reproducibility rule.
- `[[flowbaseline-canonical-config]]` — regression gate (must still PASS).

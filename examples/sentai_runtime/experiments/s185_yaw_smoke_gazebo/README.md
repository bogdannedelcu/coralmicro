# s185 — OP-S10-W19-T7 yaw smoke (Gazebo mission-level)

**WBS:** `OP-S10-W19-T7` (first mission-level integration of the runtime
yaw-anchored drone-pose recovery from `OP-S10-W19-T6b`; promoted from
the "rolled to next session" item #3 in `diary/2026-05-21.md`).

**Goal:** Validate `sentai.markers.get_drone_pose(cf2_yaw)` end-to-end
inside a real Gazebo SITL flight.  The s184 synthetic smoke proved the
picker math; this experiment proves it survives:

1. Real WhyCon detection pipeline (PXP → annulus → flood-fill → PnP)
2. Real cf2 EKF yaw drift (no GT injection — anti-cheat compliant)
3. Frame-to-frame variability of marker subset visible (FOV transient)
4. Slow yaw rotation across the full ±π range so the picker exercises
   every branch (cos>0, cos<0, near-zero R[0,0], etc.)

## Stack

Same as s183 (square 32×32 cm WhyCon pad in `sentai_whycon` world):
- CrazySim cf2 SITL inside `crazysim-garden` distrobox
- `gz_to_uds_bridge` (C++) feeds `/downward_cam/image` to sentai_sim
  over `/tmp/sentai_cam.sock` — camera frames ONLY per
  [[sentai-sim-air-gapped-from-truth]]
- `sentai_sim` runs `mission_s185.py` directly (per
  [[missions-run-in-sentai-only]]); no host-side mission code
- `gt_recorder.py` records `/world/sentai_whycon/dynamic_pose` to JSONL
  host-side for post-mortem only

## Mission

1. `markers.init('whycon')` + intrinsics + extrinsics from cf2 SDF
2. `markers.set_marker_world(MARKER_WORLD)` — 6-marker square pad
   (verbatim copy from s183)
3. crazy.init + arm + takeoff(Z_HOLD=0.78)
4. hl_stop + low-level hover-pin (s174 pattern)
5. **Yaw sweep**: slowly rotate via `hover(0, 0, YAW_RATE, Z_HOLD)`
   for 36 s at yaw_rate ≈ 0.175 rad/s (10°/s) → ~360° total
6. Per tick (10 Hz):
   - Read `cf2_pose = (x, y, z, yaw)` via CRTP LOG
   - `markers.detect_from_camera()` → n_dets
   - If n_dets ≥ 3: call `markers.get_drone_pose_tuple(cf2_yaw)`
   - Journal: `(t, cf2, n_dets, drone_pose_estimate, flips)`
7. Stop yaw + land + disarm

## Pass criteria (provisional)

- Yaw MAE (recovered vs cf2 EKF yaw) ≤ 5° at hover
- Mirror flip rate (`flip_x != 0 || flip_y != 0`) < 1% across the run
- Drone position recovered within ± 3 cm of cf2 EKF position when
  n_dets ≥ 4 (proves picker also recovers pos correctly under rotation)

These are first-cut numbers — the s183 host-side picker reached 1-2 mm
on a stationary hover; an in-flight rotation should be ≤ 1 cm on pos.

## Anti-cheat note

Per `sim/ANTI_CHEAT.md`: `sentai_sim` receives only camera frames +
CRTP LOG telemetry.  `gt_recorder.py` writes to a host JSONL consumed
ONLY by `verdict.py` post-mortem.  `cf2_yaw` fed into the picker IS
the cf2 EKF yaw via CRTP — independent of any marker observation, so
it's a valid anchor (the anchor MUST be derived from a source other
than the markers themselves, per [[yaw-anchor-mirror-picker]]).

## How to run

```bash
bash examples/sentai_runtime/experiments/s185_yaw_smoke_gazebo/run.sh
```

Outputs deposited in this folder:
- `journal.txt`         — per-tick mission log
- `summary.json`        — high-level mission outcome
- `cf2_gt.jsonl`        — Gazebo GT (drone + 6 marker poses)
- `s185_yaw_paired.csv` — verdict's paired (cf2_yaw, recovered_yaw) table
- `s185_yaw_plot.png`   — verdict's yaw error plot

## References

- `experiments/s183_whycon_square_baseline/` — pad geometry + intrinsics
- `experiments/s184_drone_pose_kabsch/` — synthetic picker smoke
- Memory: `feedback_yaw_anchor_mirror_picker.md`

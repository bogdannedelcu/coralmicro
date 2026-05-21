# s184 -- OP-S10-W19-T6b drone-pose Kabsch + yaw-anchor smoke

**WBS:** `OP-S10-W19-T6b` (runtime C port of the host-side picker
shipped under T6a in `s183_whycon_square_baseline`).

**Goal:** validate `sentai.markers.set_marker_world` /
`sentai.markers.get_drone_pose` end-to-end on synthetic body-frame
observations.  Six test cases drive every code path of the C
implementation in `sentai_markers.cc` (full SE(3) Kabsch +
permutation assignment search + Z-plane reflection + yaw-anchored
X/Y mirror flip).

## Pass criteria

| # | Scenario                       | Validates                          | Tolerance |
|---|---------------------------------|-------------------------------------|-----------|
| T1 | Identity (origin, yaw=0)        | Bare Kabsch fit                     | < 1 mm    |
| T2 | Off-centre hover (yaw=0)         | Translation recovery                | < 1 mm    |
| T3 | Non-zero yaw                    | Yaw extraction from R               | < 0.5 deg |
| T4 | 180-Z mirrored observations     | Yaw-anchor X/Y flip                 | < 1 mm    |
| T5 | Drone BELOW marker plane         | Z-plane reflection                  | < 2 mm    |
| T6 | Scrambled observation order     | Permutation assignment search       | < 1 mm    |

`get_drone_pose_tuple` returns
`(x, y, z, yaw_rad, res_max, n_used, flip_x, flip_y, flip_z)`;
the driver checks all components.

## Anti-cheat note

s184 is pure synthetic SIM math; the body-frame observations are
computed analytically in MicroPython from a known drone pose, then
injected via `sentai.markers.test_inject_obs` (the only test-API
in `sentai_markers.cc`).  No Gazebo, no camera, no ground-truth
hookup -- the entire pipeline under test is the C-side recovery
code, with no possibility of cheating via direct world-state read.

## How to run

```bash
bash examples/sentai_runtime/experiments/s184_drone_pose_kabsch/run.sh
```

The script stages the MP driver into `build-sim/sentai_fs_root/`,
imports it via `sentai_sim` over stdin, and pipes the output through
`verdict.py`.

## Method

1. `sentai.markers.init("whycon")` selects the WhyCon backend
   (the picker is backend-agnostic but a backend must be active).
2. `set_marker_world(packed_xyz)` registers the s183 6-marker
   square pad as known world geometry.
3. For each test case, the driver:
   - computes body-frame tvecs analytically from a known drone
     pose via `R_yaw^T @ (m_W - drone_W)`
   - optionally mutates them (180-Z rotation for T4, drone-below
     for T5, scramble for T6)
   - injects via `test_inject_obs(packed_tvecs)`
   - calls `get_drone_pose_tuple(cf2_yaw=true_yaw)`
   - compares recovered pose against ground truth.

## Reference implementations

- C runtime: `examples/sentai_runtime/sentai_markers.cc:set_marker_world..get_drone_pose`
- Algorithm: see memory `feedback_yaw_anchor_mirror_picker.md` and
  host-side Python in
  `experiments/s183_whycon_square_baseline/verdict_sota.py:230-468`.

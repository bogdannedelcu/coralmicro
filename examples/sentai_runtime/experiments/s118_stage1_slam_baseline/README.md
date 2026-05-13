# s118 Stage 1.A — `sentai.slam` baseline on SIM

## What this proves

The existing 2D EKF-SLAM module (`sentai.slam` from
`examples/sentai_runtime/modsentai_slam.c`) is now alive on the
**SIM** target too, foundation for upcoming 3D extensions (Stage 1.B+).

## Changes shipped (this slice)

| File | Change |
|---|---|
| `sim/modsentai_sim.c` | `#include`'s `modsentai_slam.c`; registers `MP_QSTR_slam`. Renamed 3 local MP wrappers (`sentai_fs_write/read/size` → `mp_sentai_fs_*`) to avoid C-side name collision with slam's `extern int sentai_fs_*` declarations. |
| `sim/sim_fs_c_api.c` | NEW — C-side bridge for `sentai_fs_write/read/size` (ARM has these in `modsentai_hal.cc`; SIM needs an equivalent for slam.save/load to work). |
| `sim/CMakeLists.txt` | adds `sim_fs_c_api.c` source. |
| `examples/sentai_runtime/modsentai_slam.c` | UNCHANGED. |

## Baseline test (this dir's `test_slam_2d_baseline.py`)

```
=INIT 0
=POSE0 (0.0, 0.0, 0.0)
=UPDATE_N 1
=LANDMARKS_AFTER_FIRST [{'seen': 1, 'id': 0, 'x': 2.0, 'class_id': 1, 'y': 0.0}]
=POSE_AFTER_MOVE (0.5, 0.0, 0.0)
=UPDATE2_N 1
=INFO {'landmarks': 1, 'initialized': True, ...}
=SAVE_RC 0
=POSE_AFTER_CLEAR (0.0, 0.0, 0.0)
=LOAD_RC 0
=POSE_AFTER_LOAD (0.5, 0.0, -0.01094)
=DONE
[test] PASS — slam.init+update+predict+save+load all work on SIM
```

Round-trip via FileX path (`/slam/baseline_test.bin`) preserves
landmark count + pose (with small numerical noise from EKF Kalman
gain at load).

## What's NOT done yet (next stages)

- Stage 1.B — `set_class_prior(class_id, real_size_m)` table
- Stage 1.C — `update_3d(detections, altitude)` with pseudo-depth from class prior
- Stage 1.D — `anchor_update(marker_world_pos, observed_bearing)` for loop closure

## Run

```bash
cmake --build build-sim --target sentai_sim
python3 examples/sentai_runtime/experiments/s118_stage1_slam_baseline/test_slam_2d_baseline.py
# expect: "[test] PASS — slam.init+update+predict+save+load all work on SIM"
```

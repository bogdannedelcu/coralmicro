# s166 — FlowBaseline with Gazebo GT comparison

**WBS**: OP-S8-W1-T7 + (toward) T8.  See `diary/2026-05-17.md`
(originally `ideas/op_s8_w1_handoff.md`, moved 2026-05-19).

## Why this exists

[[cf2-sitl-cheat-odom-gt]]: the CrazySim `gz-sim-odometry-publisher-system`
plugin was injecting Gazebo ground truth into cf2's EKF at 200 Hz.
Every prior cf2-side drift number — including s127 FlowBaseline's 7.4 cm
canonical — was measuring noise around perfect pose, not real flow/PnP
performance.  The plugin was disabled 2026-05-17.

This experiment is the honest replacement: same mission as s127 (15 s
hover at z=1 m using `sentai.flow` + ArUco VPE forwarder), but with
Gazebo GT recorded **host-side** in parallel and the verdict gating on
GT-based drift instead of cf2's self-reported EKF closure.

## Hard rules enforced by `run.sh`

1. **Force-respawn cf2 at origin** — unconditional `stop.sh` + force-kill,
   no `is_up` guard.  Operator rule: every trial starts from the landing
   place, not from where a prior trial left the drone.
2. **Assert spawn pose** — `gz model -m crazyflie_0 --pose` queried
   after respawn; mission aborts unless `|x|<0.05 AND |y|<0.05 AND z<0.10`.
   (T6 from `diary/2026-05-17.md`.)
3. **GT consumed host-side only** — `gt_recorder.py` runs as a separate
   host process subscribing to `/world/sentai_crazysim/dynamic_pose/info`
   via `gz topic -e`.  Never fed back into cf2 or sentai_sim.
   [[sentai-sim-air-gapped-from-truth]] compliance.
4. **Gazebo GUI mandatory** — `launch_hybrid_cf2.sh` starts both server
   + GUI.  [[gazebo-gui-required]].

## Pass criteria (tentative — first post-cheat run)

```
GT-based dist_max_m  < 0.30
GT-based dist_mean_m < 0.20
```

These are **looser** than the pre-cheat s127 gate (0.15 m on EKF) to
account for the fact that we are no longer measuring noise around a
GT-locked pose.  After 3+ trials we will promote a tightened gate to
the new canonical `[[flowbaseline-canonical-2-no-cheat]]` memory
entry (T8 closure).

## What gets logged

| File                                           | Source             | Content                                  |
|------------------------------------------------|--------------------|------------------------------------------|
| `/tmp/s166_flowbaseline_gt/gt_poses.jsonl`     | `gt_recorder.py`   | Gazebo cf2 pose @ ~200 Hz, host-time     |
| `…/s091_aruco_lowalt/hover_log.json`           | `aruco_hover.py`   | cf2 EKF samples + high-rate `ekf_trace`  |
| `/tmp/s166_flowbaseline_gt/spawn_pose.json`    | `run.sh`           | gz model pose at takeoff (T6 forensics)  |
| `/tmp/s166_flowbaseline_gt/summary.json`       | `verdict.py`       | gate metrics + frame alignment           |
| `/tmp/s166_flowbaseline_gt/plot_xy.png`        | `verdict.py`       | top-down XY: EKF (blue), GT (red)        |
| `/tmp/s166_flowbaseline_gt/plot_err.png`       | `verdict.py`       | per-axis error + dist-from-origin vs t   |

The XY plot uses distinct colours per source:
- **blue** = cf2 EKF belief
- **red** = Gazebo ground truth
- **black dot** = position setpoint (0, 0)
- **grey lines** = per-sample EKF↔GT residuals

## Run

```bash
bash examples/sentai_runtime/experiments/s166_flowbaseline_gt/run.sh
```

Exit 0 = PASS (within gate); exit 1 = FAIL.  Inspect `plot_xy.png`
and `summary.json` either way — for the post-cheat era we are
calibrating, not just gating.

## Load-bearing prerequisites

Same as s127:
- Cheat plugin must remain disabled in `model.sdf.jinja:393`.  Verify
  with `distrobox enter crazysim-garden -- gz topic -l | grep odom`
  — must return nothing.
- CrazySim cf2 firmware on `sentai-flow-sim-support` @ `e4374251`.
- `crazyflie-simulation` submodule at `aeb7ee6` + `world_no_wind.patch`
  from s127.
- coralmicro on `integration/from-180bbb5f` or descendant.
- `aruco_hover.py` patched with `t_wall` timestamps + high-rate
  `ekf_trace` (this session, 2026-05-18).

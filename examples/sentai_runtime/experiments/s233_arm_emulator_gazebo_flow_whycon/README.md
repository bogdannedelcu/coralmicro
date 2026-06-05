# S233 ARM Emulator Gazebo Flow + WhyCon

Goal: feed live Gazebo `/downward_cam/image` frames into the ARM emulator,
publish them through the shared `sentai_runtime` camera backend, then verify
that `PrepTask`, `FlowTask`, and `sentai.markers` run from the same frame path.

Run:

```bash
python3 examples/sentai_runtime/experiments/s233_arm_emulator_gazebo_flow_whycon/run_s233.py
```

Use `--renode-ui` to keep Renode's UART analyzer visible while still writing
the per-iteration artifacts.

Latest verified run:

```bash
python3 examples/sentai_runtime/experiments/s233_arm_emulator_gazebo_flow_whycon/run_s233.py \
  --camera-forward-fps 15 --camera-out-width 640 --camera-out-height 480
```

`iter23_gazebo_flow_whycon` passed FlowBaseline + WhyconBaseline with VGA
camera ingress: `cam_last_rc=921600` (`640*480*3` RGB888), `cam_frames=212`,
`prep_frames=45`, `flow_frames=45`, `marker_hits=2`, `marker_best=7`, and
`crazy_takeoff=0` / `crazy_land=0`.  The verdict requires at least one real
WhyCon hit.

Flow does not process VGA directly.  The runtime boundary is VGA camera frame
publication into the shared camera backend, then PrepTask publishes
`SENTAI_PREP_SLOT_FLOW_GRAY_80x60` for FlowTask.  `iter21` and `iter22`
validated the earlier 320x240 host-downsampled path, with `iter22` also proving
the `--renode-ui` UART analyzer path.

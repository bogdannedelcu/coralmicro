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
  --camera-forward-fps 15 --camera-out-width 640 --camera-out-height 480 \
  --camera-out-format rgb888
```

`iter31_gazebo_flow_whycon` passed FlowBaseline + WhyconBaseline with the
emulator RGB888 fast path: `cam_last_format=0`, `cam_last_rc=921600`
(`640*480*3` RGB888), `cam_frames=644`, `prep_frames=110`,
`flow_frames=102`, `prep_fps_x100=517`, `flow_fps_x100=479`,
`marker_hits=1`, and `crazy_takeoff=0` /
`crazy_land=0`.  The verdict requires at least one real WhyCon hit.

`iter29_gazebo_flow_whycon` validated the same CameraTask bridge in XRGB mode:
`cam_last_format=1`, `cam_last_rc=1228800`, `prep_fps_x100=454`,
`flow_fps_x100=422`, and `pass=True`.  This keeps an ARM-like XRGB path
available while allowing emulator-only RGB input when profiling CPU-emulated
PrepTask.

`iter24_gazebo_flow_whycon` measured the previous guest-side RGB888 publish
path at `publish_ms_sum=37859` for `210` camera frames.  `iter25` moved
RGB888->XRGB8888 conversion to the host relay and wrote directly into a
reserved virtual-camera slot, reducing publish time to `18661 ms` for `546`
camera frames.

`iter26_gazebo_flow_whycon` tried a 1 ms bridge delay after successful camera
publish, but it overfed PrepTask in the emulator and did not reach
`S233_DONE`.  Keep the stable 30 FPS bridge pacing unless the consumer side is
also changed.

Flow does not process VGA directly.  The runtime boundary is VGA camera frame
publication into the shared camera backend, then PrepTask publishes
`SENTAI_PREP_SLOT_FLOW_GRAY_80x60` for FlowTask.  `iter21` and `iter22`
validated the earlier 320x240 host-downsampled path, with `iter22` also proving
the `--renode-ui` UART analyzer path.

The emulator CameraTask is format-aware: Renode only serves a frame when the
guest-reserved slot format matches the host relay payload.  RGB888 avoids the
VGA XRGB conversion in PrepTask; XRGB8888 remains the ARM-like fallback.

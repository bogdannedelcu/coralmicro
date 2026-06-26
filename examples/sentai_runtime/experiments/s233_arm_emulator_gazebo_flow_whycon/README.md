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
  --renode-ui --camera-forward-fps 15 --camera-out-width 640 \
  --camera-out-height 480 --camera-out-format rgb888
```

PXP accelerator validation on 2026-06-05:

- `iter37_gazebo_flow_whycon` was an intentionally aborted prototype run using
  an IronPython `Python.PythonPeripheral` PXP bridge.  The MMIO contract worked,
  but a single VGA RGB888 -> Y8 resize took about `2.7-3.5 s`, making it slower
  than the guest scalar fallback.
- `iter39_gazebo_flow_whycon` validated the C# ad-hoc Renode peripheral with
  bulk `ReadBytes`/`WriteBytes`: PXP transfers dropped to `1-3 ms`, but the run
  failed WhyCon because the first Y8 implementation converted averaged RGB to
  luminance instead of averaging per-pixel luminance like the scalar fallback.
- `iter40_gazebo_flow_whycon` is the corrected headless PASS:
  `pxp_accel_calls=1078`, `pxp_accel_ok=1078`,
  `pxp_accel_fallback=0`, `prep_frames=461`,
  `flow_frames=438`, `prep_fps_x100=2706`,
  `flow_fps_x100=2571`, `marker_hits=1`, and `marker_best=3`.
- `iter41_gazebo_flow_whycon` is the corrected UI PASS with Gazebo GUI and
  Renode UI visible: `pxp_accel_calls=1090`, `pxp_accel_ok=1090`,
  `pxp_accel_fallback=0`, `prep_frames=467`, `flow_frames=443`,
  `prep_fps_x100=2718`, `flow_fps_x100=2578`, `marker_hits=2`, and
  `marker_best=7`.
- `iter42_gazebo_flow_whycon` is the post-review bridge-contract refactor
  PASS.  The guest-side PXP shim now mirrors the emulator TPU
  `SendParameters` bridge style: `STATUS/COMMAND/SEQ` plus semantic transform
  arguments, with scalar fallback hidden behind the public
  `sentai_pxp_transform()` boundary.  Results stayed stable:
  `pxp_accel_calls=1092`, `pxp_accel_ok=1092`, `pxp_accel_fallback=0`,
  `prep_fps_x100=2712`, `flow_fps_x100=2567`, `marker_hits=1`, and
  `marker_best=7`.

The current emulator-only PXP boundary is `SentaiPxpAccelerator.cs` at
`0x40902C00`, called only through the `sentai_pxp_*` shim.  It is a
command-style semantic accelerator for resize/format work, not a full NXP PXP
model and not a WhyCon/flow shortcut.

Fresh-PC UI validation on 2026-06-05:

- `iter35_gazebo_flow_whycon` passed with Gazebo GUI and Renode UI at
  `--camera-forward-fps 10`, VGA `rgb888`: `cam_frames=430`,
  `bridge_seen=504`, `bridge_served=504`, `prep_frames=108`,
  `flow_frames=99`, `prep_fps_x100=521`, `flow_fps_x100=478`,
  `marker_hits=1`, and clean cf2 command results.
- `iter36_gazebo_flow_whycon` passed with Gazebo GUI and Renode UI at
  `--camera-forward-fps 15`, VGA `rgb888`: `cam_frames=529`,
  `bridge_seen=625`, `bridge_served=625`, `prep_frames=108`,
  `flow_frames=98`, `prep_fps_x100=521`, `flow_fps_x100=473`,
  `marker_hits=1`, and clean cf2 command results.
- `iter33` and `iter34` are setup failures from attempting to launch the
  distrobox-backed Gazebo stack from the restricted sandbox.  The valid UI
  runs must be launched on the host so podman/distrobox can access
  `/run/user/<uid>/libpod`, the real display, and the GUI bridges.
- Raising forwarded camera input from 10 FPS to 15 FPS increased the number of
  camera frames observed by the bridge, but did not change the guest
  PrepTask/FlowTask ceiling.  On this PC the current VGA `rgb888` UI path is
  still bounded near 5.2 FPS prep and 4.7-4.8 FPS flow by guest-side
  camera-to-prep work.

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

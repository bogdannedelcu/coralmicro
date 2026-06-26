# TD-S10-B9 - Verify SentAI Namespaces In ARM Emulator

## Objective

Implement the A9 plan as a sequence of small emulator experiments that verify
the `sentai.*` runtime surface namespace by namespace.

B9 should not jump straight to a large "full mission" target.  It should make
each namespace visible, callable, and either verified or explicitly marked as
stubbed/deferred in the ARM emulator.  The final B9 result should be a clear
matrix that says what works in Renode, what is bridged to host hardware, what is
an emulator-safe stub, and what remains board-only.

Priority update: `sentai.crazy` moves ahead of camera/pipeline validation.
Once the emulator can command `cf2` in CrazySim/Gazebo, it becomes the test
bench for camera frames, telemetry, pose, flow, and later closed-loop missions.
This is more valuable than validating every pure algorithm namespace first.

## Rules

- Use the shared `examples/sentai_runtime` implementation whenever possible.
- Emulator-only code belongs at board/peripheral/host-bridge boundaries, not in
  duplicate namespace implementations.
- Every test gets an `sNNN` experiment folder and keeps logs/artifacts there.
- Use `sentai.fr` and `sentai.fs` for mission logs where possible.
- MicroPython starts/stops/configures/observes; persistent C/C++ tasks do the
  compute and I/O work.
- Do not revive `sentai.diag`.
- Do not use POSIX SIM as the proof path for B9 concurrency.
- Host wall-clock timing is required for physical USB Coral bridge timings.
- A namespace can PASS as `stubbed` only if the stub is deliberate, documented,
  and has stable behavior from MicroPython.

## Experiment Sequence

### s216 - Namespace Inventory Baseline

Purpose: measure the current emulator-visible `sentai` surface before changing
the root module.

Actions:

- boot the current emulator REPL target;
- run a mission from emulator FS that imports `sentai`;
- record `dir(sentai)` and `dir(sentai.<namespace>)` for every visible child;
- write inventory to FileX and copy it to the `s216` artifact folder;
- compare with `examples/sentai_runtime/modsentai.c`.

PASS:

- inventory completes without REPL lockup;
- output classifies every namespace as `present`, `missing`, `emu-subset`, or
  `unexpected`;
- `sentai.help()` behavior is recorded.

Status:

- `s216/iter05_namespace_inventory` PASS with Renode UI/analyzer enabled.
- `s216/iter06_namespace_inventory` PASS after unifying `sentai.tpu` onto the
  shared `examples/sentai_runtime/bindings/modsentai_tpu.c` binding.  Root
  export remains 14 shared names and `sentai.tpu` inventories the shared API
  plus the deliberate `SENTAI_EMU_TPU_HOST_BRIDGE` helper calls
  (`load_image_mem`, `image_mem_size`, `stats`, `start`, `stop`, `fps`,
  `fps_invoke`).  ELF symbol check shows `sentai_tpu_module` present and no
  linked `emu_tpu_module`.
- `s216/iter07_namespace_inventory` PASS after unifying `sentai.pipeline` onto
  the shared `examples/sentai_runtime/bindings/modsentai_pipeline.c` binding.
  Inventory build #1520 reports `sentai.pipeline` present with the full shared
  API (`start`, `get`, `once`, `detections`, `prep_*`, tracker config/events,
  slot routing, calibration/probe helpers, and diagnostic knobs).  The
  emulator-specific code is now limited to C backends: TPU-host targets use the
  bridge-backed `sentai_tpu_detect`, camera-less targets link
  `sentai_emu_pipeline_unavailable_backend.c`, and the Gazebo flow/WhyCon
  target links the real `detection_task.cc` plus `sentai_tracker.cc`.
- Wide inventory profile exports 14 shared names:
  `version`, `verbose`, `help`, `debug`, `console`, `run`, `io`, `rtos`,
  `tpu`, `fs`, `crazy`, `pipeline`, `fr`, `sys`.
- Missing shared names in the wide profile remain: `camera`, `usb`, `uart`,
  `mesh`, `link`, `imu`, `mic`, `sleep`, `flow`, algorithm namespaces,
  `servo`, `object_lifter`, `calib`, `markers`, `safety`, `explore`, `tfl`.
- The wide profile intentionally excludes the S228 hardware stubs to stay under
  the RT1176 RAM linker `m_text` limit while still keeping TPU/Pipeline/Crazy
  visible in one inventory run.

### s217 - Shared Root Minimal Build

Purpose: start replacing emulator-specific root modules with the shared
`modsentai.c` root.

Actions:

- build an emulator target with shared `modsentai.c`;
- include only the minimum owner namespaces needed to boot: `fs`, `fr`, `rtos`,
  `sys`, `io`, and `tpu`;
- provide explicit `SENTAI_ARM_EMU` stubs for hardware-only calls;
- regenerate QSTR cleanly if needed;
- rerun the s216 inventory mission.

PASS:

- `import sentai`, `sentai.version()`, `sentai.help()`, and `dir(sentai)` work;
- unsupported hardware functions fail with clear errors or documented stub
  values;
- the physical ARM build is not affected.

### s218 - Core Runtime Namespaces

Purpose: validate low-risk runtime owners before camera/TPU pipelines.

Namespaces:

- `sentai.fs`;
- `sentai.fr`;
- `sentai.rtos`;
- `sentai.sys`;
- `sentai.io`;
- top-level `sentai.version`, `verbose`, `help`, `debug`, `console`, `run`.

Tests:

- filesystem create/read/append/list/remove/sync;
- FlightRecorder event/scalar log and flush to FS;
- `sleep_ms`, `ticks_ms`, heap/task stats where available;
- `sentai.run("/mission.py")` from FS;
- `sentai.console()` documented behavior under LPUART6;
- `sentai.io` LED/GPIO calls return stable emulator-safe values.

PASS:

- mission can reboot/re-run without corrupting FS;
- FR log appears in the experiment folder;
- no stdout flood or stuck Renode/helper processes.

Status:

- `s218/iter01_core_namespaces` PASS with Renode UI/analyzer enabled.
- 25 checks passed, 0 failed.
- Verified top-level `version`, `verbose`, `debug`, `console`, `run`;
  `sentai.fs`; `sentai.fr`; `sentai.rtos`; `sentai.io`; `sentai.sys`.
- Verified FileX-backed mission execution via `sentai.run('/b9/s218_child.py')`
  and FlightRecorder event/scalar files flushed to FileX.

### s219 - Crazyflie CRTP Bridge To cf2

Purpose: connect emulator `sentai.crazy` to CrazySim `cf2` before the rest of
the camera/pipeline work, so Gazebo becomes the active integration bench.

Background:

- ARM runtime uses `sentai.crazy` over `CRTP -> CPX -> UART`.
- SIM used `sim/sentai_crazy_crtp_udp_bridge.cc`, which speaks direct
  CRTP-over-UDP to cf2 SITL at `127.0.0.1:19850`.
- Emulator should keep the shared `sentai.crazy` binding and the ARM semantic
  surface.  The emulator-specific part should be a transport bridge, not a new
  Python API.

Preferred bridge:

- guest `sentai_crazy.cc` continues to emit CPX-over-UART frames;
- Renode exposes a second UART/PTY/socket that is separate from the REPL;
- a host bridge reads CPX frames, extracts CRTP, forwards direct CRTP datagrams
  to cf2 UDP `19850`, and wraps inbound CRTP datagrams back into CPX frames;
- the host bridge also handles/synthesizes CPX CTS so guest TX does not block.

Fallback bridge:

- if CPX framing blocks bring-up, add an emulator-only backend under the
  `sentai_crazy.h` C ABI that forwards CRTP directly to the host bridge, using
  `sim/sentai_crazy_crtp_udp_bridge.cc` only as protocol reference;
- document it as `SENTAI_ARM_EMU` transport substitution, not as production
  behavior.

Namespaces:

- `sentai.crazy`;
- `sentai.uart`;
- `sentai.fr`;
- top-level `sentai.console`, because REPL and drone transport must stay
  separate.

Tests:

- start CrazySim/cf2 with `sim/scripts/respawn_sitl.sh`;
- boot emulator with REPL on LPUART6 and Crazy bridge on a separate transport;
- `sentai.crazy.init()` starts RX/CMD tasks without stealing the REPL;
- `sentai.crazy.ping()` reaches cf2;
- `sentai.crazy.arm()` / `disarm()` return success;
- `sentai.crazy.takeoff()` / `land()` smoke in Gazebo, initially at safe
  height/duration;
- telemetry calls such as `altitude`, `attitude`, `velocity`, `canfly`, and
  `is_flying` return plausible values;
- all bridge RX/TX counters are logged through `sentai.fr`.

PASS:

- cf2 UDP `19850` is reached from guest code through `sentai.crazy`;
- REPL remains usable while the Crazy bridge is running;
- takeoff/land or a conservative arm/ping/telemetry-only smoke is repeatable;
- cleanup stops Renode, bridge, Gazebo/cf2 helper processes.

Implementation note:

- `emu/host/sentai_crazy_cpx_udp_bridge.py` now provides the host-side
  CPX UART `<->` cf2 CRTP UDP translation and CTS handling.
- `emu/host/test_sentai_crazy_cpx_udp_bridge.py` verifies CPX route packing,
  CRC, CTS, guest-to-cf2 CRTP extraction, and cf2-to-guest CPX wrapping.
- `emu/sentai_emu_crazy_serial_bridge.cc` adds a `SENTAI_ARM_EMU` guest serial
  ABI bridge for `sentai_uart_serial_*`.
- `emu/renode/crazy_cpx_udp_mmio_bridge.py` adds the Renode MMIO endpoint that
  forwards those serial bytes to cf2 UDP.  This is the low-friction fallback
  route and keeps `sentai.crazy` itself shared.
- `sentai_emu_crazy_serial_bridge_obj` compile-checks the guest-side serial
  bridge without creating a standalone mission target yet.
- `sentai_emu_crazy_ping_smoke` now links the shared `sentai_crazy.cc`
  implementation with the emulator serial bridge and runs
  `sentai_crazy_init(576000)` / `sentai_crazy_ping(1500)` against cf2.
- `examples/sentai_runtime/experiments/s219_arm_emulator_crazy_bridge/run_s219.py`
  reproduces the full smoke and keeps artifacts in `s219/iterNN_*`.
- First archived run:
  `s219/iter01_renode_crazy_cf2_bridge/verdict_s219.json`.
  It passed with boot state `0x0B00`, `init_rc=0`, `ping_ms=5`,
  serial counters `tx=54 rx=22`, and bridge traffic in both directions:
  one guest-to-cf2 CRTP datagram and one cf2-to-guest CRTP datagram.
- Re-run with the B9 default world switched to WhyCon/circle markers:
  `s219/iter02_renode_crazy_cf2_bridge/verdict_s219.json`.
  It passed against `sentai_whycon_small` with boot state `0x0B00`,
  `init_rc=0`, `ping_ms=1`, serial counters `tx=54 rx=22`, bridge traffic in
  both directions, and no leftover cf2/Gazebo/Renode processes after cleanup.
- The runner now defaults to `sentai_whycon_small`, runs Renode, copies the
  cf2/Renode/UART bridge logs, and stops the cf2/Gazebo helper processes
  unless `--keep-sitl` is passed.
- Interactive Renode runs use `--renode-ui`, which selects the matching
  `_ui.resc` script and opens `showAnalyzer lpuart6` so UART output is visible
  live while the file backend still records experiment artifacts.
- MicroPython `sentai.crazy` smoke now passes with Renode UI/analyzer enabled:
  `s219/iter03_renode_crazy_mp_cf2_bridge/verdict_s219.json`.
  It invoked `sentai.crazy.init()` / `sentai.crazy.ping()` /
  `sentai.crazy.stop()` from the emulator MicroPython target against cf2 in
  `sentai_whycon_small`, with `CRAZY_MP_INIT=0`, `CRAZY_MP_PING_MS=1`,
  `CRAZY_MP_STOP=0`, `serial_tx=56`, `serial_rx=22`, and bridge traffic in both
  directions.
- `sentai_crazysim` is acceptable only for CRTP-only bring-up.  For camera,
  markers, flow, and closed-loop B9 work, use the WhyCon/circle-marker world
  used by the earlier B3/B4/B5 missions.

### s229 - Gazebo WhyCon Calib Precursor

Purpose: answer whether the ARM emulator can run a Gazebo-backed mission that
looks like the first step of calibration: boot SentAI in Renode, command cf2 in
the WhyCon marker world, and keep all artifacts in an `sNNN` experiment folder.

What it does:

- starts CrazySim/cf2 in `sentai_whycon_small`;
- boots `sentai_emu_crazy_calib_precursor_repl` in Renode with UART analyzer
  enabled when `--renode-ui` is used;
- runs MicroPython through the shared `sentai.crazy` binding:
  `init`, `ping`, `fly(0.35, 1800, 2200, 2200)`, `stop`;
- probes the current shared `sentai.calib` boundary
  (`get_R_cam_to_body`, `get_cam_offset_B`, `is_calibrated`);
- logs through `sentai.fr` and writes a small `/b9/s229_status.txt` marker in
  FileX;
- copies Renode, UART, bridge, SITL, command, and verdict logs into
  `examples/sentai_runtime/experiments/s229_arm_emulator_gazebo_calib_precursor/iterNN_*`.

Status:

- `s229/iter01_gazebo_whycon_calib_precursor` PASS.
- Results: `CALIB_GZ_INIT=0`, `CALIB_GZ_PING_MS=2`,
  prior run `CALIB_GZ_CALIB_STUB 9 (0.0, 0.0, 0.0) False`,
  `CALIB_GZ_FLY_RC=0`, `CALIB_GZ_STOP=0`.  New builds print
  `CALIB_GZ_CALIB_SHARED ...` from the shared runtime binding.
- Bridge counters: `guest_to_udp=5`, `udp_to_guest=3`,
  `serial_tx=131`, `serial_rx=50`.
- Cleanup check found no leftover Renode/Gazebo/cf2/bridge processes.

Limitations:

- This is not full visual calibration.  `sentai.calib` now uses the shared
  runtime binding in this target; camera/marker-dependent workers still need a
  live visual backend before the full bringup can run.
- `CALIB_GZ_ALT_AFTER=-999.0` is diagnostic only.  The current
  `sentai.crazy.altitude()` path expects the SentAI deck telemetry channel,
  while this bridge is validating CRTP/cf2 traffic.
- Full calibration needs `sentai.camera`, `sentai.markers`/WhyCon, and a live
  calibration worker backend in the emulator/Gazebo profile.

Remaining s219 work:

- Keep extending the MicroPython namespace target beyond `fly()` into
  command-by-command `arm`, `takeoff`, `hold`, `land`, and `disarm` with more
  detailed timing/bridge counters.
- Add a cf2 standard LOG bridge or a telemetry adapter if we want
  `sentai.crazy.altitude`, `attitude`, and `velocity` gates to be hard PASS
  criteria in emulator.

### s220 - Gazebo Camera Bench Hook

Purpose: after `sentai.crazy` works, make the existing Gazebo/cf2 launch stack
usable as the emulator's camera and telemetry bench.

Namespaces:

- `sentai.crazy`;
- `sentai.camera`;
- `sentai.fr`.

Tests:

- launch `sim/scripts/launch_sim.sh sentai_whycon_small <experiment-dir>`;
- verify cf2 UDP readiness and camera frame readiness from the same experiment
  runner;
- bridge Gazebo camera frames into the emulator camera provider boundary;
- record cf2 telemetry and camera frame counters in the same artifact folder;
- keep ground-truth logs post-mortem only, per anti-cheat rule.

PASS:

- emulator receives camera frames while `sentai.crazy` can still talk to cf2;
- frame source, cameraId, cf2 telemetry, and bridge counters are logged;
- no Gazebo/cf2 process is left behind after cleanup.

### s221 - Virtual Camera Provider

Purpose: build the camera input layer in `sentai_runtime`, not in SIM.

Namespaces:

- `sentai.camera`;
- internal CameraTask/provider boundary.

Tests:

- preload one frame from FileX into memory and publish at fixed FPS;
- preload multiple frames from FileX and cycle them;
- preserve cameraId and frame sequence;
- expose minimal camera stats through `sentai.camera`;
- verify that frame serving does not depend on MicroPython polling.

PASS:

- virtual camera publishes deterministic frames at configured FPS;
- frames are visible to the same buffer/notification model used by PrepTask;
- no hot-loop filesystem reads after preload.

Status:

- `s230/iter03_camera_markers` PASS validates the first production-runtime
  emulator camera provider path.
- A WhyCon BMP is staged into the FileX/LevelX NAND image, selected through
  `sentai.camera.select(-1, "/images/whycon_640x480.bmp")`, and served through
  the shared `sentai_virtual_camera` + `sentai_camera_frame_backend` path.
- Emulator-only code is constrained to `emu/sentai_emu_camera_runtime_bridge.cc`
  as the board/filesystem/HAL boundary.  The MicroPython API comes from the
  shared `examples/sentai_runtime/bindings/modsentai_camera.c` binding.
- The gray fallback now downsamples arbitrary XRGB camera frames to the fixed
  marker gray size and returns the raw camera slot after conversion, preventing
  virtual-camera slot exhaustion in repeated marker runs.

### s222 - PrepTask Input Path

Purpose: prove camera-to-prep behavior before Flow or TPU.

Namespaces:

- `sentai.camera`;
- `sentai.pipeline` prep subset.

Tests:

- `VirtualCameraTask -> PrepTask -> prep slots`;
- validate RGB/XRGB input conversion, resize, quant/input buffer ownership;
- record `prep_fps`, `prep_stats`, frame sequence, cameraId;
- run single-frame repeated and multi-frame sequence modes.

PASS:

- PrepTask consumes every available frame or records deliberate drops;
- prep slots update monotonically;
- FPS and counters are logged through FR.

Status:

- `s230/iter03_camera_markers` PASS exercises `sentai.camera.prep_once()` on
  the staged 640x480 WhyCon BMP before marker detection.
- Large camera/prep backing buffers required by this target are placed in SDRAM
  for the ARM-emulator profile so the RAM linker image stays bootable.

### s223 - Flow Namespace Mission

Purpose: verify production `FlowTask` through `sentai.flow`, not only harnesses.

Namespaces:

- `sentai.flow`;
- `sentai.camera`;
- prep slot producer.

Tests:

- start FlowTask from MicroPython;
- feed B7/B8 shifted cat frames with unequal X/Y motion;
- read offsets and confidence through `sentai.flow`;
- measure Flow-only FPS;
- test `start`, `stop`, `read/read_tuple`, stats, gray stretch, detail score,
  and search mode APIs where present.

PASS:

- offsets match the expected signed X/Y direction within documented tolerance;
- FlowTask stops cleanly and can be restarted;
- no dependency on InferTask or TPU.

### s224 - TPU Namespace Mission

Purpose: align `sentai.tpu` with the B8 physical Coral `Send*` bridge.

Namespaces:

- `sentai.tpu`;
- `sentai.fs`;
- `sentai.fr`.

Tests:

- load COCO EdgeTPU model from FileX once;
- preload cat image from FileX into guest memory;
- invoke repeatedly through guest `EdgeTpuManager -> TpuDriver::Send*`;
- collect first invoke, steady invoke, transfer, output, and host wall-clock
  bridge timings;
- verify output tensors and detection extraction.

PASS:

- repeated invoke does not reload weights;
- physical USB Coral is used through the low-level bridge;
- FPS is stable across at least two consecutive runs.

Status:

- `sentai.tpu` no longer uses a separate EMU MicroPython module.  The ARM
  emulator root table points at shared `sentai_tpu_module` from
  `modsentai_tpu.c`; emulator-specific behavior is now behind the C backend in
  `emu/sentai_emu_tpu_bridge_module.c`, which reads FileX assets and forwards
  semantic model/image/invoke/session commands to the Renode/host physical
  Coral bridge.
- Build validation on 2026-06-11: `sentai_emu_tpu_cat_repl`
  build #1499, `sentai_emu_namespace_inventory` build #1500, and TPU FPS/mem
  targets build #1501-#1505 all pass.
- Runtime namespace validation: `s216/iter06_namespace_inventory` PASS,
  build #1507, `sentai.tpu` present.
- Runtime TPU-cat validation still needs follow-up: `s213/iter112_renode_tpu_cat_repl_filex_physical_coral`
  built #1506 but timed out in Renode before any UART banner, so it is not a
  Coral/invoke validation yet.
- Fresh-PC TPU-cat follow-up on 2026-06-11:
  `s213/iter113_renode_tpu_cat_repl_filex_physical_coral` reproduced the
  apparent timeout, but short Renode diagnostics in the same iter showed the
  firmware did reach `boot_state=0x500`; `/mission.py` was missing from FileX
  and the one-shot target then fell into the interactive REPL stdin spin.
  `s213/iter114_renode_fs_stage_assets_filex_levelx_nand` restaged the model,
  cat image, and mission into the emulator NAND image (`stage_files=3`,
  `stage_bytes=8014902`, `fx_errors=0`).  `s213/iter116_renode_fs_asset_check_filex_levelx_nand`
  PASS confirmed the staged assets are visible from MicroPython after enabling
  `SENTAI_EMU_AUTORUN_HALT_AFTER` on the older FS/TPU autorun targets.
- `s213/iter117_renode_tpu_cat_repl_filex_physical_coral` then reached
  `MISSION_TPU_CAT_DONE`, but the host bridge could not find
  `build-sim/sim/tpu_posix_invoke_smoke`.  The local `build-sim` host tool
  was rebuilt after fixing SIM linkage for the shared TPU counters
  (`tpu_posix_invoke_smoke.cc` declarations moved out of the anonymous
  namespace; `g_sentai_tpu_multi_ep_routing` moved to global C linkage).
- Current TPU-cat blocker is host USB permission, not firmware or FileX:
  `s213/iter118_renode_tpu_cat_repl_filex_physical_coral` streams the model
  and image to the host bridge, launches the POSIX smoke, and exits with
  `TPU_INVOKE -3` / host smoke `exit=4`.  Direct host execution reports
  `FAIL open_posix status=1`; `lsusb` sees the Coral as `1a6e:089a`, but
  `/dev/bus/usb/002/003` is owned by `root:root` with no user write access.
  `sudo -n chmod a+rw /dev/bus/usb/002/003` could not run because interactive
  authentication is required.

### s225 - Pipeline Namespace Mission

Purpose: verify the real `sentai.pipeline` API in emulator mode.

Namespaces:

- `sentai.pipeline`;
- `sentai.camera`;
- `sentai.tpu`;
- `sentai.fs`;
- `sentai.fr`.

Tests:

- persistent PrepTask + persistent InferTask;
- `pipeline.start`, `stop`, `get`, `get_ex`, `detections`, `save`, `stats`,
  `infer_stats`, `frame_count`, `target_fps`;
- COCO cat detections through physical Coral;
- one-shot mode if supported by the existing API.

PASS:

- detections are produced from virtual camera frames;
- `pipeline.stop` does not delete/recreate task topology incorrectly;
- restart works in the same emulator boot.

Status:

- The `sentai.pipeline` MicroPython surface is now shared.  The ARM emulator
  root table points at `sentai_pipeline_module` from
  `examples/sentai_runtime/bindings/modsentai_pipeline.c`; no active
  `emu_pipeline_module` remains in the root surface.
- `s216/iter07_namespace_inventory` PASS confirms the shared pipeline API is
  exported in the namespace inventory.
- `s233/iter44_gazebo_flow_whycon` PASS confirms the shared pipeline binding
  did not regress the live Gazebo camera -> PrepTask -> FlowTask -> WhyCon
  path with PXP acceleration enabled.
- Remaining S225 work is the TPU half of the pipeline contract: persistent
  InferTask/`pipeline.detections()` over staged virtual camera frames with the
  physical Coral USB bridge.  The previous S213 boot timeout is understood and
  the FS/build-sim setup path is repaired; the active blocker is now host
  permission to open the physical Coral USB node.

### s226 - Flow + Pipeline Parallel Mission

Purpose: complete the B7 failed scenario on the B8 emulator path.

Namespaces:

- `sentai.camera`;
- `sentai.pipeline`;
- `sentai.flow`;
- `sentai.tpu`;
- `sentai.fr`.

Tests:

- virtual camera serves shifted cat frames;
- PrepTask prepares both Flow and TPU inputs;
- FlowTask reports offsets;
- InferTask reports COCO detections through physical Coral;
- MicroPython only starts/stops and collects counters/events.

PASS:

- Flow offsets and TPU detections are both valid in the same run;
- no sleep/poll/REPL scheduling dependency;
- no stuck Renode or bridge process after repeated runs.

### s227 - Console And Non-cf2 Transport Boundaries

Purpose: separate REPL, USB, UART, radio/drone, and mesh concepts without
pulling PX4/MAVLink into the current critical path.

Namespaces:

- `sentai.uart`;
- `sentai.usb`;
- `sentai.mesh`;
- `sentai.link` as documented deferred surface only.

Tests:

- document LPUART6 REPL console behavior;
- expose `sentai.uart` through the shared binding with an explicit emulator
  backend;
- expose `sentai.usb` through the shared binding with clear unsupported
  backend behavior for MSC/serial/IP;
- document how the s219 Crazy bridge differs from `sentai.link` and `mesh`;
- make `sentai.link` fail predictably or remain absent until PX4/MAVLink SITL
  is installed.

PASS:

- REPL transport is not confused with drone transport;
- unavailable physical transports fail predictably;
- `sentai.link` is explicitly deferred, not silently half-wired.

### s232 - Deferred PX4/MAVLink Link Bridge

Purpose: verify `sentai.link` after a PX4/MAVLink simulator is installed.  This
is intentionally later than the Crazyflie/cf2 path because B9 can use cf2 and
Gazebo without PX4.

Namespaces:

- `sentai.link`;
- `sentai.uart` or host socket/PTY transport selected for MAVLink.

Tests:

- start PX4 SITL or another MAVLink endpoint;
- connect emulator guest transport through a host bridge;
- verify heartbeat, command send, telemetry receive, and cleanup;
- document how this bridge coexists with the s219 Crazy bridge.

PASS:

- `sentai.link` can run a minimal command/response smoke without affecting
  `sentai.crazy`;
- all PX4-specific artifacts remain in the relevant `sNNN` experiment folder.

### s228 - Sensor And Actuator Namespace Smokes

Purpose: make hardware-dependent owner namespaces explicit in emulator.

Namespaces:

- `sentai.imu`;
- `sentai.mic`;
- `sentai.servo`;
- `sentai.calib`;
- `sentai.object_lifter`;
- `sentai.safety`.

Tests:

- `imu` uses the shared `sentai_runtime` binding and raises
  `NotImplementedError` in the ARM emulator;
- `mic` uses the shared `sentai_runtime` binding and raises
  `NotImplementedError` in the ARM emulator;
- `uart` uses the shared `sentai_runtime` binding; the S228 backend is
  unavailable, while Crazy/Gazebo targets can override the same ABI with the
  emulator MMIO serial bridge;
- `sleep` is not a separate namespace anymore; use `sentai.rtos.sleep_ms()`
  for delays;
- `servo` command/state API accepts safe no-motion values;
- `calib` uses the shared runtime binding, can load/save config and run
  non-flight math paths, while worker/task entrypoints report not-supported
  through shared unavailable backends when the target has no camera/marker
  worker;
- `object_lifter` uses the shared inverse-depth EKF implementation and can run
  deterministic math/state paths in the emulator;
- `safety` starts/stops/reports status without flight hardware.

PASS:

- every function either performs a deterministic emulator action or returns a
  clear not-supported result;
- safety-critical APIs cannot accidentally claim real actuator success.

Implementation note:

- `emu/sentai_emu_fs_module.c` now exposes `sentai.usb`, `sentai.uart`,
  `sentai.imu`, `sentai.mic`, `sentai.safety`, `sentai.servo`,
  `sentai.calib`, and `sentai.object_lifter` through the shared
  `examples/sentai_runtime/bindings` code instead.
- The default S228 behavior deliberately rejects or reports inactive for real
  USB MSC/serial/IP and UART serial.  Shared `sentai.usb` keeps the production
  `sentai.console('uart')` guard and calls an explicit unavailable EMU backend
  for `sentai_usb_*`; shared `sentai.uart` keeps the production
  `sentai.console('usb')` guard and calls weak unavailable
  `sentai_uart_serial_*` backend functions in S228; Crazy/Gazebo targets can
  override the same UART ABI with their Renode bridge.  IMU and microphone calls fail loudly with
  `NotImplementedError` rather than returning fixture data.  `sentai.servo`
  uses the shared FSM and shared MicroPython binding; the SIM backend remains
  record-only/no-motion, while optional CF2/PX4 transports are weak-linked and
  unavailable unless their bridge is present.  Only `servo.marker_*` is linked
  to the shared unavailable backend in S228 because this target has no
  camera/markers worker.
- `sentai.safety` uses the shared state machine; only the continuous
  SafetyTask worker is linked to the shared unavailable backend in S228
  because this target has no camera/markers worker.
- `sentai.calib` uses shared `modsentai_calib.c` and shared
  `sentai_calib.cc`; only `sentai_calib_task`,
  `sentai_calib_bringup`, and `sentai_calib_orientation_task` are linked to
  shared unavailable backends in S228/precursor targets.  The S228 and S229
  emulator binaries use the larger runtime linker profile because the complete
  shared calibration surface no longer fits the tiny smoke linker layout.
- `sentai.object_lifter` uses shared `modsentai_object_lifter.c` and shared
  `sentai_object_lifter.cc`; no emulator-specific lifter module remains.
- `sentai.rtos.sleep_ms()` remains the emulator-safe delay primitive;
  `sentai.safety._test_push_aruco()` remains available as a test-only abort
  injector.
- Build-only follow-up after servo/calib/usb/uart/object_lifter unification:
  `sentai_emu_hw_stub_namespace_smoke` links shared `sentai_servo.cc`,
  `sentai_calib.cc`, `sentai_object_lifter.cc`, and the shared unavailable
  worker backends; symbol checks confirm `sentai_servo_module`,
  `sentai_calib_module`, `sentai_usb_module`, `sentai_uart_module`, and
  `sentai_object_lifter_module` are present.  The legacy local
  `emu_servo_module`, `emu_calib_module`, `emu_usb_module`,
  `emu_uart_module`, and `emu_object_lifter_module` surfaces are not linked.
- The S228 runner resets the host-backed Renode raw NAND image and runs the
  existing `sentai_emu_fx_storage` setup before the namespace smoke.  This
  keeps destructive storage preparation at the emulator flash boundary; S228
  itself uses the normal shared `sentai.fs`/`sentai.fr` runtime surface.

Status:

- `s228/iter07_hw_stub_namespaces` PASS with Renode UI/analyzer enabled on
  2026-06-11.
- Build #1496.
- 69 checks passed, 0 failed.
- Storage setup PASS: `sentai_emu_fx_storage` reported boot state `0x0600`,
  `fs_read_ok=1`, and `fx_errors=0` before S228 booted.
- FlightRecorder event/scalar files were written under `/fr` in the emulator
  FileX volume and copied into the experiment artifact folder.
- Intermediate notes: `iter03` exposed the stricter shared UART behavior
  (`sentai.uart.read()` raises `OSError` when the unavailable serial backend is
  not open), so the smoke expectation was corrected.  `iter04` then reached
  all namespace checks but failed FileX/FlightRecorder writes because the
  shared emulator NAND image was not mounted/writable.  `iter05` proved that
  explicit formatting fixed FileX but put setup logic in the runtime smoke, so
  it was superseded.  `iter06` showed that deleting the raw NAND image alone is
  not sufficient for this target.  `iter07` moves setup to the existing
  emulator storage smoke and is the canonical post-unification artifact.

### s230 - Camera + WhyCon Marker Smoke

Purpose: validate camera-fed C++ vision algorithms after camera/prep is stable.

Namespaces:

- `sentai.camera`;
- `sentai.markers`;
- active backend for new work: WhyCon/circle markers via
  `sentai.markers.init("whycon")`;
- legacy backend code for ArUco/WhyCon remains internal, but there are no
  restored `sentai.aruco` or `sentai.whycon` namespace aliases.  ArUco is
  deprecated for mission use because the CPU cost is too high.

Relevant precedents:

- B1/B2: synthetic WhyCon/circle marker datasets and validation;
- B3/B4/B5: calibration, handoff, and C++ marker-control task work;
- missions `s183`, `s184`, `s187`, `s192`, `s194`, `s203`, `s205`, `s207`
  use `sentai.markers`/WhyCon-style marker observations.

Tests:

- synthetic WhyCon/circle marker image from FileX;
- marker detection from virtual camera/prep frame;
- `sentai.markers` result shape and timing;
- repeated run stability.

PASS:

- known marker is detected with expected ID/pose fields where applicable;
- namespace ownership remains `sentai.markers`.

Status:

- `examples/sentai_runtime/experiments/s230_arm_emulator_camera_markers/iter03_camera_markers`
  PASS.
- Assets staged through the guest FileX stack:
  `/markers/whycon_320x240.pgm` (76815 bytes) and
  `/images/whycon_640x480.bmp` (921654 bytes).
- Direct marker path:
  `sentai.markers.init("whycon")` +
  `sentai.markers.detect_pgm("/markers/whycon_320x240.pgm")` found one marker.
- Camera marker path:
  `sentai.camera.select(-1, "/images/whycon_640x480.bmp")` +
  `sentai.camera.prep_once()` +
  `sentai.markers.detect_from_camera()` found the same synthetic marker.
- Representative camera detection:
  center `(159.5, 119.5)`, radius `34.0`, `z ~= 0.2724`.
- Stability loop: 10/10 repeated camera/prep/detect iterations passed in
  3428 ms guest timing, reported as `loop_fps_x100=291` (~2.91 FPS in this
  Renode/MicroPython marker profile).
- Runner:
  `python3 examples/sentai_runtime/experiments/s230_arm_emulator_camera_markers/run_s230.py`
  rebuilds the target, regenerates assets, resets/stages NAND, runs Renode,
  archives UART/Renode logs, writes `verdict_s230.json`, and checks for stuck
  helper processes.
- The runner removes the S230 UART files from `emu/output` after archiving them
  into `iterNN_camera_markers`, so the durable logs stay in the experiment
  folder.

Implementation notes:

- `sentai_emu_camera_markers_repl` links the shared camera binding, shared
  markers binding, shared virtual-camera/backend/prep code, and the WhyCon
  detector.
- `sentai_emu_camera_markers_stage_assets` stages the marker image files into
  the same emulated FileX/LevelX storage used by the REPL target.
- `emu/sentai_emu_camera_runtime_bridge.cc` is guest firmware glue, not a POSIX
  SIM backend.  It supplies the camera HAL and filesystem callbacks expected by
  the shared runtime in the ARM-emulator build.
- The WhyCon PGM FileX fallback was fixed to read the full PGM through a large
  scratch buffer, matching the existing ArUco loader behavior.

Performance interpretation:

- The S230 loop is intentionally a synchronous MicroPython smoke:
  `prep_once()` + `detect_from_camera()` are called from the autorun REPL loop.
- `sentai.camera.select(-1, path)` loads the BMP once from FileX into the
  virtual-camera memory buffer, so the steady S230 loop is not dominated by
  rereading the BMP file.
- The low reported marker FPS is still not representative of the intended
  runtime pipeline because frame publication, prep, and marker detection are
  not yet persistent tasks fed by a live camera source.  It is a namespace and
  data-path correctness proof, not the target architecture for flight.

### s233 - Gazebo Camera + Markers + Flow Ascent

Purpose: resume the successful B3/B4/B5 Gazebo mission style on the ARM
emulator path.  The drone takes off in `sentai_whycon_small`, camera frames
come from Gazebo, WhyCon markers produce visual pose/centering observations,
and Flow runs simultaneously while the drone climbs to the selected visual-Z
band.

Reference missions:

- `s197_sota_calib_orientation_guarded`: A3 visual calibration sequence and
  strict marker-lock gates.
- `s203_hl_marker_control_with_flow`: B4 high-level marker control with
  `sentai.flow` assistance, ExtPos warmup, WhyCon marker observations, and
  CRTP flow packets into cf2.
- `s127/s166/s167`: FlowBaseline lineage, especially the anti-cheat rule that
  Gazebo ground truth is post-mortem only and never injected into SentAI.

Architecture:

- keep `sentai.crazy` on the existing emulator CPX/CRTP bridge to cf2;
- add a Gazebo-camera host bridge for emulator that reuses the existing
  simulator wire protocol shape from `gz_to_camera_bridge.py` /
  `camera_bridge_recv.c`: RGB888 640x480 frames with monotonic seq;
- terminate that bridge at an emulator guest boundary that calls
  `sentai_camera_backend_publish_rgb888(seq, rgb, bytes)`;
- wake shared prep/consumer paths exactly as the real camera provider does;
- run marker detection and Flow from shared `sentai_runtime` code, not from the
  host bridge and not from POSIX SIM;
- use FlightRecorder/FileX for mission logs and artifact extraction;
- keep Gazebo GT logs post-mortem only.

Implementation phases:

1. Camera ingress smoke:
   - start `sentai_whycon_small`;
   - stream Gazebo `/downward_cam/image` into emulator guest camera backend;
   - prove `sentai.camera.stats()` / frame seq / cameraId increase without MP
     polling each frame.
2. Markers-only smoke:
   - run `sentai.markers.init("whycon")` with the s203 intrinsics/layout;
   - detect from the live Gazebo camera and log `n_full`, centroid, radius,
     bbox, and visual-Z;
   - require stable detection of the marker pad at rest.
3. Flow-only smoke:
   - start the emulator FlowTask/flow publisher over the same camera/prep
     frames;
   - log `sentai.flow.body_read()`, frame seq, confidence, and Flow FPS while
     the drone is stationary and then while it lifts.
4. Combined ascent smoke:
   - port the safe subset of s203:
     setup -> acquire marker lock -> thrust-only takeoff -> center hold ->
     ExtPos warmup with `_pump_flow()` -> soft land;
   - do not run the later generic image-axis envelope until the ascent path is
     stable.
5. Verdict:
   - pass requires no stuck Renode/Gazebo/cf2/bridge processes, marker lock,
     Flow frame progression, ExtPos sends, and a soft land;
   - GT may be used only in post-mortem plots/summary.

Status:

- `s233/iter20_gazebo_flow_whycon` proved the live Gazebo camera ingress path:
  Gazebo `/downward_cam/image` -> host UDS/TCP relay -> Renode MMIO pull ->
  `sentai_camera_backend_publish_rgb888()` -> shared `PrepTask` -> `FlowTask`
  and `sentai.markers`.  The first version passed Flow and marker API smoke,
  but did not require real WhyCon hits.
- `s233/iter21_gazebo_flow_whycon` is the first real FlowBaseline +
  WhyconBaseline run on the ARM emulator path.  The mission runs in
  `sentai_whycon_small`, uses the drone-mounted Gazebo downward camera, starts
  `PrepTask` and `FlowTask`, measures a flow-only phase, then commands cf2
  through shared `sentai.crazy` (`init`, `ping`, `arm`, `takeoff`, `hl_stop`,
  hover, `land`, `stop`) while WhyCon detects from the same live camera/prep
  frames.
- `iter21` PASS:
  `cam_frames=499`, `bridge_seen=500`, `bridge_served=500`,
  `prep_frames=115`, `prep_fps_x100=642`, `flow_frames=114`,
  `flow_fps_x100=637`, `flow_base_seq_last=23`, `flow_base_nonzero=44`,
  `marker_samples=7`, `marker_hits=2`, `marker_best=7`,
  `crazy_init=0`, `crazy_ping=2`, `crazy_arm=0`, `crazy_takeoff=0`,
  `crazy_land=0`, `crazy_stop=0`.
- `s233/iter22_gazebo_flow_whycon` repeats the same test with
  `--renode-ui`; PASS with `cam_frames=490`, `prep_frames=114`,
  `flow_frames=113`, `marker_hits=2`, `marker_best=7`, and clean process
  cleanup.  This validates the Renode UART analyzer path as well as the
  headless automation path.
- The S233 verdict now requires `marker_hits > 0`; a no-error marker smoke is
  no longer enough to pass WhyCon.
- `s233/iter23_gazebo_flow_whycon` restores the camera ingress to VGA, matching
  the Gazebo sensor and historical SentAI tests.  PASS with
  `camera_out_width=640`, `camera_out_height=480`, `cam_last_rc=921600`
  (`640*480*3` RGB888), `cam_frames=212`, `bridge_seen=213`,
  `prep_frames=45`, `prep_fps_x100=255`, `flow_frames=45`,
  `flow_fps_x100=255`, `flow_base_nonzero=35`, `marker_hits=2`,
  `marker_best=7`, and clean cf2 command results.
- Resolution boundary clarified: the camera/provider path is VGA RGB888 into
  `sentai_camera_backend_publish_rgb888()`.  PrepTask owns downscales to
  `SENTAI_PREP_SLOT_GRAY_NATIVE` (320x240 Y8) and
  `SENTAI_PREP_SLOT_FLOW_GRAY_80x60` (80x60 Y8).  FlowTask consumes only the
  80x60 slot; it does not process 640x480 directly.
- PXP emulator research: Renode does not provide a functional NXP PXP model in
  the local 1.16.1 tree.  The only matching built-in entry found is an
  `imxrt1064.repl` `Tag <...> "PXP"`, which is a named/logged address region,
  not an image-processing peripheral.  Our S233 firmware target currently links
  `examples/sentai_runtime/sentai_pxp_shim_sim.c`, so `sentai_pxp_scale()` and
  `sentai_pxp_xrgb_to_y8()` run as scalar C inside the emulated Cortex-M.  That
  explains the VGA bottleneck: the RGB ingress optimization avoids XRGB->RGB
  transfer/conversion overhead, but the gray/resize work still burns emulated
  CPU cycles.
- Official Renode direction: use a custom peripheral at the PXP semantic
  boundary if this remains a bottleneck.  Renode documents Python peripherals
  and system-bus hooks for simple MMIO-triggered logic, and C# peripheral models
  for more advanced behavior; HDL co-simulation exists, but is not useful unless
  we have an RTL PXP model.  Recommended next gate is an ARM-EMU-only
  `sentai_pxp_shim_emu` that preserves the production API
  (`sentai_pxp_scale`, `sentai_pxp_xrgb_to_y8`) while dispatching the resize/Y8
  operation to a Renode host-side PXP bridge.  This is more faithful than
  precomputing PrepTask slots in the camera bridge, because PrepTask still owns
  the transform contract.
- Current S233 checkpoint after RGB ingress work:
  `s233/iter31_gazebo_flow_whycon` is the latest canonical PASS for live Gazebo
  VGA input into the ARM emulator.  Gazebo/camera bridge publishes
  `640x480 RGB888` (`cam_last_format=0`, `cam_last_rc=921600`) into the shared
  SentAI camera backend.  The run reported `cam_frames=644`,
  `bridge_seen=691`, `bridge_served=691`, `prep_frames=110`,
  `prep_fps_x100=517` (5.17 FPS), `flow_frames=102`,
  `flow_fps_x100=479` (4.79 FPS), `flow_base_nonzero=32`,
  `marker_hits=1`, `marker_best=7`, and clean cf2 command results.
- Baseline comparison: `s233/iter29_gazebo_flow_whycon` used VGA XRGB-style
  ingress (`cam_last_format=1`, `cam_last_rc=1228800`) and passed at
  `prep_fps_x100=454` (4.54 FPS) / `flow_fps_x100=422` (4.22 FPS).
  The RGB888 fast path therefore removes 307200 bytes per VGA frame from the
  bridge and improves end-to-end throughput, but does not remove the dominant
  scalar resize/Y8 cost inside the emulated guest.
- Throughput interpretation: isolated production FlowTask over already prepared
  80x60 frames previously measured about 47.61 FPS in S214/S215, so the current
  ~5 FPS S233 limit is not the flow algorithm itself.  It is the live
  camera-to-PrepTask path: Gazebo bridge + guest camera publication +
  `sentai_pxp_*` scalar resize/gray running under ARM emulation.
- Fresh-PC UI rerun on 2026-06-05:
  `s233/iter35_gazebo_flow_whycon` and `s233/iter36_gazebo_flow_whycon`
  both PASS with Gazebo GUI and Renode UI enabled.  `iter35` used
  `--camera-forward-fps 10` and reported `cam_frames=430`,
  `prep_fps_x100=521`, `flow_fps_x100=478`, `marker_hits=1`, and clean
  cf2 command results.  `iter36` used `--camera-forward-fps 15` and reported
  `cam_frames=529`, `prep_fps_x100=521`, `flow_fps_x100=473`,
  `marker_hits=1`, and clean cf2 command results.  The forwarded-frame rate
  changes bridge/camera frame counts, but not the current guest
  PrepTask/FlowTask ceiling.  The fresh PC therefore reproduces the same
  live-Gazebo VGA `rgb888` bottleneck: about 5.2 FPS prep and 4.7-4.8 FPS
  flow while preserving the production-like camera/prep/flow boundary.
- `s233/iter33_gazebo_flow_whycon` and `s233/iter34_gazebo_flow_whycon` are
  setup failures, not algorithm results.  They were launched from the
  restricted sandbox and failed before Renode because distrobox/podman could
  not access `/run/user/<uid>/libpod`.  GUI S233 runs must be launched on the
  host, where Gazebo can use the real display and Renode can open its UI.
- PXP acceleration follow-up on 2026-06-05:
  `s233/iter40_gazebo_flow_whycon` and
  `s233/iter41_gazebo_flow_whycon` both PASS after adding the emulator-only
  C# `SentaiPxpAccelerator` at `0x40902C00` and wiring
  `sentai_pxp_*` through the MMIO shim.  `iter40` headless reports
  `pxp_accel_calls=1078`, `pxp_accel_ok=1078`, `pxp_accel_fallback=0`,
  `prep_fps_x100=2706`, `flow_fps_x100=2571`, and `marker_hits=1`.
  `iter41` with Gazebo GUI and Renode UI reports `pxp_accel_calls=1090`,
  `pxp_accel_ok=1090`, `pxp_accel_fallback=0`, `prep_fps_x100=2718`,
  `flow_fps_x100=2578`, and `marker_hits=2`.  The valid accelerator uses
  Renode C# bulk `ReadBytes`/`WriteBytes`; the aborted `iter37` IronPython
  bridge proved the contract but was far too slow at roughly seconds per VGA
  resize.  `iter39` also showed that Y8 semantics must match the scalar
  fallback by averaging per-pixel luminance, not luminance of averaged RGB.
  `s233/iter42_gazebo_flow_whycon` then refactored the guest bridge contract
  to match the TPU `SendParameters` bridge style: a small
  `STATUS/COMMAND/SEQ` request plus semantic transform arguments behind
  `sentai_pxp_transform()`.  It remains PASS with `pxp_accel_calls=1092`,
  `pxp_accel_ok=1092`, `pxp_accel_fallback=0`, `prep_fps_x100=2712`,
  `flow_fps_x100=2567`, and `marker_hits=1`.
- Post-`sentai.pipeline` shared-binding regression on 2026-06-11:
  `s233/iter44_gazebo_flow_whycon` PASS with Gazebo GUI and Renode UI.  The
  run used VGA `rgb888` at `--camera-forward-fps 15` after the
  `sentai.pipeline` root surface was moved to the shared
  `examples/sentai_runtime/bindings/modsentai_pipeline.c` binding.  Results:
  `cam_frames=1042`, `bridge_seen=1163`, `bridge_served=1163`,
  `cam_last_rc=921600`, `pxp_accel_calls=1124`, `pxp_accel_ok=1124`,
  `pxp_accel_fallback=0`, `prep_fps_x100=2731`, `flow_fps_x100=2600`,
  `marker_hits=2`, `marker_best=7`, and clean `sentai.crazy` command results
  (`init=0`, `arm=0`, `takeoff=0`, `land=0`, `stop=0`).  This confirms the
  shared pipeline binding did not regress the live Gazebo camera -> PrepTask ->
  FlowTask -> WhyCon path.
- `s233/iter43_gazebo_flow_whycon` is a setup failure only: `respawn_sitl`
  failed before Renode due to the VS Code snap/podman storage mismatch between
  snap revisions `244` and `247`.  The workaround used for `iter44` was to run
  S233 with the old snap XDG storage variables and recreate the transient
  podman runtime files under `/run/user/1000/.../userdata`.
- Current constraints:
  - no full functional PXP/CSI hardware model in Renode for this target;
  - `sentai_pxp_*` is now accelerated only by an explicit emulator-only
    semantic peripheral, not by a register-complete NXP PXP model;
  - host wall-clock and guest FreeRTOS tick timing must be reported separately
    for any future bridge optimization;
  - emulator FPS depends on host CPU speed and the amount of scalar image work
    left in the emulated Cortex-M;
  - keep camera/PrepTask/FlowTask semantics production-like; emulator-only
    shortcuts should live at explicit platform boundaries such as
    `sentai_pxp_*`, not inside mission code.

Open design decision:

- Prefer a host-to-guest MMIO/ring-buffer camera peripheral over FileX or REPL
  staging for live Gazebo frames.  FileX is acceptable for static fixtures, but
  the flight path must look like CameraTask/provider frame publication.
- If the host-side PXP bridge is implemented, keep the boundary at
  `sentai_pxp_*` first, not at WhyCon/Flow outputs.  Measure host wall-clock
  transfer time separately from guest FreeRTOS ticks so we do not mistake
  emulator timing for real throughput.

### s231 - Object/Place/SLAM/Explore Smokes

Purpose: verify stateful runtime algorithms that do not require real flight.

Namespaces:

- `sentai.objects`;
- `sentai.places`;
- `sentai.slam`;
- `sentai.explore`.

Tests:

- create/list/clear object records;
- add/query places and descriptors;
- run a small SLAM/place update sequence with deterministic pose/frame data;
- run an explore state transition smoke.

PASS:

- data structures survive save/load where supported;
- counters and stats are stable across repeated emulator boots.

### s232 - ML Helper Namespace Smokes

Purpose: verify pure or mostly-pure algorithm namespaces after shared root
parity is stable.

Namespaces:

- `sentai.aifes`;
- `sentai.kmeans`;
- `sentai.pca`;
- `sentai.anomaly`;
- `sentai.dtw`;
- `sentai.hmm`;
- `sentai.rl`;
- `sentai.tfl`.

Tests:

- one deterministic tiny dataset per namespace;
- train/observe/score/predict/save/load where supported;
- TFL CPU path smoke if included in emulator build;
- FR timing for any non-trivial compute.

PASS:

- outputs match host/known expected values within fixed tolerance;
- save/load APIs work over FileX where implemented.

## Namespace Matrix

| Namespace | First B9 gate | Expected emulator status |
| --- | --- | --- |
| root `sentai` | s216/s217 | shared root required |
| `sentai.fs` | s218 | real FileX/LevelX |
| `sentai.fr` | s218 | real enough for logs |
| `sentai.rtos` | s218 | real FreeRTOS subset |
| `sentai.sys` | s218 | mixed real/stub |
| `sentai.io` | s218 | emulator-safe stubs |
| `sentai.crazy` | s219/s229/s220 | cf2 bridge priority path |
| `sentai.camera` | s220/s221/s222 | Gazebo/virtual provider real runtime |
| `sentai.pipeline` | s222/s225 | real persistent tasks |
| `sentai.flow` | s223/s226 | production FlowTask |
| `sentai.tpu` | s224/s225 | shared binding; physical Coral via EMU bridge backend |
| `sentai.tfl` | s232 | deferred until shared root stable |
| `sentai.uart` | s219/s227/s232 | shared binding; backend ABI is board/SIM/EMU-specific |
| `sentai.usb` | s227/s228 | shared binding; EMU backend unavailable for MSC/CDC/IP |
| `sentai.mesh` | s227 | deferred/stub |
| `sentai.link` | s232/deferred | wait for PX4/MAVLink simulator |
| `sentai.imu` | s228 | shared binding, board-only; raises `NotImplementedError` in emulator |
| `sentai.mic` | s228 | shared binding, board-only; raises `NotImplementedError` in emulator |
| `sentai.servo` | s228 | safe no-motion state |
| `sentai.calib` | s228/s229 | shared binding; worker backends unavailable until live visual calib path |
| `sentai.object_lifter` | s228 | shared inverse-depth EKF math/state smoke |
| `sentai.safety` | s228/s233 | shared state machine; unavailable task backend in S228, real task in Gazebo |
| `sentai.markers` | s230 | real WhyCon algorithm smoke after camera provider |
| `sentai.objects` | s231 | real state smoke |
| `sentai.places` | s231 | real state smoke |
| `sentai.slam` | s231 | deterministic mini-sequence |
| `sentai.explore` | s231 | state smoke |
| `sentai.aifes` | s232 | tiny deterministic model |
| `sentai.kmeans` | s232 | tiny deterministic dataset |
| `sentai.pca` | s232 | tiny deterministic dataset |
| `sentai.anomaly` | s232 | observe/score/save/load |
| `sentai.dtw` | s232 | tiny deterministic sequences |
| `sentai.hmm` | s232 | tiny train/predict |
| `sentai.rl` | s232 | tiny Q/MAB/DQN smoke |

## Reporting Format

Each experiment should write a short `RESULT.md` or `summary.json` containing:

- git SHA;
- Renode version;
- target ELF name;
- staged FS image path/checksum;
- namespaces exercised;
- API calls made;
- PASS/FAIL per namespace;
- wall-clock timing when host bridges are involved;
- guest tick timing when pure guest behavior is measured;
- list of bridge/helper processes before and after cleanup.

## First Implementation Step

Start with `s216_arm_emulator_namespace_inventory`.  This does not require
solving shared-root compilation yet and will give B9 a real baseline instead of
guesswork.

The second step is `s217_arm_emulator_shared_modsentai_minroot`, where the
emulator starts using the production root table and explicit owner stubs.  After
that, each namespace can move from `missing` or `emu-subset` to `verified`.

Priority branch: `s219` may start as soon as there is a callable
`sentai.crazy` surface, even if the full shared root is still incomplete.  The
important boundary is that the guest-side API remains `sentai.crazy` and the
emulator-specific work is kept in the transport bridge to cf2, not in mission
code or a new Python namespace.

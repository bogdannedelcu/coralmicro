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
- probes the current `sentai.calib` emulator stub boundary
  (`get_R_cam_to_body`, `get_cam_offset_B`, `is_calibrated`);
- logs through `sentai.fr` and writes a small `/b9/s229_status.txt` marker in
  FileX;
- copies Renode, UART, bridge, SITL, command, and verdict logs into
  `examples/sentai_runtime/experiments/s229_arm_emulator_gazebo_calib_precursor/iterNN_*`.

Status:

- `s229/iter01_gazebo_whycon_calib_precursor` PASS.
- Results: `CALIB_GZ_INIT=0`, `CALIB_GZ_PING_MS=2`,
  `CALIB_GZ_CALIB_STUB 9 (0.0, 0.0, 0.0) False`,
  `CALIB_GZ_FLY_RC=0`, `CALIB_GZ_STOP=0`.
- Bridge counters: `guest_to_udp=5`, `udp_to_guest=3`,
  `serial_tx=131`, `serial_rx=50`.
- Cleanup check found no leftover Renode/Gazebo/cf2/bridge processes.

Limitations:

- This is not full visual calibration.  `sentai.calib` is currently the s228
  deterministic stub in this target.
- `CALIB_GZ_ALT_AFTER=-999.0` is diagnostic only.  The current
  `sentai.crazy.altitude()` path expects the SentAI deck telemetry channel,
  while this bridge is validating CRTP/cf2 traffic.
- Full calibration needs `sentai.camera`, `sentai.markers`/WhyCon, and the
  real C++ calibration task ported into a shared emulator profile.

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
- expose `sentai.uart` with emulator-backed loopback or explicit stubs;
- expose `sentai.usb` with clear unsupported/stub behavior for MSC/serial;
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
- `sentai.sleep`;
- `sentai.servo`;
- `sentai.calib`;
- `sentai.object_lifter`;
- `sentai.safety`.

Tests:

- `imu` deterministic sample/stub or unsupported error;
- `mic` deterministic sample/stub or unsupported error;
- `sleep` wake/sleep calls documented for emulator;
- `servo` command/state API accepts safe no-motion values;
- `calib` can load/save config and run non-flight math paths;
- `object_lifter` and `safety` start/stop/status without flight hardware.

PASS:

- every function either performs a deterministic emulator action or returns a
  clear not-supported result;
- safety-critical APIs cannot accidentally claim real actuator success.

Implementation note:

- `emu/sentai_emu_fs_module.c` now has a guarded
  `SENTAI_EMU_HW_STUB_MODULES` profile exposing `usb`, `uart`, `imu`, `mic`,
  `sleep`, `servo`, `calib`, `object_lifter`, and `safety` as emulator-safe
  stubs.
- The default S228 behavior deliberately rejects or reports inactive for real
  USB MSC/serial, UART serial, mic capture, IMU tap hardware, safety task
  monitoring, and actuator-like servo actions.
- `sentai.imu.read()` returns a deterministic level sample
  `{x:0, y:0, z:1000, temp:25}`; `sentai.sleep.idle(..., timeout_ms, ...)`
  is timeout-only; `sentai.safety._test_push_aruco()` remains available as a
  test-only abort injector.

Status:

- `s228/iter02_hw_stub_namespaces` PASS with Renode UI/analyzer enabled.
- 45 checks passed, 0 failed.
- FlightRecorder event/scalar files were written under `/fr` in the emulator
  FileX volume and copied into the experiment artifact folder.

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
| `sentai.tpu` | s224/s225 | physical Coral via Send* bridge |
| `sentai.tfl` | s232 | deferred until shared root stable |
| `sentai.uart` | s219/s227/s232 | REPL plus separate Crazy/link transport |
| `sentai.usb` | s227 | documented unsupported/stub for MSC/CDC |
| `sentai.mesh` | s227 | deferred/stub |
| `sentai.link` | s232/deferred | wait for PX4/MAVLink simulator |
| `sentai.imu` | s228 | deterministic stub or fixture |
| `sentai.mic` | s228 | deterministic stub or fixture |
| `sentai.sleep` | s228 | safe unsupported/stub |
| `sentai.servo` | s228 | safe no-motion state |
| `sentai.calib` | s228/s229 | stub boundary now; real visual path deferred |
| `sentai.object_lifter` | s228 | safe state smoke |
| `sentai.safety` | s228 | state/task smoke |
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

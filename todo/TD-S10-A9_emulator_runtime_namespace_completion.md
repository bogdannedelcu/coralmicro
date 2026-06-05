# TD-S10-A9 - Complete SentAI Runtime Namespace Validation In ARM Emulator

## Goal

Continue A7/B7 and A8/B8 by making the ARM emulator the main proof path for
the real `sentai.*` runtime surface.

A7/B7 started the namespace-alignment work, but the POSIX SIM route hit a
scheduler/backend dead end for the Flow + TPU + virtual camera scenario.  A8/B8
then proved that Renode can run the ARM FreeRTOS model, FileX/LevelX storage,
the guest-owned EdgeTPU `Send*` boundary, physical USB Coral inference, and
production `FlowTask` in parallel.  A9 should connect those two tracks:

- keep the A7 rule that `sentai.*` is organized by owner namespace;
- stop using B7 POSIX SIM as the primary concurrency proof;
- port and validate the shared SentAI runtime bindings in the ARM emulator;
- replace B8 harness-only proofs with emulator missions that exercise the same
  MicroPython-facing APIs used on the board.

## Why A9 Exists

B8 is a strong checkpoint, but much of it is still structured as focused
emulator harnesses.  Current emulator targets expose subsets such as
`sentai.fs`, minimal `sentai.rtos`, minimal `sentai.fr`, and a B8-specific
`sentai.tpu` bridge surface.  They do not yet compile the full shared
`examples/sentai_runtime/modsentai.c` root with the same namespace table as the
ARM board.

The next risk is therefore not "can Renode boot FreeRTOS?" or "can physical
Coral inference be reached?"  Those are proven.  The risk is whether the real
SentAI runtime API, task topology, mission style, filesystem, logging, camera
provider, FlowTask, PrepTask, InferTask, and host bridges are all wired together
through the same `sentai.*` surface in emulator mode.

## Already Proven

- ARM emulator startup reaches FreeRTOS scheduler/heartbeat/idle.
- LPUART6 REPL works as the emulator console transport.
- FileX/LevelX runs in the guest over a raw-NAND backend supplied by Renode.
- Host-side asset staging can populate the emulator filesystem before boot.
- Basic emulator `sentai.fs`, `sentai.rtos`, and `sentai.fr` subsets work.
- Physical USB Coral can be reached from guest code through the low-level
  `EdgeTpuManager -> EdgeTpuExecutable -> TpuDriver::SendParameters /
  SendInputs / SendInstructions / GetOutputs / ReadEvent` boundary.
- The Coral bridge is a real USB/libusb host bridge, not a pycoral shortcut.
- Production `FlowTask` runs in ARM emulation and detects unequal X/Y motion
  from shifted cat frames.
- Production `FlowTask` and the physical Coral `Send*` bridge run in parallel
  in S215 without reproducing the B7 POSIX SIM blockage.

## Non Goals

- Do not re-open B7 as the primary implementation path.
- Do not introduce `sentai.diag`; diagnostics should live under owner
  namespaces such as `sentai.rtos`, `sentai.fs`, `sentai.tpu`, `sentai.camera`,
  `sentai.pipeline`, `sentai.flow`, and `sentai.fr`.
- Do not require full CSI/MIPI/PXP peripheral emulation before validating
  runtime logic.  A production-style virtual camera provider is acceptable.
- Do not require Renode EHCI passthrough to the Coral in A9.  Keep the B8
  `Send*` host bridge as the verified physical-USB path unless a lower-level
  route becomes clearly better.
- Do not use FileX-in-RAM as the canonical emulator filesystem.  It is too far
  from the ARM board storage model.
- Do not make MicroPython the scheduler.  MicroPython commands should start,
  stop, subscribe, and collect results from persistent C/C++ tasks.

## Missing Or Not Yet Verified In Emulator

### Root `sentai`

The shared root module in `examples/sentai_runtime/modsentai.c` is not yet the
canonical emulator root.  B8 targets still use emulator-specific module files
for bring-up.  A9 needs an emulator profile where `import sentai`,
`sentai.help()`, and the exported namespace table match the production runtime
as closely as possible.

Current shared root exports include:

`io`, `rtos`, `tpu`, `fs`, `camera`, `usb`, `uart`, `mesh`, `link`, `crazy`,
`imu`, `mic`, `sleep`, `pipeline`, `flow`, `aifes`, `kmeans`, `pca`,
`anomaly`, `dtw`, `hmm`, `rl`, `slam`, `objects`, `places`, `servo`,
`object_lifter`, `calib`, `markers`, `safety`, `fr`, `explore`, `tfl`, and
`sys`.

### `sentai.fs`

FileX/LevelX is proven in emulator, but the emulator API is still a subset
module.  A9 should switch to the shared filesystem binding or document the exact
shim boundary.  Required checks:

- `exists`, `size`, `read`, `write`, `append`, `remove`, `mkdir`, `ls`, `sync`;
- model/image/mission staging into the same persistent raw-NAND image;
- FlightRecorder artifacts written into the per-experiment `sNNN` folder;
- no stale `emu/output` dependency for final experiment logs.

### `sentai.fr`

The emulator has a basic `sentai.fr` subset, but production-style logging is not
yet the canonical measurement path for all B8 tests.  A9 should verify:

- event/scalar logging from C++ tasks;
- mission-level counters and timing markers from MicroPython;
- flushing to FileX;
- host extraction into the experiment folder;
- no direct `printf` dependency for task diagnostics.

### `sentai.rtos`, `sentai.sys`, `sentai.io`

Only minimal timing helpers are proven in the emulator REPL path.  A9 should
inventory and decide which APIs are real, stubbed, or not applicable:

- task/heap/timing stats;
- `sleep_ms` and `repl_kick` behavior under ARM FreeRTOS;
- reset/boot/recovery/memory helpers in `sentai.sys`;
- LED/GPIO/io helpers through emulator-safe stubs.

### `sentai.camera`

The physical camera peripheral is intentionally not modeled yet.  The missing
piece is a production-style virtual camera provider in `sentai_runtime`, not a
B7 SIM-only source.  It should feed the same buffer/event model as the ARM
camera path and preserve `cameraId`.

Required checks:

- single frame loaded from emulator FS into a camera buffer;
- multi-frame sequence loaded from emulator FS;
- configurable FPS and frame count;
- same producer/consumer contract used by PrepTask;
- no disk I/O in the hot frame loop after preload;
- eventual Gazebo provider can reuse the same provider interface.

### `sentai.pipeline`

B8 did not yet prove the full shared `sentai.pipeline` module in ARM emulator
mode.  A9 should validate persistent task behavior:

- PrepTask is persistent and consumes camera events/slots;
- InferTask is persistent and invokes the TPU bridge synchronously on SIM/EMU
  only where that is the selected backend;
- `start`, `stop`, `get`, `get_ex`, `detections`, `save`, `stats`,
  `infer_stats`, `on_detection`, direct tensor paths, and frame counters are
  inventoried and tested or explicitly deferred;
- `pipeline.stop` only changes state and does not recreate or leak tasks.

### `sentai.tpu`

The B8 physical Coral path is proven at the `TpuDriver::Send*` boundary, but the
MicroPython-facing emulator surface is still bridge-specific.  A9 should align
it with the shared `sentai.tpu` binding:

- model load from emulator FS;
- parameter/instruction/input/output transfer through guest `EdgeTpuManager`;
- image/tensor load from emulator FS and from preloaded memory;
- repeated invoke without reloading weights;
- timing split using host wall-clock for bridge/USB phases;
- output tensor and detection extraction compatible with production pipeline.

### `sentai.flow`

Production `FlowTask` is proven in harnesses, but the `sentai.flow` namespace is
not yet proven from an emulator REPL mission.  A9 should validate:

- `flow.start`, `flow.stop`, and read APIs from MicroPython;
- FlowTask consuming PrepTask output, not a direct harness-only slot, once the
  camera/prep provider is ready;
- unequal X/Y offset scenes from B7/B8;
- Flow-only FPS;
- Flow + Pipeline + TPU simultaneous FPS and correctness.

### `sentai.tfl`

The CPU TFLite Micro path is not validated in emulator.  A9 should decide
whether it remains a fallback test path or only a host/SIM diagnostic path.

### `sentai.usb`, `sentai.uart`, `sentai.mesh`, `sentai.link`, `sentai.crazy`

The REPL console runs over emulator LPUART6, but the board has distinct console,
USB, radio/drone, and CRTP concerns.  A9 should avoid merging those roles.

Required checks:

- document console transport selection in emulator;
- expose or stub `sentai.uart` and `sentai.usb` coherently;
- design a UART/socket bridge for Crazyflie/drone transport;
- verify `sentai.link` and `sentai.crazy` command flow separately from REPL.

### Sensors And Peripherals

The following namespaces are not yet validated in emulator and need either real
tests, emulator stubs, or an explicit deferral:

- `sentai.imu`;
- `sentai.mic`;
- `sentai.sleep`;
- `sentai.servo`;
- `sentai.calib`;
- `sentai.object_lifter`;
- `sentai.safety`.

### Vision, Mapping, And Algorithms

These C/C++ algorithm namespaces should get small emulator smoke tests after
camera/provider and filesystem are stable:

- `sentai.markers`, with the WhyCon/circle-marker backend as the active
  mission path and ArUco documented as legacy/deprecated diagnostics;
- `sentai.objects`;
- `sentai.places`;
- `sentai.slam`;
- `sentai.explore`;
- `sentai.aifes`;
- `sentai.kmeans`;
- `sentai.pca`;
- `sentai.anomaly`;
- `sentai.dtw`;
- `sentai.hmm`;
- `sentai.rl`.

## A9 Gates

1. **A9.1 Namespace inventory baseline**
   Boot the current emulator REPL, run `dir(sentai)` and `dir()` for every
   available namespace, store the result under a new `sNNN` experiment, and
   compare it with the shared ARM root namespace table.

2. **A9.2 Shared root module**
   Build an emulator target that uses the shared `examples/sentai_runtime`
   `modsentai.c` root.  Use `SENTAI_ARM_EMU` or equivalent platform guards for
   unsupported hardware, but keep the same namespace ownership model.

3. **A9.3 Filesystem parity**
   Validate the shared filesystem binding over FileX/LevelX raw NAND and stage
   model/image/mission assets in the experiment folder.

4. **A9.4 FlightRecorder parity**
   Make `sentai.fr` the standard timing/event log path for emulator missions
   and extract its output into the run artifact folder.

5. **A9.5 Virtual camera provider**
   Implement the camera provider in `sentai_runtime`, not in SIM, and feed the
   same queue/buffer/event model used by the board camera path.

6. **A9.6 PrepTask pipeline input**
   Prove `VirtualCameraTask -> PrepTask -> prep slots` in emulator, with
   cameraId and frame sequence preserved.

7. **A9.7 Flow namespace mission**
   Start FlowTask from `sentai.flow`, feed unequal X/Y shifted frames, verify
   offsets and Flow FPS from a MicroPython mission.

8. **A9.8 TPU namespace mission**
   Load the COCO model from emulator FS once, load/preload the cat image, invoke
   repeatedly through the physical Coral `Send*` bridge, and log first/steady
   invoke timing.

9. **A9.9 Pipeline namespace mission**
   Run the real `sentai.pipeline` start/stop/get/detections path through
   PrepTask + InferTask + physical Coral bridge.

10. **A9.10 Flow + Pipeline concurrency**
    Recreate the B7 failed objective in emulator: virtual camera frames with
    offsets, FlowTask active, TPU detections active, and no MP-driven scheduling
    loop.

11. **A9.11 Link/Crazy transport plan**
    Add the first UART/socket bridge proof for `sentai.link` and
    `sentai.crazy`, separate from REPL.

12. **A9.12 Remaining namespace smokes**
    Add lightweight emulator smokes for markers, safety, objects/places/SLAM,
    and the statistical/ML helper namespaces.

## Suggested First B9 Slice

Create `s216_arm_emulator_namespace_inventory`:

- boot the current emulator REPL;
- run a MicroPython script from emulator FS;
- collect `dir(sentai)` plus namespace method lists;
- write JSON/text output through `sentai.fr` and/or `sentai.fs`;
- compare against the shared root in `examples/sentai_runtime/modsentai.c`;
- mark each namespace as `shared`, `emu-subset`, `stubbed`, `missing`, or
  `deferred`.

Then create `s217_arm_emulator_shared_modsentai_minroot`:

- compile the shared `modsentai.c` root in an emulator target;
- keep only minimal working owner namespaces first: `fs`, `fr`, `rtos`, `sys`,
  and `tpu`;
- add explicit emulator stubs for hardware-only owners;
- verify `sentai.help()` and `dir(sentai)` from REPL.

## Success Criteria

- Every A9/B9 experiment follows the existing `sNNN` folder convention.
- Runtime artifacts and logs live in the experiment folder, not only in
  `emu/output`.
- The emulator build uses shared `sentai_runtime` code by default; emulator-only
  code is limited to board/peripheral/host bridge seams and named clearly.
- The physical ARM build is not regressed by emulator guards.
- No orphan Renode, bridge, or USB helper processes remain after test runs.
- The final emulator mission can run FlowTask and TPU detections together using
  `sentai.*` APIs from MicroPython, with persistent C/C++ tasks doing the work.

## Open Risks

- QSTR regeneration is required whenever shared MicroPython bindings change.
  Follow the local skill instructions for QSTR regeneration and remove any
  temporary generated files.
- The shared root may expose APIs whose hardware backends are not meaningful in
  Renode yet.  Those should be explicit stubs, not silent no-ops.
- Renode wall-clock and guest ticks measure different things.  TPU/bridge FPS
  must use host wall-clock for physical USB timing.
- The B8 `Send*` bridge is a deliberate platform seam.  It should remain aligned
  with Coral Micro `EdgeTpuManager` and libedgetpu behavior, but it is not full
  USB-controller emulation.

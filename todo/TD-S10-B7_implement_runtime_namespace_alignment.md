# TD-S10-B7 - Implement A7 Runtime Namespace Alignment

## Purpose

Implement the first safe slice of A7: inventory and align the runtime
`sentai.*` namespace surface with the A6/B6 taxonomy, without changing flight
behavior.

B7 starts in implementation mode, not research mode.  The research framing is
already in A6/B6.  B7 should make the runtime easier to inspect, compare, and
reason about while preserving the stable B5/s207 mission.

## Non-Breaking Rule

Do not retune or rewrite flight behavior in B7.  Any change that touches
`sentai.servo`, `sentai.calib`, `sentai.markers`, `sentai.flow`,
`sentai.pipeline`, or `sentai.crazy` must preserve the ability to rerun the
stable migrated B4/B5 mission.

## Single Source Rule

There should be one source implementation for a `sentai.*` namespace.  The
canonical code lives under `examples/sentai_runtime`.  SIM should include the
same binding/runtime files whenever possible.

Allowed SIM-specific code:

- minimal platform backends for OS/device differences, such as POSIX sockets,
  host filesystem roots, simulator camera intake, or USB/Coral host adapters;
- compile-time branches guarded by explicit platform macros such as
  `SENTAI_PLATFORM_SIM`;
- small shims that preserve the same C ABI as the ARM backend.

Not allowed:

- duplicate namespace bindings with divergent Python API behavior;
- separate SIM algorithms for logic that should be shared;
- silently simplified controller/model behavior unless explicitly documented as
  a simulator stub.

The desired direction is: **shared binding and runtime logic, platform-specific
backend only where hardware or OS access differs**.

## SIM File Naming Convention

B7 treats file names as architecture documentation.  A SIM file name should
say which platform boundary it implements, not which high-level algorithm it
happens to feed.

The target convention is FreeRTOS-port-like: shared SentAI runtime code owns
the behavior, while `sim/` contains only injected platform implementations.
So `sentai_sim` files should read as ports/backends/adapters, not alternate
runtime modules.

Current platform-specific families are only:

| Family | Why SIM differs | Naming rule |
| --- | --- | --- |
| camera | SIM frames come from Gazebo or virtual FS images instead of OV5640/CSI | `sentai_camera_sim_backend.*`, plus narrowly scoped `*_bridge.*` for external sockets |
| TPU | SIM talks to the Coral USB device through a POSIX/libusb backend instead of the ARM USB host stack | `sentai_tpu_posix_backend.*` |
| UART/CRTP/link | SIM uses POSIX UDP/sockets instead of board UART/CPX/radio transport | `sentai_mavlink_udp_bridge.*`, `sentai_crazy_crtp_udp_bridge.*`, or explicit `*_bridge.*` |
| system/HAL | SIM maps board HAL calls to POSIX/FreeRTOS-host behavior | `sentai_platform_sim_backend.*` |
| unsupported hardware | API exists for parity but cannot run on host yet | `sentai_<name>_sim_stub.*`, with explicit not-ready errors |

Anything above those boundaries should keep the canonical shared name under
`examples/sentai_runtime`.

Naming meanings:

- `*_backend.*`: implementation injected behind a shared C ABI.  It should not
  expose a separate Python namespace.
- `*_bridge.*`: adapter to an external process/socket/device, such as Gazebo,
  PX4, Crazyflie SITL, or a helper process.
- `*_stub.*`: deliberate not-implemented backend that preserves namespace
  parity with clear error results.
- `modsentai_sim_*.c`: legacy name.  New code should not use it.  Existing
  files with this name should either disappear into shared
  `examples/sentai_runtime/bindings/modsentai_*.c` or be renamed to a backend,
  bridge, or stub once their boundary is clear.

Examples:

- `sentai.pipeline` should use the shared binding
  `examples/sentai_runtime/bindings/modsentai_pipeline.c`.
- Detection/NMS/tracking policy should stay in shared runtime code, not in a
  `*_sim_detection*` file.
- A SIM camera producer may be named `sentai_camera_sim_backend.c` or keep the
  historical `camera_bridge_recv.c` until renamed, because it is genuinely a
  simulator camera backend.
- A file-backed virtual camera is **not** a Gazebo feature.  It belongs in
  shared `sentai_runtime` because ARM and SIM can both use
  `sentai.camera.select(-1, path)` for deterministic image/video fixtures.
- A Gazebo camera is a simulator bridge/provider.  A future rename should make
  that explicit, for example `sentai_camera_gazebo_bridge.*` or
  `gazebo_camera_bridge.*`.
- An AirSim camera, if added later, should be another provider/bridge beside
  Gazebo, not a second camera pipeline: `sentai_camera_airsim_bridge.*`.
- The SIM TPU transport is `sentai_tpu_posix_backend.cc`: a direct
  POSIX/libusb backend behind the shared `sentai.tpu` ABI.  The historical
  `sim_tpu_shim.c`/PyCoral helper bridge has been removed from the runtime
  build; PyCoral remains host-side only for baseline comparison scripts.
- `sim/modsentai_sim_io.c`, `sim/modsentai_sim_rtos.c`, and
  `sim/modsentai_sim_sys.c` are historical binding fragments and should not
  exist after this B7 slice; their APIs should come from shared bindings and
  their behavior from `sentai_platform_sim_backend.c`.

Avoid names such as `sentai_detection_sim.c`: detection is intended to be
agnostic of camera and TPU.  If it needs platform support, inject that support
through camera/TPU backend functions and keep the detection pipeline shared.

## Step 1 - Root Namespace Inventory

Source references:

- ARM root namespace: `examples/sentai_runtime/modsentai.c`
- SIM root namespace: `sim/modsentai_sim.c`

### ARM `sentai.*`

Top-level functions:

```text
version, verbose, help, debug, console, run
```

Submodules:

```text
io, rtos, tpu, fs, camera, usb, uart, mesh, link, crazy, imu, mic, sleep,
pipeline, flow, aifes, kmeans, pca, anomaly, dtw, hmm, rl, slam, objects,
places, servo, object_lifter, calib, markers, safety, fr, explore, tfl, diag,
sys
```

### SIM `sentai.*`

Top-level functions:

```text
version, verbose, help, debug, console, run
```

Submodules:

```text
io, rtos, sys, fs, camera, flow, tpu, pipeline, link, uart, usb, mesh,
imu, mic, sleep, kmeans, pca, anomaly, dtw, hmm, rl, slam, objects, places,
servo, object_lifter, calib, markers, safety, fr, explore, tfl, crazy, aifes
```

## Initial Parity Findings

### Shared Runtime Substrate

These namespaces are present on both ARM and SIM and form the safe starting
point for A7/B7 introspection:

```text
io, rtos, diag, sys, fs, camera, flow, tpu, pipeline, link, uart, usb, mesh,
imu, mic, sleep, kmeans, pca, anomaly, dtw, hmm, rl, slam, objects, places,
servo, object_lifter, calib, markers, safety, fr, explore, tfl, crazy, aifes
```

These cover the main A6/B6 substrate:

| Taxonomy role | Shared namespace |
| --- | --- |
| View producer | `camera`, `pipeline` |
| Flow / localization input | `flow` |
| Model substrate | `objects`, `object_lifter`, `places`, `slam` |
| Controller surface | `servo`, `crazy`, `calib`, `markers` |
| Evidence | `fr` |
| Safety / health | `safety`, `sys`, `rtos`, owner-local `*.stats()` |
| Mission FSM precedent | `explore` |
| Helper math / learning substrate | `kmeans`, `pca`, `anomaly`, `dtw`, `hmm`, `rl` |
| ML experiment substrate | `tpu`, `tfl`, `aifes` |
| Board IO / operator channels | `uart`, `usb`, `mesh` |
| Board local sensors / power modes | `imu`, `mic`, `sleep` |

### ARM-Only Namespaces

```text
none
```

All ARM root namespaces are now present in SIM.

Important caveat: `aifes` is still a namespace-parity stub in SIM.  `sentai.tfl`
is no longer a stub: SIM now links the same `sentai_tfl_bridge.cc` TFLite Micro
CPU-only backend used by ARM, backed by `libs_tensorflow_sim`.  This is separate
from `sentai.tpu`: SIM now has a working EdgeTPU path through the direct
POSIX/libusb backend and the same USB Coral device, proven by s209.

### SIM-Only Namespace

```text
none
```

`sentai.sim` was removed during B7.  SIM-only support now belongs in backend,
bridge, or host-side experiment code.  Runtime observability goes through
`sentai.fr`.

## B7 Implementation Tasks

Status after the current B7 slice:

- namespace inventory and model-substrate smoke scaffolding exist;
- SIM now shares more runtime surface with ARM and has fewer divergent stubs;
- virtual camera is implemented as a shared ARM/SIM source selected through
  `sentai.camera.select(-1, path)`;
- s209 proves host PyCoral and SIM/direct-USB EdgeTPU parity on the same model and
  image;
- s211 proves SIM `sentai.tfl` CPU/TFLite Micro execution using the COCO
  non-EdgeTPU model and the same 640x480 cat BMP through `load_image()`;
- SIM PrepTask/camera-bridge now publishes a TPU-ready RGB prep slot when a
  model input tensor is known, and SIM `sentai.pipeline` consumes only that
  slot; the pipeline no longer has a local image-resize fallback;
- SIM `sentai.tpu` now uses the shared ARM binding table
  `bindings/modsentai_tpu.c`; the old SIM-only binding file was removed, so
  TPU API parity is no longer split across two Python surfaces;
- `sentai.pipeline.once(timeout_ms, conf, iou, max)` now exists as a bounded
  one-shot diagnostic wrapper over the same shared `PrepTask` / `InferTask`
  pipeline.  It starts the continuous pipeline only if needed, waits for one
  result, then stops only if it started the worker itself; this avoids creating
  a second inference path or renaming `InferTask` into a TPU transport task.
- SIM `sentai.link` now uses the shared ARM binding table
  `bindings/modsentai_link.c`; the POSIX/PX4 transport remains in
  `sim/sentai_mavlink_udp_bridge.cc` as a backend.  The shared binding gained optional
  backend hooks for the SIM PX4 conveniences that were previously trapped in
  `modsentai_sim_link.c`: `stats`, `arm`, `takeoff`, `land`, `send_flow`, and
  `flow`.
- SIM `sentai.fs` now uses the shared ARM binding table
  `bindings/modsentai_fs.c`; POSIX path resolution lives in
  `sim/sentai_fs_sim_backend.c`;
- SIM flow was moved onto the shared `examples/sentai_runtime/flow_task.cc`.
  The same FlowTask source now builds on ARM and SIM.  SIM keeps only
  `sim/sentai_flow_shared_storage_sim.c` as the backing storage symbol for
  `FLOW_SHARED()`, because POSIX cannot dereference the RT1176 fixed shared
  memory address.
- TODO: decouple the ARM CSI ISR from `FlowTask`.  Today
  `CSI_IRQHandler` directly notifies the flow publisher when a sensor frame
  completes.  That is too tightly coupled to one consumer.  The target B7/A7
  shape is a generic camera/prep publication boundary: camera completion makes
  a frame available, `PrepTask` produces the enabled static resize/gray/tensor
  slots, and `FlowTask`, marker tasks, TPU/inference, and future consumers wake
  from that shared frame/prep-slot event model rather than being hardwired into
  the CSI ISR.
- `sentai_prep` now has an explicit `SENTAI_PREP_SLOT_FLOW_GRAY_80x60`
  static slot, and the SIM camera producer publishes the current 80x60 flow
  gray image into that slot when enabled.  SIM FlowTask consumes this slot and
  publishes flow state, so TPU/pipeline execution no longer owns or interferes
  with `sentai.flow`.
- `sentai.flow.read()` in SIM calls a bounded `sentai_flow_poll_once()` before
  reading `FLOW_SHARED()`.  This keeps short REPL/radio smoke tests
  deterministic even when the POSIX scheduler has not yet run the background
  FlowTask after a virtual-camera frame change.
- s209 now includes a virtual-camera flow smoke.  It starts `sentai.flow` on
  camera `-1`, selects the same 640x480 BMP twice, verifies zero motion, then
  selects a BMP offset by 2 px and verifies that phase-correlation reports a
  non-zero motion signal.  Current passing run after shared FlowTask migration:
  `s209_virtual_camera_tpu_e2e/iter79_virtual_camera_flow_static_shift`
  (`static_same: dx=0 dy=0`, `shifted: dx=243 dy=19`, `cam_id=-1`).
- legacy `sentai.sim.journal_*` was removed; use `sentai.fr` for runtime
  journaling/debug artifacts;
- SIM and ARM have a `sentai.fr` debug channel path for printf/debug traces;
- mission-source reproducibility is now a rule: run MP missions from files
  staged into the per-iteration FS root, with stdin only bootstrapping import
  and `run()`.

## B7 Pause / Dead-End Note - 2026-06-02

B7 is paused here.  The namespace/refactor slice produced useful work, but the
current virtual-camera + pipeline + TPU/Flow validation path is a dead end for
now and should not be pushed further without stepping back to a simpler known
good baseline.

What is known:

- Virtual camera replay into PrepTask without FlowTask and without TPU was
  stable across repeated SIM runs.  Passing runs:
  `s209_virtual_camera_tpu_e2e/iter368_prep_single_replay_no_flow_rerun1`
  through `iter372_prep_single_replay_no_flow_rerun5`; each published and
  consumed 30 frames with zero producer overruns.
- Starting FlowTask on the new SIM path exposed instability across reruns.
  Earlier Gazebo/B3/B4-style SIM runs did not prove this exact path; they used
  the older camera bridge/read-only flow shape and therefore are not a
  reliable proof that the new `flow.start(-1)` / virtual-camera / PrepTask-slot
  coupling is correct.
- Removing FlowTask did not fully recover the TPU pipeline path.  The current
  no-flow TPU smoke (`iter373_pipeline_pycoral_single_replay_no_flow_smoke`)
  loaded the COCO EdgeTPU model, ran one inference, queued one detection frame,
  then failed to complete the mission/shutdown.  So the latest blocker is not
  only `flow.start()`.
- The suspected fragile areas are lifecycle and ownership boundaries rather
  than raw compute: direct tensor buffer release between PrepTask and
  InferTask, virtual-camera frame-slot return/reuse, blocking waits from MP
  while POSIX helper/backend I/O is active, and the half-migrated FlowTask
  event model.

Decision for now:

- Stop B7 experimentation at this point.
- Do not treat the current virtual-camera + TPU + Flow path as a valid parity
  proof.
- Do not continue piling fixes onto this branch blindly.  The next attempt
  should restart from a known-good minimal baseline and reintroduce pieces in
  this order: camera provider queue, PrepTask only, TPU pipeline without direct
  tensor, TPU pipeline with direct tensor, then FlowTask as an independent
  consumer of PrepTask output.
- Keep the architectural target: camera producers publish frames into one
  shared camera buffer/event model; PrepTask owns resize/gray/tensor
  transforms; FlowTask and InferTask are independent consumers that only run
  when enabled.

Remaining B7 gaps:

1. Extend and use the existing namespace inventory beyond root exports into
   function-level surfaces where future primitive wrappers need clarity.
2. Extend the existing read-only smoke checks for shared model substrates:
   `objects`, `object_lifter`, `places`, and optionally `slam`, without
   starting control loops or mutating flight state.
3. Review `aifes` dependency boundaries and replace the SIM stub with a real
   shared runtime or documented host backend.  `sentai.tfl` now has a real SIM
   backend, but it remains CPU/TFLite Micro and should not be used as the proof
   path for Coral USB EdgeTPU parity; that proof lives under `sentai.tpu` plus
   `sentai.pipeline`.
4. Validate and deepen the C++ task inventory for the runtime core.
5. Update documentation when parity decisions are made.
6. Run B5/s207 as regression after any runtime binding change.
7. Continue validating the direct POSIX/libusb EdgeTPU backend now wired into
   the REPL TPU backend.  PyCoral is host-side only.  `s209` must remain the
   parity proof against host PyCoral once raw output ordering/value parity is
   fully stable.
8. Continue removing SIM-only preprocessing paths.  `sentai.pipeline` no
   longer performs local resize/preprocess; remaining review target:
   `sentai.camera.grab_gray` still has a SIM-local diagnostic resize path and
   should become a PrepTask-slot consumer or be clearly marked diagnostic-only.
9. Continue the camera producer split around the new FlowTask contract.
   Implemented slice: `camera_bridge_recv.c` is now a Gazebo RGB socket bridge
   only, and the shared frame publication path moved to
   `examples/sentai_runtime/sentai_camera_frame_backend.c`.  That shared
   backend owns latest RGB, virtual-camera publication, PrepTask slot
   publication, and the FlowTask snapshot bridge.  Remaining cleanup:
   - rename transitional `sim_camera_*` ABI names once all consumers are
     switched to `sentai.camera` / `sentai.flow` names;
   - make PrepTask the single named owner for enabled transforms in docs/code
     comments, even when the current SIM producer calls the shared helper;
   - keep `sim/sentai_flow_shared_storage_sim.c` storage-only for
     `g_sentai_flow_shared`, matching ARM's fixed shared-memory role.
10. For the FreeRTOS POSIX port, keep host-side blocking I/O out of RTOS hot
   tasks.  Bridge threads/backends may block on POSIX sockets/USB, but runtime
   tasks should receive bounded notifications/queues and consume static slots.
   This follows the practical FreeRTOS POSIX constraint observed in B7:
   `vTaskDelay`/blocking waits inside short REPL command paths can behave
   differently than on ARM, so SIM command paths should prefer bounded
   host-side waits only at the backend boundary and deterministic poll points
   for REPL queries.
11. Refactor the old `modsentai_sim_*` binding fragments into shared runtime
   bindings/backends in this order:
   - `sim/sentai_tpu_posix_backend.cc`: keep tightening the direct POSIX/libusb
     backend wired to the shared `EdgeTpuManager`/`TpuDriver` model execution
     path; keep PyCoral only for host-side baseline checks.
   - `examples/sentai_runtime/bindings/modsentai_camera_sim.c`: temporary
     SIM-specific camera binding moved out of `sim/`; next step is folding it
     into `bindings/modsentai_camera.c` behind `SENTAI_PLATFORM_SIM`, while
     keeping Gazebo/AirSim-style producers as SIM bridges and file-backed
     virtual camera in shared runtime.
   - `sim/modsentai_sim_crazy.c`: replace the SIM-specific binding with the
     shared `bindings/modsentai_crazy.c`; keep CRTP-over-UDP in
     `sentai_crazy_crtp_udp_bridge.cc` as the injected backend.
   - `sim/camera_bridge_recv.c`: split simulator socket intake from shared
     PrepTask/resize/flow publishing logic so preprocessing remains owned by
     `PrepTask`.
   - `sim/sentai_uart_serial_udp.c`, `sim/sentai_mavlink_udp_bridge.cc`,
     `sim/fx_user_stubs.c`, and `sim/sentai_flow_shared_storage_sim.c`: audit
     for naming and ownership after the three high-priority bindings above.
   bindings plus thin SIM platform backends.  First safe slice:
   `io`, `rtos`, and `sys`.  `diag` should not be ported as one large common
   namespace.  Decompose it into owner namespaces first.

### Diagnostic Namespace Direction

B7 should avoid preserving `sentai.diag` as a "god namespace".  The useful
diagnostic/status functions should live beside the subsystem that owns the
state.  `sentai.fr` remains the recorder; owner-local status APIs are the
interactive REPL diagnostics.

Proposed migration map:

| Current `sentai.diag` area | Target owner namespace | Rationale |
| --- | --- | --- |
| `health`, `sys_mode`, boot/crash status | `sentai.sys` and `sentai.safety` | System state belongs with system/safety, not a separate bag of diagnostics. |
| `dmesg`, `dmesg_stats`, `dmesg_clear`, `repl_kick` | `sentai.rtos` | Runtime/REPL supervision is OS-level.  Implemented as `sentai.rtos.dmesg*` and `sentai.rtos.repl_kick()` in SIM build #893. |
| `heap_info`, task list, CPU usage | already `sentai.rtos` | Keep there; do not duplicate in `diag`. |
| `cam_stats` | `sentai.camera.stats()` | Camera owns switch/grab/retry counters. |
| `lfs_stats`, `fx_stats`, `fx_bench`, `fx_format`, cache helpers | `sentai.fs.stats()`, `sentai.fs.bench()`, `sentai.fs.cache_*` | Storage owns storage health and cache/debug helpers. |
| `tpu_perf`, `async_stats`, `tpu_call_stats`, `tpu_chunk_size`, `tpu_zero_copy`, `tpu_trace`, `tpu_urb_timeout`, `tpu_multi_ep`, `tpu_desc_cache` | `sentai.tpu.stats()`, `sentai.tpu.config(...)`, or specific `sentai.tpu.*` verbs | TPU counters and tunables should be discoverable where TPU commands live. |
| `aruco_bench` | `sentai.markers.bench()` | ArUco/WhyCon diagnostics belong to the marker subsystem. |
| `m4_aruco_bench`, `m4_bench_diag` | explicit future backend namespace or remove from SIM | M4 is board/platform-specific; do not force it into SIM. |
| `storage_log` | `sentai.fr` debug channel or `sentai.fs.read('/log/...')` | Logs are artifacts, not a parallel diagnostic API. |
| `flexram_info` | `sentai.rtos.mem()` or `sentai.sys.memory()` | Memory partitioning is platform/system metadata. |

Migration rule:

- new diagnostics go into the owner namespace, not `sentai.diag`;
- `sentai.diag` is no longer registered by the SIM or ARM root module;
  the historical `bindings/modsentai_diag.c` file was removed and old helpers
  should be mined from git history into owner namespaces only when needed;
- SIM should not grow a large `sentai_diag_sim_backend.c` full of board-only
  stubs.  If a status has no meaningful SIM backend, the owner namespace should
  return a clear not-supported result;
- after owner-local APIs exist, remove or reduce `sentai.diag` rather than
  keeping two ways to ask the same question.

### Current Slice - SIM Binding Fragment Cleanup

Goal: keep the Python `sentai.*` API in one canonical source file under
`examples/sentai_runtime`, while SIM provides only POSIX/x86 backend hooks.

Implementation order:

1. Move `sentai.io`, `sentai.rtos`, and `sentai.sys` in SIM from historical
   `sim/modsentai_sim_*.c` fragments to the shared binding files:
   `bindings/modsentai_io.c`, `bindings/modsentai_rtos.c`, and
   `bindings/modsentai_sys.c`. **Done in build #887.**
2. Put the SIM-specific behavior in `sim/sentai_platform_sim_backend.c`:
   LED prints, FreeRTOS POSIX sleep/ticks, no-op REPL watchdog kick,
   recovery-mode defaults, and clean simulator exit for
   `sentai.sys.reset()`. **Done.**
3. Remove the old SIM `sentai.diag` fragment instead of porting the
   god-namespace. **Done in build #893.**  The useful runtime log/REPL pieces
   moved to `sentai.rtos.dmesg()`, `sentai.rtos.dmesg_stats()`,
   `sentai.rtos.dmesg_clear()`, and `sentai.rtos.repl_kick()`.
   `sentai.camera.stats()` now owns the former camera fault counters, and
   SIM exposes the same method with virtual-camera/backend status.  The
   inactive `bindings/modsentai_diag.c` file was removed; future recovery of
   old diagnostics should be done from git history into owner namespaces.
4. Move `sentai.fs` from the historical SIM fragment to the shared
   `bindings/modsentai_fs.c`, with `sim/sentai_fs_sim_backend.c` providing the
   POSIX/fs-root port layer. **Done in build #889.**
5. Remove legacy `sentai.sim.journal_*`. **Done.**  `sentai.fr` is the
   canonical runtime journal/debug recorder; old experiments that used
   `sentai.sim.journal_*` should be refreshed rather than keeping a parallel
   SIM-only journal API.
6. Later slices should refactor the remaining historical SIM fragments in
   dependency order: `camera`, `flow`, and `crazy`.  `link` is done: the
   Python binding is shared, while `sentai_mavlink_udp_bridge.cc` is the SIM backend.
   ARM
   diagnostics that still matter should be restored from git history into
   `sentai.sys`, `sentai.fs`, `sentai.tpu`, and `sentai.markers`.  Do not
   recreate `sentai.diag`.

Rule for the later slices: if a fragment contains real algorithm/pipeline
logic, move that logic to shared `sentai_runtime`; if it contains only a
platform boundary such as POSIX sockets, host FS root, or simulator transport,
keep it in SIM as a backend with a clear name.

Verification for this slice:

```text
cmake --build build-sim --target sentai_sim -j2
[100%] Built target sentai_sim

SIM REPL smoke:
sentai.io.led_on()
sentai.rtos.ticks_ms()
sentai.rtos.tasks()
sentai.sys.boot_attempts()
sentai.sys.recovery_mode()

python3 examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/run_s209.py
OK s209 .../iter37_virtual_camera_tpu_cat
```

FS/journal cleanup verification:

```text
cmake --build build-sim --target sentai_sim -j2
[100%] Built target sentai_sim

SIM REPL smoke:
hasattr(sentai, "sim") == False
sentai.fs.write/read/read_str/append/size/exists/ls/sync all pass
sentai.fr.stats("debug") returns a valid stats tuple

python3 examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/run_s209.py
OK s209 .../iter40_virtual_camera_tpu_cat
```

SIM/ARM diag namespace decomposition verification:

```text
cmake --build build-sim --target sentai_sim -j2
[100%] Built target sentai_sim

SIM REPL smoke:
hasattr(sentai, "diag") == False
hasattr(sentai.rtos, "dmesg") == True
sentai.rtos.dmesg_stats() == (0, 0) on a fresh FS root
sentai.rtos.repl_kick() returns None
sentai.camera.stats() returns owner-local camera/backend status

Code check:
SIM and ARM root modules no longer register MP_QSTR_diag, and the historical
modsentai_diag binding file is deleted.

python3 examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/run_s209.py
OK s209 .../iter45_virtual_camera_tpu_cat
```

Artifact check for the same run:

- `fs_root/fr/debug.log`, `events.csv`, and `scalars.csv` are present.
- `fs_root/mission_s209.py` is present; stdin only imports and runs the mission.
- `fs_root/images/cat_640x480.bmp` is the virtual camera input image.
- `fs_root/images/detected_overlay.bmp` is produced by sentai_runtime on the
  virtual FS; host PNG overlays are kept beside the run for visual inspection.
- `fs_root/detections.txt`, `host_detections.txt`, and `sim_detections.txt`
  match one-to-one for the COCO cat baseline.

### Detection / Pipeline REPL Modes

The historical ARM tests used three useful modes.  B7 should preserve the same
shape in SIM, but with SIM-specific devices injected behind the shared runtime
APIs.

1. Single raw TPU detection:

   ```python
   sentai.camera.select(-1, "/images/cat_640x480.bmp")
   sentai.camera.to_tensor()
   sentai.tpu.invoke()
   sentai.pipeline.detections(100)
   sentai.pipeline.save("/images/detected_overlay.bmp")
   ```

   This is the direct diagnostic path: one prepared tensor, one invoke, then
   decode the current TPU outputs.  For TPU/backend smoke tests only, the image
   can be loaded directly with `sentai.tpu.load_image(path)` before
   `sentai.tpu.invoke()`, but that bypasses the camera/PrepTask path.

2. One detection through the pipeline:

   ```python
   sentai.camera.select(-1, "/images/cat_640x480.bmp")
   sentai.pipeline.once(2000, 0.25, 0.45, 50)
   ```

   This is the preferred one-shot pipeline diagnostic because it still goes
   through the shared `PrepTask` / `InferTask` path and has a bounded timeout.

3. Continuous pipeline / calibration:

   ```python
   sentai.pipeline.start(0.25, 0.45, 50, False)
   sentai.pipeline.get_ex(2000)  # optional warmup/drain
   # repeat bounded get_ex calls for N frames
   sentai.pipeline.stop()
   ```

   For longer benchmarks, use board-side helpers such as
   `sentai.pipeline.calibrate(model, frames, timeout_ms, ...)` or a staged MP
   mission file.  The host should not drive one REPL command per frame.

Rule: compute-intensive work stays in C++ tasks.  MP is only the high-level
command/orchestration layer: load/select/start/status/stop.  A test mission may
loop in MP to switch deterministic virtual-camera frames and collect status,
but resize, flow, inference, NMS, tracking, and buffer movement must stay in
`PrepTask`, `FlowTask`, `InferTask`, or owner-local C++ code.

s209 follows this rule after B7 cleanup: stdin only imports
`mission_s209_flow_tpu` and calls `run()`.  The mission runs on the virtual FS,
starts the shared tasks, selects deterministic 640x480 BMP frames, collects
bounded status, and writes artifacts back to the same FS root.

Current SIM scheduling caveat: `PrepTask`, `InferTask`, and `FlowTask` do run
as FreeRTOS POSIX tasks, but REPL/MP command paths must not rely on long
`sleep_ms()` or `pipeline.get_ex(timeout)` waits while USB TPU work is active.
The stable diagnostic pattern for now is bounded non-blocking polling
(`get_ex(0)` / owner-local status calls) with tiny `repl_kick()` backoff.  The
next cleanup target is a proper SIM command/executive wait primitive so MP can
start a long-running operation, return to REPL, and query/abort it without
holding the interpreter inside a blocking wait.

ARM check:

- `cmake --build build -j2` rebuilt `examples/sentai_runtime/libmicropython`
  with `sentai.diag` removed from the root module and `sentai.camera.stats()`
  added.  The full build then failed later in the pre-existing WICED/NXP
  `vApplicationStackOverflowHook` declaration conflict, outside this refactor.

Link binding cleanup verification:

```text
cmake --build build-sim --target sentai_sim -j2
[100%] Built target sentai_sim

SIM REPL smoke:
hasattr(sentai.link, "stats") == True
hasattr(sentai.link, "arm") == True
hasattr(sentai.link, "send_detection") == True
hasattr(sentai.link, "receive") == True
sentai.link.stats() == (0, 0, 0, 0, 0, 0, 0, 0, 0) before init
sentai.link.available() == 0

python3 examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/run_s209.py
OK s209 .../iter46_virtual_camera_tpu_cat
```

## Critical Point - C++ Task Inventory

B7 must also clarify the C++ task layer, because this is the real runtime core
under the `sentai.*` namespace.  Namespace parity alone is not enough if we do
not know which background loops are producers, which are consumers, and how
they are initialized.

The inventory should document each `*Task` or task-like worker with:

- name and source file;
- namespace/API that starts or observes it;
- initialization path and required prerequisites;
- produced data, consumed data, and ownership of buffers;
- start/stop/status/error contract;
- whether it runs on ARM, SIM, or both;
- whether it is canonical shared code or a platform backend;
- logging/FR events emitted by the task;
- shutdown and failure behavior.

Known task families to catalog first:

| Task / worker family | Current role to document |
| --- | --- |
| `PrepTask` / camera bridge producer | central image preprocessing, shared frame slots, flow/SLAM/detection input |
| `InferTask` / detection loop | TPU/object inference consumer of preprocessed frames |
| `flow_task` | optical-flow publisher or PrepTask-integrated flow producer |
| `slam_task` | model-substrate consumer of PrepTask `SLOT_RGB_64` |
| `sentai_safety_task` | safety monitor and abort latch |
| `sentai_fr_task` | flight-recorder drain task |
| `sentai_calib_orientation_task` | migrated B3 calibration/orientation state machine |
| `sentai_servo_marker_task` | migrated B4 marker-control state machine |
| `sentai_calib_task` | legacy calibration/autotune worker to mark for cleanup or compatibility |
| `sentai_crazy` RX/CMD tasks | Crazyflie CRTP bridge and setpoint streaming |
| `sentai_mesh` RX task | mesh-radio text/protobuf receive loop |
| SIM bridge tasks | simulator camera, link, flow-forwarder, and CRTP/MAVLink workers |

The architectural rule for this inventory is the same as the single-source
rule: expensive image preprocessing should happen once, preferably in
`PrepTask` or its SIM equivalent, and the rest of the runtime should consume
shared slots or pointers.  B7 should explicitly flag duplicate preprocessing,
extra memcpy, or SIM-only reimplementations as gaps to fix later.

## Task Inventory - First Pass

This first pass is descriptive, not a refactor.  It records what exists so the
next B7 slices can align names and ownership without changing B5/s207 flight
behavior.

| Task / worker | Source | Platform | API / namespace | Role |
| --- | --- | --- | --- | --- |
| `det_prep` / PrepTask | `examples/sentai_runtime/detection_task.cc` | ARM/SIM shared runtime | camera/detection runtime | Canonical camera producer for the detection pipeline.  Grabs camera frames through platform hooks, performs PXP-style resize / quantization, can write directly into the TPU input buffer, publishes prep slots, and signals consumers. |
| `det_infer` / InferTask | `examples/sentai_runtime/detection_task.cc` | ARM/SIM shared runtime | TPU/object detection runtime | Consumer of PrepTask output.  Invokes TPU through platform hooks and publishes detection/NMS results. |
| `camera_bridge` | `sim/camera_bridge_recv.c` | SIM | simulator camera backend | SIM equivalent of PrepTask producer.  Receives Gazebo frames, publishes flow snapshots, latest RGB, and prep slots, including TPU RGB when requested.  Platform backend, not a separate algorithm policy. |
| `sentai.pipeline` binding | `examples/sentai_runtime/bindings/modsentai_pipeline.c` | ARM/SIM shared binding | `sentai.pipeline.start/stop/get/get_ex/detections/save/stats` | Shared MP surface over `detection_task.cc`.  SIM no longer has `modsentai_sim_pipeline.c`; simulator differences are injected through camera and TPU backend hooks. |
| `flow_pub` | `examples/sentai_runtime/flow_task.cc` | ARM | `sentai.flow.start/stop/read` | ARM optical-flow publisher.  Grabs camera frames, PXP-scales to 80x60, computes flow, publishes `flow_shared_t`.  Historical M7-only path; B7 should compare it with PrepTask/camera_bridge data ownership. |
| `slam` | `examples/sentai_runtime/slam_task.cc` | ARM/SIM | `sentai.slam.start/stop/current/stats` | Consumer of PrepTask `SENTAI_PREP_SLOT_RGB_64`.  Computes place descriptors and publishes compact match state. |
| `sentai_safety_task` | `examples/sentai_runtime/sentai_safety_task.*` | ARM/SIM-style FreeRTOS/POSIX | `sentai.safety` worker API | Safety monitor worker.  Consumes camera frames zero-copy, runs ArUco check, pushes results into the safety state machine. |
| `sentai_fr_task` | `examples/sentai_runtime/sentai_fr_task.*` | ARM/SIM-style FreeRTOS/POSIX | `sentai.fr` worker API | Flight-recorder drain task.  Keeps FR queue draining on a fixed cadence without exposing channel internals to missions. |
| `sentai_calib_orientation_task` | `examples/sentai_runtime/sentai_calib_orientation_task.*` | ARM/SIM runtime | `sentai.calib.orientation_*` | Migrated B3 orientation/calibration state machine.  MP mission starts phases and reads status/result/current thrust. |
| `sentai_servo_marker_task` | `examples/sentai_runtime/sentai_servo_marker_task.*` | ARM/SIM runtime | `sentai.servo_marker_*` via `sentai.servo`/mission bindings | Migrated B4 marker-control state machine.  Owns acquire, center hold, extpos warmup, handoff hover, axis motion, and landing phases. |
| `sentai_calib_task` | `examples/sentai_runtime/sentai_calib_task.cc` | ARM/SIM runtime | legacy calib/autotune API | Legacy calibration/autotune worker.  Mark for compatibility review after B3/B4 task migration. |
| Crazyflie bridge tasks | `examples/sentai_runtime/sentai_crazy.*`, `sim/sentai_crazy_crtp_udp_bridge.cc` | ARM/SIM backend-specific | `sentai.crazy`, link backend | CRTP receive and command/setpoint streaming loops.  Platform-specific transport, shared semantic surface. |
| Link / mesh tasks | `examples/sentai_runtime/sentai_mesh.*`, `sim/sentai_mavlink_udp_bridge.cc` | ARM/SIM backend-specific | `sentai.link`, `sentai.mesh` | Radio/UART/protobuf receive, heartbeat, and SIM flow forwarding.  Important for future REPL/prime command channel. |
| Runtime watchdog / HAL tasks | `examples/sentai_runtime/sentai_runtime.cc`, `examples/sentai_runtime/modsentai_hal.cc` | ARM | `sentai.sys`, HAL/internal | Hardware watchdog, recovery watchdog, button, IMU/tap event loops.  Mostly platform supervision, not mission primitives. |

## PrepTask Data-Flow Contract

`PrepTask` is the most important B7 architectural boundary.

- ARM producer: `detection_task.cc:prep_task_fn`, task name `det_prep`.
- SIM producer: `camera_bridge_recv.c:camera_bridge_task`, task name
  `camera_bridge`.
- Shared contract: `examples/sentai_runtime/sentai_prep.h`.
- Canonical slots:
  - `SENTAI_PREP_SLOT_GRAY_NATIVE`: Y8 native/scaled view;
  - `SENTAI_PREP_SLOT_RGB_64`: RGB888 64x64 view;
  - `SENTAI_PREP_SLOT_GRAY_64`: Y8 64x64 view;
  - `SENTAI_PREP_SLOT_TPU_RGB`: RGB888 model-input view, committed with the
    actual tensor dimensions inside a 640x480 statically allocated maximum.
- Ownership rule: producer writes, commits, and bumps sequence; consumers read
  latest finalized slots or compact snapshots.
- Consumer rule: consumers must not move full frames through MP heap and must
  not redo expensive per-pixel preprocessing when a prep slot already exists.
- Scheduling rule: signals are coalesced where appropriate.  Consumers should
  tolerate last-frame-wins semantics rather than building unbounded queues.

## Camera Backend Refactor Direction

`camera_bridge` should be treated as the SIM backend of `sentai.camera`, not as
a separate perception pipeline.

Current state:

- `sim/camera_bridge_recv.c` receives Gazebo frames, stores the latest RGB
  frame, computes SIM flow, and publishes PrepTask slots.
- `examples/sentai_runtime/bindings/modsentai_camera_sim.c` exposes the
  temporary SIM `sentai.camera` MP namespace
  and reads latest RGB through `sim_camera_latest_rgb(...)`.
- `sentai.pipeline`, `sentai.flow`, safety, markers, and SLAM already depend
  on camera-like C hooks rather than directly on Gazebo.

Desired B7 direction:

- keep Gazebo/socket code as a platform backend detail, but put the ownership
  under `sentai.camera`;
- expose a small C backend contract such as "latest RGB/XRGB frame",
  "frame seq/timestamp", and "active source id";
- keep `sentai.camera.select(-1, path)` as the file-backed virtual source and
  `sentai.camera.select(0/1)` as physical/simulated camera sources;
- make PrepTask the only owner of expensive resize/grayscale/model-input
  production from that camera frame;
- make Flow, Pipeline, Markers, Safety, SLAM, and TPU consumers of camera or
  PrepTask contracts, not consumers of `camera_bridge` internals.

So the refactor target is not to move all bridge code into the Python
`sentai.camera` binding file.  The target is to rename and split it so the
SIM-specific socket receiver is clearly a camera backend, while preprocessing
and consumer policy stay outside the camera namespace.

Known consumers today:

| Consumer | Data consumed | Notes |
| --- | --- | --- |
| InferTask | direct TPU tensor or staging buffer from PrepTask | ARM path already has direct tensor handoff to avoid memcpy. |
| SlamTask | `SENTAI_PREP_SLOT_RGB_64` | Good model-substrate pattern: C++ consumes slot, MP sees compact status. |
| SafetyTask | camera zero-copy gray frame | Should be reviewed against prep slots, but current design intentionally avoids MP copies. |
| Flow | ARM `flow_pub` currently owns its own camera/PXP path; SIM camera bridge computes flow | This is the main duplicate-preprocessing candidate to review later, without retuning B5. |
| Detection pipeline | camera backend hooks -> PrepTask/direct tensor -> TPU backend hooks | Shared `detection_task.cc` now owns the pipeline on ARM and SIM.  SIM no longer has a separate `sentai.pipeline` algorithm; it injects only camera and TPU backend behavior. |

## B7 Gaps From Task Inventory

1. `flow_pub` and SIM `camera_bridge` both perform image scaling/gray
   preparation.  This is now the most important remaining dataflow mismatch:
   flow should either become a PrepTask consumer or publish through the same
   shared `flow_shared_t` contract from a clearly documented producer.
2. SIM `sentai.pipeline` no longer has a local resize fallback.  It is now a
   PrepTask slot consumer.  Remaining cleanup is to move the old SIM
   `sentai.camera.grab_gray` resize helper onto PrepTask slots too, or mark it
   as a diagnostic-only escape hatch.
3. `sentai_calib_task` appears legacy after B3/B4 migration.  Mark exact APIs
   and users before deleting anything.
4. `aifes` is still a namespace-parity stub in SIM.  `tfl` has moved beyond
   namespace parity: it now runs the shared CPU-only TFLite Micro backend on
   x86, proven by s211.
5. SIM EdgeTPU now uses the direct POSIX/libusb backend in the runtime path.
   The direct path preserves the existing `TpuDriver` C++ driver and replaces
   only the board-specific NXP USB Host transport.  Remaining work is output
   parity and device recovery hardening, not helper replacement.
6. The source namespace inventory check now has function-level mode.  Current
   large mismatches are not hidden: `flow`, `pipeline`, `camera`, `rtos`,
   `diag`, `sys`, `link`, and `crazy` still need API-level decisions.  `tpu`
   has already moved back to the shared binding surface.
7. The first task lifecycle vocabulary is documented below; it should be kept
   lightweight and refined only when implementation pressure proves a gap.

## REPL TPU / Image Recognition Target

The TPU migration target is not merely `TpuDriver::Initialize()`.  The early
operator-loop sketch was file-based and did **not** use the camera:

```python
import sentai
sentai.camera.select(-1, '/images/cat_640x480.bmp')
sentai.tpu.load('/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite')
sentai.tpu.load_image('/images/cat_640x480.bmp')
sentai.tpu.invoke()
sentai.pipeline.detections(300)
sentai.pipeline.save('/images/detected_overlay.bmp')
```

The implemented B7 shape is cleaner than that sketch: command the runtime from
the REPL using the virtual/board filesystem, select a file-backed virtual
camera, run the shared TPU/pipeline postprocess path, and read compact
detections or generated FS artifacts.  This avoids moving image byte arrays
through MP:

```python
sentai.camera.select(-1, '/images/cat_640x480.bmp')
sentai.tpu.load_image('/images/cat_640x480.bmp')
sentai.tpu.invoke()
sentai.pipeline.detections(300)
sentai.pipeline.save('/images/detected_overlay.bmp')
```

Camera id `-1` means "static image from FS".  Existing code that already
consumes the camera interface can run through the normal continuous pipeline:
TPU preprocessing, detectors, trackers, and future test pipelines all see a
normal camera frame.  The current s209 regression intentionally uses the
synchronous `tpu.load_image/invoke` path because the shared continuous
`det_prep/det_infer` pipeline still has a SIM/POSIX scheduling gap after the
SIM-only pipeline binding was deleted.  That gap is now explicit rather than
hidden behind a separate simulator algorithm.

### Existing Namespace Surface

What already exists:

| Namespace | Relevant API | Current status |
| --- | --- | --- |
| `sentai.fs` | `write`, `read`, `exists`, `size`, `ls`, `mkdir`, `remove` | Good enough for virtual FS model/image staging in SIM and board FS staging on ARM. |
| `sentai.tpu` ARM binding | `load`, `load_image`, `invoke`, `ready`, raw output access, quantization, rows/values, `save_output`, slot APIs | Correct REPL shape for EdgeTPU.  This is the API surface to preserve. |
| `sentai.tpu` SIM binding | shared ARM binding table: `load`, `load_image`, `invoke`, `ready`, raw output access, quantization, rows/values, `save_output`, slot APIs | Uses direct POSIX/libusb backend glue over the same `EdgeTpuManager`/`TpuDriver` stack as ARM.  Runtime no longer depends on PyCoral or a helper process. |
| `sentai.tfl` | `load`, `set_input`, `load_image`, `invoke`, output access | CPU/TFLite Micro, not EdgeTPU.  SIM now links the real shared backend and passes a COCO cat smoke with the non-EdgeTPU model. |
| `sentai.pipeline` ARM/SIM | shared `start`, `stop`, `get`, `get_ex`, `once`, `detections`, tracker helpers, `save` | Decodes SSD MobileNet COCO-style 4-output tensors after TPU invoke.  SIM uses the same binding and shared detection task; `once(...)` is a bounded test wrapper over the continuous `PrepTask`/`InferTask` pipeline, while `save(path)` writes a runtime-generated BMP overlay to the SIM FS without exposing image bytes to MP. |

### Current Missing Pieces

1. **Direct POSIX TPU is wired to REPL.**  `sentai.tpu.load()` now runs through
   a static `tpu_posix` FreeRTOS task, opens Coral USB through libusb, and uses
   the shared CoralMicro `EdgeTpuManager`/`TpuDriver`/TFLM path.  No pthread
   worker or PyCoral helper exists in the SIM TPU backend; PyCoral remains
   allowed only in host-side baseline experiments.
2. **SIM `sentai.tpu` binding surface parity is now achieved.**  SIM includes
   the shared `modsentai_tpu.c`; backend support for `load_image`,
   `save_output`, slot introspection, `row`, `value`, `output_floats`, and
   `dump_eps` is provided by `sentai_tpu_posix_backend.cc`.  `dump_eps` reports
   the real Coral USB endpoint map from libusb.
3. **Image-from-FS preprocessing is implemented through virtual camera plus
   direct TPU `load_image`, with a shared one-shot pipeline wrapper now added
   for bounded worker tests.**  The B7-supported smoke path is
   `sentai.camera.select(-1, path)`, `sentai.tpu.load_image(path)`,
   `sentai.tpu.invoke()`, then shared `sentai.pipeline.detections/save`.
   The shared continuous `sentai.pipeline.start/get_ex/stop` path remains the
   production shape.  `sentai.pipeline.once(...)` is now available for tests
   that need a single detection result without permanently leaving the pipeline
   running.
4. **COCO detection decode exists on the TPU/pipeline path.**  `s209` now
   exercises `sentai.camera.select(-1, path)`, `sentai.tpu.load_image(path)`,
   `sentai.tpu.invoke()`, `sentai.pipeline.detections(...)`, and
   `sentai.pipeline.save(...)` against the direct POSIX/libusb backend.
   Remaining parity work: validate raw output values one-to-one against the
   host PyCoral baseline and keep `s209` green after USB device recovery.
   the EdgeTPU COCO model and host PyCoral baseline.  `sentai.tfl` is a
   separate CPU/TFLite Micro path; it can load/invoke the non-EdgeTPU COCO
   model, but it does not yet have the same semantic bbox decoder/parity check.
5. **`sentai.tfl.load_image` now exists for virtual-camera BMP input.**  It
   loads the board/SIM FS image through `sentai_virtual_camera`, resizes to the
   model input tensor in C++, and avoids MP image byte arrays.  PNG/JPEG and
   richer postprocessing remain follow-up work.  It is still not the EdgeTPU
   path and should not be used to prove Coral USB parity.

### Updated B7 Plan

The previous "next slice" is now partly implemented.  The plan from here is:

1. **Keep B5/s207 as the flight regression.**  Any runtime binding, task, or
   perception-dataflow change must preserve the migrated B4 mission behavior.
2. **Finish the task lifecycle audit.**  Validate the table below against the
   actual MP bindings and mark missing `status`, `done`, `abort`, `stop`, and
   readiness calls that would block future `sentai.prime` wrappers.
3. **Clean up PrepTask/dataflow ownership.**  Decide whether SIM
   `sentai.pipeline`, ARM `flow_pub`, markers, safety, and TPU should consume
   shared PrepTask slots or documented platform snapshots.  Flag duplicate
   resize/preprocessing/memcpy, but do not retune B5 while doing it.
4. **Keep the COCO cat baseline.**  Perception smokes should use:
   `/images/cat_640x480.bmp`,
   `/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite` for EdgeTPU, and
   `/models/tf2_ssd_mobilenet_v2_coco17_ptq.tflite` for CPU/TFLite Micro.
   Tiny models are allowed only for link/kernel sanity tests.
5. **Harden the EdgeTPU SIM backend.**  The direct POSIX/libusb transport is
   wired into `sentai.tpu.load/load_image/invoke`, and the Python-helper runtime
   path is retired.  Remaining backend work is recovery, raw-output parity, and
   eventual queued/overlapped USB transfer optimization if throughput becomes
   important.
6. **Extend the TFL path only after decode is clear.**  `sentai.tfl` can now
   load a file-backed BMP into the model input and invoke CPU/TFLite Micro in
   SIM.  The remaining useful work is a COCO output decoder and comparison
   against the s209 EdgeTPU/host baseline, not more MP image plumbing.
7. **Decide AIfES honestly.**  Either port the real runtime into SIM or keep it
   as an explicit namespace-parity stub until a concrete experiment needs it.
8. **Improve inventory tooling.**  Add function-level namespace inventory and
   read-only smoke checks where useful, but avoid building a `sentai.prime`
   dispatcher before the owner/status/abort conventions are proven.
9. **Add an architecture dependency audit.**  Run or build a tool over
   `examples/sentai_runtime`, `sim`, and the linked project sources to identify
   cyclic dependencies, accidental SIM-to-runtime back edges, duplicated
   backend ownership, and modules that mix platform transport with shared
   runtime logic.  This should become a repeatable B7 check before larger
   namespace refactors.

The virtual-camera implementation is now part of the baseline, not future
work.  Camera id `-1` means "static image from FS"; it is intended to be shared
by ARM and SIM so TPU, markers, flow, safety, and future perception consumers
can see deterministic camera frames without MP byte arrays.

The current PrepTask milestone is build-verified in SIM:

```text
cmake --build build-sim --target sentai_sim -j2
[100%] Built target sentai_sim
```

Future extension: the same virtual-camera source should be able to play a
deterministic frame sequence, not only one static image.  That would let us run
Flow, Markers, TPU, Safety, and PrepTask consumers against repeatable
frame-by-frame fixtures on both ARM and SIM.

## Task Lifecycle / API Notes

## Implemented Slice - Virtual Camera

Current state:

- added shared `sentai_virtual_camera` source under `examples/sentai_runtime`;
- wired `sentai.camera.select(-1, path)` on SIM and ARM bindings;
- kept the current public camera verb as `select`; a `switch` alias can be
  added later when QSTRs/API naming are intentionally refreshed;
- BMP 24/32-bit static image loading works through `sentai.fs` / virtual FS;
- SIM `sim_camera_latest_rgb()` now prefers the virtual camera when active,
  so existing SIM camera consumers see the static frame;
- ARM `sentai_cam_grab_latest()` can return the virtual XRGB frame and
  `sentai_cam_return_raw()` treats that frame id as no-op;
- SIM smoke passed with `/images/cat.bmp` staged in `build-sim/sentai_fs_root`:
  `fs.size('/images/cat.bmp') > 0`,
  `select(-1, '/images/cat.bmp') == 0`, and `frame_count() == 1`.
- staged a camera-native cat fixture and COCO EdgeTPU detector for the next
  TPU smoke:
  - repo fixture: `test_data/cat_640x480.bmp`;
  - SIM FS fixture: `/images/cat_640x480.bmp`;
  - SIM FS model:
    `/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite`;
  - expected model family: SSD MobileNet v2 COCO17 postprocess EdgeTPU;
  - expected semantic class: `cat` (COCO class id 17 in the usual 90-class
    label map).
- verified through SIM REPL that `/images/cat_640x480.bmp` exists in FS,
  loads through `sentai.camera.select(-1, ...)`, and advances camera scalar
  status (`frame_count() == 1`).  Do not use MP byte-array frame reads even
  for diagnostics.  If we need pixel-level verification, C/C++ should write a
  derived artifact back to FS, such as a JPEG/BMP/hash/diagnostic text, and MP
  should only verify the artifact path, size, checksum, or compact metadata.
  Production tests must keep image buffers preallocated in C/C++ and expose
  only pointers, slots, compact status, detections, or scalar summaries
  through MP.

Experiment FS-root rule: every `sentai_sim` experiment must launch with an
explicit `SENTAI_SIM_ROOT=<experiment_iter>/fs_root`.  Do not use the shared
`build-sim/sentai_fs_root` for experiment artifacts.  Each iter folder owns
its image/model inputs, generated FS files, helper logs, SIM stdout, and compact
summary so the run can be reproduced later.

Mission-source rule: the mission code that runs in MP should also live in that
same per-iteration FS root.  Host scripts may bootstrap the REPL with a tiny
`import mission_x; mission_x.run()` command, but the mission body must be copied
to `<iter>/fs_root/mission_x.py` before execution.  This keeps the exact source
that ran reproducible beside the model, image, FR logs, and generated outputs.

SIM experiment-numbering rule: simulator-only experiments use odd `s*`
numbers.  The virtual-camera TPU parity experiment is `s209`; reserve the even
neighbor for a future ARM counterpart if needed.

Implemented SIM E2E parity experiment:

- experiment: `examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e`;
- each run creates `iterNN_<label>/fs_root` and launches `sentai_sim` with
  `SENTAI_SIM_ROOT` pointing at that per-run root;
- `sentai_sim` now keeps stdout/stderr visible for the host/REPL, but routes
  captured bytes as producers into the `sentai.fr` debug channel.  FR remains
  the single writer of `<fs_root>/fr/debug.log`; hot FreeRTOS task diagnostics
  from virtual camera, camera bridge, PrepTask, InferTask, TPU/PyCoral, and
  POSIX/libusb use `sentai_logf()` instead of direct `printf`/`fprintf`;
- ARM `_write()` now pushes printf bytes into the bounded `sentai.fr` `debug`
  channel before applying the USB-console `verbose` gate.  The recorder task
  drains that ring to the path opened with `sentai.fr.open("debug", path)`,
  using append on ARM.  This keeps debug logging best-effort and out of hot
  control paths;
- test flow:
  1. host PyCoral runs the COCO EdgeTPU model against the same 640x480 BMP;
  2. SIM starts the TPU helper, selects the virtual camera image, loads the
     same board-style model path from its per-run FS root, stages the same BMP
     into the TPU input with `sentai.tpu.load_image(...)`, invokes, then reads
     compact detections through the shared pipeline postprocessor;
  3. host detections and SIM detections must match exactly.
- first passing run:
  `s209_virtual_camera_tpu_e2e/iter02_virtual_camera_tpu_cat`;
- current passing regression run:
  `s209_virtual_camera_tpu_e2e/iter35_virtual_camera_tpu_cat`;
- current run imports `/mission_s209.py` from the experiment FS root; stdin
  only performs `import mission_s209; mission_s209.run()`;
- result: host PyCoral detections and SIM detections are byte-for-byte equal
  after aligning the host baseline preprocessing to the same integer
  area-average resize used by the SIM TPU image loader and PXP shim.
- REPL stdout is not treated as a robust data channel.  Async debug from
  background tasks can interleave with printed values, so s209 writes compact
  detections to `/detections.txt` in the experiment FS root and parses that
  file host-side.  This follows the existing rule: small metadata can cross MP,
  but reproducible test artifacts live in FS.
- debug overlay policy: generated visual artifacts should come from the
  runtime FS, not from MP image bytes.  s209 now calls
  `sentai.pipeline.save('/images/detected_overlay.bmp')`; the SIM runtime
  draws the best cat detection over the last TPU input view and writes
  the BMP directly to `<iter>/fs_root/images/detected_overlay.bmp` from C.
  Host-side PNG overlays may still be generated as convenience views, but they
  are secondary to the runtime-produced FS artifact.
- current verified artifacts in iter35:
  `/mission_s209.py`, `/detections.txt`, `/fr/debug.log`, and
  `/images/detected_overlay.bmp` under the per-run `fs_root`.

ARM caveat: the new virtual camera source compiled, but the full ARM
`sentai_runtime` link is currently blocked by existing executable-level issues
(`m_text` overflow and unresolved `sentai_crazy_send_extpos`).  The ARM
runtime still needs a full link/flash smoke once those are fixed.

Host validation status: the cat/model pair is validated through host PyCoral
and the SIM TPU helper using the same USB EdgeTPU.  s209 requires exact compact
detection equality between those two paths.

These notes capture the current command/status surface.  They are deliberately
plain: B7 should name what exists before changing any behavior.

| Family | Start / enable | Stop / abort | Status / observe | Lifecycle class |
| --- | --- | --- | --- | --- |
| Flow | `sentai.flow.start(cam_id)` -> `sentai_flow_start`; ARM also has `sentai_flow_enable` for publisher creation | `sentai.flow.stop()` -> `sentai_flow_stop` | `sentai.flow.read()`, gray helpers, debug/stats fields in binding | Producer-class perception worker.  ARM owns camera/PXP path; SIM flow is produced by `camera_bridge`. |
| SLAM places worker | `sentai.places.start_slam()` -> `sentai_slam_start` | `sentai.places.stop_slam()` -> `sentai_slam_stop` | `sentai.places.slam_current()`, `sentai.places.slam_stats()` | Consumer of PrepTask `RGB_64`; good pattern for compact C++ worker status. |
| Object EKF-SLAM module | `sentai.slam.init(...)` initializes in-memory EKF state, not a worker | no worker stop; `sentai.slam.clear()` resets state | `sentai.slam.info()`, `pose()`, `landmarks()` | Synchronous model module.  Do not confuse with `slam_task.cc` place-recognition worker. |
| Safety | `sentai.safety.init()`, `enable_aruco(...)`, `task_start()` | `task_stop()`, `clear()` resets latch/config | `aborted()`, `stats()` | Supervisor worker.  Feeds safety state machine from camera/marker observations. |
| Flight recorder | `sentai.fr.init()`, `task_start()` | `task_stop()` drains and joins | `stats()`, pushed events/scalars/frames | Evidence drain worker.  Producers call push APIs; task owns disk flush cadence. |
| B3 orientation calibration | `sentai.calib.orientation_*_start()` phase starts; `orientation_task_start()` for preflight/full worker entry | `orientation_emergency_stop()`, `orientation_task_stop()` | `orientation_task_is_done()`, `orientation_result_tuple()`, `orientation_status_tuple()`, `orientation_current_thrust()` | Controller/executive worker migrated from s205/B3.  MP mission controls phase order. |
| B4 marker control | `sentai.servo.marker_*_start()` phases: setup, acquire, center hold, extpos warmup, handoff hover, axis motion, land | `sentai.servo.marker_task_stop()` | `marker_task_is_done()`, `marker_result_tuple()`, `sentai.servo.status()` / marker status binding | Controller/executive worker migrated from s207/B4.  MP mission controls phase order. |
| Legacy calib autotune | `sentai.calib.task_start(axis, dur_s, vmax_m_s)` | `sentai.calib.task_stop()` | `sentai.calib.task_is_done()`, Kp helpers | Legacy controller worker, already marked B5 delete candidate.  Keep until users are confirmed gone. |
| Crazyflie bridge | `sentai.crazy.init(...)` | `sentai.crazy.stop()`, high-level stop/stop motors functions | `sentai.crazy.is_running()`, stats/recv APIs depending on backend | Transport/control backend.  ARM owns `crazy_rx` and `crazy_cmd`; SIM owns UDP `crazy_rx`. |
| Link / MAVLink | `sentai.link.init(...)` | `sentai.link.stop()` | stats, pose, REPL tunnel receive path | Transport worker.  SIM has `link_rx`, `link_hb`, optional `link_fwd`. |
| Mesh | `sentai.mesh.init(...)` | `sentai.mesh.stop()` | `available()`, `receive()`, `node()` | Transport worker.  ARM mesh has `mesh_rx`; SIM currently uses backend stubs for namespace parity. |
| SIM camera bridge | auto-started by simulator runtime | process lifetime | latest RGB, flow snapshot, prep slots | SIM producer equivalent for camera/PrepTask contract. |
| Detection pipeline | `sentai.pipeline.start(conf, iou, max, track)` | `sentai.pipeline.stop()` | `running()`, `stats()`, `get()`, `get_ex()`, `detections()`, `save()` | Shared ARM/SIM worker over `det_prep` and `det_infer`.  SIM only injects camera and TPU backends. |

Vocabulary for future docs:

- **producer worker**: owns a sensor or preprocessing output and publishes
  slots/snapshots (`PrepTask`, `camera_bridge`, current ARM `flow_pub`);
- **consumer worker**: wakes on producer data and publishes compact model/status
  output (`slam_task`, `InferTask`);
- **controller/executive worker**: owns actuator/control phases and must expose
  done/result/status/abort (`sentai_calib_orientation_task`,
  `sentai_servo_marker_task`);
- **transport worker**: moves bytes/messages and should avoid mission policy
  (`crazy`, `link`, `mesh`);
- **supervisor worker**: monitors safety/health and latches abort/fault state
  (`sentai_safety_task`, watchdog tasks);
- **evidence worker**: drains logs/FR data without participating in control
  (`sentai_fr_task`).

## Implemented Slice

- SIM now includes shared `sentai_runtime` bindings for `uart`, `usb`, `mesh`,
  `imu`, `mic`, `sleep`, `aifes`, `kmeans`, `pca`, `anomaly`, `dtw`, `hmm`,
  `rl`, `slam`, and `tfl`.
- Top-level operator helpers `help`, `debug`, `console`, and `run` now live in
  shared `examples/sentai_runtime/bindings/modsentai_top.c` and are included by
  both ARM and SIM.
- SIM platform-specific behavior for these shared bindings is isolated in
  `sim/sentai_platform_sim_backend.c`.
- `sentai.aifes` remains an explicit SIM stub: `ready()` is false and
  load/invoke operations return errors until its runtime backend is ported.
- `sentai.tfl` now uses the shared TFLite Micro CPU-only bridge in SIM.  It
  loads `.tflite` models from the SIM FS root, allocates a tensor arena, accepts
  compact input bytes or a file-backed BMP through `load_image()`, invokes, and
  exposes output metadata/data through the existing binding.
- `sentai.slam` parity is now present in SIM, so D7-style model-substrate
  experiments do not need a SIM-only fallback for that namespace.
- `sentai.tpu` now uses `sentai_tpu_posix_backend.cc` in SIM.  It performs
  cold DFU from `apex_latest_single_ep_bin.c` when the USB stick enumerates as
  `1a6e:089a`, reopens app mode `18d1:9302`, initializes the shared
  `libs/tpu/edgetpu_driver.cc` path, and executes model load/invoke through a
  static FreeRTOS `tpu_posix` task.  The MicroPython binding surface itself is
  shared with ARM through `bindings/modsentai_tpu.c`.
- `modsentai_imu.c` now provides a portable `M_PI` fallback so the shared
  binding compiles outside the ARM toolchain too.
- CMake links the generated mesh protobuf/nanopb sources in SIM because the
  shared mesh binding uses the same `sentai_mesh.h` contract as ARM.
- `scripts/sentai_namespace_inventory.py` compares ARM and SIM root namespace
  exports from source and fails if an unexpected one-sided export appears.
- `examples/sentai_runtime/diag/smoke_namespace_model.py` is a read-only MP
  smoke script for the model substrate (`objects`, `object_lifter`, `places`,
  `slam`) and core B5 controller/evidence namespaces.
- `scripts/sentai_sim_model_smoke.py` copies that MP smoke script into the SIM
  filesystem root and runs it through `sentai.run(...)`.

## Priority Alignment Against A7/B6

B7 is aligned with A7/B6 only if we describe the current state precisely:

- A7 still stands as written: inventory, naming, SIM/ARM parity, safe
  introspection, and B5/s207 regression remain the guardrails;
- achieved: **runtime surface parity** between ARM and SIM for exported
  `sentai.*` names;
- achieved: top-level helpers are now single-source shared bindings;
- achieved: model-substrate names from B6/D8 (`objects`, `object_lifter`,
  `places`, `slam`) are visible in SIM;
- achieved with caveat: `aifes` is visible but still a SIM stub, so it is
  namespace parity only;
- achieved: `sentai.tfl` has CPU/TFLite Micro runtime parity in SIM for the
  core load/set_input/load_image/invoke/output path;
- achieved: `sentai.tpu` Python API surface parity is restored by deleting the
  SIM-only binding and including `bindings/modsentai_tpu.c`;
- achieved with caveat: direct POSIX USB/libusb transport parity is proven by
  `tpu_posix_smoke`, and the shared SIM `sentai.tpu.load/load_image/invoke`
  path now uses that backend through the CoralMicro `EdgeTpuManager`/TFLM
  stack.  PyCoral is no longer in the SIM runtime path; it remains only a
  host-side baseline for experiment comparisons.  The remaining caveat is
  exact low-confidence postprocess parity with host PyCoral, not transport
  integration;
- not done: `sentai.prime`, command dictionary, minimal executive, active
  primitive table, ACK/RUN/OK/ERR protocol;
- not done: formal board-model query facade from B6/D7;
- started: C++ task inventory and PrepTask producer/consumer documentation.

## Realized So Far

This is the concrete B7 progress at this point:

1. ARM and SIM root `sentai.*` namespace parity is now achieved, with no
   SIM-only runtime namespace left.  `sentai.sim` was removed; SIM-specific
   behavior now belongs in backends, bridges, stubs, or host-side experiment
   code.
2. Several namespaces that were ARM-only are now included from the shared
   `examples/sentai_runtime` binding sources in SIM instead of being duplicated.
3. Top-level helper commands `sentai.help`, `sentai.debug`, `sentai.console`,
   and `sentai.run` were moved to the shared `modsentai_top.c` binding.
4. SIM platform-specific behavior was isolated in `sentai_platform_sim_backend.c`
   so shared bindings can compile without inventing separate SIM APIs.
5. `sentai.aifes` now has an explicit SIM namespace stub: discoverable,
   honest, and not pretending runtime support exists yet.
6. `sentai.tfl` now links the shared `sentai_tfl_bridge.cc` backend into SIM
   through `libs_tensorflow_sim`; s211 proves TFLite Micro CPU inference on
   x86 using the COCO non-EdgeTPU model and the same cat BMP staged in the
   per-iteration FS root.
7. `scripts/sentai_namespace_inventory.py` now checks ARM/SIM namespace drift
   from source.
8. `smoke_namespace_model.py` plus `scripts/sentai_sim_model_smoke.py` now
   verify the model-substrate namespace in SIM without starting control loops
   or mutating the world model.
9. The first C++ task inventory is documented, including the key PrepTask
   producer/consumer contract and the remaining flow/pipeline duplication
   questions.
10. The current task lifecycle/API surface is documented for flow, SLAM,
   safety, FR, B3/B4 controllers, legacy autotune, Crazyflie/link/mesh
   transport, SIM camera bridge, and SIM pipeline.
11. Direct host EdgeTPU USB transport is implemented:
    `sim/usb_host_edgetpu_posix.c` provides the `USB_HostEdgeTpu*` contract
    over `libusb`, including cold DFU firmware load.  `tpu_posix_smoke` remains
    the low-level transport check, while `sentai_tpu_posix_backend.cc` is the
    runtime backend.  Local `libedgetpu.so.1` inspection confirms that the real
    Google USB stack is not a simple blocking `libusb_bulk_transfer` loop: it
    uses `libusb_submit_transfer`, queued bulk-in requests, overlapping
    bulk-in/out options, and an event pump.  It also confirms that 256-byte
    bulk-IN packets/chunks are a real USB2/device-level constraint.  The SIM
    backend now submits libusb transfers and pumps events cooperatively from
    the FreeRTOS POSIX task with a short 100 us poll, avoiding the earlier
    `SIGALRM`/blocking-poll failure and avoiding log/sleep-dependent transfer
    progress.  Transfer deadlines use `CLOCK_MONOTONIC` rather than the
    FreeRTOS POSIX tick so a host-side I/O wait cannot wedge the TPU task past
    its timeout.  Full queued overlap remains future optimization; PyCoral
    remains host-side only.
12. SIM `sentai.tpu` now includes the shared ARM binding table and deletes the
    old SIM-specific `modsentai_sim_tpu.c`; `sentai_tpu_posix_backend.cc`
    supplies the platform backend aliases (`sentai_load_model`,
    `sentai_load_image`, `sentai_save_output`, slot aliases, and diagnostics).
13. SIM `sentai.pipeline` now includes the shared ARM binding table and deletes
    the old SIM-specific `modsentai_sim_pipeline.c`; `detection_task.cc` is the
    shared implementation, while simulator camera behavior is injected by
    camera backends/bridges and TPU behavior by `sentai_tpu_posix_backend.cc`.
    `pipeline.once(...)` was added as a bounded diagnostic command on top of
    the same shared `PrepTask`/`InferTask` workers.
14. s209 was updated to run from `/mission_s209.py` staged inside the per-run
    FS root and to use the synchronous virtual-image TPU path:
    `camera.select(-1, path)`, `tpu.load_image(path)`, `tpu.invoke()`, then
    shared `pipeline.detections/save`.  The continuous `start/get_ex/stop`
    worker remains a documented SIM scheduling gap, not a hidden SIM-specific
    fallback.
    Current validation: `iter109_virtual_camera_tpu_cat` passes with the direct
    POSIX/libusb backend, writes `fs_root/fr/debug.log`, `fs_root/detections.txt`,
    and `fs_root/images/detected_overlay.bmp`, and matches the host PyCoral
    baseline on high-confidence COCO cat detections.  Low-confidence tail
    ordering still needs raw-output/postprocess parity work before claiming
    exact all-detection equality.
    Startup hygiene added after timeout testing: s209 runners now terminate
    stale `sentai_sim` / Gazebo camera bridge / TPU smoke processes, unlink
    `/tmp/sentai_cam.sock`, and persist partial `sim_output.txt` on timeout so
    blocked runs leave no orphan process and remain diagnosable.
15. SIM `sentai.io`, `sentai.rtos`, `sentai.sys`, and `sentai.fs` now use the
    shared ARM binding tables.  The deleted historical fragments are:
    `modsentai_sim_io.c`, `modsentai_sim_rtos.c`, `modsentai_sim_sys.c`, and
    `modsentai_sim_fs.c`.
16. Legacy `sentai.sim.journal_*` was removed completely.  The runtime journal
    path is `sentai.fr`; older experiments that still call `sentai.sim` should
    be refreshed when rerun.
17. SIM `sentai.crazy` now includes the shared ARM binding table
    `bindings/modsentai_crazy.c`; the old `modsentai_sim_crazy.c` binding is
    no longer included.  `sentai_crazy_crtp_udp_bridge.cc` remains the
    injected CRTP-over-UDP backend and now supplies the extra common-binding
    hooks (`link_send`, dispatch pop, attitude release) needed for API parity.
    The file name describes the actual simulator transport boundary.
18. SIM MAVLink/PX4 transport was renamed from `sentai_link_sim.cc` to
    `sentai_mavlink_udp_bridge.cc` so the file name describes the actual
    simulator boundary: MAVLink messages over POSIX UDP toward PX4/Gazebo
    SITL.  The Python surface remains the shared `sentai.link` binding.
19. The temporary SIM camera binding moved out of `sim/` to
    `examples/sentai_runtime/bindings/modsentai_camera_sim.c`.  This is not the
    final destination; it is a staging step before folding the SIM branch into
    `bindings/modsentai_camera.c` and leaving only camera providers/bridges in
    `sim/`.

This matches A7's non-goal: B7 should not introduce the full `sentai.prime`
dispatcher or mission language yet.  The B6 priority now is not more API
surface work; it is understanding and documenting the C++ task/runtime core so
future primitives know which controller/task owns each loop.

Recommended next B7 priorities:

1. Validate the lifecycle inventory against the actual MP bindings and mark
   any missing status/abort surfaces that would block future primitives.
2. Extend the smoke checks toward top-level helpers and controller status
   surfaces while keeping them read-only.
3. Extend the direct POSIX TPU transport from cooperative event pumping to
   queued/overlapped bulk-in/out if throughput becomes important.  Keep the
   implementation in FreeRTOS POSIX tasks and libusb, with no PyCoral helper in
   `sentai_sim`.  s209 remains the parity gate against host PyCoral.
4. Keep `aifes` runtime backend migration as a follow-up after the task
   inventory clarifies where ML inference/training should actually run.  For
   `tfl`, the remaining gaps are COCO output decoding, bbox parity against the
   s209 baseline, broader model coverage, and ARM/SIM output-parity smokes.
5. Keep B5/s207 as regression after every runtime alignment change.

Smoke test:

```text
python3 scripts/sentai_namespace_inventory.py
ARM-only exports -> none
SIM-only exports -> none
Allowed SIM-only exports -> sim

sentai.run('/smoke_namespace_model.py')
OK namespace_model_smoke
True

python3 scripts/sentai_sim_model_smoke.py
OK namespace_model_smoke

cmake --build build-sim --target tpu_posix_smoke -j2
[100%] Built target tpu_posix_smoke

./build-sim/sim/tpu_posix_smoke
[tpu-posix] opened Coral USB interface=0 out=01,02,03 in=81,82 irq=83
OK tpu_posix_smoke temp_c=32.05

Cold USB state was also verified once:
`1a6e:089a` DFU mode accepted `apex_latest_single_ep_bin.c`, verified the
firmware upload, reset, and re-enumerated as app mode `18d1:9302`.

import sentai
'imu' in dir(sentai)  -> True
'mesh' in dir(sentai) -> True
'slam' in dir(sentai) -> True
'usb' in dir(sentai)  -> True
'aifes' in dir(sentai)-> True
'tfl' in dir(sentai)  -> True
sentai.imu.read()     -> {'x': 0.0, 'y': 0.0, 'z': 1000.0, 'temp': 25.0}
sentai.mesh.node()    -> 0
sentai.console()      -> 'usb'
sentai.run('/tmp.py') -> executes script from the SIM/board filesystem
sentai.aifes.ready()  -> False
sentai.tfl.ready()    -> False before load, True after a successful load

cmake --build build-sim --target sentai_sim -j2
[100%] Built target sentai_sim

s211_tfl_cpu_sim_smoke/iter03_coco_cat_tfl_cpu:
sentai.tfl.load('/models/tf2_ssd_mobilenet_v2_coco17_ptq.tflite', 8192) -> 0
sentai.tfl.ready()      -> True
sentai.tfl.input_dims() -> (1, 300, 300, 3)
sentai.tfl.load_image('/images/cat_640x480.bmp') -> 0
sentai.tfl.invoke()     -> 482 ms
sentai.tfl.num_outputs()-> 4

python3 scripts/sentai_namespace_inventory.py --functions:
sentai.tpu -> shared binding, 29 ARM funcs, 29 SIM funcs, no tpu API drift

s209_virtual_camera_tpu_e2e/iter35_virtual_camera_tpu_cat:
host PyCoral detections == SIM sentai.tpu/pipeline detections
shared pipeline postprocessor consumes TPU outputs; virtual-image input uses
the same area-average resize baseline as host PyCoral

s209_virtual_camera_tpu_e2e/iter14_tpu_shared_load_image_smoke:
sentai.tpu.load_image('/images/cat_640x480.bmp') -> 0
sentai.tpu.invoke() -> 0
sentai.tpu.save_output('/outputs.csv') -> 0
sentai.fs.exists('/outputs.csv'), sentai.fs.size('/outputs.csv') -> True, 1657
```

Later GDB check after direct POSIX/libusb refactor:

- `gdb --args build-sim/sim/sentai_sim` attached cleanly and showed that a
  crash was not in MicroPython sleep or the REPL loop.  The abort was in
  `TpuDriver::Initialize()`, first CSR read:
  `Read32(chip_config_.GetApexCsrOffsets().omc0_00)`.
- The POSIX backend logged `libusb_control_transfer` timeout
  (`rc=-7`) for `bm=c0 req=1 val=a000 idx=0001 len=4`.  After a software
  USB reset, the Coral stick disappeared from `lsusb`, so this specific run
  needs a physical replug/power-cycle before semantic s209 parity can be
  revalidated.
- The simulator was hardened so this first CSR read failure returns `false`
  from TPU initialization instead of aborting the whole `sentai_sim` process.
  With the Coral absent, s209 now exits cleanly with `TPU_LOAD -1`,
  `TPU_READY False`, `TPU_INVOKE -1`, and still writes the normal FS/log
  artifacts.

Note: s211 is currently an execution/backend smoke for CPU COCO, not yet a
full semantic detection parity test against s209.  The non-EdgeTPU COCO model
runs and produces four float outputs through TFLite Micro, but B7 has not yet
standardized the TFL output decoder/bbox comparison.  The baseline requirement
is nevertheless fixed: perception smokes should use the COCO cat image/model
family unless the test is explicitly about a tiny kernel/link sanity model.

Latest s209 flow+TPU status:

- `mission_s209_flow_tpu.py` now follows the ARM-style task model: MP loads the
  model, starts `flow` and `pipeline`, selects virtual camera frames from the
  per-run FS, then reads task-owned results.  It no longer drives per-frame host
  exec snippets.
- `camera.select(-1, path)` is kept as a lightweight virtual-source selection.
  It no longer synchronously wakes `PrepTask` from inside the MP call on SIM;
  `PrepTask` polls the virtual frame source with a bounded SIM wait.  This
  avoids the FreeRTOS POSIX case where a higher-priority task preempted the REPL
  inside `camera.select()` and the call did not reliably return.
- `camera_bridge_recv.c` is now non-blocking at the socket boundary.  The
  Gazebo bridge listens on `/tmp/sentai_cam.sock`, but `accept/read/write`
  paths poll with short `vTaskDelay()` yields instead of holding a raw blocking
  POSIX syscall inside a FreeRTOS task.  This fixed the reproducible
  `libusb_init()` hang when the camera bridge was started before
  `sentai.tpu.load()`.
- `PrepTask` owns the flow resize/gray slot publication.  `FlowTask` and
  detection consumers read prepared buffers; the virtual camera path does not
  do hidden resize/preprocess work in MP.
- `sentai_tpu_posix_backend.cc` no longer has a separate `tpu_posix` worker
  layered under `InferTask`.  Load/invoke now execute in the caller task, which
  matches the ARM shape: MP can issue one-shot commands, while continuous
  inference runs in `InferTask`.
- `s209_virtual_camera_tpu_e2e/iter271_virtual_camera_flow_tpu_shift` completed
  a 3-frame smoke: frame 0 static flow was zero, offset frames produced nonzero
  flow, and TPU detections were recorded with the COCO cat image.  Later repeats
  showed two concrete bugs and one hardware/test blocker:
  - blocking `camera_bridge` accept path could stall `libusb_init()`; fixed;
  - using MP/REPL as the frame scheduler is not acceptable.  The pipeline now
    exposes `sentai.pipeline.frame_count([after, timeout_ms])` /
    `sentai.pipeline.count(...)` as a task-owned detection event counter, so MP
    can sleep as a subscriber and then read the latest snapshot instead of
    polling the hot path;
  - after several killed runs and a software `usbreset`, the Coral USB device
    disappeared from `lsusb`; replug/power-cycle is required before validating
    the final flow+TPU offset run again.

Current expected next validation after Coral replug:

1. `build-sim/sim/tpu_posix_smoke` must pass first.
2. Minimal SIM load/invoke:
   `sentai.tpu.load('/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite')`
   should return `0`.
3. `python3 examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/run_s209_flow_tpu.py --frames 3 --step-px 2 --no-validate`
   should write `fs_root/fr/debug.log`, `fs_root/flow_tpu_results.txt`, and
   `fs_root/images/detected_overlay_last.bmp`.

2026-06-01 correction after the host-backend spike:

- Direct POSIX/libusb EdgeTPU remains useful as a low-level transport
  experiment, but it is not yet the stable SIM runtime path.  It can initialize
  and sometimes invoke, but longer s209 runs can poison or stall the Coral USB
  state.
- A selectable host backend was added with `SENTAI_TPU_BACKEND=pycoral`.
  `sentai_sim` keeps the normal `sentai.tpu` and `sentai.pipeline` surfaces,
  while a host helper owns the full TFLite interpreter + EdgeTPU delegate over
  a Unix socket.  This preserves custom ops and `Detection_PostProcess` instead
  of inventing a raw EdgeTPU protocol.
- Verified smoke: `sentai.tpu.load`, `sentai.tpu.load_image`, repeated
  `sentai.tpu.invoke`, and `sentai.pipeline.detections()` work against the
  COCO cat model/image through the PyCoral backend.  The first invoke is ~28-29
  ms after delegate/model setup; subsequent invokes are ~12-13 ms.
- The continuous `PrepTask`/`InferTask` s209 flow+TPU run still exposes a SIM
  scheduler/handshake gap.  Short runs can process offset frames, but longer
  runs still wedge around MP polling, virtual-camera wakeups, or POSIX
  FreeRTOS waits.  Treat this as the next B7 blocker, separate from TPU output
  correctness.
- Follow-up implementation moved the virtual camera closer to the ARM camera
  model:
  - `sentai_virtual_camera.cc` now owns a C++ `VirtualCameraTask` that plays
    BMP frames from a board-FS directory (`frame_000.bmp`, `frame_001.bmp`, ...)
    at a configured FPS/count via `sentai.camera.play(dir, fps, count)`.
  - `sentai.camera.play_stop()` and `sentai.camera.playing()` expose the task
    state to MP; MP no longer pushes frame byte arrays or loops over host-side
    image files.
  - virtual frames are tagged as source camera id `-1`, so `PrepTask`,
    `InferTask`, and `FlowTask` observe the same source-id convention as
    physical/simulated camera producers.
  - SIM REPL priority was lowered and `sentai.rtos.repl_kick()` now uses the
    POSIX host scheduler yield instead of FreeRTOS `taskYIELD()`, because
    `taskYIELD()` can fail to return reliably while MP is polling task results.
- Latest validation state after this change:
  - build succeeds through `sentai_sim` build #1145;
  - s209 reaches concurrent virtual-camera + flow + TPU execution: frame 0 has
    zero flow, shifted frames produce nonzero flow, and COCO cat detections move
    in x;
  - the collection/orchestration fix has started: `InferTask` publishes a
    detection event counter independent from the result queue, and s209 waits
    on that counter before reading a snapshot.  The next refinement is a small
    C++ history/ring buffer that lets MP query the whole playback summary after
    camera playback finishes.
- Implementation notes from this spike:
  - host backend bootstrap must not use `vTaskDelay()` in the helper connect
    loop; POSIX `usleep()` made helper startup deterministic;
  - the PyCoral helper must use `set_tensor()` / `get_tensor()` copies, not
    retained `tensor()` numpy views, otherwise TFLite refuses `invoke()`;
  - the host backend must honor `sentai_tpu_set_input_done_sema()` by signaling
    once the input bytes have been sent to the helper, mirroring ARM's
    SendInputs-done handoff.

2026-06-02 prep-only divide-and-conquer result:

- Fixed the virtual-camera publication contract: `sentai_virtual_camera_after_select()`
  now publishes the loaded XRGB frame through the shared camera backend, so a
  virtual frame behaves like a completed camera frame instead of only waking a
  waiter with no publication side effect.
- Added `sentai.camera.replay(fps, count)` for diagnostics.  It republishes the
  already-selected virtual frame from the static virtual-camera buffer, so tests
  can remove filesystem reads from the camera hot loop:
  `select(-1, "/images/shift/frame_000.bmp")` reads once, then `replay(...)`
  serves frames from RAM.
- Added temporary SIM camera hooks `prep_enable`, `prep_disable`, `prep_once`,
  `prep_reset`, and `prep_stats` to isolate `VirtualCameraTask -> prep slots`
  without starting TPU or `InferTask`.  These are deliberately diagnostic and
  should move to the shared camera binding when `modsentai_camera_sim.c` is
  retired.
- Root-caused the SIM wedge with GDB: raw stdout/stderr tee pthreads were
  receiving FreeRTOS POSIX `SIGALRM`/`SIGUSR1` scheduler signals.  The tick
  handler then ran from a non-FreeRTOS pthread and could deadlock the scheduler
  mutex while MP was in `sentai.rtos.sleep_ms()`.
- Fixed the tee pthread entry to block `SIGALRM` and `SIGUSR1`.  This keeps
  host-side logging pthreads outside the FreeRTOS POSIX scheduler signal path.
- Passing validation:
  `python3 examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/run_s209_prep_only.py --duration-ms 3000 --frames 10 --min-delta 5 --label prep_only_mem`
  produced `iter337_prep_only_mem` with `delta_frames=10`,
  `delta_aux=10`, `flow_slot_seq_delta=10`, and
  `fs_root/fr/debug.log` containing the virtual-camera replay trace.

## Current Conclusion

The first B7 slice stayed non-behavioral for flight: it widened SIM namespace
surface parity by including shared runtime bindings and adding only platform
backend shims or explicit stubs.  Since then, `sentai.tfl` has crossed from
stub parity into real CPU/TFLite Micro SIM execution, while `sentai.tpu` now
has shared ARM/SIM binding parity and continues to own the EdgeTPU path.
`sentai.aifes` remains a visible stub.

The next B7 work should shift from namespace parity to the runtime core:
document `*Task` ownership, PrepTask data flow, producer/consumer buffers,
status/cancel/abort contracts, and regression checks.  No `sentai.prime`
dispatcher or model-query facade should be implemented until that task
inventory is clear.

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
io, rtos, diag, sys, fs, camera, flow, tpu, pipeline, link, uart, usb, mesh,
imu, mic, sleep, kmeans, pca, anomaly, dtw, hmm, rl, slam, objects, places,
servo, object_lifter, calib, markers, safety, fr, explore, tfl, crazy, aifes,
sim
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
| Safety / health | `safety`, `diag`, `sys` |
| Mission FSM precedent | `explore` |
| Helper math / learning substrate | `kmeans`, `pca`, `anomaly`, `dtw`, `hmm`, `rl` |
| ML experiment substrate | `tpu`, `tfl`, `aifes` |
| Board IO / operator channels | `uart`, `usb`, `mesh` |
| Board local sensors / power modes | `imu`, `mic`, `sleep` |

### ARM-Only Namespaces

```text
none
```

All ARM root namespaces are now present in SIM, except the expected SIM-only
`sim` namespace in the other direction.

Important caveat: `aifes` and `tfl` are namespace-parity stubs in SIM for now.
Their shared Python bindings are included, but the simulator backend returns
clear not-ready/error results until the AIfES and TFLite Micro runtimes are
linked or replaced by a documented host backend.

### SIM-Only Namespace

```text
sim
```

Expected.  This is simulator-specific support and should not be considered a
board runtime primitive.

## B7 Implementation Tasks

Status after the current B7 slice:

- namespace inventory and model-substrate smoke scaffolding exist;
- SIM now shares more runtime surface with ARM and has fewer divergent stubs;
- virtual camera is implemented as a shared ARM/SIM source selected through
  `sentai.camera.select(-1, path)`;
- s209 proves host PyCoral and SIM/EdgeTPU helper parity on the same model and
  image;
- SIM and ARM have a `sentai.fr` debug channel path for printf/debug traces;
- mission-source reproducibility is now a rule: run MP missions from files
  staged into the per-iteration FS root, with stdin only bootstrapping import
  and `run()`.

Remaining B7 gaps:

1. Add a small host-side namespace inventory tool or script that can report the
   root `sentai.*` surface from source or from a running runtime.
2. Add a read-only smoke check for shared model substrates:
   `objects`, `object_lifter`, `places`, and optionally `slam` where present.
3. Review `aifes` and `tfl` dependency boundaries and replace their SIM stubs
   with real shared runtimes or documented host backends.
4. Build a C++ task inventory for the runtime core.
5. Update documentation when parity decisions are made.
6. Run B5/s207 as regression after any runtime binding change.
7. Wire the direct POSIX/libusb EdgeTPU transport into the REPL TPU backend, or
   explicitly keep the Python helper as the documented SIM backend.
8. Review SIM `sentai.pipeline` resize/detection paths against the PrepTask
   contract so the current smoke pipeline does not become accidental duplicate
   production preprocessing.

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
| `det_prep` / PrepTask | `examples/sentai_runtime/detection_task.cc` | ARM | camera/detection runtime | Canonical ARM camera producer.  Grabs camera frames, performs PXP resize / quantization, can write directly into the TPU input buffer, publishes prep slots, and signals consumers. |
| `det_infer` / InferTask | `examples/sentai_runtime/detection_task.cc` | ARM | TPU/object detection runtime | Consumer of PrepTask output.  Invokes TPU and publishes detection/NMS results. |
| `camera_bridge` | `sim/camera_bridge_recv.c` | SIM | simulator camera backend | SIM equivalent of PrepTask producer.  Receives Gazebo frames, publishes flow snapshots, latest RGB, and prep slots.  Platform backend, not a separate algorithm policy. |
| `pipeline` | `sim/modsentai_sim_pipeline.c` | SIM | `sentai.pipeline.start/stop/stats` | SIM smoke loop for camera -> resize -> TPU invoke.  Needs follow-up review because it has its own resize path and should not become a second production preprocessing pipeline. |
| `flow_pub` | `examples/sentai_runtime/flow_task.cc` | ARM | `sentai.flow.start/stop/read` | ARM optical-flow publisher.  Grabs camera frames, PXP-scales to 80x60, computes flow, publishes `flow_shared_t`.  Historical M7-only path; B7 should compare it with PrepTask/camera_bridge data ownership. |
| `slam` | `examples/sentai_runtime/slam_task.cc` | ARM/SIM | `sentai.slam.start/stop/current/stats` | Consumer of PrepTask `SENTAI_PREP_SLOT_RGB_64`.  Computes place descriptors and publishes compact match state. |
| `sentai_safety_task` | `examples/sentai_runtime/sentai_safety_task.*` | ARM/SIM-style FreeRTOS/POSIX | `sentai.safety` worker API | Safety monitor worker.  Consumes camera frames zero-copy, runs ArUco check, pushes results into the safety state machine. |
| `sentai_fr_task` | `examples/sentai_runtime/sentai_fr_task.*` | ARM/SIM-style FreeRTOS/POSIX | `sentai.fr` worker API | Flight-recorder drain task.  Keeps FR queue draining on a fixed cadence without exposing channel internals to missions. |
| `sentai_calib_orientation_task` | `examples/sentai_runtime/sentai_calib_orientation_task.*` | ARM/SIM runtime | `sentai.calib.orientation_*` | Migrated B3 orientation/calibration state machine.  MP mission starts phases and reads status/result/current thrust. |
| `sentai_servo_marker_task` | `examples/sentai_runtime/sentai_servo_marker_task.*` | ARM/SIM runtime | `sentai.servo_marker_*` via `sentai.servo`/mission bindings | Migrated B4 marker-control state machine.  Owns acquire, center hold, extpos warmup, handoff hover, axis motion, and landing phases. |
| `sentai_calib_task` | `examples/sentai_runtime/sentai_calib_task.cc` | ARM/SIM runtime | legacy calib/autotune API | Legacy calibration/autotune worker.  Mark for compatibility review after B3/B4 task migration. |
| Crazyflie bridge tasks | `examples/sentai_runtime/sentai_crazy.*`, `sim/sentai_crazy_sim.cc` | ARM/SIM backend-specific | `sentai.crazy`, link backend | CRTP receive and command/setpoint streaming loops.  Platform-specific transport, shared semantic surface. |
| Link / mesh tasks | `examples/sentai_runtime/sentai_mesh.*`, `sim/sentai_link_sim.cc` | ARM/SIM backend-specific | `sentai.link`, `sentai.mesh` | Radio/UART/protobuf receive, heartbeat, and SIM flow forwarding.  Important for future REPL/prime command channel. |
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
  - `SENTAI_PREP_SLOT_GRAY_64`: Y8 64x64 view.
- Ownership rule: producer writes, commits, and bumps sequence; consumers read
  latest finalized slots or compact snapshots.
- Consumer rule: consumers must not move full frames through MP heap and must
  not redo expensive per-pixel preprocessing when a prep slot already exists.
- Scheduling rule: signals are coalesced where appropriate.  Consumers should
  tolerate last-frame-wins semantics rather than building unbounded queues.

Known consumers today:

| Consumer | Data consumed | Notes |
| --- | --- | --- |
| InferTask | direct TPU tensor or staging buffer from PrepTask | ARM path already has direct tensor handoff to avoid memcpy. |
| SlamTask | `SENTAI_PREP_SLOT_RGB_64` | Good model-substrate pattern: C++ consumes slot, MP sees compact status. |
| SafetyTask | camera zero-copy gray frame | Should be reviewed against prep slots, but current design intentionally avoids MP copies. |
| Flow | ARM `flow_pub` currently owns its own camera/PXP path; SIM camera bridge computes flow | This is the main duplicate-preprocessing candidate to review later, without retuning B5. |
| SIM pipeline | latest RGB copy from camera bridge plus local resize | Acceptable smoke path for now, but not canonical for production preprocessing. |

## B7 Gaps From Task Inventory

1. `flow_pub` and SIM `camera_bridge` both perform image scaling/gray
   preparation.  This may be legitimate platform separation today, but B7
   should document whether flow becomes a PrepTask consumer or remains a
   producer-class worker.
2. SIM `sentai.pipeline` has a local resize path.  Keep it as smoke/demo until
   we decide if it should consume PrepTask slots or a shared resize backend.
3. `sentai_calib_task` appears legacy after B3/B4 migration.  Mark exact APIs
   and users before deleting anything.
4. `aifes` and `tfl` are namespace-parity stubs in SIM.  They are not task
   parity yet.
5. SIM EdgeTPU has both the older Python helper path and a new direct
   POSIX/libusb transport smoke.  The direct path preserves the existing
   `TpuDriver` C++ driver and replaces only the board-specific NXP USB Host
   transport.  Full `sentai.tpu.load/invoke` parity still needs the model
   load/invoke layer to be wired on top of this transport.
6. The source namespace inventory check exists; a deeper function-level
   inventory is still a possible follow-up.
7. The first task lifecycle vocabulary is documented below; it should be kept
   lightweight and refined only when implementation pressure proves a gap.

## REPL TPU / Image Recognition Target

The TPU migration target is not merely `TpuDriver::Initialize()`.  The target
operator loop is file-based and does **not** use the camera:

```python
import sentai
sentai.tpu.load('/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite')
sentai.tpu.load_image('/images/coco_cat.jpg')      # or set_input(fs.read(...))
sentai.tpu.invoke()
sentai.pipeline.detections(300)
```

The goal is to command this from the REPL using the virtual/board filesystem:
copy a COCO/SSD model into FS, copy an image into FS, run recognition, and read
classified detections or raw tensor outputs.

For the B7 smoke, the image must be a classic static test image, such as
`test_data/cat.bmp` or `sim/gazebo/materials/textures/imagenet_cat.png`, copied
or converted into the SIM FS root.  The camera bridge, Gazebo camera feed, and
live `sentai.pipeline.step()` camera path are intentionally out of scope for
this smoke.

Preferred follow-up shape: add a virtual file-backed camera source instead of
inventing one-off image paths in every pipeline.  For example:

```python
sentai.camera.switch(-1, '/images/cat.png')
```

or an equivalent `camera.select_file(...)` API could make camera id `-1` mean
"static image from FS".  After that, existing code that already consumes the
camera interface can run unchanged: `sentai.pipeline.step()`, TPU preprocessing,
detectors, trackers, and future test pipelines all see a normal camera frame.
This is cleaner than adding per-pipeline `load_image` shortcuts everywhere.

### Existing Namespace Surface

What already exists:

| Namespace | Relevant API | Current status |
| --- | --- | --- |
| `sentai.fs` | `write`, `read`, `exists`, `size`, `ls`, `mkdir`, `remove` | Good enough for virtual FS model/image staging in SIM and board FS staging on ARM. |
| `sentai.tpu` ARM binding | `load`, `load_image`, `invoke`, `ready`, raw output access, quantization, rows/values, `save_output`, slot APIs | Correct REPL shape for EdgeTPU.  This is the API surface to preserve. |
| `sentai.tpu` SIM binding | `load`, `invoke`, `ready`, raw output access, `set_input` | Currently uses `sim_tpu_helper.py`; missing ARM parity names such as `load_image`, `save_output`, slot/raw helpers. |
| `sentai.tfl` | `load`, `set_input`, `load_image`, `invoke`, output access | API shape is useful, but it is CPU/TFLite Micro, not EdgeTPU.  On SIM it is still a stub backend. |
| `sentai.pipeline` SIM | `step`, `start`, `predict`, `detections`, tracker helpers | Already decodes SSD MobileNet COCO-style 4-output tensors after a TPU invoke.  It assumes current `sentai_tpu_*` outputs. |
| `sentai.pipeline` ARM | richer camera/infer/tracker control surface | Canonical production pipeline; SIM should converge toward shared semantics, not grow a separate policy. |

### Current Missing Pieces

1. **Direct POSIX TPU is not wired to REPL yet.**  `tpu_posix_smoke` proves
   libusb transport + DFU + `TpuDriver::Initialize()`, but `sentai.tpu` in SIM
   still uses the Python helper.
2. **SIM `sentai.tpu` binding is not ARM surface-parity.**  It should either
   include the shared `modsentai_tpu.c` or expose equivalent names:
   `load_image`, `save_output`, slot introspection, `row`, `value`,
   `output_floats`, and `dump_eps`.
3. **Image-from-FS preprocessing is not implemented for direct TPU.**  The
   minimal first pass can accept raw resized RGB bytes via
   `sentai.tpu.set_input(sentai.fs.read('/images/input_300x300.rgb'))`.
   The user-facing pass should add `sentai.tpu.load_image(path)` for JPEG/PNG
   or documented raw RGB input.
4. **COCO detection decode exists, but in `sentai.pipeline`.**  After
   file-based `load_image` or manual `set_input` + `invoke`,
   `sentai.pipeline.detections()` can decode SSD 4-output tensors without
   requiring the camera bridge.  `sentai.pipeline.step()` is not part of this
   smoke because it consumes live camera frames.  Tracker update still assumes
   `s_pipe_resized` when using histogram paths.
5. **`sentai.tfl.load_image` is a stub.**  It is not the EdgeTPU path and
   should not be used to prove COCO/Coral USB parity.

### Recommended Next Slice

Do this in small, testable steps:

1. Add a direct SIM C/C++ implementation of the `sentai_tpu_*` ABI on top of
   the POSIX/libusb transport and shared `edgetpu_driver.cc`.
2. For the first REPL smoke, support `load`, `set_input`, `invoke`, output
   metadata/data, and `pipeline.detections()` using
   `models/testconv1-edgetpu.tflite` first, then
   `models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite`.
3. Stage a static image in the SIM FS root.  First stage can use pre-resized
   raw RGB to avoid image decoder risk; second stage should accept a classic
   BMP/PNG/JPEG through `sentai.tpu.load_image(path)` or, preferably, a
   file-backed virtual camera.

```python
import sentai
sentai.tpu.load('/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite')
sentai.tpu.set_input(sentai.fs.read('/images/coco_300x300.rgb'))
sentai.tpu.invoke()
sentai.pipeline.detections(300)
```

4. Verify the final no-camera REPL shape:

```python
import sentai
sentai.tpu.load('/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite')
sentai.tpu.load_image('/images/cat.png')
sentai.tpu.invoke()
sentai.pipeline.detections(300)
```

Only after this passes should B7 reconnect the live camera/pipeline path to the
same backend.

5. Add the virtual camera path:

```python
sentai.camera.switch(-1, '/images/cat.png')
sentai.pipeline.step()
sentai.pipeline.detections(300)
```

This validates the full pipeline without Gazebo/camera hardware while avoiding
new image-specific APIs in `sentai.pipeline`.

Implementation note: the virtual camera is not SIM-only.  It should be a
shared ARM/SIM camera source selected through the normal `sentai.camera`
namespace, with camera id `-1` meaning "static image from FS".  The platform
backends may differ only at the file/device boundary.  Once selected, TPU,
markers, flow, safety, and pipeline code should consume it as a normal camera
frame.  For the first safe slice, BMP is sufficient as the classic test-image
format; PNG/JPEG decoding can come later if needed.

This is also an ARM hardware test fixture, not only a SIM convenience.  With a
static image selected as the active camera, we can run deterministic
end-to-end TPU detection tests on the real board and assert that the expected
objects and bounding boxes are produced from known pixels.

Future extension: the same virtual-camera source should be able to play a
deterministic frame sequence, not only one static image.  That would let us run
Flow, Markers, TPU, Safety, and PrepTask consumers against repeatable
frame-by-frame "video" fixtures on both ARM and SIM.

Test-fixture convention: virtual-camera BMP fixtures should normally be
camera-native 640x480, 4:3.  This keeps TPU bounding boxes, marker image
coordinates, and flow scale directly comparable with live camera runs.

PrepTask rule: the virtual camera should feed the same frame source that
PrepTask consumes.  Resize/slot generation for Flow, Markers, TPU, Safety, and
other perception consumers should stay in PrepTask or the documented SIM
PrepTask equivalent, not in ad-hoc per-consumer virtual-image shortcuts.

This preserves the single-source rule: the REPL surface remains `sentai.tpu`
and `sentai.pipeline`; SIM-specific work is only the POSIX USB/backend and
possibly image decode plumbing.

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
- `sentai_sim` now tees process stdout/stderr into
  `<fs_root>/fr/debug.log` while keeping console output visible.  This captures
  C/C++ `printf`/`fprintf` debug from startup, camera bridge, pipeline, and
  TPU path next to `events.csv` and `scalars.csv`;
- ARM `_write()` now pushes printf bytes into the bounded `sentai.fr` `debug`
  channel before applying the USB-console `verbose` gate.  The recorder task
  drains that ring to the path opened with `sentai.fr.open("debug", path)`,
  using append on ARM.  This keeps debug logging best-effort and out of hot
  control paths;
- test flow:
  1. host PyCoral runs the COCO EdgeTPU model against the same 640x480 BMP;
  2. SIM starts the TPU helper, selects the virtual camera image, loads the
     same board-style model path from its per-run FS root, calls
     `sentai.pipeline.step()`, then reads compact detections;
  3. host detections and SIM detections must match exactly.
- first passing run:
  `s209_virtual_camera_tpu_e2e/iter02_virtual_camera_tpu_cat`;
- current passing debug-log run:
  `s209_virtual_camera_tpu_e2e/iter10_virtual_camera_tpu_cat`;
- current run imports `/mission_s209.py` from the experiment FS root; stdin
  only performs `import mission_s209; mission_s209.run()`;
- result: host PyCoral detections and SIM detections are byte-for-byte equal
  after aligning the host baseline resize to SIM's integer nearest-neighbour
  resize.
- REPL stdout is not treated as a robust data channel.  Async debug from
  background tasks can interleave with printed values, so s209 writes compact
  detections to `/detections.txt` in the experiment FS root and parses that
  file host-side.  This follows the existing rule: small metadata can cross MP,
  but reproducible test artifacts live in FS.
- debug overlay policy: generated visual artifacts should come from the
  runtime FS, not from MP image bytes.  s209 now calls
  `sentai.pipeline.save('/images/detected_overlay.bmp')`; the SIM runtime
  draws the best cat detection over the current 640x480 camera frame and writes
  the BMP directly to `<iter>/fs_root/images/detected_overlay.bmp` from C.
  Host-side PNG overlays may still be generated as convenience views, but they
  are secondary to the runtime-produced FS artifact.
- current verified artifacts in iter10:
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
| SIM pipeline | `sentai.pipeline.start(fps)` | `sentai.pipeline.stop()` | `running()`, `stats()`, `tick()` | SIM smoke/demo worker.  Keep separate from canonical PrepTask until ownership is resolved. |

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
- `sentai.aifes` and `sentai.tfl` are exposed as explicit SIM stubs:
  `ready()` is false and load/invoke operations return errors until their
  runtime backends are ported.  This preserves namespace discoverability
  without pretending ML execution is implemented.
- `sentai.slam` parity is now present in SIM, so D7-style model-substrate
  experiments do not need a SIM-only fallback for that namespace.
- `sentai.tpu` still has the pragmatic SIM Coral USB helper path through
  `sim_tpu_helper.py` and `sim_tpu_shim.c`, but B7 now also has a direct
  POSIX/libusb smoke target (`tpu_posix_smoke`).  It performs cold DFU from
  `apex_latest_single_ep_bin.c` when the USB stick enumerates as
  `1a6e:089a`, reopens app mode `18d1:9302`, initializes the shared
  `libs/tpu/edgetpu_driver.cc` path, and reads EdgeTPU temperature.
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

- achieved: **runtime surface parity** between ARM and SIM for exported
  `sentai.*` names;
- achieved: top-level helpers are now single-source shared bindings;
- achieved: model-substrate names from B6/D8 (`objects`, `object_lifter`,
  `places`, `slam`) are visible in SIM;
- achieved with caveat: `aifes` and `tfl` are visible but are SIM stubs, so
  this is namespace parity, not ML-runtime parity;
- achieved with caveat: direct POSIX USB/libusb transport parity is proven by
  `tpu_posix_smoke`, but SIM `sentai.tpu.load/invoke` still uses the helper
  until the TFLite Micro / executable model layer is wired to that transport;
- not done: `sentai.prime`, command dictionary, minimal executive, active
  primitive table, ACK/RUN/OK/ERR protocol;
- not done: formal board-model query facade from B6/D7;
- started: C++ task inventory and PrepTask producer/consumer documentation.

## Realized So Far

This is the concrete B7 progress at this point:

1. ARM and SIM root `sentai.*` namespace parity is now achieved, with only the
   expected SIM-only `sentai.sim` namespace.
2. Several namespaces that were ARM-only are now included from the shared
   `examples/sentai_runtime` binding sources in SIM instead of being duplicated.
3. Top-level helper commands `sentai.help`, `sentai.debug`, `sentai.console`,
   and `sentai.run` were moved to the shared `modsentai_top.c` binding.
4. SIM platform-specific behavior was isolated in `sentai_platform_sim_backend.c`
   so shared bindings can compile without inventing separate SIM APIs.
5. `sentai.aifes` and `sentai.tfl` now have explicit SIM namespace stubs:
   discoverable, honest, and not pretending runtime support exists yet.
6. `scripts/sentai_namespace_inventory.py` now checks ARM/SIM namespace drift
   from source.
7. `smoke_namespace_model.py` plus `scripts/sentai_sim_model_smoke.py` now
   verify the model-substrate namespace in SIM without starting control loops
   or mutating the world model.
8. The first C++ task inventory is documented, including the key PrepTask
   producer/consumer contract and the remaining flow/pipeline duplication
   questions.
9. The current task lifecycle/API surface is documented for flow, SLAM,
   safety, FR, B3/B4 controllers, legacy autotune, Crazyflie/link/mesh
   transport, SIM camera bridge, and SIM pipeline.
10. Direct host EdgeTPU USB transport is implemented as an experimental smoke:
    `sim/usb_host_edgetpu_posix.c` provides the `USB_HostEdgeTpu*` contract
    over `libusb`, including cold DFU firmware load, and
    `sim/tpu_posix_smoke.cc` verifies `TpuDriver::Initialize()` plus
    temperature read against the Coral USB stick.

This matches A7's non-goal: B7 should not introduce the full `sentai.prime`
dispatcher or mission language yet.  The B6 priority now is not more API
surface work; it is understanding and documenting the C++ task/runtime core so
future primitives know which controller/task owns each loop.

Recommended next B7 priorities:

1. Validate the lifecycle inventory against the actual MP bindings and mark
   any missing status/abort surfaces that would block future primitives.
2. Extend the smoke checks toward top-level helpers and controller status
   surfaces while keeping them read-only.
3. Wire model load/invoke onto the direct POSIX TPU transport, replacing the
   helper path only after parity is proven with a small model such as
   `models/testconv1-edgetpu.tflite`.
4. Keep `aifes`/`tfl` runtime backend migration as a follow-up after the task
   inventory clarifies where ML inference/training should actually run.
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
sentai.tfl.ready()    -> False
```

## Current Conclusion

The first B7 slice stayed non-behavioral: it widened SIM namespace surface
parity by including shared runtime bindings and adding only platform backend
shims or explicit stubs.  This is aligned with A7/B6 as long as `aifes`/`tfl`
are treated as visible-but-not-implemented ML runtime backends in SIM.

The next B7 work should shift from namespace parity to the runtime core:
document `*Task` ownership, PrepTask data flow, producer/consumer buffers,
status/cancel/abort contracts, and regression checks.  No `sentai.prime`
dispatcher or model-query facade should be implemented until that task
inventory is clear.

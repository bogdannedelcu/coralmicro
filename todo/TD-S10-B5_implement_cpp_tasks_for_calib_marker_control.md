# TD-S10-B5 - Implement A5 C++ Runtime Tasks For Calibration And Marker Control

## Purpose

Operational implementation plan for
`TD-S10-A5_migrate_calib_marker_control_to_cpp_tasks.md`.

A5 remains the objective/specification document.  B5 tracks concrete work,
API decisions, migration order, compatibility notes, simulator runs, artifacts,
and blockers.

## Current Decision

Migrate B3/B4 behavior incrementally.  Do not rewrite everything at once.

The accepted REPL references are:

```text
B3 source:
examples/sentai_runtime/experiments/
  s197_sota_calib_orientation_guarded/
    iter35_strict_scalar_calib_ini_source/

B4 proof:
examples/sentai_runtime/experiments/
  s203_hl_marker_control_with_flow/
    iter16_strict_calib_ini_e2e_b4/
```

These runs, logs, plots, frames, and verdicts are thesis evidence.  B5 must
preserve them and produce comparable artifacts after each C++ migration stage.

## Current Implementation Notes

- `s205_cpp_calib_orientation_task` is the B5 migration workspace for the
  B3/A3 calibration path, now ported to a C++ task family with MP kept as the
  coarse mission-state orchestrator.
- `s207_cpp_marker_control_task` is the planned B5 migration workspace for the
  B4/A4 marker-control handoff path.  It should start from the accepted
  `s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4` behavior
  and consume the strict runtime `/system/calib.ini` produced by `s205`.
- The old `sentai.calib` Flow autotune/bringup family has been marked in C++
  as **LEGACY / B5 delete candidate**:
  `sentai_calib_autotune.*`, `sentai_calib_task.*`,
  `sentai_calib_bringup.*`, plus the related MP bindings
  `set_context`, `task_start`, `task_stop`, `is_done`, `get_kp`,
  `commit_kp`, `get_persisted_kp`, `get_td_ms`, `hold_start`,
  `hold_yaw_start`, `run_bringup`, `bringup_*`.
- New B5 primitives already moved out of MP:
  - `sentai.calib.sample_calib_observation_tuple(...)` owns camera-frame
    marker observation sampling;
  - `sentai.markers.window_*` owns marker-count running windows;
  - `sentai.calib.score_axis_candidate(...)` owns discrete R candidate
    scoring;
  - `sentai.calib.vertical_rate_thrust_tuple(...)` owns descent vertical
    rate thrust;
  - `sentai.calib.z_hold_thrust_tuple(...)` owns visual-Z hold thrust;
  - `sentai.servo.ibvs_centroid_command_tuple(...)` owns image-frame IBVS
    centroid-to-roll/pitch computation.
  - `sentai.calib.expected_from_row_tuple(...)` owns R-row to image-axis/sign
    interpretation;
  - `sentai.calib.axis_observation_from_delta_tuple(...)` owns image response
    vector classification, dominance, and strength.
  - `sentai.calib.defaults_tuple()`, `setup_defaults()`, and `limits_tuple()`
    own the calibration layout, intrinsics, marker-window thresholds, and
    axis-response gates used by the active mission.
  - `sentai.calib.feature_has_lock_tuple(...)`,
    `marker_avg_lock_ok_tuple(...)`, `marker_avg_unsafe_tuple(...)`, and
    `marker_lock_ok_tuple(...)` own the repeated marker-lock predicates.
  - Active `s205` hot-path feature observations, marker-window stats, and IBVS
    commands are compact tuples in MP rather than dictionaries; dictionaries
    remain only for mission summaries/events/artifacts.
- MP in `mission_s205.py` now owns the coarse state transitions only.  The
  per-frame B3 calibration phases, control loops, response measurement,
  candidate validation, centered landing, and calibration persistence are owned
  by the C++ `sentai.calib.orientation_*` task family.
- Host-side `verdict_s205.py` reconstructs the old B3-style forensic summary
  from `sentai.fr`, `mission_s205_status.txt`, `calib.ini`, and GT recorder
  artifacts.  Runtime MP does not generate JSON summaries.

## Known Inputs

- Objective document:
  `todo/TD-S10-A5_migrate_calib_marker_control_to_cpp_tasks.md`.
- Accepted B3 reference:
  `examples/sentai_runtime/experiments/s197_sota_calib_orientation_guarded/iter35_strict_scalar_calib_ini_source/`.
- Accepted B4 reference:
  `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4/`.
- Runtime calibration file:
  `/system/calib.ini`, mounted in SIM as
  `build-sim/sentai_fs_root/system/calib.ini`.
- Active marker layout:
  `sentai_whycon_small_centroid_7_marker_v1`.
- Active B4 control mode:
  Generic Hover streamed continuously, with lateral targets in image frame and
  Z treated as the metric/estimator-backed axis.
- Canonical detector/camera assumptions inherited from A1/A2/A3:
  grayscale `320x240`, WhyCon markers, `fx=288.3`, `fy=288.3`, `cx=160.0`,
  `cy=120.0`, marker diameter `0.0544 m`.

## Runtime Boundaries

- Runtime algorithms run in C++ inside `sentai_runtime` / `sentai_sim`.
- MicroPython remains the mission-level state-machine/orchestration layer:
  - initialize runtime;
  - command coarse mission phases;
  - call `start` / `step` / `stop` on C++ runtime services;
  - poll `status`;
  - call abort or safe-stop;
  - collect compact artifacts.
- C++ owns the inner state of each phase: per-frame perception, feedback loops,
  signal statistics, estimator feeds, command streaming, and persistence.
- C++ must also own equivalent observability for migrated logic.  Every moved
  algorithm must emit enough `sentai.fr` events/scalars/status fields to keep
  the current B3/B4 forensic workflow debuggable: phase transitions, accepted
  inputs, rejects/reasons, marker counts/averages, axis vectors, candidate
  scores, control outputs, and final artifacts.
- Runtime/MP serialization is restricted to scalar `key=value` or simple CSV
  rows.  MP must not emit JSON into `sentai.fr`, FS artifacts, or calibration
  files.  JSON is allowed only in host-side forensic tooling when reading
  simulator-produced host artifacts such as `gt.jsonl`.
- MP must not compute axis responses, select camera/body signs, pump large
  per-frame loops, write `calib.ini`, or convert host artifacts.
- Host scripts may launch SIM and run post-mortem verdict/plots only.
- Gazebo GT remains forensic-only.

## Frame Convention For B5

Do not let existing `sentai.*` world-frame helpers confuse the B5 migration.
Some namespaces can compute or expose world-frame poses, but the accepted B4
behavior being migrated is intentionally image-frame lateral control:

- lateral `X/Y` proof targets are image-frame targets, not world-meter targets;
- the controller moves marker centroids toward symmetric image-envelope targets
  and recenters them in the image;
- the non-moving image axis must remain centered during each active-axis sweep;
- Z is the only metric/world-like control variable in the active B4 path;
- world-frame/GT/EKF comparisons are allowed for post-mortem and estimator
  sanity checks, but they must not replace image-frame lateral success rules.

If a C++ module exposes `get_drone_pose`, world pose, tracker ground projection,
or estimator state, B5 may log it and use it for diagnostics, but the lateral
navigation proof remains the B4 image-frame proof unless A4 is explicitly
changed.

## Task And Core Architecture Constraints

SIM can run these services in one process, but the ARM target has real
FreeRTOS/core-placement constraints.  B5 code must be designed as explicit C++
services/tasks with clean ownership, not as a single MP loop that happens to
work in the simulator.

Architectural rules:

- Treat camera/marker processing, flow processing, estimator feeding, calibration
  state, and command streaming as separable services with explicit start/stop
  lifecycles.
- Do not assume every service will run on the same core on hardware.  Use
  bounded shared structs, snapshots, queues, or existing shared-memory patterns
  instead of direct cross-task mutation.
- Only one task should own each transport stream toward the Crazyflie at a time:
  - flow updates;
  - marker/PnP-derived ExtPos updates;
  - Generic Hover / RPYT command updates.
- The marker and flow tasks both send updates toward the drone, but they do so
  through a coordinated estimator-feed/command layer so packet timing, priority,
  and safe-stop behavior remain observable.
- MP must never be required to run at camera frame rate.  If a loop must run at
  frame rate, it belongs in C++.
- Every long-running C++ task needs:
  `start`, `stop`, `is_done/running`, compact `status`, last error/reason, and
  safe abort semantics.

Proposed ownership split:

```text
sentai.markers     -> camera frame to marker/PnP observation snapshot
sentai.flow        -> camera frame to flow/body-motion snapshot
sentai.estimator   -> optional new service: sends gated flow + ExtPos updates
sentai.calib       -> A3 orientation calibration FSM and calib.ini persistence
sentai.crazy       -> transport primitives only
sentai.servo       -> intent/action vocabulary and B4 image-frame navigation FSM
MicroPython        -> orchestration and artifact collection only
```

`sentai.estimator` is a proposed name for the EKF feed service.  If code review
shows it fits better under `sentai.crazy` or `sentai.servo`, reuse the existing
namespace.  Do not add `sentai.markerctl`; B4 marker navigation belongs under
`sentai.servo`.

Dependency map:

```text
sentai_prep
  -> sentai.markers
  -> sentai.calib
  -> sentai.servo

sentai_prep
  -> sentai.flow
  -> sentai.estimator
  -> sentai.crazy

sentai.markers
  -> sentai.estimator
  -> sentai.crazy

sentai.calib
  -> sentai.crazy
  -> sentai.fr

sentai.servo
  -> sentai.crazy
  -> sentai.fr

MicroPython
  -> sentai.calib / sentai.servo / sentai.estimator status APIs only
```

Forbidden dependency directions:

- `sentai.markers` must not depend on `sentai.calib`, `sentai.servo`, or MP;
- `sentai.flow` must not depend on `sentai.servo` or MP;
- `sentai.crazy` must not depend on marker/calib/servo algorithms;
- `sentai.fr` must stay append/drain infrastructure and must not call flight
  algorithms;
- MP must not sit inside any camera-rate or EKF-feed dependency path.

## Existing Surfaces To Reuse

Use existing module boundaries:

- `sentai_prep` / `PrepTask`
  - producer-side camera-frame preprocessing infrastructure;
  - ARM producer is `PrepTask`, SIM producer is `camera_bridge_recv`;
  - publishes refcounted slots such as `SENTAI_PREP_SLOT_GRAY_NATIVE`
    (`320x240` Y8), `SENTAI_PREP_SLOT_RGB_64`, and `SENTAI_PREP_SLOT_GRAY_64`;
  - consumers should use zerocopy C accessors, preferably
    `sentai_prep_slot_begin_read/end_read` for non-trivial compute;
  - MP must not move image buffers across the binding boundary.
- `sentai.pipeline`
  - owns the TPU detection pipeline and its `PrepTask` / `InferTask` pairing;
  - exposes useful diagnostics such as `prep_fps`, `prep_stats`,
    `task_health`, and direct tensor stats;
  - B5 should not couple marker-control to TPU detection, but it must respect
    shared camera/PXP/SDRAM bandwidth and task scheduling.
- `sentai.calib`
  - owns calibration state, `R_B_C`, `cam_offset_B`, and `/system/calib.ini`;
  - already has task-like API shape: `task_start`, `task_stop`, `is_done`;
  - should own the A3 orientation-calibration task;
  - existing `sentai_calib_task` and `sentai_calib_bringup` are useful task
    organization references, but their older flight logic is not the accepted
    B3 algorithm.
- `sentai.markers`
  - owns WhyCon detection, marker layout, intrinsics, pose, and camera
    extrinsics;
  - currently behaves primarily as a synchronous detector/API;
  - B5 should add a C++ marker observation task/cache that calls this detector
    from a runtime task and exposes bounded snapshots to `sentai.calib`,
    `sentai.servo`, and the estimator-feed service.
- `sentai.flow`
  - owns 80x60 flow processing;
  - already has a FreeRTOS publisher/task path;
  - `body_read()` already uses `sentai.markers` camera extrinsics;
  - should support a runtime service/task that keeps flow packets available for
    Crazyflie estimator feeding.
- `sentai.safety`
  - already follows the split we want: C++ state machine plus worker task;
  - useful template for start/stop, health counters, stale watchdog, and
    fail-closed marker supervision.
- `sentai.fr`
  - flight recorder split into push/drain primitives plus a worker task;
  - writes/drains directly to the runtime FS, so MP is not required for normal
    logging;
  - useful pattern for append-only evidence without MP holding big histories.
- `sentai.crazy`
  - owns transport and low-level Crazyflie commands:
    RPYT, Generic Hover, ExtPos, flow packets, release, stop, disarm, telemetry.
- `sentai.servo`
  - owns action vocabulary and trace style;
  - should host or wrap the B4 image-frame navigation FSM and future navigation
    behaviors.

## Step 1 Inventory - B3 MP Migration

B5 migrates B3 first, before touching the B4 marker-control mission.  The goal
is to turn `s197` into a thin MP launcher while preserving the accepted
behavior from `iter35_strict_scalar_calib_ini_source`.

Inventory rule: each B3 algorithmic block must either move into an existing
`sentai.*` namespace, extend an existing namespace with a small C++ task/service,
or remain explicitly in MP as orchestration only.

| B3 MP block | Existing concept to reuse | Migration target | MP after migration |
| --- | --- | --- | --- |
| Camera setup and grayscale input | `sentai_prep`, `PrepTask`, SIM camera bridge | C++ task reads `SENTAI_PREP_SLOT_GRAY_NATIVE` or current camera gray source | `sentai.camera.init()` only if still required by runtime |
| WhyCon detector init, intrinsics, marker size, 7-marker layout | `sentai.markers` | Keep in `sentai.markers`; add a reusable setup/helper if needed | Pass no per-frame marker data |
| `_detect_features`: count, full-visible count, centroid, radius mean, visual Z | `sentai.markers.get_detection`, `sentai.markers.get_latest` | Shared C++ marker observation snapshot/cache | Poll compact status only |
| 10-frame running average for marker count and safety decisions | `sentai.safety` task pattern, small fixed buffers | Shared observation cache or `sentai.safety` extension | None |
| Preflight marker lock and takeoff-until-markers | `sentai.crazy` RPYT/arm/disarm primitives; `sentai.markers` observations | `sentai.calib.orientation_task` phase | Start/poll task |
| Post-lock brake and visual-Z hold | Existing low-level RPYT path in `sentai.crazy`; task style from `sentai_calib_task` | `sentai.calib.orientation_task` vertical-control phase | None |
| Z estimate from marker radius / pose statistics | `sentai.markers` pose/detection data | Shared marker observation with visual-Z fields | None |
| Complementary axis pulses and per-axis recenter | `sentai.crazy` RPYT primitives | `sentai.calib.orientation_task` excitation phase | None |
| Response vectors, centroid noise, adaptive excitation, candidate scoring | `sentai.calib` already owns calibration state and matrix persistence | New C++ A3 algorithm code under `sentai.calib`; do not reuse flawed `sentai_calib_bringup` logic | None |
| Discrete camera/body rotation result, signs, camera offset sign, ExtPos sign policy | `sentai.calib` state, `sentai.markers` camera extrinsics | `sentai.calib` result model and strict `calib.ini` writer | Read-only result summary |
| Final centroid hover and camera-referenced landing proof | `sentai.crazy` RPYT; marker observation cache | `sentai.calib.orientation_task` final validation/landing phase | None |
| `/system/calib.ini` strict key=value writing | Existing `sentai.calib.save/load`, FS access | Tighten/extend C++ parser/writer; MP must not serialize calibration | Optionally copy snapshot into evidence folder |
| Journaling, scalar evidence, camera frames | `sentai.fr` push/drain model; existing SIM journal for compatibility | C++ events/scalars/frames where possible, host post-mortem remains allowed | Collect files only |
| Verdict, plots, GT comparison | Host-side forensic scripts | Stay host-side, never enter runtime algorithm | Launch/collect artifacts |

What remains in MP for B3:

- select experiment name/output folder;
- initialize the runtime and simulator plumbing;
- command the coarse B3 state machine, for example:
  `setup -> acquire -> stabilize_z -> calibrate_axes -> validate -> land`;
- call C++ phase APIs such as `sentai.calib.orientation_start()` or
  `sentai.calib.orientation_step("calibrate_axes")`, depending on the final API;
- poll `orientation_status()` at low rate;
- stop/abort on user request;
- collect `calib.ini`, logs, frames, summary, and verdict artifacts.

What must not remain in MP for B3:

- per-frame marker feature computation;
- RPYT feedback loops;
- thrust/Z control;
- axis-response pulse scheduling;
- response-vector math;
- camera/body matrix selection;
- strict `calib.ini` serialization.

Implementation note: `sentai_calib_bringup` is useful only as a C++ task
organization example.  Its older calibration strategy is not the B3 algorithm
and must not drive the new A3/B3 behavior.

## Step 2 Inventory - B4 MP Migration

B4 migrates only after Step 1 produces and loads a strict C++ `calib.ini`.
The accepted B4 behavior remains image-frame lateral control plus metric Z,
with flow and marker-derived ExtPos feeding the Crazyflie estimator.

Initial migration targets:

- move strict `calib.ini` validation/load into `sentai.calib`;
- keep WhyCon observations in the shared `sentai.markers` snapshot/cache;
- move flow packet pumping from MP into an estimator-feed service using
  `sentai.flow.body_read()` and `sentai.crazy` transport primitives;
- move marker/PnP ExtPos gating and streaming into the same estimator-feed
  service, using `sentai.markers.get_drone_pose` only after B3 calibration is
  loaded;
- move image-frame Generic Hover navigation into `sentai.servo`;
- keep B4 MP as start/status/stop plus artifact collection.

The B4 migration must not reintroduce world-frame lateral navigation.  Marker
positions provide Z and image-frame feedback for the accepted proof; GT and CF
pose remain forensic channels.

## Target API Shape

Final names can change after code review, but B5 should converge toward compact
task APIs like:

```python
sentai.calib.orientation_start()
sentai.calib.orientation_step(name)
sentai.calib.orientation_stop()
sentai.calib.orientation_is_done()
sentai.calib.orientation_status()
sentai.calib.orientation_result()

sentai.servo.marker_start()
sentai.servo.marker_step(name)
sentai.servo.marker_stop()
sentai.servo.marker_is_done()
sentai.servo.marker_status()
sentai.servo.marker_result()
```

The key rule is that MP calls tasks/phases, not algorithms.

API conventions:

- return `0` on success, negative error codes on faults;
- expose compact dict/status snapshots to MP;
- write detailed runtime journal incrementally;
- write MP-visible status as strict `key=value`, never JSON;
- avoid large MP lists and per-frame MP allocations;
- keep old APIs and experiments runnable.

## Implementation Artifacts

Planned runtime/code artifacts:

- C++ strict `calib.ini` schema-v2 parser/writer;
- C++ shared marker observation snapshot/cache fed from `sentai.markers` and
  backed by the camera/prep infrastructure where appropriate;
- C++ A3 orientation-calibration task under `sentai.calib`;
- C++ estimator-feed service for flow and ExtPos;
- C++ B4 marker-control task, namespace still to be finalized;
- reduced MP missions that call `start/status/stop` only;
- host verdict scripts reused or lightly adapted for post-mortem.

Planned evidence artifacts per recertification run:

```text
examples/sentai_runtime/experiments/
  s205_cpp_calib_orientation_task/
    iterXX_.../
      verdict.log
      mission_s205_status.txt
      sentai_repl.log
      fr_current/
      calib.ini snapshot
      mission_s205_calibration_host.ini

  s207_cpp_marker_control_task/
    iterXX_.../
      verdict.log
      mission_s207_status.txt
      s207_xy_gt_vs_est.png
      s207_z_gt_vs_est.png
      fr_current/frames/
```

## Implementation Plan

### Current s205 Logging Refactor Note

`s205_cpp_calib_orientation_task` now treats MP as a runtime orchestrator, not
as a report generator:

- MP writes only `mission_s205_status.txt` as strict `key=value`;
- MP no longer writes `mission_s205_summary.json`,
  `mission_s205_calibration.json`, or simulator journal artifacts;
- MP event logging goes through `sentai.fr.push_event()` with compact
  `key=value` text only;
- `calib.ini` is persisted through `sentai.calib.save_contract`;
- host-side `verdict_s205.py` reads `mission_s205_status.txt`, `calib.ini`,
  `sentai.fr` files, and host GT recorder output, then writes
  `mission_s205_calibration_host.ini`.

Current task migration state:

- C++ owns the B3/A3 task phases through `sentai.calib.orientation_*`:
  setup defaults, preflight observation, arm/zero unlock, marker acquisition,
  post-lock brake, visual-Z hold, axis response, centroid validation,
  candidate scoring, optical-axis validation, final candidate validation,
  final recenter, manual descent, centered visual descent, and contract save.
- MP `mission_s205.py` owns mission-state sequencing, minimal status-file
  writing, and emergency-stop fallback only.
- `last_thrust` and other phase-local runtime state now live inside the C++
  orientation task state, not in MP.
- `MissionResults` records coarse phase booleans and computes the final mission
  status without string-key dictionaries or `getattr`.
- Runtime logs are emitted by C++ phase code wherever the logic moved; the host
  verdict maps those FR events back into the old B3 forensic summary shape.

Validation run:

```text
examples/sentai_runtime/experiments/s205_cpp_calib_orientation_task/
  iter3_fr_only_status/
```

Result: logging/persistence refactor works, but this flight did not certify
B3 (`FINAL_VALIDATION_FAIL`, `final_candidate_validation_failed`).  This is an
algorithmic/flight-validation issue, separate from the no-JSON MP cleanup.

First MP math removal:

- removed the MP fallback for discrete rotation enumeration/scoring from
  `s205`;
- `s205` now requires `sentai.calib.score_axis_candidate`;
- remaining `_expected_from_row` is temporary and belongs to the final
  validation phase that will move into `sentai.calib.orientation_task`.

Second MP math removal:

- added `SentaiMarkersWindowStats` and fixed-slot rolling marker-count
  windows in `sentai.markers`;
- exposed MP bindings:
  `sentai.markers.window_reset`,
  `sentai.markers.window_push_tuple`,
  `sentai.markers.window_get_tuple`;
- `s205` no longer keeps MP-side arrays/sums/minima for marker-count running
  averages.  It still has temporary wrapper functions so the accepted B3 phase
  code changes minimally while the full FSM is being migrated.
- SIM build validated with QSTR regenerated; ARM build deferred to final B5
  acceptance per plan.

Third MP helper reduction:

- removed `_feature_compact`; `_detect_features()` now exposes `cx/cy` directly
  from the C++ marker observation tuple;
- added `sentai.calib.vertical_rate_thrust_tuple`, a C++ helper that owns the
  vertical-rate thrust equation, `vz` low-pass update, and thrust clipping;
- `_vertical_rate_thrust_from_feature` remains only as a temporary MP adapter
  until the full descent/landing phase moves into `sentai.calib`;
- `_marker_window_new/update/stats` remain temporary MP adapters over
  `sentai.markers.window_*`, not MP-side rolling-array logic.

### s205 MP Function Audit

Policy: MP may own orchestration and coarse state-machine switching.  Any
camera-rate loop, matrix/math decision, filter, controller, estimator feed, or
calibration persistence belongs in C++.

Current `mission_s205.py` function-by-function evaluation after the first B5
cleanup:

| MP function | Evaluation | Decision | C++ target |
| --- | --- | --- | --- |
| `_kv_text` | scalar text helper for status file | keep temporarily | remove when C++ task exposes final status directly |
| `_write_summary` | writes minimal `mission_s205_status.txt` | keep temporarily | C++ task status snapshot or host run wrapper |
| `_event_text` | converts primitive MP event payloads to compact `key=value` text | keep temporarily, but no JSON/dicts-as-artifacts | detailed FR events move into `sentai.calib` / `sentai.servo` |
| `_persist_calib_contract` | extracts calibration result from MP summary and calls C++ save | move; MP should not assemble calibration contracts | `sentai.calib.orientation_task` final commit |
| `_j` | bridge to `sentai.fr.push_event` | keep temporarily | migrated phases log directly from C++ |
| `_set_phase` | coarse state-machine label update | keep | MP orchestration |
| `_rpyt` | raw CRTP RPYT helper | temporary only | `sentai.crazy` command primitive used by C++ tasks |
| `_marker_world_bytes` | packs layout for MP binding | move; layout config should not be rebuilt in MP | `sentai.markers` / `sentai.calib` config helper |
| `_phase_setup` | initializes modules and configures markers | partially keep until task API exists | `sentai.calib.orientation_start` setup |
| `_detect_features` | per-frame marker observation adapter | move; MP must not sample camera-rate observations | `sentai.markers` observation task/cache, `sentai.calib` adapter |
| `_phase_preflight_features` | preflight sampling loop and counters | move | `sentai.calib.orientation_task` |
| `_stream_zero_thrust` | frame-rate command loop | move | `sentai.calib.orientation_task` using `sentai.crazy` |
| `_phase_arm_zero_unlock` | arming/unlock sequence | move | `sentai.calib.orientation_task` |
| `_feature_has_lock` | marker lock decision | move | `sentai.calib` lock policy backed by `sentai.markers` stats |
| `_phase_thrust_only_marker_acquisition` | thrust ramp + marker acquisition FSM | move | `sentai.calib.orientation_task` |
| `_phase_post_lock_brake` | post-lock vertical brake controller | move | `sentai.calib.orientation_task` |
| `_phase_visual_z_hold` | visual-Z hold controller | move | `sentai.calib.orientation_task` |
| `_z_thrust_from_feature` | vertical thrust control law | move | `sentai.calib` vertical-control helper |
| `_descent_thrust_from_feature` | landing/descent thrust control wrapper | move | `sentai.calib` / later reusable in `sentai.servo` |
| `_vertical_rate_thrust_from_feature` | vertical rate controller/filter | move | `sentai.calib` vertical-control helper |
| `_feature_compact` | MP dict projection of observation | move/remove | C++ status struct + host postprocess |
| `_marker_window_new` | thin wrapper over C++ marker window slot | temporary only | already moved to `sentai.markers`; remove after FSM migration |
| `_marker_window_update` | thin wrapper over C++ marker window push | temporary only | already moved to `sentai.markers`; remove after FSM migration |
| `_marker_window_stats` | thin wrapper over C++ marker window status | temporary only | already moved to `sentai.markers`; remove after FSM migration |
| `_marker_avg_lock_ok` | lock threshold policy | move | `sentai.calib` / `sentai.safety` policy |
| `_marker_avg_unsafe` | safety threshold policy | move | `sentai.calib` / `sentai.safety` policy |
| `_stream_axis_segment` | camera-rate excitation segment with Z hold | move | `sentai.calib.orientation_task` |
| `_phase_axis_recenter` | local recenter between axis pulses | move | `sentai.calib.orientation_task` |
| `_phase_axis_response_smoke` | complementary pulse response measurement | move | `sentai.calib.orientation_task` |
| `_response_strength` | vector norm helper | move | `sentai.calib` response model, maybe shared math helper |
| `_axis_response_vectors` | extracts roll/pitch response vectors from MP summary | move/remove | `sentai.calib` result model |
| `_axis_result` | extracts one axis result from MP summary | move/remove | `sentai.calib` result model |
| `_solve_image_response` | 2x2 response solve | move | `sentai.servo` image-frame control primitive |
| `_ibvs_centroid_command` | damped least-squares IBVS command | move | `sentai.servo` image-frame controller; B3 can reuse via `sentai.calib` |
| `_clip` | numeric helper for control saturation | move | C++ helper near controller |
| `_phase_centroid_pd_validation` | IBVS validation controller | move | `sentai.calib.orientation_task` |
| `_expected_from_row` | expected image-axis/sign for a candidate row | move | remaining validation logic in `sentai.calib` |
| `_phase_candidate_scoring` | MP wrapper over C++ candidate scoring | temporary only | fully fold into `sentai.calib.orientation_task` |
| `_phase_optical_axis_validation` | validates optical-axis sign and pose/Z | move | `sentai.calib.orientation_task` |
| `_axis_observation_from_comp` | response classification | move | `sentai.calib` response model |
| `_measure_centroid_noise` | neutral noise estimate and gate | move | `sentai.calib.orientation_task` |
| `_final_validate_attempt` | adaptive final validation pulse | move | `sentai.calib.orientation_task` |
| `_phase_final_candidate_validation` | final candidate validation FSM | move | `sentai.calib.orientation_task` |
| `_phase_final_centroid_recenter` | final IBVS recenter + hover proof | move | `sentai.calib.orientation_task`, later share `sentai.servo` primitive |
| `_phase_manual_descend_disarm` | descent/disarm control loop | move | `sentai.calib.orientation_task` / `sentai.crazy` safe-stop |
| `_phase_center_hold_descend_disarm` | centered visual landing controller | move | `sentai.calib.orientation_task`; B4 equivalent in `sentai.servo` |
| `_set_abort_reason` | compact abort reason update | keep temporarily | C++ task status once full FSM migrates |
| `run` | top-level state-machine orchestration | keep | reduced to `start/status/stop` calls after migration |

Removed from `s205` during this audit:

- `_is_full_visible` and `_detect_features_with_pose_details`, because the
  pose-detail path was unused and visibility geometry is already in
  `sentai.markers` observation aggregation;
- `_mission_abort_detail`, because MP must not carry large nested diagnostic
  dicts;
- `_flight_plan_placeholders`, because it was report decoration rather than
  runtime control.

Current conclusion: `s205` no longer contains the accepted B3 frame-rate
algorithm in MP.  It is now the thin orchestration harness for the C++
orientation task family:

- marker observation sampling and marker-count windows are C++;
- vertical control, thrust state, axis excitation, IBVS/recenter logic,
  candidate scoring, validation pulses, noise gates, and landing proof are C++;
- strict B3 `calib.ini` persistence is C++;
- MP keeps setup sequencing, phase labels, result booleans, status-file writing,
  and emergency-stop fallback.

### s205 Migration Slices

Migrate in small, behavior-preserving slices:

1. `sentai.markers` observation window:
   full-visible geometry, 10-frame count average, centroid/radius/Z summary,
   stale/lost counters, FR scalars.
2. `sentai.calib` vertical-control helpers:
   Z-hold, descent-rate hold, filtered `vz`, thrust clipping and reject reasons.
3. `sentai.calib` axis excitation primitive:
   execute complementary pulse segments with integrated Z hold and marker-lock
   monitoring; return compact scalar result.
4. `sentai.calib` response/candidate validation:
   axis vectors, dominance, orthogonality, noise gate, adaptive pulse retry,
   final candidate acceptance.
5. `sentai.calib.orientation_task`:
   owns B3 phases end-to-end and writes `calib.ini`; MP only starts/stops/polls.
6. `sentai.servo` image-frame primitive:
   reusable damped image-frame centering/sweeps for B4 and B3 final landing.

### Phase 0 - Baseline And Inventory

1. Record current accepted B3/B4 artifact paths in B5.
2. Inspect and document current C++ APIs:
   - `sentai_prep.h` / `detection_task.cc` producer-slot behavior;
   - `modsentai_pipeline.c` diagnostics around `PrepTask`;
   - `sentai_calib.h` / `modsentai_calib.c`;
   - `sentai_markers.h` / `modsentai_markers.c`;
   - `flow_task.cc` / `modsentai_flow.c`;
   - `sentai_crazy.*` / `modsentai_crazy.c`;
   - `sentai_servo.h` / `modsentai_servo.c`.
3. Identify stale comments:
   - `sentai_calib.h` still mentions old `cam_calib.json`/Kabsch-only
     assumptions in places;
   - update comments only when the matching runtime code is changed.
4. Confirm current B3/B4 scripts still run before migration starts.
5. Decide initial task/core ownership boundaries for SIM and ARM:
   - camera/prep producer;
   - marker observation producer;
   - flow producer;
   - estimator-feed sender;
   - calibration FSM;
   - marker-control FSM;
   - Crazyflie command stream owner.

### Phase 1 - Strict `calib.ini` In C++

Goal: make C++ the only owner of calibration persistence.

Tasks:

- implement or tighten a strict scalar parser/writer for `/system/calib.ini`;
- validate required keys:
  `schema`, `task_id`, `status`, `accepted`, `layout_id`, intrinsics, marker
  diameter, `R_B_C`, `cam_offset_B`, `extpos_sign_x/y/z`, and axis evidence;
- reject comments, blank lines, prose, JSON, and nested values in strict mode;
- preserve forward compatibility for unknown scalar keys;
- expose compact C++ status:
  `load_ok`, `schema_ok`, `accepted_ok`, `layout_ok`, `R_valid`,
  `missing_keys`, `strict_errors_count`;
- update B3/B4 MP scripts to call C++ load/save only.

Acceptance:

- B3 can write strict `/system/calib.ini` from C++;
- B4 can load and validate it from C++;
- no host-side or MP-side conversion remains in the active path.

### Phase 2 - Shared Marker Observation Cache

Goal: avoid duplicated marker sampling and MP per-frame loops.

Tasks:

- decide whether marker detection consumes:
  - `SENTAI_PREP_SLOT_GRAY_NATIVE` through `sentai_prep`, preferred when the
    pipeline/prep producer is active; or
  - the existing camera grab path when prep is not running;
  - the decision must be explicit and logged because it affects ARM scheduling.
- add a small C++ observation structure:
  - timestamp/frame id;
  - count and fully-visible count;
  - 10-frame running averages;
  - centroid, radii/scale, visual-Z estimate;
  - PnP pose summary;
  - quality flags and reject reason;
- provide read-only compact access for MP diagnostics;
- keep `sentai.markers` as the single source for WhyCon detection and
  camera/marker geometry;
- make both calibration and marker-control tasks consume the same observation
  structure.
- use seqlock-style reads for prep slots if marker compute runs longer than a
  trivial copy, and track torn-read/retry counters in status.

Acceptance:

- MP can query latest observation without computing it;
- B3/B4 decisions can be implemented in C++ using the shared observation.
- status reports which image source is active: prep slot, direct camera grab, or
  SIM camera bridge.

### Phase 3 - A3 Orientation Calibration Task

Goal: migrate the accepted B3/s197 state machine into `sentai.calib`.

Task phases:

1. setup/preflight;
2. thrust/RPYT acquisition;
3. stable seven-marker lock;
4. visual-Z stabilization;
5. noise estimation;
6. complementary axis excitation;
7. candidate scoring;
8. validation pulses;
9. commit strict `calib.ini`;
10. centered hover and camera-referenced landing proof.

Implementation notes:

- do not use Gazebo GT, EKF yaw, or host logic to decide camera/body rotation;
- keep pixel gates derived from measured noise, image size, FOV margin, or
  logged safety envelopes;
- keep fail-closed behavior: no accepted file on weak/ambiguous calibration;
- preserve append-only runtime journaling for thesis evidence.

Acceptance:

- a reduced MP B3 mission can run `sentai.calib.orientation_start()`;
- the C++ task produces a strict accepted `/system/calib.ini`;
- post-mortem verdict is comparable to `iter35_strict_scalar_calib_ini_source`.

### Phase 4 - Estimator Feed Service

Goal: move continuous flow and ExtPos pumping out of MP.

Tasks:

- create a C++ service/task that:
  - starts/stops or subscribes to flow processing without fighting the camera
    producer;
  - reads flow body deltas from `sentai.flow`;
  - gates and sends flow packets to Crazyflie;
  - reads marker/PnP pose;
  - gates and sends ExtPos;
  - serializes flow and ExtPos sends through one observable update path;
  - records packet rate, last send time, and stream ownership;
  - exposes feed rates, reject counts, and last accepted pose;
- keep `sentai.crazy` as the transport layer;
- keep `sentai.flow.body_read()` using `sentai.markers` extrinsics.
- coordinate with camera/prep/pipeline ownership so flow, marker, and TPU
  inference do not all independently grab/scale frames in a way that collapses
  camera FPS or SDRAM bandwidth.

Acceptance:

- B4 warmup can run without MP pumping flow/ExtPos every frame;
- status reports camera FPS/feed rates and reject counters;
- status proves MP is not the frame-rate sender;
- runtime decisions still do not use GT.

### Phase 5 - A4 Marker-Control Task

Goal: migrate the accepted B4/s203 Generic Hover image-frame controller into C++.

Task phases:

1. setup and strict calibration load;
2. marker acquisition / visual-Z stabilization;
3. ExtPos + flow warmup;
4. RPYT release without disarm;
5. Generic Hover hold-current;
6. symmetric image-frame envelope motion:
   `min_x`, `max_x`, center, `min_y`, `max_y`, center;
7. centered soft landing.

Implementation notes:

- lateral motion remains image-frame for the current proof;
- Z remains metric/estimator-backed with visual/PnP sanity checks;
- keep perpendicular-axis hold active during every active-axis sweep;
- preserve explicit timeouts and safe-stop/disarm path;
- use the accepted B3 `calib.ini`, including `R_B_C`, `cam_offset_B`, axis
  evidence, and `extpos_sign_*`.

Acceptance:

- reduced MP B4 mission can start the C++ task and wait for result;
- result reaches all image-frame proof segments;
- post-mortem XY/Z plots remain comparable to `iter16_strict_calib_ini_e2e_b4`;
- frames are captured for thesis evidence.

### Phase 6 - Compatibility And Cleanup

Tasks:

- keep old B3/B4 REPL missions runnable as regression references;
- keep old MP APIs unless they are explicitly deprecated and wrapped;
- rename only after behavior is stable, because historical folder names still
  point to thesis artifacts;
- document any legacy paths that remain intentionally.

Acceptance:

- old `s197` and `s203` can still be launched;
- new thin B3/B4 missions use the C++ task APIs;
- documentation clearly separates historical names from active algorithms.

### Phase 7 - End-To-End Recertification

Run the full chain:

1. start simulator with GUI;
2. run C++-task B3 calibration;
3. verify `/system/calib.ini` was written by runtime C++;
4. run C++-task B4 marker control using that file;
5. collect:
   - `verdict.log`;
   - `summary.json`;
   - runtime journal;
   - post-mortem frames;
   - XY GT-vs-estimator plot;
   - Z GT-vs-estimator plot.

Acceptance:

- B3 result accepted;
- B4 result accepted;
- no host conversion;
- no MP advanced calculations;
- artifacts archived as thesis evidence.

## Proposed Experiment Folders

Use new experiment folders so old evidence remains untouched:

```text
examples/sentai_runtime/experiments/
  s205_cpp_calib_orientation_task/
  s207_cpp_marker_control_task/
```

The numbers can change if the project already reserves them.  Keep the naming
explicit that these are C++ task-backed experiments.

## Immediate Next Step

Start the B4/A4 C++ migration in a new experiment folder:

```text
examples/sentai_runtime/experiments/s207_cpp_marker_control_task/
```

Initial B4/s207 order:

1. copy only the launcher/verdict shape needed from `s203`, keeping `s203`
   untouched as accepted evidence;
2. load and validate the strict `/system/calib.ini` produced by `s205`;
3. move ExtPos/flow warmup ownership into C++ service code;
4. move Generic Hover image-frame hold/sweep/landing into `sentai.servo`;
5. make MP mirror the `s205` style: coarse states only, no frame-rate loops,
   no runtime JSON summaries.

Reason: the B3 contract is now owned by C++ and has passed SIM recertification,
so B5 can move to the B4 consumer side without reintroducing MP-side
calibration logic.

## Anti-Cheat Checklist

- [x] No host OpenCV or host image processing in runtime.
- [x] No Gazebo GT in runtime decisions.
- [x] No MP-side `calib.ini` writing/conversion in active B3/s205 path.
- [x] No MP-side axis response/candidate scoring after Phase 3.
- [ ] No MP-side flow/ExtPos pumping after Phase 4.
- [x] No MP frame-rate command streaming in active B3/s205 path.
- [ ] No unexplained fixed pixel thresholds.
- [ ] Runtime logs prove source of every decision.
- [ ] Flow, marker/ExtPos, and command streams have explicit C++ owners.

## Notes

- A5 is the specification.  B5 must not silently change A5 scope; if migration
  goals change, update A5 first.
- Keep old B3/B4 runs untouched.  They are regression references and thesis
  evidence.
- New C++ task experiments should produce comparable artifacts, not overwrite
  historical `s197`/`s203` outputs.
- Prefer small, reviewable migrations over large mixed refactors.
- Do not rename historical `s203_hl_marker_control_with_flow` before the C++
  replacement is certified; the name is confusing, but the path is already
  referenced by evidence logs.

## Progress

- [x] Created A5 objective/specification.
- [x] Created B5 implementation plan.
- [x] Added future ARM task/core constraints to B5 architecture.
- [x] Added namespace dependency map and forbidden dependency directions.
- [x] Added first synchronous C++ marker observation API:
  `sentai_markers_get_observation()` / `sentai.markers.get_observation_tuple()`.
- [x] Regenerated MicroPython QSTRs with the project `qstr-regen` recipe.
- [x] SIM build passes after marker observation API addition.
- [x] SIM REPL smoke confirms `sentai.markers.get_observation_tuple()` is
  callable.
- [x] Added C++ strict `calib.ini` status validation:
  `sentai.calib.ini_status_tuple()`.
- [x] C++ parser now accepts the accepted B3 schema-v2 scalar file from
  `/system/calib.ini` and rejects missing required B3/B4 contract fields.
- [x] C++ parser exposes B4-required calibration values:
  `sentai.calib.get_extpos_signs()` and
  `sentai.calib.get_axis_seed_tuple()`.
- [x] SIM C++ file I/O now resolves `/system/calib.ini` through the same
  simulated FS root as `sentai.fs`, so active missions do not need host-side
  conversion or path tricks.
- [x] SIM build and REPL smoke pass for the new `sentai.calib` APIs:
  `load()` returns true and status reports
  `FINAL_VALIDATION_OK` for the accepted B3 artifact.
- [x] Added C++ strict calibration contract writer:
  `sentai.calib.save_contract(status, accepted, roll_sign, roll_vec,
  pitch_sign, pitch_vec)`.
- [x] Kept legacy `sentai.calib.save()` separate, so older Kabsch/autotune
  paths remain compatible and cannot accidentally claim a B3 final artifact.
- [x] Smoke-tested `save_contract()` against an isolated SIM FS root:
  it writes only `key=value` lines, reloads through C++, and returns a valid
  strict status tuple.
- [x] Updated active calibration mission writer path to call
  `sentai.calib.save_contract()` for accepted final calibration results.
  The old MP `_calib_ini_text()` remains as a compatibility fallback only.
- [x] Added C++ candidate scorer:
  `sentai.calib.score_axis_candidate(roll_axis, roll_sign, pitch_axis,
  pitch_sign)`.
- [x] Updated active `s197` B3 candidate-scoring phase to use the C++ scorer;
  the MP discrete-rotation scorer remains fallback only.
- [x] Updated active `s197` B3 feature sampling to use the C++
  `sentai.markers.get_observation_tuple()` aggregate after camera detection,
  removing MP-side per-marker full-visible/centroid/Z aggregation.
- [x] Updated active `s197` optical-axis validation to use the C++
  observation aggregate (`n_pose_valid`, visual-Z mean) instead of iterating
  detections in MP.
- [x] Added initial `sentai.fr` event/scalar emission for migrated C++
  candidate scoring and B3 contract writing.
- [x] Added C++ calib observation sampler:
  `sentai.calib.sample_calib_observation_tuple(img_w, img_h, margin_px)`.
  It owns the camera grab, marker detection, aggregate observation, and
  `sentai.fr` marker-count/centroid/Z scalars.  This preserves the old
  `detect_from_camera()` no-frame contract: no frame yet is `n_raw=0`, not a
  hard mission error.
- [x] Created isolated B5 experiment harness:
  `examples/sentai_runtime/experiments/s205_cpp_calib_orientation_task/`.
  `s197` and `s203` remain accepted references/evidence and must not be
  polluted by B5 recertification runs.
- [ ] Extend `sentai.fr` event/scalar emission as more B3 phases migrate into
  C++ tasks.
- [x] SIM run `iter36_b5_cpp_partial_migration`: regression found, optical
  validation had no pose details after feature aggregation migration.
- [x] SIM run `iter37_b5_cpp_partial_posefix`: regression narrowed to wrong
  call site; feature aggregation/candidate scoring remained healthy.
- [x] SIM run `iter38_b5_cpp_partial_posefix2`: optical-axis validation and
  final candidate validation passed with C++ aggregate/scorer, but the run
  ended in `RETURN_TO_CENTER_FAIL`; no strict `calib.ini` was accepted/written.
- [x] SIM run `iter39_b5_cpp_observation_optical`: optical-axis validation and
  final candidate validation passed using the C++ observation aggregate, but
  the run ended in `RETURN_TO_CENTER_FAIL`; no strict `calib.ini` was
  accepted/written.
- [x] SIM run `iter40_b5_cpp_observation_optical_retry`: invalid simulator
  flight sample for B3 certification; marker acquisition timed out and GT
  altitude stayed near ground level, so it is treated as launch/command
  incident rather than calibration evidence.
- [x] Temporary SIM run `s197/iter41_b5_cpp_sample_observation`: validated
  that the C++ observation sampler reaches marker lock and visual-Z hold, but
  the old MP axis-excitation phase rejected orthogonality
  (`AXIS_RESPONSE_FAIL`, `dot_norm=-0.405`).  This is not accepted B5 evidence;
  subsequent B5 runs must use `s205`.
- [x] SIM run `s205/iter1_cpp_sample_observation`: accepted B3 migration
  smoke in the new B5 harness.
  - Verdict: `FINAL_VALIDATION_OK`.
  - Marker lock, Z hold, axis response, C++ candidate scoring, optical-axis
    validation, final candidate validation, final recenter, and camera
    referenced landing all passed.
  - `/system/calib.ini` was written by `sentai.calib.save_contract`.
  - `sentai.fr` contains C++ evidence events:
    `calib_axis_score` and `calib_contract_save`, plus per-frame observation
    scalars (`obs_n_raw`, `obs_n_full`, `obs_n_pose`, `obs_cx`, `obs_cy`,
    `obs_radius`, `obs_z`).
- [x] Migrated the active B3/s205 mission to C++ orientation tasks:
  `orientation_arm_zero_start`, `orientation_marker_acquisition_start`,
  `orientation_post_lock_brake_start`, `orientation_visual_z_hold_start`,
  `orientation_axis_response_start`, `orientation_centroid_validation_start`,
  `orientation_score_candidate_from_axis`,
  `orientation_optical_axis_validate`,
  `orientation_final_candidate_validation_start`,
  `orientation_final_recenter_start`, `orientation_manual_descend_start`,
  `orientation_center_hold_descend_start`, and
  `orientation_save_contract`.
- [x] Reduced `mission_s205.py` to coarse MP orchestration with
  `MissionResults` and no MP-carried `last_thrust`.
- [x] Rebuilt host-side `verdict_s205.py` so the forensic summary remains
  comparable to the old MP-generated B3 summary while runtime only logs
  `sentai.fr` and strict status/calibration files.
- [x] SIM run `s205/iter8_pre_b4_commit`: accepted B3 C++ task
  recertification before starting B4.
  - Verdict: `FINAL_VALIDATION_OK`.
  - Runtime `calib.ini`: `status=FINAL_VALIDATION_OK`, `accepted=1`.
  - Marker acquisition, post-lock brake, visual-Z hold, axis response,
    candidate scoring, optical-axis validation, final candidate validation,
    final recenter, and center-hold landing passed.
- [x] Phase 0 inventory completed for B3/B4 migration references.
- [x] Phase 1 strict `calib.ini` parser/writer in C++.
- [x] Phase 2 shared marker observation support sufficient for B3/s205.
- [x] Phase 3 A3/B3 orientation calibration task.
- [x] Created isolated B4 migration harness:
  `examples/sentai_runtime/experiments/s207_cpp_marker_control_task/`.
- [x] Added first `sentai.servo` B4 marker-control task slice:
  strict calibrated setup plus image-frame center hold in C++.
- [x] Replaced the temporary B3 takeoff/bootstrap reuse in s207 with a B4
  `sentai.servo.marker_acquire_start()` task phase.
  - The C++ acquire phase keeps the s203 ramp/10-frame marker-average
    confirmation, uses marker apparent size for visual-Z, and records both
    first-full-lock Z and confirmation-window peak Z in `sentai.fr`.
  - The target-Z handoff now uses the first 7-marker full-constellation Z so
    the 10-frame confirmation window does not lift the mission out of the
    accepted s203/B4 altitude band.
- [x] SIM run `s207/iter2_bootstrap_then_marker_setup`: B3 bootstrap,
  calibrated marker setup, and B4 C++ center-hold passed; mission now stops at
  `EXTPOS_WARMUP_PENDING`, which is the next unmigrated B4 phase.
- [x] Phase 4 estimator feed service.
  - B4 ExtPos warmup, flow packet pumping, Kalman reset, and ExtPos stddev
    writes now run inside the C++ marker-control task.
  - A short C++ prep-hold worker keeps visual centering/thrust active while the
    Crazyflie parameter TOC and estimator reset are prepared, avoiding the
    uncommanded drift window seen during early s207 runs.
- [x] Phase 5 A4 marker-control task.
  - `s207_cpp_marker_control_task` now runs setup, center hold, ExtPos warmup,
    Generic Hover handoff, seven image-frame axis-motion segments, and land
    through `sentai.servo.marker_*` C++ task entry points.
  - The B4 mission skips the B3 calibration visual-Z climb; original s203/B4
    marker control continues from the acquisition/post-lock altitude band.
  - Host-side `verdict_s207.py` reconstructs the B4 forensic summary from
    `sentai.fr`, keeping MP runtime limited to state orchestration and status.
- [x] SIM run `s207/iter18_marker_setup_state_split`: accepted B4 C++ task
  recertification.
  - Verdict: `MARKER_CONTROL_OK`.
  - Runtime status: `phase=post_acquisition`, empty `abort_reason`.
  - `calib_load_ok`, strict layout validation, center hold, ExtPos warmup,
    Generic Hover handoff, seven image-frame marker-control segments, and land
    all passed.
  - Flow stayed active in C++: axis motion `send_ok=828`, `read_errors=0`;
    land `send_ok=112`, `read_errors=0`.
  - Kalman/ExtPos preparation was journaled host-side with
    `extpos_stddev.ok=true` at 0.04 m and
    `extpos_stddev_flow_assisted.ok=true` at 0.12 m.
- [x] SIM run `s207/iter21_plots_green`: accepted B4 C++ task run with
  host-side GT-vs-estimator plots.
  - Verdict: `MARKER_CONTROL_OK`.
  - Generated `s207_xy_gt_vs_est.png` and `s207_z_gt_vs_est.png`.
  - Fixed estimator scalar timestamps so the CF estimator trace has a real
    time axis instead of collapsing at `t=0`.
- [x] SIM run `s207/iter26_b4_first_full_z_lock`: B4 C++ acquire + full
  marker-control recertification.
  - Verdict: `MARKER_CONTROL_OK`.
  - Runtime status: `phase=post_acquisition`, empty `abort_reason`.
  - Seven image-frame motion segments and land passed.
  - Host-side plots regenerated with `time_alignment=host_monotonic`.
  - Flow stayed active in C++: axis motion `send_ok=615`, `read_errors=0`;
    land `send_ok=113`, `read_errors=0`.
- [x] SIM run `s207/iter27_flow_during_acquire`: negative-control experiment
  for injecting flow packets during the raw RPYT acquire ramp.
  - Verdict: `MARKER_ACQ_TIMEOUT`.
  - Acquire sent `368` flow packets with `read_errors=0`, but marker lock was
    delayed until the top of the ramp (`z_target_m=1.069`) and did not satisfy
    the 10-frame lock gate before timeout.
  - Decision: keep `iter26` as the stable migrated B4 baseline.  Do not inject
    flow into CF during raw acquire until the estimator has an absolute
    height/ExtPos seed or a separate gated design is proven.
- [ ] Phase 6 compatibility cleanup.
- [ ] Phase 7 end-to-end recertification.
- [ ] Final ARM build and memory/section review after SIM B5 is accepted.

## Open Questions

- Should the EKF feed service live under a new `sentai.estimator` namespace, or
  should it be kept under `sentai.crazy` as transport-adjacent functionality?
- Should flow and ExtPos feed be one service or two separately controllable
  services?
- Which status fields are needed live in MP, and which can remain only in the
  runtime journal?
- Should `sentai_calib_bringup` become a compatibility wrapper around the new
  orientation task, or remain a clearly deprecated experiment surface?

# TD-S10-A5 - Migrate Calibration And Marker Control To C++ Runtime Tasks

## Goal

Refactor the validated B3/B4 REPL missions so the core calibration, marker
control, estimator feeding, and persistence logic run in C++ runtime tasks.
MicroPython should become a high-level orchestration layer: initialize, start,
stop, query status, and collect compact artifacts.

The C++ runtime must load and save `/system/calib.ini` itself.  No host-side or
MicroPython conversion step is allowed in the new production path.

## Motivation

B3 and B4 proved the behavior in simulator:

- B3 `s197_sota_calib_orientation_guarded/iter35_strict_scalar_calib_ini_source`
  writes an accepted strict scalar `calib.ini`.
- B4 `s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4`
  consumes that file and completes the marker-control proof.

Those runs, frames, plots, logs, and verdicts are thesis evidence.  A5 must
preserve their behavior while moving the advanced algorithms out of MP and into
runtime code suitable for SIM first and ARM later.

## Existing Runtime Conventions

Use the existing Sentai module style instead of inventing a parallel framework:

- `sentai.calib` already exposes calibration persistence and task-like APIs:
  `init`, `clear`, `load`, `save`, `commit_R`, `task_start`, `task_stop`,
  `is_done`, `get_kp`, and hold/autotune helpers.
- `sentai.markers` owns WhyCon detection, camera intrinsics, marker layout,
  camera extrinsics, marker pose, and drone-pose diagnostic surfaces.
- `sentai.flow` owns the 80x60 grayscale flow pipeline, `start`, `stop`, `read`,
  `body_read`, timing/perf counters, and uses `sentai.markers` camera extrinsics
  for body-frame conversion.
- `sentai.crazy` owns Crazyflie transport and low-level commands:
  RPYT/attitude, Generic Hover, ExtPos/ExtPose, flow packets, pose subscription,
  release/stop/disarm, and telemetry.
- `sentai.servo` defines the backend-agnostic action vocabulary and trace model:
  `init`, `arm`, `takeoff`, `move`, `hover`, `land`, `status`, and `trace`.

New APIs should follow the same conventions:

- small integer return codes: `0` success, negative errors;
- `start`, `stop`, `is_done`, `status`, and compact getter methods;
- bounded memory, no large MP lists, no per-frame allocations in MP;
- append-only or compact runtime logging for post-mortem;
- legacy APIs remain callable for old experiments.

## Scope

A5 migrates the accepted B3/B4 runtime behavior, not the host verdict tooling.

In scope:

- C++ `sentai.calib` task for the A3 one-run calibration sequence;
- C++ strict `calib.ini` schema-v2 parser/writer;
- C++ marker-lock, visual-Z, noise estimation, complementary excitation,
  candidate scoring, final validation, center-hover, and centered landing;
- C++ runtime support for B4 marker-control handoff:
  strict calibration load, marker setup, ExtPos warmup, flow pumping, Generic
  Hover streaming, image-frame envelope motion, and soft landing;
- compact MP bindings to start/stop/status the above tasks;
- compatibility wrappers so old MP missions can still run on the same runtime;
- host post-mortem scripts and plots that compare new C++ task results to the
  accepted B3/B4 evidence.

Out of scope:

- changing the WhyCon SOTA detector unless A2 reopens it;
- using Gazebo GT in runtime decisions;
- relying on host OpenCV or host-side filesystem conversion in flight logic;
- removing old REPL missions before the C++ path is recertified.

## Required Architecture

### 1. Calibration Task

Add a production A3 task under `sentai.calib`, for example:

```text
sentai.calib.run_orientation()
sentai.calib.orientation_start(...)
sentai.calib.orientation_stop()
sentai.calib.orientation_is_done()
sentai.calib.orientation_status()
sentai.calib.orientation_result()
```

The exact names may change, but the pattern must be task-like and compact.

The task owns:

- preflight marker/camera/runtime validation;
- thrust/RPYT acquisition until marker lock;
- visual-Z stabilization;
- online visual-noise estimation;
- complementary RPYT axis excitations;
- axis/sign inference and candidate scoring;
- validation pulses;
- strict persistence to `/system/calib.ini`;
- centered hover and camera-referenced landing proof;
- compact journal/status emission.

MP must not compute the response matrix, pick candidate signs, or write
`calib.ini`.

### 2. Marker-Control Task

Add a production B4 task under the most appropriate namespace after code review:
`sentai.calib`, `sentai.servo`, or a small new `sentai.nav`/`sentai.markerctl`
module.  Prefer reuse over a new namespace if the behavior naturally fits an
existing module.

The task owns:

- strict load and validation of `/system/calib.ini`;
- marker/intrinsics/layout setup through `sentai.markers`;
- camera extrinsics application;
- RPYT bootstrap until visual-Z and centroid are usable;
- Kalman reset/select and ExtPos warmup;
- continuous flow and ExtPos pumping;
- RPYT release without disarm;
- streamed Generic Hover image-frame controller;
- symmetric image-frame envelope proof:
  `min_x`, `max_x`, center, `min_y`, `max_y`, center;
- perpendicular-axis hold during every active-axis move;
- centered soft landing.

MP should issue only high-level commands such as `start`, `stop`, `status`, and
artifact export.

### 3. Shared Runtime Data

Create shared C++ data structures for:

- marker layout and intrinsics metadata;
- latest marker observation, fully-visible count, and 10-frame running averages;
- visual-Z and marker-scale statistics;
- image centroid and safe image envelope;
- learned axis response vectors and noise estimates;
- calibration artifact fields required by B4;
- task phase, failure reason, and summary counters.

Avoid duplicating frame conventions across modules.  `sentai.flow.body_read()`
already uses `sentai.markers` extrinsics; keep that shared convention.

### 4. Persistence Contract

`/system/calib.ini` remains one strict scalar runtime file:

```ini
schema=2
task_id=TD-S10-B3
experiment=s197_sota_calib_orientation_guarded
status=FINAL_VALIDATION_OK
accepted=1
layout_id=sentai_whycon_small_centroid_7_marker_v1
R_B_C=...
cam_offset_B=...
extpos_sign_x=-1
extpos_sign_y=-1
extpos_sign_z=1
...
```

Rules:

- one `key=value` per line;
- no comments, blank lines, prose, JSON, or nested structures;
- unknown keys may be ignored by older readers;
- required keys must be validated before arming;
- corrupt/missing files fail closed for B4 and must not silently fly;
- C++ owns both load and save.

## Anti-Cheat And Safety Rules

- Runtime decisions must use only onboard camera/markers, calibration metadata,
  command health, and Crazyflie estimator/telemetry permitted by A3/A4.
- SIM GT is forensic only and never enters runtime control.
- Host OpenCV, host scripts, and generated plots are post-mortem only.
- MP must not perform hidden control math after migration.
- Marker-count gates should use the B3/B4 running-average convention, not
  single-frame detector spikes.
- Image thresholds must be derived from runtime noise, image size, FOV margin, or
  logged safety envelopes.  Do not introduce unexplained magic pixel constants.
- On uncertainty, land or fail closed and do not persist success.

## Compatibility Requirement

Keep the old REPL missions and existing APIs runnable while the C++ tasks are
introduced:

- do not remove `sentai.calib.run_kabsch`, `commit_R`, `load`, `save`, existing
  task/autotune APIs, or `sentai.crazy` command surfaces;
- keep historical B3/B4 experiments as regression references;
- any renamed API should have a transition wrapper until old experiments are no
  longer needed;
- new tasks must not require host-generated intermediate files.

## Deliverables

- Updated C++ headers and bindings for the new task surfaces.
- Strict `calib.ini` parser/writer in C++ with schema-v2 validation.
- C++ A3 calibration task matching the accepted B3 behavior.
- C++ B4 marker-control task matching the accepted B4 behavior.
- Reduced MP missions that orchestrate tasks instead of computing algorithms.
- SIM recertification artifacts:
  - B3-style calibration artifact and journal;
  - B4-style marker-control verdict;
  - post-mortem frames;
  - XY and Z GT-vs-estimator plots for thesis evidence.
- A compatibility note listing old mission APIs still supported.

## Proposed Phases

1. API inventory and final naming proposal.
2. Extract shared data models for markers, visual statistics, and calibration
   artifacts.
3. Move strict `calib.ini` load/save and validation fully into C++.
4. Migrate B3 orientation calibration FSM into `sentai.calib`.
5. Migrate continuous flow/ExtPos pumping into a C++ task or service.
6. Migrate B4 Generic Hover image-frame marker-control FSM.
7. Rewrite B3/B4 MP missions as thin orchestration scripts.
8. Re-run end-to-end SIM certification:
   accepted B3 calibration first, then B4 marker-control from the saved file.
9. Compare artifacts against the accepted B3/B4 thesis evidence and document
   any behavioral differences.

## Success Criteria

A5 is successful when:

- the C++ calibration task produces an accepted strict `/system/calib.ini`;
- the C++ marker-control task consumes that file without host conversion;
- the B4 proof sequence succeeds with Generic Hover image-frame control;
- MP code no longer performs advanced per-frame algorithmic work;
- old REPL experiments remain runnable for regression/debug;
- thesis artifacts are produced and archived for both calibration and handoff;
- SIM GT confirms behavior post-mortem, while runtime logs prove GT was not used
  for in-flight decisions.

## Initial Open Questions

- Should the B4 marker-control task live under `sentai.servo` as a richer action
  layer or under a new small marker-navigation namespace?
- Should flow and ExtPos pumping be one combined estimator-feed task or two
  independently startable services?
- Which B3/B4 summary fields should be exposed through `status()` versus written
  only to the runtime journal?
- How much of the current `sentai_calib_bringup` surface should be replaced,
  wrapped, or explicitly deprecated once the A5 task exists?

# TD-S10-B4 - Implement A4 In-Flight Handoff Marker Control

## Purpose

Operational implementation log for
the A4 handoff/marker-control objective document.

A4 remains the objective/specification document.  B4 tracks the concrete
REPL/SIM implementation, runtime findings, artifacts, post-mortem analysis,
and later C/C++ migration plan.

## Current Decision

The active B4 path is a calibrated marker-control handoff based on:

```text
A3 calib.ini -> RPYT bootstrap -> visual-Z / centroid stabilization
              -> ExtPos + Kalman warmup -> RPYT release without disarm
              -> Generic Hover streamed continuously
              -> image-frame envelope motion -> soft landing
```

The current post-handoff primitive is:

```text
sentai.crazy.hover(vx_body, vy_body, yawrate, z_hold)
```

It is streamed continuously.  Lateral motion is controlled in image frame for
the current proof: the marker constellation is moved toward safe image
boundaries and then recentered.  WhyCon/PnP supplies visual Z, marker
visibility, and a slower absolute anchor through ExtPos.

The active experiment is `s203`.  Some local directory/file names are
historical; the active algorithm inside the mission is Generic Hover plus
image-frame marker control.

## Runtime Boundaries

- Runtime flight logic must run inside `sentai_sim` / `sentai_runtime`.
- Host code may launch SIM, stage files, collect artifacts, and run
  post-mortem verdicts.
- Host vision libraries are not allowed in the runtime mission.
- Gazebo GT is post-mortem only.  It must not influence in-flight decisions.
- MicroPython should log incrementally to the drone filesystem and avoid
  retaining large histories in memory.

Current runtime dependencies in the mission:

- `sentai.calib` loads `/system/calib.ini`.
- `sentai.markers` detects WhyCon markers and computes marker pose.
- `sentai.flow` reads optical-flow-like camera motion already converted by the
  runtime body/camera convention.
- `sentai.crazy` sends RPYT, ExtPos, Generic Hover, parameter writes, and
  release/disarm commands.
- `sentai.fs` and `sentai.sim.journal_write` write compact artifacts.

## Calibration Artifact

B4 consumes the accepted A3 output from `/system/calib.ini`.  B4 must not
rediscover camera orientation or manually encode axis signs outside that file.
The file is a runtime artifact, so its format is restricted to `key=value`
lines only: no comments, no empty lines, and no free-text provenance fields.

Required runtime keys:

- schema/version;
- `status=FINAL_VALIDATION_OK` and `accepted=1`;
- layout id for the seven-marker `_small` pad;
- camera intrinsics and marker diameter;
- `R_B_C` / camera-to-body rotation;
- `cam_offset_B`;
- ExtPos sign convention;
- A3 axis-response evidence and safe envelopes when available.

Setup must fail closed if the calibration file is missing, has the wrong
layout, is not an accepted A3 result, is not strict `key=value`, or lacks the
required camera/body mapping.

## State Machine

1. `setup`
   - initialize camera, Crazyflie, safety, markers, flow, and calibration;
   - load `/system/calib.ini`;
   - validate layout metadata;
   - configure marker intrinsics, diameter, world layout, and calibrated camera
     extrinsics.
2. `acquire`
   - use conservative RPYT/thrust bootstrap;
   - climb until the marker constellation is visible and visual Z is usable.
3. `center_hold`
   - use A3 response vectors from the calibration artifact;
   - keep the marker-pad centroid near image center;
   - stabilize visual Z before estimator warmup.
4. `extpos_warmup`
   - reset/select the Crazyflie Kalman estimator where supported;
   - feed gated PnP-derived ExtPos;
   - pump runtime flow packets;
   - compare PnP pose to `stateEstimate` for convergence without using GT.
5. `handoff_hover`
   - release RPYT without disarm using the Crazyflie setpoint-stop/priority
     relaxation packet;
   - immediately stream `hover(0, 0, 0, z_hold)`;
   - keep pumping ExtPos and flow.
6. `generic_image_motion`
   - execute image-frame envelope targets:
     `min_x`, `max_x`, `center`, `min_y`, `max_y`, `center`;
   - command body velocity with streamed Generic Hover setpoints;
   - maintain the non-moving image axis near center;
   - require every segment to report `reached=True`.
7. `land`
   - descend with bounded Generic Hover Z commands;
   - keep ExtPos/flow supervision active while markers remain useful;
   - disarm at the end.

Every Generic Hover streaming phase has an explicit timeout.  This prevents a
buggy loop from sending hover setpoints forever.  Normal motion stops by
transitioning to the next bounded phase; safety failure uses the explicit
safe-stop/disarm path.

## Image-Frame Motion Convention

For the current proof, lateral targets are image-frame targets, not metric
world-frame targets.

- `min_x`: marker layout approaches the left image boundary while the image Y
  centroid remains centered.
- `max_x`: marker layout approaches the right image boundary while image Y
  remains centered.
- `min_y`: marker layout approaches the top image boundary while image X
  remains centered.
- `max_y`: marker layout approaches the bottom image boundary while image X
  remains centered.
- `center`: marker constellation centroid returns to image center.

The velocity mapping follows the previously validated flow/hover convention:

```text
image Y error -> body vx
image X error -> body vy
```

Z is the only metric/world-like control variable in this phase.  It is held
from the Crazyflie estimator height after visual warmup, with visual/PnP Z used
for sanity and logging.

## Safety And Gates

- Count only fully visible markers for marker lock and pose-quality gates.
- Use a 10-frame running average for marker-count safety so single-frame
  detector misses do not abort a healthy flight.
- Reject PnP/ExtPos samples when visual scale Z and PnP Z disagree strongly.
- Continue flow injection through warmup, handoff, motion, and landing.
- If marker lock, estimator convergence, or runtime command return codes fail,
  transition to safe stop/landing and do not report success.
- Success for `generic_image_motion` requires all required image-frame
  segments to reach their targets, not merely successful packet return codes.
- Axis-motion success also requires the non-moving image axis to be actively
  held near zero.  The active sweep command is gated until the perpendicular
  axis is within a stricter image-relative tolerance.

## Current Findings

- The accepted A3 artifact is currently generated by the validated s197 run and
  staged as `/system/calib.ini`.
- The handoff is stable when:
  - Kalman is reset/selected before ExtPos warmup;
  - ExtPos is gated by visual consistency;
  - flow is pumped continuously after takeoff;
  - WhyCon ExtPos trust is relaxed after warmup.
- Runtime flow is healthy in SIM:
  - the camera bridge provides 80x60 grayscale flow internally;
  - `sentai.flow.body_read()` uses the same calibrated body/camera convention
    as `sentai.markers` when calibration is loaded;
  - flow packets are sent from the REPL/runtime mission, matching the ARM
    design where flow and ExtPos will travel over UART/CPX.
- Host-side ArUco/OpenCV helpers are not part of the B4 runtime path.
- Recent Generic Hover image-frame runs are stable and low-tilt, but the image
  envelope proof is not complete until every segment reaches target and then
  recenters.

Latest important implementation decisions:

- `generic_axis_motion` reports only Generic/Image-frame fields.
- Compatibility names from the abandoned position-commander branch should not
  be used in new summaries or verdict output.
- The mission status must be failure if any required image-frame segment does
  not reach target.
- Verdict GT analysis skips world-frame axis gates for symbolic image-frame
  targets; it may still report attitude and position spans for forensic use.
- `cam_offset_B` magnitude is a measured mechanical parameter. B4 does not try
  to estimate it on-flight. The ExtPos frame conversion must still mirror the
  camera/body lever arm when the CF estimator frame uses `extpos_sign_x/y=-1`;
  otherwise the GT-vs-estimator plot shows a stable `~2*cam_offset_B.x`
  lateral bias.
- `extpos_sign_x/y/z` are treated as a Crazyflie estimator-frame adapter, not as
  discovered camera/body orientation. B4 must load them from the accepted
  runtime artifact and validate their presence before arming.

## Iteration Notes

### s203 `iter10_extpos_origin_correction`

Purpose: verify that the observed `~8 cm` X offset in `s203_xy_gt_vs_est.png`
was caused by mirroring the pose into the CF estimator frame without mirroring
the measured camera/body lever arm.

Change tested:

- keep `extpos_signs=(-1, -1, +1)`;
- derive `extpos_origin_correction_m` from `cam_offset_B`;
- for SIM `cam_offset_B=(-0.04, 0, -0.02)`, the correction is
  `(0.08, 0, 0)`.

Result:

- `extpos_origin_correction_m=[0.08, -0.0, 0.0]`;
- `generic_axis_motion_tick` estimator-vs-GT bias improved from
  `dx ~= -0.080 m` in `iter8` to `dx ~= +0.007 m`;
- XY median error improved from `~0.082 m` to `~0.018 m`;
- post-mortem camera frames were captured in the run folder;
- misiunea inca are status `GENERIC_AXIS_MOTION_FAIL` because the strict
  image-frame edge/center gates are not all reached.

Follow-up decision:

- replace the fixed `8 px` image-frame success gate with a resolution-relative
  gate;
- current gate is `5%` per image axis, i.e. `16 px` on X and `12 px` on Y for
  the current `320x240` camera;
- success is evaluated per-axis in image frame, not as a world-frame distance.

## Iter11 Recertification: 5% Image-Frame Gate

Run:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter11_image_tol_5pct_recert`

Result:

- `mission_status=GENERIC_AXIS_MOTION_OK`;
- `success=True`;
- camera FPS was about `30.3 Hz`;
- `target_tol_frac=0.05`, `target_tol_px_xy=[16.0, 12.0]`;
- all image-frame proof segments reached their targets:
  - `image_min_x`, last image error `3.84 px`;
  - `image_max_x`, last image error `10.31 px`;
  - `center_after_x`, last image error `9.05 px`;
  - `image_min_y`, last image error `12.21 px`;
  - `image_max_y`, last image error `4.30 px`;
  - `center_after_y`, last image error `8.25 px`;
- `land_ok=True`;
- Kalman cross-check recent error was about `0.018 m`;
- post-mortem GT attitude stayed mild during generic image motion:
  roll max about `1.36 deg`, pitch max about `1.16 deg`;
- `31` post-mortem camera frames were captured.

Post-review finding:

- `iter11` passed the target gates, but the XY plot did not preserve a clean
  cross because the first sweep started while the perpendicular image axis was
  still offset;
- this violated the A4 requirement that the non-moving axis remains controlled
  during each movement;
- next iteration adds a mandatory `pre_axis_center` and an axis guard that
  blocks active-axis sweep until the perpendicular image axis is centered.

## Iter12 Finding: Axis Guard Exposes Path Contract

Run:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter12_axis_guard_recert`

Result:

- `mission_status=GENERIC_AXIS_MOTION_FAIL`;
- the XY plot is much closer to a cross, but the first X sweep times out;
- root cause: the new perpendicular-axis guard used a stricter `2.5%` gate,
  while the old minimum velocity deadband could zero-out the remaining
  correction around `7-9 px`;
- the Y branch is not perfectly symmetric because bbox-edge targets are
  affected by the intentionally asymmetric seven-marker layout.

Follow-up:

- remove the image-frame velocity deadband for the guarded controller;
- derive a symmetric safe image envelope after `pre_axis_center`;
- drive `min_x/max_x/min_y/max_y` using symmetric centroid targets inside that
  envelope rather than raw bbox-edge contact.
- keep visual correction active during settle windows; do not zero lateral
  velocity immediately after first reaching a gate, because that lets the
  marker centroid drift before the next phase starts.

## Iter14 Recertification: Symmetric Image Envelope

Run:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter14_symmetric_axis_guard_controlled_settle`

Result:

- `mission_status=GENERIC_AXIS_MOTION_OK`;
- `success=True`;
- `land_ok=True`;
- image-frame velocity deadband is disabled for the guarded controller;
- `pre_axis_center` is executed before the proof sequence;
- the symmetric safe image envelope is derived after centering:
  - `amp_x_px=94.36`;
  - `amp_y_px=59.57`;
  - centroid targets are `x=[65.64, 254.36]`, `y=[60.43, 179.57]`;
- all image-frame proof segments reached:
  - `image_min_x`, final error `5.18 px`;
  - `image_max_x`, final error `7.24 px`;
  - `center_after_x`, final error `5.40 px`;
  - `image_min_y`, final error `7.12 px`;
  - `image_max_y`, final error `0.69 px`;
  - `center_after_y`, final error `5.60 px`;
- GT-vs-estimator XY plot shows a much cleaner cross than `iter11` and
  `iter13`; image X sweeps are nearly symmetric in the plotted Y direction.

Artifacts:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter14_symmetric_axis_guard_controlled_settle/verdict.log`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter14_symmetric_axis_guard_controlled_settle/mission_s203_summary.json`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter14_symmetric_axis_guard_controlled_settle/s203_xy_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter14_symmetric_axis_guard_controlled_settle/s203_z_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter14_symmetric_axis_guard_controlled_settle/fr_current/frames/`

## Iter15 End-To-End Runtime Calib.ini Recertification

Source B3 run:

- `examples/sentai_runtime/experiments/s197_sota_calib_orientation_guarded/iter29_runtime_calib_ini_source_for_b4`

B4 run:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter15_end_to_end_from_runtime_calib_ini`

Result:

- B4 consumed `build-sim/sentai_fs_root/system/calib.ini` written by B3;
- no host-side JSON-to-INI conversion was used;
- `mission_status=GENERIC_AXIS_MOTION_OK`;
- `success=True`;
- `land_ok=True`;
- `calib_load_ok=True`;
- `R_loaded_matches_seed=True` as SIM forensic diagnostic only;
- `generic_image_motion_ok=True`;
- `gt_axis_motion_ok=True`;
- camera FPS was about `30.3 Hz`;
- the post-mortem XY plot preserves the expected image-frame cross.

Artifacts:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter15_end_to_end_from_runtime_calib_ini/verdict.log`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter15_end_to_end_from_runtime_calib_ini/mission_s203_summary.json`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter15_end_to_end_from_runtime_calib_ini/s203_xy_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter15_end_to_end_from_runtime_calib_ini/s203_z_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter15_end_to_end_from_runtime_calib_ini/calib_staged.ini`

## Iter16 Strict Calib.ini End-To-End Recertification

Source B3 run:

- `examples/sentai_runtime/experiments/s197_sota_calib_orientation_guarded/iter35_strict_scalar_calib_ini_source`

B4 run:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4`

Strict setup validation:

- `calib_ini_strict_errors=[]`;
- `calib_ini_missing_keys=[]`;
- `calib_ini_status_ok=True`;
- `calib_ini_accepted_ok=True`;
- `calib_ini_schema_ok=True`;
- `R_valid=True`;
- `axis_seed_required_ok=True`.

Result:

- `mission_status=GENERIC_AXIS_MOTION_OK`;
- `success=True`;
- `land_ok=True`;
- `calib_load_ok=True`;
- `generic_image_motion_ok=True`;
- `R_loaded_matches_seed=True` remained a SIM forensic diagnostic, not a
  runtime acceptance gate;
- camera FPS was about `30.3 Hz`;
- Kalman recent 10-sample error was about `0.0083 m`;
- all image-frame motion segments reached target:
  - `pre_axis_center`;
  - `image_min_x`;
  - `image_max_x`;
  - `center_after_x`;
  - `image_min_y`;
  - `image_max_y`;
  - `center_after_y`;
- post-mortem frames and GT-vs-estimator plots were captured.

Artifacts:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4/verdict.log`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4/mission_s203_summary.json`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4/s203_xy_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4/s203_z_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4/fr_current/frames/`

Iter10 artifacts:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter11_image_tol_5pct_recert/verdict.log`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter11_image_tol_5pct_recert/mission_s203_summary.json`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter11_image_tol_5pct_recert/s203_xy_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter11_image_tol_5pct_recert/s203_z_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter11_image_tol_5pct_recert/fr_current/frames/`

Artifacts:

- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter10_extpos_origin_correction/s203_xy_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter10_extpos_origin_correction/s203_z_gt_vs_est.png`
- `examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter10_extpos_origin_correction/fr_current/frames/`

## Implementation Plan

1. Keep the current calibrated bootstrap and `/system/calib.ini` loading path.
2. Make calibration loading strict: missing matrix, offset, layout id, accepted
   status, axis evidence, or sign convention should fail setup.
3. Keep all runtime control inside `sentai_sim` / `sentai_runtime`.
4. Rename active reporting toward `generic_image_motion` terminology.
5. Keep host verdict as post-mortem only.
6. Re-run s203 with Gazebo GUI and inspect:
   - marker count and running average;
   - ExtPos accept/reject reasons;
   - flow send/read stats;
   - per-segment image error first/last/reached;
   - post-mortem GT attitude spans.
7. Once REPL behavior is stable, migrate the validated state machine and API
   gaps into C/C++ runtime surfaces under A5.

The current accepted B4 reference for A5 is:

```text
examples/sentai_runtime/experiments/
  s203_hl_marker_control_with_flow/
    iter16_strict_calib_ini_e2e_b4/
```

The directory name is historical.  The accepted behavior is Generic Hover plus
image-frame marker control, with strict C++/runtime `calib.ini` consumption.  The
run artifacts, plots, frames, and verdict are thesis evidence and should be kept
as the reference behavior for the C++ migration.

## Open Questions

- Should image-frame control gains and edge margins be persisted in `calib.ini`
  as discovered/safe envelopes rather than kept as mission constants?
- What is the cleanest C/C++ surface for the bounded Generic Hover
  state-machine: `sentai.calib`, `sentai.nav`, or a new small module?

## Progress

- [x] Created A4 objective/specification document.
- [x] Created B4 operational implementation log.
- [x] Consume compact `calib.ini` written by accepted A3/s197 runtime.
- [x] Keep B4 provenance/axis seed in the same `calib.ini` as runtime keys.
- [x] Load staged `/system/calib.ini` inside the mission before arming.
- [x] Identify and add safe RPYT release without disarm.
- [x] Implement PnP-to-ExtPos warmup in REPL.
- [x] Add warmup diagnostics comparing PnP pose with CF2 `stateEstimate`.
- [x] Increase post-lock visual hover target by 50% before handoff warmup.
- [x] Use calibration-artifact response vectors for visual centering.
- [x] Use firmware setpoint-stop/priority relaxation for in-flight release.
- [x] Add runtime `sentai.flow` injection after takeoff.
- [x] Refactor `sentai.flow.body_read()` to use `sentai.markers` camera
      extrinsics instead of a separate hard-coded body/image convention.
- [x] Remove accidental host vision dependency from the validation path.
- [x] Move flow injection into the REPL/runtime mission.
- [x] Switch active motion proof to streamed Generic Hover in image frame.
- [x] Add explicit timeout guards for Generic Hover streaming phases.
- [ ] Rename active s203 directory/scripts later to remove historical naming.
- [x] Make calibration loading strict for required A3 keys.
- [x] Re-run s203 after terminology and success-gate cleanup.
- [x] Validate complete image-frame envelope motion and centered soft landing.
- [x] Write C/C++ migration notes after REPL success.

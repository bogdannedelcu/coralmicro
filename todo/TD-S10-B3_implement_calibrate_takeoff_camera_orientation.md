# TD-S10-B3 - Implement A3 Takeoff Camera Orientation Calibration

## Purpose

Operational implementation log for
`TD-S10-A3_calibrate_takeoff_camera_orientation.md`.

A3 remains the objective/specification document.  B3 tracks concrete runtime
findings, implementation decisions, commands, progress, failed attempts, and
blockers.

## Current Decision

Do not continue from the old `sentai_calib_bringup` state machine as the
primary calibration flow.  Build a staged A3/B3 mission that first acquires
markers, stabilizes visual Z, then discovers lateral axes with small
complementary RPYT pulses.

Existing experiments such as `s194` are work in progress and not evidence of
success.  Treat them as negative evidence and use their logs to avoid repeating
unsafe assumptions.

Create a fresh experiment:

```text
examples/sentai_runtime/experiments/
  s195_sota_uncalibrated_visual_servo/
```

Use `s194` only as an example for experiment organization, simulator launch,
journaling, artifact collection, and post-mortem practice.  Do not inherit its
control algorithm.

## Known Inputs

- Target world:
  `_small` WhyCon Gazebo world, with exact layout asserted at runtime.
- Camera:
  `320x240` grayscale, `fx=288.3`, `fy=288.3`, `cx=160.0`, `cy=120.0`.
- Marker diameter:
  `0.0544 m`.
- Current expected marker layout, centered so the mean XY marker centroid is
  `(0, 0)`:
  - `NW = (-0.082857143, +0.065714286, 0.005)`
  - `NE = (+0.077142857, +0.065714286, 0.005)`
  - `W  = (-0.062857143, -0.014285714, 0.005)`
  - `E  = (+0.057142857, -0.014285714, 0.005)`
  - `SW = (-0.082857143, -0.094285714, 0.005)`
  - `SE = (+0.077142857, -0.094285714, 0.005)`
  - `N  = (+0.017142857, +0.085714286, 0.005)`.
- Spawn convention:
  the Crazyflie downward camera is mounted at `(-0.04, 0, -0.02)` relative to
  `base_link`; for upright spawn, place the body at `(0.04, 0, 0)` so the
  camera optical center, and therefore the image center ray, starts above the
  marker-pad origin `(0, 0, 0)`.
- Control reference:
  all calibration, center-hover, and landing decisions are made in camera
  coordinates.  The controller drives the marker-pad centroid to image center;
  body-frame GT is used only post-mortem.
- Visibility convention:
  count only markers whose full outer circle is inside the image.
- s195 acquisition convention:
  first climb continues until all 7 markers are fully visible for 10
  consecutive frames.  Four markers remain useful as a lower diagnostic floor,
  but do not satisfy the calibration-band lock.

## Runtime Code Review

### Reuse

Use these pieces directly:

- `sentai.markers.init("whycon")`
- `sentai.markers.set_intrinsics(...)`
- `sentai.markers.set_marker_size(...)`
- `sentai.markers.set_marker_world(...)`
- `sentai.markers.detect_from_camera()`
- `sentai.markers.get_count()`
- `sentai.markers.get_detection_tuple(i)`
- `sentai.markers.get_pose_tuple(i)` for diagnostics
- `sentai.markers.get_binary(...)` / `dump_binary_pgm(...)` for debug
- `sentai.calib.commit_R(...)`
- `sentai.calib.save()` / `load()`
- `sentai.calib.get_R_cam_to_body()`
- `sentai.calib.rotation_angle_deg(...)`

The current WhyCon detector provides the fields needed for A3:

- marker centroid;
- fitted axes and angle;
- `radius_outer`;
- `pose_valid`;
- per-marker `tvec_cam`;
- detection timing and binary debug surface.

This is enough for marker lock, FOV margin, aggregate marker scale, visual Z,
and local motion response.

### Avoid In The Primary A3 Decision Path

- Do not use `sentai_calib_bringup` as the A3 state machine.
- Do not use `sentai_calib_task` for axis discovery because it consumes cached
  `R_cam_to_body`.
- Do not use `sentai.markers.get_drone_pose_tuple(yaw)` as the primary source
  for orientation discovery.  It uses yaw anchoring and mirror disambiguation,
  so it is a diagnostic/validation path until A3 has accepted a mapping.
- Do not call `sentai.markers.set_cam_extrinsics(...)` before the orientation
  is accepted.  Keep raw camera-optical detections during discovery.
- Do not use post-bootstrap lateral hover/position commands while the
  camera/body mapping is unknown.

## s194 Review

`examples/sentai_runtime/experiments/s194_calib_axis_id_pd_closed_loop/` is a
work-in-progress experiment with failed attempts.  It is useful mainly as a
post-mortem log.

Observed failures:

- `iter1`: axis ID started after a false Z-stable condition.  PnP Z climbed
  from about `0.24 m` to `0.51 m`, then pitch exploration lost FOV.
- `iter2`: smaller lateral pulses did not solve Z drift.  Z gate timed out
  after drift toward about `0.54 m`.
- `iter3`: Z PID improved behavior but still timed out.  EKF Z diverged while
  PnP still saw markers, so EKF cannot be the primary A3 signal.
- `iter4`: Z gate passed once, but pitch `PD-park` timed out after lateral
  drift.  This reached the real lateral-control failure.
- `iter5`: naive XY hold before knowing the camera/body mapping made things
  worse, saturated small commands, and accumulated about `9 cm` X drift.

Conclusions for B3:

- Axis ID must not start until marker count, full visibility, scale/Z, and
  filtered visual-Z derivative are stable.
- XY hold before axis discovery is unsafe if it assumes identity mapping.
- On axis-id failure, stop and land.  Do not continue into handoff or
  `run_bringup`.
- The s194 verdict harness is invalid for s194 because it still looks for
  `mission_s193_summary.json`; B3 needs its own verdict.
- Current `mission_s194.py` appears partially edited for an iter6 direction:
  comments say XY hold was removed, but `pid_tick` still references
  `pitch_cmd_xy` / `roll_cmd_xy`.  Fix before reusing or rerunning that file.

## Implementation Plan

1. Create/maintain this B3 implementation log.
2. Confirm the active `_small` world marker layout and record whether it has
   six or seven markers.
3. Build `s195_sota_uncalibrated_visual_servo` as the clean A3 mission.
4. Add a B3 mission artifact folder under `dataset/TD-S10-B3/`.
5. Implement preflight:
   - initialize camera, markers, calib, crazy, safety;
   - configure intrinsics, marker diameter, marker world coordinates;
   - verify camera frames are changing;
   - verify marker detection on stationary frames when possible.
6. Implement thrust-only marker acquisition:
   - zero-unlock;
   - bounded thrust ramp;
   - neutral roll/pitch/yaw;
   - stop on stable fully-visible marker lock.
7. Implement marker-lock and visual-Z stabilization:
   - use marker count, full-visibility margin, aggregate scale, and filtered
     scale/Z derivative;
   - do not use lateral hold yet.
8. Implement complementary axis probes:
   - very small RPYT pulses;
   - positive and negative pulse pairs;
   - image-space response first: centroid delta and scale delta;
   - log full per-tick diagnostics.
9. Score candidate rotations:
   - determinant `+1`;
   - residual and candidate margin;
   - consistency across pulse pairs.
10. Validate selected candidate with smaller correction motions.
11. Commit and persist only accepted `R_cam_to_body`.
12. Add a B3 verdict script that reads B3 `summary.json` and journal outputs.
13. Run repeated SIM attempts with Gazebo GUI visible.
14. After every run, inspect host-side GT artifacts for post-mortem only:
    - `fr_current/gt.jsonl`
    - `gt_recorder.log`
    - B3/s195 verdict output
    - altitude span, final altitude, and XY drift from takeoff origin.
    GT must never be fed back into `sentai_sim`, cf2, `sentai.markers`, or
    the mission decision path.

## s195 Design Guardrails

s195 must remain aligned with the A3 SOTA interpretation:

- Decisions are based on image-space WhyCon features first: fully visible
  count, centroid, radius/scale, visual-Z proxy, and their derivatives.
- Do not call `sentai.markers.get_drone_pose_tuple(yaw)` in the calibration
  decision path because it imports yaw anchoring.
- Do not call `sentai.calib.run_bringup()` as the primary state machine.
- Do not use EKF/yaw/Gazebo GT to decide the camera/body rotation.
- Do not enable lateral hold before measuring the local image response.
- Do not commit/persist `R_cam_to_body` until candidate scoring and validation
  motions pass.
- On ambiguity, timeout, weak marker lock, or weak candidate margin, land and
  keep the previous calibration.

Initial s195 scope:

- `README.md` documents method and next steps.
- `run.sh` launches the same SIM stack and collects `mission_s195_*` artifacts.
- `mission_s195.py` currently performs setup and stationary WhyCon feature
  sampling, thrust-only marker acquisition, post-lock brake, and visual-Z hold.
  Later calibration phases are explicit placeholders until visual-Z behavior
  is verified.
- Current acquisition lock requirement: 7 fully visible markers for 10
  consecutive frames.
- Current post-lock behavior: after `7x10`, run a short visual-Z brake/settle
  phase before Z-hold so the controller does not inherit upward velocity from
  the takeoff ramp.
- `verdict_s195.py` reads `mission_s195_summary.json` and host-side
  `fr_current/gt.jsonl` after the run, reporting marker lock, GT altitude
  span, final altitude, X/Y drift/span, yaw delta/span, and algorithm-vs-GT
  visual-Z error when the mission produced a valid summary.  This is forensic
  only.

MicroPython data-volume rule:

- keep MP-side summaries small;
- do not store full per-frame/per-tick series in Python lists;
- write only decimated journal ticks and compact aggregates from MP;
- do large comparisons, statistics, GT alignment, and report tables on the
  host side in verdict/report scripts;
- increase the SIM MP heap only if compact data still blocks a required
  mission step.  Current SIM heap is defined in `sim/main_sim.c` as
  `MP_HEAP_SIZE`.

## Progress

- [x] Read A1/B1 and A2/B2 to align document structure.
- [x] Rewrote A3 as objective/specification only.
- [x] Created B3 as the operational implementation/review log.
- [x] Reviewed existing `sentai.markers` and `sentai.calib` surfaces.
- [x] Reviewed s194 mission logs as failed-attempt evidence.
- [x] Created fresh s195 experiment folder.
- [x] Added s195 launcher, README, and preflight feature scaffold.
- [x] Added thrust-only marker-acquisition smoke.
- [x] Added s195 post-mortem GT verdict script.
- [x] Confirm active `_small` marker layout at runtime.
- [x] Create B3 mission/artifact runner.
- [x] Implement preflight and marker-lock smoke.
- [x] Implement visual-Z stabilization smoke.
- [x] Implement first complementary pulse smoke.
- [x] Implement IBVS centroid validation smoke after axis/sign discovery.
- [x] Implement candidate scoring smoke without persistence.
- [x] Implement final validation motions for selected `R_cam_to_body`.
- [x] Enable guarded `commit_R/save` after final validation.
- [x] Persist accepted calibration to runtime `/system/calib.ini`.
- [x] Add B3 verdict/report artifacts.

## Accepted Calibration Artifact For Downstream Tasks

The current accepted A3/B3 seed for A4/B4 is:

```text
examples/sentai_runtime/experiments/
  s197_sota_calib_orientation_guarded/
    iter27_camera_landing_contract/
      mission_s197_calibration.json
```

Reason:

- `accepted: true`
- `status: FINAL_VALIDATION_OK`
- `layout_id: sentai_whycon_small_centroid_7_marker_v1`
- `R_cam_to_body = [0, -1, 0, -1, 0, 0, 0, 0, -1]`
- `camera_referenced_landing_ok: true`
- landing contract:
  `center_hold_active_until_avg_full_markers_reaches_disarm_threshold`

For B4/s203, accepted B3 runs must write the runtime `calib.ini` directly from
inside `sentai_runtime`/REPL.  B4 consumes that file through
`sentai.calib.load()`.  The host must not convert JSON into `calib.ini` as part
of the validation path.  The file is strict `key=value`, with no comments, no
blank lines, and no prose fields:

```ini
schema=2
status=FINAL_VALIDATION_OK
accepted=1
R_B_C=...
cam_offset_B=-0.04,0.0,-0.02
axis_roll_vec_px=...
axis_pitch_vec_px=...
axis_roll_noise_sigma_px=...
axis_pitch_noise_sigma_px=...
```

The s199 envelope-survey experiment is useful research context, but it has not
yet produced a better accepted artifact than s197/iter27.  Do not seed A4 from
stale s199 `calib.ini` files.

## C/C++ Migration TODO

After the REPL mission remains stable, migrate the s197 calibration behavior
into C/C++ under `sentai.calib`.  This is now tracked as A5.

- A3-style takeoff-until-marker-lock state machine;
- marker-count running average and full-visibility gates;
- visual-Z hold and center-referenced landing contract;
- independent axis/sign discovery using noise-aware complementary excitations;
- accepted `R_cam_to_body` persistence into `/system/calib.ini`;
- compact journaling/artifact writing that does not retain large arrays in
  MicroPython memory.

The C++ implementation must preserve the same objective contract as A3: all
constants used for decisions are either physical metadata, loaded calibration
metadata, or discovered/statistically estimated during flight.

The current accepted reference for that migration is the strict scalar B3 source
run:

```text
examples/sentai_runtime/experiments/
  s197_sota_calib_orientation_guarded/
    iter35_strict_scalar_calib_ini_source/
```

This run and its artifacts are thesis evidence.  A5 must preserve behavioral
parity with it while reducing MicroPython to orchestration only.

## s195 Run Notes

### iter2 - thrust-only acquisition, 4-marker lock

- Status: `MARKER_ACQ_OK`.
- Lock: 4 fully visible markers at `t=8.382 s`, `z_cam~=0.304 m`.
- GT post-mortem:
  - `z_max~=0.424 m`;
  - final XY drift `~0.032 m`;
  - yaw span `<0.05 deg`.
- Conclusion: thrust-only acquisition works, but 4 markers is not enough for
  the calibration band.

### iter3 - first visual-Z hold, 4-marker lock

- Status: `VISUAL_Z_HOLD_FAIL`.
- Failure: `marker_loss`.
- Visual-Z hold had `65` valid ticks but dropped to 3 fully visible markers
  while descending toward the too-low `0.30 m` target.
- GT post-mortem:
  - `z_max~=0.429 m`;
  - final XY drift `~0.044 m`;
  - yaw span `<0.05 deg`.
- Conclusion: target `0.30 m` is too low for stable calibration; it reduces
  full-visible marker count.

### iter4 - acquisition requires 7 markers x 10 frames

- Status: `VISUAL_Z_HOLD_OK`.
- Lock: 7 fully visible markers for 10 consecutive frames at `t=8.844 s`,
  `z_cam~=0.448 m`.
- Visual-Z hold:
  - all 121 requested ticks valid with 7 full markers;
  - `z_cam_max~=0.584 m`;
  - `z_cam_last~=0.455 m`;
  - thrust range `30895..34000`.
- GT post-mortem:
  - `z_max~=0.617 m`;
  - final XY drift `~0.114 m`;
  - yaw span `<0.05 deg`.
- Conclusion: the 7-marker calibration band is reachable and stable, but the
  drone enters Z-hold with upward velocity and overshoots.  Do not start
  lateral axis pulses yet.  Next step is to reduce vertical overshoot and XY
  drift using an immediate post-lock thrust brake / gentler acquisition ramp,
  still without using EKF/yaw/GT in the mission decision path.

### iter5 - post-lock brake only after 7x10

- Status: `VISUAL_Z_HOLD_FAIL`.
- Lock: 7 fully visible markers for 10 consecutive frames at `t=9.108 s`,
  but already high: `z_cam~=0.541 m`.
- Brake started too late and Z continued to overshoot:
  - `z_cam_max~=0.644 m`;
  - GT `z_max~=0.693 m`;
  - final XY drift `~0.071 m`.
- Conclusion: waiting until all 10 confirmation frames complete before
  braking lets the drone climb too far.  Brake must start on the first
  7-marker frame, while the 10-frame lock confirmation is still accumulating.

### iter6 - brake during 7x10 confirmation

- Status: `VISUAL_Z_HOLD_OK`.
- Lock: 7 fully visible markers for 10 consecutive frames at `t=9.009 s`.
- During the 10-frame confirmation, thrust is reduced to `30500`, so Z-hold
  starts with less vertical inertia.
- Visual-Z hold:
  - all 121 requested ticks valid with 7 full markers;
  - `z_cam_max~=0.492 m`;
  - `z_cam_last~=0.456 m`;
  - thrust range `32000..33075`.
- GT post-mortem:
  - `z_max~=0.535 m`;
  - final XY drift `~0.072 m`;
  - yaw span `<0.05 deg`.
- Conclusion: early brake is the correct direction.  The remaining problem is
  not marker visibility; it is Z-control quality and passive XY drift during
  the long vertical maneuver.

## Z-Control Decision

s195 should use a simple auditable dynamic controller before considering
advanced control:

- Use a discrete visual-Z PD/PID over `z_cam_mean_m`, with filtered `vz`
  estimated from consecutive WhyCon frames.
- Add anti-windup if integral action is introduced.
- Add thrust slew-rate limiting so command changes do not inject extra
  lateral acceleration through tiny attitude biases.
- Prefer a calibration band over an exact scalar target, for example keeping
  7 markers fully visible and `z_cam` in a stable range.
- Keep MPC/LQR out of the first flight implementation.  They require a more
  reliable state estimate, sample timing, and identified dynamics.  They can
  be revisited after the visual-Z loop and marker-band behavior are stable.

## IBVS Centroid Validation Decision

After the pitch/roll axis identity and signs are accepted, B3 should validate
the mapping with a small image-space centroid controller before any
`commit_R/save`:

- The controller is not used before axis/sign discovery.
- The measured pitch/roll complementary response vectors form a local 2x2
  image-response matrix.
- A conservative PD loop commands bounded roll/pitch corrections to reduce
  marker-constellation centroid error relative to image center.
- Visual-Z thrust control remains active during this validation.
- The validation accepts only if marker lock is preserved and centroid error
  decreases or reaches a small tolerance.
- If centroid error grows beyond the safety margin, marker count drops, or the
  response matrix is degenerate, land without persisting calibration.

This is the SOTA-consistent bridge between pulse-based uncalibrated Jacobian
estimation and a later camera/body rotation candidate: first estimate the
local response, then prove it can close a small image-space loop.

Current smoke limits:

- duration: `2.0 s`;
- max roll/pitch command: `0.45 deg`;
- proportional gain: `0.08`;
- derivative gain: `0.0`;
- centroid deadband: `6 px`;
- require either final error under `12 px` or at least `3 px` improvement;
- abort if error worsens by more than `8 px`.
- use the inverse sign of the impulse Jacobian for sustained centroid hold.
  Iter20 showed that the short attitude impulse response identifies axes and
  signs, but the two-second lateral position response has opposite sign.

## Axis Identification Principle

Axis identification must be local and independent per axis:

- Each axis gets its own local image baseline.
- Drift accumulated before an axis probe is not interpreted as that axis'
  response.
- The measured signal is the local response to complementary `+axis/-axis`
  perturbations.
- A complementary pair identifies both:
  - the dominant image/camera direction affected by that body command
    (`image-X` vs `image-Y`, including 90-degree camera/body rotations);
  - the sign/sense of that response.
- For example, if `+pitch/-pitch` produces a response vector dominated by
  image-Y, body pitch maps primarily to the camera/image Y direction.  If it
  is dominated by image-X, the camera/body mounting is rotated by 90 degrees
  relative to that body axis.  The sign of the vector records the axis sense.
- The second body axis must be measured from a fresh local baseline, then the
  two response vectors must be checked for near-orthogonality before any
  camera/body rotation candidate can be accepted.
- Between axes, the routine may run a small image-space recover/recenter step
  using the just-measured local response, but this is a runtime safety/action
  step, not a manually supplied calibration constant.
- This matches uncalibrated visual servoing practice: estimate the local image
  Jacobian/visual-motor response online from small perturbations, then use the
  current local estimate for control or candidate scoring.  Relevant SOTA:
  Piepmeier & Lipkin 2003 uncalibrated eye-in-hand visual servoing, and later
  robust/local image-Jacobian estimation work.

### iter11 - active recenter attempt

- Status: `AXIS_RESPONSE_FAIL`.
- Change:
  - after each axis pair, try up to two image-space recenter pulses using the
    just-measured complementary response.
- Result:
  - recenter overcorrected both axes;
  - pitch return error grew from `~13.4 px` to `~27.0 px`;
  - roll return error grew to `~22.5 px`;
  - roll minimum full marker count dropped to `5`;
  - GT XY drift grew to `~0.251 m`.
- Conclusion:
  - Active recenter based on a single noisy local response is unsafe at this
    stage.
  - Disable active recenter in s195 for now.  Keep the local-baseline and
  return-gate logic, and improve drift by shortening/reshaping pulses or
  using repeated low-energy samples rather than corrective recenter pulses.

### iter12 - centroid-only, no active recenter

- Status: `AXIS_RESPONSE_FAIL`.
- Change:
  - active recenter disabled (`AXIS_RECENTER_MAX_PASSES=0`);
  - keep local baseline, strict return gate, centroid-only X/Y signal.
- Conditions:
  - 7 fully visible markers preserved throughout pulses;
  - visual-Z hold target `0.55 m`;
  - GT `z_max~=0.599 m`;
  - GT XY radius max `~0.152 m`.
- Complementary image responses:
  - pitch `comp_px ~= (0.16, -4.91)`;
  - roll `comp_px ~= (-4.73, -0.38)`.
- Return gate:
  - pitch return error `~11.52 px`, rejected;
  - roll return error `~13.75 px`, rejected.
- Conclusion:
  - Centroid-only X/Y is sufficient to observe the axis directions online.
  - The problem is not lack of X/Y signal.  The problem is accumulated lateral
    drift across a full two-axis probe sequence without a reliable recovery
    step.
  - Next safer step: run one-axis-at-a-time probe/validation inside the same
    one-run framework, accept the measured response for an axis only if its
    local return gate passes, then decide whether to continue to the second
    axis or fail closed.

Operator decision after iter6:

- Prefer a slightly higher calibration band than the minimal 7-marker band.
- Rationale: later X/Y excitation can move the marker constellation toward the
  image boundary, so markers should start smaller in the frame and with more
  FOV margin.
- s195 target changed from `z_cam=0.42 m` to `z_cam=0.55 m` for the next
  smoke, while keeping the 7 fully visible marker requirement.

### iter7 - higher calibration band

- Status: `VISUAL_Z_HOLD_OK`.
- Lock: 7 fully visible markers for 10 consecutive frames at `t=8.844 s`.
- Post-lock brake:
  - `z_cam_last~=0.459 m`;
  - `vz_filt_last~=-0.015 m/s`.
- Visual-Z hold:
  - target `z_cam=0.55 m`;
  - all 121 requested ticks valid with 7 full markers;
  - `z_cam_max~=0.559 m`;
  - `z_cam_last~=0.559 m`;
  - thrust range `32000..33742`.
- GT post-mortem:
  - `z_max~=0.592 m`;
  - final XY drift `~0.051 m`;
  - yaw span `<0.03 deg`.
- Conclusion: the higher band is better for the next A3 stage.  Markers stay
  small enough for FOV margin, Z converges smoothly, and passive XY drift is
  lower than iter6.  Use this band for the first complementary X/Y pulse smoke.

### iter8 - first complementary pitch/roll pulse smoke

- Status: `AXIS_RESPONSE_OK`.
- Conditions:
  - 7 fully visible markers preserved throughout pulses;
  - `z_cam_last~=0.559 m`;
  - GT `z_max~=0.600 m`;
  - GT XY radius max `~0.049 m`;
  - yaw span `<0.1 deg`.
- Image response observed:
  - pitch `+1 deg` moved centroid mostly in image Y;
  - roll `+1 deg` produced measurable image X response;
  - responses are not yet clean enough for candidate scoring because the
    first implementation used one phase baseline for both axes, so roll was
    contaminated by residual drift after pitch.
- Implementation follow-up:
  - measure each axis from its own local baseline;
  - report segment deltas `first->last`;
  - compute complementary delta as `(positive_segment - negative_segment)/2`
    to reduce drift/delay bias.
  - require the post-pair settle centroid to return near the local baseline
    before accepting the axis smoke.

### iter9 - local baseline and complementary deltas

- Status: `AXIS_RESPONSE_OK`.
- Conditions:
  - 7 fully visible markers preserved throughout pulses;
  - visual-Z hold target `0.55 m`;
  - `z_cam_last~=0.559 m`;
  - GT `z_max~=0.600 m`;
  - yaw span `<0.03 deg`.
- Complementary image responses:
  - pitch `comp_px ~= (-0.63, -8.15)`;
  - roll `comp_px ~= (-7.73, -0.13)`.
- Interpretation:
  - pitch response is dominated by image Y;
  - roll response is dominated by image X;
  - responses are close to orthogonal and usable as live evidence for the
    later candidate scorer.
- Risk:
  - GT XY drift grew to `~0.157 m`, which is too high for committing a final
    calibration.
- Conclusion:
  - The one-run approach is feasible: acquire band, stabilize Z, excite axes,
    measure response online.
  - Before candidate scoring/validation, shorten and/or center the axis-probe
    protocol so drift does not accumulate across the pitch and roll sequences.

### iter10 - strict complementary return gate

- Status: `AXIS_RESPONSE_FAIL`.
- Change:
  - shorter pulses: `1 deg` for `0.25 s`;
  - local baseline per axis;
  - post-pair return gate: settle centroid must return within `8 px` of the
    axis-local baseline.
- Conditions:
  - 7 fully visible markers preserved throughout pulses;
  - visual-Z hold target `0.55 m`;
  - GT `z_max~=0.601 m`;
  - yaw span `<0.1 deg`;
  - GT XY radius max `~0.074 m`, improved from iter9 `~0.157 m`.
- Complementary image responses:
  - pitch `comp_px ~= (-0.01, -5.68)`, return error `~6.10 px`, accepted;
  - roll `comp_px ~= (-4.57, -0.12)`, return error `~10.37 px`, rejected.
- Conclusion:
  - Strict complementary gating works: it preserves the useful axis signal and
    rejects runs where the drone did not return close enough before the next
    calibration decision.
  - The next improvement should not relax the gate.  Instead, add a small
    image-space recenter/recover step between axes or reduce/shape the roll
    pulse so return error stays within the accepted band.

### iter13 - one-axis-at-a-time pitch smoke

- Status: `AXIS_RESPONSE_OK`.
- Change:
  - `AXIS_SMOKE_MAX_AXES=1`, so only pitch is measured in this smoke;
  - centroid-only X/Y response;
  - local baseline and strict return gate preserved.
- Conditions:
  - 7 fully visible markers preserved;
  - visual-Z hold target `0.55 m`;
  - GT `z_max~=0.596 m`;
  - GT XY radius max `~0.090 m`.
- Pitch response:
  - `comp_px ~= (0.03, -5.13)`;
  - return error `~7.23 px`, accepted under the `8 px` gate.
- Conclusion:
  - One-axis-at-a-time probing is the correct structure.
  - Pitch axis can be identified from centroid response and passes the local
    return gate.
  - Next step: run the same one-axis smoke for roll only.  If roll also passes
    independently, then implement the full one-run flow that measures pitch,
    re-stabilizes, measures roll from a new baseline, checks orthogonality, and
    only then moves to candidate scoring.

### iter14 plan - pitch then roll plus final centroid cleanup

- Change:
  - run both axes in one continuous sequence: pitch first, then roll;
  - keep a fresh local baseline for each axis;
  - keep active per-axis recenter disabled because iter11 showed that a single
    noisy response can overcorrect badly;
  - after both axis responses are accepted, run a separate final centroid
    cleanup before landing.
- Final cleanup rule:
  - use the measured pitch/roll complementary response vectors as a small
    local image-Jacobian estimate;
  - solve only for bounded low-amplitude R/P pulses that move the marker
    centroid toward the image center;
  - command at most three pulses, each clamped to `0.45 deg`;
  - keep visual-Z thrust active during cleanup;
  - skip cleanup and land if the axis response was not accepted, if one axis
    is missing, or if the two response vectors are weak/degenerate.
- A3/B3 alignment:
  - the cleanup does not persist calibration and does not encode a manual
    camera/body mapping;
  - it is allowed only after the current-run observations identify both axes;
  - if the axis gate fails, fail closed and descend instead of using rejected
    response data for XY control.
- Axis sign extraction:
  - for each accepted complementary response vector, record the dominant image
    axis (`image-x` or `image-y`);
  - record the sign of the dominant component for the positive body command;
  - require a minimum dominance ratio so ambiguous diagonal responses are
    rejected;
  - after both axes pass, require near-orthogonality before using the pair for
    candidate scoring or cleanup.

### iter14 - pitch and roll pass; final cleanup is not ready

- Status: `AXIS_RESPONSE_OK`.
- Conditions:
  - 7 fully visible markers preserved throughout acquisition, Z hold, both
    axis probes, and cleanup;
  - visual-Z hold target `0.55 m`;
  - `z_cam_last~=0.558 m`;
  - GT `z_max~=0.600 m`;
  - GT XY radius max `~0.117 m`.
- Axis responses:
  - pitch `comp_px ~= (0.11, -5.03)`, return error `~2.98 px`;
  - roll `comp_px ~= (-5.37, -0.29)`, return error `~3.87 px`;
  - both axes pass the strict `8 px` local return gate;
  - the response vectors are strong and near-orthogonal, so the A3 axis
    identification step is now demonstrated in one run.
- Final centroid cleanup:
  - attempted to move centroid toward image center using the measured response
    vectors as a 2x2 image Jacobian;
  - commanded three clamped pulses of `roll=-0.45 deg`, `pitch=+0.45 deg`;
  - the centroid moved farther from center after each settle;
  - final error grew to `~55.37 px` while all 7 markers remained visible.
- Conclusion:
  - the impulse response measured during axis probes is valid for identifying
    axes, but it is not yet a reliable settled-position controller;
  - final recenter must be treated as a separate control problem, not as a
    direct reuse of the single-pulse image Jacobian;
  - for safety, s195 changes final cleanup to at most one bounded pulse and
    aborts cleanup immediately if the settled error worsens.
  - s195 now extracts explicit axis signs from the complementary response:
    the sign is the sign of the dominant image component induced by the
    positive body-axis pulse.

### iter15 - fail-closed on return gate

- Status: `AXIS_RESPONSE_FAIL`.
- Conditions:
  - 7 markers remained fully visible;
  - visual-Z hold succeeded;
  - launch reported low camera FPS, about `2 Hz`, so timing/drift was poor.
- Axis signals:
  - pitch still had a strong dominant `image-y` response;
  - roll still had a strong dominant `image-x` response.
- Rejection:
  - pitch return error `~10.70 px`;
  - roll return error `~11.65 px`;
  - both exceed the strict `8 px` return gate.
- Code follow-up:
  - if an axis fails return/sign clarity, stop probing further axes and land;
  - final cleanup is skipped unless both axes are accepted.

### iter16 - accepted axis identity and signs

- Status: `AXIS_RESPONSE_OK`.
- Conditions:
  - 7 fully visible markers preserved;
  - visual-Z hold target `0.55 m`;
  - `z_cam_last~=0.558 m`;
  - GT `z_max~=0.601 m`;
  - GT XY radius max `~0.0895 m`;
  - launch reported low camera FPS, about `3 Hz`, but the run still passed
    all online gates.
- Accepted axis/sign observations:
  - positive pitch response:
    - dominant image axis: `image-y`;
    - sign: negative;
    - `comp_px ~= (0.00, -5.52)`;
    - dominance ratio effectively infinite because image-X coupling was near
      zero;
    - return error `~5.02 px`.
  - positive roll response:
    - dominant image axis: `image-x`;
    - sign: negative;
    - `comp_px ~= (-5.43, -0.55)`;
    - dominance ratio `~9.92`;
    - return error `~7.35 px`.
  - normalized pitch/roll dot product `~0.10`, accepted as near-orthogonal.
- Final cleanup:
  - one bounded cleanup pulse still worsened image-center error from
    `~40.98 px` to `~42.69 px`;
  - conclusion remains that direct settled-position recentering is not solved
    by the single-pulse Jacobian used for axis identification.
- Code follow-up:
  - final cleanup disabled for now (`FINAL_RECENTER_MAX_PASSES=0`);
  - keep axis/sign detection as the current B3 success;
  - next implementation step is candidate scoring/validation from the accepted
    signed response pair, not centroid recentering.

### iter17 - confirmation with final cleanup disabled

- Status: `AXIS_RESPONSE_OK`.
- Conditions:
  - 7 fully visible markers preserved;
  - visual-Z hold target `0.55 m`;
  - `z_cam_last~=0.560 m`;
  - GT `z_max~=0.6005 m`;
  - GT XY radius max `~0.087 m`;
  - launch reported low camera FPS, about `4 Hz`, but all online gates passed.
- Accepted axis/sign observations:
  - positive pitch response:
    - dominant image axis: `image-y`;
    - sign: negative;
    - `comp_px ~= (0.10, -5.26)`;
    - dominance ratio `~54.97`;
    - return error `~1.44 px`.
  - positive roll response:
    - dominant image axis: `image-x`;
    - sign: negative;
    - `comp_px ~= (-4.99, -0.41)`;
    - dominance ratio `~12.22`;
    - return error `~4.99 px`.
  - normalized pitch/roll dot product `~0.063`, accepted as near-orthogonal.
- Conclusion:
  - axis identity and signs are reproducible across iter16 and iter17;
  - the current reliable milestone is:
    `+pitch -> -image-y`, `+roll -> -image-x`;
  - next step is to convert this signed response pair into candidate rotation
    scoring and run explicit validation motions before any `commit_R/save`.

### iter18/iter19 - acquisition timeout due to SITL arm timing

- Status: `MARKER_ACQ_TIMEOUT`.
- Observation:
  - GT `z_max` stayed near ground (`~0.01 m`);
  - camera frames arrived, but the drone did not lift despite thrust commands;
  - cf2 log showed the startup sequence `SUP: Can not fly` followed by
    `SUP: Ready to fly`.
- Interpretation:
  - the single early `sentai.crazy.arm()` likely landed in the SITL not-ready
    window;
  - `sentai.crazy.arm()` returning means the packet was sent, not that the
    supervisor accepted flight state.
- Code follow-up:
  - add a short zero-thrust preamble;
  - send arm;
  - stream zero-thrust unlock packets;
  - retry arm after the SITL has had time to become ready;
  - stream a short second zero-thrust unlock before the takeoff ramp.

### iter20 - centroid PD with impulse sign fails

- Status: `CENTROID_VALIDATION_FAIL`.
- Earlier phases:
  - acquisition, visual-Z hold, and axis/sign detection all passed;
  - accepted signs remained `+pitch -> -image-y`, `+roll -> -image-x`;
  - axis orthogonality `dot_norm~=0.068`.
- Centroid validation:
  - initial centroid error `~25.87 px`;
  - final centroid error `~34.62 px`;
  - all 7 markers remained visible;
  - failure reason: `centroid_error_worse`.
- Interpretation:
  - the impulse response measured by short R/P pulses is valid for axis/sign
    discovery;
  - sustained centroid hold over seconds behaves like a lateral-position
    response and appears to have the opposite sign;
  - update the validation controller to use the inverse sign of the impulse
    Jacobian, with smaller gains and command clamp.

### iter21 - axis reject after excessive passive drift

- Status: `AXIS_RESPONSE_FAIL`.
- Earlier phases:
  - acquisition and visual-Z hold passed;
  - first pitch response had the expected dominant `image-y` negative sign.
- Rejection:
  - pitch return error `~14.7 px`, above the `8 px` gate;
  - GT post-mortem showed Y drift up to `~20 cm` before landing.
- Interpretation:
  - the vehicle can drift laterally during the long Z-hold before axis
    identification starts;
  - when this drift is too high, the strict local return gate correctly
    rejects the axis sample.
- Code follow-up:
  - shorten the Z-hold smoke window from `4.0 s` to `2.5 s`;
  - keep Z-hold valid-marker gates, but start axis identification sooner so
    passive XY drift has less time to accumulate.

### iter22 - roll sign clear but strict return gate rejects

- Status: `AXIS_RESPONSE_FAIL`.
- Earlier phases:
  - acquisition and shortened visual-Z hold passed;
  - pitch passed;
  - roll had clear dominant `image-x` negative sign;
  - pitch/roll orthogonality was good (`dot_norm~=0.053`).
- Rejection:
  - roll return error `~10.19 px`, above the old `8 px` return gate.
- Interpretation:
  - the return gate was useful before IBVS validation existed;
  - now axis/sign can be accepted under a slightly wider return gate because
    the subsequent centroid PD validation is the real closed-loop test and
    still fails closed before any persistence.
- Code follow-up:
  - widen `AXIS_RETURN_MAX_PX` from `8 px` to `12 px`;
  - keep marker count, dominance, sign clarity, and orthogonality checks.

### iter23 - full inverse PD is still not suitable

- Status: `CENTROID_VALIDATION_FAIL`.
- Earlier phases:
  - axis/sign detection passed with `AXIS_RETURN_MAX_PX=12`;
  - accepted signs remained `+pitch -> -image-y`, `+roll -> -image-x`.
- Centroid validation:
  - initial error `~27.74 px`;
  - final error `~31.05 px`;
  - all 7 markers remained visible.
- Tick-level observation:
  - Y was already close to center, then crossed and overcorrected;
  - X had the larger error and roll command saturated too low to reduce it
    enough.
- Code follow-up:
  - replace full 2x2 inverse control with dominant-axis control:
    roll corrects image-X, pitch corrects image-Y;
  - add a `6 px` deadband so near-centered axes are not chased;
  - increase max command to `0.45 deg` while keeping the error-worse abort.

### iter24 - dominant-axis centroid validation passes

- Status: `CENTROID_VALIDATION_OK`.
- Earlier phases:
  - acquisition passed;
  - shortened visual-Z hold passed with 7 fully visible markers;
  - axis/sign detection passed;
  - pitch/roll orthogonality `dot_norm~=0.077`.
- Accepted axis/sign observations:
  - positive pitch response:
    - dominant image axis: `image-y`;
    - sign: negative;
    - `comp_px ~= (0.01, -5.37)`;
    - return error `~4.24 px`.
  - positive roll response:
    - dominant image axis: `image-x`;
    - sign: negative;
    - `comp_px ~= (-4.66, -0.37)`;
    - return error `~1.96 px`.
- Centroid validation:
  - controller: dominant-axis PD/P, inverse impulse sign, deadband `6 px`,
    max command `0.45 deg`;
  - all 7 markers remained fully visible;
  - initial centroid error `~57.39 px`;
  - final centroid error `~51.54 px`;
  - improvement `~5.85 px`, passing the `3 px` smoke threshold.
- Interpretation:
  - this is the first one-run B3 smoke that demonstrates the chain:
    takeoff -> 7-marker lock -> visual-Z hold -> axis/sign discovery ->
    image-space centroid validation;
  - centroid validation is still weak and should be improved before real
    hardware, but it is enough to prove the measured signs can close a small
    image-space loop in SIM.
- Next step:
  - implement candidate scoring for determinant `+1` `R_cam_to_body`
    candidates using the accepted signed response pair;
  - keep `commit_R/save` disabled until candidate scoring and final validation
    motions pass in the same run.

## Candidate Scoring Smoke

Candidate scoring is now implemented as a no-persistence phase after IBVS
centroid validation:

- Generate all discrete signed permutation rotations with determinant `+1`.
- Interpret the current-run observations as:
  - body pitch command identifies body-X response in image space;
  - body roll command identifies body-Y response in image space.
- Score each candidate by comparing the candidate's body-X/body-Y rows against
  the observed dominant image axis and sign.
- Accept a candidate only if:
  - best score is `0`;
  - margin over the second candidate is at least `1`;
  - centroid validation already passed.
- Do not call `sentai.calib.commit_R` or `sentai.calib.save` in this smoke.

Bug fix:

- `final_centroid_recenter.reason` now distinguishes:
  - `skipped_axis_response_not_accepted`;
  - `skipped_centroid_validation_not_accepted`;
  - `skipped_candidate_not_accepted`;
  instead of always blaming axis response.

### iter25 - candidate scoring passes without persistence

- Status: `CANDIDATE_SCORING_OK`.
- Earlier phases:
  - acquisition passed;
  - visual-Z hold passed;
  - axis/sign detection passed;
  - centroid validation passed.
- Axis/sign observations:
  - `+pitch -> -image-y`, `comp_px ~= (0.04, -5.28)`;
  - `+roll -> -image-x`, `comp_px ~= (-4.96, -0.33)`;
  - orthogonality `dot_norm~=0.058`.
- Centroid validation:
  - initial error `~33.02 px`;
  - final error `~22.54 px`;
  - improvement `~10.48 px`;
  - all 7 markers remained visible.
- Candidate scoring:
  - best score `0.0`;
  - margin over second candidate `1.0`;
  - selected candidate:
    `R_cam_to_body = [0, -1, 0; -1, 0, 0; 0, 0, -1]`;
  - no `commit_R` or `save` was called.
- Interpretation:
  - the full no-persistence chain now passes in SIM:
    takeoff -> marker lock -> visual-Z -> axis/sign -> centroid validation ->
    determinant `+1` candidate scoring;
  - candidate margin is only at the current smoke threshold, so persistence
    should remain disabled until final validation motions make the margin
    stronger or provide a second independent confirmation.
- Next step:
  - add final validation motions for the selected `R_cam_to_body`;
  - after final validation passes in the same run, enable guarded
    `commit_R/save`.

## Final Candidate Validation Smoke

Final candidate validation is now implemented as a no-persistence phase after
candidate scoring:

- Use the selected `R_cam_to_body` candidate.
- Run smaller independent complementary validation pulses:
  - pitch: `0.6 deg`, `0.20 s`;
  - roll: `0.6 deg`, `0.20 s`;
  - neutral settle: `0.25 s`.
- For each body axis, compare the observed dominant image axis and sign with
  the axis/sign predicted by the selected candidate row.
- Keep the same marker and dominance safety gates:
  - 7 fully visible markers;
  - dominance ratio at least `2`;
  - response strength at least `2 px`;
  - return error at most `14 px`.
- Do not call `sentai.calib.commit_R` or `sentai.calib.save` in this smoke.

New status ladder:

- `CANDIDATE_SCORING_OK` is no longer final success;
- a run must now pass final validation to reach `FINAL_VALIDATION_OK`;
- persistence remains disabled until `FINAL_VALIDATION_OK` is reproducible.

### iter26 - final validation passes without persistence

- Status: `FINAL_VALIDATION_OK`.
- Run command:
  `bash examples/sentai_runtime/experiments/s195_sota_uncalibrated_visual_servo/run.sh iter26`
- Guardrails:
  - no `get_drone_pose_tuple`;
  - no EKF/yaw/Gazebo GT in the mission decision path;
  - no `sentai.calib.commit_R`;
  - no `sentai.calib.save`;
  - GT was used only by the host-side verdict after landing.
- Acquisition:
  - 7 fully visible markers locked for 10 consecutive frames at `t=9.108 s`;
  - lock `z_cam_mean_m~=0.532 m`;
  - visual-Z hold passed with `75` valid ticks and target `0.55 m`.
- Axis/sign observations:
  - `+pitch -> -image-y`, `comp_px ~= (-0.05, -5.99)`,
    dominance `~116.88`, return error `~6.47 px`;
  - `+roll -> -image-x`, `comp_px ~= (-5.52, -0.22)`,
    dominance `~25.28`, return error `~7.71 px`;
  - orthogonality `dot_norm~=0.048`, accepted.
- Centroid validation:
  - initial error `~37.04 px`;
  - final error `~10.97 px`;
  - improvement `~26.07 px`;
  - all 7 markers remained visible.
- Candidate scoring:
  - best score `0.0`;
  - margin over second candidate `1.0`;
  - selected candidate:
    `R_cam_to_body = [0, -1, 0; -1, 0, 0; 0, 0, -1]`;
  - candidate was not committed or saved.
- Final validation:
  - pitch validation expected `image-y:-1` and observed `image-y:-1`;
  - roll validation expected `image-x:-1` and observed `image-x:-1`;
  - both validation pulses preserved 7-marker visibility and passed return
    gates.
- GT post-mortem:
  - `z_max~=0.5995 m`;
  - `xy_radius_max~=0.0876 m`;
  - yaw span `~0.06 deg`.
- Conclusion:
  - the no-persistence one-run A3 chain is now validated once end-to-end:
    takeoff -> 7-marker lock -> visual-Z hold -> axis/sign discovery ->
    centroid validation -> candidate scoring -> final candidate validation;
  - persistence should still remain disabled until this exact chain passes
    repeatedly and selects the same `R_cam_to_body`.

### iter27 - repeat final validation plus landing GT check

- Status: `FINAL_VALIDATION_OK`.
- Run command:
  `bash examples/sentai_runtime/experiments/s195_sota_uncalibrated_visual_servo/run.sh iter27`
- Result:
  - same selected candidate as iter26:
    `R_cam_to_body = [0, -1, 0; -1, 0, 0; 0, 0, -1]`;
  - no `commit_R` or `save` was called;
  - GT was used only after landing for post-mortem validation.
- Acquisition and calibration:
  - 7 fully visible markers locked for 10 consecutive frames at `t=9.108 s`;
  - visual-Z hold passed with target `0.55 m`;
  - `+pitch -> -image-y`, dominance `~382.03`, return error `~5.94 px`;
  - `+roll -> -image-x`, dominance `~13.66`, return error `~3.08 px`;
  - centroid validation improved from `~43.99 px` to `~35.25 px`;
  - final validation matched the selected candidate on both axes.
- Landing GT check against takeoff location:
  - start pose:
    `x~=0.0000 m`, `y~=0.0000 m`, `z~=0.0012 m`,
    `yaw~=0.00 deg`;
  - final pose:
    `x~=-0.00024 m`, `y~=-0.01631 m`, `z~=0.0150 m`,
    `yaw~=-0.053 deg`;
  - final XY landing offset from takeoff:
    `~0.0163 m`;
  - final two-second touchdown-window mean XY offset:
    `~0.0163 m`;
  - maximum XY radius during the run:
    `~0.0483 m`;
  - maximum GT altitude:
    `~0.5992 m`.
- Interpretation:
  - the vehicle lands close to the takeoff point in SIM, within about
    `1.6 cm` in the final GT sample;
  - the landing is not mathematically exact, but it is within the current
    smoke-test envelope and much smaller than the maximum in-flight lateral
    excursion;
  - if A3 later requires a tighter landing criterion, add a final image-space
    recenter/landing controller after calibration acceptance.

## s197 Organized Guarded Successor

s197 was created as a cleaned-up successor to s195:

```text
examples/sentai_runtime/experiments/
  s197_sota_calib_orientation_guarded/
```

Changes relative to s195:

- keep the proven s195 phase structure;
- abort immediately if full-marker lock is weak before an axis pulse;
- clarify candidate scoring labels so command responses are not confused with
  body-axis names;
- add `optical_axis_validation` before final validation:
  - selected candidate body-Z row must have the expected camera-Z sign;
  - all 7 fully visible markers must have valid positive `tz`;
  - this validates the inferred optical/body-Z convention before any future
    `commit_R/save`;
- keep persistence disabled.

Run notes:

- `s197/iter1`: failed closed at axis response.  Pitch pulse dropped to
  `min_full=5` and dominance was weak, so the new guard correctly stopped
  before candidate scoring.
- `s197/iter2`: passed with `FINAL_VALIDATION_OK`, selected the same
  `R_cam_to_body = [0, -1, 0; -1, 0, 0; 0, 0, -1]`, and
  `optical_axis_validation_ok=True`.
- `s197/iter3`: used to verify launcher FPS reporting.  It reported
  `camera FPS ~= 30.3 Hz`; the mission itself failed earlier at
  `VISUAL_Z_HOLD_FAIL` because the vehicle drifted/lost marker lock, which is
  acceptable fail-closed behavior for this smoke.

Launcher note:

- The earlier `2-3 Hz` camera message was a measurement artifact in
  `sim/scripts/launch_sim.sh`.
- The old readiness check counted sparse diagnostic `[bridge] seq=` log lines.
  The actual `bridge_recv` sequence/timestamps in `s197/iter2` show about
  `30.3 Hz`.
- The launcher now prefers `bridge_recv` seq/timestamps and falls back to the
  old line-count method only if timestamps are unavailable.

Additional s197 notes:

- Added `abort_decision` journaling in the REPL mission.  Before descent, the
  mission now writes `abort_reason`, `abort_detail`, and a journal event with
  the failing phase's compact summary.
- Added final `return_to_center + center_hover` as part of mission success.
  `FINAL_VALIDATION_OK` now requires final candidate validation, centroid
  return, and a short hover near the image center.
- `s197/iter7`: reached `final_centroid_recenter` after all orientation gates
  passed, then raised exception `centroid_px`.  Root cause: final recenter used
  `_feature_compact(...)` output but read the old `centroid_px` key.  Fixed by
  using `n_full/cx/cy`.
- `s197/iter8`: full success:
  - `mission_status=FINAL_VALIDATION_OK`;
  - `mission_abort_reason=` empty;
  - selected the same
    `R_cam_to_body = [0, -1, 0; -1, 0, 0; 0, 0, -1]`;
  - `visual_z_hold` reached target `0.64 m`;
  - `final_recenter_ok=True`;
  - `final_center_hover_ok=True`;
  - hover max centroid error `~16.53 px`, last hover error `~13.12 px`.
- Remaining issue:
  - GT final XY offset was still `~0.071 m` in `s197/iter8`;
  - interpretation: centering/hover works before landing, but the final manual
    descent does not maintain visual centroid.  A3 needs a controlled
    center-hold descent phase if landing on the centroid is part of the
    success criterion.

## Notes

- Keep A3 unchanged unless the objective/specification changes.
- Use B3 for implementation details, run notes, failed attempts, and concrete
  code-level observations.
- Generated runtime artifacts belong under `dataset/TD-S10-B3/`.
- Gazebo ground truth may be recorded host-side for post-mortem only.
- Do not claim success from a run that was headless or that did not land/clean
  up safely.

## Open Blockers

- Re-run final validation several times to verify reproducibility of
  `FINAL_VALIDATION_OK` and the selected
  `R_cam_to_body = [0, -1, 0; -1, 0, 0; 0, 0, -1]`.
- Decide when the evidence is strong enough to enable guarded
  `sentai.calib.commit_R(...)` and `sentai.calib.save()`.
- Keep final recenter disabled until it has a separate safe controller; it is
  not required for accepting the camera/body orientation.

## s197 IBVS Controller Update

SOTA sources used for the current s197 direction:

- Chaumette and Hutchinson visual-servoing tutorials:
  - use image feature error `e = s - s*` and an interaction/image Jacobian;
  - avoid switching to pose/body-frame assumptions before the camera/body
    transform is validated;
  - map to s197: centroid hold, return-to-center, and landing are IBVS-style
    camera-coordinate loops.
- Uncalibrated IBVS / online image-Jacobian estimation:
  - estimate the local relationship between commands and image-feature motion
    from the current run;
  - map to s197: complementary roll/pitch pulses build a local `2x2` response
    matrix used by DLS control and candidate scoring.
- Noise-aware uncalibrated visual servoing / Kalman-filtered Jacobian work:
  - visual-feature tracker noise must be handled explicitly or Jacobian
    estimation can converge to the wrong result;
  - map to s197: response validation now measures centroid jitter before each
    axis check and gates evidence relative to measured noise, not fixed pixels.
- Adaptive IBVS / adaptive excitation for underactuated quadrotors:
  - when visual response is coherent but weak, adapt gain/excitation inside a
    safe envelope instead of failing immediately;
  - map to s197: final validation escalates from small pulses to larger safe
    pulses only when marker lock, axis, sign, dominance, and return behavior
    remain consistent.

Implementation rule from A3:

- Avoid magic pixel constants.  Any remaining fixed pixel values in s197 are
  temporary scaffolding and should be converted to:
  - measured `sigma_px` gates for deadband/response observability;
  - FOV-margin and marker-visibility safety checks;
  - convergence/divergence trends over windows;
  - command saturation and Jacobian conditioning checks.
- MP must not serialize large forensic objects.  It appends detailed events to
  the journal and writes only compact summaries.
- On success, write a compact calibration artifact on the simulated drone FS,
  separate from the journal and summary.  This artifact should persist the
  empirically discovered constants/estimates from the run, not hand-tuned
  values.

Planned calibration artifact schema:

- `schema_version`
- `task_id = TD-S10-B3`
- `run_id`
- `world/layout_id`
- `image`: width, height, intrinsics
- `marker`: diameter, layout coordinates/hash
- `R_cam_to_body`
- `axis_response`:
  - roll complementary response vector;
  - pitch complementary response vector;
  - orthogonality/conditioning metrics;
  - accepted excitation amplitudes.
- `noise`:
  - centroid sigma estimates used in validation/landing;
  - noise gates derived from sigma;
  - sample counts.
- `controller_estimate`:
  - local DLS/IBVS response matrix;
  - damping/gain values if still fixed, flagged as safety envelope or
    temporary scaffold.
- `validation`:
  - final candidate validation status;
  - per-axis confidence/attempt count;
  - final recenter/hover status;
  - landing/center-hold status.
- `safety_envelope`:
  - max pulse used;
  - command clamps;
  - marker/FOV constraints;
  - altitude band reached.

Do not store large frame-by-frame traces in this artifact.  Those stay in the
append-only journal for host-side post-mortem.

Replace the dominant-axis/sign-only centroid controller with a local
image-Jacobian controller:

- Use the measured complementary roll and pitch response vectors as columns of
  a `2x2` image response matrix.
- Because previous runs showed that sustained lateral-position response has the
  opposite sign from the short image impulse, use the negated response vectors
  for the hold/landing controller while keeping the raw vectors in logs.
- Solve the coupled XY correction with damped least squares instead of two
  independent scalar sign rules.
- Apply the same controller in centroid validation, final recenter,
  center-hover, and center-hold descent.
- Keep visual-Z thrust correction active in the same loop.

First run:

- `s197/iter11_ibvs_centered` used the new DLS-IBVS controller.
- Result: orientation gates passed, candidate validation passed, final recenter
  improved centroid error from `~31.23 px` to `~15.95 px`.
- It failed only at center-hover because hover max error reached `~18.06 px`
  against an `18 px` threshold.
- Tuning response:
  - extend final recenter from `3.5 s` to `4.5 s`;
  - allow `0.70 deg` max final-centering command, matching the already used
    final validation pulse magnitude;
  - keep the center-hover tolerance at `18 px`, but do not abort immediately
    when a single sample is just above it;
  - require a stable `1.2 s` window under threshold, with up to `3.0 s` total
    hover time while the IBVS error is still bounded/converging;
  - abort hover only on marker loss, timeout without a stable window, or clear
    divergence beyond the best hover error.

Second run:

- `s197/iter12_ibvs_adaptive_hover` did not reach final recenter.  It failed
  in final candidate validation because the roll validation response was
  directionally correct but only `~1.80 px`, just below the old `2.0 px`
  response gate.
- This is not a hard safety failure: marker lock, sign, dominance, return
  error, and optical-axis validation were all acceptable.
- Follow-up design correction:
  - do not use a fixed `1 px`/`2 px` response threshold for final validation;
  - do not scale response validity by marker radius or altitude in this phase;
  - treat final validation as an online excitation/observability problem.
- New final-validation rule:
  - before testing each axis, hold neutral RPY and measure centroid jitter for
    `FINAL_VALIDATE_NOISE_SAMPLES` frames;
  - derive a per-axis noise gate from `sigma_px * FINAL_VALIDATE_NOISE_SIGMA_MULT`
    with a small floor only to avoid accepting zero-jitter artifacts;
  - run complementary `+pulse/-pulse` validation;
  - accept only when axis, sign, dominance, marker lock, return error, and
    response-above-noise are all consistent;
  - if response is coherent but below the noise gate, return/settle and retry
    with larger excitation within the safe pulse envelope;
  - log every attempt and the measured noise gate for forensic review.

Third run:

- `s197/iter13_noise_aware_adaptive_excitation` did not reach final validation.
- Abort reason was `axis_response_failed`, caused by the roll axis
  `return_err_px ~= 14.2` exceeding the old `AXIS_RETURN_MAX_PX = 12`.
- Important: roll/pitch axis signs were still identified correctly, marker
  visibility stayed at `7`, and orthogonality passed.  The failure was another
  fixed return-to-baseline gate, not loss of visual evidence.
- Update: keep axis return error as a diagnostic, but do not fail
  `axis_response` solely because the centroid did not return under the old
  baseline gate.  Later IBVS centroid validation/recenter phases are the
  proper closed-loop recovery tests.
- Altitude update:
  - before X/Y excitations, climb to a calibration altitude derived from the
    stable Z target: `Z_calib = 1.30 * Z_HOLD_TARGET_M`;
  - cap the target at `Z_HOLD_TARGET_MAX_M = 0.88 m`;
  - keep the observed lock/brake Z as input too, but multiply it by the same
    gain before comparing;
  - purpose: shrink the marker constellation in the image and create FOV
    margin so excitation plus drift is less likely to push markers out of
    frame.

Fourth run:

- `s197/iter14_z130_noise_aware` showed the higher takeoff/Z stabilization is
  workable:
  - target `Z ~= 0.832 m`;
  - measured `z_cam_max ~= 0.856 m`, so the target was reached;
  - final `z_cam_last ~= 0.775 m`, slightly below the old
    `target - tol ~= 0.792 m` gate.
- Abort reason was `visual_z_hold_failed`, caused by checking only the last Z
  sample instead of accepting that the calibration altitude had been reached.
- Update:
  - raise `Z_HOLD_TARGET_MAX_M` to `1.05 m`;
  - keep marker loss as the real high-altitude abort;
  - accept Z-hold if the target was reached at peak during the phase, even if
    the last sample has settled slightly below the tolerance band.

Fifth run:

- `s197/iter15_z_peak_accept` visually executed the full mission path and the
  append-only journal confirms:
  - Z-hold passed at `z_cam ~= 0.836 m`;
  - final candidate validation passed both axes using noise-aware adaptive
    excitation;
  - final recenter and center-hover passed, with hover error reaching
    `~2.49 px`;
  - center-hold descent later exceeded the current `45 px` center-error guard
    and disarmed.
- The generated `summary.json` was stale/misleading because MicroPython hit a
  `MemoryError` while serializing the large nested summary at mission end.
- Update:
  - keep detailed forensic data in append-only journal events;
  - write only a compact summary surface from MP;
  - fallback to a minimal summary on `MemoryError`.

Current implementation update:

- `mission_s197.py` now writes `mission_s197_calibration.json` on the
  simulated drone FS when final validation has run.
- `run.sh` copies this artifact into the iteration folder alongside the
  journal and compact summary.
- The artifact is intentionally compact and contains accepted matrix,
  candidate metadata, axis response vectors, final-validation noise gates,
  recenter/landing status, and safety envelope metadata.
- Landing center-hold deadband is being moved away from fixed pixels: it is
  derived from a neutral centroid-noise measurement before descent, with only
  a small floor as a safety/scaffold guard.

Sixth run:

- `s197/iter16_artifact_noise_landing` was useful as a review run:
  - compact summary worked;
  - final validation failed because the pitch noise gate was `~8.57 px`;
  - the measured response was coherent and grew with larger excitation, but
    stayed below that inflated gate.
- Root cause: the neutral "noise" estimator was measuring slow centroid drift
  as if it were jitter.
- Update:
  - `_measure_centroid_noise` now estimates linear drift during neutral RPY
    and computes `sigma_px` from detrended residuals;
  - raw sigma and drift are still logged for forensic review;
  - s197 no longer calls `sentai.calib.load()` during setup.  It writes the
    FS artifact at the end, but does not reload prior calibration state.

Centroid validation role update:

- Visual observation and iter17 both show that takeoff/Z-hold can drift
  laterally while keeping all markers visible.
- This drift should not block camera/body orientation calibration.  Axis
  response and final candidate validation are the evidence for the matrix.
- `centroid_pd_validation` is now diagnostic only: it logs whether the current
  local IBVS controller improves center error, but candidate scoring,
  optical-axis validation, final validation, and artifact writing are allowed
  to proceed when axis response is valid.

Landing descent update:

- The final center-hold landing should no longer use a blind linear thrust
  ramp.
- New rule: keep the camera-referenced IBVS centering loop active, but compute
  descent thrust from visual `z_cam` velocity so the drone follows a bounded
  downward velocity until the near-ground visual condition is reached.
- Do not pause descent on an absolute pixel error while marker lock is still
  present.  The IBVS loop recenters laterally in parallel; descent stops only
  on the near-ground visual condition, marker loss, timeout, or the larger
  safety guard.
- Disarm remains tied to the visual condition of the fully visible marker count
  reaching the configured near-ground threshold (`4` for this smoke), not to a
  wall-clock timeout.
- Forensics to inspect after the next run:
  - `center_hold_descend_rate_control`;
  - `center_hold_descend_tick.vz_filt_m_s`;
  - `center_hold_descend_tick.descend_thrust`;
  - `center_hold_descend_tick.descent_paused`;
  - final `z_cam_*`, marker count, and GT XY landing offset.

State-machine review after iter20/iter21:

- The mission now behaves like an implicit state machine, but the state
  contract is spread across phase functions instead of being explicit.
- This creates rule drift:
  - visual-Z hold uses a target-altitude PD controller;
  - axis pulses and final recenter reuse visual-Z hold;
  - landing uses a separate vertical-rate controller;
  - recenter, hover, descent, and final validation each use slightly different
    marker-lock and pixel/error gates.
- iter21 shows the landing-rate controller does descend, but it still stops on
  timeout before the four-marker near-ground condition.  The descent went from
  `z_cam ~= 0.84 m` to `z_cam ~= 0.40 m`, while GT still reported
  `z ~= 0.21 m`; therefore timeout is currently a control-policy limit, not a
  marker-detection limit.
- iter24 confirmed the same pattern with the running-average marker policy:
  calibration, final validation, final recenter, and center hover succeeded;
  all seven markers stayed visible during landing (`avg_full_markers = 7.0`),
  descent reached only `z_cam ~= 0.36 m`, and the phase ended by timeout
  before the four-marker near-ground trigger.  The landing descent window was
  therefore extended from `8 s` to `14 s` while keeping the same visually
  measured descent-rate controller.
- iter25 confirmed the longer descent window is active (`duration_s = 14.0`),
  but exposed the next local landing guard: descent stopped on
  `center_error_too_large` at `err_px ~= 45.5`, while all seven markers were
  still visible (`avg_full_markers = 7.0`) and `z_cam ~= 0.44 m`.  This is no
  longer a marker-count or timeout problem; it is the phase-local fixed pixel
  guard interrupting an otherwise visually locked descent.
- Fix after iter25: remove the absolute `center_error_too_large` landing abort.
  During center-hold descent, centroid error remains logged as forensic data
  (`err_max_px`, `err_last_px`), but it no longer stops the mission while marker
  lock is present.  Landing exits on the near-ground marker-count condition,
  sustained marker loss, or the operational timeout.
- Acceptance contract update: the calibration artifact is accepted only when
  final candidate validation, final recenter/hover, and the camera-referenced
  landing proof are all true.  The landing proof does not use GT: it requires
  center-hold descent to stay active until the 10-frame running average of fully
  visible markers reaches the near-ground threshold (`avg_full_markers <= 4`).
- Landing controller update: center-hold descent should not keep forcing
  downward velocity while the lateral controller is saturated and centroid
  error is worsening.  In that case descent is delayed and the controller uses a
  visual zero-vertical-rate hold while it recenters, based on command
  saturation plus 10-frame error trend rather than a fixed pixel abort.
- New s199 direction: after final centroid hover and before landing, add an
  `axis_envelope_survey` phase:
  - move to positive X visual envelope, then negative X visual envelope, then
    center;
  - move to positive Y visual envelope, then negative Y visual envelope, then
    center;
  - hover on centroid again, then perform slow camera-referenced landing;
  - envelope extrema are detected from marker center/radius against image
    bounds (`fov_margin_px`), not GT;
  - accepted runs persist the discovered calibration constants to `calib.ini`
    on the drone/sim FS.
- A3 principle for s199: this is still part of the calibration process, not a
  general navigation task.  The axis-envelope survey is a calibration phase
  that validates the already inferred camera/body mapping over a wider visual
  operating envelope and discovers image-space limits that can be persisted for
  later takeoff/landing policy.  It must remain camera-only, use no GT, keep
  the non-moving image axis centered while probing the active axis, and fail
  closed without persisting if the visual evidence becomes ambiguous.
- s199 axis-envelope control convention:
  - during `x_max/x_min`, X is the active image axis and Y is the guarded
    perpendicular axis;
  - during `y_max/y_min`, Y is active and X is guarded;
  - the active axis may move toward the visual FOV envelope only while the
    perpendicular centroid error is not trending worse;
  - if the perpendicular axis drifts, freeze the active-axis target at the
    current centroid and recover the guarded axis to center before continuing;
  - this is a camera-only IBVS rule and does not use GT.
- s199 `iter1_axis_envelope_survey` post-mortem:
  - axis/sign identification looked correct: pitch response was dominated by
    image-Y with negative sign, roll response by image-X with negative sign;
  - candidate matrix and final validation passed before the new envelope
    phase;
  - failure occurred on `x_max`: `cx` moved in the expected direction
    (`~158 -> ~253 px`), but `cy` drifted away from center (`~110 -> ~59 px`);
  - GT post-mortem showed large lateral drift, mostly world-Y
    (`span ~= 0.38 m`), with yaw still zero;
  - conclusion: this was not primarily a wrong axis/sign inference.  The
    envelope phase incorrectly let the active-axis command dominate while the
    perpendicular axis was not held.  Fix: guarded perpendicular-axis recovery
    during envelope probing.
- s199 `iter2_guarded_orth_axis` post-mortem:
  - axis/sign identification again passed with the same mapping;
  - final candidate validation passed;
  - `x_max` no longer pushed markers close to image loss
    (`avg_full_markers=7.0`, `min_fov_margin_px~=31 px`);
  - however, it timed out before reaching the envelope because recovery mode
    consumed about half the leg (`79/151` ticks) and the perpendicular
    correction was too weak while drift continued;
  - fix: make perpendicular-axis recovery the priority mode by using stronger
    recovery gain and a longer bounded survey leg.  This is still a safety
    envelope, not a learned calibration result; success remains image-only and
    requires reaching the FOV/marker-margin condition.
- s199 `iter3_stronger_orth_recovery` post-mortem:
  - this run did not reach the envelope phase because final centroid recenter
    failed first;
  - axis/sign identification, candidate scoring, and final validation passed;
  - GT forensic drift was lower than iter2 (`xy_radius_max ~= 0.13 m`), so the
    flight was not the obvious failure mode;
  - root cause: final recenter exited after a single tick when the centroid was
    below an old fixed pixel tolerance, then the hover gate rejected a later
    visual oscillation using another fixed pixel divergence threshold;
  - fix: final recenter should run its time window and accept/reject using
    marker lock plus convergence trend, not one instantaneous pixel threshold.
- s199 `iter4_recenter_trend_envelope` post-mortem:
  - run aborted with a code exception in `final_centroid_recenter`
    (`err_win` was referenced before initialization);
  - this is not flight evidence and does not invalidate the axis/sign result;
  - fix: initialize and maintain the recenter error window in the final
    recenter phase before using it for trend evaluation.
- s199 `iter5_recenter_bugfix_envelope` post-mortem:
  - final recenter passed and the new envelope phase progressed further;
  - `x_max` correctly reached the positive visual envelope;
  - `x_min` was falsely accepted because the markers were still touching the
    positive edge from the previous leg.  The stop condition saw "an edge" but
    not the requested target-side edge;
  - fix: edge success now requires both image-margin evidence and target-side
    evidence.  For example, `x_min` can only complete from an edge condition
    after the centroid has crossed to the negative side of image center.
  - center recovery after an extreme is allowed the same longer bounded window
    as envelope motion, because returning from an image edge is a large visual
    correction.
- s199 `iter6_target_side_envelope` post-mortem:
  - run failed before envelope because axis-response identification rejected
    the roll axis;
  - pitch was clean, but roll response was ambiguous/diagonal and the
    orthogonality check failed;
  - this is the correct fail-closed behavior for one noisy excitation, but the
    one-run A3 process should not give up immediately when markers are still
    locked;
  - fix: add a bounded axis-response retry.  On rejected axis response, hold
    visual Z with neutral RPY for a short settle window, then repeat the
    pitch/roll complementary probe.  Persist only if one attempt passes all
    gates.
- s199 `iter7_axis_retry_target_side` post-mortem:
  - axis/sign and final validation passed;
  - `x_max` timed out because the envelope survey started while the
    perpendicular axis was already far from center (`cy` around `146 px`);
  - the previous hover gate was too tolerant and accepted a hover whose error
    later grew;
  - fix: final hover trend acceptance now requires the hover error to end no
    worse than the recenter endpoint, or to show a decreasing running-average
    trend.  The envelope survey also starts with an explicit `pre_center` leg
    before the first `x_max`.
- s199 `iter8_precenter_stricter_hover` post-mortem:
  - `pre_center` and `x_max` succeeded;
  - direct `x_max -> x_min` failed because the image starts the `x_min` leg
    from a degraded edge/cropped-marker state, making the centroid and
    response less reliable;
  - fix: add recovery center legs between opposite extrema:
    `x_max -> x_center_after_max -> x_min -> x_center`, then the same pattern
    for Y.  This preserves the intended max/min/center evidence while keeping
    each edge probe initialized from a usable centered image.
- s199 `iter9_center_between_extrema` post-mortem:
  - did not reach envelope because both bounded axis-response attempts failed
    orthogonality;
  - attempt 1 had pitch clean but roll ambiguous/mostly image-Y;
  - attempt 2 had pitch contaminated by image-X and roll image-X, so the two
    response vectors became nearly anti-parallel;
  - fail-closed behavior was correct and no `calib.ini` was written;
  - next improvement should make axis-response retries more independent:
    reset/recenter the centroid between attempts or split pitch/roll retries
    per-axis instead of repeating the full two-axis sequence from the drifted
    state.
- s199 artifact hygiene:
  - `run.sh` now removes stale `mission_s199_*` and `calib.ini` files from the
    sim FS before every run;
  - a failed s199 run must not snapshot an old `calib.ini` and present it as a
    fresh calibration result.
- Next implementation should introduce an explicit small flight context and
  shared policy helpers:
  - one `VisualState` sampled once per tick;
  - one `MarkerLockPolicy` for acquisition, calibration, hover, and landing;
  - one `ZController` with modes `CLIMB_TO_BAND`, `HOLD_BAND`,
    `DESCEND_RATE`, and `ZERO_THRUST`;
  - one `CentroidIBVSController` used by centroid validation, final recenter,
    center hover, and landing;
  - phase code should choose desired mode/state, not reimplement thresholds.
- Until that refactor exists, avoid adding more phase-local constants or
  one-off gates.
- Added explicit `phase_transition` journal events in `mission_s197.py`.
  Every top-level transition now logs previous phase, next phase, and the
  reason, so post-mortem analysis can align control behavior with the active
  mission state.
- Marker-count gates are now smoothed over a 10-frame running average.
  The policy is `avg_full > N - 1` where older code expected all `N` markers
  on an instantaneous frame.  For the current seven-marker pad this means
  `avg_full > 6.0`.
- Safety gates use the same running-average policy to avoid aborting because
  the detector misses a marker for one frame.  A hard floor remains only for
  real loss of usable visual evidence, currently `AXIS_HARD_MIN_FULL_MARKERS`
  for motion/validation phases and repeated zero-marker loss for descent.
- Axis and final-validation segment logs now include `axis_segment_summary`
  with `min_full`, `avg_full`, thresholds, and `marker_lock_ok`.

Current s197 phase map:

- `setup`: initialize camera, marker detector, marker layout, simulator, and
  safety surfaces.
  - Entry reason: `mission_start`.
  - Exit: setup APIs return success.
- `preflight_features`: sample marker detections before arming.
  - Entry reason: `setup_ok`.
  - Exit: preflight sample window finishes.
- `arm_zero_unlock`: arm Crazyflie and send zero-thrust unlock pulses.
  - Entry reason: `preflight_sampled`.
  - Exit: arming succeeds; otherwise status `ARM_FAIL`.
- `thrust_only_marker_acquisition`: climb with neutral RPY until the marker
  band is detected.
  - Entry reason: `armed`.
  - Exit: 7 fully visible markers for 10 consecutive frames; otherwise
    recovery path with `marker_acquisition_timeout`.
- `post_lock_brake`: damp vertical inertia immediately after visual lock.
  - Entry reason: `marker_lock_acquired`.
  - Exit: filtered vertical velocity is small enough while marker lock holds.
- `visual_z_hold`: climb/hold to the calibration altitude band with all
  markers visible.
  - Entry reason: `post_lock_brake_ok`.
  - Exit: target band reached and marker lock held; otherwise
    `visual_z_hold_failed`.
- `axis_response_smoke`: apply complementary pitch/roll excitations and
  measure image-space axis, sign, dominance, and orthogonality.
  - Entry reason: `visual_z_hold_ok`.
  - Exit: both axes pass response checks; otherwise `axis_response_failed`.
- `centroid_pd_validation`: diagnostic IBVS centering check.
  - Entry reason: `axis_response_ok`.
  - Exit: always non-blocking for candidate scoring in current s197.
- `candidate_scoring`: score discrete camera/body rotation candidates from
  measured pitch/roll image response.
  - Entry reason: `axis_response_ok`.
  - Exit: unique acceptable candidate; otherwise `candidate_scoring_failed`.
- `optical_axis_validation`: verify selected optical-axis sign using marker
  pose diagnostics.
  - Entry reason: `candidate_ok`.
  - Exit: candidate optical axis passes; otherwise
    `optical_axis_validation_failed`.
- `final_candidate_validation`: re-excite both axes with noise-aware gates to
  confirm candidate matrix.
  - Entry reason: `optical_axis_ok`.
  - Exit: both axes validate; otherwise `final_candidate_validation_failed`.
- `final_centroid_recenter`: drive marker-pad centroid back toward image
  center and hold it briefly.
  - Entry reason: `final_candidate_validation_ok`.
  - Exit: recenter and hover proof pass; otherwise `return_to_center_failed`.
- `manual_descend_disarm`: fallback recovery descent when calibration did not
  reach final recenter.
  - Entry reason: `select_recovery_or_landing`.
  - Exit: disarm attempted.
- `center_hold_descend_disarm`: camera-referenced landing after successful
  calibration/recenter.
  - Entry reason: `final_recenter_ok`.
  - Exit: near-ground marker-count trigger, marker-loss safety, timeout, or
    large center-error safety guard.
- `post_acquisition`: final status assignment, artifact write, summary write.
  - Entry reason: `flight_sequence_done`.

## Iter29 Runtime Calib.ini Source For B4

Run:

- `examples/sentai_runtime/experiments/s197_sota_calib_orientation_guarded/iter29_runtime_calib_ini_source_for_b4`

Result:

- `mission_status=FINAL_VALIDATION_OK`;
- `mission_phase=post_acquisition`;
- B3 wrote `calib.ini` directly from `sentai_runtime`/REPL to
  `/system/calib.ini`;
- `run.sh` copied the runtime-written file into the iteration folder as
  `calib.ini`;
- no host-side JSON-to-INI conversion was used for the accepted artifact.

Key artifact checks:

- `schema=2`;
- `status=FINAL_VALIDATION_OK`;
- `accepted=1`;
- `layout_id=sentai_whycon_small_centroid_7_marker_v1`;
- `R_B_C=0,-1,0,-1,0,0,0,0,-1`;
- `extpos_sign_x=-1`, `extpos_sign_y=-1`, `extpos_sign_z=1`.

Runtime `calib.ini` format rule:

- only `key=value` lines;
- no comments;
- no empty lines;
- no free-text provenance fields;
- keep only calibration, detector, estimator-sign, and safety parameters needed
  by runtime consumers.

## Iter35 Strict Scalar Calib.ini Source

Run:

- `examples/sentai_runtime/experiments/s197_sota_calib_orientation_guarded/iter35_strict_scalar_calib_ini_source`

Purpose:

- regenerate the accepted A3 artifact after removing host-side conversion and
  after replacing non-scalar `calib.ini` values with simple runtime keys.

Result:

- `mission_status=FINAL_VALIDATION_OK`;
- `accepted=1`;
- `status=FINAL_VALIDATION_OK`;
- `calib.ini` was written by `sentai_runtime`/REPL to `/system/calib.ini`;
- no JSON-to-INI host conversion was used;
- strict scalar format check passed:
  - no comments;
  - no blank lines;
  - no dict/list/freeform serialized values;
  - only `key=value` records.

Important runtime keys now present:

- `R_B_C`;
- `cam_offset_B`;
- `candidate_score`, `candidate_margin`, `candidate_det`;
- `axis_orthogonality_ok`, `axis_orthogonality_dot_norm`;
- `axis_roll_*` and `axis_pitch_*` response vectors, signs, dominance, return
  errors, and noise gates;
- `extpos_sign_x/y/z`, treated as Crazyflie estimator-frame adapter metadata;
- safe envelope scalars such as `safe_z_target_m`, `safe_z_max_m`, and
  `safe_max_pulse_deg`.

Stability note:

- `FINAL_RECENTER_DURATION_S` and `FINAL_CENTER_HOVER_MAX_S` were increased so
  the final proof can converge instead of aborting early during a healthy
  trend;
- weak/noisy first validation pulses now retry with the scheduled larger
  excitation instead of immediately rejecting the candidate.

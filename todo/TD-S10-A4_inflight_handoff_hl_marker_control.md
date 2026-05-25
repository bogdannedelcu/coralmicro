# TD-S10-A4 - In-Flight Handoff To Estimator-Backed Marker Control

## Goal

Define a fail-safe in-flight handoff process for the Crazyflie: start from a
previously persisted `sentai.calib` calibration artifact, use WhyCon/PnP to
anchor the drone pose over the marker pad, switch from bootstrap RPYT control
to a Crazyflie estimator-backed commander, then prove stable control over the
marker pad with a bounded movement sequence.

The handoff is successful only if the drone remains controlled over the marker
layout after switching commander mode.  The proof sequence is:

1. acquire marker lock and load the saved calibration;
2. feed PnP-derived position into the Crazyflie estimator;
3. switch in flight away from bootstrap RPYT without disarming;
4. hold the same position over the markers after the switch;
5. execute `-X`, `+X`, center, `-Y`, `+Y`, center using the post-handoff
   commander;
6. hover centered over the marker-pad origin;
7. perform a soft landing.

This task depends on A3.  A4 does not rediscover camera orientation; it consumes
the accepted `sentai.calib` output from A3 and validates that the calibrated
pose can drive a higher-level flight controller.

## Problem Statement

The A3 calibration can identify the camera/body mapping and produce a compact
calibration artifact, but the current control path still relies heavily on
bootstrap RPYT commands.  RPYT is useful for early acquisition because it does
not require a trusted position estimator, but it is not ideal for longer
centering, bounded movement, or soft landing once a usable visual pose exists.

Crazyflie estimator-backed command modes can provide smoother velocity or
position behavior, but they rely on the onboard estimator.  A direct in-flight
switch from RPYT to position-style control has been unreliable in prior tests
because the estimator was not necessarily warmed up and aligned to the PnP pose
before the commander handoff.

A4 therefore treats the handoff as a coordinated estimator-plus-commander
transition, not merely as a command API change.

## Scope

A4 specifies the target behavior and acceptance criteria.

B4 implements the first version in MicroPython/REPL inside `sentai_sim`, then
documents what must move into C/C++ after the behavior is validated.

Out of scope for A4:

- rediscovering camera orientation from scratch;
- changing the WhyCon detector;
- using Gazebo ground truth in the mission decision path;
- designing a full navigation stack beyond the bounded marker-pad exercise.

## Required Inputs

- A valid saved `sentai.calib` artifact from A3, containing at minimum:
  - `R_cam_to_body`;
  - marker layout metadata;
  - camera intrinsics and marker diameter;
  - visual response/Jacobian diagnostics if available.
- Live grayscale camera frames.
- Live WhyCon detections and PnP pose derived from the configured marker pad.
- RPYT bootstrap capability for marker acquisition and emergency fallback.
- Crazyflie commander APIs:
  - bootstrap `attitude()` / RPYT;
  - generic hover velocity commands for the active proof;
  - emergency stop/disarm.
- A way to feed PnP-derived position into the Crazyflie estimator, preferably
  position-only external position first.

Gazebo ground truth may be recorded for post-mortem in SIM, but must not decide
handoff success or command generation.

## Control And Frame Conventions

- The marker-pad origin is the camera-control reference after A3: the projected
  marker constellation centroid corresponds to world `(0, 0, 0)` because the
  marker layout is centroid-centered.
- A4 must use the saved A3 calibration to convert camera-frame PnP estimates
  into the drone/body/world convention expected by the estimator or commander.
- During handoff warmup, prefer position-only estimator updates over forcing
  yaw/quaternion unless yaw handling has been explicitly validated.
- The switch must be atomic from the drone safety perspective:
  - RPYT setpoint streaming must stop without disarming;
  - the replacement commander must already be active or begin immediately;
  - no commander watchdog gap may occur.

## Handoff Sequence

1. Preflight.
2. Load and validate the saved `sentai.calib` artifact:
   - layout id matches `_small`;
   - seven-marker layout matches A1/A2/A3 metadata;
   - `R_cam_to_body` is present and valid;
   - marker size and intrinsics match runtime detector setup.
3. Acquire marker lock using the conservative A3-style bootstrap:
   - neutral RPYT thrust-only or low-authority RPYT;
   - all visible markers are quality-gated;
   - no estimator-backed lateral control before pose is trusted.
4. Estimate camera/drone pose from WhyCon/PnP using the saved calibration.
5. Warm up Crazyflie estimator:
   - stream PnP-derived external position at a stable rate;
   - require a stable marker/PnP lock window;
   - require estimator pose, if observed, to be bounded and not diverging.
6. Commander handoff:
   - release bootstrap RPYT without disarming;
   - immediately start the post-handoff commander at the current visual pose;
   - first target is hold-current, not movement.
7. Post-handoff hold proof:
   - stay over the same marker-pad pose for a short stability window;
   - maintain marker lock and estimator/PnP consistency.
8. Marker-pad motion proof:
   - move `-X`;
   - move `+X`;
   - return center;
   - move `-Y`;
   - move `+Y`;
   - return center.
9. Center hover.
10. Soft land using the post-handoff commander while continuing visual/marker
    supervision.

## Commander Strategy

The active A4/B4 commander primitive is Generic Hover:

- `hover(vx, vy, yaw_rate, z)` must be streamed continuously;
- lateral motion is closed in image frame around marker features;
- Z remains metric and estimator-backed, with visual/PnP Z used for sanity;
- every streaming phase has an explicit timeout and safe-stop path.

Position-style command layers may be revisited later, but they are not part of
the active A4 success contract until their estimator interface is validated.

## Success Criteria

A4 success is camera/PnP based, not GT based:

- saved calibration loads and passes metadata validation;
- marker/PnP lock remains valid through estimator warmup;
- in-flight handoff occurs without disarm, watchdog cut, or marker loss;
- after handoff, the drone holds the same marker-relative pose within a
  noise/trend-derived stability gate;
- `-X`, `+X`, center, `-Y`, `+Y`, center are executed in the expected image-frame
  directions; these are marker-image envelope moves, not world-meter targets;
- the non-moving axis remains controlled during each movement;
- final centered hover completes;
- soft landing completes under post-handoff supervision;
- the run writes a compact handoff artifact with calibration id, commander
  mode, estimator warmup statistics, pose residuals, motion proof results, and
  landing status.

In SIM, GT may be used only as a post-mortem forensic check.

## Failure Policy

- If calibration metadata does not match the active marker layout, do not arm.
- If marker/PnP lock is weak, stay in bootstrap recovery or land.
- If estimator warmup diverges, do not switch commander.
- If handoff cannot be made without a command gap, land using the current safe
  path.
- If post-handoff hold fails, return to safe recovery or land; do not execute
  the motion proof.
- If marker lock is lost during the proof sequence, stop movement and land.
- Never persist a successful A4 handoff artifact from a failed run.

## Deliverables

- A4 objective/specification document.
- B4 operational log and implementation plan.
- REPL/SIM experiment proving the handoff.
- Host-side verdict script that summarizes camera/PnP metrics and SIM GT
  post-mortem.
- Later C/C++ migration plan for the validated REPL behavior.

## C/C++ Migration Direction

A4 first validates behavior in REPL/SIM, but the production direction is a C++
runtime task.  The final MicroPython mission should not implement per-frame
vision/control math.  It should load/start/stop runtime subsystems, monitor
compact status, and collect artifacts.  The C++ runtime owns `calib.ini`
loading, marker/flow pumping, estimator feeding, image-frame motion control, and
success/fail state transitions.

The accepted B4 simulator runs, post-mortem camera frames, GT-vs-estimator
plots, and verdict logs are thesis evidence and should remain archived as proof
of the handoff behavior that the C++ task must preserve.

# TD-S10-A3 - Calibrate Takeoff Camera Orientation

## Goal

Define a fail-safe calibration process that discovers the rotation between the
drone body frame and an onboard camera whose mounting orientation is unknown.

The calibration result is a valid camera-to-body rotation matrix, persisted by
`sentai.calib`, so later marker pose, hold, VPE, and mission code can interpret
camera measurements in the drone body frame.

The persisted calibration artifact must include more than the rotation matrix.
At the end of a successful run, `sentai.calib` should write a compact
calibration file on the drone filesystem containing the accepted matrix plus
the empirically discovered run parameters needed to reuse or audit the result:
local image-response/Jacobian estimate, measured feature-noise statistics,
validated axis/sign evidence, confidence metrics, safe command envelopes used
during validation, marker-layout metadata version, and timestamp/run id.

This task is a flight-control calibration task, not an offline dataset task.
It must run in `sentai_sim` first and remain portable to the real Crazyflie
runtime.

The final target is a complete one-run `sentai.calib` calibration.  The
operator should not manually provide the camera orientation, axis signs,
response angles, fitted constants, or per-run tuning results.  The system may
use conservative safety envelopes, marker layout metadata, camera intrinsics,
and marker size, but the camera/body mapping and usable flight response must
be discovered online from the same flight.

## Problem Statement

At the start of the calibration we do not know how the camera axes map to the
drone body axes.  The camera may be rotated by 90-degree steps or have axis
signs inverted.  Because of that:

- camera-frame X/Y/Z must not be trusted as drone-body X/Y/Z;
- lateral position hold must not assume a body/camera mapping;
- takeoff cannot rely on a visual Z estimate until WhyCon markers enter the
  camera field of view;
- any wrong assumption can drive the drone out of the marker pad FOV or into
  an unsafe state.

The calibration must therefore start conservatively, acquire markers, stabilize
visual Z, and only then run small complementary body-axis excitations to infer
axis mapping and signs from image feedback.

The experiments in B3 are scaffolding for this autonomous process.  They may
measure intermediate quantities, but the final A3 implementation must fold the
sequence into one continuous runtime flow:

- takeoff until the calibration marker band is found;
- stabilize the visual calibration band;
- excite axes and estimate response online;
- choose and validate the camera/body rotation candidate;
- persist accepted calibration or fail closed.

## Inputs

Allowed runtime inputs:

- grayscale frames from the onboard/downward camera;
- WhyCon marker detections derived from those frames;
- known marker layout and marker size for the `_small` WhyCon world;
- RPYT commands issued by the calibration routine;
- minimal command/link/safety health needed to abort or land.

The calibration decision path must not use Gazebo ground truth.

Manual constants must be limited to physical metadata and safety envelopes.
Do not encode discovered axis directions, camera orientation, sign mapping, or
per-run response angles as configuration.  They are outputs of calibration,
not inputs.

Image-space constants must also not be treated as calibration truth.  Pixel
thresholds such as response magnitude, deadband, return tolerance, hover
tolerance, or centering success must be determined online from measured visual
noise, feature stability, FOV margin, Jacobian conditioning, and convergence
trend.  Fixed pixel values may appear only as conservative safety-envelope
floors/caps and must be logged as such, not as evidence that calibration is
correct.

Crazyflie IMU/EKF/yaw telemetry may be logged for post-mortem or used as a
conservative safety guard, but it must not decide the camera-to-body rotation.
The purpose of A3 is to discover that mapping from commanded motion and visual
response.

## Known Scene Convention

Use the configured Gazebo `_small` WhyCon world.  The runtime must assert the
exact marker layout before calibration starts.

Canonical small-pad metadata, consistent with A1/A2:

- image resolution: `320x240`;
- camera intrinsics: `fx=288.3`, `fy=288.3`, `cx=160.0`, `cy=120.0`;
- marker outer diameter: `0.0544 m`;
- marker coordinates, centered so the mean XY marker centroid is `(0, 0)`:
  - `NW = (-0.082857143, +0.065714286, 0.005)`
  - `NE = (+0.077142857, +0.065714286, 0.005)`
  - `W  = (-0.062857143, -0.014285714, 0.005)`
  - `E  = (+0.057142857, -0.014285714, 0.005)`
  - `SW = (-0.082857143, -0.094285714, 0.005)`
  - `SE = (+0.077142857, -0.094285714, 0.005)`
  - `N  = (+0.017142857, +0.085714286, 0.005)`.

Important visibility convention:

- Standard calibration decisions count only fully visible markers.
- A marker is fully visible only when its projected outer circle is completely
  inside the image with configured margin.
- Cropped/partial markers may be logged, but they do not satisfy marker-lock
  or pose-quality gates.
- For the seven-marker `_small` world, the first takeoff/acquisition phase
  should continue until all 7 markers are fully visible for at least 10
  consecutive frames.  This establishes a clean calibration band before any
  visual-Z hold or lateral axis excitation begins.

## SOTA Anchors

This task should be framed as a conservative, uncalibrated visual-servoing and
camera/body calibration problem, specialized for a small quadrotor and a
WhyCon marker pad.

Relevant literature and practical takeaways:

- Hutchinson, Hager, and Corke, "A Tutorial on Visual Servo Control",
  IEEE Transactions on Robotics and Automation, 1996,
  `https://doi.org/10.1109/70.538972`.
  Abstract essence: visual servoing uses computer-vision measurements inside
  the robot feedback loop and separates the field into image-based and
  position-based approaches.
  A3 takeaway: treat marker image features as control feedback, explicitly
  model coordinate frames, and be careful when switching from image-space
  control to pose/body-frame control.

- Chaumette and Hutchinson, "Visual Servo Control. I. Basic Approaches",
  IEEE Robotics and Automation Magazine, 2006,
  `https://doi.org/10.1109/MRA.2006.250573`.
  Abstract essence: the paper formalizes the visual-servo problem, explains
  image-based visual servoing (IBVS) and position-based visual servoing
  (PBVS), and discusses performance/stability issues.
  A3 takeaway: start with IBVS-like quantities while extrinsics are unknown
  (centroid, scale, FOV margin), then move toward PBVS-like pose only after
  the camera/body transform is accepted.

- Hosoda and Asada, "Versatile Visual Servoing without Knowledge of True
  Jacobian", IROS 1994, and later uncalibrated IBVS work such as
  "Uncalibrated visual servoing using the fundamental matrix",
  Robotics and Autonomous Systems, 2009,
  `https://doi.org/10.1016/j.robot.2008.04.002`.
  Abstract essence: uncalibrated visual servoing estimates an image Jacobian
  online, relating actuator motion to image-feature motion, instead of
  requiring perfect camera/robot calibration in advance.
  A3 takeaway: the lateral calibration should estimate the local response
  from small commanded RPYT perturbations and observed marker-image deltas,
  not assume the response matrix before measuring it.

- Krajnik et al., "External Localization System for Mobile Robotics",
  ICAR 2013, and Krajnik et al., "A Practical Multirobot Localization System",
  Journal of Intelligent and Robotic Systems, 2014.
  Abstract essence: WhyCon-style black/white circular planar markers provide a
  fast, practical localization system suitable for mobile robots, with source
  code and experiments validating precision and runtime behavior.
  A3 takeaway: use WhyCon as the lightweight perception primitive, but keep
  marker detection, marker-count stability, and pose reliability as explicit
  gates before feeding control.

- Nitsche et al., "WhyCon: An Efficient, Marker-based Localization System",
  IROS Open Source Aerial Robotics Workshop, 2015.
  Abstract essence: WhyCon focuses on precise, fast, reliable, easy-to-use
  localization from circular markers and is designed for efficient deployment.
  A3 takeaway: prefer the documented concentric-marker geometry and robust
  count/centroid/scale extraction over ad-hoc visual cues.

- Kim, Lee, and Kim, "Image Based Visual Servoing for an Autonomous Quadrotor
  with Adaptive Backstepping Control", ICCAS 2011.
  Abstract essence: the quadrotor receives image-feature errors through an
  image Jacobian, while the flight controller tracks the resulting commands
  under uncertain dynamics.
  A3 takeaway: quadrotor vision control must respect underactuation and
  dynamics.  A3 should use small bounded RPYT pulses, wait for settling, and
  reject aggressive corrections until the visual response is known.

Methodology distilled from these sources:

- separate image-space stabilization from pose/body-frame control;
- estimate the local visual response before closing lateral feedback;
- estimate visual-feature noise online and normalize gating decisions by that
  measured uncertainty instead of using absolute pixel thresholds;
- adapt excitation amplitude/gain when the response is coherent but not yet
  observable above the current noise floor;
- use FOV margin, marker visibility, command saturation, and sustained
  divergence as safety conditions rather than hard-coded pixel-error gates;
- keep target features inside the image throughout every excitation;
- use small complementary motions to cancel drift and identify signs;
- reject low-confidence Jacobian/orientation estimates instead of forcing a
  calibration.

## Calibration Principles

- Fail closed: uncertain calibration must land/recover and must not persist a
  new matrix.
- One run must be self-contained: no manual interpretation between phases, no
  hand-entered axis signs, and no preselected response mapping from prior
  experiments.
- Prefer image-space features during the unknown-orientation phase: marker
  count, full-visibility margin, marker constellation centroid, marker scale,
  and filtered visual Z.
- Stabilize Z before lateral calibration.  Do not start axis identification
  while marker scale/Z is still drifting.
- Use small, bounded RPYT excitations.  Every positive lateral excitation must
  be paired with a complementary negative excitation so drift cancels.
- Keep the marker constellation inside the camera FOV throughout the process.
- Infer axes and signs from observed response, not from assumed camera mount
  orientation.
- During calibration, centering, hover, and landing, use camera coordinates as
  the control reference.  The target is not the drone body origin; it is the
  camera optical axis over the marker-pad origin.  Therefore the closed loop
  drives the projected marker-pad centroid to the image center.
- Use enough repeated samples to reject noisy, ambiguous, or inconsistent
  responses.
- Avoid magic pixel constants.  Deadbands, response gates, hover/landing
  acceptance, and validation confidence should be derived from the current
  flight's feature-noise estimate, repeated-excitation consistency, and
  closed-loop convergence.  If a fixed bound remains, it must be justified as a
  physical/safety envelope and not as a tuning result required for success.
- Persist only after validation motions confirm the selected mapping.
- Persist the accepted calibration as a compact FS artifact containing:
  - `R_cam_to_body`;
  - local command-to-image response/Jacobian estimate;
  - measured visual-noise statistics used for gates/deadbands;
  - accepted excitation amplitudes and confidence metrics;
  - marker layout/intrinsics/diameter metadata version;
  - safety envelope summary and validation status.
  Detailed per-frame forensic data remains in the append-only journal, not in
  the persisted calibration artifact.

## Required Calibration Sequence

1. Preflight.
2. Thrust-only takeoff until enough fully visible markers are detected.
3. Stable marker lock: for the current seven-marker pad, 7 fully visible
   markers for 10 consecutive frames.
4. Vision-only Z calibration and Z stabilization.
5. Climb or settle to a calibration scale/FOV band.
6. Complementary X/Y body-axis excitations.
7. Orientation candidate inference.
8. Validation motions with smaller or equal command magnitudes.
9. Commit and persist accepted `R_cam_to_body`.
10. Run a vision-only IBVS-style return-to-center phase:
    - use the measured image response/Jacobian from the current run;
    - drive the marker-pad centroid to the image center, which corresponds to
      hovering the camera optical axis above world `(0, 0, 0)` because the
      marker layout is centroid-centered;
    - keep visual-Z hold active while correcting XY.
11. Hold above the centered pad for a short stability window.
12. Run a bounded image-only axis-envelope survey before landing:
    - move along calibrated camera/body X toward the positive visual envelope
      until marker geometry reaches the image-margin condition;
    - return through center, then repeat toward the negative X envelope;
    - return to center and repeat the same positive/negative envelope survey
      for Y;
    - all envelope limits are detected from camera observations only, using the
      projected marker centers/radii and image bounds, not simulator GT.
13. Hover above the centered pad again after the envelope survey.
14. Land or hand off only after the hover-above-centroid proof succeeds.
    Landing should descend slowly while continuing the same center-hold visual
    servo loop until the configured near-ground/disarm condition is reached.

Any phase may abort to a safe recovery/landing path.

## Centered Landing Requirement

The successful landing proof is camera-referenced:

- after orientation calibration, the drone must hover with the marker-pad
  centroid close to the image center;
- descent must keep running the same camera-based center-hold loop;
- vertical descent should be closed-loop too: use visual `z_cam`/marker scale
  to maintain a bounded, approximately constant descent rate instead of a
  blind thrust ramp;
- disarm/land success is defined only by camera-observable evidence: keep the
  center-hold loop active throughout descent and disarm only when the smoothed
  fully-visible marker count reaches the configured near-ground threshold
  (`avg_full_markers <= 4` over the current 10-frame window);
- if lateral centering loses authority during descent, the descent controller
  should delay downward motion and continue centering before proceeding;
- Gazebo body-frame ground truth is not part of the success criterion.  It may
  be used only after simulator runs for forensic analysis.

## Orientation Inference

The candidate set should cover valid discrete camera/body rotations:

- axis permutations;
- axis sign flips;
- determinant `+1` rotations only;
- 90-degree-step camera mounting assumptions where applicable.

For every candidate, score how well it explains the observed response to
commanded body-axis perturbations.  The accepted candidate must have:

- determinant close to `+1`;
- low residual between predicted and observed response;
- a clear margin over the second-best candidate;
- consistent signs across repeated positive/negative pulse pairs;
- sufficient valid samples per axis;
- no safety aborts during sampling.

Ambiguous or low-margin runs must not persist calibration.

The accepted candidate must be computed from observations gathered during the
current run.  Previous run data and B3 smoke-test conclusions may guide
implementation, thresholds, and safety envelopes, but must not be substituted
for live evidence.

## Safety Requirements

Hard aborts:

- no marker lock after takeoff timeout;
- marker count below the configured fully-visible minimum for too long;
- marker scale/Z is unstable or jumps beyond limits;
- marker constellation approaches image boundary;
- RPYT commands exceed configured bounds;
- thrust exceeds configured envelope;
- command watchdog is not refreshed;
- Crazyflie link or simulator safety reports failure;
- operator abort request.

Soft reject-with-land:

- X/Y response vectors are not distinguishable;
- complementary pulses do not cancel drift;
- candidate orientation has weak margin;
- observed cross-axis coupling exceeds threshold;
- validation motion does not move the image response in the expected direction.

Rejected calibration must restore the previous persisted orientation, if any,
and must not mark `sentai.calib` as calibrated.

## Data And Artifacts

Each run should write artifacts under a task-specific folder:

```text
dataset/
  TD-S10-B3/
    calib_takeoff_YYYYMMDD_HHMMSS/
      journal.txt
      summary.json
      samples.jsonl
      phase_trace.jsonl
      overlays/
      plots/
```

Required `summary.json` fields:

- world path and marker layout;
- camera intrinsics and marker size;
- RPYT command limits;
- takeoff acquisition time;
- marker-lock timestamp;
- marker count and visibility distribution;
- selected orientation candidate and candidate ranking;
- validation result;
- accept/reject status and reason;
- persistence status;
- landing/cleanup status.

The artifacts should allow a reviewer to reproduce the online decision:
which observations were used, which candidates were considered, why the
winner was accepted or rejected, and whether all required validation motions
passed in that same run.

The accepted B3 simulator runs, camera frames, journals, plots, and strict
`calib.ini` artifacts are thesis evidence.  They must be retained as
post-mortem proof, not treated as disposable smoke-test output.

Required per-sample data:

- timestamp and phase;
- RPYT command;
- marker count and full-visibility count;
- image-space centroid and scale;
- visual Z estimate and filtered derivative;
- available pose estimate and quality;
- response delta used for candidate scoring.

Host-side Gazebo ground truth may be recorded only for post-mortem plots and
reports.  It must never be fed into the runtime decision path.

## Acceptance Criteria

Minimum SIM acceptance:

- calibration runs in `sentai_sim` with Gazebo GUI visible;
- runtime perception uses only camera frames and allowed metadata;
- no Gazebo ground truth is consumed by the calibration decision path;
- takeoff-until-markers acquires the configured minimum fully visible markers;
- visual Z stabilizes before lateral axis identification begins;
- complementary X/Y excitations remain within safe FOV and command limits;
- the algorithm selects a determinant `+1` orientation with clear candidate
  margin;
- validation motions confirm the selected axis signs;
- rejected or ambiguous runs do not persist calibration;
- accepted runs persist calibration through `sentai.calib`;
- the mission lands or safely hands off after calibration;
- artifacts are written under `dataset/TD-S10-B3/`.

Research/report acceptance:

- per-run tables show acquisition time, marker visibility, selected rotation,
  candidate residuals, validation drift, and accept/reject reason;
- failure cases are classified, not hidden;
- host-side GT comparison is used only after the run to quantify accuracy.

## Open Questions

- Which `_small` layout is canonical for A3: six symmetric markers or the
  seven-marker asymmetric variant?
- What raw thrust envelope is safe in SIM and later on hardware?
- What minimum number of fully visible markers should be required for marker
  lock and for each later phase?
- What candidate-margin threshold is sufficient to persist calibration?
- Should yaw/IMU be allowed as a safety veto, or only as post-mortem telemetry?

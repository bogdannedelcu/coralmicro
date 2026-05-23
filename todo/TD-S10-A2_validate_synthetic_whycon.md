# TD-S10 — Validate WhyCon on Synthetic Gazebo Dataset

## Goal

Validate WhyCon detection and pose estimation on a synthetic Gazebo dataset by
comparing detector outputs against `manifest.jsonl`.  The validation must be
implementation-agnostic and should support at least two detector backends:

- a local OpenCV-based reference implementation;
- the `sentai_sim` C++ implementation, which runs in the same FreeRTOS-like
  simulation environment as the embedded stack.

The purpose is to separate detector failure from PnP/pose/control failure.
The concrete validation objective is an ablation-style comparison between the
local OpenCV reference backend and the `sentai_sim` WhyCon implementation on
the same synthetic frames.  The comparison must be per-frame and per-marker
where possible, so we can tell whether failures come from dataset difficulty,
parameter choices, or implementation-specific behavior.

The final target is a complete ablation over every measured variable exposed
by both algorithms.  For every frame, compare OpenCV and `sentai_sim` on marker
count, matched markers, centroid, radius/axes, orientation, and estimated
camera pose relative to the dataset ground/world origin `(0, 0, 0)`.  Then
aggregate those per-variable differences into summary statistics and compact
tables that can be used directly in a research report.

Important methodology constraint: the OpenCV and `sentai_sim` paths should be
two implementations of the same detector algorithm, exercised on the same
images, camera metadata, marker geometry, and matching/evaluation rules.  Do
not tune the `sentai_sim` path merely to reach similar aggregate performance.
If the two paths diverge, treat the divergence as a porting difference,
parameter/specification mismatch, or algorithmic question to investigate.
Corrections should move the two implementations toward the same documented
algorithmic behavior, not toward coincident headline metrics.

When the intended detector behavior is unclear, consult the relevant OpenCV
documentation, source behavior, and established WhyCon / concentric-marker
literature before changing either implementation.  Record the chosen algorithm
and parameter semantics explicitly so the comparison remains reproducible and
defensible as a research result.

This task validates images and JSON produced by
`TD-S10_create_synthetic_dataset_whycon.md`.

## Inputs

Expected raw dataset layout:

```text
dataset/
  TD-S10-B1/
    whycon_gazebo_synth_YYYYMMDD_HHMMSS/
      manifest.jsonl
      config.json
      frames/
        *.pgm
```

Validation should not depend on a running Gazebo instance.  It consumes only
the dataset directory, images, `manifest.jsonl`, and `config.json`.

## Output

Write results into a task-specific validation artifacts folder, not mixed with
the raw dataset producer output:

```text
dataset/
  TD-S10-B2/
    whycon_gazebo_synth_YYYYMMDD_HHMMSS/
      validation_YYYYMMDD_HHMMSS/
        results.jsonl
        summary.json
        failures.csv
        overlays/
          optional_frame_000123.png
```

Each validation run should record the source dataset path in `summary.json`.
Do not overwrite earlier validation runs by default.

## Validation Backends

The validation procedure should compare results from multiple implementations
without making the dataset depend on either of them.

### OpenCV Reference

The OpenCV implementation is the local reference path, but it must not be an
ad-hoc detector invented only for this task.  It should implement, using OpenCV
image-processing primitives, the documented WhyCon family of algorithms from
the circular black/white marker literature.

Reference algorithm family:

- T. Krajnik et al., "External Localization System for Mobile Robotics",
  ICAR 2013.
- T. Krajnik et al., "A Practical Multirobot Localization System", Journal of
  Intelligent and Robotic Systems, 2014.
- M. Nitsche et al., "WhyCon: An Efficient, Marker-based Localization System",
  IROS Open Source Aerial Robotics Workshop, 2015.
- The open-source `lrse/whycon` implementation and documentation.

OpenCV should be understood here as the implementation substrate for common
operations such as thresholding, connected components, contour/ellipse
geometry, and PnP.  OpenCV itself does not provide an official `cv2.whycon`
module.  Ground truth remains the dataset manifest, not OpenCV output.

Record the OpenCV version and all relevant detector parameters in the
validation output, including the specific literature-backed algorithm variant
and any deviations from the referenced WhyCon method.

### `sentai_sim` C++ Implementation

The `sentai_sim` implementation is the production-candidate path.  It is
expected to use the C++ WhyCon logic that runs in the simulated FreeRTOS-like
environment.

Record the build identifier, git commit if available, and all relevant detector
parameters in the validation output.

### Shared Configuration

Both backends should be configured from the dataset metadata:

- backend: `whycon`
- camera intrinsics from `config.json`
- marker size from `config.json`
- marker count and marker world coordinates from `config.json` /
  `manifest.jsonl`; do not hardcode the old 6-marker layout in either
  backend or validator.
- same image resolution as the frame file
- same grayscale pixel values
- explicit threshold/concentric/shape-filter settings

If an implementation cannot expose all intermediate fields, the validator
should still report the fields it can observe and mark unavailable fields
explicitly.

### Algorithmic Parity

The ablation is valid only if the compared backends implement the same intended
detector semantics.  OpenCV is the reference implementation environment for
the baseline, while `sentai_sim` is the embedded/FreeRTOS implementation of
that same baseline.  Differences introduced by embedded constraints must be
documented and justified.

Required parity items:

- same grayscale input pixels;
- same thresholding family and threshold parameters;
- same contour/component connectivity assumptions;
- same marker-structure test, for example concentric ring/dot hierarchy;
- same radius/area/circularity/axis/center-offset acceptance gates;
- same duplicate suppression strategy;
- same camera intrinsics, marker diameter, and pose-estimation convention;
- same registered marker layout, including any asymmetric marker added to
  break pose ambiguity;
- same matching/evaluation thresholds in the external validator.

Non-goals:

- Do not disable a validation gate only to increase recall.
- Do not relax `sentai_sim` thresholds until its aggregate recall resembles
  OpenCV unless that relaxation is part of the documented reference algorithm.
- Do not compare two different detector algorithms as if they were an
  implementation ablation.  If a different algorithm is evaluated, label it as
  a separate backend/variant.

## Per-Frame Result Schema

Write one JSON object per frame to `results.jsonl`:

```json
{
  "frame_id": 0,
  "image_path": "frames/frame_000000_x+0.000_y+0.000_z0.500_r+0.0_p+0.0_yaw000_n6.pgm",
  "expected_count": 6,
  "backends": {
    "opencv": {
      "detected_count": 6,
      "matched_count": 6,
      "false_positive_count": 0,
      "missed_count": 0,
      "matches": [
        {
          "marker_id": "NW",
          "expected_pixel": [82.4, 57.1],
          "detected_pixel": [82.9, 56.8],
          "centroid_error_px": 0.58,
          "expected_radius_outer": 31.2,
          "detected_axis_a": 31.8,
          "detected_axis_b": 30.9,
          "axis_ratio": 1.03
        }
      ],
      "pose_eval": {
        "available": true,
        "camera_pose_world_est": {
          "x": 0.001,
          "y": -0.002,
          "z": 0.508,
          "roll_deg": 0.4,
          "pitch_deg": -0.7,
          "yaw_deg": 44.2
        },
        "camera_pose_world_gt": {
          "x": 0.0,
          "y": 0.0,
          "z": 0.5,
          "roll_deg": 0.0,
          "pitch_deg": 0.0,
          "yaw_deg": 45.0
        },
        "x_error_m": 0.002,
        "y_error_m": 0.003,
        "z_error_m": 0.008,
        "yaw_error_deg": 1.2
      }
    },
    "sentai_sim": {
      "detected_count": 6,
      "matched_count": 6,
      "false_positive_count": 0,
      "missed_count": 0
    }
  },
  "backend_delta": {
    "count_delta": 0,
    "matched_delta": 0,
    "marker_deltas": [
      {
        "marker_id": "NW",
        "centroid_delta_px": 0.42,
        "axis_a_delta_px": -0.3,
        "axis_b_delta_px": 0.2,
        "angle_delta_deg": 1.1
      }
    ],
    "pose_delta": {
      "available": true,
      "translation_delta_m": 0.012,
      "x_delta_m": 0.004,
      "y_delta_m": -0.006,
      "z_delta_m": 0.009,
      "roll_delta_deg": 0.3,
      "pitch_delta_deg": 0.5,
      "yaw_delta_deg": 1.1
    }
  },
  "status": "pass"
}
```

Use `status` values such as:

- `pass`
- `missed_marker`
- `false_positive`
- `centroid_error`
- `pose_error`
- `detector_error`
- `manifest_error`

## Matching Strategy

WhyCon detections may not have stable IDs.  For each backend, match detections
to ground truth evaluation-visible markers by 2D pixel distance.

Visibility convention:

- Standard A2 metrics count only manifest markers with
  `visibility_reason == "visible"`, meaning the full projected outer marker
  circle is inside the image bounds.
- Manifest markers with `visibility_reason == "partial_crop"` are excluded
  from default expected counts, recall, complete-frame success, centroid/scale
  error, and pose evaluation.
- Detector outputs that match a `partial_crop` manifest marker should be
  ignored in default metrics.  They are neither true positives nor false
  positives.  Count them separately as ignored/stress detections when useful.
- Cropped-marker recovery may be evaluated as a separate stress/variant metric,
  but it must be labeled explicitly and must not be mixed into the default
  paper-baseline ablation.

1. Load evaluation-visible markers from the manifest.
2. Load detector outputs: centroid, axes, angle, pose if available.
3. Build a distance matrix between expected centers and detected centers.
4. Assign pairs using greedy nearest-neighbor for small N, or Hungarian
   matching when available.
5. Reject a match if distance is above a threshold.

Recommended threshold:

- Start with `max(5 px, 0.25 * expected_pixel_radius_outer)`.
- Record the threshold used in `summary.json`.

Do not require detector order to match manifest order.

## Metrics

Report metrics globally and grouped by:

- `scene_id`
- resolution
- Z stratum
- yaw bins, e.g. 45 degree bins
- roll/pitch magnitude bins
- expected marker count
- edge/crop visibility category

Required summary metrics:

- frame count
- detection recall per backend: matched evaluation-visible markers / expected
  evaluation-visible markers
- false positives per frame per backend
- complete-frame success rate per backend: detected all expected markers and
  no false positives
- centroid error mean/median/p95/max in pixels per backend
- radius/axis error mean/median/p95 per backend if expected radius exists
- pose X/Y/Z/Yaw error per backend if pose estimation is exercised
- pose roll/pitch/yaw error per backend when orientation is estimated
- pose estimates must be quality-gated by reprojection error.  A frame with
  enough detections but a high reprojection-error pose should be reported as
  pose-unavailable/degenerate, not as a valid camera position.  This prevents
  coplanar or weak-geometry PnP failures from polluting X/Y/Z statistics.
- pose-ambiguity diagnostics per backend: after the primary pose estimate,
  search for alternate planar-PnP solutions and alternate marker-detector
  correspondences that produce comparable reprojection error but a materially
  different camera pose.  This is required because circular WhyCon markers are
  individually symmetric and a symmetric marker layout can make wrong
  correspondences look geometrically plausible.
- per-frame OpenCV vs `sentai_sim` deltas for every shared measured variable:
  count, matched count, centroid, radius, axis_a, axis_b, angle, pose X/Y/Z,
  pose roll/pitch/yaw, and translation norm
- angle deltas should be reported in two forms:
  - all matched markers, as a diagnostic;
  - a stable-angle subset, as the formal angle-parity metric, using a
    documented minimum eccentricity or axis-ratio threshold such as
    `axis_a / axis_b >= 1.10` in both backends.  Near-circular detections have
    weakly conditioned major-axis orientation, so all-marker angle p95 must
    not be used alone to reject detector parity.
- aggregate OpenCV vs `sentai_sim` statistics for each measured variable:
  mean, median, p95, max, MAE, bias, and RMSE where meaningful
- OpenCV vs `sentai_sim` disagreement rate
- OpenCV vs `sentai_sim` centroid delta on mutually matched detections
- count of frames that failed due to manifest/image read errors

Required report tables:

- Backend detection summary table: backend, frames, expected markers, matched
  markers, recall, false positives per frame, complete-frame success, centroid
  p95.
- Backend geometry summary table: backend, radius/axis error mean/median/p95,
  axis ratio distribution, orientation error mean/median/p95 where available.
- Backend pose-vs-ground-truth table: backend, pose-valid frames, translation
  RMSE, translation p95, X/Y/Z MAE, roll/pitch/yaw MAE, yaw p95, and count of
  frames with near-ambiguous alternate pose/correspondence solutions.
- Pose ambiguity table: frame, backend, primary reprojection error, alternate
  solution type, alternate reprojection error, translation delta, yaw delta,
  and marker assignment for any near-ambiguous solution.
- OpenCV-vs-`sentai_sim` ablation table: comparable frames, count disagreement
  rate, matched-marker disagreement rate, centroid delta p95, axis/radius
  delta p95, all-marker angle delta p95, stable-angle delta p95, translation
  delta mean/p95/max, yaw delta mean/p95/max.
- Failure-mode table grouped by z stratum, expected marker count, and
  edge/crop category.

The key diagnostic split:

- Count/recall failure in both backends suggests dataset difficulty,
  thresholding, component filtering, or concentric validation needs attention.
- OpenCV passes but `sentai_sim` fails suggests an implementation or parameter
  mismatch in the C++ path.
- `sentai_sim` passes but OpenCV fails suggests the reference implementation is
  incomplete or configured differently.
- Good count but high centroid error points to ellipse fitting, blob geometry,
  or perspective bias.
- Good centroid but bad Z/pose points to PnP, camera model, marker-size, or
  correspondence logic.
- Good offline pose but bad flight behavior points away from WhyCon and toward
  control, EKF, CRTP, or handoff.

## Ablation Outputs

The validator should make OpenCV vs `sentai_sim` comparison easy to inspect:

- one shared `results.jsonl` row per frame containing both backend outputs;
- per-frame backend disagreement fields, including count disagreement,
  matched-marker disagreement, centroid delta, axis/radius delta, and pose
  delta when available;
- per-frame camera pose estimates from both backends, expressed in the same
  world frame as manifest `pose_world_camera`, with origin `(0, 0, 0)` at the
  ground reference point;
- visual overlays that can show OpenCV and `sentai_sim` detections separately
  and, when useful, together on the same source frame;
- summary groups that report where the two backends diverge by z stratum,
  expected marker count, and edge/crop category.
- `report_tables/` or equivalent CSV/Markdown outputs containing the compact
  ablation tables listed in the metrics section.

The OpenCV backend is not ground truth.  It is an ablation baseline.  Ground
truth remains the manifest.

## Failure Artifacts

For failed or near-threshold frames, generate optional overlay images showing:

- expected marker centers and IDs
- detected centers
- match lines
- false positives
- missed markers

Keep overlays optional or capped, for example first 100 failures, so validation
does not explode disk usage.

Write `failures.csv` with:

```csv
frame_id,image_path,backend,status,expected_count,detected_count,matched_count,max_centroid_error_px,z_m,roll_deg,pitch_deg,yaw_deg
```

## Acceptance Criteria

- The validator reads `config.json`, `manifest.jsonl`, and every referenced
  image successfully.
- It evaluates the OpenCV reference and `sentai_sim` C++ implementation, or
  clearly marks a backend as unavailable in `summary.json`.
- It writes `results.jsonl`, `summary.json`, and `failures.csv`.
- It does not assume manifest marker order equals detector order.
- It reports separate count, centroid, pose, and backend-disagreement metrics.
- It reports per-frame deltas between OpenCV and `sentai_sim` for every shared
  measured variable.
- It reads the marker layout from dataset metadata, including marker count,
  marker diameter, and world coordinates, so datasets with 6 or 7 markers are
  evaluated with the correct geometry.
- It reports aggregate statistics for every shared measured variable and
  produces compact tables suitable for research reporting.
- It exits non-zero on malformed datasets or validation infrastructure crashes.
- It exits zero if validation completed, even if some frames are detection
  failures.  Detection quality is reported in `summary.json`, not treated as a
  process crash.

## Suggested Quality Gates

Initial gates should be advisory until the dataset distribution is stable:

- complete-frame success rate >= 95% on non-edge frames
- marker recall >= 98% on non-edge frames
- false positives <= 0.02 per frame
- centroid p95 <= 3 px on non-edge frames
- Z p95 <= 3 cm if pose evaluation is enabled
- OpenCV vs `sentai_sim` complete-frame decision agreement >= 98% on non-edge
  frames

Use stricter gates only after confirming the dataset images match the real
flight camera bridge closely.

## Open Questions

- Which detector parameters must be normalized between OpenCV and
  `sentai_sim` before comparing them?
- Which intermediate fields can `sentai_sim` expose without changing detector
  behavior?
- Should pose validation be a separate phase after raw detection validation,
  or should both always run together?

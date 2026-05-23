# TD-S10-B2 - Implement A2 Synthetic WhyCon Validation

## Purpose

Operational implementation log for `TD-S10-A2_validate_synthetic_whycon.md`.
A2 remains the validation objective/specification document; B2 tracks concrete
decisions, implementation steps, commands, progress, and blockers.

B2 is not complete after the OpenCV baseline.  Its actual goal is an
ablation-style, frame-by-frame comparison between:

- `backends.opencv`: local reference detector;
- `backends.sentai_sim`: production-candidate WhyCon implementation.

Full ablation has two phases:

- Phase 1: marker geometry parity: count, centroid, fitted circle/ellipse
  axes, and orientation.
- Phase 2: camera pose parity: estimate the camera pose relative to the
  ground/world origin `(0, 0, 0)` from detected WhyCon markers and compare it
  to `manifest.jsonl` `pose_world_camera`.

## Current Decision

Start by building an OpenCV baseline backend before wiring the `sentai_sim`
C++ backend.

Reasoning:

- OpenCV is local, inspectable, and faster to iterate than the FreeRTOS-like
  simulation path.
- The OpenCV path gives a transparent reference for thresholding, contour
  extraction, ellipse fitting, and centroid matching.
- Ground truth remains `manifest.jsonl`; OpenCV is a baseline detector, not
  truth.
- Once OpenCV produces useful metrics and overlays, the same validator schema
  can accept `sentai_sim` outputs and report backend disagreement.

Validation should proceed in two phases:

1. Detection geometry only: count markers, estimate center, circle/ellipse
   size, and orientation/axis where available.  Generate visual overlays with
   the detected center and circle/ellipse drawn in red over the original image.
2. Pose/PnP: only after phase 1 looks sane by metrics and visual inspection.
   The pose target is the camera pose relative to the ground origin `(0, 0, 0)`.
   Both OpenCV and `sentai_sim` should use the same marker world coordinates
   from the manifest for this comparison.

## Target Dataset For First Baseline

Use the latest visually inspected B1 smoke set:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439/
```

Known properties:

```text
frame_count          30
marker_distribution {4: 12, 5: 8, 6: 10}
z_distribution       {0.3: 8, 0.5: 8, 0.75: 7, 1.0: 7}
```

## Environment

The system Python currently does not have OpenCV installed.  There are several
venv-like folders in the workspace, so validation commands should be explicit
about the interpreter.

Use the local project venv:

```bash
./venv/bin/python
```

Verified environment check:

```text
./venv/bin/python       cv2 4.13.0
./venv-coral/bin/python no cv2: ModuleNotFoundError
./venv_coral/bin/python missing
./venv_edgetpu/bin/python missing
```

Therefore the OpenCV baseline should be run with `./venv/bin/python`, not with
plain `python3` and not with `venv-coral`.

## Implementation Plan

1. Create a validator script that consumes only:
   - dataset directory;
   - `config.json`;
   - `manifest.jsonl`;
   - `frames/*.pgm`.
2. Implement an OpenCV backend first:
   - load grayscale PGM without changing pixel values;
   - threshold dark marker regions against the lighter ground;
   - find contours / connected components;
   - identify WhyCon-like concentric structures;
   - estimate center, circle/ellipse axes, and ellipse orientation for each
     detected marker where available.
3. Match detections to manifest visible markers by 2D centroid distance.
4. Write validation outputs under the B2 artifacts directory:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/
  validation_YYYYMMDD_HHMMSS/
    results.jsonl
    summary.json
    failures.csv
    overlays/
```

5. Generate overlays for failures and near-threshold cases.
6. Generate detection-inspection overlays for all frames, with OpenCV detected
   centers, circles/ellipses, and orientation axes drawn in red.
7. Add a phase-2 pose solver that uses matched marker detections plus manifest
   world coordinates to estimate camera pose relative to `(0, 0, 0)`.
8. Keep `sentai_sim` backend unavailable in `summary.json` until the C++ path
   is wired in.

## OpenCV Baseline Strategy

First pass should optimize for diagnosability rather than cleverness:

- Use simple thresholding plus contour hierarchy to find dark ring / dark
  center relationships.
- Prefer stable, recorded parameters over auto-magic:
  - threshold mode and value;
  - minimum/maximum area;
  - circularity / ellipse axis ratio bounds;
  - concentric-distance tolerance;
  - expected radius tolerance from manifest, if used.
- Record every parameter in `summary.json`.
- Start with count and centroid metrics before attempting pose/PnP.
- The first visual check should inspect red OpenCV overlays, not just numeric
  centroid errors.  We want to see whether the detector finds the actual
  rings, whether radii/axes look plausible, and whether ellipse orientation is
  stable on perspective/edge cases.

Good first success condition:

- On the 30-frame smoke set, OpenCV should produce sane detections and useful
  overlays even if it does not yet pass all quality gates.

## Matching And Metrics

Use A2 matching:

- match each backend detection to visible manifest markers by pixel distance;
- reject matches above `max(5 px, 0.25 * expected_pixel_radius_outer)`;
- do not assume detector order matches manifest order.

Initial summary metrics:

- frame count;
- expected marker distribution;
- detected marker distribution;
- marker recall;
- false positives per frame;
- complete-frame success rate;
- centroid error mean / median / p95 / max;
- grouped metrics by z stratum and expected marker count.

## Output Layout Adjustment

A2's original examples use `dataset/whycon_gazebo_synth_*`.
For this task family, B1 stores raw generated datasets under:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_*/
```

B2 should put validation artifacts under:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_*/
```

The validation output should record the source dataset path in `summary.json`
so artifacts remain traceable without being colocated with raw frames.

## Progress

- [x] Read A2 validation specification.
- [x] Confirmed OpenCV baseline is the right first implementation step.
- [x] Confirmed `cv2` is available in `./venv/bin/python` as version `4.13.0`.
- [x] Created this B2 implementation log.
- [x] Implement OpenCV baseline validator.
- [x] Run validator on `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439/`.
- [x] Inspect summary metrics and overlays.
- [x] Locate `sentai_sim` WhyCon implementation and public entry points.
- [x] Build or expose a `sentai_sim` WhyCon runner for dataset PGM frames.
- [x] Add `backends.sentai_sim` detections to `results.jsonl`.
- [x] Add basic OpenCV-vs-sentai disagreement metrics and separate overlays.
- [x] Add per-variable OpenCV-vs-sentai deltas for currently shared measurements.
- [x] Add report-ready CSV/Markdown ablation tables.
- [x] Start phase-2 pose/PnP validation for frames with enough matches.
- [x] Add sentai_sim run metadata/build line to validation summary.
- [x] Add yaw-bin, roll/pitch-bin, and edge/crop grouped detection summaries.
- [x] Add dedicated backend geometry summary table.
- [x] Add per-frame OpenCV-vs-sentai disagreement table.
- [x] Add pose-ambiguity diagnostics for planar PnP and symmetric marker
  correspondence alternatives.
- [x] Register dataset marker world coordinates in the `sentai_sim` runner
  from `config.json`, including the asymmetric 7th marker.
- [x] Add a pose reprojection quality gate so invalid PnP branches are
  excluded from pose statistics instead of reported as valid X/Y/Z estimates.

## Notes

- Do not require Gazebo for validation.
- Do not modify B1 datasets during validation.  B2 validation artifacts belong
  under `dataset/TD-S10-B2/`.
- Detection failure should be reported in `summary.json`; it is not a process
  crash.
- Malformed dataset files or unreadable images should make the validator exit
  non-zero.
- Pose ambiguity is evaluated as a diagnostic, not as an automatic failure:
  the validator searches alternate IPPE planar-PnP solutions and all feasible
  marker correspondence permutations for 4-7 visible markers.  It records
  cases where an alternate solution has close reprojection error but a
  materially different translation or yaw.
- The validator and `sentai_sim` runner must read marker count, marker
  diameter, and marker world coordinates from `config.json` / `manifest.jsonl`.
  Do not hardcode the previous 6-marker layout in B2.

## OpenCV Baseline Run 2026-05-23

Command:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439
```

Latest output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_094218/
```

Artifacts:

- `results.jsonl`
- `summary.json`
- `failures.csv`
- `overlays/` for failure overlays with expected/detected/match lines
- `detected_overlays/` for every frame, drawing OpenCV detected center,
  circle/ellipse, and orientation axis in red
- `opencv_detection_contact_sheet.png`

Summary:

```text
frame_count                 30
expected_markers            148
matched_markers             119
marker_recall               0.804
false_positives_total       0
complete_frame_success_rate 0.50
centroid_error_p95_px       2.46
```

By z:

```text
0.30 m: matched 42 / 46
0.50 m: matched 28 / 34
0.75 m: matched 27 / 33
1.00 m: matched 22 / 35
```

Interpretation:

- The OpenCV baseline is useful for phase-1 visual inspection: matched
  detections have good centroid accuracy and no false positives in this run.
- The baseline still misses many small or edge/cropped markers, especially at
  `1.00 m`; this is detector-configuration/algorithm behavior, not a dataset
  read/render failure.
- The red detection contact sheet is the primary artifact to inspect before
  moving to pose/PnP.

Per-frame comparability:

- `results.jsonl` already stores one object per frame with `backends.opencv`.
- The `backends.sentai_sim` slot exists but is currently marked unavailable.
- The next implementation step should fill the same per-frame fields for
  `sentai_sim`: detected count, matched count, false positives, missed count,
  matches, centroid errors, axes, angle/orientation, and pose fields if
  exposed.
- This lets us compare OpenCV and `sentai_sim` frame-by-frame without relying
  on detector output order or stable WhyCon IDs.

## sentai_sim WhyCon Implementation Map

Main implementation:

- `examples/sentai_runtime/sentai_aruco.cc`
  - WhyCon block starts around `OP-S10-W17-T1`.
  - Internal output struct: `sentai_whycon_marker_t` with `cx`, `cy`,
    `axis_a`, `axis_b`, `angle`, `comp_id`, `tvec_cam`, `rvec_cam`,
    `reproj_err_px`, and `pose_valid`.
  - Detection stages:
    - `whycon_adaptive_threshold_`: WhyCon-specific cv2-like adaptive
      threshold, block `11`, C `4`.
    - `whycon_detect_inplace_`: top-level internal pipeline.
    - `whycon_filter_and_moments_`: filters connected components, computes
      centroid, axes, and orientation from moments/contour geometry.
    - `whycon_w3_check_`: optional concentric inner-disc/ring validation.
    - `whycon_pnp_inplace_`: closed-form per-marker pose estimate.
  - Public C entry points:
    - `sentai_whycon_detect_buffer(const uint8_t* gray, int w, int h)`
    - `sentai_whycon_test_pgm(const char* path)`
    - `sentai_whycon_get_markers(sentai_whycon_marker_t* out, int cap)`
    - `sentai_whycon_set_concentric_check(int on)`
    - `sentai_whycon_set_diameter(float meters)`
    - `sentai_whycon_stage_cyc5(...)`

Dispatcher / production-facing API:

- `examples/sentai_runtime/sentai_markers.cc`
  - Forward-declares WhyCon helpers from `sentai_aruco.cc`.
  - `sentai_markers_init(SENTAI_MARKERS_BACKEND_WHYCON)` enables W3
    concentric validation by default.
  - `sentai_markers_set_intrinsics(...)` and
    `sentai_markers_set_marker_size(...)` configure pose-related fields.
  - `sentai_markers_detect_frame(...)` calls
    `sentai_whycon_detect_buffer(...)` for a caller-supplied grayscale frame.
  - `sentai_markers_get_pose(...)` returns unified `SentaiMarkersPose`, but
    this unified struct preserves centroid and pose while hiding raw
    `axis_a`, `axis_b`, and `angle`.

Header / ABI:

- `examples/sentai_runtime/sentai_markers.h`
  - Public marker backend enum and `SentaiMarkersPose`.

## A2 vs B2 Gap Closure 2026-05-23

Implemented the missing A2 reporting pieces in
`sim/scripts/validate_whycon_synthetic_dataset.py`.

Latest combined validation command:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439 \
  --sentai-results dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_103023/sentai_sim_results.jsonl
```

Latest output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_103717/
```

New/confirmed report artifacts:

- `report_tables/backend_detection_summary.{csv,md}`
- `report_tables/backend_geometry_summary.{csv,md}`
- `report_tables/backend_pose_vs_ground_truth.{csv,md}`
- `report_tables/opencv_vs_sentai_ablation.{csv,md}`
- `report_tables/backend_grouped_detection_summary.{csv,md}`
- `report_tables/failure_mode_by_z_expected_count.{csv,md}`
- `report_tables/opencv_vs_sentai_frame_disagreements.{csv,md}`

The validation summary now records:

- validator git commit short hash;
- `sentai_sim` result path and run directory;
- parsed `sentai_sim_summary.json`;
- `sentai_sim_repl.log` path;
- simulator build line, e.g. build `#548`.

Current result snapshot:

```text
OpenCV marker recall       119 / 148 = 0.804
OpenCV pose-valid frames   20 / 30
sentai_sim marker recall   3 / 148 = 0.020
sentai_sim pose-valid      0 / 30
```

Interpretation:

- The A2 report format is now covered for detection, geometry, grouped
  metrics, pose-vs-ground-truth, and OpenCV-vs-sentai deltas.
- `sentai_sim` still cannot produce camera pose on this dataset because it
  never reaches four matched markers in a frame.
- The remaining scientific blocker is no longer reporting structure; it is
  sentai WhyCon recall.  The next useful work is detector diagnostics inside
  `sentai.markers`: per-frame reject counters and threshold/binary debug
  dumps, so we can explain why Gazebo markers are mostly rejected as
  `nested`/`border`/`w3` cases.

## sentai_sim OpenCV-Parity Port 2026-05-23

Clarification:

- A2/B2 should not tune a separate WhyCon-lite detector until it resembles
  OpenCV.  The immediate goal is to port the OpenCV baseline semantics into
  `sentai_sim`/FreeRTOS and then compare that port with the Python OpenCV
  reference.

Implemented in `examples/sentai_runtime/sentai_aruco.cc`:

- Added an OpenCV-baseline parity path for WhyCon:
  - global inverse threshold sweep: `100`, `130`, `150`;
  - foreground connected-component extraction;
  - outer circular component gate: area, radius, circularity;
  - concentric dark-dot pairing using bbox/moment center;
  - center-offset and dot/outer-radius ratio gates;
  - dedupe across threshold passes;
  - existing `sentai.markers` output ABI remains unchanged.
- Increased component capacity from `96` to `240`, because the Gazebo floor
  texture creates many small threshold components before real markers,
  especially in the higher-z frames.
- Removed W3 relaxation from the active result.  The parity path does not use
  W3 as a substitute for the OpenCV baseline structure test.

Final run after cleanup:

```bash
cmake --build build-sim --target sentai_sim -j2
./venv/bin/python sim/scripts/run_sentai_sim_whycon_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439 \
  --sentai-results \
  dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_110520/sentai_sim_results.jsonl
```

Outputs:

```text
sentai output:
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_110520/

validation output:
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_110520/
```

Result snapshot:

```text
OpenCV marker recall       119 / 148 = 0.804
sentai_sim marker recall   128 / 148 = 0.865
OpenCV false positives       0
sentai_sim false positives   0
OpenCV pose-valid frames    20 / 30
sentai_sim pose-valid       23 / 30
count/match disagreements    4 / 30
```

Interpretation:

- The initial `sentai_sim` failure was not a filesystem/REPL problem.
- The largest bug was semantic mismatch: the old embedded path used a
  different adaptive-threshold/W3/nesting pipeline, not the OpenCV baseline.
- The OpenCV-parity port now matches or exceeds OpenCV on manifest recall
  without false positives on this smoke set.
- The remaining 4 disagreement frames are all `z=1.0 m` cases where the C++
  port detects additional manifest-valid markers that the current Python
  OpenCV baseline misses.  These should be treated as port/reference
  divergence to inspect visually, not as a recall failure.

Next focused work:

1. Inspect the 4 disagreement frames visually with both overlays.
2. Decide whether B2's Python OpenCV baseline should be updated to the same
   small-dot pairing rule used in the C++ port, or whether the C++ port should
   be tightened to match OpenCV's exact contour hierarchy behavior.
3. Once the reference/port semantics are locked, re-run the ablation and then
   move to a larger B1 dataset.

## Four-Frame Disagreement Analysis 2026-05-23

Analyzed the only 4 frames where the OpenCV baseline and the C++/sentai port
disagree after the OpenCV-parity port:

```text
15  frame_000015_x-0.500_y-0.250_z1.000_r+4.0_p-3.0_yaw315_n6.pgm
19  frame_000019_x-0.500_y+0.000_z1.000_r+0.0_p+0.0_yaw045_n5.pgm
23  frame_000023_x-0.500_y+0.000_z1.000_r+4.0_p-3.0_yaw045_n5.pgm
27  frame_000027_x-0.500_y+0.000_z1.000_r+0.0_p+0.0_yaw135_n5.pgm
```

All 4 are high-altitude `z=1.0 m` frames with edge/crop pressure.  The
visual comparison artifact is:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_110520/debug_four_divergent_frames.png
```

Observed per-frame differences:

```text
frame 15: OpenCV 4/4, sentai 5/5, sentai additionally detects W.
frame 19: OpenCV 1/1, sentai 4/4, sentai additionally detects NW, SW, SE.
frame 23: OpenCV 2/2, sentai 4/4, sentai additionally detects W, E.
frame 27: OpenCV 1/1, sentai 4/4, sentai additionally detects SW, NW, NE.
```

Visual interpretation:

- The additional `sentai_sim` detections are real WhyCon markers visible in
  the rendered frame and matched to manifest markers, not false positives.
- The validator reports `sentai_sim` false positives as zero in these frames.
- The divergence is therefore not "sentai too permissive" in the usual
  detection-quality sense.  It is an algorithm-spec mismatch between the
  current Python OpenCV baseline and the C++ port.

Likely root cause:

- The Python OpenCV baseline currently requires a contour hierarchy pattern
  equivalent to outer dark ring -> inner light hole -> dark dot.
- At `z=1.0 m`, especially near edges/crops, the dark center dot is very small
  and the contour hierarchy can become unstable or too small for the baseline
  contour path to accept.
- The C++ port uses the same threshold sweep and outer-marker gates, but pairs
  the center dot using component bbox/moment geometry rather than requiring the
  dot to survive as a fully traceable contour child/grandchild.  This recovers
  the visible small markers.

Decision still required:

1. If the intended reference algorithm is strict OpenCV contour hierarchy,
   tighten the C++ port to exactly match that hierarchy, accepting lower recall
   on these high-z edge cases.
2. If the intended reference algorithm is connected-component concentric
   ring/dot detection implemented with OpenCV primitives, update the Python
   OpenCV baseline to use the same small-dot component pairing as the C++ port.

Preferred next step before changing code:

- Read/record the relevant OpenCV contour/connected-component behavior and
  the WhyCon/concentric-marker algorithm assumption we want to claim.  Then
  make both implementations match that documented reference.

## Active Debug Plan - sentai_sim WhyCon Recall

Decision 2026-05-23:

- Keep using the existing 30-frame smoke dataset:
  `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439/`.
- Do not generate a larger B1 dataset yet.  The current set is sufficient for
  improving the embedded WhyCon detector because it already contains z,
  yaw/roll/pitch, and 4/5/6-marker cases.
- Target improvement: bring `sentai_sim` WhyCon recall closer to the OpenCV
  baseline on the same 30 frames before expanding the dataset.

Debug strategy:

1. Treat the latest OpenCV baseline as the algorithmic reference to port into
   `sentai_sim`, not merely as a loose comparison target.
2. Avoid detector-behavior compromises such as disabling W3, relaxing gates
   until recall rises, or using a different WhyCon-lite interpretation.
3. The production-candidate test path should expose the same measured outputs
   as OpenCV, but the underlying C++ detector should implement the same
   baseline logic: global threshold sweep, ring/hole/dot structure,
   contour/shape gates, centroid matching, and dedupe.
4. Preserve the `sentai.markers` public API and the existing B2 result schema.
5. After every change, run:

```bash
cmake --build build-sim --target sentai_sim -j2
./venv/bin/python sim/scripts/run_sentai_sim_whycon_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439 \
  --sentai-results <latest-sentai-output>/sentai_sim_results.jsonl
```

6. Compare the new `backend_detection_summary` and
   `opencv_vs_sentai_ablation` tables against the current baseline:

```text
OpenCV marker recall       119 / 148 = 0.804
sentai_sim marker recall     3 / 148 = 0.020
sentai_sim pose-valid        0 / 30
```
  - Public functions for init, intrinsics, marker size, frame detection, count,
    pose, and stats.

MicroPython binding:

- `examples/sentai_runtime/bindings/modsentai_whycon.c`
  - Exposes timing/test APIs such as `_test_pgm(path)`, `_set_concentric(on)`,
    `_set_diameter(metres)`, `_stage_cyc5()`.
  - Exposes `_get_marker_details(i, bytearray(56))`, which writes the full
    56-byte `sentai_whycon_marker_pub_t` including axes, angle, pose, and
    `pose_valid`.

Implication for B2:

- Best phase-1 comparison target is the raw WhyCon ABI, not only the unified
  `sentai_markers_get_pose`, because phase 1 needs axes and orientation.
- Practical options:
  1. Build a small native host CLI that links `sentai_aruco.cc` and calls
     `sentai_whycon_test_pgm` / `sentai_whycon_get_markers`.
  2. Run through the MicroPython binding if the sentai runtime build is already
     available and can read the dataset PGM paths.
  3. Use `sentai_markers_detect_frame` for production parity, but add or expose
     a raw-details query so B2 can compare `axis_a`, `axis_b`, and `angle`.

## sentai_sim REPL Runner

Decision:

- Use the production-facing SIM namespace `sentai.markers`, not the old
  `sentai.whycon` namespace.  In `sim/modsentai_sim.c`, the legacy
  `sentai.whycon` MicroPython binding is explicitly de-registered after the
  W19 unified marker rename.
- Run detection inside `build-sim/sim/sentai_sim`, with dataset frames exposed
  through `build-sim/sentai_fs_root` using symlinks.
- Feed canonical grayscale `P5` PGM pixels to
  `sentai.markers.detect_buffer(gray, 320, 240)`.

Implementation artifact:

- `sim/scripts/run_sentai_sim_whycon_dataset.py`
  - Creates `dataset/TD-S10-B2/<dataset_id>/sentai_sim_YYYYMMDD_HHMMSS/`.
  - Symlinks the B1 dataset into SIM FS as `/td_s10_b1_dataset`.
  - Symlinks the output folder into SIM FS as `/td_s10_b2_out`.
  - Stages a MicroPython script into `build-sim/sentai_fs_root`.
  - Starts `build-sim/sim/sentai_sim` and imports the staged script through
    the REPL.
  - The staged script reads each frame through `sentai.fs.read(...)`, parses
    the PGM header, runs `sentai.markers.init("whycon")` and
    `sentai.markers.detect_buffer(...)`, then writes:
    - `sentai_sim_results.jsonl`
    - `sentai_sim_summary.json`
    - `sentai_sim_repl.log`

Important API limitation:

- `sentai.markers.get_pose_tuple(i)` exposes `(id, pixel_cx, pixel_cy, tvec,
  rvec, reproj_err, backend, pose_valid)`.
- It does not expose raw WhyCon `axis_a`, `axis_b`, or `angle`.
- Therefore this SIM path currently validates count, centroid, and pose tuple.
  Orientation/ellipse validation still needs either:
  - a diagnostic binding that exposes raw WhyCon marker details from
    `sentai_aruco.cc`, or
  - a host CLI that calls the raw `sentai_whycon_get_markers(...)` ABI.

## sentai.markers Refactor Plan

Decision:

- Keep `sentai.markers` as the canonical public namespace.  Do not reintroduce
  `sentai.whycon` as a parallel user-facing API.
- Encapsulate WhyCon inside `sentai.markers`, but expose enough diagnostic
  geometry for dataset validation:
  - centroid;
  - fitted axes;
  - fitted angle/orientation;
  - component id;
  - pose tuple where available;
  - detector/backend id and `pose_valid`.
- Keep all dataset validation runnable from inside `sentai_sim` through
  MicroPython REPL and `sentai.fs`, with the host only staging files and
  collecting artifacts.

Minimal implementation scope:

1. Add `SentaiMarkersDetection` to `sentai_markers.h`.
2. Populate that struct in `sentai_markers.cc` for WhyCon by copying the raw
   `sentai_whycon_marker_t` fields after `sentai_whycon_detect_buffer(...)`.
   ArUco can return pose-derived centroid only for now, or return unavailable
   raw axis fields.
3. Expose:
   - `sentai_markers_get_detection(i, out)`;
   - `sentai.markers.get_detection(i, out_buf)`;
   - `sentai.markers.get_detection_tuple(i)`.
4. Add `sentai.markers.detect_pgm(path)` so REPL scripts do not need to parse
   PGM headers in MicroPython.  It should read from SIM/host filesystem in the
   same way as existing SIM file helpers and then call the active backend.
5. Update the B2 SIM runner to prefer:

```python
sentai.markers.init("whycon")
n = sentai.markers.detect_pgm("/td_s10_b1_dataset/frames/frame_000000.pgm")
for i in range(n):
    det = sentai.markers.get_detection_tuple(i)
```

Acceptance for this refactor:

- `build-sim/sim/sentai_sim` builds.
- A REPL smoke can call `sentai.markers.detect_pgm(...)`.
- A REPL smoke can retrieve raw WhyCon `axis_a`, `axis_b`, and `angle`.
- B2 artifacts contain enough data to compare OpenCV and `sentai_sim` on
  count, centroid, circle/ellipse scale, and orientation.
- Phase 2 artifacts contain enough data to compare OpenCV and `sentai_sim`
  camera-pose estimates against the manifest camera pose relative to the
  ground origin `(0, 0, 0)`.

Implementation status 2026-05-23:

- Added `SentaiMarkersDetection` to
  `examples/sentai_runtime/sentai_markers.h`.
- Added `sentai_markers_detect_pgm(...)` and
  `sentai_markers_get_detection(...)` in
  `examples/sentai_runtime/sentai_markers.cc`.
- Added MicroPython bindings in
  `examples/sentai_runtime/bindings/modsentai_markers.c`:
  - `sentai.markers.detect_pgm(path)`
  - `sentai.markers.get_detection(i, out_buf)`
  - `sentai.markers.get_detection_tuple(i)`
- Added SIM-only QSTR entries in `examples/sentai_runtime/qstrdefs_sim_extra.h`
  and the current generated QSTR header.
- Updated `sim/scripts/run_sentai_sim_whycon_dataset.py` to use
  `detect_pgm(...)` and emit comparable detection geometry:
  `center`, `axis_a`, `axis_b`, `angle_rad`, `tvec_cam`, `rvec_cam`.
- Updated `sim/scripts/validate_whycon_synthetic_dataset.py` to ingest
  `--sentai-results`, match sentai detections with the same centroid matcher,
  and write both `backends.opencv` and `backends.sentai_sim` into
  `results.jsonl`.

QSTR regeneration:

```bash
cd /home/bogdan/work/coralmicro/examples/sentai_runtime
rm -rf build-embed
make -f ../../third_party/micropython/ports/embed/embed.mk \
    MICROPYTHON_TOP=../../third_party/micropython \
    USER_C_MODULES=$(pwd)/modules \
    micropython-embed-package
```

Status: OK.  This follows `.claude/skills/qstr-regen/SKILL.md`; the generated
QSTR table now contains:

```text
MP_QSTR_detect_pgm
MP_QSTR_get_detection
MP_QSTR_get_detection_tuple
```

SIM build:

```bash
cmake --build build-sim --target sentai_sim -j2
```

Status: OK, produced `sentai_sim` build `#548` after canonical QSTR regen.

REPL self-test:

```python
sentai.markers.init("whycon")
sentai.markers.synth_one_whycon(160, 120, 24)
sentai.markers.get_detection_tuple(0)
```

Returned one synthetic marker with geometry:

```text
(0, 160.0, 120.0, 27.9261, 27.9261, 0.0, 0, ...)
```

Smoke command:

```bash
./venv/bin/python sim/scripts/run_sentai_sim_whycon_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439 --limit 3
```

Smoke output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_100302/
```

Smoke result:

```json
{"backend":"sentai_sim.markers.whycon","init_rc":0,"frames":3,"frames_with_detections":0,"detections_total":0}
```

Full 30-frame command:

```bash
./venv/bin/python sim/scripts/run_sentai_sim_whycon_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439
```

Full 30-frame output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_100339/
```

Full 30-frame result:

```json
{"backend":"sentai_sim.markers.whycon","init_rc":0,"frames":30,"frames_with_detections":3,"detections_total":3}
```

Observed detections:

- `frame_000003...`: 1 marker
- `frame_000027...`: 1 marker
- `frame_000028...`: 1 marker
- All other frames: 0 markers

Diagnostic notes:

- The internal synthetic self-test works:
  `sentai.markers.synth_one_whycon(160,120,24)` returns 1 marker in
  `sentai_sim`.
- The B1 PGM path through `sentai.fs.read(...)` works, and the C++ detector is
  reached.
- The detector log for missed frames shows many connected components rejected
  as `nested` or `border`, for example:

```text
[whycon_filter v3] n_comp=36 accepted=0 rejects: nested=29 border=7 ...
```

Interpretation:

- This is now a real ablation signal: OpenCV baseline sees most markers in the
  same B1 dataset, while `sentai_sim` WhyCon detects only 3 markers total over
  30 frames.
- The likely mismatch is not the REPL/filesystem harness.  It is between the
  Gazebo-rendered grayscale marker appearance and the current assumptions in
  the embedded WhyCon filter, especially connected-component hierarchy,
  border-touch behavior, and/or W3 concentric validation.

## Full Ablation Run 2026-05-23

Sentai SIM dataset runner:

```bash
./venv/bin/python sim/scripts/run_sentai_sim_whycon_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439
```

Sentai output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_101734/
```

Sentai summary:

```json
{"backend":"sentai_sim.markers.whycon","init_rc":0,"frames":30,"frames_with_detections":3,"detections_total":3}
```

Observed sentai detections:

- `frame_000017...`: 1 marker, matched.
- `frame_000027...`: 1 marker, matched.
- `frame_000028...`: 1 marker, matched.
- All other frames: 0 markers.

Combined OpenCV-vs-sentai validation:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439 \
  --sentai-results \
  dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_101734/sentai_sim_results.jsonl
```

Combined output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_101900/
```

Summary:

```text
OpenCV    matched 119 / 148, recall 0.804, false positives 0
sentai    matched   3 / 148, recall 0.020, false positives 0
disagree  29 / 30 frames have count or match disagreement
```

Artifacts:

- `results.jsonl`: per-frame `backends.opencv` and `backends.sentai_sim`.
- `summary.json`: backend metrics and disagreement examples.
- `opencv_detection_contact_sheet.png`
- `sentai_detection_contact_sheet.png`
- `detected_overlays/`
- `sentai_detected_overlays/`

Interpretation:

- The full ablation pipeline is now working for phase 1 geometry.
- `sentai_sim` output is produced inside `sentai_sim` through REPL/FS, not by a
  host-side detector.
- The current embedded WhyCon detector severely under-detects Gazebo B1
  frames compared with the OpenCV baseline.
- For the three markers it does detect, centroid matching succeeds and
  centroid error is small, so the main failure mode is recall, not false
  positives or gross localization error.

## Next Implementation Step

Continue B2 with phase 2 camera-pose parity:

1. Add an OpenCV pose solver using matched marker centers and manifest marker
   world coordinates.
2. Add a sentai pose path using the same matched markers or the
   `sentai.markers` drone-pose API, but ensure the association is comparable.
3. Compare estimated camera pose against manifest `pose_world_camera` relative
   to ground origin `(0, 0, 0)`.
4. Separately investigate sentai recall failure using detector diagnostics:
   connected-component hierarchy, `nested` rejects, `border` rejects, and W3
   concentric validation.

## Gap Analysis Against Current A2

A2 now requires a complete research-report-ready ablation over every shared
measured variable.  B2 currently satisfies the detection-run plumbing and a
first phase-1 comparison, but not the full A2 target.

Done:

- OpenCV backend runs on the B1 dataset.
- `sentai_sim` backend runs inside `sentai_sim` through REPL/FS.
- Both backends write into one per-frame `results.jsonl`.
- Both backends expose count, matched count, centroid, axes, angle/orientation
  where detections exist.
- `summary.json` reports backend detection recall, false positives,
  complete-frame success, centroid stats, and basic disagreement count.
- Separate OpenCV and `sentai_sim` visual overlays/contact sheets exist.

Partial:

- Backend disagreement now includes a `backend_delta` object per frame for
  currently available shared variables:
  - count delta;
  - matched count delta;
  - centroid delta;
  - radius delta;
  - `axis_a` / `axis_b` delta;
  - angle delta.
  This remains sparse because `sentai_sim` detects only 3 markers in the
  30-frame run.
- Grouping exists for z and expected marker count, but not yet for yaw bins,
  roll/pitch bins, or explicit edge/crop category.
- Failure artifacts now include sentai failure rows, but not yet all
  backend-delta failure modes.
- Pose evaluation starts for any backend with at least 4 matched markers.
  OpenCV has pose on 20 frames in the latest run; `sentai_sim` has 0 pose-valid
  frames because it never reaches 4 matched markers.

Missing:

- Camera pose estimation relative to `(0, 0, 0)` for `sentai_sim` will only
  become meaningful after recall improves enough to provide at least 4 matched
  markers.
- Per-frame pose deltas between OpenCV and `sentai_sim` are implemented but
  unavailable in the latest dataset because `sentai_sim` has 0 pose-valid
  frames.
- Report-ready tables exist, except a dedicated backend geometry summary table
  still needs to be split out if required by the final report.
- `sentai_sim` build metadata and detector parameter metadata in
  `summary.json`.

Recommended next order:

1. Add sentai build metadata and detector parameter metadata to `summary.json`.
2. Add yaw-bin, roll/pitch-bin, and edge/crop grouping.
3. Add a dedicated backend geometry summary table if needed by the report.
4. Investigate sentai recall failure using detector diagnostics:
   connected-component hierarchy, `nested` rejects, `border` rejects, and W3
   concentric validation.
5. Re-run after any detector change and update the ablation tables.

## Full Ablation Run 2026-05-23, With Phase-1 Deltas And Pose

Sentai SIM dataset runner:

```bash
./venv/bin/python sim/scripts/run_sentai_sim_whycon_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439
```

Sentai output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_103023/
```

Sentai summary:

```json
{"backend":"sentai_sim.markers.whycon","init_rc":0,"fx":288.299988,"fy":288.299988,"cx":160.000000,"cy":120.000000,"marker_diameter_m":0.054400,"frames":30,"frames_with_detections":3,"detections_total":3}
```

Combined OpenCV-vs-sentai validation:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439 \
  --sentai-results \
  dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_103023/sentai_sim_results.jsonl
```

Combined output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_103129/
```

Summary:

```text
OpenCV    matched 119 / 148, recall 0.804, pose-valid frames 20
sentai    matched   3 / 148, recall 0.020, pose-valid frames 0
disagree  29 / 30 frames have count or match disagreement
```

Report tables:

```text
report_tables/backend_detection_summary.{csv,md}
report_tables/backend_pose_vs_ground_truth.{csv,md}
report_tables/opencv_vs_sentai_ablation.{csv,md}
report_tables/failure_mode_by_z_expected_count.{csv,md}
```

Current pose note:

- OpenCV pose is now estimated relative to the dataset ground/world origin.
- The pose convention correction matches the B1 projection convention
  `r_wc = Rz(yaw) * Ry(pitch) * Rx(roll) * r0`.
- Latest OpenCV pose summary:
  - 20 pose-valid frames;
  - translation RMSE `0.0106 m`;
  - translation p95 `0.0211 m`;
  - yaw MAE `0.123 deg`;
  - yaw p95 `0.475 deg`.
- `sentai_sim` has no pose-valid frames because it detects too few markers.

## Paper-Backed WhyCon Baseline Update 2026-05-23

Clarification from A2: the OpenCV backend is not meant to be an invented local
detector.  It is the reference implementation environment for a
literature-backed WhyCon detector.  OpenCV supplies the image-processing
primitives; the algorithmic behavior should follow the published WhyCon family
for black/white circular markers.

References to cite in reports and metadata:

- T. Krajnik et al., "External Localization System for Mobile Robotics",
  ICAR 2013.
- T. Krajnik et al., "A Practical Multirobot Localization System", Journal of
  Intelligent and Robotic Systems, 2014.
- M. Nitsche et al., "WhyCon: An Efficient, Marker-based Localization System",
  IROS Open Source Aerial Robotics Workshop, 2015.
- `lrse/whycon`, the open-source WhyCon implementation and documentation.

Implementation adjustment:

- Replaced the earlier contour-hierarchy OpenCV baseline with a
  connected-component/concentric-marker baseline closer to the WhyCon family:
  global threshold variants, 8-connected components, outer circular component
  geometry, inner dot/ring concentricity check, duplicate suppression, and the
  same external manifest-based matching/evaluation.
- Updated validator metadata so `summary.json` records the baseline as
  `connected_component_concentric_whycon`, with explicit literature notes.
- This remains a host-side Python/OpenCV implementation for transparency; it
  is not an official OpenCV module because OpenCV does not ship `cv2.whycon`.

Rerun on current smoke dataset:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439 \
  --sentai-results \
  dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_110520/sentai_sim_results.jsonl
```

Output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_111927/
```

Detection summary:

```text
OpenCV    matched 128 / 148, recall 0.864865, false positives 0
sentai    matched 128 / 148, recall 0.864865, false positives 0
disagree  0 / 30 frames have count or match disagreement
```

Ablation summary:

```text
comparable_frames          30
marker_delta_count         128
centroid_delta_p95_px      0.615
pose_delta_count           23
translation_delta_mean_m   0.00914
translation_delta_p95_m    0.02765
yaw_delta_mean_deg         0.00721
yaw_delta_p95_deg          0.22638
```

Interpretation:

- The detection/count mismatch is now gone on the smoke 30 dataset.
- Remaining differences are mostly measurement-definition issues rather than
  recall failure:
  - `sentai_sim` currently reports axes/radius from its raw component geometry,
    while OpenCV reports ellipse/contour-derived geometry, so axis/radius
    deltas are not yet a clean 1-to-1 algorithmic comparison.
  - Orientation is especially ambiguous for near-circular ellipses and should
    be treated as diagnostic unless both paths define the same axis convention.
  - Pose deltas are small enough for this smoke set, but the axis/radius
    semantic mismatch should be fixed before claiming full algorithmic parity.

## Geometry-Parity Follow-up 2026-05-23

Next target after count/recall parity:

- Make `radius_outer`, `axis_a`, `axis_b`, and `angle` mean the same thing in
  both backends.
- Current mismatch:
  - OpenCV path used `cv2.fitEllipse` over the external contour for axes.
  - `sentai_sim` uses connected-component moments/eigenvalues for axes and a
    bounding-box-derived outer radius.
  - For a black annulus plus center dot, contour ellipse and component moments
    are not equivalent, so the existing axis/radius deltas are partly a
    measurement-definition artifact.
- Decision:
  - Keep `radius_outer = max(component_bbox_width, component_bbox_height) / 2`
    in both backends.
  - Change the OpenCV-backed reference geometry to use the same
    component-moment eigenvalue semantics as `sentai_sim`.
  - Keep contour-derived area/circularity only for the outer-shape acceptance
    gate, not for reported axes.
  - Treat `angle` as a diagnostic field with 180-degree periodicity; because
    near-circular markers make the major-axis direction weakly conditioned,
    use pose/centroid as primary parity measures unless both paths expose a
    stable identical angle convention.

Success criteria for this step:

- Rerun on smoke 30 with unchanged sentai output if possible.
- Count disagreement remains `0 / 30`.
- Centroid delta remains sub-pixel p95.
- Axis/radius deltas shrink substantially because both paths now report the
  same geometric definition.

## Geometry-Parity Result 2026-05-23

Implemented:

- Added `radius_outer` to the `sentai.markers` diagnostic detection payload.
  The field is appended to `get_detection_tuple()` so the existing first
  fields remain stable.
- Updated `run_sentai_sim_whycon_dataset.py` to write `radius_outer` from
  `sentai_sim`, instead of reconstructing it as `max(axis_a, axis_b)`.
- Updated the OpenCV-backed reference geometry to use connected-component
  moment/eigenvalue semantics for `axis_a`, `axis_b`, and `angle`.
- Aligned `sentai_sim` and OpenCV reference on component centroid/moments;
  contour geometry remains only an acceptance gate for area/circularity.

Build:

```text
sentai_sim build #558 (2026-05-23 11:27:12)
```

Sentai smoke run:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/sentai_sim_20260523_112734/
```

Combined validation:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_112734/
```

Detection summary:

```text
OpenCV    matched 128 / 148, recall 0.864865, false positives 0
sentai    matched 128 / 148, recall 0.864865, false positives 0
disagree  0 / 30 frames have count or match disagreement
```

Backend delta summary:

```text
centroid_delta_p95_px       0.22004
radius_delta_p95_px         0.5
axis_a_delta_mae_px         0.04417
axis_a_delta_rmse_px        0.07506
axis_b_delta_mae_px         0.04447
axis_b_delta_rmse_px        0.07853
pose_delta_count            23
translation_delta_mean_m    0.00434
translation_delta_p95_m     0.01674
yaw_delta_mean_deg         -0.00216
yaw_delta_p95_deg           0.03631
```

Interpretation:

- Detection parity is achieved on the smoke 30 dataset.
- Geometry parity is now good for centroid, radius, and axes.
- `angle_delta_p95_deg` remains high (`38.17 deg`) even after moment parity.
  This is expected for nearly circular projected blobs: when `axis_a` and
  `axis_b` are close, the major-axis direction is weakly conditioned and may
  flip/rotate without materially changing centroid, radius, axes, or pose.
  Keep angle as a diagnostic field, but do not use it alone as a success/fail
  criterion unless a minimum eccentricity gate is added.

Next useful step before scaling the dataset:

- Add an angle-stability qualifier to the validator, for example evaluate
  `angle_delta_deg` only when `axis_a / axis_b >= 1.10` or when eccentricity is
  above a documented threshold.
- Keep the current smoke 30 as the regression set; do not generate a larger
  B1 dataset until this criterion is recorded and stable.

## Angle-Stability Report Update 2026-05-23

Implemented the angle-stability qualifier in
`sim/scripts/validate_whycon_synthetic_dataset.py`:

- `angle_stability_min_axis_ratio = 1.10`
- `angle_delta_deg` is still reported for all matched markers.
- `angle_delta_stable_deg` is reported only for markers where both backends
  have `axis_a / axis_b >= 1.10`.

Latest validation:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_112857/
```

Report table:

```text
comparable_frames          30
count_disagreement_frames  0
marker_delta_count         128
centroid_delta_p95_px      0.22004
radius_delta_p95_px        0.5
axis_a_delta_p95_px        0.00029
axis_b_delta_p95_px        0.09807
angle_delta_p95_deg        38.1713
angle_stable_count         13
angle_stable_p95_deg       2.14846
pose_delta_count           23
translation_delta_mean_m   0.00434
translation_delta_p95_m    0.01674
yaw_delta_p95_deg          0.03631
```

Interpretation:

- The high all-marker angle p95 is not a detector mismatch by itself; it is
  dominated by low-eccentricity detections where major-axis direction is
  mathematically unstable.
- On the stable-angle subset, OpenCV and `sentai_sim` agree well:
  `angle_stable_p95_deg = 2.15`.
- This smoke set is now good enough as a regression set for the current
  algorithmic-parity work.

Remaining before calling A2 complete:

- Decide whether the report should include all-marker angle only as
  diagnostic and stable-angle as the formal parity metric.
- Add the same angle-stability rule to A2 if we accept it as methodology.
- Optionally add a compact "publication table" that includes recall, centroid,
  radius, axes, stable angle, and pose in one row.

## OpenCV Recall Experiment - Cropped Markers 2026-05-23

Question: can the paper-backed OpenCV reference increase recall before we port
anything to `sentai_sim`?

Miss analysis on the smoke 30 dataset:

```text
paper baseline missed markers  20
missed marker category         20 / 20 cropped
interior recall                20 / 20
near-edge recall               12 / 12
cropped recall                 96 / 116
```

Interpretation:

- The remaining recall loss is not caused by normal fully visible WhyCon
  markers.
- All misses are markers whose projected bbox leaves the image.  For these,
  component centroid is biased toward the visible arc, so a strict
  centroid-to-ground-truth-center match can fail even when the visible marker
  structure is present.

Implemented an explicit OpenCV-only experimental variant:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439 \
  --opencv-variant edge_partial \
  --overlay-limit 0
```

Variant behavior:

- `paper` remains the default parity baseline.
- `edge_partial` lowers the outer circularity gate for low-circularity edge
  candidates and applies `cv2.minEnclosingCircle` to the visible contour to
  estimate the full-circle center/radius.
- This is intentionally not compared against `sentai_sim` as a parity result
  until the same behavior is ported and validated there.

Outputs:

```text
paper baseline:
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_114124/

edge_partial experiment:
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_092439/validation_20260523_114132/
```

Results:

```text
paper        matched 128 / 148, recall 0.864865, false positives 0, complete frames 0.50
edge_partial matched 139 / 148, recall 0.939189, false positives 0, complete frames 0.80

paper cropped recall        96 / 116
edge_partial cropped recall 107 / 116
```

Conclusion:

- Yes, recall can be improved on this smoke set, but the gain comes from an
  edge/cropped-marker extension rather than from the fully visible WhyCon core.
- This extension is plausible for a robust practical detector, but it should be
  treated as a named algorithm variant.  It is not yet the A2 parity baseline.
- Next step, if accepted: port `edge_partial` to `sentai_sim` as a separate
  variant and rerun full ablation against the OpenCV `edge_partial` reference.

## 100-Frame Ablation 2026-05-23

Dataset:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_114246/
```

Dataset distribution:

```text
frame_count          100
marker_distribution {4: 28, 5: 18, 6: 54}
z_distribution       {0.3: 16, 0.5: 28, 0.75: 28, 1.0: 28}
```

Sentai run:

```bash
./venv/bin/python sim/scripts/run_sentai_sim_whycon_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_114246
```

Output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/sentai_sim_20260523_114846/
sentai_sim build #558 (2026-05-23 11:27:12)
```

Paper-baseline ablation:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_114246 \
  --sentai-results \
  dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/sentai_sim_20260523_114846/sentai_sim_results.jsonl \
  --overlay-limit 0
```

Output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/validation_20260523_114854/
```

Paper-baseline result:

```text
OpenCV paper matched 483 / 526, recall 0.918251, false positives 0, complete frames 0.68
sentai_sim   matched 483 / 526, recall 0.918251, false positives 3, complete frames 0.65

count disagreement frames 3 / 100
centroid_delta_p95_px     0.271884
axis_a_delta_p95_px       0.000362
axis_b_delta_p95_px       0.163386
angle_stable_count        35
angle_stable_p95_deg      3.76007
pose_delta_count          87
translation_delta_p95_m   0.0174059
yaw_delta_p95_deg         0.108553
```

OpenCV `edge_partial` experiment against same sentai output:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_114246 \
  --opencv-variant edge_partial \
  --sentai-results \
  dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/sentai_sim_20260523_114846/sentai_sim_results.jsonl \
  --overlay-limit 0
```

Output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/validation_20260523_114856/
```

`edge_partial` result:

```text
OpenCV edge_partial matched 505 / 526, recall 0.960076, false positives 12, complete frames 0.75
sentai_sim          matched 483 / 526, recall 0.918251, false positives 3, complete frames 0.65

count disagreement frames 33 / 100
centroid_delta_p95_px     0.271884
axis_a_delta_p95_px       0.000362
axis_b_delta_p95_px       0.163386
angle_stable_count        35
angle_stable_p95_deg      3.76007
pose_delta_count          87
translation_delta_p95_m   0.0172099
yaw_delta_p95_deg         0.167010
```

Grouped recall:

```text
OpenCV paper:
  interior   232 / 232
  near_edge   12 / 12
  cropped    239 / 282

OpenCV edge_partial:
  interior   232 / 232
  near_edge   12 / 12
  cropped    261 / 282
```

Interpretation:

- On 100 frames, the paper-backed OpenCV baseline and `sentai_sim` have the
  same matched recall, which is a strong parity signal.
- `sentai_sim` still reports 3 false positives relative to the manifest
  matching threshold, causing 3 count-disagreement frames.
- `edge_partial` improves OpenCV recall by `+22` matched markers, all from
  cropped cases, but introduces `12` false positives.  It is therefore a
  promising recall variant, not yet a clean replacement for the paper parity
  baseline.
- The next technical decision is whether to port `edge_partial` to
  `sentai_sim` and then tune its cropped-marker acceptance gates to reduce
  false positives while keeping the recall gain.

## Visibility Convention Impact 2026-05-23

A1/A2 now define default expected markers as fully visible markers only.
Markers with projected bboxes that leave the image are `partial_crop` stress
cases and are excluded from default recall, complete-frame, centroid/scale, and
pose metrics.

Impact on the B2 runs above:

- The 30-frame and 100-frame datasets used in the previous ablations were
  generated before this convention and counted `partial_crop` markers as
  visible/expected.
- The previous `paper` and `sentai_sim` recall values are therefore stricter
  than the new default metric because they penalize cropped markers.
- `edge_partial` remains useful as a stress/variant result for cropped-marker
  recovery, but it should not be required for the default paper-baseline
  metric.

Implementation status:

- Updated `sim/scripts/validate_whycon_synthetic_dataset.py` so the default
  expected set is `visibility_reason == "visible"`.
- Added `--include-partial-crop` to reproduce the older stress metric.
- Default metrics now ignore detector outputs that match `partial_crop`
  manifest markers.  These detections are neither true positives nor false
  positives; they are counted separately as ignored detections.

Re-run on the pre-existing 100-frame dataset:

```text
default standard metric:
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/validation_20260523_120842/

legacy include-partial-crop metric:
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/validation_20260523_120843/
```

Standard metric result:

```text
OpenCV paper matched 444 / 444, recall 1.0, false positives 0, complete frames 1.00, ignored crop detections 39
sentai_sim   matched 444 / 444, recall 1.0, false positives 3, complete frames 0.97, ignored crop detections 39
```

Legacy stress metric result:

```text
OpenCV paper matched 483 / 526, recall 0.918251, false positives 0
sentai_sim   matched 483 / 526, recall 0.918251, false positives 3
```

Interpretation:

- The core detector has perfect recall on fully visible markers in this
  100-frame dataset.
- The remaining `sentai_sim` false positives are not explained away by
  partial-crop markers and should be inspected separately.
- Cropped-marker recovery should remain a separate stress/variant metric.

OpenCV `edge_partial` under the new standard metric:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/validation_20260523_120958/

OpenCV edge_partial matched 444 / 444, recall 1.0, false positives 12, ignored crop detections 61
sentai_sim          matched 444 / 444, recall 1.0, false positives 3,  ignored crop detections 39
```

Interpretation:

- Once `partial_crop` markers are excluded from default recall, `edge_partial`
  no longer improves the standard true-positive count; both OpenCV variants
  already match all 444 fully visible markers.
- `edge_partial` still introduces 12 false positives, so it should not be used
  for the default metric.
- Its useful role is a separately labeled cropped-marker stress experiment.

## Pose Ambiguity Diagnostic 2026-05-23

Added a diagnostic for the question: can marker/layout symmetry produce a
second plausible camera pose?

Implementation:

- For each backend pose estimate, record primary reprojection RMSE.
- Run `cv2.solvePnPGeneric(..., SOLVEPNP_IPPE)` to inspect planar-PnP
  alternate solutions.
- Try all marker correspondence permutations for frames with 4-6 visible
  markers, because WhyCon marker IDs are not visually encoded and a symmetric
  layout may make a wrong assignment plausible.
- Keep alternate solutions with positive depth and positive camera `z`.
- Mark a near-ambiguous alternate when reprojection is close to the primary
  solution and translation or yaw is materially different.

Latest run:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_114246/validation_20260523_121953/
```

Artifacts:

```text
report_tables/pose_ambiguity_candidates.csv
report_tables/pose_ambiguity_candidates.md
report_tables/backend_pose_vs_ground_truth.csv
report_tables/backend_pose_vs_ground_truth.md
```

Summary:

```text
OpenCV    pose frames 75, near-ambiguous frames 45
sentai_sim pose frames 75, near-ambiguous frames 45
```

Interpretation:

- The diagnostic confirms that the current marker layout has many symmetric
  correspondence alternatives with essentially identical reprojection error.
- Many are 180-degree yaw alternatives with tiny translation delta, which is
  consistent with circular markers plus a symmetric marker arrangement.
- Some frames also expose IPPE-style planar alternatives.  These should be
  treated as a pose-disambiguation concern, not as detector recall failure.
- For flight/localization use, the solver should constrain correspondence by
  known marker layout, previous pose, positive-depth/above-ground checks, and
  continuity, then reject or flag frames where an alternate solution is too
  close.

## 7-Marker Layout Validation 2026-05-23

Updated B2 to follow the current B1 layout with seven WhyCon markers:

```text
NW (-0.08, +0.08, 0.005)
NE (+0.08, +0.08, 0.005)
W  (-0.06, +0.00, 0.005)
E  (+0.06, +0.00, 0.005)
SW (-0.08, -0.08, 0.005)
SE (+0.08, -0.08, 0.005)
N  (+0.02, +0.10, 0.005)
```

Implementation changes:

- `sim/scripts/run_sentai_sim_whycon_dataset.py` now serializes
  `config.json` `markers[*].world_xyz` into `sentai.markers.set_marker_world`.
- The sentai run summary records `marker_world_count`, `marker_world_rc`,
  marker IDs, and `marker_diameter_m`.
- Per-frame sentai output includes `drone_pose_world` from
  `sentai.markers.get_drone_pose_tuple(...)` for diagnostics.
- `sim/scripts/validate_whycon_synthetic_dataset.py` continues to compute the
  reportable camera pose for both OpenCV and sentai detections from the same
  manifest marker coordinates.
- Added `pose_max_reprojection_rmse_px = 0.5`.  Frames where PnP lands on a
  high-reprojection branch are marked pose-unavailable and excluded from
  X/Y/Z aggregate statistics.
- Pose ambiguity permutation search now supports up to seven visible markers.

Dataset:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_123126/
```

Sentai run:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_123126/sentai_sim_20260523_123720/
```

Validation run:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_123126/validation_20260523_124004/
```

Detection result:

```text
OpenCV     matched 154 / 154, recall 1.0, false positives 0
sentai_sim matched 154 / 154, recall 1.0, false positives 0
```

Pose result after reprojection gating:

```text
OpenCV     pose frames 28 / 30, translation RMSE 0.00915 m, p95 0.01841 m
sentai_sim pose frames 30 / 30, translation RMSE 0.00896 m, p95 0.01779 m
OpenCV-vs-sentai translation delta p95 0.01589 m over 28 comparable frames
```

Interpretation:

- The validator now uses the correct seven-marker geometry for count,
  centroid, and pose.
- The asymmetric marker reduces ambiguity but does not remove every weak-pose
  case.  Two OpenCV frames with four nearly edge-aligned markers produced a
  bad PnP branch; the reprojection gate prevents those bad camera positions
  from entering pose statistics.
- For a formal dataset, B1 should also avoid or label pose-degenerate
  four-marker edge cases if pose quality is the primary metric.

## Runtime Pose Assignment Options 2026-05-23

The B2 validator may use exhaustive correspondence/permutation search because
it is offline and diagnostic.  This should not be treated as the preferred
runtime strategy for real flight or high-rate simulation.

Reason:

```text
4! = 24
6! = 720
7! = 5040
```

With seven registered world markers, brute-force assignment can become the
dominant pose cost even though image detection remains mostly pixel-count
limited.  For real flight and real-time simulation, prefer a constrained
assignment strategy:

- Use previous pose as a prior: predict where each registered marker should
  project in the current frame, then match detections near those predictions.
- Use yaw/IMU/EKF as a prior: reject assignment branches whose recovered yaw
  or body-frame orientation disagrees with independent attitude estimates.
- Use nearest-neighbor over projected marker positions as the first candidate
  assignment, then fall back only if residuals are poor.
- Use Hungarian matching on the projected-marker vs detected-marker distance
  matrix instead of trying every permutation.
- Use early rejection on residual: while evaluating an assignment, stop as
  soon as the partial or final residual exceeds the current best/allowed
  threshold.
- Keep the layout asymmetric: the extra `N` marker reduces rotational
  ambiguity and makes nearest-neighbor/Hungarian assignment more reliable.

Recommended split:

- Offline B2 validation: keep exhaustive alternatives available so ambiguity
  is measurable and reportable.
- Simulated flight / real flight: use prior-guided assignment with
  reprojection residual gates, plus a fallback diagnostic mode for ambiguous
  frames.

## Large Batch Ablation 2026-05-23

Goal: generate a statistically more useful validation batch and rerun the full
OpenCV / sentai_sim ablation across detection, geometry, and camera pose
variables.

Requested target was 500 images.  With the current A1/B1 visibility policy
(`evaluation_visible >= 4`, cropped markers excluded from the expected count),
the deterministic grid produced 368 valid frames.  This is below 500, but it is
large enough for the next debugging pass and avoids adding low-quality
three-marker edge cases.

Source dataset:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_133537/
```

Dataset distribution:

```text
frames              368
marker_distribution {4: 73, 5: 56, 6: 56, 7: 183}
z_distribution      {0.30: 16, 0.50: 32, 0.75: 112, 1.00: 208}
roll_pitch          {(0.0, 0.0): 200, (4.0, -3.0): 168}
yaw_distribution    {0: 49, 45: 45, 90: 47, 135: 44,
                     180: 46, 225: 45, 270: 48, 315: 44}
```

Notes:

- First large generation attempt stopped at 170 / 368 frames because Gazebo
  `set_pose` timed out.  B1 generator now retries `set_pose` before failing.
- The successful batch generated all 368 frames and a contact sheet under the
  dataset `preview/` directory.
- The original `_small` world was not modified; the B1 generator injects the
  extra asymmetric marker into a run-local world.

Sentai run:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/sentai_sim_20260523_135732/
```

Sentai summary:

```text
frames                    368
frames_with_detections    368
detections_total          2251
frames_with_drone_pose    354
marker_world_count        7
marker_ids                NW, NE, W, E, SW, SE, N
```

Validation run:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/validation_20260523_141003/
```

Command:

```bash
./venv/bin/python sim/scripts/validate_whycon_synthetic_dataset.py \
  dataset/TD-S10-B1/whycon_gazebo_synth_20260523_133537 \
  --sentai-results dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/sentai_sim_20260523_135732/sentai_sim_results.jsonl \
  --skip-ambiguity
```

Reason for `--skip-ambiguity`: exhaustive ambiguity search for 7 visible
markers can become too slow for full-batch iteration.  It remains useful as a
targeted diagnostic on selected frames.

Detection summary:

| backend | frames | expected_markers | matched_markers | recall | false_positives_per_frame | complete_frame_success | centroid_p95_px |
| --- | --- | --- | --- | --- | --- | --- | --- |
| opencv | 368 | 2189 | 2189 | 1.000000 | 0.005435 | 0.994565 | 1.51118 |
| sentai_sim | 368 | 2189 | 2185 | 0.998173 | 0.051630 | 0.937500 | 1.50771 |

Grouped detection observations:

- OpenCV recall was 1.0 for every z stratum, expected-count group, yaw bin,
  roll/pitch bin, and edge category.
- sentai_sim missed 4 expected markers total.  Misses appeared only at `z=1.0`
  in this batch.
- sentai_sim false positives were higher than OpenCV: 19 total vs 2 total.
- Matched centroid geometry is effectively equivalent: OpenCV p95 centroid
  error 1.511 px, sentai_sim p95 centroid error 1.508 px.

Pose versus ground truth:

| backend | pose_valid_frames | translation_rmse_m | translation_p95_m | x_mae_m | y_mae_m | z_mae_m | roll_mae_deg | pitch_mae_deg | yaw_mae_deg | yaw_p95_deg |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| opencv | 364 | 0.128351 | 0.0410935 | 0.0219133 | 0.0114690 | 0.0071681 | 1.27338 | 0.62063 | 0.197608 | 0.221727 |
| sentai_sim | 367 | 0.0155885 | 0.0285586 | 0.0065462 | 0.0063278 | 0.0059818 | 0.395524 | 0.406824 | 0.089908 | 0.221193 |

Important interpretation:

- OpenCV pose is usually close: median translation error is 0.0092 m and p95 is
  0.0411 m.
- OpenCV translation RMSE is much worse than p95 because of a small number of
  approximately 1 m mirrored-branch pose outliers.
- These outliers occur in 4-marker edge/interior weak-geometry cases at
  `z=1.0`.  Several bad branches still have very low reprojection RMSE, so a
  reprojection-only gate cannot reject all of them.
- sentai_sim pose estimates are more robust on this batch: max translation
  error is 0.0672 m, p95 is 0.0286 m.

Largest OpenCV pose outliers:

| frame_id | expected | edge | translation_error_m | reprojection_rmse_px | image |
| --- | --- | --- | --- | --- | --- |
| 362 | 4 | interior | 1.010065 | 0.391666 | `frame_000362_x+0.500_y+0.000_z1.000_r+0.0_p+0.0_yaw270_n4.pgm` |
| 356 | 4 | interior | 1.006101 | 0.386139 | `frame_000356_x+0.500_y+0.000_z1.000_r+0.0_p+0.0_yaw090_n4.pgm` |
| 55 | 4 | near_edge | 1.003730 | 0.025205 | `frame_000055_x-0.500_y+0.250_z1.000_r+0.0_p+0.0_yaw090_n4.pgm` |
| 63 | 4 | near_edge | 1.000348 | 0.023539 | `frame_000063_x-0.500_y+0.250_z1.000_r+0.0_p+0.0_yaw270_n4.pgm` |
| 367 | 4 | interior | 0.963950 | 0.082552 | `frame_000367_x+0.500_y+0.250_z1.000_r+0.0_p+0.0_yaw270_n4.pgm` |
| 364 | 4 | interior | 0.960852 | 0.090500 | `frame_000364_x+0.500_y+0.250_z1.000_r+0.0_p+0.0_yaw090_n4.pgm` |

OpenCV versus sentai_sim deltas:

| comparable_frames | count_disagreement_frames | marker_delta_count | centroid_delta_p95_px | pose_delta_count | translation_delta_mean_m | translation_delta_p95_m | translation_delta_max_m | yaw_delta_p95_deg |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 368 | 21 | 2185 | 0.363643 | 363 | 0.027878 | 0.0396604 | 1.04283 | 0.20013 |

Conclusion for this batch:

- Detection parity is strong.  The two implementations agree at sub-pixel
  scale for matched marker centers.
- sentai_sim still needs false-positive cleanup and the exact miss cases
  should be inspected, but recall is already very high.
- The largest remaining validation risk is pose branch ambiguity in weak
  4-marker configurations, not raw marker centroid quality.
- The 7-marker asymmetric layout helps, but it does not help frames where only
  a weak 4-marker subset is visible.
- For report-quality pose statistics, publish both:
  - all evaluation-visible frames;
  - pose-conditioned subsets that separate 4-marker edge/near-edge weak
    geometry from 5-7 marker cases.
- For runtime flight/simulation, use the already listed prior-guided
  assignment tools: previous pose, yaw/IMU prior, projected nearest-neighbor or
  Hungarian matching, and early residual rejection.

Report-ready artifacts:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/validation_20260523_141003/report_tables/
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/validation_20260523_141003/summary.json
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/validation_20260523_141003/opencv_detection_contact_sheet.png
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/validation_20260523_141003/sentai_detection_contact_sheet.png
```

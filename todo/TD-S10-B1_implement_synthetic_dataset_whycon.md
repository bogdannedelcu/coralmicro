# TD-S10-B1 — Implement A1 Synthetic WhyCon Dataset Generator

## Purpose

Operational implementation log for `TD-S10-A1_create_synthetic_dataset_whycon.md`.
A1 remains the objective/specification document; B1 tracks concrete decisions,
implementation steps, commands, progress, and blockers.

## Current Decision

Use Gazebo only as a renderer for the WhyCon ground pad.  Do not spawn or fly
the Crazyflie.  Do not start cf2 SITL.  The dataset generator controls a
camera above the pad, captures frames at `320x240`, and writes all ground truth
metadata to `dataset/TD-S10-B1/<run>/manifest.jsonl`.

GUI must be active during capture so the operator can visually inspect what is
happening.

## Known Inputs

- World:
  `/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_whycon_small.sdf`
- Distrobox:
  `crazysim-garden`
- Camera image size:
  `320x240`
- Canonical detector input format:
  grayscale `P5` `.pgm`, one byte per pixel.  PNGs are preview artifacts only.
- Current sampling altitude policy:
  `z <= 1.00 m`
- Camera reference intrinsics from prior experiments:
  `fx=288.3`, `fy=288.3`, `cx=160.0`, `cy=120.0`
- Marker diameter for small pad:
  `0.0544 m`
- Marker world coordinates:
  - `NW = (-0.08, +0.08, 0.005)`
  - `NE = (+0.08, +0.08, 0.005)`
  - `W  = (-0.06, +0.00, 0.005)`
  - `E  = (+0.06, +0.00, 0.005)`
  - `SW = (-0.08, -0.08, 0.005)`
  - `SE = (+0.08, -0.08, 0.005)`

## Implementation Plan

1. Create a camera-only dataset world derived from `sentai_whycon_small.sdf`,
   with a controllable `dataset_camera` model and a `/dataset_cam/image` topic.
2. Create a dataset generator script that:
   - creates a timestamped folder under `dataset/`;
   - launches Gazebo with GUI enabled;
   - moves the dataset camera through a deterministic pose set;
   - captures one image per pose;
   - stores images losslessly as grayscale PGM;
   - writes `manifest.jsonl` and `config.json`;
   - supports a small smoke mode first.
3. Compute manifest ground truth from camera pose, intrinsics, marker geometry,
   and image bounds.
4. Run a smoke capture with GUI active and inspect that frames are non-empty
   and contain the WhyCon pad.
5. Leave full-grid generation as a documented command after smoke passes.

## Progress

- [x] Located canonical small WhyCon world in CrazySim external repo.
- [x] Confirmed no drone/cf2 is required for A1.
- [x] Created this B1 implementation log.
- [x] Create or generate camera-only temporary world.
- [x] Implement dataset generator script.
- [x] Run smoke capture with Gazebo GUI active.
- [x] Verify output folder, images, `manifest.jsonl`, and `config.json`.

## Implementation Artifacts

- `sim/scripts/generate_whycon_synthetic_dataset.py`
  - Generates a temporary `sentai_whycon_small_dataset` world.
  - Adds a `dataset_camera` model publishing `/dataset_cam/image`.
  - Launches Gazebo through `crazysim-garden` with GUI enabled.
- Captures canonical grayscale `P5` PGM frames at `320x240`.
- Writes `manifest.jsonl`, `config.json`, and `render_world.sdf`.
- Writes `preview/contact_sheet.png` plus per-frame PNG previews after each
  successful run, when Pillow is available.

## Notes

- Keep A1 unchanged unless the objective itself changes.
- Generated datasets belong under `dataset/TD-S10-B1/`.
- Ground truth is allowed in the dataset manifest, but must not be embedded
  into detector inputs.

## Smoke Run 2026-05-23

Command:

```bash
python3 sim/scripts/generate_whycon_synthetic_dataset.py --mode smoke --max-frames 3 --settle-s 0.3
```

Output folder:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_090221/
```

Status: INVALID / do not use as dataset.

Findings:

- Gazebo GUI launched and `/dataset_cam/image` appeared.
- The temporary world lived under `dataset/TD-S10-B1/.../_work`, so Gazebo resolved
  relative texture paths against that folder and failed to load
  `suburbs_bg.png`, `checker.png`, and `whycon_krajnik.png`.
- `set_pose` produced Gazebo errors: `Unable to update the pose for entity
  id:[0], name[]`, so the dataset camera was probably not moving between
  frames.
- The three generated PGM frames had identical min/max/mean, confirming the
  smoke output is not valid.

Next fix:

- DONE in script: stage a `materials` symlink/copy next to the generated
  temporary world and launch Gazebo with explicit `GZ_SIM_RESOURCE_PATH`.
- DONE in script: generate a temporary GUI config that subscribes Picture-in-
  Picture to `/dataset_cam/image` instead of `/downward_cam/image`.
- DONE in script: changed `set_pose` to resolve the Gazebo entity id with
  `gz model -m dataset_camera -p` and then send `id: <entity_id>` in the pose
  request.  Gazebo accepted `id: 45` during the smoke run.
- DONE: confirmed that textures appear, PiP subscribes to `/dataset_cam/image`,
  and frames differ across camera poses.

## Smoke Run 2026-05-23, Retry

Command:

```bash
python3 sim/scripts/generate_whycon_synthetic_dataset.py --mode smoke --max-frames 3 --settle-s 0.3
```

Output folder:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_090751/
```

Status: VALID smoke dataset.

Findings:

- Gazebo GUI launched with
  `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_090751/_work/sentai_dataset_gui.config`.
- Picture-in-Picture was configured as `WhyCon dataset camera (320x240)` and
  subscribed to `/dataset_cam/image`.
- `/dataset_cam/image` became ready and the camera model resolved to Gazebo
  entity id `45`.
- Captured three PGM frames at z heights `0.30 m`, `0.50 m`, and `0.75 m`.
- `manifest.jsonl` contains three entries, each with all six expected WhyCon
  markers visible and with projected centers/radii changing according to
  camera height.
- Frame hashes and statistics differ, confirming the camera moved:

```text
frame_000000... md5=baeec0d6b2fd5ff7ff9ff7bc1481cfc9 mean=109.37 std=58.20
frame_000001... md5=b3756b29fdadde5bbd6dba1517813a67 mean=98.44  std=42.45
frame_000002... md5=789af67dfa09d4c54f3e02b39e160135 mean=96.78  std=34.49
```

- Visual inspection of PNG previews confirms the intended content:
  - z `0.30 m`: six large WhyCon markers over the ground texture;
  - z `0.50 m`: six markers still clear, with more surrounding floor visible;
  - z `0.75 m`: six smaller markers visible, plus wider Gazebo scene context.
- Preview PNG files were generated only for inspection under:
  `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_090751/preview/`.
- The original Gazebo `_small` world was not modified.  The generator reads it
  and writes a run-local copy under `dataset/TD-S10-B1/.../_work/`.
- The smoke command is expected to stop after three frames because it was run
  with `--mode smoke --max-frames 3`.

Next step:

- Run a larger deterministic generation pass after visually confirming the GUI
  view is acceptable.

## Smoke Run 2026-05-23, Attitude Sweep

Code change:

- Updated `sim/scripts/generate_whycon_synthetic_dataset.py` so smoke and grid
  sampling use `z <= 1.00 m`.
- Reordered smoke poses so the first 10 frames include yaw, roll, and pitch
  cases instead of only vertical-height cases.
- The original Gazebo `_small` world remains untouched; only run-local copies
  are generated under `dataset/TD-S10-B1/.../_work/`.

Command:

```bash
python3 sim/scripts/generate_whycon_synthetic_dataset.py --mode smoke --max-frames 10 --settle-s 0.3
```

Output folder:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_091159/
```

Status: VALID attitude smoke dataset.

Findings:

- Captured 10 frames with GUI active.
- Altitudes covered: `0.30`, `0.50`, `0.75`, and `1.00 m`.
- Attitude and position variants included:
  - roll/pitch around `+3/-2`, `-3/+2`, `+5/0`, `0/-5`,
    `+7/-4`, `-7/+4` degrees;
  - yaw values `0`, `45`, `90`, `135`, `180`, and `225` degrees.
- Manifest reports 9 frames with all 6 markers visible and 1 edge-case frame
  with 5 expected markers.  In that frame, marker `E` is partial crop and
  marker `SE` is out of FOV, consistent with visual inspection.
- Preview PNGs were generated for inspection under:
  `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_091159/preview/`.
- Visual inspection confirms that frames show the six-marker WhyCon pad over
  the Gazebo ground texture; yaw/roll/pitch changes rotate and shift the pad
  plausibly in the image.

Next step:

- Decide whether edge-case frames with partial/out-of-FOV markers should be
  included in the first full dataset or separated into a stress-test subset.

## Sampling Policy Update 2026-05-23

Decision:

- Edge-case frames with 5 and 4 expected visible markers are valid and should
  be part of the regular dataset.
- Frames with exactly 3 expected visible markers are valid only as
  extreme/stress cases.
- The regular dataset lower bound is 4 expected visible markers.  Poses with
  fewer than 4 geometrically visible markers are filtered out of the regular
  grid dataset.
- Keep the current altitude ceiling at `z <= 1.00 m`.

Code changes:

- Updated A1 to replace the old `1.25` and `1.50 m` z strata with the current
  `0.30`, `0.50`, `0.75`, `1.00 m` policy.
- Expanded grid sampling in `sim/scripts/generate_whycon_synthetic_dataset.py`
  to `x,y in [-0.75, +0.75] m`.
- Added `REGULAR_MIN_EXPECTED_MARKERS = 4`,
  `STRESS_MIN_EXPECTED_MARKERS = 3`, and a geometric visibility filter for
  regular grid pose generation.
- Added a `visibility_policy` block to generated `config.json`.

Static sampler check, without launching Gazebo:

```text
raw_grid_poses        3136
filtered_poses_min3    464
distribution            {3: 74, 4: 82, 5: 60, 6: 248}
```

Superseded clarification:

- The check above used the old `min3` regular policy.  It is superseded by the
  later decision that regular frames require at least 4 markers and 3-marker
  cases belong only in stress subsets.

Updated static sampler check, after regular `min4` policy:

```text
raw_grid_poses          3136
filtered_regular_min4    390
distribution              {4: 82, 5: 60, 6: 248}
first30_distribution      {4: 8, 5: 6, 6: 16}
```

## 30-Frame Visual Inspection Set 2026-05-23

Command:

```bash
python3 sim/scripts/generate_whycon_synthetic_dataset.py --mode grid --max-frames 30 --settle-s 0.3
```

Output folder:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_091942/
```

Status: VALID visual smoke set.

Findings:

- Captured 30 frames with GUI active.
- Manifest distribution:

```text
frame_count          30
marker_distribution {4: 8, 5: 6, 6: 16}
z_distribution       {0.3: 16, 0.5: 14}
missing_images       0
```

- Generated preview PNGs for all frames and a contact sheet:
  `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_091942/preview/contact_sheet_30.png`.
- Visual inspection confirms that 4- and 5-marker frames are genuine edge/crop
  cases over the Gazebo ground texture, not empty or broken renders.
- Gazebo processes were stopped after inspection.

Clarification:

- This visual set covered only two altitude strata because `--max-frames 30`
  truncated the ordered grid before later z buckets were reached:

```text
z_distribution {0.3: 16, 0.5: 14}
```

- This is not representative enough for future 30-frame inspection sets.
  The generator now interleaves grid poses by z before applying
  `--max-frames`, so small subsets cover the full z range.

Updated static check for the next 30-frame grid subset:

```text
first30_z_distribution      {0.3: 8, 0.5: 8, 0.75: 7, 1.0: 7}
first30_marker_distribution {4: 12, 5: 8, 6: 10}
```

## Dataset Folder Organization

Decision:

- TD-S10-B1 datasets live under `dataset/TD-S10-B1/` so generated data from
  different todo items does not mix at the root of `dataset/`.

Changes:

- Moved existing `whycon_gazebo_synth_*` runs from `dataset/` into
  `dataset/TD-S10-B1/`.
- Updated `sim/scripts/generate_whycon_synthetic_dataset.py` so its default
  output root is `dataset/TD-S10-B1/`.
- Updated A1 output-location examples to show the task-specific dataset
  subfolder.

## 30-Frame Z-Interleaved Smoke Set 2026-05-23

Command:

```bash
python3 sim/scripts/generate_whycon_synthetic_dataset.py --mode grid --max-frames 30 --settle-s 0.3
```

Output folder:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439/
```

Status: VALID visual smoke set.

Findings:

- Captured 30 frames with GUI active.
- Grid poses are now interleaved by altitude before `--max-frames`, so the
  subset covers all intended z strata:

```text
frame_count          30
marker_distribution {4: 12, 5: 8, 6: 10}
z_distribution       {0.3: 8, 0.5: 8, 0.75: 7, 1.0: 7}
missing_images       0
```

- Generated contact sheet:
  `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_092439/preview/contact_sheet_30_z_interleaved.png`.
- Visual inspection confirms clear variation in marker scale and scene extent
  across `0.30`, `0.50`, `0.75`, and `1.00 m`.
- Gazebo processes were stopped after inspection.

Follow-up implementation:

- A1 now requires preview artifacts under `preview/`, with at least one contact
  sheet for quick visual inspection.
- The generator now automatically writes `preview/contact_sheet.png` and
  per-frame PNG previews after a successful run.
- Contact sheets use a small white gutter between tiles to avoid visual
  blending between adjacent frames.

## A1 Compliance Check

Status: PARTIAL / smoke implementation complete, full A1 dataset not complete.

Met:

- Creates timestamped dataset folders under `dataset/TD-S10-B1/`.
- Writes lossless grayscale `P5` `.pgm` frames under `frames/`.  These are the
  only detector inputs for A2/B2; generated `.png` files are for human
  inspection only.
- Writes one `manifest.jsonl` row per image.
- Every manifest `image_path` exists in the validated smoke run.
- Records `scene_id`, source world path, camera intrinsics, image resolution,
  exact pose per frame, marker world coordinates, projected marker centers,
  projected bbox, marker radius, and visibility classification.
- Uses implementation-agnostic data: no OpenCV, sentai_sim, MicroPython, or
  detector-specific API is required to consume the dataset.
- Keeps ground truth in metadata only, not in detector inputs.
- Documents the camera frame convention in `config.json`.
- Includes at least one all-visible frame and one partial/edge visibility
  frame in the 10-frame attitude smoke run.
- Uses Gazebo as renderer only, isolating perception from flight control,
  EKF, CRTP queues, and takeoff dynamics.
- Leaves the original Gazebo `_small` world unchanged; run-local copies live
  under `dataset/TD-S10-B1/.../_work/`.

Partially met:

- Z strata: A1 originally listed `0.30, 0.50, 0.75, 1.00, 1.25, 1.50 m`, but
  current working policy limits the useful range to `z <= 1.00 m` based on
  operator intuition.  This should be reflected in the final sampling policy.
- Sampling: deterministic smoke/grid exists, but the random/normal component
  with recorded seed is not implemented yet.
- `config.json` has core fields, but should be expanded before a final run
  with git commit, random seed, full pose ranges, grayscale conversion details,
  occlusion-modeling policy, and marker-size source.
- Smoke subset exists with 10 frames, but A1 recommends around 20 frames.

Not met yet:

- Full dataset generation has not been run.
- Original/full pad coverage has not been implemented or located; only the
  small pad world `sentai_whycon_small.sdf` is covered.
- A simple standalone verifier script has not been added yet.
- `config.json` is not yet sufficient to fully regenerate the final intended
  dataset because it records `mode` and frame count but not the exact pose
  list, seed, or all sampler ranges.

## 100-Frame Grid Dataset 2026-05-23

Command:

```bash
./venv/bin/python sim/scripts/generate_whycon_synthetic_dataset.py \
  --mode grid \
  --max-frames 100 \
  --settle-s 0.25
```

Output folder:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_114246/
```

Status: VALID 100-frame working dataset for A2/B2 ablation.

Findings:

- Captured 100 frames with Gazebo GUI active.
- Gazebo was stopped cleanly after capture.
- Every manifest `image_path` exists.
- Generated preview/contact sheet:
  `dataset/TD-S10-B1/whycon_gazebo_synth_20260523_114246/preview/contact_sheet.png`.

Manifest distribution:

```text
frame_count          100
marker_distribution {4: 28, 5: 18, 6: 54}
z_distribution       {0.3: 16, 0.5: 28, 0.75: 28, 1.0: 28}
missing_images       0
```

Notes:

- This is still deterministic grid sampling, not the final mixed
  deterministic+random sampler requested by A1.
- It is sufficient for the next B2 ablation step because it covers all Z
  strata and contains 4-, 5-, and 6-marker cases.

## Visibility Convention Update 2026-05-23

A1 now defines `markers_expected` as the number of fully visible markers only.
Markers whose projected outer circle leaves the image must be labeled
`partial_crop` and kept in the manifest for diagnostics, but they should not be
counted in the standard expected-marker count.

Important impact:

- Existing B1 datasets generated before this update, including
  `whycon_gazebo_synth_20260523_092439` and
  `whycon_gazebo_synth_20260523_114246`, counted `partial_crop` markers as
  visible/expected.
- Their previous B2 recall numbers are therefore useful for cropped-marker
  stress analysis, but not for the new default A2 standard metrics.
Implementation status:

- Updated `sim/scripts/generate_whycon_synthetic_dataset.py` so
  `partial_crop` markers now have `visible = false` and
  `evaluation_visible = false`.
- `markers_expected` and filename `_nX` now count only fully visible markers.
- Added `markers_partial_crop` per manifest row.
- Added `visibility_policy.markers_expected_definition` and
  `visibility_policy.partial_crop_counts_as_expected = false` to new
  `config.json` files.
- Existing datasets are not rewritten in place; regenerate to get manifests
  that fully follow the new A1 convention.

## Asymmetric 7-Marker Smoke 2026-05-23

Added one extra WhyCon marker to the generated dataset world to reduce
pose/correspondence ambiguity caused by the previous symmetric layout.

Implementation:

- The original external `_small` Gazebo world is still not modified.
- `sim/scripts/generate_whycon_synthetic_dataset.py` now injects the extra
  marker only into the run-local rendered world copy.
- Conceptual marker coordinate: `(0.25, 1.25)`.
- Small-world scaled coordinate in the generator: `whycon_N = (0.02, 0.10,
  0.005)`, using the existing mapping where conceptual `1.0` corresponds to
  `0.08 m`.

Command:

```bash
./venv/bin/python sim/scripts/generate_whycon_synthetic_dataset.py \
  --mode grid \
  --max-frames 30 \
  --settle-s 0.3
```

Output folder:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_123126/
```

Smoke summary:

```text
frame_count          30
marker_distribution {4: 13, 5: 6, 6: 5, 7: 6}
z_distribution       {0.3: 8, 0.5: 8, 0.75: 7, 1.0: 7}
contact_sheet        preview/contact_sheet.png
```

Visual inspection:

- The new marker is visible in all-visible frames and the manifest records
  seven markers.
- The smoke still covers 4-, 5-, 6-, and 7-marker cases across all Z strata.
- This dataset is the current candidate for rerunning the B2 ambiguity
  diagnostic after the asymmetric layout change.

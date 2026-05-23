# TD-S10 — Create Synthetic Gazebo Dataset for WhyCon

## Goal

Create a reproducible synthetic image dataset from Gazebo for evaluating
WhyCon marker detection and pose estimation.  The dataset must be independent
of any one implementation so it can be used by both a local OpenCV reference
pipeline and the `sentai_sim` C++ implementation.

This task is diagnostic only: it should isolate the perception layer from
flight control, EKF coupling, CRTP packet queues, and takeoff dynamics.

## Output Location

Store generated data under a task-specific subfolder in `dataset/`, using one
subfolder per dataset run:

```text
dataset/
  TD-S10-B1/
    whycon_gazebo_synth_YYYYMMDD_HHMMSS/
      manifest.jsonl
      config.json
      frames/
        frame_000000_x+0.000_y+0.000_z0.500_r+0.0_p+0.0_yaw000_n6.pgm
        frame_000001_x+0.050_y-0.100_z0.750_r+1.2_p-2.0_yaw045_n6.pgm
      preview/
        contact_sheet.png
        optional_debug_overlay_000000.png
```

Detector input images must be stored as grayscale `P5` `.pgm` at `320x240`.
This is the canonical dataset format for A1/B1 because it matches the
single-channel input expected by the embedded WhyCon pipeline and avoids
decoder/color-conversion differences between validators.

PNG files are allowed only as derived preview/debug artifacts under
`preview/`; they must not be treated as canonical detector inputs.

Each dataset run should also include human-inspection artifacts under
`preview/`.  At minimum, generate a contact sheet that shows all smoke frames
or a representative subset of large datasets.  The contact sheet should label
each tile with frame id, altitude, and expected marker count so later reviews
can quickly spot broken rendering, wrong camera pose, missing textures, or
poor coverage across z strata.

## Dataset Scope

Generate camera views looking down at the WhyCon marker pad in Gazebo.
Cover both the original/full pad and the small 0.5x pad if both are available.
Each pad/world should produce its own dataset subfolder or its own `scene_id`
field in the manifest.

The dataset must record:

- Gazebo world or scene identifier.
- Camera intrinsics and image resolution.
- Camera pose distribution and the exact sampled pose for every frame.
- Deterministic random seed if random sampling is used.
- Ground truth marker world coordinates.
- Ground truth projected marker centers in image pixels.
- Ground truth visibility classification.

Marker layouts should avoid exact rotational/reflection symmetry where
possible.  If the canonical small pad uses symmetric WhyCon circles, add or
select an asymmetric marker position, for example the proportional conceptual
coordinate `(0.25, 1.25)`, so pose estimation can distinguish otherwise
near-identical correspondence permutations.

## Visibility Convention

For standard A2 detection and pose metrics, only markers whose full projected
outer circle is inside the image are counted as expected markers.

Use the known marker world coordinates, camera pose, camera intrinsics, and
marker outer diameter to project each marker into the image.  Compute the
projected center, projected outer radius, and projected bbox:

```text
x0 = u - r
y0 = v - r
x1 = u + r
y1 = v + r
```

Visibility classes:

- `visible`: marker center is in front of the camera and the full projected
  outer circle bbox is inside the image bounds.
- `partial_crop`: marker center is inside the image but some part of the outer
  circle bbox leaves the image bounds.
- `out_of_fov`: marker center is outside the image.
- `behind_camera`: marker is behind the camera optical plane.
- `too_small`: projected outer radius is below the dataset minimum.
- `occluded`: reserved for future occlusion-aware render/geometry checks.

`markers_expected` must count only `visible` markers, not `partial_crop`
markers.  Partial/cropped markers should remain in the manifest with their
projected geometry and `visibility_reason = "partial_crop"` so they can be
used for stress diagnostics, but they are excluded from standard recall,
complete-frame, centroid, scale, and pose metrics.

If a stress dataset intentionally evaluates cropped-marker recovery, it should
write a separate metric such as `markers_partial_crop` or use an explicit
validation variant.  Do not mix cropped-marker recovery into the default
expected-marker count.

## Sampling Strategy

Use a mixed deterministic plus random design.  Pure random sampling can miss
edge cases; pure grid sampling over-represents tidy poses.

Recommended baseline:

- XY grid centered on world origin: start with `x,y in [-0.75, +0.75] m`,
  step `0.10` or `0.25 m`, then keep regular frames where at least 4 markers
  are expected visible.  The wider grid is intentional: the dataset should
  include 6-, 5-, and 4-marker cases, not only tidy all-visible poses.
- Z strata: `0.30, 0.50, 0.75, 1.00 m`.
- Roll/pitch: normal distribution around 0, e.g. sigma `2-5 deg`, clipped at
  `10-15 deg`.
- Yaw: uniform over `0-360 deg`, plus fixed canonical values
  `0, 45, 90, 135, 180, 225, 270, 315`.
- Add stress cases where markers are near image edges or partially cropped.
  Frames with 5 or 4 fully visible markers are valid and useful in the regular
  dataset.
  Frames with exactly 3 markers are valid only as extreme/stress cases and
  should be identifiable as such by dataset subfolder, scene variant, or
  manifest/config metadata.

For every generated frame, store the actual pose used.  Do not infer pose from
the filename during validation; the filename is only a human-readable hint.

## Ground Truth Manifest

Write one JSON object per frame to `manifest.jsonl`.  Required fields:

```json
{
  "frame_id": 0,
  "image_path": "frames/frame_000000_x+0.000_y+0.000_z0.500_r+0.0_p+0.0_yaw000_n6.pgm",
  "scene_id": "sentai_whycon_world",
  "camera": {
    "resolution": [320, 240],
    "fx": 288.3,
    "fy": 288.3,
    "cx": 160.0,
    "cy": 120.0
  },
  "pose_world_camera": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.5,
    "roll_deg": 0.0,
    "pitch_deg": 0.0,
    "yaw_deg": 0.0
  },
  "markers_expected": 6,
  "markers": [
    {
      "marker_id": "NW",
      "world_xyz": [-0.16, 0.16, 0.005],
      "visible": true,
      "visibility_reason": "visible",
      "pixel_center": [82.4, 57.1],
      "pixel_radius_outer": 31.2,
      "projected_bbox": [50.8, 25.4, 114.0, 88.8]
    }
  ]
}
```

Visibility reasons should be stable strings:

- `visible`
- `out_of_fov`
- `behind_camera`
- `too_small`
- `partial_crop`
- `occluded`

If Gazebo does not provide occlusion information directly, compute geometric
visibility from the known marker plane and camera projection.  Be explicit in
`config.json` about whether occlusion is modeled or not.

## Config File

Write a `config.json` next to the manifest.  Include:

- Dataset creation method and git commit if available.
- Gazebo world name/path.
- Dataset timestamp.
- Random seed.
- Camera intrinsics and resolution.
- Pose sampling ranges.
- Marker size definition, especially the WhyCon outer diameter.
- Marker world coordinates.
- Any image post-processing: grayscale conversion, compression, resize,
  thresholding, blur, noise, exposure settings.

## Dataset Principles

- The rendered images are the only perception input.  Ground truth belongs in
  the manifest and must not be embedded into images or detector inputs.
- Camera frame conventions must be documented clearly enough that independent
  validators can interpret pose and projection in the same way.
- The dataset should preserve the visual conditions that matter for flight:
  resolution, grayscale conversion to canonical `P5` PGM, lighting, shadows,
  texture, marker scale,
  and camera orientation.
- If additional variants are generated, separate them by dataset subfolder or
  `scene_id`: for example clean lighting, realistic lighting, original pad,
  and small 0.5x pad.
- Use deterministic filenames and deterministic manifest ordering.
- Include a small smoke subset, e.g. 20 frames, so validators can be tested
  without processing the full dataset.

## Acceptance Criteria

- A dataset folder is created under `dataset/`.
- `manifest.jsonl` has one valid JSON object per image.
- Every `image_path` in the manifest exists.
- The frame count matches the planned pose count.
- At least one frame covers each Z stratum.
- At least one frame has all markers visible.
- At least one frame has 5 expected fully visible markers.
- At least one frame has 4 expected fully visible markers.
- At least one frame has partial/edge visibility.
- No regular dataset frame should have fewer than 4 expected fully visible
  markers.
- Optional stress subsets may include exactly 3 expected fully visible
  markers, but not fewer.
- `config.json` is sufficient to regenerate the dataset.
- A simple verifier can read all images and all JSON lines without errors.
- Every canonical frame is a grayscale `P5` `.pgm` image at `320x240`; any
  PNG is inspection-only.
- A contact sheet or equivalent preview artifact exists under `preview/` for
  quick visual inspection.
- The dataset is implementation-agnostic: no required field depends on OpenCV,
  MicroPython, `sentai_sim`, or any detector-specific API.
- The manifest contains enough information to evaluate count, centroid, scale,
  and pose errors without running Gazebo again.

## Open Questions

- Which Gazebo world is canonical for the original WhyCon pad, and what is the
  exact path/name for the small 0.5x pad if it still exists outside the repo?
- Are marker diameters derived from SDF box size, PNG texture geometry, or a
  calibrated physical marker-size value?  Choose one canonical source and
  record it in `config.json`.
- Should images include Gazebo lighting/shadows exactly as flight, or should
  there also be a clean/no-shadow variant?

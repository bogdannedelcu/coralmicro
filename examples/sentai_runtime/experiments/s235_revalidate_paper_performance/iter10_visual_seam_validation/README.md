# Iter10 - Visual Seam Validation

Build: `SentAI v1.0 build 1544 (2026-06-24 11:07:59)`

Scope:

- capture a small visual artifact set for `drain=1` and `drain=2`;
- validate content using the first vertical bar / left-column signature;
- do not use JPEG save time as a performance metric.

Important measurement note:

`sentai.camera.save_jpeg()` perturbs timing heavily because FileX writes can
take hundreds of milliseconds to seconds.  The JPEGs here are visual artifacts
only.  Timing/FPS conclusions remain from the `peek5_b40()` and pipeline
wall-clock runs.

## Method

The useful visual discriminator is not whole-image structure.  In the synthetic
camera patterns, the first vertical bar is:

- bright/white for the cam0 BARS pattern;
- dark/black for the cam1 alternate pattern in the captured artifact set.

The host-side classifier samples the left bar at `x=40` across nine vertical
positions:

- all bright -> `LEFT_WHITE`;
- all dark -> `LEFT_DARK`;
- both bright and dark -> `LEFT_MIXED`, a visible vertical seam / mixed source.

## Results

Controlled BARS/HBAND run (`run01`, 2 frames per drain):

| drain | frames | correct | mismatch | mixed |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 2 | 2 | 0 | 0 |
| 2 | 2 | 1 | 1 | 0 |

Detailed left-column verdicts:

| file | drain | tag | left-column verdict | expected | result |
| --- | ---: | ---: | --- | --- | --- |
| `frames/run01_d1_00.jpg` | 1 | 0 | `LEFT_WHITE` | `LEFT_WHITE` | correct |
| `frames/run01_d1_01.jpg` | 1 | 1 | `LEFT_DARK` | `LEFT_DARK` | correct |
| `frames/run01_d2_00.jpg` | 2 | 0 | `LEFT_DARK` | `LEFT_WHITE` | mismatch |
| `frames/run01_d2_01.jpg` | 2 | 1 | `LEFT_DARK` | `LEFT_DARK` | correct |

The follow-up BARS-vs-real run (`run02`) is retained as visual context but is
less controlled: a real-scene frame can naturally contain bright/dark vertical
variation, so it is not the primary verdict source.

Conclusion: the visual artifacts agree with the `peek5_b40()` probe.  For
continuous `ratio(1,1)`, `drain=1` is visually/content-correct on this build,
while `drain=2` can produce tag/content phase errors.  This does not mean
`drain=2` is always invalid for manual or batched switching; it is invalid for
the continuous every-frame alternation path.

Artifacts:

- `run_visual_seam.log` - initial `camera.jpeg()` attempt; reset after two
  frames, retained as failed protocol evidence.
- `run_visual_seam_savejpeg.log` - controlled BARS/HBAND JPEG capture.
- `run_visual_seam_bars_real.log` - secondary BARS/real-scene capture.
- `run01_manifest.csv`, `run02_manifest.csv`
- `visual_seam_verdict.csv`, `visual_seam_verdict.json`
- `left_column_verdict.csv`, `left_column_verdict.json`
- `frames/*.jpg`

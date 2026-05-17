# s159 — sentai.aruco synthetic smoke (OP-S6-W3-T7)

WBS: `OP-S6-W3` — on-board ArUco detector + PnP, fresh re-implementation
per `[[no-broken-branch-test-reuse]]` (the legacy `sentai_aruco_shim`
files from branch `feature/ov5640-camera-support` were left there
because the integration branch reverted to a known-good state; design
ideas reused, code written fresh).

## SOTA basis (no "wheel reinvention")

Per CLAUDE.md compute-in-C principle + operator audit:

| Pipeline stage | SOTA reference |
|---|---|
| Adaptive box-mean threshold | Bradley & Roth 2007; integral image Viola & Jones 2001 |
| 4-connected component labelling (flood fill via explicit stack) | Rosenfeld & Pfaltz 1966 |
| Bit-pattern sampling with bilinear interpolation + 3×3 majority | Garrido-Jurado et al. *Pattern Recognition* 2014 (ArUco) |
| 4-rotation hamming dictionary match | standard ArUco (Garrido-Jurado 2014) |
| Pose from 4 image corners — DLT homography → R\|t decomposition | Hartley & Zisserman, *Multiple View Geometry* (2003) §8.1 |
| Rodrigues rvec ↔ R | classical Lie-group exp/log |

**Known SOTA gaps** (left as TODO for follow-up WPs, not in OP-S6-W3 MVP):
- Quad-corner extraction via x±y extremes is a heuristic (works for
  cooperative markers).  Real SOTA = Douglas-Peucker polygon
  simplification on the contour pixels.  Brittleness shows up under
  oblique views or partial occlusion.
- PnP is DLT, NOT IPPE (Collins & Bartoli IJCV 2014).  IPPE is the
  current SOTA for planar markers — analytically gives two pose
  candidates with proven near-degeneracy stability.  DLT is older
  and less stable when the marker is near-parallel to the image plane.
- 4×4_50 dictionary is currently an 8-entry placeholder table
  (synthetic test patterns); the real OpenCV 4×4_50 50-entry table
  must be loaded for Gazebo validation.  PNG-to-dict-bits helper
  script is the next deliverable.

## What this experiment proves

12 gates, all synthetic, all in C — the camera frame buffer never
crosses the MP binding boundary (per CLAUDE.md compute-in-C rule).
The driver calls `sentai.aruco._test_synth_and_detect(id, side, rot)`
which generates a 320×240 frame internally + runs detect + caches the
result; MP only sees the resulting marker dictionaries.

| Gate | Behaviour | Threshold |
|---|---|---|
| T1 | Detect id=0 at canonical rotation, 96 px marker | id=0 ∧ hamming=0 ∧ reproj < 1 px ∧ tvec_z ∈ [0.24, 0.26] m |
| T2.{1,2,3} | Detect id=0 at 90°/180°/270° CW rotation | same as T1 |
| T3.{a,b,c} | Detect ids 1, 2, 3 from the placeholder dictionary | id correctly identified |
| T4.{a,b} | Detect at 48 px and 120 px marker sides | tvec_z scales as `marker_size * fx / pixel_side` |
| T5 | Counters advance | frames_total ≥ 9, markers_total ≥ 9 |
| T6 | `clear()` empties cache | `get_latest()` returns `[]` |
| T7 | `rvec_to_R` and `R_to_rvec` round-trip on (0.3, -0.4, 0.5) | max element diff < 1e-5 |

## How to run

```bash
bash examples/sentai_runtime/experiments/s159_aruco_smoke/run.sh
```

PASS iff `[s159] OVERALL PASS` appears AND every `[s159] Tn` reports
`PASS`.

## What this does NOT prove

- Live Gazebo frame validation — deferred until the real OpenCV
  4×4_50 dictionary is loaded (PNG-to-bits helper TODO).
- Robustness to noise, motion blur, partial occlusion — single
  noise-free synthetic frame only.
- ARM-side timing — Phase 1 builds clean on ARM but the camera
  zero-copy hook is SIM-only; ARM bring-up = separate WP after CSI
  pipeline integration.
- Numerical stability near degeneracy (marker parallel to image
  plane) — DLT will get noisy here; IPPE upgrade is the fix.

## Cross-references

- Algorithm: `examples/sentai_runtime/sentai_aruco.{h,cc}` —
  reference comments cite Bradley-Roth, Viola-Jones, Garrido-Jurado,
  Hartley-Zisserman per stage.
- Memory: `[[op-s6-w3-aruco-shipped]]` (next pass).
- Consumer (deferred): `OP-S6-W1-T7` / `EXP-s158` — mission_s153
  integration of takeoff calibration via `sentai.calib` fed by
  `sentai.aruco.detect_from_camera()`.

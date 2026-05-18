# s177 — Corner subpixel refinement prototype (real captured frames)

## Goal

Validate that **subpixel corner refinement** (Förstner / OpenCV
`cornerSubPix`) cuts per-marker PnP Z noise on REAL rendered frames
captured during a yaw rotation.  s176 already proved our PnP is
rotation-invariant for clean corners — so the s174 altitude drift is
upstream of PnP, in corner detection.

Operator 2026-05-18: validate in Python first; only then port to C
and decide on CMSIS-DSP / SIMD optimizations for ARM.

## Inputs

Two real PGMs captured during s174 trial fr_20260518_171833:

- `frame_horiz_n4.pgm` — markers axis-aligned in image, sentai_aruco
  on-board detected n=4
- `frame_rot45_n1.pgm` — markers rotated ~45° in image (drone yawed
  partway through hover), sentai_aruco on-board detected only n=1

## Pipeline

1. cv2.aruco detect on BOTH frames, two passes each:
   - `CORNER_REFINE_NONE`  — corners as found by contour finder
   - `CORNER_REFINE_SUBPIX` — Förstner-style sub-pixel iteration
2. For each detected marker, compute:
   - 4 corner shifts (refined − unrefined), in pixels
   - PnP via `SOLVEPNP_IPPE_SQUARE` (reference algorithm)
   - tvec[2] = depth from camera to marker
3. Report:
   - n_detected per (frame × refinement)
   - max/mean corner shift (px)
   - per-marker z (m), per refinement pass
   - dz across passes per marker (mm)
4. Save side-by-side visualizations with corner overlay.

## Pass criterion

If subpixel refinement
- changes corner positions by > 0.2 px on average, AND
- changes per-marker PnP z by > 1 mm,

then we've confirmed the corner-noise hypothesis is the real T16
altitude drift root cause, and porting subpixel refinement to
`sentai_aruco.cc` is justified (= T18-C in the WBS).

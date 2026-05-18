# s176 — Findings

Date: 2026-05-18.  Run on commit pre-T18-A revert.

## Setup

- One captured frame `frame_original.pgm` (320x240, 4 markers visible)
- Markers detected ONCE by `cv2.aruco` (4/4 ok)
- For θ = 0°, 5°, 10°, …, 90°: rotate the 4 corner pixel coords
  algebraically around image center; do NOT warp the image
- Run three PnP variants on the rotated corners:
  - **REF**:   `cv2.solvePnP(SOLVEPNP_IPPE_SQUARE)` — OpenCV ref
  - **OURS1**: our homography decomposition (1 solution)
  - **OURS2**: OURS1 + R_180(v) second-solution pick by reproj

## Result

```
algo  | max |Δz vs θ=0| across markers (mm)
------+-----------------------------------------
REF   | 0.00   [PASS]
OURS1 | 0.00   [PASS]
OURS2 | 0.00   [PASS]
```

All three are **perfectly rotation-invariant** when given clean
corner inputs.  `OURS2` never picked Solution 2 (Solution 1 always
won on reproj err) — confirms there is no IPPE ambiguity to
disambiguate in our scene (marker plane is approximately parallel
to the image plane, both IPPE solutions collapse).

## Interpretation

The 26 mm Δz reported by `s175` came **100% from corner-detection
noise** introduced by `cv2.warpAffine`'s bilinear interpolation on
the rotated image.  PnP math is not at fault.

This means:

1. **A (IPPE 2-solution)** in `sentai_aruco.cc` is currently a
   functional no-op: the second branch never wins.  The code is
   correct, just unused in our scene geometry.  Future-proofing
   only.

2. **B (median + outlier reject)** in the VPE forwarder is still
   useful: per-marker corner noise differs (each marker is at a
   different image position, different aliasing pattern), so a
   median across the 4 PnP estimates smooths out outliers
   that the mean would propagate.

3. **C (subpixel corner refinement)** is the actual root-cause fix
   for the s174/T16 altitude drift.  When the rendered camera
   frame shows the markers at different in-plane rotations, the
   contour-extracted corners drift sub-pixel; this propagates to
   PnP Z.  `cv2.aruco` has `CORNER_REFINE_SUBPIX` (Förstner /
   gradient-based); we should port the equivalent to
   `sentai_aruco.cc`.

## Next

- s177: validate C (subpixel refinement) in Python on a sequence
  of rendered yaw frames before porting.
- Revert / keep T18-A in C: operator decides.
- T18-B remains pending; can ship independently of T18-A.

# s177 — Findings (T18-C subpix + T18-D/E DP extractor)

Date: 2026-05-18.  Real captured frames from s174 yaw mission
`fr_20260518_171833`.

## Summary

| Frame              | Pre-T18  | Post-T18-C/D/E | Post-T18-F/G | cv2.aruco ref |
|--------------------|----------|----------------|--------------|----------------|
| `frame_horiz_n4`   | 4/4 ✓   | 4/4 ✓         | 4/4 ✓       | 4/4           |
| `frame_rot45_n1`   | 0–1/4 ✗ | 3/4           | **4/4 ✓**   | 4/4           |

Final: **100 %** detection at 45° rotation.  The decisive fix was
switching connected-component flood-fill from 4- to 8-connectivity
(matches cv2.connectedComponents default).  A 45°-rotated marker's
black border ring touches itself only diagonally between adjacent
rows, so 4-conn fragmented it into 6+ pieces all below the
SENTAI_ARUCO_MIN_QUAD_AREA=256 gate; 8-conn keeps it as one big
component.

## Mechanism

The legacy `aruco_extract_quad_legacy` picked corners as bbox-axis
extrema (min/max of x+y and x-y).  This matches the true marker
corners only for axis-aligned markers; for rotated markers the
extrema are 1–3 px off the true diamond vertices.  Imprecise corners
propagate to (a) the bit decoder (samples land on the wrong cells)
and (b) the PnP residual gate (reproj 5–10 px instead of < 1 px).

The new `aruco_extract_quad` ported into `sentai_aruco.cc`:

1. **Moore-Neighbor 8-connected border trace** of the labeled
   component, starting at the top-left boundary pixel.
2. **Iterative Douglas-Peucker** simplification (eps swept over
   [0.02..0.10] × perimeter until the polygon has exactly 4 vertices).
3. **Förstner sub-pixel corner refinement** (T18-C, already ported
   byte-for-byte from cv2.cornerSubPix — match to 0.0000 px on
   `our_subpix_pure.py`).
4. Reorder + PnP as before.

Douglas-Peucker is ported from OpenCV's
`modules/imgproc/src/approx.cpp` (Intel Corporation 2000,
BSD-3-Clause).  Moore-Neighbor is Wikipedia-grade with no copy.

## The 4th marker on rot45 — root cause (T18-F)

**Bit decoder fails — needs cv2 `_extractBits` port.**

We instrumented the bit decoder.  Every dict-fail produced a
pattern like `0xFFFF` (all 16 inner cells black) or similar
high-density patterns.  Hamming distance to the dictionary is
7–10 bits (we initially mis-reported 3 due to a bug — the
`best_hamm` initialiser was set to `MAX_HAMMING + 1 = 3` so any
larger hamming kept the initial value; fixed by setting it to
17 = strictly above any possible value).

The 0xFFFF / mostly-black patterns mean the cell sampling grid
landed on the black border ring of the marker (which is always
black by ArUco convention) instead of on the inner data cells.
The corners ARE accurate to sub-pixel (T18-C confirms that), but
the per-cell bilinear sampling at fractional alpha/beta in the
ORIGINAL image is sensitive to small remaining quad-shape errors.

**cv2.aruco's approach** (`modules/objdetect/src/aruco/aruco_detector.cpp:324`):

```cpp
Mat transformation = getPerspectiveTransform(corners, resultImgCorners);
warpPerspective(_image, resultImg, transformation,
                Size(resultImgSize, resultImgSize), INTER_NEAREST);
threshold(resultImg, resultImg, 125, 255, THRESH_BINARY | THRESH_OTSU);
// then count white pixels in each cell with cellMarginRate=0.13
```

I.e. warp the grayscale image to a canonical N×N square FIRST,
then Otsu, then per-cell majority on the warped binary.  This is
robust to (1) perspective distortion, (2) lighting (Otsu adapts),
(3) sub-pixel corner errors (cells are large in canonical space
so a 1-px corner error is < 10 % of cell width).

Our approach (bilinear corner-interpolation + sub-pixel sample of
the original image) is fundamentally less robust.  T18-F is the
plan to port cv2's strategy.

## What's in tree (changes that compile cleanly on SIM)

- `sentai_aruco.cc`: new `aruco_refine_corner_subpix` (T18-C),
  new `aruco_extract_quad` with Moore-Neighbor + DP (T18-D/E),
  legacy version renamed to `aruco_extract_quad_legacy`,
  `best_hamm` init fix (17 not 3).

## Files

- `prototype.py` — original Python sweep (s176-style on s175 frame)
- `our_subpix_pure.py` — pure-Python cv2.cornerSubPix port,
  matches reference to 0.0000 px
- `frame_horiz_n4.pgm`, `frame_rot45_n1.pgm` — operator-provided
  real captures from s174 fr_20260518_171833
- `c_vs_python.sh` — C-side (sentai_sim) vs Python reference
- `c_port_design.md` — early design notes
- `FINDINGS.md` — this file

## Next

- **T18-F**: port `_extractBits` (perspective warp + Otsu).
  Expected to close the rot45 4/4 gap.
- T18-B (median + outlier reject in VPE forwarder): code already
  written, blocked on ARM build (pre-existing T13/T16 link issues).
- HW bench T18-C on real M7 before merge (task #51).

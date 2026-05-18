# s178 — cv2 vs ours per-stage dump on frame 133

Date: 2026-05-18.  Per-stage instrumentation findings.

## Frame 133 — cv2.aruco baseline

cv2.aruco (with CORNER_REFINE_SUBPIX) detects 4/4 markers at scales 23, 51, 101, 201.
Marker corners (post-subpix):
- id=0: (212.96,169.01)(192.75,166.83)(195.68,146.65)(215.42,149.92)
- id=1: (218.12,116.98)(198.63,113.99)(201.57, 94.66)(221.25, 96.93)
- id=2: (131.16,106.70)(111.68,104.07)(113.93, 84.79)(134.29, 86.99)
- id=3: (125.02,159.38)(105.66,156.31)(107.93,136.78)(128.17,139.71)

## Our pipeline — frame 133

All 4 markers DECODE successfully (mid=0, 1, 2, 3 with hamm 0-1).
Bit decoder works.  Pipeline failure is at the **PnP reproj gate (3.0
px)**.

| id | reproj | tvec_z (m) | Δcorners vs cv2 (max px) |
|----|--------|------------|--------------------------|
| 0  | 6.998  | 0.740      | ~1.2                     |
| 1  | 3.984  | 0.826      | ~1.2                     |
| 2  | 6.552  | 0.812      | ~1.5                     |
| 3  | 2.121  | 0.992      | ~0.3 ✓                   |

id=3 reproj = 2.121 px ≤ 3.0 → PASSES gate.
ids 0, 1, 2 reproj > 3.0 → REJECTED.

True drone altitude is ~1.0 m (per id=3 tvec_z).  ids 0, 1, 2 give
z=0.74-0.83 m → 17-26 % low.  This is consistent with the
1-1.2 px corner imprecision producing biased perspective: PnP
"sees" the marker as larger-than-real → closer than real.

## Root cause

Our subpix refinement (T18-C, win_half=2) doesn't converge as tightly
as cv2.cornerSubPix.  Specifically, the **safety revert** triggers
when subpix would move a corner > win_half = 2 px → corner stays at
its (less accurate) DP-extracted integer-pixel position.

cv2 starts from `findContours + approxPolyDP` corners, which are
SUB-pixel accurate via fitted-line intersection.  We start from
`Moore-Neighbor + DP` corners, which are INTEGER pixel.  Subpix has
to refine more aggressively for ours.

## Existing port found

`jcmellado/js-aruco` (MIT, JavaScript, ~600 GitHub stars overall in
deps) — from-scratch ArUco detector, NO OpenCV dependency, 50 KB
total.  Implementation files:
- aruco.js  — top-level pipeline
- cv.js     — adaptiveThreshold, findContours, approxPolyDP,
              isContourConvex, warp, otsu, threshold
- posit*.js — POSIT pose estimation (we use IPPE instead)
- svd.js    — Jacobi SVD for posit

Notable techniques we don't have:
1. **Explicit CW corner winding** via cross product
   `(c1-c0) × (c2-c0) < 0 → swap c[1] and c[3]`.  Added in this
   session as T18-K; no detection-rate change (Moore-Neighbor was
   already producing CW).
2. **`notTooNear` dedup**: drop overlapping candidates by 4-corner
   euclidean distance (minDist param).  Avoids multi-detect of same
   marker at adjacent scales.
3. **Border-zero check**: enforce that the OUTER border of the
   warped 7x7 patch is BLACK before bit decode.  Stronger than cv2's
   `_getBorderErrors`.
4. **Findcontours + approxPolyDP with sub-pixel line intersection**.
   This is the key — js-aruco's contour pixels and DP corners are
   FLOAT positions, computed from fitted edge lines.  Our
   Moore-Neighbor produces only integer-pixel positions.

## Path forward

The detection-rate gap (40% vs 92%) is primarily caused by our
**integer-pixel contour corner extraction** vs cv2/js-aruco's
**sub-pixel float corner extraction via edge-line intersection**.

Subpix refinement on top of integer corners moves them closer to true
position but with safety-revert at win_half=2px, some corners get
"stuck" 1-2 px off → PnP reproj high → reject.

**T18-M (next session)**: Port js-aruco's findContours + approxPolyDP
that produce SUB-PIXEL float vertices from the start.  No safety
revert needed since starting corners are already accurate.

Alternatively / additionally:
- **T18-N**: raise reproj gate to 8 px (looser) to accept slightly-
  off detections.  Risk: more false positives.  Recommend pair with
  T18-L (_getBorderErrors) for noise filtering.
- **T18-O**: replace Moore-Neighbor + DP with js-aruco's
  findContours + approxPolyDP+lineFit.

## Files

- `frame_133.pgm` — the test frame
- `FINDINGS.md` — this file

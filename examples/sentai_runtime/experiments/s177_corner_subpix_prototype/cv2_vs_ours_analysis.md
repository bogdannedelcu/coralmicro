# Analysis: sentai_aruco vs cv2.aruco — full pipeline diff

Date: 2026-05-18.  Triggered by operator's observation that s174
yaw mission detects only 142/356 frames (40 %) at 4/4 while cv2.aruco
catches 326/356 (92 %).  Investigation on frame 133
(`t203309164_n1_f000133.pgm`).

## Frame 133 diagnostic

cv2.aruco sees 4 markers at:
- id=0: bbox (193-215, 147-169) center (204, 158)
- id=1: bbox (199-221,  95-117) center (210, 105)
- id=2: bbox (112-134,  85-107) center (122,  95)
- id=3: bbox (106-128, 137-159) center (116, 148)

Our threshold + 8-conn label (scale 51) gives 4 isolated components
matching each cv2 marker:
- lab=68 area=302 bbox=(193,147)-(215,169)  ← id=0 component present ✓
- lab=55 area=299 bbox=(199, 95)-(221,116)  ← id=1 ✓
- lab=47 area=297 bbox=(112, 85)-(134,106)  ← id=2 ✓
- lab=65 area=313 bbox=(106,137)-(127,159)  ← id=3 ✓

All 4 pass: area > 256, bbox dims > 8, aspect ~1, fill ~0.55.
**So labeling is NOT the bug.**

Our pipeline detects only id=3.  Dbg counts at scale 51:
`quad=0 dict=4 reproj=3 ok=1`.  Of 8 candidates that reach dict, 4
fail dict, 3 fail reproj, 1 passes.

So 3 of {id=0, id=1, id=2} fail either bit-decode or PnP-reproj.

## Pipeline-stage diff vs cv2

| Stage | cv2.aruco | Ours |
|---|---|---|
| **Multi-scale threshold** | win sizes 3, 13, 23 step 10 + constant 7 | win sizes 23, 51, 101, 201 + constant 7 |
| **Pyramid downsample** | yes (image pyramid 1.0, 0.71, 0.5, 0.35) | no |
| **Binary inversion** | `THRESH_BINARY` (bright > thresh) → marker pixels = 0 in binary | `gray < mean - C` → marker pixels = 1 |
| **Contour finder** | `findContours(RETR_LIST, CHAIN_APPROX_NONE)` — Suzuki-Abe 1985 with topological structure | flood-fill connected-components labeling |
| **Connectivity** | findContours uses 4-conn for FG (interior) and 8-conn for BG (between regions) | 8-conn for FG (we changed from 4 in T18-H) |
| **Min contour size** | `params.minMarkerPerimeterRate * max(W,H)` = 0.03 × 320 = 9.6 px perimeter ≈ 2.4 px side | `SENTAI_ARUCO_MIN_QUAD_AREA = 256 px²` (~16 px side) |
| **Max contour size** | `params.maxMarkerPerimeterRate * max(W,H)` = 4.0 × 320 = full image | no max (we have bbox + aspect + fill but not perimeter cap) |
| **Polygon approx** | `approxPolyDP(eps = polygonalApproxAccuracyRate * perimeter, closed=True)` default 0.03 perimeter | iterative eps sweep [0.02..0.10] × perimeter |
| **Convexity check** | `isContourConvex` — reject if non-convex | none |
| **Corner ordering** | "winding direction" check + reorder CCW | Moore-Neighbor produces CW; assumed cyclic-rotation tolerant |
| **Dedup overlapping candidates** | yes, `_filterTooCloseCandidates` (min Euclidean dist between corners) | no |
| **Corner refinement** | Förstner subpix (T18-C-equivalent) | T18-C subpix (validated 0.0000 px vs cv2.cornerSubPix) |
| **Bit extraction** | warp via `getPerspectiveTransform` + `warpPerspective(INTER_NEAREST)` + Otsu + cell-majority with `cellMarginRate=0.13` | T18-F: same algorithm ported |
| **Border error check** | `_getBorderErrors`: count NON-BLACK bits in border ring; reject if > 35 % | **MISSING** (we don't validate border) |
| **Dict identify** | `Dictionary::identifyMarker` with `errorCorrectionRate=0.6` → max ~3 bit errors corrected | XOR-popcount, min over 4 rotations, gate at `SENTAI_ARUCO_MAX_HAMMING=2` |
| **PnP** | `solvePnP(SOLVEPNP_IPPE_SQUARE)` — analytical 2-solution; pick by reproj | homography decomposition → 1 solution; no IPPE 2-fold disambiguation |
| **Reproj gate** | implicit (best of 2 IPPE solutions is kept) | `ARUCO_REPROJ_GATE_PX = 3.0` |

## Most likely root causes for the 60 % gap

Ranked by impact-on-frame-133:

1. **Corner-ordering convention** (HIGH probability).  Moore-Neighbor
   produces CW order; cv2's findContours produces CCW for outer
   contours.  Our dict-decode tries 4 rotations but NOT the mirror.
   A CCW-traversed marker decodes to a MIRRORED 16-bit pattern; the
   nearest of 16 (4 dict × 4 rotations) candidates is at hamming 5-8
   → reject.  This matches our observation of 4 dict-fails per scale.

2. **Hamming gate too tight** (MEDIUM).  Our `SENTAI_ARUCO_MAX_HAMMING = 2`
   matches OpenCV's `errorCorrectionRate ≈ 0.4 × 16 = 6` (max 3-bit
   correction).  Relaxing to 3 might recover some markers but invites
   false positives.

3. **No border-error check** (LOW for now, MEDIUM long-term).  Our
   pipeline can match noise quads to dict entries; missing the border
   sanity-check is a robustness gap, not a recall gap.

4. **No image pyramid** (LOW for frame 133, markers are 22 px so
   no downsample needed; HIGH for very-far markers).

5. **No contour dedup** (LOW for clean scenes; could matter when same
   marker is caught at two scales with offset corners).

## Recommended fix order

1. **Mirror handling in dict-decode**: try BOTH CW and CCW corner
   orders before deciding "no match".  Cheap (~10 LoC, 2× dict lookup
   loop).  Should close most of the 40 % → 92 % gap if the hypothesis
   is right.

2. **Border-error check** (port `_getBorderErrors`): reject candidates
   whose border ring has > 35 % bright bits.  ~30 LoC.

3. **Image pyramid for small markers** (deferred — markers ≥ 20 px
   in our scene, pyramid not yet needed).

4. **Contour dedup** (deferred — multi-scale dedup we already do by
   marker-id is enough for our case with 4 known ids).

Plan: implement (1) and re-run frame-133 comparison.  If 4/4, run on
full s174 frame set.  If still gapped, layer in (2).

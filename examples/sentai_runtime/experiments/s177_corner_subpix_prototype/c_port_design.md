# T18-C — sentai_aruco corner subpixel refinement: C port design

Source: validated in Python at `our_subpix.py`.  Matches
`cv2.cornerSubPix` within ~5 mm Z typical, ~20 mm worst case on
high-rotation frames (pnp_comparison.log).  Drops per-marker Z
spread on rotated frames from 40 mm → 14 mm.

## API surface

```c
// In sentai_aruco.cc (or new sentai_aruco_subpix.cc), private helper.
// Called by the existing detect loop right after the contour finder
// produces 4 corner candidates, before the dictionary decode.
//
//   img       : grayscale frame, row-major, W bytes per row
//   W, H      : frame dimensions
//   cx_io     : in/out corner x (pixels, may be fractional after call)
//   cy_io     : in/out corner y
//
// No-op if the corner sits within 1+WIN_HALF pixels of any image
// edge (graceful fallback to the contour-finder value).
static void sentai_aruco_refine_corner(
    const uint8_t* img, int W, int H,
    float* cx_io, float* cy_io);
```

## Algorithm (verbatim from Python)

```
WIN_HALF = 2   ⇒ 5x5 window
ITER_MAX = 30
EPS_PX   = 0.01

for iter = 0..ITER_MAX-1:
    cx_i = round(*cx_io)
    cy_i = round(*cy_io)
    bounds-check; abort if window crosses image edge
    A00 = A01 = A11 = 0
    b0 = b1 = 0
    for dy = -2..+2:
        for dx = -2..+2:
            px = cx_i + dx
            py = cy_i + dy
            gx = 0.5 * (img[py, px+1] - img[py, px-1])
            gy = 0.5 * (img[py+1, px] - img[py-1, px])
            w  = G[dy+2, dx+2]   # precomputed 5x5 Gaussian LUT
            wgx = w * gx
            wgy = w * gy
            A00 += wgx * gx
            A01 += wgx * gy
            A11 += wgy * gy
            b0 += (wgx * gx) * px + (wgx * gy) * py
            b1 += (wgx * gy) * px + (wgy * gy) * py
    det = A00*A11 - A01*A01
    if |det| < 1e-9: abort
    new_x = ( A11 * b0 - A01 * b1) / det
    new_y = (-A01 * b0 + A00 * b1) / det
    step² = (new_x - *cx_io)² + (new_y - *cy_io)²
    *cx_io = new_x
    *cy_io = new_y
    if step² < EPS_PX² : break
```

## Memory + CPU budget (M7 @ 800 MHz)

| Resource | Cost |
|---|---|
| ROM | ~50 LoC compiled → ~400 B Thumb-2 |
| RAM | 25 floats LUT (read-only, `.rodata`) + ~8 floats stack |
| Per-corner cycles | ~10 000 (5-10 iters typical) → **12 µs** |
| 16 corners (4 markers) | ~200 µs per frame |
| Pipeline at 6.6 Hz | 200 µs × 6.6 = **1.3 ms/s = 0.13 % CPU** |

Negligible.  Fits comfortably in the existing `.sdram_text` budget
(see [[itcm-budget]]).

## CMSIS-DSP / SIMD optimization options

Evaluated for our specific 5x5 inner loop:

| Optimization | Mechanism | Expected gain | Recommend? |
|---|---|---|---|
| `__USUB8` for gradient | 4-byte parallel pixel subtract | 2-3× on gradient phase only (~30 % total) | **MAYBE**, only if profiling shows refinement >100 µs/frame |
| `arm_mat_inverse_f32` for 2x2 | CMSIS routine | NEG — 100-cycle overhead vs 4-multiply inline | NO |
| `arm_dot_prod_f32` | Vectorized MAC | NEG — our loop is interleaved | NO |
| Gaussian LUT (already in design) | Skip `expf()` | 25 calls × ~120 cycles each = 3 000 cycles saved | YES (already in spec) |
| Fixed-point version | int16 throughout | 1.5-2× on M4 only; M7 has FPU | NO (M7 FPU is fast) |
| Cache pixel rows once | Avoid 25 repeated `img[py*W+px]` index computes | Compiler-handled with `-O2`; row pointers in loop body | YES (free) |

**Plan: ship the straightforward float version first; profile under
real pipeline load; add `__USUB8` only if needed.**

## Risks

1. **Numerical divergence on near-aliased corners**: Python sees up to
   1.7 px divergence vs cv2 on rot45 id=1.  The algorithm CAN oscillate
   if window straddles two strong edges.  Mitigation: keep ITER_MAX=30
   and check for non-decreasing step² as early-abort.  Acceptable to
   fall back to the contour-finder value on divergence.

2. **Detector dropout under rotation**: `cv2.aruco` detected 4/4 in
   both frames; `sentai_aruco` only 1/4 on rot45.  This is **upstream
   of subpix** and a separate T18-D investigation (contour-finder
   threshold + minimum-perimeter tunables).  Subpix alone won't recover
   missed detections.

3. **Linear interpolation in window**: cv2 actually uses bilinear
   sampling of the image so the window can sit at fractional pixel
   coords.  Ours rounds to integer center per iter.  Trade-off: simpler
   code + 1-cycle-per-load LUT vs slightly slower convergence.
   Empirically converges in ≤5 iters on our test corners.

## Order of operations

1. T18-B (median + outlier reject) — already coded, just needs ARM build
   to be unblocked (separate issue from T18-A/C).  Ship independent.
2. T18-A (IPPE 2-solution) — revert from `sentai_aruco.cc` (proven
   no-op for this scene) OR keep as future-proofing.  Operator decides.
3. T18-C (this) — port the function above + wire into detect loop.
4. Re-run s174 yaw mission.  Expected: altitude drift ≤ 5 cm during
   360° yaw (down from 60 cm).

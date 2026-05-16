# s140 — GIST-lite coarse scene descriptor

**Date**: 2026-05-16
**Predecessor**: s139 (PHOG shipped)
**Plan**: §23.5 step 2 piece 2/4 — Track A places.

GIST-lite is a thesis-MVP simplification of full Gabor-GIST
(Oliva & Torralba IJCV 2001).  Full GIST = 32 Gabor filters × full
image is ~120 M ops/frame on M7 — too heavy.  s140 ships a coarse
approximation:

- **Decimate** input gray by 4× (320×240 → 80×60)
- **4 directional gradients** at each decimated pixel: DX (Sobel-X),
  DY (Sobel-Y), D45 (rotated Sobel NE-direction), D135 (rotated
  Sobel NW-direction)
- **4×4 spatial pool** → 16 cells, mean |response| per cell
- **L2 norm per cell across 4 orientations** (preserves
  cross-orientation discrimination + lighting invariance)
- **Output**: 4 × 16 = 64 floats

```c
int sentai_gist_compute(const uint8_t* gray, int w, int h, float* out);
```

MP: `sentai.places.compute_gist(bytes, w, h) → list[64 floats]`.

## SIM smoke (16/16 PASS)

Synthetic image checks:

| Image | Expected | Observed | Note |
|---|---|---|---|
| Uniform gray | all zero | sum < 1e-3 ✓ | no gradient |
| Vertical edges (L/R 0/255) | DX peak | DX=6.53, DY=0, D45/D135=3.27 ✓ | DX clearly dominant |
| Horizontal edges (T/B 0/255) | DY peak | DY=6.53, DX=0, D45/D135=3.27 ✓ | mirror of above |
| NW-SE diagonal (y>x = 255) | D135 peak | D135=6.34, D45=0.09 ✓ | gradient ⊥ edge |

Plus invariants:
- length == 64
- per-cell L2 norm ∈ {0} ∪ [0.9, 1.1]
- determinism (same input → bit-identical)
- invalid input rejected

## Engineering notes

- **Per-cell vs per-orientation L2 norm**: first attempt normalized
  per orientation block (16 cells per orientation, L2-normalize that
  block).  Result: DX and D135 both saturated at L2=1.0 even though
  raw DX response was 1020 vs D135 response was 765 — orientation
  comparison became meaningless.  Fix: L2-normalize the 4-orient
  vector PER cell.  Cross-orientation discrimination preserved.
- **Rotated Sobel for diagonals**: D45/D135 use 3×3 rotated Sobel
  ([0±1±2; ∓1 0 ±1; ∓2∓1 0]).  Anti-aliasing of axis edges through
  3×3 window still bleeds ~half of the X/Y response into diagonal
  channels — accept as feature of the simplified design.
- **Decimation buffer**: 80×60 = 4800 bytes stack static.  Capped at
  160×120 (covers up to 640×480 input).  ARM static SDRAM.

## Performance estimate (untested ARM)

- 320×240 input → 80×60 decimated (4800 pixels)
- Per decimated pixel: 4 kernels × ~10 ops each = 40 ops
- Total: 4800 × 40 = 192k ops + decimation (320×240 read) ≈ 350k ops
- M7 @ 800 MHz: ~0.5 ms expected.  Comfortably cold-path.

s144 will measure on ARM DWT.

## What this does NOT do

- **Full Gabor multi-scale bank** — out of scope for thesis-MVP.
  Approximated by 4 directional Sobel gradients on decimated image.
- **Color (Lab a/b)** — separate component (HSV in s141).
- **Quantization to int8** — happens at s143 in PCA stage.

## Related

- `[[s139-phog-shipped]]` — PHOG (piece 1/4)
- `[[places-two-track-decision]]` — Track A no-DNN primary
- `[[itcm-budget]]` — ARM placement
- Next: s141 (HSV hist), s142 (FFT-mag), s143 (combine 64B), s144
  (Gazebo loop closure)

---
name: s140-gist-shipped
description: "s140 GIST-lite descriptor SIM PASS 2026-05-16. New sentai_gist.{h,cc} computes 64-float coarse scene descriptor (4 directional gradients × 4×4 spatial pool, decimated 4×). SIM 16/16 PASS: vertical→DX peak (6.53 vs 0), horizontal→DY peak, NW-SE diagonal→D135 (6.34, perpendicular to edge), per-cell L2 norm preserved. Track A piece 2/4 per Oliva & Torralba IJCV 2001 lite."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16.**  Pasul 2 §23.5 (Track A places) — piece 2/4.

## What this ships

`sentai_gist.{h,cc}` — 64-D coarse scene descriptor:
- Input: gray uint8 image, w×h (≥ 16×16)
- Decimate 4× by 4×4 block averaging
- 4 directional gradients: DX (Sobel-X), DY (Sobel-Y), D45/D135
  (rotated 3×3 Sobels)
- 4×4 spatial pool, mean |response| per cell
- **L2 norm per CELL across 4 orientations** (cross-orient
  discrimination + lighting invariance)
- Output: 4 × 16 = 64 floats

```c
int sentai_gist_compute(const uint8_t* gray, int w, int h, float* out);
```

MP: `sentai.places.compute_gist(bytes, w, h)`.

## SIM smoke (16/16 PASS)

| Image | Result | Status |
|---|---|---|
| Uniform | sum < 1e-3 (all zero) | ✓ |
| Vertical edges (L/R 0/255) | DX=6.53, DY=0, D45/D135=3.27 (DX clear dominant) | ✓ |
| Horizontal edges | DY=6.53, DX=0, D45/D135=3.27 | ✓ |
| NW-SE diagonal (y>x=255) | D135=6.34, D45=0.09 (gradient ⊥ edge) | ✓ |
| Per-cell L2 norm | ∈ {0} ∪ [0.9, 1.1] | ✓ |
| Determinism | bit-identical reruns | ✓ |
| Invalid input | None (w<16, buffer<w*h) | ✓ |

## Key engineering decision: per-cell vs per-orient L2 norm

First attempt L2-normalized PER ORIENTATION block (16 cells of DX
together, 16 cells of DY together, etc.).  Result: DX, D45, D135 all
saturated at L2=1.0 even though raw DX response was 1020 vs D135's
765.  **Cross-orientation discrimination was LOST** — the descriptor
couldn't tell vertical from diagonal edges.

Fix: L2-normalize the **4-orientation vector PER CELL**.  Each cell
becomes a 4-D unit vector representing local orientation distribution.
Magnitudes vary across cells (spatial structure preserved), but a
cell with strong DX vs strong DY stays distinguishable.

This is closer to the "concatenated normalized blocks" used in HOG /
SIFT than to per-band normalization of Gabor responses.

## Performance estimate (untested ARM)

- 320×240 input → 80×60 decim (4800 pixels)
- ~40 ops/pixel × 4800 = 192k ops + decim 320×240 read = ~350k ops
- M7 @ 800 MHz: ~0.5 ms expected.  Cold-path.

## Related

- `[[s139-phog-shipped]]` — piece 1/4
- `[[places-two-track-decision]]` — Track A
- `[[itcm-budget]]` — ARM placement (.sentai_slow linker route)
- Next: s141 (HSV color hist) → s142 (FFT-mag) → s143 (combine 64B)
  → s144 (Gazebo loop closure)

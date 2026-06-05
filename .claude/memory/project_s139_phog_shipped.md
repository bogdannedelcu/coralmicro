---
name: s139-phog-shipped
description: "s139 PHOG descriptor compute SIM PASS 2026-05-16. New sentai_phog.{h,cc} computes 168-float Pyramid HOG (3 levels × 8 bins × 21 cells) from gray image. SIM smoke 16/16 PASS: uniform→zero, vertical edges→bin 0, horizontal edges→bin 4, L2 norms ≈ 1 per cell, deterministic. First piece of Track A places per ideas/objects_plan.md §22.2."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16.**  Pasul 2 din §23.5 (Track A places) — first component.

## What this ships

`sentai_phog.{h,cc}` — Bosch 2007 PHOG implementation:
- Input: gray uint8 image, w×h (≥ 8×8)
- Pyramid: L0 (1 cell, whole image) + L1 (2×2 = 4 cells) + L2 (4×4 = 16 cells)
- 8 orientation bins per cell, [0, π) (unsigned gradient)
- Sobel 3×3 for gradient, atan2 for orientation, L2 norm per cell
- Output: 168 floats (`8 × 21 = 168`)

```c
int sentai_phog_compute(const uint8_t* gray, int w, int h, float* out);
```

MP binding: `sentai.places.compute_phog(bytes, w, h) → list[168 floats]`.

## SIM smoke results (16/16 PASS)

3 synthetic images tested:

| Image | Expected peak | Observed peak | Energy |
|---|---|---|---|
| Uniform gray | none (zero gradient) | descriptor all zero | sum < 1e-3 |
| Vertical edges (L/R 0/255) | bin 0 (gradient on X) | **bin 0** | 13.0, others 0 |
| Horizontal edges (T/B 0/255) | bin 4 (gradient on Y) | **bin 4** | 13.0, others 0 |

Plus invariants:
- Length exactly 168
- All values finite, non-negative
- Cell L2 norms ∈ {0} ∪ [0.9, 1.1]
- Determinism: same input → bit-identical output
- Invalid input rejected (NULL, w<8, buffer < w*h)

## Engineering details

- Sobel kernels computed inline (no allocation, border-clamped)
- atan2 + fold to [0, π) for unsigned gradient (PHOG / HOG convention)
- Magnitude < 1.0 suppressed (noise floor)
- L2 norm with eps=1e-6 to avoid div by zero on empty cells
- ARM placement `.sentai_slow` SDRAM (linker enumeration of
  `sentai_phog.cc.obj` matching L4/L5/L6 pattern, per [[itcm-budget]])

## Performance estimate (untested on ARM)

- 320×240 frame = 76800 pixels
- Per pixel: Sobel (8 multiplies + 2 abs) + atan2 (~2 µs M7) + bin
  assignment = ~10 µs/pixel scalar
- Total estimate: ~10 ms per frame on M7 @ 800 MHz
- Cold-path (1 Hz from place-fingerprint task) — within budget

s144 will measure actual ARM timing via DWT bench.

## What this does NOT do (s140+)

- **GIST descriptor** (s140) — global scene shape via Gabor filters
- **HSV color histogram** (s141) — 16 hue × 4 sat = 64 bins, but raw float
- **Log-polar FFT-mag** (s142) — rotation-invariant component
- **Concatenate + PCA project to 64B int8** (s143) — final place
  descriptor that fits L3 slot
- **Gazebo loop closure test** (s144) — drone visit + revisit + match

s139 alone proves only that PHOG math is correct on the SIM build.

## Why MicroPython `bytearray` doesn't work in embed

First smoke run crashed: `NameError: name 'bytearray' isn't defined`.
MicroPython embed port doesn't include `bytearray` by default.  Fixed
test driver to build a list of ints + `bytes(list)`.  Noted here for
future SIM driver writers.

## Related

- `[[l3-shipped]]` — L3 places gallery (consumer of descriptors)
- `[[places-two-track-decision]]` — Track A no-DNN primary path
- `[[itcm-budget]]` — ARM placement discipline
- Next: s140 GIST → s141 HSV → s142 FFT-mag → s143 64B → s144 loop closure

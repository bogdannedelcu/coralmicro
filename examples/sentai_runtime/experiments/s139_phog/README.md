# s139 — PHOG descriptor compute (Track A foundation)

**Date**: 2026-05-16
**Predecessor**: s138 (LOST state shipped)
**Plan**: §23.5 step 2 — Track A places.

First piece of Track A place fingerprint per
`ideas/objects_plan.md §22.2`:
PHOG + GIST + HSV + log-polar FFT-mag → 64B int8 descriptor → H3 + L3 gallery.

s139 ships **PHOG only**.  s140-s142 add GIST/HSV/FFT-mag; s143
concatenates + quantizes to 64B; s144 wires Gazebo loop closure test.

## What ships in s139

```c
// New file: sentai_phog.{h,cc}
int sentai_phog_compute(const uint8_t* gray, int w, int h, float* out);
// → 168 floats:
//   8 orientation bins × (1 + 4 + 16) cells (3 pyramid levels)
//   per-cell L2 normalization (Bosch 2007 / Dalal & Triggs 2005)
```

MP binding: `sentai.places.compute_phog(bytes, w, h) → list[float] of 168`.

Cold-path ~1 Hz on M7 (~10 ms expected); routed to `.sentai_slow`
SDRAM section per `[[itcm-budget]]`.

## Behavior under test

PHOG is deterministic — given the same image, same descriptor.  s139
verifies algebraic properties on three synthetic images:

1. **Uniform gray** (no edges) → all-zero descriptor (no gradient).
2. **Vertical-edge synthetic** (left/right halves) → peak in bins
   corresponding to **horizontal gradient** (vertical edges produce
   gx ≠ 0, gy ≈ 0, so angle ≈ 0 → bin 0).
3. **Horizontal-edge synthetic** (top/bottom halves) → peak in bins
   corresponding to **vertical gradient** (horizontal edges produce
   gx ≈ 0, gy ≠ 0, so angle ≈ ±π/2 → bin ≈ BINS/2).

Plus invariants:
- Length is exactly 168
- Each cell's L2 norm is either 0 (no gradient) or ≈ 1 (normalized)
- All values are finite, non-negative

## Pass criteria

```
phog len == 168
uniform_image:    all cells with sum < 1e-3 OR L2 ≈ 0 (no gradient)
vertical_edges:   most energy in bin 0 ± 1 (horizontal gradient)
horizontal_edges: most energy in bin BINS/2 ± 1 (vertical gradient)
each L2-normalized cell sum_of_squares ≈ 1.0 (within 5e-2)
all values finite, no NaN/Inf
```

## Files

```
s139_phog/
├── README.md         # this file
├── _t_phog.py        # SIM smoke driver
└── run.sh            # SIM smoke runner
```

## Why no Gazebo test here

s139 tests math correctness only — no drone, no closed-loop.  Gazebo
integration comes at **s144** when descriptor + H3 + match + loop
closure are all wired together.

## Related

- `[[l3-shipped]]` — L3 places gallery (consumer of descriptors)
- `[[places-two-track-decision]]` — Track A vs Track B
- `[[itcm-budget]]` — ARM placement discipline
- Next: s140 (GIST), s141 (HSV), s142 (FFT-mag), s143 (combine 64B),
  s144 (Gazebo loop closure)

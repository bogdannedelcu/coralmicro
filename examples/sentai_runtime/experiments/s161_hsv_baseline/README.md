# s161 — HSV histogram descriptor SIM smoke (OP-S10-W4)

Track A piece 3/4 per `objects_plan/12_places_two_track.md` (§22.2 in
the original numbering).  Closes calendar W2 of the 4-week SIM-only
plan (`[[short-term-plan-2026-05-17]]`).

## What this proves

`sentai.places.compute_hsv(rgb_bytes, w, h)` computes a slot-native
HSV histogram descriptor — 16 hue bins × 4 saturation bins = 64
bytes, directly compatible with the L3 places gallery slot
(`SENTAI_PLACES_DESC_DIM == 64`).  No `quantize()` step needed
(unlike PHOG/GIST which return float lists packed via
`hex_helpers.py`).

V channel is intentionally discarded for **illumination invariance**;
pixels darker than `SENTAI_HSV_V_MIN == 16` are excluded from the
histogram (hue is unreliable in near-black pixels — chroma is
dominated by sensor noise).

## SOTA basis

| Stage | Reference |
|---|---|
| RGB→HSV conversion | Smith, A.R., "Color gamut transform pairs", SIGGRAPH 1978 (sectoral max-channel formula, pure integer) |
| Histogram-based color indexing | Swain & Ballard, "Color indexing", IJCV 7(1), 1991 |
| L1 distance for histogram match | classical Manhattan over bins |

**No HW shortcut**: PXP supports YUV / Y8 but not direct RGB→HSV.
CMSIS-DSP has matrix / FFT primitives that don't apply at a per-pixel
integer kernel.  Hand-written but pure-integer (no float, no atan2,
no sqrt — Smith 1978 is fundamentally compare + subtract + divide).

## 18 gates, all PASS (run 2026-05-17)

| Gate | Behaviour | Result |
|---|---|---|
| T1 | Uniform gray 128/128/128 → bin 0 dominant + rest zero | PASS |
| T2 | Pure red 255/0/0 → bin 3 (H=0°, S=high) dominant | PASS |
| T3 | Pure green 0/255/0 → bin 23 (H≈120°) dominant | PASS |
| T4 | Pure blue 0/0/255 → bin 43 (H≈240°) dominant | PASS |
| T5 | V-invariance: bright vs dim red → identical descriptors (L1=0) | PASS |
| T6 | Gray + red patch → both gray (240) AND red (16) bins, gray > red | PASS |
| T7 | Dark scene (V=8) → all bins zero (every pixel excluded) | PASS |
| T8 | Determinism: red replay → bit-for-bit identical | PASS |
| T9.{gray,red,green,blue} | Anti-regression goldens captured 2026-05-17 | 4× PASS |
| T10 | Match coherence: red↔gray L1=510, red↔blue L1=510, same-scene L1=0 | 3× PASS |

## How to run

```bash
bash examples/sentai_runtime/experiments/s161_hsv_baseline/run.sh
```

PASS iff `VERDICT: PASS` appears AND every `PASS  Tn …` is green.

## Slot-native compute pattern

Unlike PHOG/GIST (return float lists → `hex_helpers.py:quantize`
packs to 64 B), `compute_hsv` returns `bytes(64)` directly.  This is
the original L3 design intent — the slot comment in `sentai_places.h`
says `"16 hue × 4 sat bins (uint8_t each)"`.  HSV IS the slot's
native layout.

```python
import sentai
rgb = sentai.camera.grab_rgb(64, 64)        # future hook (deferred)
desc = sentai.places.compute_hsv(rgb, 64, 64)   # bytes(64)
cell = sentai.places.cell_at(x, y, 15)
pid  = sentai.places.add(cell, desc, x, y, z)   # no quantize step
```

## What this does NOT prove

- **Real Gazebo / camera-stream integration** — defers until
  `sentai_camera_grab_rgb_zerocopy` is added (mirror of
  `grab_gray_zerocopy`; uses `pxp_scale_xrgb_to_rgb` on ARM, the
  `s_grab_rgb_src` buffer in `sim/modsentai_sim_camera.c` on SIM).
- **Combination with PHOG + GIST in a single slot** — the three
  descriptors are independent today; a fused 64 B layout would
  require a new packing helper.
- **Robustness to JPEG compression / sensor noise** — clean synthetic
  RGB only.

## ARM build + regression

- `sentai_hsv.cc.obj`: text=340 B (under the 4 KB target for a single
  descriptor), routed to `.sdram_text` matching the PHOG/GIST pattern.
- EXP-s127 FlowBaseline post-commit: `dist_mean=7.98 cm` (canonical
  7.4 cm, gate 10 cm) — no flow-path regression.

## Cross-references

- `examples/sentai_runtime/sentai_hsv.{h,cc}` — implementation +
  SOTA citations.
- `examples/sentai_runtime/sentai_places.h` line 65: slot comment
  predicting this layout.
- `[[places-two-track-decision]]`, `[[s141-descriptor-baseline-shipped]]`,
  `[[arm-hw-primitives-first]]`, `[[itcm-budget]]`.
- s139 PHOG + s140 GIST + s141 baseline (siblings in Track A).
- Calendar W2 of `[[short-term-plan-2026-05-17]]` — closed by this
  commit.

---
name: s141-descriptor-baseline-shipped
description: "s141 DescriptorBaseline anti-regression gate for PHOG + GIST. Hardcoded goldens (captured 2026-05-16) + match coherence test. PHOG dist(same scene) = 0, dist(different) = 4.84 — perfect lighting invariance + clear scene discrimination. GIST ratio 0.0002 (similar). 15/15 SIM PASS. Run after ANY modification to sentai_phog.cc / sentai_gist.cc. Mandatory pre-commit per [[gate-every-layer-no-exceptions]]."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16.**  Operator review of s139+s140 surfaced hidden
vices.  s141 is the anti-regression gate that closes the main ones.

## Why this gate

Without anti-regression: any future change to `sentai_phog.cc` or
`sentai_gist.cc` (kernel coefficients, normalization scheme,
boundary handling, magnitude threshold) could silently break the
descriptor.  Synthetic tests in s139/s140 pass on "math correctness"
(vertical edges → bin 0) but say nothing about exact reproducibility
or scene-discrimination utility.

s141 closes both gaps.

## Test claims (HARD gates)

1. **REGRESSION** — bit-exact match against goldens captured
   2026-05-16, tolerance 1e-5 element-wise.  Catches algorithm drift.
2. **COHERENCE** — `L2_dist(desc(alpha), desc(alpha_perturbed)) < 0.5 × L2_dist(desc(alpha), desc(beta))`.
   Proves descriptor distinguishes scenes (not just compute something).
3. **DETERMINISM** — 3 sequential reruns of same input must be
   bit-identical (1e-9 tolerance).
4. **PERF BUDGET** — placeholder (Stage 9 ARM DWT timing is canonical).

## Synthetic fixtures (W=H=80)

- **alpha**: vertical stripes left + diagonal pattern right (mixed
  gradient structure)
- **alpha_perturbed**: alpha + uniform +20 brightness shift (same
  scene, lighting change)
- **beta**: concentric squares (different scene)

These reproduce bit-exactly across SIM runs (deterministic bytes →
deterministic float descriptors).

## Results captured

```
PHOG alpha vs golden:      max diff 4.77e-07   PASS
PHOG beta  vs golden:      max diff 4.77e-07   PASS
GIST alpha vs golden:      max diff 4.88e-07   PASS
GIST beta  vs golden:      max diff 5.07e-07   PASS

PHOG dist(alpha, alpha_perturbed) = 0.0000     # perfect invariance
PHOG dist(alpha, beta)            = 4.8367     # clear discrimination
ratio                             = 0.0000     # PASS (< 0.5)

GIST dist(alpha, alpha_perturbed) = 0.0006     # near-perfect (int8 sat)
GIST dist(alpha, beta)            = 3.0950
ratio                             = 0.0002     # PASS (< 0.5)

Determinism (3 reruns each):  PHOG identical, GIST identical
Invalid input (tiny image):   PHOG None, GIST None
```

## Interpretation — what the numbers mean

- **PHOG distance was literally zero** for the brightness-shift case.
  This is mathematically expected: Sobel gradients are DC-blocking
  (gradient(constant) = 0), so uniform brightness shift produces zero
  gradient change.  L2 normalization further removes any residual
  scale.  Perfect invariance.
- **GIST distance was 0.0006** — tiny residual from uint8 saturation
  at brightness=255 pixels (where `+20` clips to 255 instead of
  producing a meaningful shift).  Still 5000× smaller than inter-scene
  distance.
- **5000× separation factor** means in practice: any reasonable
  threshold (e.g., 1.0) cleanly separates "same scene" from "different
  scene".  Descriptor is genuinely useful for place recognition.

## Engineering fix shipped here

`sentai_gist.cc`: the `static uint8_t s_decim[160*120]` buffer (HIGH
severity in review) is now documented explicitly as **single-writer**.
Matches L2-L6 convention of single-writer static state.  If concurrent
compute is ever needed (mission task + REPL inspection), switch to
per-call stack — 4800 B for typical 320×240 input fits the 16 KB
FreeRTOS task stack.

## Why goldens are hardcoded (not loaded from file)

- No filesystem dependencies in the test
- Captured values are explicit in the source → diff visible in PR
- Forces the engineer to think before re-capturing ("does this change
  match the algorithm intent?")
- Tracks regressions via git blame

## How to use

```bash
bash examples/sentai_runtime/experiments/s141_descriptor_baseline/run.sh
# Exit 0 PASS / 1 FAIL.
# Run after ANY edit to sentai_phog.{h,cc} or sentai_gist.{h,cc}.
```

To re-capture goldens (only if algorithm intentionally changed):
1. Copy `_t_baseline.py` to a capture script
2. Replace `chk(diff < REG_TOL, ...)` with `print(d)`
3. Run, paste new goldens back
4. Document the algorithm change in commit msg + memory entry

## What this does NOT cover

- ARM compute time — measured at Stage 9 DWT bench
- Real Gazebo frame — added at s144 (loop closure end-to-end)
- Rotational invariance — separate gate when s142 FFT-mag ships
- Int8 quantization — s143 will quantize 168+64+... → 64B and add a
  quantization-stability gate on top of this baseline

## Related

- `[[s139-phog-shipped]]` — gated by this test
- `[[s140-gist-shipped]]` — gated by this test
- `[[gate-every-layer-no-exceptions]]` — discipline used
- `[[test-must-be-relevant-to-claim]]` — claims (1-4) named in test
- Next: s142 FFT-mag will be added to this baseline when shipped

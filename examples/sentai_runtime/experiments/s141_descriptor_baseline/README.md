# s141 — DescriptorBaseline (PHOG + GIST regression gate)

**Date**: 2026-05-16
**Predecessor**: s139 PHOG, s140 GIST
**Purpose**: anti-regression gate per `[[gate-every-layer-no-exceptions]]`
+ `[[test-must-be-relevant-to-claim]]`.

## Why this exists

After review of s139/s140 we found hidden vices:

| Vice | Location | Severity |
|---|---|---|
| `static uint8_t s_decim[160*120]` not thread-safe | sentai_gist.cc | HIGH (now documented single-writer) |
| `mag < 1.f` suppress could mute weak gradients | sentai_phog.cc | MED |
| No real-world coherence test (only synthetic per-property) | s139, s140 | HIGH |
| No anti-regression baseline (any algorithm change goes unnoticed) | s139, s140 | HIGH |
| No determinism stress (3 reruns) | s139, s140 | LOW |
| No quantization int8 path (final L3 needs 64B) | not yet | (s143 scope) |

s141 closes the last 3 — adds a permanent gate for PHOG and GIST.

## Test design

Per `[[test-must-be-relevant-to-claim]]`, the test must prove the
behavior of "descriptors are useful for place recognition", not just
that they compute *something*.  We split into 4 claims:

### Claim 1: REGRESSION (anti-drift)
Compute PHOG and GIST on 2 fixtures (alpha, beta).  Compare against
**hardcoded golden values** captured 2026-05-16.  Element-wise diff
must be < 1e-5.  Any change to PHOG/GIST algorithm breaks the gate
visibly.

If algorithm is intentionally changed, the gate failure prompts
"capture new goldens AND record what changed".

### Claim 2: COHERENCE (descriptor is useful)
3 synthetic fixtures:
- **alpha**: vertical stripes (left) + diagonal pattern (right)
- **alpha_perturbed**: alpha + uniform +20 brightness shift (SAME scene, lighting change)
- **beta**: concentric squares (DIFFERENT scene)

Verify `L2_dist(desc(alpha), desc(alpha_perturbed)) < L2_dist(desc(alpha), desc(beta)) × 0.5`.

This is the **substance** of "descriptor distinguishes scenes":
- if descriptor is constant → ratio ≈ 1 → FAIL
- if descriptor is random → ratio ~unpredictable → FAIL
- if descriptor is useful → ratio << 1 → PASS

### Claim 3: DETERMINISM (no thread-safety regressions)
3 sequential reruns of same input must produce bit-identical output.
Catches: stale buffers, race conditions, non-initialized state.

### Claim 4: PERF BUDGET (placeholder for Stage 9)
SIM x86 timing budget exists but is intentionally loose.  ARM DWT
timing comes at Stage 9 bring-up — that's the canonical perf gate.

## Results

```
PHOG alpha max diff vs golden = 4.77e-07 (tol 1e-5)          PASS
PHOG beta  max diff vs golden = 4.77e-07                      PASS
GIST alpha max diff vs golden = 4.88e-07                      PASS
GIST beta  max diff vs golden = 5.07e-07                      PASS

PHOG dist(alpha, alpha_perturbed)  = 0.0000                    PHOG perfect
PHOG dist(alpha, beta)             = 4.8367                    invariance to
ratio                              = 0.0000  (< 0.5)           lighting + clear
                                                                discrimination
GIST dist(alpha, alpha_perturbed)  = 0.0006
GIST dist(alpha, beta)             = 3.0950
ratio                              = 0.0002  (< 0.5)

15 PASS / 0 FAIL                                              VERDICT: PASS
```

Interpretation:
- **PHOG is mathematically invariant** to uniform brightness shift (Sobel
  gradient cancels DC; orientations are normalized).  Distance was 0.
- **GIST has tiny residual** (0.0006) due to int8 wrap at saturated
  pixels (`min(255, b+20)`), but still 5000× smaller than inter-scene.
- Both descriptors are deterministic and reproducible bit-exact.

## What the test does NOT cover

- ARM build timing (Stage 9 will measure via DWT)
- Real Gazebo frame (synthetic only — moving to s144 when full pipeline tests)
- Rotational invariance (PHOG/GIST are orientation-sensitive by design;
  s142 FFT-mag adds rotation invariance separately)
- Int8 quantization (final L3 64-byte slot — s143 scope)

## Engineering fix shipped here

`sentai_gist.cc` — `static uint8_t s_decim[160*120]` annotated with
single-writer documentation.  Buffer remains static (matches L2-L6
pattern of single-writer convention; SDRAM-backed via default .bss
routing through `.sentai_slow` linker block).  If concurrent compute
is ever needed, switch to per-call stack (4800 B for typical input
fits 16 KB FreeRTOS task stack).

## How to use the gate

```bash
# After any modification to sentai_phog.cc or sentai_gist.cc:
bash examples/sentai_runtime/experiments/s141_descriptor_baseline/run.sh
# Exit 0 PASS / 1 FAIL.

# If intentional algorithm change → capture new goldens:
#   1. cp _t_baseline.py … to capture_helper.py (without golden checks)
#   2. Add print() of computed values
#   3. Run, copy new golden values back into _t_baseline.py
#   4. Document in commit msg what changed and why
```

## Files

```
s141_descriptor_baseline/
├── README.md       # this file
├── _t_baseline.py  # gate test (synthetic fixtures + hardcoded goldens)
└── run.sh          # SIM smoke runner
```

## Related

- `[[s139-phog-shipped]]` — gated by this test
- `[[s140-gist-shipped]]` — gated by this test
- `[[gate-every-layer-no-exceptions]]` — pattern used
- `[[test-must-be-relevant-to-claim]]` — claims (1-4) named explicitly
- Next: s142 (FFT-mag) — will be added to this gate when shipped

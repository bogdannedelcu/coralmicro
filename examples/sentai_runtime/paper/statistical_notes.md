# Statistical analysis notes

A short companion to the main results that makes explicit every
statistical choice a reviewer might question.  Each subsection
answers one specific question that came up during the work and is
therefore worth anticipating.  The numeric justifications reference
the same per-iteration CSVs archived under
[`../experiments/`](../experiments/) and summarised in
[evaluation.md](evaluation.md).

---

## 1. Why n = 20 (E14/E15) and n = 40 (E16/E18)?

Let `σ` be the sample standard deviation of per-iteration times and
`SE = σ/√n` the standard error of the mean.  To support a claim that
two means differ by Δ with 95 % confidence, we want
`|Δ| ≫ 2 × SE`.

Observed `σ` values for the dominant measurements, from the archived
CSVs:

| Measurement | Observed σ (ms) | n | SE = σ/√n (ms) |
|---|---:|---:|---:|
| E15 `frame_interval_ms` (parallel, post-eDMA) | 1.8 | 20 | 0.40 |
| E16 `total_frame_ms` (alternating, pre-fix) | 32.5 | 40 (20 per direction × 2) | 5.1 aggregate, 0.5 per direction |
| E16 `total_frame_ms` (alternating, post-Fix B) | 2.5 | 40 | 0.40 |
| E18 sweep A/B `total_frame_ms` | 2.2 | 40 | 0.35 |
| E18 sweep C `total_frame_ms` | 2.5 | 40 | 0.40 |

Reported effect sizes are 9–83 ms.  SE is ~0.4 ms.  The ratio of
effect to standard error is therefore 20–200 ×, so a larger sample
would not change any headline conclusion.  The x20 choice for E14/E15
and x40 for E16/E18 is a pragmatic trade-off: large enough that the
confidence interval is comfortably narrower than the reported
differences, small enough that a session completes in 3–10 s of
wall time (relevant because REPL-driven experiments are more likely
to suffer a transient fault — CDC-ACM stall, LFS-busy race — the
longer they run).

We did not run a formal power analysis a priori because the effect
sizes were unknown before the first measurements; the x20 choice was
reactive (20 is the default `repetitions` for the parallel pipeline
in the early diag modules) and the x40 choice was a conscious upgrade
once the alternating sweeps were introduced and we wanted ≥ 19
samples per direction after the first-drop and the alternation split.

---

## 2. Why the first sample is dropped

The first iteration inside a measurement loop differs from subsequent
iterations in three ways:

1. **Switch-count parity**: E16/E18 alternating starts on one
   direction (cam_a by convention) before any flip has happened in
   the measured loop.  Its "drain" path is therefore different from
   the drain paths of iterations 1 … N−1.
2. **DMA queue occupancy**: the warm-up frame leaves buffers queued;
   the first measurement iteration consumes those.  Subsequent
   iterations see a full steady-state queue at entry.
3. **TPU package-cache miss**: the first `invoke()` after a model
   load, or after the pipeline has been stopped and restarted, pays
   a one-time package-cache walk that later invokes do not.

Dropping the first sample removes all three confounds with a cost of
n−1 instead of n.  At n = 40, that is a 2.5 % data loss, worth
trading for unbiased steady-state statistics.  Note that *none* of
the three confounds above could be corrected by a longer run; only by
dropping the outlier.

We have observed the first-sample elevation empirically: in every
`s045_e18_post_refactor/003_e18_C_alt_cam0_cam1.csv` run, the first
row's `total_frame_ms` is ~15 ms above the cohort mean.  The
appendix generator drops it automatically
([`../experiments/_build_appendix.py`](../experiments/_build_appendix.py)
in `e16_session` and `e18_session`).

---

## 3. Bessel-corrected σ, not population σ

We report `statistics.stdev` (Bessel-corrected sample standard
deviation, denominator `n−1`) rather than `statistics.pstdev`
(population, denominator `n`).  With n = 39–40 the difference is
< 2 % and does not affect any conclusion, but Bessel's correction is
the unbiased estimator when the sample is drawn from a larger
notional population (e.g. a bootstrapped rerun), which is the
situation we are in: each measurement run is one sample from the
distribution of "runs on this hardware with these parameters".

---

## 4. Mean vs median

The headline per-stage tables in [evaluation.md](evaluation.md) use
arithmetic means because:

1. The distributions are close to symmetric once the first sample is
   dropped (skewness below 0.5 on all measured stages post-warm-up).
2. Arithmetic means commute with the `1000/mean_ms` FPS derivation —
   reporting median and then deriving "median FPS" would be a
   different, more complicated statistic that did not match the
   reader's intuition of "average throughput".
3. Outliers in the archived CSVs are rare (< 1 %) and when they
   appear they are either a single `max` above the cohort by ~15 ms
   (e.g. `s043/003_e18_C_alt_cam0_cam1.csv` row 38 with total 154 ms
   vs cohort mean 145 ms, consistent with a CDC-ACM transient) or a
   cluster at the end of a session where a sensor-side wake effect
   lagged.  Either way the contribution to the mean is well below the
   cross-session noise band.

Where we *do* report median — the per-direction split in alternating
sweeps (cam0 vs cam1) — we do so because the data is bimodal and the
mean of the combined cohort is not meaningful.  The appendix tables in
[evaluation.md](evaluation.md) §4 split cam0 and cam1 into separate
unimodal sub-tables for exactly this reason.

---

## 5. No p-values, no confidence intervals on derived statistics

We do not report p-values, t-tests, or ANOVA.  Reasons:

1. Effect sizes are all ≥ 10 × the standard error, so any standard
   test would return p < 10⁻¹⁰ — not informative.
2. The hypotheses we are testing are directional and
   pre-registered in the form of the implementation chapters (e.g.
   "the eDMA memcpy should save 9 ms", "Fix B should eliminate the
   directional asymmetry"); after-the-fact null-hypothesis testing
   adds nothing.
3. The **cross-session agreement** in Table 4 of
   [evaluation.md](evaluation.md) is the rigorous claim: three
   independent measurements of the same quantity agree within ≤ 1 ms.
   That is stronger evidence than any single-session confidence
   interval, because it tests the stability of the measurement
   pipeline — not just the within-session variance.

Where a single number matters *in isolation* (e.g. the 83 ms per-
switch tax), we quote it alongside its σ and its n so a reader can
compute any confidence interval they care about.

---

## 6. FPS derivation — why `1000 / mean(total_ms)` instead of direct FPS

Every reported FPS is derived from the mean of per-iteration
`total_frame_ms`, not measured as "frames in a fixed time window".
The equivalence holds under the following condition:

> If per-iteration times are i.i.d. with mean μ and finite variance,
> then the long-run frames-per-second observed over a window of
> length T converges almost surely to 1000/μ as T → ∞.

Our iteration times satisfy i.i.d.-after-warm-up-drop within session.
The σ (1–3 ms on a 60–150 ms mean) is well within the regime where
the equivalence holds numerically: 1000/mean differs from a window-
count FPS by < 0.1 %.  See any standard text on renewal processes;
the 1000/mean derivation is the pointwise estimator.

We use the derivation because it lets us report FPS for a single
measurement window (a sweep), not just a long sustained run, and it
surfaces the per-stage contributions directly — the user can read off
"the reason I am at 16 FPS is that `invoke + to_tensor + detect =
62 ms`" rather than trying to attribute a windowed frame count.

---

## 7. Handling of sessions with n < 20

Sessions [s016](../experiments/s016_e15_512/) through
[s020](../experiments/s020_e15_512/) and
[s027](../experiments/s027_e15_512/) through
[s030](../experiments/s030_e15_512/) contain single-shot runs with
n = 1 each — they were pre/post-eDMA "does it still work" sanity
probes during the bring-up phase and do not provide a defensible
mean.  These sessions are **excluded from every statistical
comparison** in this paper.  They appear in Appendix C of
[experiments/README.md](../experiments/README.md) for completeness
but their "FPS" rows are derived from a single data point and
explicitly flagged as such.

Sessions [s036](../experiments/s036_e17_drain_ab/),
[s037](../experiments/s037_e17_drain_ab/), and
[s040](../experiments/s040_e17_drain_ab/) are empty sessions where
the REPL driver failed mid-run and no experiment completed.  They
are present in the archive so the session-ID sequence stays
contiguous; they contribute nothing to any result in this paper.

---

## 8. What the reader should check

A reviewer who wants to stress-test the statistical choices in this
paper can re-run the appendix generator against the archived CSVs
and confirm that:

1. Every headline mean in the paper tables matches the regenerated
   appendix to within rounding.
2. The σ values reported are Bessel-corrected (they match the
   `statistics.stdev` function applied to the raw columns with the
   first sample dropped).
3. The cross-session agreement in Table 4 of
   [evaluation.md](evaluation.md) holds with any reasonable choice of
   warm-up-drop length (0, 1, or 2 samples) — the claim is robust,
   not cherry-picked.

The command to run is in [artifact.md](artifact.md) §7 "Data integrity
check".

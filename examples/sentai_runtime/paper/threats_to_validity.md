# Threats to validity

Honest caveats, aggregated from the implementation chapters so a
reviewer can see them in one place.  Each threat below lists its
source (how we noticed it), its likely impact on the reported
results, and the mitigation we did or did not apply.  The structure
follows the MDPI methodology-article convention of explicit
construct / internal / external / conclusion validity threats,
adapted to an embedded-systems measurement context.

---

## 1. Construct validity — are we measuring what we claim?

### 1.1 "FPS" is always a derived statistic, not an observed one

Every reported frame-per-second number in this paper is computed as
`1000 / mean_total_ms` from a raw CSV of per-iteration wall-clock
measurements.  We never sample frame counts over a long window.  The
two forms are mathematically equivalent only if the distribution of
iteration times is well-behaved (low skew, bounded tail).  In
practice the distribution is slightly right-skewed at both 15 fps and
30 fps sensor rates (typical `σ` of 1.5–3 ms on a 60–150 ms mean);
the derived FPS is therefore within 0.1 % of a hypothetical
window-count measurement.  See [statistical_notes.md](statistical_notes.md)
§"FPS derivation".

**Mitigation**: report both `total_ms` mean and derived FPS in every
result table; a reviewer can re-derive the other.

### 1.2 Per-stage timings add Python dispatch overhead that varies

`select_ms`, `to_tensor_ms`, `detect_ms` are measured with
`ticks_ms()` snapshots around the MicroPython dispatch.  Each snapshot
adds ~100 µs of Python interpreter overhead to the measured stage.
At the 1 ms tick resolution of the counter this is below the noise
floor and does not affect the reported numbers, but it is worth
flagging for a reviewer who adds their own instrumentation: we never
claim sub-millisecond precision on any host-side stage timer.

The one stage whose duration is reported from the firmware itself
(`invoke_ms` — returned by `sentai.tpu.invoke()`) has no Python
overhead in its measurement.  That is why `invoke_ms` has the
tightest reproducibility in the cross-session table (Table 4 of
[evaluation.md](evaluation.md)).

### 1.3 The alternating sweep measures a worst-case scheduling policy

Experiment E16 / E18 sweep C flips the MUX on *every* iteration.
Nothing in the shipping firmware requires the application layer to
alternate this aggressively.  The 6.86 FPS "alternating" number is
therefore a **lower bound** on application throughput, not an
expected operating point.  Users adopting the
`sentai.camera.ratio(a, b)` scheduler for asymmetric capture will see
effective FPS interpolate between the fixed (~16 FPS) and the
alternating (~6.9 FPS) extremes as a function of the switch density.

---

## 2. Internal validity — could something else explain the results?

### 2.1 Single-board measurements

Every result in this paper was measured on one physical SentAI v1.0
board.  We did not swap OV5640 camera modules, did not change the
host USB cable or port between sessions, and did not run on a
different instance of the same hardware revision.

Consequences:
- A between-sample-board deviation (e.g. sensor silicon variation
  causing different AEC/AGC convergence) would not be visible in our
  measurements.
- The 2 ms residual cam1 − cam0 asymmetry reported post-Fix B may or
  may not reproduce on a different unit — we cannot tell.

**Mitigation**: the measurement framework is fully open and
reproducible (see [artifact.md](artifact.md)); a follow-up study on a
second board would directly test this.

### 2.2 Static-scene confound

All measurements were captured against a static scene documented by
the before/after JPEGs in every session folder (see
[experimental_setup.md](experimental_setup.md) §5).  The scene
happens to contain zero instances of the model's target class, which
means:
- `num_detections` is 0 on every frame, so NMS takes a consistent
  (and fast) path.
- We do not observe the tail behaviour of the post-processing
  pipeline when detection counts spike.

**Mitigation**: the per-iteration CSVs record `num_detections` so a
future measurement against a busy scene can compare directly.  The
scene JPEGs are checked into [`../experiments/`](../experiments/) so a
reviewer can confirm the scene visually.

### 2.3 Thermal state across sessions

Each of our sessions takes 3–10 seconds of wall time.  The RT1176
does not thermal-throttle in that window, and we did not observe any
drift in the per-iteration times within a session.  But the paper's
cross-session comparison (Table 4 of [evaluation.md](evaluation.md))
compares sessions that may have been taken with different "warm-up"
exposure of the host environment — e.g. the host USB subsystem's
buffering state can differ.

The ≤ 1 ms cross-session agreement observed in Table 4 is evidence
against any material thermal or host-state confound at the sub-
second measurement granularity we use.  We do not claim robustness
for multi-minute sustained-stress measurements.

### 2.4 Firmware build drift across a reported comparison

The cross-session comparison in Table 4 of
[evaluation.md](evaluation.md) spans three firmware builds (pre-
refactor, post-refactor confirmation, post-NASA-JPL-review).  Each
build was flashed with `scripts/flashtool.py -e sentai_runtime` (no
`--ram`; true persistent flash) and rebooted cleanly before the
measurement.  The `build_version.h` number in each session's
`summary.txt` documents which build produced the data.

The intent of the comparison is *verifying performance neutrality*
of the review refactor, not isolating a specific change.  We do not
claim that any single NASA-JPL fix in isolation is responsible for
the 0.3 ms shift in cam0 total between `s043` and `s045`; within
noise, it is not.

### 2.5 The `drain=1` knob is retained but not recommended

The shipping default is `switch_drain(2)`.  Moving to
`switch_drain(1)` was empirically (see [cam_switch.md](cam_switch.md)
§"Known limitations"):

- **Not a timing win**: the `cam->GetRawFrame` call after the drain
  loop blocks long enough to absorb the one-frame saving.  Observed:
  201 ms per iteration at both `drain=1` and `drain=2` in
  [`s041_e17_eof_check`](../experiments/s041_e17_eof_check/).
- **Not visually clean**: residual sensor-side artefacts (AEC/AGC
  convergence on the newly selected OV5640) remain even after the
  mid-buffer seam is eliminated by the flip-on-EOF fix.  Not every
  frame is affected; a thumbnail-level review misses it.

Retaining the knob as an experimentation hook has a small cost (the
`switch_drain_set/_get` accessors in the MicroPython module); the
value of being able to reproduce the limitation is greater than the
cost of carrying the code.  No production configuration should use
`switch_drain(1)` on this hardware.

---

## 3. External validity — where do the results generalise?

### 3.1 Model-dependent numbers

Every reported invoke time (30 ± 3 ms in E18) and every per-stage
total is specific to the YOLOv5-enhanced single-class model
documented in [experimental_setup.md](experimental_setup.md) §4.  A
heavier backbone, a larger head, or more classes would change
`invoke_ms` without changing the per-switch tax (which is camera-
topology bound).  A lighter model would do the opposite — making the
switch tax a larger fraction of the total.

When a reader adapts these results to a new model, the rule is:
- Switch tax per flip (~83 ms) is **stable** against model choice.
- TPU invoke time scales **~linearly** with input-tensor pixel count
  and roughly with FLOPs on EdgeTPU.
- PXP resize + quant scales with the output size (fixed at 512×512
  in our measurements) and is **not** a strong function of sensor
  native resolution.

See [cam_switch.md](cam_switch.md) §"What E18 tells us about
application design" for the predictive arithmetic linking these
knobs to effective FPS.

### 3.2 Sensor-topology-dependent numbers

The 83 ms per-switch tax is specific to the single-MIPI-lane +
analogue-MUX topology of the SentAI board.  Platforms with two
independent MIPI-CSI2 receivers (or an MIPI-bridge chip with virtual
channel support — see [related_embedded_inference.md](related_embedded_inference.md))
would not pay this tax at all.  The *method* — flip-on-EOF timing,
ratio scheduler, fault counters — generalises; the *numbers* do not.

### 3.3 Resolution-dependent expectations

All `evaluation.md` numbers are at 512×512 logical resolution.
Experiments also ran at VGA (640×480) and QVGA (320×240) and are
documented inline in [cam_switch.md](cam_switch.md) §"Per-camera
resolution".  Summary:
- Fixed-camera FPS is **sensor-rate-bound** above ~16 FPS; changing
  resolution does not raise the ceiling meaningfully.
- Alternating FPS improves noticeably at QVGA (~10 FPS) because
  PXP + JPEG costs scale with pixel count.
- VGA is **slightly worse** than 512×512 for JPEG encoding because it
  has more pixels (307 k vs 262 k).

### 3.4 Hardware revision scope

Every result applies to **SentAI board v1.0 only**.  A future board
revision that rewires the MUX, adds a second CSI receiver, or
changes the sensor modules would invalidate the constants in this
paper but should preserve the *framework* (session-based
measurement, flip-on-EOF as a safe MUX-transition primitive, fault
counters as a measurement-integrity check).

---

## 4. Conclusion validity — are the statistics defensible?

### 4.1 Sample size

`n = 20` (E15) and `n = 40` (E16, E18) with the first sample
dropped.  With `σ ≈ 2 ms` on a ~60 ms mean this gives a 95 %
confidence interval of `±0.9 ms`, well below the effect sizes we
report (+9.5 ms from eDMA, +83 ms switch tax, −64 ms asymmetry
collapse).  See [statistical_notes.md](statistical_notes.md) for the
derivation.

### 4.2 No formal hypothesis testing

We do not compute p-values.  All of our "effects" are at least an
order of magnitude larger than the measurement noise; a t-test
against a null of "no change" would produce `p < 10⁻¹⁰` and not add
information.  Where we report a null result (Fix A having no
measurable effect), we say so explicitly and show the raw numbers so
a reviewer can confirm.

### 4.3 Cross-session comparisons rely on held-constant covariates

Every cross-session table in this paper holds model, scene, resolution
and sensor configuration constant, varying only the firmware build or
an A/B runtime flag.  If any of those covariates had drifted, it would
show up as a shift in the fixed-camera baseline — which empirically
remained at 62.6 ± 0.4 ms across [`s043`](../experiments/s043_e18_headtail_drain2/),
[`s044`](../experiments/s044_e18_headtail_drain2/), and
[`s045`](../experiments/s045_e18_post_refactor/) (see Table 4 of
[evaluation.md](evaluation.md)).  That agreement is the strongest
available evidence that the comparisons isolate the variable under
test.

### 4.4 Single-run sessions excluded from statistics

Sessions `s016`–`s020` and `s027`–`s030` contain a single data point
each (n = 1) and are not used for any mean comparison in this paper.
They are retained in the archive (Appendix C of
[experiments/README.md](../experiments/README.md)) because they
document the progression of single-shot pre/post-eDMA checks before
the x20 repetition protocol stabilised; they do not contribute
statistical weight to the headline results.

---

## 5. What we did not do, and why it is not a threat

For transparency, the following were considered and explicitly
declined in this iteration:

| Not done | Why not | When it would matter |
|---|---|---|
| Cross-device measurement | single prototype hardware | production deployment across N units |
| Thermal stress over minutes | 3-s measurement windows don't throttle | always-on field deployment |
| Power / current measurement | no instrumentation on the test rig | battery-bounded mission profile |
| FSIN master/slave sensor sync | hardware rework required, ≤ 2 ms residual asymmetry not worth the silicon change | if directional asymmetry became the dominant error source |
| 60 fps 720p mode | NXP driver binning-mode init missing; CSI-2 did not lock on our probe | a use case forced by a model whose per-frame cost fits a 17 ms budget |
| Virtual-channel-based parallel capture | requires a MIPI-bridge chip not on the board | a dual-sensor application that cannot tolerate any switch tax |

Each of these is a future-work item
([future_work.md](future_work.md)) rather than a concealed threat to
the results as reported.

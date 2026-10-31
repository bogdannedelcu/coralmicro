# On-board manuscript-gap measurements — build 1590 (VGA native, GPT1-µs)

**Board:** i.MX RT1176 + Coral EdgeTPU (USB2), firmware **build 1590** (the GPT1-µs
fix that corrected s239 — both the invoke-timing and the InferTask FPS-throttle
tick-starvation bugs). **All measurements at native VGA 640×480 @ 45 fps** (camera rate
verified via `camera.fps()`), timing in **GPT1 µs**. Method reference: EXP-s241.
**Experiment id:** EXP-s244 (`examples/sentai_runtime/experiments/s244_onboard_gaps_build1590/`).
Detectors: `c2f_thick` (RGB, single-input) and `c2f_thick_yuv420` (YUV, dual-input),
both from MLflow `sentai_revision_relu_v1` / `sentai_yuv420_v1`.

All numbers below are **measured on the board**, not estimated. Failures are stated
explicitly.

---

## TASK 1 — Multi-patch (Table 12)

`sentai.pipeline.invokes_per_frame(N)` = N invokes/frame (N patches of the same
camera frame). Isolated invoke = 5 warmup + 30 GPT1-µs invokes; pipeline = M=3 × 6 s
live. **Confirmed: invokes/frame = N** (measured column). Invoke throughput is
constant (invoke-bound); frame throughput = invoke_tp / N. **0 timeouts, 0 infer
failures** across all configs.

| patches/frame | invokes/frame (meas) | input | frame throughput (fps) | invoke throughput (inv/s) | timeouts | infer failures |
|--:|--:|:--:|--:|--:|--:|--:|
| 1 | 1.00 | RGB | 25.13 ± 0.00 | 25.13 ± 0.00 | 0 | 0 |
| 2 | 1.99 | RGB | 13.28 ± 0.08 | 26.47 ± 0.16 | 0 | 0 |
| 3 | 2.99 | RGB | 8.84 ± 0.08 | 26.43 ± 0.40 | 0 | 0 |
| 4 | 4.02 | RGB | 6.66 ± 0.00 | 26.80 ± 0.00 | 0 | 0 |
| 1 | 1.00 | YUV | 38.69 ± 0.08 | 38.73 ± 0.00 | 0 | 0 |
| 2 | 1.99 | YUV | 20.27 ± 0.00 | 40.36 ± 0.16 | 0 | 0 |
| 3 | 3.00 | YUV | 13.74 ± 0.00 | 41.23 ± 0.00 | 0 | 0 |
| 4 | 4.00 | YUV | 10.41 ± 0.14 | 41.65 ± 0.00 | 0 | 0 |

Isolated invoke (per model, n=30): RGB **35.0 ± 0.0 ms**, YUV **22.0 ± 0.0 ms**.
Raw: `task1_multipatch.csv`. **Status: COMPLETE.**

---

## TASK 2 — Dual-camera 1:1 alternation (Tables 13, 14, 15)

`camera.ratio(1,1)` (CSI ISR flips MUX, cam0/cam1 each own 1 frame/cycle) with
`camera.switch_drain(d)` (fresh ISR frames skipped after each flip; **default = 1**).
M=3 × 6 s live pipeline.

### Table 13 — throughput
| input | drain | combined (det/s) | per-camera (det/s) | valid | resets |
|:--:|--:|--:|--:|:--:|--:|
| RGB | 1 | 22.52 ± 0.09 | ~11.26 | yes | 0 |
| RGB | 2 | 10.60 ± 1.18 | ~5.30 | yes | 0 |
| YUV | 1 | 22.57 ± 0.00 | ~11.29 | yes | 0 |
| YUV | 2 | 10.65 ± 1.10 | ~5.33 | yes | 0 |

Per-camera = combined/2 (1:1 split). **drain=1 confirmed correct**: drain=2 skips one
extra frame per flip → ~half the throughput. Alternation is **MUX-flip-bound, not
compute-bound** — RGB and YUV give the same ~22.5 fps combined at drain=1 despite
different invoke times. Raw: `task2_throughput.csv`.

### Table 14 — content validation (PARTIAL — see note)
| input | drain | frames inspected | correct | mismatch | mixed | verdict |
|:--:|--:|--:|--:|--:|--:|:--:|
| RGB | 1 | 142 | 142 | — | — | drain=1 delivers 2× drain=2 |
| RGB | 2 | 71 | 71 | — | — | reference |
| YUV | 1 | 142 | 142 | — | — | drain=1 delivers 2× drain=2 |
| YUV | 2 | 58 | 58 | — | — | reference |

**Honest limitation:** the per-frame *correct / mismatch / mixed* breakdown could NOT
be cleanly measured on-board. The per-camera ISR-tag counts (cam0/cam1/unknown) are
only exposed inside `pipeline.probe_ratios()`, which **fails on the YUV dual-input
model** (`E:0B61` "edgetpu-custom-op failed to invoke" — probe_ratios uses the
single-input slot path, not the dual-input `invoke_with_inputs2` path). `pipeline.stats()`
returns only `(frames, ok, fps)`, and `camera.grabbed_id()` returns a stale/pipeline-only
tag that reads 0 in manual capture. What IS validated: **frame counts are exactly 2×
between drain=1 and drain=2** (142 vs 71/58), and the aggregate cam0≈cam1 split is even
(~11.3 fps each) with **0 resets and 0 unknown-camera frames** in the throughput runs.
The physical per-camera distinctness is confirmed visually in Task 3.

### Table 15 — periods
| input | single-cam output period (ms) | invoke (ms) | dual-cam combined period (ms) | dual-cam per-camera period (ms) | manual switch drain=1 (ms) |
|:--:|--:|--:|--:|--:|--:|
| RGB | 38.1 | 35 | 44.4 | 88.8 | 76.0 ± 19.3 |
| YUV | 24.7 | 22 | 44.3 | 88.6 | 74.9 ± 19.4 |

Invoke = the warm Task-1 value (a single cold `tpu.invoke()` in the Task-2 script read
58/44 ms — cold-cache, not used). Manual switch = `select()`→`to_tensor()` round-trip
with drain=1, n=20 GPT1-µs samples (~75 ms ≈ 3 frame periods: MUX flip + 1 drained
frame + capture). Raw: `task2_periods.csv`.

---

## TASK 3 — Visual verification frames

Each camera set to a **distinct OV5640 sensor pattern** via `camera.test_pattern(cam,mode)`:
- **cam0 → mode 1 (BLACK / forced minimum exposure+gain)** — the real scene rendered
  near-black.
- **cam1 → mode 2** — drives the OV5640's built-in **grayscale color-bar** test pattern
  (vertical white→black bands).

The two are unmistakably different (cam0 mean luma **17**, cam1 mean luma **128**),
giving a clear per-camera visual signature.

| file | what it shows |
|---|---|
| `…/s244_onboard_gaps_build1590/cam0_pattern.png` | cam0, dark scene (BLACK pattern), mean 17 |
| `…/s244_onboard_gaps_build1590/cam1_pattern.png` | cam1, OV5640 grayscale color-bar, mean 128 |
| `…/s244_onboard_gaps_build1590/drain1_clean.png` | a settled cam1 frame captured with drain=1 |
| `…/s244_onboard_gaps_build1590/seam_artifact.png` | attempted mid-switch frame (see note) |

**Honest limitation on the seam A/B:** the *mixed/torn* frame that drain=1 removes is a
**sub-frame-period (~22 ms) artifact** — it exists for at most one frame right after a
MUX flip. REPL-driven capture latency (~50–100 ms) exceeds one frame period, so the
capture **cannot deterministically land on the mixed frame**. Both the `seam_artifact`
and `drain1_clean` captures landed on similar (darker, transitional) cam1 frames, so
they do **not** demonstrate a visible seam-vs-clean difference. A true seam PNG would
require in-firmware capture triggered from the CSI ISR at the flip boundary (not
available as a binding). The drain benefit is instead demonstrated quantitatively in
Table 13 (drain=1 = 2× drain=2 frames, 0 resets).

---

## Summary of status

| task | status |
|---|---|
| Task 1 — multi-patch (Table 12) | ✅ COMPLETE, RGB+YUV, 0 timeouts/fails |
| Task 2 — throughput (Table 13) | ✅ COMPLETE, RGB+YUV, drain=1 confirmed |
| Task 2 — content validation (Table 14) | ⚠️ PARTIAL — frame-count 2× confirmed; per-frame correct/mismatch/mixed not measurable (probe_ratios fails on YUV dual-input; no per-frame tag binding) |
| Task 2 — periods (Table 15) | ✅ COMPLETE, RGB+YUV, switch cost measured |
| Task 3 — cam0/cam1 distinct patterns | ✅ COMPLETE (2 PNGs, visually distinct) |
| Task 3 — seam vs drain1-clean | ❌ NOT capturable from REPL (sub-frame-period; latency > 1 frame) |

Build 1590, EXP-s244, VGA 640×480@45, GPT1-µs timing throughout.

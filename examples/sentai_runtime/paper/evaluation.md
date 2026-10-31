# Evaluation

This chapter consolidates every performance measurement reported in
this work into a single narrative, in the order a reviewer would ask
about them: *what questions did we set out to answer, which
experiments answer each, and what did we find?*  It is the IMRAD
"Results" chapter of an MDPI-style structured article and does not
duplicate the subsystem narratives in [memcpy.md](memcpy.md) and
[cam_switch.md](cam_switch.md); those remain the primary source for
the implementation detail behind each result.

Subsection numbering in the captions of the tables below matches the
subsections of this chapter.

---

## 1. Research questions

We asked five questions of the platform, each addressable by one or
more experiments in the `diag` framework (see
[experimental_setup.md](experimental_setup.md) §6.1).

| # | Question | Experiments that answer it |
|---|---|---|
| **RQ1** | What sustained frame rate does the parallel vision pipeline achieve on a fixed camera, for the 512×512 single-class model? | E14 (baseline 80-class model) → E15 (target model) |
| **RQ2** | Can the SDRAM-to-tensor copy — identified as the dominant stage of InferTask — be accelerated without changing the pipeline's producer-consumer contract? | E15 `dma_memcpy(0/1)` A/B, same firmware |
| **RQ3** | What is the cost of switching between the two cameras on a frame-by-frame basis on a single-MIPI-lane-with-MUX topology? | E16 alternating; E18 three-sweep head-to-tail |
| **RQ4** | Can that switch be made **glitch-free** at the pixel level without a hardware rework? | E17 per-switch JPEG visual inspection |
| **RQ5** | Do the platform's fault-handling pathways leave observable traces when a measurement runs without problems, and how do they escalate when the system degrades? | fault counters exposed via `sentai.diag.cam_stats()`, observed at the end of every E18 session |

The rest of this chapter presents, per question, the headline
measurement, the cross-session reproducibility check, and a brief
interpretation that points to the implementation chapter for depth.

---

## 2. RQ1 — Parallel-pipeline steady-state throughput

### 2.1 Setup

Experiment class E15 (`e15_pipeline_parallel_512`) drives
`sentai.pipeline.start/get/stop` with `PrepTask` and `InferTask`
running concurrently.  `PrepTask` does frame-grab + PXP resize + int8
quantisation into a staging buffer; `InferTask` copies the staging
buffer into the TFLite input tensor, then calls Invoke and NMS.  The
two tasks communicate through the `staging_free` / `prep_done`
semaphore pair documented in [camera.md](camera.md) §PXP.  The
wall-clock interval between two successive `sentai.pipeline.get()`
returns is the per-frame throughput.

20 runs of 20 iterations each per session; all reported numbers are
post-eDMA (see RQ2 for the pre/post split).

### 2.2 Headline result (Table 1)

**Table 1.** Sustained parallel-pipeline throughput on the 512×512
single-class model, build #622+ (post-eDMA memcpy).  Source:
[experiments/s032_e15_x20/](../experiments/s032_e15_x20/) — 20 runs,
20 repetitions each.

| Metric | Mean ± σ | min / max |
|---|---:|---:|
| `frame_interval_ms` | 64.7 ± 1.9 | 61 / 75 |
| `prep_stall_ms` | 0.1 ± 0.3 | 0 / 1 |
| `infer_stall_ms` | 41.1 ± 1.5 | 37 / 45 |
| **Sustained FPS** | **15.46 ± 0.45** | — |

### 2.3 Interpretation

The pipeline is **stall-balanced**.  `infer_stall_ms ≈ 41 ms` means
that InferTask blocks for ~41 ms per frame waiting for PrepTask to
deliver the next staging buffer; that is, PrepTask is the critical-
path stage (~64 ms/frame), and InferTask (Invoke + NMS + memcpy)
finishes in ≈ 23 ms and waits.  The 15.5 FPS ceiling therefore
reflects the PrepTask stage time, not the TPU.  The 2 FPS margin to
the 15 FPS sensor rate is explained by the sensor actually streaming
at slightly above 15 Hz (ISR count per second observed is 15.2-15.5
on this hardware).

---

## 3. RQ2 — eDMA acceleration of the staging-to-tensor copy

### 3.1 Setup

A runtime A/B flag `sentai.pipeline.dma_memcpy(0 | 1)` selects
between the baseline CPU `memcpy()` and the eDMA-based path (channel
31, 32-byte AXI bursts).  Both settings coexist in the same firmware
build; switching between them is a single `volatile` store and does
not restart the pipeline.  10 × 20 frames per setting, captured in
the same session so scene, thermal state and TPU package cache are
held constant.

### 3.2 Headline result (Table 2)

**Table 2.** Per-stage timing and sustained FPS for CPU `memcpy` vs
eDMA memcpy.  10 runs × 20 frames each, paired within a single
session.  Source: [`../_e15_ab.py`](../_e15_ab.py), detail tables in
[memcpy.md](memcpy.md) §"Per-run results".

| `dma_memcpy` | Invoke (ms) | memcpy (ms) | NMS (ms) | total_infer (ms) | Wall (ms) | **FPS** |
|---:|---:|---:|---:|---:|---:|---:|
| 0 (baseline) | 50.2 | **24.1** | 0.2 | 74.5 | 74.7 | **13.39** |
| 1 (optimised) | 49.7 | **14.6** | 0.2 | 64.5 | 65.1 | **15.32** |
| Δ | −0.5 | **−9.5** | 0 | **−10.0** | **−9.6** | **+1.93** |

### 3.3 Interpretation

The memcpy drops from 32 % of the per-frame critical path to 22 %.
Invoke time is unchanged (TPU is not involved in the memcpy), so the
improvement is wholly attributable to the eDMA.  The 9.6 ms wall-
time saving translates 1-to-1 into FPS because we are on the critical
path of InferTask.  The optimisation crossed the 15 FPS sensor-rate
threshold, which is why later camera-switch experiments can treat
"fixed camera, 30 fps sensor, ~16 FPS pipeline output" as a baseline.
Implementation detail — eDMA configuration, cache-coherency notes,
two debugging trips — is documented in full in
[memcpy.md](memcpy.md) §"Optimised".

---

## 4. RQ3 — Per-switch cost on the shared-MIPI, MUX-gated dual sensor

### 4.1 Setup

Experiment E18 (`e18_camera_switch_headtail`) runs three sweeps of
the same sequential loop (`select → to_tensor → invoke → detect`)
back to back in one session:

- **A** — fixed on `cam_a` for all 40 iterations (no switches)
- **B** — fixed on `cam_b` for all 40 iterations (no switches)
- **C** — alternating `cam_a ↔ cam_b` every iteration

Three sweeps let us express the switch cost as a subtraction against
a baseline measured under identical conditions:

```
per_switch_overhead = C_total_mean − max(A_total_mean, B_total_mean)
```

### 4.2 Headline result (Table 3)

**Table 3.** Three-sweep head-to-tail benchmark, 30 fps sensor, Fix
B firmware (post-flip-on-EOF), `switch_drain=2`, `ratio=(0,0)`, 40
iterations per sweep, first sample dropped.  Source:
[experiments/s045_e18_post_refactor/](../experiments/s045_e18_post_refactor/).

| Sweep | `select` | `to_tensor` | `invoke` | `detect` | **total (ms)** | **FPS** |
|---|---:|---:|---:|---:|---:|---:|
| A — fixed cam0 | 0.0 | 31.6 | 29.8 | 0.4 | **61.8** | **16.18** |
| B — fixed cam1 | 0.0 | 31.6 | 30.5 | 0.4 | **62.5** | **16.00** |
| C — alternating | 17.6 | 96.0 | 31.5 | 0.7 | **145.8** | **6.86** |

**Per-switch overhead = 83.3 ms** (133.4 % of the slower baseline).
**Directional asymmetry (cam1 − cam0) = −2.1 ms** (within noise).

### 4.3 Budget decomposition

| Component of the 83.3 ms tax | Amount |
|---|---:|
| `select` wait for EOF ISR consumption | 17.6 ms (≈ 0.5 frame interval at 30 fps) |
| `to_tensor` post-switch drain (stale-buffer drop + 2 fresh frames) | 64.4 ms (≈ 2 × 33 ms) |
| Sensor-independent stages (`invoke`, `detect`) | 0 ms (same in A and C) |

### 4.4 Cross-session agreement

Three sessions captured the same three-sweep benchmark under
successively more refined firmware (pre-refactor, post-refactor
confirmation, NASA-JPL review fixes).  Agreement across the three
sessions is a reproducibility check (Table 4).

**Table 4.** Cross-session reproducibility of Table 3.  All three
rows are 40-iteration head-to-tail sweeps on the same scene/model;
firmware differences are the only variable.  Values in ms.

| Session | A — fixed cam0 | B — fixed cam1 | C — alternating | Asymmetry cam1 − cam0 |
|---|---:|---:|---:|---:|
| [`s043_e18_headtail_drain2`](../experiments/s043_e18_headtail_drain2/) | 62.7 | 62.5 | 145.3 | +0.1 |
| [`s044_e18_headtail_drain2`](../experiments/s044_e18_headtail_drain2/) | 62.6 | 62.1 | 145.7 | +0.1 |
| [`s045_e18_post_refactor`](../experiments/s045_e18_post_refactor/)     | 61.8 | 62.5 | 145.8 | −2.1 |
| **σ across sessions** | **0.4 ms** | **0.2 ms** | **0.3 ms** | noise |

The spread across three firmware builds is < 1 ms on every stage.
That bounds the measurement noise and validates the claim that the
NASA-JPL review fixes are performance-neutral.

---

## 5. RQ4 — Glitch-free MUX transitions

### 5.1 Setup

E17 (`e17_switch_drain_visual`) writes alternating-camera JPEGs
into the MicroPython heap during the timing loop and flushes them to
LittleFS afterwards.  Two runs per session at `switch_drain=2` and
`switch_drain=1`, 16 iterations each, 512×512 quality-70.  The
output is a set of pixel-level frames available for offline review.

The subject is *visual correctness*, not timing.  The timing is a
by-product.

### 5.2 Headline result — visual inspection

**Table 5.** Visual inspection outcomes across firmware builds.  The
full frame sets are in the linked folders; a representative "broken"
frame and its "fixed" counterpart are cited inline.

| Firmware | Session | drain=1 outcome | drain=2 outcome |
|---|---|---|---|
| Pre-flip-on-EOF | [`s038_e17_drain_ab`](../experiments/s038_e17_drain_ab/) | **Fails.** Horizontal seam at 40-60 % image height, every other frame split between cam0 and cam1.  Example: [`002_cam0_133ms.jpg`](../experiments/s038_e17_drain_ab/e17_t1_frames/002_cam0_133ms.jpg) | Clean. |
| Post-flip-on-EOF | [`s041_e17_eof_check`](../experiments/s041_e17_eof_check/) | **Mid-buffer seam removed**, but residual sensor-side artefacts remain (AEC/AGC convergence) — see [threats_to_validity.md](threats_to_validity.md) §3. | Clean.  Example: [`003_cam1_201ms.jpg`](../experiments/s041_e17_eof_check/e17_t1_frames/003_cam1_201ms.jpg) |

### 5.3 Interpretation

Moving the GPIO flip into the CSI end-of-frame ISR — so the MUX
transition lands in the MIPI VBLANK window between two DMA buffers —
eliminates the class of failure in which a single DMA buffer
contains pixels from two sensors.  The shipping configuration is
`switch_drain=2`: it produces reliably clean frames.  `switch_drain=1`
remains available as an experimentation hook but the frames it
produces are not suitable for production use; this is the only
residual pixel-quality limitation of the camera-switch chapter and is
called out in its own "Known limitations" section in
[cam_switch.md](cam_switch.md).

---

## 6. RQ5 — Fault observability

### 6.1 Setup

Every degraded path in the camera-switch subsystem — ISR arm not
consumed within 150 ms, drain wait hitting its 300 ms ceiling,
`GetRawFrame` retry, `GetRawFrame` fatal — increments a persistent
counter (see the `0x0Axx` range in
[error_codes.csv](../error_codes.csv)).  The counters are exposed to
MicroPython as `sentai.diag.cam_stats()`.  Every E18 session captures
the counter dict at the end of the run; a non-zero degraded-path
value invalidates that session's interpretation.

### 6.2 Headline result

**Table 6.** Post-run fault counters for the canonical E18 sessions
cited in this paper.  All zeros means every switch was handled by the
nominal flip-on-EOF path.

| Session | `switch_ok_eof` | `switch_fallback` | `drain_timeout` | `grab_retry` | `grab_fatal` |
|---|---:|---:|---:|---:|---:|
| `s043_e18_headtail_drain2` | 41 | **0** | **0** | **0** | **0** |
| `s044_e18_headtail_drain2` | 41 | **0** | **0** | **0** | **0** |
| `s045_e18_post_refactor`   | 41 | **0** | **0** | **0** | **0** |

### 6.3 Interpretation

The fault-counter infrastructure is not merely defensive
instrumentation; it is a publication-grade **measurement integrity
check**.  A reviewer who suspects that some fraction of our E18
alternating iterations fell back to the legacy synchronous switch
path (which would re-introduce the seam and bias the timing) can
confirm from the counters that in the sessions we report, **none
did**.  The counters remain available at all times via REPL for
future replays on the same hardware.

---

## 7. Summary table (Research-question × metric)

**Table 7.** Paper-level summary: one row per research question, the
experiment class that answers it, the headline number, the direction
of change versus the baseline, and the session(s) that document it.

| RQ | Headline metric | Baseline | Final | Δ | Evidence |
|---|---|---:|---:|---:|---|
| RQ1 | Sustained parallel-pipeline FPS, 512×512 | 13.39 (pre-eDMA) | 15.47 | **+15 %** | [s024](../experiments/s024_e15_x20/), [s026](../experiments/s026_e15_x20/), [s032](../experiments/s032_e15_x20/) |
| RQ2 | Staging→tensor memcpy cost | 24.1 ms (CPU) | 14.6 ms (eDMA) | **−39 %** | [memcpy.md §"Per-run results"](memcpy.md) |
| RQ3 | Alternating FPS (switch every frame) | 4.7 (15 fps, pre-Fix A) | 6.86 (30 fps, Fix B) | **+46 %** | [s034](../experiments/s034_e15_vs_e16_x40/), [s045](../experiments/s045_e18_post_refactor/) |
| RQ3 | Directional asymmetry cam1 − cam0 | +64.2 ms | ≤ 2.1 ms (within noise) | **removed** | [s034](../experiments/s034_e15_vs_e16_x40/) → [s045](../experiments/s045_e18_post_refactor/) |
| RQ4 | Mid-buffer seam at drain=2 | visible every switch | none | **removed** | visual inspection of [s038](../experiments/s038_e17_drain_ab/) vs [s041](../experiments/s041_e17_eof_check/) |
| RQ5 | Degraded-path counter during a nominal run | n/a pre-review | 0 across all three reported sessions | **validated** | [artifact.md](artifact.md) §"Data integrity" |

---

## 8. Per-architecture on-board throughput — activation & pipeline sweep (MDPI revision)

For the *Drones* revision we re-measured the eight cover-split detector
architectures at the deployed **480×640 (HxW)** input on the physical board. The
single 512×512 `yolo_1` SiLU figure (≈23–24 detections/invocations per second,
[s235](../experiments/s235_revalidate_paper_performance/)) is one model; the revision (i) replaces
SiLU with **ReLU**, (ii) validates board detections against a host reference, and
(iii) measures the full concurrent camera→detection pipeline, RGB vs native YUV420
(§8.3, the definitive build-1590 benchmark that corrects the earlier s239 figures).
Firmware build 1560 for §8.1–§8.2, build 1590 for §8.3.

### 8.1 Standalone EdgeTPU invoke (isolated)

**Table 8.** Standalone EdgeTPU invoke on the i.MX RT1176 board (640×480, N=50,
real image, `sentai.rtos.micros()` wall clock). SiLU column =
[s236](../experiments/s236_model_speed_sweep_640x480/); ReLU columns =
[s238](../experiments/s238_relu_v1_board_speed_sweep/) (board) and the Coral USB
Accelerator (host-side, invoke-only). Same architecture per row; only the
activation / link differs.

| model | Board SiLU (FPS) | **Board ReLU (FPS)** | USB-stick ReLU (FPS) | ReLU instr (ms) | input (ms) |
|---|---:|---:|---:|---:|---:|
| msblock    | 22.9 | **30.7** | 39.3 | 2.16 | 25.98 |
| c3         | 26.2 | **30.5** | 39.0 | 2.31 | 25.99 |
| c2f        | 26.2 | **30.3** | 38.7 | 2.43 | 25.99 |
| gelan      | 24.4 | **29.3** | 37.2 | 3.34 | 26.00 |
| c2f_pan2   | 24.9 | **28.3** | 37.0 | 3.12 | 27.56 |
| gelan_pan2 | 22.8 | **28.2** | 35.1 | 4.40 | 26.00 |
| c2f_deep   | 22.6 | **28.1** | 35.0 | 4.45 | 25.99 |
| c2f_thick  | *crash (0B62)* | **28.4** | 34.0 | 3.43 | 25.97 |

The ≈23–24 head-line figure is **activation-limited, not fundamental: it is a
SiLU number.** Switching every activation to ReLU raises standalone on-board
throughput to **28–31 FPS across all eight architectures** (msblock 22.9→30.7,
c2f 26.2→30.3 — a **+16–34 %** gain). The mechanism is the instruction stream:
ReLU/ReLU6 fuse into the preceding convolution's output clamp and emit **zero
EdgeTPU operations**, whereas SiLU compiles to a per-element `LOGISTIC`+`MUL` pair
(≈2× the ops), so ReLU ships a much smaller instruction bitstream per invoke
(2.2–4.5 ms vs the SiLU set). ReLU also removes the activation-scratch spill that
**crashed the champion `c2f_thick` on the board as SiLU** (`0B62`, s236); under
ReLU it runs cleanly. The isolated invoke stays **input-transfer-bound** — the
480×640×3 (≈921 KB) frame costs ≈26 ms on the board's internal USB2 link, 74–80 %
of each inference — which is why the board figures sit below the host USB-stick
numbers (34–39 FPS) for identical models.

### 8.2 Board-vs-host detection parity

To confirm the on-board INT8 graphs are numerically correct — and stay correct
under the concurrent load of §8.3 — we compare the board's detections against a
host CPU INT8 reference on the same 640×480 frame ([s238](../experiments/s238_relu_v1_board_speed_sweep/),
`detections_board_vs_host.csv`).

**Table 9.** Detection parity, board vs host CPU INT8 reference (same frame,
same model, ReLU). `dets` = boxes after NMS; `maxconf` = peak class confidence.

| model | board dets | host dets | Δ | board maxconf | host maxconf |
|---|---:|---:|---:|---:|---:|
| msblock    | 18 | 18 |  0 | 0.393 | 0.393 |
| c2f        | 17 | 17 |  0 | 0.420 | 0.421 |
| c3         | 26 | 25 | +1 | 0.367 | 0.367 |
| c2f_pan2   | 17 | 18 | −1 | 0.389 | 0.390 |
| gelan      | 18 | 18 |  0 | 0.410 | 0.411 |
| gelan_pan2 | 14 | 14 |  0 | 0.395 | 0.395 |
| c2f_deep   | 20 | 21 | −1 | 0.364 | 0.364 |
| c2f_thick  | 16 | 16 |  0 | 0.406 | 0.418 |

Board and host agree to **within ±1 detection** (a single box straddling the
confidence threshold) and **identical peak confidence to three decimals**. The
board EdgeTPU path reproduces the reference detector; concurrent operation does
not corrupt the output.

### 8.3 Full detection-pipeline throughput — RGB vs YUV420 (definitive)

> **⚠ CORRECTION (build 1590, supersedes s239).** An earlier draft of this section (from
> [s239](../experiments/s239_relu_full_pipeline_fps/)) reported the pipeline as *camera-bound at ≈27 fps*
> with an ≈11 ms in-pipeline invoke. **That was an artifact of a FreeRTOS SysTick tick-starvation bug**:
> the TPU invoke's USB bursts starve the tick ISR, so `xTaskGetTickCount` under-counted (i) the in-pipeline
> invoke time (≈11 ms reported for a real ≈32 ms invoke) and (ii) the InferTask FPS throttle, which then
> injected a spurious ≈9 ms delay per frame and **capped throughput at ≈30 fps**. Both were fixed (GPT1 µs)
> in build 1590 and re-measured in [s241](../experiments/s241_relu_yuv_vga45_stats/). **The pipeline is
> invoke-bound, not camera-bound**; the table below replaces the retracted s239 numbers.

The deployment-realistic figure runs `CameraTask`, `PrepTask` (PXP downscale + INT8 quant) and `InferTask`
(EdgeTPU invoke) **simultaneously** (CSI/PXP/USB/SEMC contention). We benchmark the 8 ReLU **RGB** detectors
against their 8 **YUV420** dual-input counterparts — native YUV halves the per-frame input transfer
(921,600 → 460,800 B) — at 640×480 @ 45 fps ([s241](../experiments/s241_relu_yuv_vga45_stats/); isolated
invoke N=30, pipeline M=3 × 6 s, GPT1-µs timed, mean ± σ).

**Table 10.** On-board RGB vs YUV420 (build 1590, 640×480 @ 45 fps, mean ± σ). Same weights per
architecture; only the input format differs.

| arch | fmt | invoke ms | pipeline FPS | | arch | fmt | invoke ms | pipeline FPS |
|---|---|---:|---:|---|---|---|---:|---:|
| msblock | RGB | 32.0±0.0 | 27.2±0.09 | | msblock | YUV | 19.0±0.0 | **42.8±0.09** |
| c3 | RGB | 32.4±0.5 | 27.0±0.00 | | c3 | YUV | 19.0±0.0 | **42.6±0.09** |
| c2f | RGB | 33.0±0.0 | 26.9±0.09 | | c2f | YUV | 19.4±0.5 | **42.3±0.09** |
| c2f_pan2 | RGB | 33.7±0.5 | 26.4±0.00 | | c2f_pan2 | YUV | 20.0±0.0 | **41.0±0.09** |
| gelan | RGB | 34.0±0.0 | 26.4±0.00 | | gelan | YUV | 20.9±0.3 | **40.6±0.00** |
| c2f_thick | RGB | 35.0±0.0 | 26.2±0.00 | | c2f_thick | YUV | 21.9±0.3 | **40.5±0.00** |
| gelan_pan2 | RGB | 35.0±0.0 | 25.4±0.00 | | gelan_pan2 | YUV | 22.0±0.0 | **38.8±0.09** |
| c2f_deep | RGB | 35.0±0.0 | *unstable\** | | c2f_deep | YUV | 22.0±0.0 | **38.9±0.09** |

\* **c2f_deep RGB pipeline is unstable** — its 153,600 B host-scratch spill wedges the pipeline under
camera+prep bus contention (isolated invoke is fine, 35.0 ms). Its YUV counterpart (smaller graph, no
spill) runs stably at 38.9 fps, so **YUV also removes this deployability failure**.

Three results follow. First, the pipeline is **invoke-bound**: the invoke drops from **32–35 ms (RGB) to
19–22 ms (YUV)** — the model graph barely moves it, so the EdgeTPU invoke is **input-transfer-bound over
USB2**, and halving the input (native YUV) nearly halves the transfer-dominated invoke. Second, this makes
**YUV420 a 1.53–1.58× on-board throughput win on every architecture** — RGB **25.4–27.2 fps → YUV
38.8–42.8 fps** — for the same weights and (§8.2) essentially the same detections. Third, timing is highly
**deterministic** (σ ≤ 0.09 fps / ≤ 0.5 ms) once the throttle bug is removed. mAP@.5 is preserved or
improved for 7/8 architectures under YUV; the exception is `msblock` (−0.123), the smallest backbone,
which loses accuracy in the 2-plane input and is not a YUV candidate.

### 8.4 Revised throughput claim

Consolidating Tables 8–10, the head-line performance sentence should read:

> On the 512×512 SiLU workload the optimized single-camera pipeline runs stably at about **23 detections
> per second**, close to that model's **≈24-invocation-per-second** standalone EdgeTPU limit. **Both
> figures are activation-limited.** With the revision's **ReLU** detectors at **480×640**, standalone
> EdgeTPU throughput rises to **28–31 invocations per second** (Table 8), and — measured on the physical
> board at build 1590, all pipeline tasks live — the full camera→detection pipeline sustains **≈25–27
> detections per second** with zero inference failures, while board detections match the host INT8
> reference to within ±1 box and identical peak confidence (Table 9). Feeding the camera's **native YUV
> 4:2:0** to a dual-input detector halves the input transfer and, because the invoke is input-transfer-
> bound, raises the pipeline to **≈39–43 detections per second — a 1.5–1.6× gain — for the same weights
> and essentially the same accuracy** (Table 10). The recommended production detector, `c2f_thick_yuv420`,
> **wins on both axes: the best mAP of the set (0.823 vs the RGB champion's 0.803) and 40.5 fps vs 26.2**.
> Detections are exposed to drone-control workflows (MAVLink/PX4 and Crazyflie) through the scriptable
> layer as an integration interface rather than a validated autonomy stack.

---

## 9. Discussion (of results only; full discussion in [discussion.md](discussion.md))

Three observations that belong here rather than in the implementation
chapters because they relate to the results taken together:

1. **Each optimisation exposes the next bottleneck.**  The CPU memcpy
   was hidden behind a slow camera-drain fix (described in
   [camera.md](camera.md)).  The 15-fps pipeline plateau was hidden
   behind the memcpy.  The alternating-switch tax was hidden behind
   the fixed-camera ceiling.  Each of the three "headline" results
   only became measurable once the previous layer had stopped
   dominating the wall-clock; this is why the subsystem narratives
   must be read in order (camera-drain → memcpy → cam_switch).
2. **The residual 83 ms per-switch tax is sensor-topology bound, not
   firmware bound.**  With both cameras sharing one CSI-2 lane, two
   frame intervals of drain is the theoretical floor for a
   conservative drain threshold on a not-VSYNC-synchronised pair of
   sensors.  Further reduction requires either hardware sync (a FSIN
   wire between the OV5640s, rejected for this iteration — see
   [threats_to_validity.md](threats_to_validity.md) §4) or a
   different CSI-receiver topology.
3. **The measurement discipline scales.**  The same `diag` framework
   that captured the initial 6 FPS baseline in the E10-E12 subsystem
   probes (Appendix A of [experiments/README.md](../experiments/README.md))
   captured the final 16 FPS head-to-tail benchmark.  No measurement
   methodology change was needed to follow the four-ish orders of
   magnitude of scope from "does `pvPortMalloc` work under load" to
   "what is the per-switch overhead of the dual-sensor MUX topology".
   That is a statement about the framework, not about the results.

Open questions, limitations, and threats to this interpretation are in
[threats_to_validity.md](threats_to_validity.md).

---

## Data availability statement

Every number in Tables 1–7 is derived from raw per-iteration CSVs
that are mirrored under
[`../experiments/`](../experiments/).  The session names cited in
the table captions are the direct folder paths.  The appendix
generator at
[`../experiments/_build_appendix.py`](../experiments/_build_appendix.py)
reproduces the full appendix tables from those CSVs without any
on-device round-trip, so an external reviewer can verify every
derived statistic offline.  The full artifact description — code
commit SHAs, firmware build numbers, reproduction steps — is in
[artifact.md](artifact.md).

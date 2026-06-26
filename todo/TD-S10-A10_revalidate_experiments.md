# TD-S10-A10 - Revalidate Paper Performance Experiments

## Goal

Re-measure the performance-improvement section of the paper with timing sources
that are valid for MCU + USB EdgeTPU wall-clock behavior.

This task exists because the original Section 4.2 measurements mixed several
different timing domains:

- DWT cycle counters, which are useful for CPU-active intervals but are not a
  physical wall-clock measure for USB transfers that complete through EHCI,
  interrupts, DMA, and RTOS scheduling;
- `sentai.rtos.ticks_ms()` / `xTaskGetTickCount()`, which undercounted the
  blocking USB invoke path in the 512x512 TPU recheck;
- frame counters and camera ISR counters, which are still useful but must be
  clearly separated from TPU invoke timing;
- host-side pycoral measurements, which are separate from board measurements
  and must not be mixed with MCU claims.

The A10 objective is to produce a corrected, traceable, table-by-table data set
for the paper's runtime-performance chapter.

## Paper Scope

A10 covers the current paper section:

- 4.2.1 Overall Performance Progression
- 4.2.2 Pure TPU Throughput
- 4.2.3 Stable End-to-End Pipeline Throughput
- 4.2.4 Multi-Patch Throughput
- 4.2.5 Dual-Camera Switching and Alternation
- 4.2.6 Reality-to-Detection Latency
- 4.2.7 Summary

The draft currently has duplicate numbering for "Table 8".  A10 treats table
identity by section and content, not by the duplicated draft number.  B10 should
emit corrected table names/numbers after the reruns are complete.

## Measurement Rules

1. Board performance claims must be measured on the physical board.
2. TPU invoke wall time must use a real wall-clock source:
   `sentai.rtos.micros()` / `TimerMicros()` on board, plus host wall-clock
   sanity checks when useful.
3. USB transfer breakdown must use URB wall-time instrumentation, not DWT as
   wall time.
4. DWT may be kept only for CPU-active diagnostic context and must be labeled
   as cycles/active time, not transfer duration.
5. `sentai.rtos.ticks_ms()` and `xTaskGetTickCount()` are not accepted as the
   primary timing source for EdgeTPU USB invoke duration.
6. Camera-only FPS may use camera ISR/frame counters if the counter path is
   documented and does not derive from the affected TPU timing path.
7. Pipeline FPS must be computed from frame counts over measured wall time,
   not from a requested sleep duration or a tick-derived `wall_ms`.
8. Per-stage pipeline timing must be re-instrumented or interpreted carefully:
   existing `prep_stats`, `infer_stats`, `pipeline.calibrate`, and
   `invoke_ms_sum` are contaminated by tick-based timing.
9. Every run must store scripts, firmware build logs, board logs, pulled
   FileX/FlightRecorder artifacts, summaries, and source diffs under the
   experiment folder.
10. Large board asset transfer should use mounted drive / MSC, not HTTP upload.

## Artifact Root

B10 owns:

`examples/sentai_runtime/experiments/s235_revalidate_paper_performance/`

Each rerun must use an `iterNNN_*` subfolder.  Board-side mission logs should
also be written to FileX through `sentai.fr` / `sentai.fs` where possible, then
copied back into the same host-side iteration folder.

## Tables To Revalidate

### Overall Progression

Original claims:

- pure TPU standalone: 32 FPS -> 73-75 FPS;
- end-to-end single-patch pipeline: 1.8 FPS -> 41-42 FPS;
- multi-patch: 48.5 FPS and 56 FPS TPU;
- camera switch latency around 14 ms;
- continuous 1:1 alternation: 8.7 FPS -> 19.5 FPS.

Required output:

- corrected baseline/final FPS values;
- stability/failure-rate values;
- note which values are obsolete because they came from invalid TPU timing;
- retain camera-only claims only if rerun or traceably counter-based.

### Pure TPU Optimization Tweaks

Original claims:

- 64 KB chunks: 32 FPS;
- 128 KB chunks: 27 FPS;
- 33 KB chunks: 73 FPS;
- zero-copy: 75 FPS;
- persistent semaphore: 75.9 FPS;
- heap-free header path: 75.9 FPS.

Required output:

- rerun the chunk/optimization matrix with wall-clock invoke timing;
- keep the same model and input tensor shape used by the paper;
- record bytes per phase and URB wall-time breakdown for each relevant step.

### Standalone Invoke Breakdown

Original Table 9 used DWT-derived timing and is invalid as physical USB wall
time.

Required output:

- corrected per-invoke wall time;
- URB wall-time by phase: parameters, instructions, input, output, event;
- bytes per invoke;
- residual interpretation only if it is defensible after wall-clock accounting.

### Stable End-To-End Pipeline

Original claims:

- end-to-end infer FPS around 40.7;
- PrepTask FPS around 41.1;
- per-stage averages for cam_grab, PXP, sem_wait, total_prep, invoke.

Required output:

- recompute FPS from counted frames over measured wall time;
- do not trust old `pipeline.calibrate().fps_x100` unless it is migrated to
  `TimerMicros`;
- re-instrument pipeline invoke and prep stages with wall-clock microseconds or
  explicitly label old tick-based fields as non-authoritative.

### Multi-Patch Throughput

Original claims:

- free 1 patch: 44 FPS;
- 30 Hz / 2 patches: 48.5 FPS;
- 20 Hz / 3 patches: 54.8 FPS;
- 15 Hz / 4 patches: 56 FPS.

Required output:

- rerun `invokes_per_frame` and prep-rate matrix with wall-clock FPS;
- separate frame FPS from TPU invoke FPS;
- record failures, missed frames, and any backpressure.

### Dual-Camera Switching And Alternation

Original claims:

- cold switch latency: 11/14/18 ms;
- during pipeline: 5/12/21 ms;
- continuous 1:1 alternation old/new drain rates;
- visual seam validation had 0 mixed-frame artifacts.

Required output:

- switch latency measured with wall-clock microseconds or camera-frame based
  counters;
- alternation throughput measured from frame counts over wall time;
- visual seam validation rerun or marked unchanged if it is content-based and
  independent from TPU timing.

### Reality-To-Detection Latency

Original values are derived from pipeline FPS and per-stage timings.

Required output:

- recompute latency bounds from corrected camera period, corrected output
  period, and corrected stage timings;
- clearly distinguish measured values from derived bounds.

## Acceptance Criteria

A10/B10 are complete when:

- every table in Section 4.2 is classified as `valid`, `corrected`, or
  `removed/deferred`;
- every corrected value links to a specific `s235/iterNNN_*` artifact folder;
- the paper section can be rewritten without relying on DWT-as-wall-time or
  tick-derived TPU invoke timing;
- old `experiment.md` claims that are superseded are annotated or summarized
  in B10 so future readers do not reuse them accidentally.


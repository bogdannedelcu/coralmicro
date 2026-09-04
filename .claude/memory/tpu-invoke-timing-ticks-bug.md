---
name: tpu-invoke-timing-ticks-bug
description: TPU invoke ms was measured with xTaskGetTickCount which under-counts (SysTick starved) — fixed to GPT1 µs; benchmarks before build 1581 are bogus
metadata: 
  node_type: memory
  type: project
  originSessionId: 218d6445-6c73-4c0f-bb51-4d13943aaeb1
---

The `sentai_tpu_invoke*` functions timed the invoke with `xTaskGetTickCount()`.
During a TPU invoke the USB host task (high priority) starves the SysTick ISR, so
the tick counter UNDER-COUNTS: it reported **~6 ms for a real ~35 ms invoke**.
Every invoke/FPS number derived from `sentai.tpu.invoke()`'s return (or the
pipeline's `infer_ms_sum`) BEFORE build 1581 is bogus — e.g. "167 fps" on c2f_thick
RGB, which is physically impossible (921600 B input alone is ~23 ms over USB2-HS
at ~40 MB/s).

**Fixed (build 1581):** all four sites (`sentai_tpu_invoke_internal`,
`_with_input`, `_slot`, `_slot_with_input` in sentai_runtime.cc) now use
`vPortGetRunTimeCounterValue()` (GPT1 hardware µs counter — the same source as
`sentai.rtos.micros()`, NOT ISR-dependent) and return real ms. Verified on board:
`tpu.invoke()` now returns 35 (RGB), matching a GPT1-wrapped measurement.

**Correct invoke benchmark (real µs, isolated, n=60, desc_cache off):**
- RGB c2f_thick (921600 B input): median **35.16 ms → 28.4 fps**.
- YUV420 c2f_thick dual-input (460800 B: UV 153600 + Y 307200): median **21.97 ms
  → 45.5 fps**. => **YUV420 is +60% FPS** (−37.5% invoke), from halving the input
  transfer (the invoke is input-transfer-bound; the ~13 ms saved ≈ the 461 KB
  transfer delta over USB2). Variance ~1% (tight). See [[camera-yuv420-native-
  capture]] + s240 bench_{rgb,yuv}_us.log.

**Rule of thumb:** always time the TPU invoke with `sentai.rtos.micros()` (GPT1) or
wrap it host-side with µs, NEVER FreeRTOS ticks — SysTick is unreliable during the
USB-heavy invoke.

**SECOND SITE of the SAME bug — the InferTask FPS throttle (fixed build 1589).**
`detection_task.cc` InferTask has an `s_target_fps` throttle (default 45, power/
brownout protection) that did `elapsed = xTaskGetTickCount() - t0` and slept
`min_period - elapsed`. Because ticks under-count the invoke, `elapsed` read ~13 ms
for a real 23 ms frame → the throttle injected a spurious ~9 ms vTaskDelay EVERY
frame. Net: the pipeline ran ~30 fps instead of the invoke-bound rate. Confirmed
zero-code by `sentai.pipeline.target_fps(0)` (30→40.5 fps, +35%). Fixed by stamping
`t0_us = vPortGetRunTimeCounterValue()` and computing elapsed in GPT1 µs; the
throttle now adds ZERO delay at 23 ms invoke yet still caps a faster model at 45 fps.
Measured @45 fps camera, throttle fixed: **YUV420 dual-input 40.6 fps** (was 30,
+35%), **RGB c2f_thick 26.2 fps** (was 22.2, +18%) → YUV is +55% over RGB. Both are
now invoke-bound (period ≈ invoke). NOTE for future perf work: profiling proved the
gap was NOT NMS (`sentai_tpu_detect` ~0 ms), NOT prep (infer_wait 0.1 ms → PrepTask
fully hidden, so a deeper/double input buffer would NOT help — the single OCRAM
`.tpu_input` buffer is fine), NOT the camera (VGA45 registers verified, camera.fps()
=45). Always grep for `xTaskGetTickCount` in any code path that spans an invoke.

Implication for the pipeline at 45 fps camera (22 ms frame period): RGB (35 ms
invoke) is invoke-bound → caps ~28 fps; YUV420 (22 ms) ≈ the frame period → can hit
~45 fps. The end-to-end pipeline wiring (Phase 4b: PrepTask stages Y+UV, InferTask
uses sentai_tpu_invoke_with_inputs2) is needed to measure this + the +flow case.

**E2E pipeline result (build 1584-1586, detection-only @45 fps camera, measured via
pipeline.frame_count over 15-28 s, real ms):**
- RGB c2f_thick single-input: **22.2 fps** sustained (invoke-bound, last_rc=36 ms).
- YUV420 dual-input, first wiring (release PrepTask only after full invoke, no
  overlap): **29.3 fps**.
- YUV420 after the **overlap refinement** (byte-target release sema, see below):
  **30-33 fps** (n=4 runs: 29.3/30.0/30.3/32.7; last_rc=22 ms). +32-48% over RGB.
- Sustained FPS < the 45 fps invoke-ceiling because prep (PXP xyuv→yuv420) + camera
  grab don't fully hide under the invoke; the dual-input sync recovers most but not
  all of the single-input overlap.

**Overlap refinement (the "release after LAST plane" fix):** the dual-input invoke
issues 2 SendInputs — Y (307200 B) then UV (153600 B), both in the same OCRAM `buf`.
The legacy one-shot input-done sema fires on the FIRST SendInputs (frees buf while UV
still transferring). Added `sentai_tpu_set_input_done_sema2(sema, byte_target)` in
edgetpu_driver.cc: accumulates SendInputs bytes, fires the PrepTask-release sema only
when >= byte_target (460800 = Y+UV). detection_task.cc dual branch arms it so PrepTask
overlaps the compute+output tail like the single-input path. Verified 2 SendInputs/
invoke, no param-caching double-send, via `sentai.tpu.trace(1)`.

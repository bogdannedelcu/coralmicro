# Camera-switch latency — E15 vs E16 on the SentAI dual-sensor MUX

## Abstract

The SentAI board carries two OV5640-derived camera modules multiplexed onto
a single MIPI-CSI lane via a GPIO-controlled analogue mux.  Selecting a
camera is effectively free at the MUX level (a GPIO register write that
returns in micro-seconds), but the first frame grabbed *after* a switch is
not: the sensor driver has to drain frames already queued from the previous
sensor and then wait for at least two fresh ISR frames from the newly
selected one before `to_tensor()` is allowed to return.  This document
quantifies that cost by running two back-to-back experiments on identical
firmware: **E15**, the parallel 15 FPS pipeline pinned to `cam0`, and
**E16**, a sequential loop that alternates `cam0 ↔ cam1` every single
frame.  The comparison measures the per-switch overhead at **≈ 147 ms**,
with a stable **65 ms asymmetry** in favour of switching *to* `cam0`.  The
result sets a concrete upper bound on any dual-camera scheme that relies
on frame-level MUX toggling.

## Test conditions

### Hardware

| Component | Value |
|-----------|-------|
| MCU | NXP i.MX RT1176 (Cortex-M7 @ 800 MHz + Cortex-M4) |
| On-chip EdgeTPU | Coral/Google TPU, internal USB2 bus |
| External RAM | 16 MB SDR-SDRAM on SEMC (166 MHz) |
| Cameras | 2× OV5640-based coralmicro modules, 1280×720 native, 15 FPS streaming |
| Camera mux | GPIO-controlled analogue MUX on shared MIPI-CSI lane |
| USB to host | CDC-ACM (REPL) + CDC-NCM (IP 10.0.0.1) |

### Firmware

| Parameter | Value |
|-----------|-------|
| Build | `sentai_runtime` build #624, eDMA memcpy enabled (see [memcpy.md](memcpy.md)) |
| Verbose | `sentai.verbose(0)` for the whole loop body in both runs |
| Session | `/diags/s034_e15_vs_e16_x40/` (persisted on device LittleFS) |

### Model under test

Identical to [memcpy.md](memcpy.md):

| Parameter | Value |
|-----------|-------|
| File | `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite` |
| Input | `uint8[1, 512, 512, 3]` — 786 432 B |
| Output | `uint8[1, 1344, 6]` — YOLOv5-enhanced, 1-class head |
| Resolution fed to model | 512×512 (post-PXP resize from 1280×720) |

### Scene

- Static scene throughout both runs — same room, same framing, no moving
  targets.  `num_detections == 0` on every frame, so NMS still runs its
  full candidate scan (1344 anchors) but the sort/IoU stages exit early
  — a constant background cost that cancels in the E15 vs E16 comparison.
- Before-and-after snapshots are saved from *both* cameras by the shared
  helper `diag.snapshot_both_cameras(when)` in
  [diag/_session.py](../diag/_session.py):
  ```
  /diags/s034_e15_vs_e16_x40/scene_cam0_before_512x512.jpg   (23 611 B)
  /diags/s034_e15_vs_e16_x40/scene_cam0_after_512x512.jpg    (23 540 B)
  /diags/s034_e15_vs_e16_x40/scene_cam1_before_512x512.jpg   (29 301 B)
  /diags/s034_e15_vs_e16_x40/scene_cam1_after_512x512.jpg    (29 692 B)
  ```

### Methodology

- **Back-to-back in the same firmware image, same session**: E15 runs
  first, then one `gc.collect()`, then E16.  No reflash, no reboot, no
  camera restart between them.  Scene, thermal state, TPU cache and
  model pointer are all preserved — removing every confound that could
  come from firmware drift.
- **40 measurement frames per experiment.**  E15 warmup is handled
  internally by the parallel pipeline (one drained `pipeline.get()`
  before timing); E16 does one full warm cycle on each camera before the
  measurement loop begins.  The first measurement sample is dropped in
  both cases before computing statistics.
- **Different measurement primitives by design**:
  - E15 reports `frame_interval_ms` = wall-clock delta between
    consecutive `sentai.pipeline.get()` returns, i.e. the steady-state
    interval between emitted frames.  That is the right number for a
    parallel pipeline whose internal stages overlap.
  - E16 reports `total_frame_ms` = sum of per-stage waits inside a
    sequential `select → to_tensor → invoke → detect` loop.  In a
    sequential run this also equals the wall-clock per-frame interval.
  Both numbers are therefore directly comparable as "how many ms until
  the host sees the next detection result".
- Benchmark driver: `_e15_vs_e16.py` invoked via `repl_run.py`.
- Experiment code:
  [diag/e_pipeline.py:e15_pipeline_parallel_512](../diag/e_pipeline.py)
  (parallel, fixed camera) and
  [diag/e_pipeline.py:e16_camera_switch_512](../diag/e_pipeline.py)
  (sequential, alternating cameras).

## Experiment E15 — fixed camera, parallel pipeline (baseline)

`diag.e15_pipeline_parallel_512(repetitions=40)`.  Locks the camera to
`cam0` for the whole run and uses the firmware pipeline (`PrepTask` +
`InferTask`) so camera-capture + PXP-resize + quant of frame `N+1` run in
parallel with TPU `Invoke` of frame `N`.  Wall interval between emitted
frames is then `max(prep_stage, infer_stage)`, not their sum.

### Per-frame intervals (40 samples, first two dropped)

```
frame_interval_ms (ms, 38 samples after warm-up)
min     61
max     68
mean    64.5
stdev    1.8
FPS    15.5
```

All 38 intervals fell in the range **61…68 ms**, σ ≈ 1.8 ms.  At this
resolution and model, `prep_stage` dominates the pipeline — the CSV's
`infer_stall_ms` column is non-zero every frame (~41 ms), meaning the
infer task waits ~41 ms for prep to hand off the next frame.  Nothing
in the measurement path is close to saturation of either USB endpoint.

## Experiment E16 — alternating `cam0 ↔ cam1`, sequential pipeline

`diag.e16_camera_switch_512(cam_a=0, cam_b=1, repetitions=40)`.  The loop
body is deliberately sequential:

```python
for i in range(repetitions):
    cam = cam_a if (i % 2 == 0) else cam_b
    t0 = _ticks(); sentai.camera.select(cam);   sel  = _ticks() - t0
    t0 = _ticks(); sentai.camera.to_tensor();   tens = _ticks() - t0
    inv = sentai.tpu.invoke()
    t0 = _ticks(); dets = sentai.tpu.detect(...); det = _ticks() - t0
```

The parallel `sentai.pipeline` is *not* started because `PrepTask` owns the
camera MUX during its inner `cam_grab_latest()` loop — a mid-pipeline
`camera.select()` would race with the active grab and the measurement
would be undefined.  E16 is therefore a per-call-graph decomposition of a
switch-every-frame scheme, which is exactly what this experiment is
designed to bound.

### Aggregate results (40 samples, first one dropped)

```
total_frame_ms (ms, 39 samples)
min      176
max      250
mean    211.4
stdev    32.5
FPS      4.7
```

The σ is an order of magnitude larger than E15 because the loop is
**bimodal**: even frames (going to `cam0`) and odd frames (going to
`cam1`) have different post-switch drain cost.  Splitting by the camera
that produced each measured frame makes the distribution clearly
unimodal again:

| direction | frames | total (ms) | to_tensor (ms) | invoke (ms) | detect (ms) | select (ms) |
|---|---:|---:|---:|---:|---:|---:|
| → `cam0` (after `cam1`) | 19 | **178.5 ± 1.5** | 149.4 ± 0.9 | 29 ± 2 | 0.5 ± 0.5 | 0 |
| → `cam1` (after `cam0`) | 20 | **242.7 ± 2.1** | 214.7 ± 1.0 | 28 ± 2 | 0.5 ± 0.5 | 0 |
| **asymmetry** (cam1 − cam0) | — | **+64.2 ms** | +65.3 ms | ≈ 0 | ≈ 0 | ≈ 0 |

Within each camera σ ≈ 1–2 ms, so every ms of the switch cost is
reproducible, not jitter.

## E15 vs E16 — head-to-head

| metric | E15 (fixed `cam0`, parallel) | E16 (alternating, sequential) | Δ |
|---|---:|---:|---:|
| frames (measured) | 38 | 39 | — |
| wall frame time — mean ± σ | **64.5 ± 1.8 ms** | **211.4 ± 32.5 ms** | **+146.9 ms** |
| min / max | 61 / 68 | 176 / 250 | — |
| effective FPS | **15.5** | **4.7** | −10.8 (−70 %) |
| overhead per switched frame | — | ≈ 147 ms | — |

The switch-every-frame scheme collapses the pipeline from the sensor-rate
ceiling (15 FPS) to **4.7 FPS**, a 3.3× slowdown, and does so
deterministically — a 5-second burst of alternation loses about **54
inferences** relative to fixed-camera operation.

## Interpretation

### The MUX flip itself is effectively free

`sentai.camera.select(id)` measured **0 ms** every time.  Internally it is
`CameraTask::SwitchCamera(id)` which performs a single `GPIO kCamMux = id`
store plus `g_cam_switch_pending = true` — documented in
[camera.md](camera.md) §2.  No CSI reinitialisation, no PLL re-lock, no
register-list write to the sensor.  The whole cost of "switching cameras"
lives in the *next* frame grab.

### All overhead is absorbed by `to_tensor()`

E16's per-stage split is unambiguous:

- `select` = 0 ms
- `invoke` ≈ 28–30 ms (identical across both cameras — same model,
  same EdgeTPU path)
- `detect` ≈ 0.5 ms (same NMS, zero-candidate exit)
- `to_tensor` takes the rest: **149 ms to `cam0`**, **215 ms to `cam1`**

`to_tensor()` calls `sentai_cam_get_raw_with_recovery()` which, with
`g_cam_switch_pending` set, enters the slow path:

1. **Drain queued stale frames** — the CSI DMA may already have written
   one or two buffers with pixels from the *previous* sensor before the
   MUX flip took effect.  The driver walks `g_camera_frame_seq` forward
   and discards those.
2. **Wait for ≥ 2 fresh frames** — at 15 FPS (~67 ms/frame), two fresh
   frames is 133 ms minimum, which is the floor consistent with our
   `cam0` measurement (149 ms) and within one frame of the `cam1`
   measurement (215 ms).  The extra time is ISR-path overhead plus the
   PXP resize + quant the real `to_tensor` does on the returned frame.

This confirms the design note in [camera.md](camera.md) line 124:
*"drain stale frames (~134ms), wait for ≥ 2 frames from the new
camera"* — our steady-state measurements land on exactly that figure for
one direction and one extra frame interval for the other.

### The 65 ms asymmetry is structural, not noise

With σ ≈ 1 ms on each camera group, the 65 ms gap between `→ cam0` and
`→ cam1` is not jitter.  It reads as **one extra frame interval at
15 FPS (≈ 66 ms)**, i.e. `cam1` needs one more fresh ISR frame than
`cam0` before the driver releases a clean buffer.  Candidate causes:

- **Different rotation policies** — [camera.md §2](camera.md) documents
  `cam0` rotated 180° and `cam1` rotated 0°.  Rotation is applied
  through the OV5640 `MIRROR H/V` registers at sensor-init time, so it
  does not re-run per frame.  Unlikely to be the cause.
- **Per-sensor stream resume latency** — the MIPI-CSI lane is shared;
  whichever sensor is deselected keeps streaming into a buffer that
  the receiver ignores, but stream-resume on the newly selected sensor
  may differ between the two OV5640 instances due to board-layout
  differences or minor sensor-register configuration.
- **Buffer-queue state at the moment of switch** — if `cam1` tends to
  have one more in-flight buffer than `cam0` when the MUX flips, the
  drain walks one extra queue entry.

The 65 ms is small enough to be a fixed property of one of the above
rather than a real variable cost; a focused follow-up could pin it down
by logging `g_camera_frame_seq` across the switch and counting dropped
buffers.

### Why E15 wall time is comparable to E16's per-stage invoke alone

E15 reports 64.5 ms wall.  E16 reports 29 ms of `invoke`.  The rest of
E15's budget is the *parallel* `prep_stage` — which includes the exact
same PXP resize + quant + camera-grab work E16 does sequentially, but
hidden behind `invoke` because it runs in a separate FreeRTOS task with
its own staging buffer.  Without the switch, that hiding works; with a
switch every frame, the next frame's `prep_stage` cannot start until the
camera has stabilised, which is precisely the 147 ms overhead
documented here.

## Fix A — race-free snapshot (landed, null timing result)

The first optimisation we tried pins down the sequence-number snapshot
that the drain logic uses to detect "≥ 2 fresh frames from the new
camera".  Before the fix, `g_cam_switch_seq = g_camera_frame_seq` ran in
`sentai_cam_switch` (`sentai_runtime.cc`) *before* `cam->SwitchCamera()`
dispatched the MUX-flip request through the `CameraTask` queue.  The
handler only flips the GPIO `Δq ≈ 1-10 ms` later — a window during which
the CSI ISR can tick `g_camera_frame_seq` for a frame whose DMA was
already in-flight with old-camera pixels.  That increment then counts
toward the delta threshold, so the caller can short-circuit the drain
and observe an old-camera frame.

The fix moves the snapshot into `CameraTask::HandleSwitchCameraRequest`
(`libs/camera/camera.cc`), one instruction before the `GpioSet()`:

```cpp
case coralmicro::SwitchCameraId::kCameraBack:
    g_cam_switch_seq = g_camera_frame_seq;
    coralmicro::GpioSet((coralmicro::Gpio) Gpio::kCamMux, MUX_BACK_CAMERA);
    break;
```

`cam->SwitchCamera()` uses the blocking `SendRequest` path
(`libs/base/queue_task.h:48`) with a binary-semaphore callback invoked at
`camera.cc:1189`, so by the time the wrapper returns the handler has
already run and the snapshot-flip pair is visible.  The remaining race
window is 1–2 instructions (~10 ns at 800 MHz) — bounded, and the
worst-case miscount is at most one frame in a direction that is already
safe (both statements ran).

### Measured impact

Same scene, same firmware except for the patch, 40 frames per
experiment, sessions `s034` (before) vs `s035` (after):

| direction | before fix (s034) | after fix (s035) | Δ |
|---|---:|---:|---:|
| E15 wall | 64.5 ± 1.8 ms | 64.7 ± 2.3 ms | +0.2 ms |
| E16 wall (total) | 211.4 ± 32.5 ms | 212.4 ± 33.5 ms | +1.0 ms |
| → `cam0` total | 178.5 ± 1.5 | 178.6 ± 2.2 | +0.1 ms |
| → `cam1` total | 242.7 ± 2.1 | 244.6 ± 2.3 | +1.9 ms |
| asymmetry (cam1 − cam0) | +64.2 ms | +66.0 ms | noise |

**The fix is a no-op at this resolution.**  That is the honest result:
every number is inside the run-to-run noise band.

### What we conclude from the null result

The queue-latency race is *real* — sending a request through a FreeRTOS
queue and then blocking on a semaphore is a non-trivial interval — but
empirically `Δq ≈ 0` on this board when the camera task is otherwise
idle, so the old code was already landing its snapshot within the
correct frame.  The patch makes the invariant explicit and kills a race
that could matter on a loaded queue, which is worth keeping (it is a
correctness fix, not a performance fix), but the **65 ms cam1-vs-cam0
asymmetry is not caused by the snapshot timing**.  It has to live in the
frame-cadence structure itself: the direction-specific phase at which
the MUX flip lands inside the 67 ms DMA-buffer cycle determines whether
the drain threshold `>= 2` is reached after two or three full frame
intervals.  Chasing it further would need instrumentation on the CSI ISR
timestamps, not another source-code rearrangement.

## Conclusions

1. **Per-switch cost is bounded and dominated by sensor stream stability,
   not by firmware overhead.**  The GPIO MUX flip + stale-frame drain +
   fresh-frame wait accounts for ~133–215 ms depending on direction, and
   the remaining 1–2 ms is at the noise floor of `_ticks()`.  There is
   no obvious further optimisation on the firmware side without
   redesigning the CSI stream machinery.
2. **Switching every frame is not viable for dual-camera ML at 15 FPS.**
   A switch-per-frame scheme caps throughput at ~4.7 FPS.  Any dual-camera
   application has to either:
   - Switch on a multi-frame cadence (e.g. 30 consecutive frames per
     camera amortises the switch cost to ~5 ms/frame and preserves
     close-to-steady-state FPS), or
   - Run a software tracker on one sensor and only switch on a
     trigger event, or
   - Accept the 4.7 FPS ceiling as the operating point for a
     stereo-style alternating scheme.
3. **The asymmetry matters.**  Any scheduling policy that treats both
   cameras as equivalent will underestimate the cost of switching to
   `cam1` by ~65 ms (≈ 40 %).  Alternating schedules should either be
   measured on the actual direction pair they will use or be designed
   to pay the worse case on every switch.
4. **The `camera.select` / `to_tensor` decomposition is trustworthy.**
   `select` reported 0 ms across 39 frames.  `to_tensor` produced σ ≈ 1 ms
   within each camera group.  The measurement method is good enough for
   further experiments — e.g. a variant that prefetches a frame before
   the switch, or that runs `invoke` on the old camera's frame while
   the next camera is stabilising.

## Reproducing the measurement

```bash
# Push latest diag/ to the board (REPL-based chunked fs.write — see
# project_upload_diag_repl memory entry for why HTTP upload is not used).
# The uploader lives inside the package it manages but runs on Linux.
python3 diag/_host_upload_repl.py --file e_pipeline.py --file __init__.py \
                                  --file _session.py

# Run both experiments in a single session on the device.
python3 repl_run.py --timeout 180 --script _e15_vs_e16.py

# Pull the CSVs off the board over HTTP GET (reads are reliable — only
# writes hang on this firmware).
curl -s http://10.0.0.1/api/raw/diags/s034_e15_vs_e16_x40/001_e15_pipeline_par_cam0_512x512.csv
curl -s http://10.0.0.1/api/raw/diags/s034_e15_vs_e16_x40/002_e16_camswitch_cam0_cam1.csv
```

Raw CSVs, scene snapshots from both cameras (before + after), and
per-experiment description `.txt` files are persisted in the session
folder on the device and survive reboot.

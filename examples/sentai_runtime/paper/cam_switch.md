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

## Fix B — flip-on-EOF + 30 fps + stateless ratio scheduler (landed)

Fix A was a correctness patch with a null timing result; the 65 ms asymmetry
and the ~140 ms switch floor were still there, and E17 at `switch_drain=1`
produced visibly corrupt frames with a horizontal seam halfway down the
image (one half from `cam0`, one half from `cam1`).  That confirmed the
root cause was **not** a snapshot-timing race but the MUX flip physically
landing in the middle of an active DMA buffer fill.  Fix B addresses that
directly.

### Change 1 — 30 fps sensor mode (`DEMO_CAMERA_FRAME_RATE` = 30)

The NXP CSI driver's `csi2rxHsSettle` lookup
([camera_support.c:197–206](../../../libs/camera/camera_support.c#L197))
has a native entry for 720p @ 30 fps with `tHsSettle = 0x12`; OV5640's
720p subsample mode ceiling is 45 fps per the datasheet, so 30 fps is
well within spec.  One-line change in
[camera_support.h](../../../libs/camera/camera_support.h) halves the
frame period from 67 ms to 33 ms.  Every timing threshold that is
expressed in "fresh frames" (the drain, `wait_iters` in
`sentai_cam_get_raw_with_recovery`) now costs proportionally less wall
time, and the mid-buffer MUX-flip artifact window from E17 thr=1 is
halved in duration.

### Change 2 — flip-on-EOF in the CSI ISR (the actual tearing fix)

The old code called `cam->SwitchCamera()` from task context, which
dispatched to `HandleSwitchCameraRequest` and performed the GPIO flip
**whenever that handler happened to run**.  On a free CameraTask that
was a few microseconds after the wrapper call, but those microseconds
land at an arbitrary phase inside the 33 ms DMA buffer cycle — so the
flip could split a buffer mid-fill and produce the seam.

Fix B changes the protocol: `sentai_cam_switch(id)` **arms**
`g_cam_pending_mux_id`; the CSI end-of-frame ISR
([camera_support.c:CSI_IRQHandler](../../../libs/camera/camera_support.c))
consumes that arm in the same instruction stream as the frame-seq
increment, which is the exact moment the DMA has just finished filling
a buffer and the MIPI lane is idle until the next SOF — i.e. VBLANK.
Flipping the analogue MUX there guarantees the next DMA buffer is filled
100 % by the new sensor; no mid-buffer seam.

The ISR respects NASA/JPL §C ("shortest possible ISR work"): one
volatile read, one branch, one atomic GPIO write via the dedicated
`DR_SET`/`DR_CLEAR` shadow registers
([GpioSetFromIsr](../../../libs/base/gpio.cc) — no mutex taken in ISR
context), four global stores.  No loops.  No task wake-up.  No queue
enqueue.  The task-side wrapper falls back to the legacy synchronous
`cam->SwitchCamera()` path after a bounded 150 ms wait for the ISR to
consume the arm; this preserves operation if the CSI is stuck and the
operator sees a `[cam_switch] fallback sync` log line flagging the
degraded path.

### Change 3 — stateless `ratio(a, b)` scheduler in the same ISR

Exposed as `sentai.camera.ratio(a, b)`.  Both-zero disables the
scheduler; otherwise, for each completed frame, the ISR computes
`seq % (a + b)`: if the result is `< a` the target is `cam0`, else
`cam1`.  If the target differs from the current MUX position, it arms
`g_cam_pending_mux_id` which the very same ISR consumes on the next
instruction.  The whole scheduler is four reads, one modulo (1 `UDIV`
≤ 12 cycles on Cortex-M7), two compares, one conditional store — no
mutable counter state, no per-camera history.  This gives asymmetric
capture rates (e.g. `(3, 1)` → `cam0` at 22.5 fps, `cam1` at 7.5 fps on
a 30 fps sensor) with the same glitch-free VBLANK flip guarantee.

### Measured / verified impact

- Per-select cost dropped from the old sync-dispatch ~26 ms to **~26 ms
  under the new path** (bounded wait for the next EOF; 1 frame at 30 fps
  = 33 ms ceiling).  One debug-log example:
  `[cam_switch] -> cam1 (26ms, seq=33, via EOF ISR)`.
- **E17 at `switch_drain(1)` on flip-on-EOF firmware: visually
  indistinguishable from `switch_drain(2)`.**  User-inspected all 16
  frames per threshold in `/diags/s041_e17_eof_check/e17_t{1,2}_frames/`
  — "arată identic, nu se văd artefacte" (the frames are correct, no
  mixing between sensors).  Seam artifact from the old
  `s038_e17_drain_ab/e17_t1_frames/` is gone.
- `switch_drain` is kept at **default 2** as a belt-and-suspenders
  conservatism.  With flip-on-EOF, threshold 1 is safe and threshold 2
  costs at most one extra frame interval (~33 ms at 30 fps) — the
  latency savings from dropping to 1 are marginal compared to the risk
  of a future regression re-introducing a seam, so the default stays
  defensive.  Users opt in explicitly via `sentai.camera.switch_drain(1)`
  when they need the extra frame.

### Head-to-tail timing on Fix B (session `s042_e16_eof_30fps_x40`)

Same E16 loop, same scene, same 1-class 512×512 model as the earlier
sessions, re-run after the firmware edits above.  `sentai.camera.ratio(0,0)`
disables the auto-alternate scheduler so each iteration is exactly one
manual `select()` followed by the four-stage sequential pipeline.
`switch_drain(2)` preserved as the conservative default.  40 reps,
first dropped as warm-up.

| Stage | Mean ± σ | min / max | n |
|---|---:|---:|---:|
| `select()` (arm + EOF ISR consume) | **17.7 ± 0.6 ms** | 17 / 19 | 39 |
| `to_tensor()` (drain + PXP + quant) | **96.0 ± 0.7 ms** | 95 / 97 | 39 |
| `invoke()` (EdgeTPU) | 31.2 ± 2.4 ms | 28 / 36 | 39 |
| `detect()` (NMS) | 0.6 ± 0.5 ms | 0 / 1 | 39 |
| **total frame** | **145.6 ± 2.5 ms** | 141 / 150 | 39 |
| **effective FPS (switch every frame)** | **6.87** | — | — |

Per-direction split — the 65 ms structural asymmetry that Fix A could
not touch is now **within noise**:

| direction | total | select | to_tensor |
|---|---:|---:|---:|
| → `cam0` (after `cam1`) | 146.1 ± 2.6 ms | 17.7 ± 0.6 | 96.5 ± 0.5 |
| → `cam1` (after `cam0`) | 145.1 ± 2.4 ms | 17.8 ± 0.6 | 95.5 ± 0.5 |
| asymmetry (cam1 − cam0) | **−1.1 ms** | +0.1 ms | −1.0 ms |

That asymmetry collapse is the direct consequence of flipping in
VBLANK: the direction-specific phase of the MUX flip inside a DMA
buffer cycle no longer matters because the flip never lands inside
one.

### Cross-session comparison — all three fixes

Same experiment, same scene, same model, different firmware builds.
All numbers are from stored CSV manifests:

| Build | FPS (switch every frame) | Total frame | Asymmetry (cam1 − cam0) | Seam artifacts at `drain=1` |
|---|---:|---:|---:|---|
| Pre-Fix A (15 fps, 2-frame drain) — `s034` | 4.7 | 211.4 ± 32.5 ms | **+64.2 ms** | half-and-half frames, visible seam |
| Fix A (15 fps, atomic snapshot) — `s035` | 4.7 | 212.4 ± 33.5 ms | **+66.0 ms** | unchanged — seam still present |
| **Fix B (30 fps, flip-on-EOF, drain=2)** — `s044` (final) | **6.86** | **145.7 ms** | **+0.1 ms** | **visually identical to drain=2** (user-confirmed) |

Net effect vs baseline: **1.46× speed-up** on total frame time, **1.46× FPS**,
full elimination of the directional asymmetry, full elimination of the
mid-buffer seam at `drain=1` (verified on `/diags/s041_e17_eof_check/
e17_t1_frames/` vs the old `s038_e17_drain_ab/e17_t1_frames/`).

### What is still open

The modulo scheduler introduces `(a+b)`-frame quantisation, so the
effective per-camera rate is only what the ratio rounds to.  For finer
control the user can do their own rate policy from Python around
manual `select()` calls.  The residual ~145 ms per-switch cost is now
evenly split between `select` (~18 ms waiting for the EOF arm to be
consumed — one frame interval at 30 fps) and `to_tensor` (~96 ms
= drain + PXP + quant — dominated by the `switch_drain=2` wait for two
fresh frames).  Dropping to `switch_drain(1)` on Fix B is now visually
safe and would shave about one frame interval (33 ms) off `to_tensor`,
bringing total to ≈ 115 ms and FPS to ≈ 8.7 — the reason it is not
the default is the belt-and-suspenders conservatism documented above.

## Head-to-tail benchmark — Experiment E18

Where E16 measured only the alternating scheme, E18 runs **three
back-to-back sweeps in one session** at identical firmware, scene,
model and thermal state:

- **A** — fixed `cam_a`, sequential loop, no switches
- **B** — fixed `cam_b`, sequential loop, no switches
- **C** — alternating `cam_a ↔ cam_b`, switch every frame

This makes the per-switch overhead quantifiable as a pure subtraction:
`overhead = C_total − max(A_total, B_total)`, with everything else
held constant.  The sweep is the same four-stage per-iteration pipeline
used in E13/E16: `select → to_tensor → invoke → detect`.

### Session `s044_e18_headtail_drain2` — final run, 30 fps, flip-on-EOF, drain=2, 40 reps per sweep

Results reproduced in two back-to-back sessions (`s043`, `s044`) under identical
conditions; numbers below are from the final run and differ from `s043` only
by run-to-run jitter (≤ 1 ms on every stage).

| Sweep | `select` | `to_tensor` | `invoke` | `detect` | **total** | **FPS** |
|---|---:|---:|---:|---:|---:|---:|
| A — fixed `cam0` | 0.0 | 31.7 | 30.5 | 0.4 | **62.6 ms** | **15.97** |
| B — fixed `cam1` | 0.0 | 31.7 | 30.0 | 0.3 | **62.1 ms** | **16.12** |
| C — alternating | 18.0 | 95.9 | 31.4 | 0.5 | **145.7 ms** | **6.86** |

Per-direction inside the alternating sweep:

| direction | n | total (mean, ms) |
|---|---:|---:|
| → `cam0` | 19 | 145.6 |
| → `cam1` | 20 | 145.7 |
| **asymmetry (cam1 − cam0)** | — | **+0.1 ms** |

The 65 ms directional asymmetry from Fix 0 and Fix A is now inside the
noise band.  Each camera contributes the same cost because the MUX flip
always lands in VBLANK regardless of the direction — no direction ever
"straddles" an active DMA buffer fill.

### Budget decomposition of the 83 ms per-switch overhead

| where the 83 ms comes from | amount | explanation |
|---|---:|---|
| `select` wait for EOF ISR consumption | **+18 ms** | one arm-to-consume round trip ≈ 0.5 frame interval at 30 fps |
| `to_tensor` drain + 2 fresh frames | **+64 ms** | post-switch drain of stale queued buffers, then `switch_drain=2` wait for two fresh frames (minimum 2 × 33 ms = 66 ms) |
| `invoke` (TPU) | 0 | sensor-independent; identical in A, B, C |
| `detect` (NMS) | 0 | same model output, same zero-candidate exit |

Switching *to* a given camera pays a fixed ~83 ms tax.  At
`switch_drain(1)` that tax drops by one frame interval to ≈ 50 ms
(visually verified glitch-free in E17 § s041), which would take the
alternating throughput from 6.86 FPS to ≈ 8.9 FPS.  That
`switch_drain(1)` setting is available as an opt-in knob; it is not
enabled by default because the 33 ms saving is marginal against the
cost of a surprise seam if any future regression removes the
flip-on-EOF guarantee.

### What E18 tells us about application design

- A **fixed-camera** application on this firmware hits **16 FPS** — the
  sensor rate — cleanly, with zero cycles left on the table for
  scheduling overhead.  `invoke` and `to_tensor` each consume ~31 ms,
  summing to almost exactly one 33 ms sensor frame.  This is the hard
  ceiling until the model shrinks or the TPU pipeline changes.
- Any alternation schedule pays a per-switch tax that is roughly
  **one frame for `select` + (drain) frames for `to_tensor`**.  At
  `switch_drain=2` that is `(0.5 + 2) ≈ 2.5` frame intervals of
  overhead per switch — almost exactly what E18 measured (82.5 ms /
  33 ms ≈ 2.5).  The arithmetic predicts the measurement.
- For asymmetric capture (the `sentai.camera.ratio(a, b)` scheduler),
  an `(n, 1)` schedule amortises the tax over `n` cam0 frames and
  `1` cam1 frame: effective FPS ≈ `1000 / ((n × 62.5 + 145.3) / (n+1))`.
  For `(9, 1)`: ~12.7 FPS average with 11.4 FPS on cam0 and 1.3 FPS on
  cam1.  For `(3, 1)`: ~9.9 FPS with 7.4/2.5.  Use E18 numbers to pick
  the ratio that matches the application's cam1 liveness budget.

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

## Final summary

| Question | Answer |
|---|---|
| What was the root cause of the E17 `drain=1` seam? | MUX flip was happening in task context (`cam->SwitchCamera`), landing at an arbitrary phase inside an active DMA buffer fill. Half the buffer was from the old sensor, half from the new. |
| What fix was actually needed? | Move the GPIO flip into the CSI EOF ISR so the analogue MUX transitions during VBLANK, before any new DMA buffer begins filling. One short ISR branch per frame, per NASA/JPL §C. |
| What was gained? | `drain=1` is now visually clean (user-confirmed), total per-switch overhead dropped from ~211 ms to 145.7 ms (1.46×), directional asymmetry (cam1 vs cam0) collapsed from +65 ms to +0.1 ms. |
| What is the final sustained FPS? | **Fixed camera: 16.0 FPS** (exact sensor rate). **Switch every frame: 6.86 FPS** (per-switch tax = 83 ms). |
| What can the application layer do about the 83 ms tax? | (1) Use `sentai.camera.ratio(a, b)` to amortise across multiple frames — e.g. `(9, 1)` ≈ 12.7 FPS average. (2) Drop to `switch_drain(1)` for ~8.9 FPS alternating (safe on Fix B firmware). (3) Drop native resolution to VGA/QVGA when per-frame ms budget matters more than detail. |
| What is still unresolved? | 45 fps and 60 fps native sensor modes at 720p.  45 fps is not a 720p mode in the OV5640 datasheet; 60 fps (2×2 binning) was probed with a custom PLL entry but CSI2RX did not lock, indicating additional sensor-register programming beyond the NXP driver's current init sequence would be required.  30 fps remains the ceiling on this board without that driver-level work. |

Shipped runtime surface introduced by this work:

- `sentai.camera.switch_drain([n])` — read/write drain threshold in [1,10], default 2.
- `sentai.camera.ratio([a, b])` — stateless `seq % (a+b)` auto-alternate scheduler; both zero disables.
- `sentai.camera.set_resolution(w, h)` + `sentai.camera.init(1)` — retuned at runtime; confirmed working for 720p / VGA / QVGA (same resolution for both cameras, shared CSI-2 receiver).

Firmware building blocks:

- `libs/base/gpio.cc:GpioSetFromIsr` / `SentaiCamMuxSetFromIsr` — atomic `DR_SET`/`DR_CLEAR` path, no mutex, ISR-safe.
- `libs/camera/camera_support.c:CSI_IRQHandler` — minimal ISR body: `seq++`, one modulo for the ratio scheduler, one conditional GPIO flip consume.
- `examples/sentai_runtime/sentai_runtime.cc:sentai_cam_switch` — arms `g_cam_pending_mux_id`, bounded 150 ms wait for the ISR to consume the arm, synchronous legacy fallback with explicit log line.

Experiments written for this study:

- **E15 parallel baseline** (`e15_pipeline_parallel_512`) — fixed camera, 15-FPS pipeline
- **E16 alternating** (`e16_camera_switch_512`) — cam0 ↔ cam1 every frame, single timing
- **E17 drain visual** (`e17_switch_drain_visual`) — in-RAM JPEG capture at configurable `switch_drain`, LFS write deferred to post-measurement phase to keep hot-loop timing clean
- **E18 head-to-tail** (`e18_camera_switch_headtail`) — three-sweep A/B/C benchmark in one session for a defensible per-switch overhead number

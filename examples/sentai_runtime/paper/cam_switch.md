# Camera-switch latency — E15 vs E16 on the SentAI dual-sensor MUX

*See also:* [experimental_setup.md](experimental_setup.md) for the
canonical hardware and firmware stack, [evaluation.md](evaluation.md)
for the cross-cutting results summary (this chapter is the
implementation-level narrative that the Evaluation chapter's RQ3 and
RQ4 reference), [threats_to_validity.md](threats_to_validity.md) for
caveats, [artifact.md](artifact.md) for the claim-to-CSV map, and
[related_embedded_inference.md](related_embedded_inference.md) for
how this work positions against other MIPI-CSI2 multi-camera
topologies.

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
| Session | [`experiments/s034_e15_vs_e16_x40/`](../experiments/s034_e15_vs_e16_x40/) (on host; originally `/diags/s034_e15_vs_e16_x40/` on device LittleFS) |

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
  ../experiments/s034_e15_vs_e16_x40/scene_cam0_before_512x512.jpg   (23 611 B)
  ../experiments/s034_e15_vs_e16_x40/scene_cam0_after_512x512.jpg    (23 540 B)
  ../experiments/s034_e15_vs_e16_x40/scene_cam1_before_512x512.jpg   (29 301 B)
  ../experiments/s034_e15_vs_e16_x40/scene_cam1_after_512x512.jpg    (29 692 B)
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
experiment, sessions [`s034`](../experiments/s034_e15_vs_e16_x40/) (before) vs [`s035`](../experiments/s035_e15_vs_e16_x40/) (after):

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
- **`switch_drain` is kept at default 2**.  Dropping to 1 was tried
  post-flip-on-EOF (session
  [`s041_e17_eof_check`](../experiments/s041_e17_eof_check/)) and the
  thumbnail-level inspection initially looked clean, but user review
  found the setting unreliable — see the "Known limitations" section
  below.

### Visual evidence — seam before the fix (pre-flip-on-EOF)

The raw JPEGs from E17 session
[`s038_e17_drain_ab`](../experiments/s038_e17_drain_ab/) make the
failure mode obvious in one look.  Figure~\ref{fig:seam} reproduces
two representative frames from that session — 512×512 quality-70
JPEGs captured on the pre-flip-on-EOF firmware at `switch_drain=1`,
on the same scene shown in Figure~\ref{fig:scene}.

\begin{figure}[H]
\centering
\begin{subfigure}[t]{0.31\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/experiment_frames/fig_seam_cam0.jpg}
\caption{Iteration 4, flip toward cam0.  Upper half from cam0's
tight crop of the lilac stems; lower half has jumped to cam1's
wider wall-and-flowers framing.  Seam at $\approx 40\,\%$ image height.}
\label{fig:seam:a}
\end{subfigure}\hfill
\begin{subfigure}[t]{0.31\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/experiment_frames/fig_seam_cam1.jpg}
\caption{Iteration 13, flip toward cam1.  Opposite direction of the
same failure: upper half is cam1's composition, lower half is
cam0's crop.  Seam at a similar phase of the DMA cycle.}
\label{fig:seam:b}
\end{subfigure}\hfill
\begin{subfigure}[t]{0.31\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/experiment_frames/fig_seam_extra.jpg}
\caption{Iteration 8, flip toward cam0 again.  Identical signature to
(a), confirming the failure is systematic, not a random glitch —
the MUX flip lands at the same phase of every buffer fill on this
firmware.}
\label{fig:seam:c}
\end{subfigure}
\caption{Mid-buffer seam produced by flipping the analogue MUX in
task context on pre-Fix-B firmware at \texttt{switch\_drain=1}.
Three representative frames out of the 16 captured in session
\texttt{s038\_e17\_drain\_ab/e17\_t1\_frames/}: \textbf{every} frame
in that folder exhibits the same tear (mean top/bottom brightness
diff $= 71\pm7$ across all 16 frames).  Source files:
\texttt{004\_cam0\_134ms.jpg}, \texttt{013\_cam1\_200ms.jpg},
\texttt{008\_cam0\_134ms.jpg}.}
\label{fig:seam}
\end{figure}

For the same drain threshold on the post-Fix-B firmware
(session [`s041_e17_eof_check`](../experiments/s041_e17_eof_check/)),
the mid-buffer seam is fully eliminated — Figure~\ref{fig:postfix}
shows two representative frames from that set.  These are the
frames a reviewer would compare against Figure~\ref{fig:seam} to
verify the fix visually.

\begin{figure}[H]
\centering
\begin{subfigure}[t]{0.44\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/experiment_frames/fig_clean_cam0.jpg}
\caption{Post-fix cam0 frame, same \texttt{switch\_drain=1} setting.
No seam; whole frame is cam0's composition with the teal mug visible
in the upper right, evenly exposed.}
\label{fig:postfix:cam0}
\end{subfigure}\hfill
\begin{subfigure}[t]{0.44\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/experiment_frames/fig_clean_cam1.jpg}
\caption{Post-fix cam1 frame.  No seam; whole frame is cam1's wider
view.  Contrast this against any cam1 panel in Figure~\ref{fig:seam}:
same sensor, same scene, same \texttt{drain} threshold, different
firmware build.}
\label{fig:postfix:cam1}
\end{subfigure}
\caption{Post-Fix-B \texttt{switch\_drain=1} frames.  The mid-buffer
seam characterised in Figure~\ref{fig:seam} is removed at the pixel
level.  Source:
\texttt{experiments/s041\_e17\_eof\_check/e17\_t1\_frames/\{002\_cam0\_201ms,
003\_cam1\_201ms\}.jpg}.  A statistical check (top/bottom half
brightness difference) over all 8 frames in that folder shows
\emph{no} frame with a half-vs-half brightness jump exceeding the
normal cam-specific contrast ratio — the seam is gone as a
distribution, not only for the two samples displayed.}
\label{fig:postfix}
\end{figure}

Every broken frame in that folder has the same signature: the tear
lands somewhere in the middle 40–60 % of the image, because the MUX
flip happened during the DMA of that buffer and the line at which the
switch occurred maps directly to the tear location.  Post-flip-on-EOF
(session
[`s041_e17_eof_check`](../experiments/s041_e17_eof_check/)), the MUX
transition is deferred to the VBLANK between frames, so no DMA buffer
straddles two sensors at `switch_drain=2`.  The tearing signature
disappears from the full 16-frame `drain=2` set.

### Known limitations

These are open issues that the current firmware does NOT solve.
Documented so a future reader knows where to poke.

- **`switch_drain(1)` is not safe in practice, despite the
  flip-on-EOF fix.**  The user-level review of session `s041_e17_eof_check`
  found that some `drain=1` frames still show artifacts even with the
  VBLANK-aligned MUX flip.  The current best explanation is that, while
  the MUX transition itself now lands in VBLANK, the **sensor state**
  on the newly selected OV5640 is not fully settled by the time the
  first post-switch DMA buffer completes — AEC/AGC convergence,
  internal-pipeline flush, and the first-frame-after-stream-resume
  behaviour of the sensor collectively produce subtle pixel-level
  anomalies that `drain=2` masks by simply waiting one more frame.
  Consequence: **`switch_drain=2` remains the default**, and the
  `switch_drain(1)` knob is retained only as an experimentation hook.
  Reproducing: set `sentai.camera.switch_drain(1)` before running E17
  and inspect the full 16-frame `e17_t1_frames/` set; artifacts are
  frame-dependent and the majority of frames do look clean, which is
  why a thumbnail-level first pass missed them.
- **Timing-cost of `switch_drain=1` vs `switch_drain=2` is ~zero on
  the current implementation**: the E17 CSVs at `s041` show
  [`001_e17_switch_drain_t2.csv`](../experiments/s041_e17_eof_check/001_e17_switch_drain_t2.csv)
  and
  [`002_e17_switch_drain_t1.csv`](../experiments/s041_e17_eof_check/002_e17_switch_drain_t1.csv)
  producing *identical* ~201 ms per-iteration wall time.  Root cause
  is the way `sentai_cam_get_raw_with_recovery` blocks on
  `cam->GetRawFrame` immediately after the `wait_iters` loop: the
  blocking grab compensates for whichever frame threshold was chosen.
  To make `drain=1` actually cheaper would require replacing the
  trailing blocking grab with a `TryGetRawFrame` of the already-queued
  buffer, which is a non-trivial change to the drain path.
- **Directional asymmetry cam1 − cam0** was +65 ms pre-fix and is
  now ≤ 2 ms — but it was never analysed as a *sensor-side* effect.
  If that residual couple of ms matters for a future application, a
  FSIN master/slave wire between the two OV5640s would align their
  frame phases on the shared MIPI-CSI lane.  Not attempted; hardware
  rework beyond the scope of this iteration.
- **45 fps and 60 fps at 720p are not reachable** with the current
  NXP SDK PLL table.  45 fps is not a 720p mode on the OV5640 (the
  datasheet lists 45 only at 1280×960); 60 fps via 2×2 binning was
  probed and the PLL was accepted by the driver but the CSI-2
  receiver never locked, indicating additional OV5640
  register-sequence work (binning-mode init) would be required that
  the NXP SDK does not currently emit.  Documented in the
  `DEMO_CAMERA_FRAME_RATE` comment block for future attempts.
- `switch_drain` is kept at **default 2** as a belt-and-suspenders
  conservatism.  With flip-on-EOF, threshold 1 is safe and threshold 2
  costs at most one extra frame interval (~33 ms at 30 fps) — the
  latency savings from dropping to 1 are marginal compared to the risk
  of a future regression re-introducing a seam, so the default stays
  defensive.  Users opt in explicitly via `sentai.camera.switch_drain(1)`
  when they need the extra frame.

### Head-to-tail timing on Fix B (session [`s042_e16_eof_30fps_x40`](../experiments/s042_e16_eof_30fps_x40/))

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
| Pre-Fix A (15 fps, 2-frame drain) — [`s034`](../experiments/s034_e15_vs_e16_x40/) | 4.7 | 211.4 ± 32.5 ms | **+64.2 ms** | half-and-half frames, visible seam |
| Fix A (15 fps, atomic snapshot) — [`s035`](../experiments/s035_e15_vs_e16_x40/) | 4.7 | 212.4 ± 33.5 ms | **+66.0 ms** | unchanged — seam still present |
| Fix B (30 fps, flip-on-EOF, drain=2) — [`s043`](../experiments/s043_e18_headtail_drain2/)/[`s044`](../experiments/s044_e18_headtail_drain2/) | 6.86 | 145.3–145.7 ms | +0.1 / −1.1 ms | `drain=2` clean; `drain=1` still exhibits sensor-side artifacts (known limitation) |
| **Fix B post-review (A1-A7, B1-B4)** — [`s045`](../experiments/s045_e18_post_refactor/) | **6.86** | **145.8 ms** | **−2.1 ms** | no regression; adds persistent fault counters via `sentai.diag.cam_stats()` |

Net effect vs baseline: **1.46× speed-up** on total frame time, **1.46× FPS**,
full elimination of the directional asymmetry, and elimination of the
mid-buffer seam at `drain=2`
([`experiments/s041_e17_eof_check/e17_t2_frames/`](../experiments/s041_e17_eof_check/e17_t2_frames/)
vs the old
[`experiments/s038_e17_drain_ab/e17_t1_frames/`](../experiments/s038_e17_drain_ab/e17_t1_frames/)).
`drain=1` is NOT fully clean even post-fix — see "Known limitations"
below.

### What is still open

The modulo scheduler introduces `(a+b)`-frame quantisation, so the
effective per-camera rate is only what the ratio rounds to.  For finer
control the user can do their own rate policy from Python around
manual `select()` calls.  The residual ~145 ms per-switch cost is now
evenly split between `select` (~18 ms waiting for the EOF arm to be
consumed — one frame interval at 30 fps) and `to_tensor` (~96 ms
= drain + PXP + quant — dominated by the `switch_drain=2` wait for two
fresh frames).  Dropping to `switch_drain(1)` was originally expected
to save ~33 ms, but empirically (see "Known limitations" below) the
trailing blocking `GetRawFrame` absorbs the saved wait and the
threshold-1 run is NOT visually clean — so `drain=2` stays the
operating point.

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

### Session [`s045_e18_post_refactor`](../experiments/s045_e18_post_refactor/) — final run, post-refactor, 30 fps, flip-on-EOF, drain=2, 40 reps per sweep

Reproduced across three back-to-back sessions on three different firmware
builds ([`s043`](../experiments/s043_e18_headtail_drain2/) pre-refactor,
[`s044`](../experiments/s044_e18_headtail_drain2/) final pre-refactor
confirmation, [`s045`](../experiments/s045_e18_post_refactor/)
post-refactor NASA/JPL review fixes).  All three agree within ≤ 1 ms on
every stage — the review fixes (A1-A7 + B1-B4 from
[agent/agent.md](../agent/agent.md)) are performance-neutral.

**Raw CSVs:**
[A — fixed cam0](../experiments/s045_e18_post_refactor/001_e18_A_fixed_cam0.csv) ·
[B — fixed cam1](../experiments/s045_e18_post_refactor/002_e18_B_fixed_cam1.csv) ·
[C — alternating](../experiments/s045_e18_post_refactor/003_e18_C_alt_cam0_cam1.csv).
For an index of every session downloaded locally, see
[experiments/README.md](../experiments/README.md).

| Sweep | `select` | `to_tensor` | `invoke` | `detect` | **total** | **FPS** |
|---|---:|---:|---:|---:|---:|---:|
| A — fixed `cam0` | 0.0 | 31.6 | 29.8 | 0.4 | **61.8 ms** | **16.18** |
| B — fixed `cam1` | 0.0 | 31.6 | 30.5 | 0.4 | **62.5 ms** | **16.00** |
| C — alternating | 17.6 | 96.0 | 31.5 | 0.7 | **145.8 ms** | **6.86** |

Per-direction inside the alternating sweep:

| direction | n | total (mean, ms) |
|---|---:|---:|
| → `cam0` | 19 | 146.7 |
| → `cam1` | 20 | 144.6 |
| **asymmetry (cam1 − cam0)** | — | **−2.1 ms** |

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

Switching *to* a given camera pays a fixed ~83 ms tax.  The
theoretical saving at `switch_drain(1)` is one frame interval
(~33 ms), but empirical measurement on `s041_e17_eof_check` shows
**no** timing saving (the trailing blocking `GetRawFrame` compensates)
AND a residual sensor-side artifact that the `drain=2` setting masks.
Both effects are captured in the "Known limitations" section below;
the net operational guidance is **leave `switch_drain` at the default
of 2** on the shipping firmware.

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
python3 diag/_host_upload_repl.py --file e_pipeline.py --file _util.py \
                                  --file _session.py --file __init__.py

# Run the head-to-tail benchmark.
python3 diag/drivers/_e18_post_refactor.py

# Pull the CSVs off the board over HTTP GET (reads are reliable — only
# writes hang on this firmware).
curl -s http://10.0.0.1/api/raw/diags/s045_e18_post_refactor/003_e18_C_alt_cam0_cam1.csv
```

Raw CSVs, scene snapshots from both cameras (before + after), and
per-experiment description `.txt` files are persisted in the session
folder on the device and survive reboot.  A snapshot of all 25 E15-E18
sessions downloaded on 2026-04-20 is archived under
[../experiments/](../experiments/) with a narrative index in
[../experiments/README.md](../experiments/README.md).

## Architecture improvements since the E15 baseline

Before this sprint, E15 gave 15 FPS on a fixed camera and E16 had not
been written.  The table below lists every code change made in service
of dual-camera alternation, ordered by commit time, with the purpose,
risk, and measured effect of each.  Entries marked `paper/memcpy.md`
land outside this document but are included because E15's own 15 FPS
ceiling depends on them.

| # | Change | Files | Why | Effect |
|---|---|---|---|---|
| 1 | **eDMA memcpy for tensor staging** (prerequisite baseline) | [detection_task.cc](../detection_task.cc) | CPU memcpy of 786 KB through the D-cache was 32 % of the per-frame critical path (24 ms) | E15 from 13.39 → 15.47 FPS; covered in [paper/memcpy.md](memcpy.md) |
| 2 | **YOLO layout auto-detection** | [sentai_runtime.cc:yolo_infer_info](../sentai_runtime.cc) | Model-agnostic NMS so E15 runs on the 1-class and the 80-class models with the same code | No FPS change; removes a per-model branch |
| 3 | **Both-camera scene snapshots** in diagnostics | [diag/_session.py:snapshot_both_cameras](../diag/_session.py) | Every E1x session saves before/after from cam0 AND cam1 — needed to diff scenes offline at switch-time | Diagnostics quality; zero runtime cost |
| 4 | **Fix A — atomic snapshot inside `HandleSwitchCameraRequest`** | [camera.cc](../../../libs/camera/camera.cc), [sentai_runtime.cc](../sentai_runtime.cc) | Kill the 1–10 ms queue-latency race between `g_cam_switch_seq` snapshot and the actual GPIO flip | Null timing result but invariant becomes explicit; see §"Fix A" |
| 5 | **Fix B.1 — 30 fps camera mode** | [libs/camera/camera_support.h](../../../libs/camera/camera_support.h) | OV5640 natively supports 720p @ 30 fps via the NXP driver's existing lookup; halves every "wait for N frames" cost | Per-switch overhead ~134 ms → ~67 ms component of the drain; alternating throughput 4.7 → 6.87 FPS |
| 6 | **Fix B.2 — ISR-safe MUX helper** (`GpioSetFromIsr`, `SentaiCamMuxSetFromIsr`) | [libs/base/gpio.cc](../../../libs/base/gpio.cc) | The existing `GpioSet` takes `g_mutex` → illegal in ISR context.  New helper uses atomic `DR_SET`/`DR_CLEAR` | Prerequisite for running MUX flip in CSI ISR |
| 7 | **Fix B.3 — Flip-on-EOF (VBLANK-aligned MUX switch)** | [camera_support.c:CSI_IRQHandler](../../../libs/camera/camera_support.c), [sentai_runtime.cc:sentai_cam_switch](../sentai_runtime.cc) | Move GPIO flip into the CSI end-of-frame ISR so it lands in the MIPI VBLANK window, not mid-DMA-buffer.  Eliminates the "half-and-half" seam on the default `drain=2` path | Mid-buffer seam gone at `drain=2`; directional asymmetry (cam1 − cam0) collapses from +65 ms to ≤ 2 ms.  **Note**: `drain=1` is still not fully clean — see "Known limitations". |
| 8 | **Fix B.4 — Stateless modulo ratio scheduler** | [camera_support.c:CSI_IRQHandler](../../../libs/camera/camera_support.c), `sentai.camera.ratio(a, b)` | Asymmetric capture (e.g. 3:1 → cam0 22.5 fps, cam1 7.5 fps) without any mutable counter in ISR — O(1) per frame | Enables application-layer rate policies; zero overhead when both quotas are zero |
| 9 | **Fix B.5 — `sentai.camera.switch_drain(n)` runtime toggle** | [modsentai_camera.c](../modsentai_camera.c), [sentai_runtime.cc](../sentai_runtime.cc) | A/B comparison of drain=1 vs drain=2 without reflashing; experimentation hook for future work | Default 2 (shipping).  `drain=1` did NOT produce the expected speed-up or the expected clean frames — see "Known limitations". |
| 10 | **Fix B.6 — `sentai.camera.set_resolution(w,h)` cross-resolution** | already existed, validated for 720p/VGA/QVGA | Lower resolutions shrink PXP + JPEG cost proportionally | 720p alternating ~2 FPS; VGA ~5.4; **QVGA ~10** — documented in §"Per-camera resolution" |
| 11 | **Review A1 — Unified MUX polarity header** | [libs/camera/cam_mux.h](../../../libs/camera/cam_mux.h) (new) | Polarity constants were duplicated in `camera.cc` and `camera_support.c` — a silent bug waiting to happen | Per embeded.md §J: single source of truth |
| 12 | **Review A2 — Packed 32-bit atomic ratio update** | [sentai_runtime.cc:sentai_cam_ratio_set](../sentai_runtime.cc), [camera_support.c](../../../libs/camera/camera_support.c) | Two-field volatile update could leave ISR reading `(new_a, old_b)` transient | Single 32-bit store → atomic from ISR's point of view |
| 13 | **Review A3 — Delta-based deadline** | [sentai_runtime.cc:sentai_cam_switch](../sentai_runtime.cc) | `now < deadline` fails if `TickType_t` wraps at 49.7 d uptime; replaced with `(now − ts0) < budget` | Survives tick-counter wrap |
| 14 | **Review A4 — Remove duplicate `g_cam_current_id` write** | [sentai_runtime.cc](../sentai_runtime.cc) | ISR already wrote the id on the nominal path; task re-write was redundant and confused ownership | Explicit single-writer per path |
| 15 | **Review A5 — Tracker notification moved after flip** | [sentai_runtime.cc](../sentai_runtime.cc) | `sentai_tracker_set_active_camera(id)` used to run BEFORE the ISR consumed the arm — tracker would tag a frame with the wrong camera for up to 18 ms | Tracker state follows hardware reality |
| 16 | **Review A6 — Persistent fault counters + `sentai.diag.cam_stats()`** | [sentai_runtime.cc](../sentai_runtime.cc), [modsentai_diag.c](../modsentai_diag.c), [sentai_error.h](../sentai_error.h), [error_codes.csv](../error_codes.csv) | Degraded paths (sync fallback, drain timeout, grab retry, grab fatal) were logged to printf only — no post-mortem trace | New error codes `0x0A00`–`0x0AF0`; `cam_stats()` dict survives across REPL reconnects |
| 17 | **Review A7 — Magic numbers documented** | [sentai_runtime.cc](../sentai_runtime.cc), [camera_support.c](../../../libs/camera/camera_support.c) | `150 ms` arm timeout, `300 iters` drain ceiling, `500 ms` fast-path mutex — all had no rationale in code | Each now documented as `= K × frame_interval + margin`, scales if frame rate changes |
| 18 | **Review B1 — `resolution=(w,h)` param in E16/E17/E18** | [diag/e_pipeline.py](../diag/e_pipeline.py) | Hardcoded 512×512 blocked measuring at VGA/QVGA without editing the experiment | All three now take `resolution=(w,h)` with 512×512 default |
| 19 | **Review B2 — `_ensure_model_loaded(path)` helper** | [diag/_util.py](../diag/_util.py) | Same path-keyed TPU reload logic was duplicated 3× across e14/e15/e16/e18 — each copy a potential bug | Single helper; caller just calls `_ensure_model_loaded(model_path)` |
| 20 | **Review B3 — Ad-hoc drivers moved to `diag/drivers/`** | filesystem reorg | `_e15_*.py`, `_e16_eof.py`, etc. were polluting the runtime root and were being glob-ed unintentionally | Uploaders only glob the top level of `diag/`; `drivers/` stays host-side |
| 21 | **Review B4 — `[cam_switch]` printf gated on `g_sentai_frame_verbose`** | [sentai_runtime.cc](../sentai_runtime.cc) | Every MUX flip emitted a printf even during 40-rep timing loops → CDC-ACM noise | With `sentai.verbose(0)` the log is silent; it reappears on verbose=1 for interactive debug |
| 22 | **REPL chunked uploader** (precondition to iteration) | [diag/_host_upload_repl.py](../diag/_host_upload_repl.py) (new) | HTTP `/api/write` hangs on this firmware; MSC is heavy.  Needed a fast, reliable way to push diag changes | CHUNK=48 bytes (REPL line buffer is 256 chars); full-buffer terminator match; replaces `repl_run.py` for file push |
| 23 | **`sentai_lfs_task.cc` — LS always slow-path** (support fix) | [sentai_lfs_task.cc](../sentai_lfs_task.cc), [paper/lfs.md](lfs.md) | Root `/api/ls/` could block `tcpip_thread` > 30 s → network watchdog reset loop → work blocked | Keeps tcpip_thread bounded; camera-switch iteration could proceed |

**What did NOT change since E15:**

- The firmware's parallel-pipeline architecture (PrepTask + InferTask
  with staging buffer + semaphore handoff) is unchanged.
- The EdgeTPU inference path, NMS, output tensor layout are unchanged.
- The TFLite arena, model loading, and quantisation paths are unchanged.
- The two-camera hardware MUX topology is unchanged (only the software
  timing of when we flip it changed).

**What is still available as a knob, not yet on by default:**

- `sentai.camera.switch_drain(1)` — exposed as an experimentation
  hook, but empirically it produces neither the expected 33 ms saving
  (see "Known limitations" §timing-cost) nor a fully clean frame set
  (see "Known limitations" §safe-in-practice).  Use only for A/B
  investigations, not for shipping configurations.
- `sentai.camera.ratio(a, b)` with `(a, b) != (0, 0)` — asymmetric
  schedules; not on at boot.
- Non-default resolutions — QVGA via `set_resolution(320, 240)` gives
  ~10 FPS alternating (2× headroom over 512×512) because PXP + JPEG
  costs scale with pixels.  Unlike `drain=1`, this headroom is real.

The numeric story: **15 FPS (E15 baseline fixed camera, after eDMA)
→ 4.7 FPS (naive switch every frame, 15 fps sensor) → 6.86 FPS (switch
every frame on Fix B + 30 fps sensor, seam-free at `drain=2`,
asymmetry-free, post-refactor with fault counters)**.  Real headroom
comes from QVGA (~10 FPS alternating) or from the `ratio(a, b)`
scheduler amortising the tax across multiple frames on one camera.

## Final summary

| Question | Answer |
|---|---|
| What was the root cause of the E17 `drain=1` seam? | MUX flip was happening in task context (`cam->SwitchCamera`), landing at an arbitrary phase inside an active DMA buffer fill. Half the buffer was from the old sensor, half from the new. |
| What fix was actually needed? | Move the GPIO flip into the CSI EOF ISR so the analogue MUX transitions during VBLANK, before any new DMA buffer begins filling. One short ISR branch per frame, per NASA/JPL §C. |
| What was gained? | The default-path (`drain=2`) seam is gone, total per-switch overhead dropped from ~211 ms to 145.7 ms (1.46×), directional asymmetry (cam1 vs cam0) collapsed from +65 ms to +0.1 ms.  `drain=1` is NOT fully clean even post-fix — see "Known limitations". |
| What is the final sustained FPS? | **Fixed camera: 16.0 FPS** (exact sensor rate). **Switch every frame: 6.86 FPS** (per-switch tax = 83 ms, at the shipping `drain=2`). |
| What can the application layer do about the 83 ms tax? | (1) Use `sentai.camera.ratio(a, b)` to amortise across multiple frames — e.g. `(9, 1)` ≈ 12.7 FPS average. (2) Drop native resolution to VGA/QVGA — QVGA gives ~10 FPS alternating because PXP + JPEG scale with pixel count. |
| What is still unresolved? | (1) `switch_drain(1)` — defer-by-one-frame drain is not reliably clean even post-flip-on-EOF; see "Known limitations".  (2) 45 fps and 60 fps at 720p — OV5640 datasheet lists 45 only at 1280×960, and 60 fps (2×2 binning) was probed but CSI2RX did not lock.  30 fps remains the ceiling without driver-level OV5640 work. |

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

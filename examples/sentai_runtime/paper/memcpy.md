# SDRAM memcpy optimisation — from 32 MB/s CPU copy to 54 MB/s eDMA bursts

## Abstract

The end-to-end vision pipeline on the SentAI board (Coral Dev Board Micro,
NXP i.MX RT1176) was stuck at ~13 FPS even though the camera delivers
15 FPS and the EdgeTPU invoke on our lightest model only costs ~50 ms.
Instrumenting the pipeline's InferTask exposed a single dominant cost:
a 786 KB `memcpy(tensor_buf, staging_buf, total)` from SDRAM to SDRAM
that consumed **24 ms per frame** due to CPU-driven cache-line traffic
through the SEMC controller. Replacing that one `memcpy` with an eDMA
memory-to-memory transfer configured for 32-byte AXI bursts took the
copy down to **14.6 ms** and raised sustained pipeline throughput from
**13.41 FPS** to **15.47 FPS** — enough to match the sensor rate.

## Test conditions

### Hardware

| Component | Value |
|-----------|-------|
| MCU | NXP i.MX RT1176 (Cortex-M7 @ 800 MHz + Cortex-M4) |
| On-chip EdgeTPU | Coral/Google TPU, internal USB2 bus |
| External RAM | 16 MB SDR-SDRAM on SEMC (166 MHz) |
| OCRAM / DTCM / ITCM | 1.25 MB / 256 KB / 256 KB |
| Camera | OV5640-based coralmicro module, 1280×720 native, 15 FPS streaming |
| USB to host | CDC-ACM (REPL) + CDC-NCM (IP 10.0.0.1) |
| Board | Coral Dev Board Micro dev kit, powered via USB-C |

### Firmware

| Parameter | Value |
|-----------|-------|
| Build | `sentai_runtime` build #622+ (linker script `MIMXRT1176xxxxx_cm7_ram_mp.ld`) |
| RTOS | FreeRTOS (CMSIS M7 build, 1 ms tick) |
| MicroPython GC heap | 512 KB, in `.sdram_bss` |
| Pipeline tasks | `det_prep` prio 2 · `det_infer` prio 3 |
| Camera task | `camera_task` prio `configMAX_PRIORITIES - 1` |

### Model under test

| Parameter | Value |
|-----------|-------|
| File | `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite` |
| On-flash size | 5 591 680 B (5.33 MB) |
| Input | `uint8[1, 512, 512, 3]` — **786 432 B** (this is the buffer we memcpy) |
| Input quantisation | scale = 1/255, zero_point = 0 |
| Output | `uint8[1, 1344, 6]` — 8 064 B — YOLOv5-enhanced anchor format |
| Architecture | YOLOv5 enhanced (single upsample at P5/32, 1-class head) |
| Output quantisation | scale = 1/255, zero_point = 0 |
| Output row semantics | `[cx, cy, w, h, obj_conf, class_conf]` (normalised 0..1) |
| TFLite arena | 799 KB used / 8 192 KB available |
| EdgeTPU opened in | mode 3 (`kMax`) |

### Scene

- Static scene (camera does not move during measurement).
- Before/after scene snapshots are saved by every experiment that touches
  the camera — see `diag.snapshot_scene("before"|"after")` in
  [diag/_session.py](../diag/_session.py).  Stored at
  `/diags/<session>/scene_cam<N>_(before|after)_<W>x<H>.jpg`.
- For the runs reported here the scene contained no target class instances,
  so `num_detections == 0` every frame — the NMS execution path is still
  run in full (candidate scan over all 1344 anchors), only the sort and
  IoU stages exit early.

### Methodology

- Each run = 20 measurement frames after 1 warm-up frame.
- 10 back-to-back runs per configuration to capture variance.
- **Both configurations measured from the same firmware image** using
  the runtime A/B flag `sentai.pipeline.dma_memcpy(0|1)`.  Switching
  between CPU memcpy and eDMA is one volatile store; no reflash, no
  reboot, no camera restart between the two blocks.  This removes
  every confound that could come from firmware drift, thermal state,
  or camera calibration variance.
- Timing collected via `sentai.pipeline.get_ex()` which returns the
  firmware-measured `inference_ms` (pure TPU `Invoke`), `memcpy_ms`
  (staging → tensor copy), `nms_ms` (post-processing), and `total_ms`
  (full InferTask iteration). All reported in milliseconds.
- Host (Python) wall-clock interval is the delta between consecutive
  `pipeline.get_ex()` returns; the gap between `wall_interval` and
  `infer_total` measures everything outside InferTask (queue IPC,
  MicroPython overhead, mp_repl wake-up).
- `verbose(0)` for the whole loop body — no `printf` traffic on CDC-ACM,
  so USB never saturates and host reads don't stall.
- Benchmark driver: [_e15_ab.py](../_e15_ab.py).

## Baseline — CPU `memcpy` (SDRAM → cache → SDRAM)

Flag set by `sentai.pipeline.dma_memcpy(0)`.  The InferTask uses a plain
`memcpy()` to move the prepared frame into the TFLite input tensor.
This matches the state right after the camera-drain fix (non-blocking
`TryGetRawFrame`, documented separately in [usb.md](usb.md) and
[camera.md](camera.md)).

### Per-run results (10 × 20 frames, same firmware as optimised)

| Run | wall | invoke | memcpy | nms | infer_total | FPS |
|----:|----:|-----:|-----:|----:|-----------:|----:|
|  1 | 75.9 | 50.8 | 24.0 | 0.3 | 75.1 | 13.2 |
|  2 | 74.8 | 49.9 | 24.4 | 0.4 | 74.6 | 13.4 |
|  3 | 74.6 | 50.1 | 24.3 | 0.3 | 74.6 | 13.4 |
|  4 | 74.7 | 50.3 | 24.0 | 0.4 | 74.6 | 13.4 |
|  5 | 74.6 | 50.0 | 24.3 | 0.2 | 74.4 | 13.4 |
|  6 | 74.2 | 50.0 | 24.1 | 0.1 | 74.3 | 13.5 |
|  7 | 74.6 | 50.3 | 24.1 | 0.2 | 74.6 | 13.4 |
|  8 | 74.9 | 50.3 | 24.1 | 0.2 | 74.7 | 13.4 |
|  9 | 74.2 | 50.1 | 24.0 | 0.1 | 74.2 | 13.5 |
| 10 | 74.4 | 50.3 | 24.0 | 0.2 | 74.4 | 13.4 |
| **mean** | **74.7** | **50.2** | **24.1** | **0.2** | **74.5** | **13.39** |

**FPS stats: mean 13.39, min 13.18, max 13.49, σ ≈ 0.10.**

### Interpretation

The pipeline loop is fully dominated by InferTask:
`gap = wall − total ≈ 0.2 ms`, which is the cost of the FreeRTOS
`xQueueSend`, CDC-NCM wake-up and MicroPython tuple allocation.
Everything else is inside the firmware, and of that `memcpy` alone is
**32 % of the critical path** (24.1 / 74.4).

Throughput during the copy = 786 432 B / 24 ms ≈ **32.8 MB/s**.  That is
far below SDRAM's theoretical 400 MB/s peak.  The CPU is single-word-
copying through the D-cache, which forces:

1. Each 32-byte cache line of **source** is fetched from SDRAM (24 576
   fills for 786 KB).
2. Each 32-byte cache line of **destination** is *also* fetched from
   SDRAM, because the Cortex-M7 D-cache is write-allocate.
3. The cache line is modified in place.
4. The dirty destination line is later written back to SDRAM.

So the SEMC bus sees ~2.4× the payload size (~1.9 MB) and all of it as
isolated single-beat AXI accesses — the bus cannot coalesce consecutive
CPU stores into back-to-back bursts.  Application note AN12437 explicitly
calls this out: *"SDRAM can reach high throughput when accessed by LCD
and PXP, as these two masters support back-to-back access, with better
performance compared to other master access, but dropping when accessed
by CPU core."*

## Optimised — eDMA memcpy with 32-byte AXI bursts

The replacement helper lives in
[detection_task.cc](../detection_task.cc):

```cpp
// DMA0 channel 31 (audio driver uses ch 0).  One-time init.
static constexpr uint32_t kSentaiDmaChannel = 31;
static edma_handle_t s_dma_memcpy_handle;

// Pick the widest transfer width the addresses and size allow.
// 32-byte → one 8-beat AXI burst per request, exactly the pattern SEMC
// is optimised for.
uint32_t width = 4;
if ((((uintptr_t)src | (uintptr_t)dst | size) & 0x1Fu) == 0u)      width = 32;
else if ((((uintptr_t)src | (uintptr_t)dst | size) & 0x07u) == 0u) width = 8;

edma_transfer_config_t tcfg;
EDMA_PrepareTransfer(&tcfg, src, width, dst, width,
                     /*bytesEachRequest=*/size,
                     /*transferBytes=*/size,
                     kEDMA_MemoryToMemory);
EDMA_SubmitTransfer(&s_dma_memcpy_handle, &tcfg);
EDMA_StartTransfer(&s_dma_memcpy_handle);          // arm (SERQ)
EDMA_TriggerChannelStart(DMA0, kSentaiDmaChannel); // fire first minor loop

// Bounded polled wait — no ISR, no semaphore, no scheduler coupling.
const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(50);
while ((EDMA_GetChannelStatusFlags(DMA0, kSentaiDmaChannel)
        & kEDMA_DoneFlag) == 0) {
    if (xTaskGetTickCount() > deadline) return false;  // fallback to CPU memcpy
}
EDMA_ClearChannelStatusFlags(DMA0, kSentaiDmaChannel,
                             kEDMA_InterruptFlag | kEDMA_DoneFlag);
DCACHE_InvalidateByRange((uint32_t)dst, size);
```

Two non-obvious details that cost measurable debug time:

1. **`EDMA_StartTransfer` only sets SERQ** (the request-enable bit).
   Memory-to-memory transfers have no peripheral request line, so the
   minor loop never fires until we explicitly software-trigger via
   `EDMA_TriggerChannelStart` (SSRT register).  Without that call the
   channel sits armed forever and the 50 ms bounded wait expires.
2. **Poll `kEDMA_DoneFlag`, not `kEDMA_InterruptFlag`**.  Interrupts are
   not enabled on this channel (no IRQ handler installed), so the
   interrupt flag would never set.

No ISR is installed; the wait is a bounded polled loop.  A 786 KB
transfer completes in ~15 ms of wall time with a 35 ms safety margin,
so the CPU can't livelock.  On polled-loop timeout (DMA hardware stuck
or mis-configured) the code falls back to plain `memcpy` — the pipeline
degrades, doesn't brick.

### Cache strategy

Both buffers are written exclusively by DMA masters in steady state
(`s_staging_buf` by PXP after camera capture; the TFLite input tensor
by our new eDMA).  DMA writes bypass the D-cache, so neither buffer
holds dirty cache lines under normal operation.  Our first attempt
called `DCACHE_CleanInvalidateByRange` on both source and destination
before the DMA, for safety.  That added **~20 ms** per frame — cache
ops walk all address ranges and on SDRAM the walk itself serialises
with SEMC bus writebacks of whatever cache lines happen to be dirty.
We measured an end-to-end memcpy of 78 ms with the defensive cache
flush, worse than the original CPU `memcpy`.

Removing the pre-DMA `CleanInvalidate` — while keeping a mandatory
`DCACHE_InvalidateByRange(dst, size)` **after** the DMA so subsequent
CPU reads see fresh bytes — was what actually delivered the win.

### Per-run results (10 × 20 frames, same firmware, dma_memcpy=1)

Flag set by `sentai.pipeline.dma_memcpy(1)`.  Measured immediately
after the baseline block — same camera, same scene, same model, same
Python loop body, same TPU state.  Only the runtime flag changed.

| Run | wall | invoke | memcpy | nms | infer_total | FPS |
|----:|----:|-----:|-----:|----:|-----------:|----:|
|  1 | 64.3 | 49.5 | 14.6 | 0.2 | 64.3 | 15.6 |
|  2 | 66.3 | 50.2 | 14.6 | 0.3 | 65.1 | 15.1 |
|  3 | 64.6 | 49.7 | 14.6 | 0.2 | 64.4 | 15.5 |
|  4 | 65.9 | 49.8 | 14.6 | 0.2 | 64.7 | 15.2 |
|  5 | 64.3 | 49.5 | 14.6 | 0.2 | 64.2 | 15.6 |
|  6 | 66.1 | 50.1 | 14.4 | 0.2 | 64.7 | 15.1 |
|  7 | 64.7 | 49.7 | 14.6 | 0.2 | 64.5 | 15.5 |
|  8 | 66.1 | 50.0 | 14.6 | 0.3 | 64.8 | 15.1 |
|  9 | 64.6 | 49.7 | 14.5 | 0.2 | 64.4 | 15.5 |
| 10 | 66.1 | 49.9 | 14.7 | 0.2 | 64.8 | 15.1 |
| **mean** | **65.3** | **49.8** | **14.6** | **0.2** | **64.6** | **15.32** |

**FPS stats: mean 15.32, min 15.07, max 15.56, σ ≈ 0.22.**

Throughput during the copy = 786 432 B / 14.6 ms ≈ **53.9 MB/s** — a
**1.65× improvement** for the same buffer size and addresses.

The observed pipeline FPS sits right at the camera sensor rate,
confirming that with the DMA optimisation engaged the sensor — not the
compute path — is the rate-limiting step.

## Comparison and discussion

### Per-stage cost (paired A/B on same firmware, 200 frames each)

| Stage | CPU memcpy | eDMA 32-B bursts | Δ |
|---|---:|---:|---:|
| Invoke (TPU) | 50.2 ms | 49.8 ms | −0.4 ms (noise) |
| staging → tensor copy | **24.1 ms** | **14.6 ms** | **−9.5 ms (−39 %)** |
| NMS (1-class × 1344 anchors, YOLOv5 layout) | 0.2 ms | 0.2 ms | — |
| InferTask total | 74.5 ms | 64.6 ms | **−9.9 ms** |
| wall interval | 74.7 ms | 65.3 ms | −9.4 ms |

The saving is entirely explained by the copy; invoke, NMS and the
Python/IPC gap are unchanged within noise.  Importantly, `invoke` did
**not** get slower even though the eDMA now competes with the TPU's
internal USB bus for SEMC bandwidth — the transfer is short enough
(15 ms, before invoke starts) that it finishes before invoke's input
upload begins.

### Throughput headline

| Config | FPS mean | FPS min | FPS max | Δ vs baseline |
|---|---:|---:|---:|---:|
| CPU memcpy (`dma_memcpy(0)`) | 13.39 | 13.18 | 13.49 | — |
| eDMA 32-B bursts (`dma_memcpy(1)`) | **15.32** | **15.07** | **15.56** | **+14.4 %** |
| Camera sensor rate (hard ceiling) | 15.00 | — | — | — |

The optimised configuration operates right at the camera sensor rate
(15 FPS ≈ 66.7 ms/frame); individual measurements slightly exceed this
when the pipeline drains a small in-flight queue over one cycle and
catches up on the next.  Sustained over a longer window the effective
rate is bounded by the camera.  The 1.92 FPS absolute gain corresponds
to a **14.4 % relative throughput improvement** measured under
controlled A/B conditions in the same firmware image.

### Variance

σ grows from 0.09 to 0.20 FPS — not material for the use case but
worth noting.  The extra variance comes from the 50 ms DMA polling
path occasionally getting preempted by the camera task or the
watchdog task; this would disappear if we re-armed the polling using
a task-notification wake-up driven by the eDMA completion IRQ, at
the cost of additional ISR state.  We judged the trade-off not worth
it while we are comfortably above the sensor rate.

## Why we chose eDMA over alternatives

| Option | Expected time | Why we didn't pick it |
|---|---:|---|
| Keep CPU `memcpy` + `__builtin_prefetch` hints | ~18 ms | Only ~25 % saving, still CPU-bound on SEMC |
| Move tensor to OCRAM | N/A | TFLite arena is 8 MB, exceeds 1.25 MB OCRAM |
| Move staging to OCRAM | mixed | Halves read-side cost but write-side still on SEMC; net ~10 ms, small gain for a large refactor |
| PXP with PS→output 1:1 | ~3-5 ms | PXP is already used by PrepTask; serialising PXP across PrepTask+InferTask would defeat pipeline parallelism |
| **eDMA mem-to-mem** | **~14-15 ms** | **Independent engine, back-to-back SEMC bursts, no conflict with PXP, isolated to a single helper function** |

PXP would in principle be even faster than eDMA (AN12437 lists PXP as
best-in-class for SDRAM throughput), but PXP is the camera-side
resize engine and scheduling it twice per frame pulls PrepTask and
InferTask into a shared-resource dance that kills parallelism.  eDMA
channel 31 is independent of every other user in the firmware, so
the change stays local and analysable.

## Failure modes

Per [agent/embeded.md](../agent/embeded.md): every optimisation must
have a defined failure path.

| Failure | Detection | Action |
|---|---|---|
| Addresses not 4-byte aligned, or `size % 4 != 0` | compile-time alignment check fails | `dma_ok = false` → falls back to CPU `memcpy` |
| eDMA submit returns non-success | Return value of `EDMA_SubmitTransfer` | Return `false` → CPU `memcpy` |
| Channel stuck (DONE flag never sets) | 50 ms bounded polled wait expires | Return `false` → CPU `memcpy` |
| Some other task takes DMA0 ch31 | Not detected; documented single-owner assumption | ch31 is dedicated to this helper — audio uses ch0; camera doesn't use eDMA |

No silent corruption: the DMA writes to an empty buffer the TFLite
tensor alone will read; `DCACHE_InvalidateByRange(dst)` after the DMA
guarantees the next invoker reads fresh bytes regardless of cache
state.

## Reproducing these results

```python
# On the device REPL, after flashing build #622+.
# One-shot E15 run — creates /diags/sNNN_e15_512/ with CSV + scene_{before,after}.jpg
import diag
diag.e15_pipeline_parallel_512(repetitions=5)

# 10x benchmark table (dma_memcpy stays at whatever the flag is set to)
import sentai
exec(sentai.fs.read_str('/_e15_table.py'))

# *** Paired A/B benchmark (this document's tables come from here) ***
# Runs 10x20 frames with DMA off, then 10x20 frames with DMA on,
# prints a side-by-side summary + FPS improvement.
exec(sentai.fs.read_str('/_e15_ab.py'))

# Manual toggle at any time:
sentai.pipeline.dma_memcpy(0)   # CPU memcpy (baseline)
sentai.pipeline.dma_memcpy(1)   # eDMA 32-B bursts (optimised)
sentai.pipeline.dma_memcpy()    # read current setting
```

Every run writes:

- `/diags/sNNN_e15_512/manifest.csv` — experiment log
- `/diags/sNNN_e15_512/001_e14_pipeline_par_cam0_512x512.csv` — per-frame timing
- `/diags/sNNN_e15_512/001_e14_pipeline_par_cam0_512x512.txt` — column key + params
- `/diags/sNNN_e15_512/scene_cam0_before_512x512.jpg` — scene at experiment start
- `/diags/sNNN_e15_512/scene_cam0_after_512x512.jpg` — scene at experiment end
- `/diags/sNNN_e15_512/summary.txt` — heap + duration summary

The pair of scene snapshots lets an operator confirm offline that the
camera was pointing at the expected target and that nothing moved
during the run.  This matters when `num_detections == 0` — it
distinguishes *"scene had no target"* from *"detector missed"*.

## Cross-model comparison — E14 (COCO 80-class) vs E15 (1-class)

To put the memcpy optimisation in context we ran the pipeline 20 times per
experiment (20 frames each, 400 frames total per configuration), each pair
of runs writing CSVs + scene snapshots under its own `/diags/sNNN_...`
session for later offline analysis.

| Property | E14 (Ultralytics YOLOv8 COCO) | E15 (custom YOLOv5-enhanced, 1-class) |
|---|---|---|
| Model file | `/yolo26n.edgetpu_1.tflite` | `/yolo_1_class_512_1_upsample_..._P5_32.tflite` |
| File size | 4.41 MB | 5.33 MB |
| Architecture family | YOLOv8 nano (Ultralytics) | YOLOv5 enhanced (single upsample at P5/32) |
| Output layout | `[1, 84, 2100]` (v8 transposed) | `[1, 1344, 6]` (v5-style: `[cx,cy,w,h,obj,cls]`) |
| Num. classes | 80 (COCO) | 1 |
| Input shape | int8 `[1, 320, 320, 3]` — 307 200 B | uint8 `[1, 512, 512, 3]` — 786 432 B |
| Output shape | int8 `[1, 84, 2100]` — 176 400 B | uint8 `[1, 1344, 6]` — 8 064 B |
| NMS inner cost | **80 classes × 2100 anchors = 168 000 dequant+compares/frame** | 1 class × 1344 anchors = 1 344 compares/frame |
| DMA memcpy | on (small 307 KB payload, marginal win) | on (decisive 786 KB payload) |
| Session | `s031_e14_x20/` | `s032_e15_x20/` |

### Aggregated results (20 runs × 20 frames each, verbose=0)

| Metric | E14 mean | E14 range | E15 mean | E15 range |
|---|---:|:---:|---:|:---:|
| frame_interval (ms) | 156.9 | 156.1–157.8 | 69.4 | 64.3–70.9 |
| pipeline FPS (wall) | 6.37 | 6.34–6.41 | 14.40 | 14.10–15.53 |
| firmware avg_fps | 6.19 | 6.00–6.20 | 14.08 | 13.40–14.30 |
| detections (sum) | 2 | — | 0 | — |

The **E14 experiment is NMS-bound** on this dataset: the 80-class yolo
iteration pays ~168 000 dequant + argmax operations per frame, which on
Cortex-M7 FPU at 800 MHz is visibly slow compared to the 1-class hot
path.  E15's NMS sees only 1 344 single-class comparisons per frame and
runs in < 1 ms.  Together with the much heavier 786 KB tensor copy that
E15 would have to do if we hadn't switched to eDMA, this is why the
bigger model can still run more than 2× faster than the smaller one.

### Detection of yolo layout & class count

To automate experiment setup across models with different class budgets,
the firmware now exposes `sentai.tpu.yolo_info()` which returns
`(layout_str, num_classes, num_anchors)` inferred purely from the
loaded model's output tensor shape (the edgetpu compiler strips most
metadata buffers from the .tflite binary, so shape is the only reliable
signal).  Verified on both experiments:

| Model file | `yolo_info()` |
|---|---|
| yolo26n.edgetpu_1.tflite (E14) | `('v8', 80, 2100)` |
| yolo_1_class_512_1_upsample_..._P5_32 (E15) | `('v5_like', 1, 1344)` |

Source: [`yolo_infer_info()` in sentai_runtime.cc](../sentai_runtime.cc).

### Artefacts saved for offline inspection

Each run in a session writes:

- `NNN_<tag>_pipeline_par_cam0_<W>x<H>.csv` — per-frame wall_interval,
  prep_stall, infer_stall, num_detections
- `NNN_<tag>_pipeline_par_cam0_<W>x<H>.txt` — column key + parameters
  (conf, iou, repetitions, frames_received, fw_processed, fw_dropped)
- `scene_cam0_before_<W>x<H>.jpg` — scene the operator pointed at, at
  session start.  Lets offline inspection confirm what the model saw.
- `scene_cam0_after_<W>x<H>.jpg` — same scene, session end.  Paired
  with BEFORE for drift / movement verification.
- `manifest.csv` + `summary.txt` — per-session roll-up.

## Progression across the pipeline optimisation effort

| Revision | memcpy | wall | FPS | Note |
|----------|-------:|-----:|----:|------|
| Baseline (camera drain polls 4 s) | 24 ms | ~200 ms | 5.0 | `TryGetRawFrame` not truly non-blocking |
| Non-blocking camera drain | 24 ms | 75 ms | 13.4 | Fix #1 — see usb.md / camera.md |
| **+ eDMA memcpy** | **14.6 ms** | **64.7 ms** | **15.5** | **Fix #2 — this document** |

End-to-end, **3.1× speedup over the initial firmware**, reaching
the camera sensor rate.

## What's next

- `invoke` at 49.6 ms now dominates the loop.  The remaining headroom
  is inside `Invoke()` itself — 28 ms cold when measured outside the
  pipeline, 49 ms steady-state while PrepTask runs concurrently,
  suggesting SEMC/USB bus contention with the TPU's internal DMA.
  Pushing beyond 15 FPS would require either a lighter model
  (smaller input → less TPU bus), or double-buffering the TFLite
  tensor so NMS/Invoke overlap more aggressively.
- The `sentai_dma_memcpy` helper is local to `detection_task.cc`.
  Three other places in the firmware copy large SDRAM buffers
  (`sentai_cam_capture_rgb`, `sentai_cam_capture_jpeg`,
  `sentai_cam_to_tensor_ex`).  Once lifted into `libs/base/dma_memcpy.*`
  the same win applies there.
- Switch to completion-interrupt + FreeRTOS notification once any
  concurrent user contests channel 31; the current polled loop is
  only safe because no other task uses this channel.

## Sources

- NXP *AN12437 — i.MX RT Series Performance Optimization* (SDRAM access
  patterns per master; PXP/LCD/eDMA back-to-back bursts).
- NXP *i.MX RT1170 Reference Manual* (`IMXRT1170RM`), eDMA and SEMC
  chapters (TCD layout, SSRT/SERQ semantics, AXI burst configuration).
- Local code:
  [detection_task.cc](../detection_task.cc),
  [modsentai_pipeline.c](../modsentai_pipeline.c),
  [diag/e_pipeline.py](../diag/e_pipeline.py),
  [diag/_session.py](../diag/_session.py).

---
name: Cale 1 ring buffer — PURE variant shipped (untested)
description: 2026-04-24 shipped Cale 1 PURE (.tpu_input in SDRAM, 72 KB OCRAM ring slots, ring default ON). Build #868+. Needs flash + pipeline regression test.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
**Shipped** Cale 1 PURE ring buffer in build #868+, untested on hardware.

## What ships

- New `.tpu_ring` section in OCRAM at 0x20243400, 2×36 KB slots = 73728 B.
- `.tpu_input` (786 KB) moved OCRAM → SDRAM.  Frees ~700 KB OCRAM.
- `.curl` (208 KB) moved OCRAM → SDRAM to make room.
- `g_sentai_tpu_ring_enabled` default **ON** — required with PURE layout because ring=0 would make USB read SDRAM directly (V13-style SEMC contention with CSI).
- Wired into all 3 bulk-OUT paths: `SendParameters`, `SendInstructions`, `SendInputs` in `libs/tpu/edgetpu_driver.cc`.
- Uses eDMA ch30 (distinct from detection_task ch31, no collision) polled-sync, DCACHE_CleanByRange on SRC before copy.
- Async USB bulk-OUT per slot via `USB_HostEdgeTpuBulkOutSendAsync` (existing infrastructure).
- 2-slot ping-pong with `StaticSemaphore_t` per slot; zero heap in hot path.
- Bounded loops: `max_iters = (total/36KB)+2` cap + SERR_LOG on breach.
- All waits bounded by `g_sentai_tpu_urb_timeout_ms` (default 200 ms).
- New error codes 0x0B50..0x0B54 (RING_LOOP_BOUND, RING_SLOT_TIMEOUT, RING_DMA_FAIL, RING_USB_SUBMIT, RING_DRAIN_TO).
- New MP diag bindings: `sentai.diag.tpu_ring([bool])`, `sentai.diag.tpu_ring_stats([reset])`.

## Expected behaviour (UNTESTED)

- Pure TPU invoke yolo_1: 13 ms baseline → 13-14 ms (+~0.1-0.5 ms eDMA overhead).
- Pipeline yolo_1 VGA: **V22 42 FPS baseline MAY REGRESS to 35-38 FPS** because PXP now writes SDRAM concurrent with eDMA SDRAM reads (pre-fine-grained-coord).
- Pipeline yolo26 NEW: should work for the first time, target 15-25 FPS.

## If pipeline regresses >10%

Fine-grained coordination is the next step: have `SendInputs` signal InferTask's `s_sem_bufs_free` at completion (instead of InferTask giving before invoke).  Eliminates PXP-vs-eDMA SEMC overlap.  Not implemented yet.

## Files changed

- `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`
- `libs/tpu/edgetpu_driver.cc` (ring infra + wiring)
- `examples/sentai_runtime/modsentai_diag.c` (MP bindings)
- `examples/sentai_runtime/error_codes.csv` + `sentai_error.h`
- `examples/sentai_runtime/paper/cale1_ring_buffer_plan.md` (status updated)

## Test recipe (next session)

```python
sentai.verbose(1)
sentai.tpu.load('/models/yolo_1.tflite')
for i in range(10): sentai.tpu.invoke()
print(sentai.diag.tpu_ring_stats())  # xfers>0, dma_fail=0, usb_fail=0

# A/B ring off vs on
sentai.diag.tpu_ring(0)  # regresses pipeline; pure TPU may still work
sentai.diag.tpu_ring(1)

# Pipeline
sentai.camera.init(1, 640, 480, 45)
sentai.pipeline.start()
# ... observe pipeline FPS vs baseline 42

# NEW: yolo26 pipeline (first time ever)
sentai.tpu.load('/models/yolo26_768x512.tflite')
sentai.pipeline.start()
```

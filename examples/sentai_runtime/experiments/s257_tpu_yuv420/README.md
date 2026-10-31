# EXP-s257 — Edge TPU inference on the YUV420 dual-input detectors

**WBS:** OP-S10-W24-T3.  **Board:** rev 1.1 unit 58eb, build 1694 (RAM-loaded,
same image persisted in flash), both OV5640 at VGA 45 fps in native YUV
(`camera.yuv_mode(1)` before `init`).  **Date:** 2026-10-31.

## Question

Do the YUV420 Edge TPU models (two inputs: UV 320×240×2 + Y 640×480×1, the s240
lineage) run on the current firmware straight from the camera's native YUV
frames, and how does the invoke time compare with the RGB (921 KB tensor) models
measured in s236?

## Models

Taken from disk (`/home/bogdan/work/ch569w/experiments/exp021_yolov5_640_board/models/`,
the Aug-20 / Sep-16 exports; the MLflow server copies were not fetched — its TLS
certificate does not verify from this host, operator to list/download if newer
ones exist):

| model | bytes | inputs |
|---|--:|---|
| msblock_yuv420_int8_edgetpu | 213 856 | in[0] uint8[1,240,320,2] UV, in[1] uint8[1,480,640,1] Y |
| c3_yuv420_int8_edgetpu | 258 912 | same |
| c2f_yuv420_int8_edgetpu | 271 200 | same |
| c2f_pan2_yuv420_int8_edgetpu | 344 928 | same |
| gelan_yuv420_int8_edgetpu | 361 312 | same |
| c2f_thick_yuv420_int8_edgetpu | 951 136 | same |

Pushed once over HTTP (`/api/write/models/<name>`, 1 MB cap — all fit).  The
loader prints `Inputs: 2` and sets the dual-input path automatically; arena
485 KB / 8 MB for every model.

## Method

`run_yuv420.py [--no-upload] --seconds 12 --fps 45`: one boot, `yuv_mode(1)`,
`camera.init(1, 45)`, then per model `tpu.load` → `pipeline.infer_reset` →
`pipeline.start(0.5)` → 12 s → `pipeline.stats()` + `pipeline.infer_stats()` →
`pipeline.stop`.  Pass criteria: every model loads, `fail == 0`, pipeline rate
within 15 % of the camera rate, no CSI / phase events in dmesg.

## Results (pass 2, `results_b1694_r2.csv`, `run_b1694_r2.log`)

| model | invoke ms (avg) | pipeline fps | frames / 12 s | fail | s236 RGB invoke ms |
|---|--:|--:|--:|--:|--:|
| msblock_yuv420 | 20.0 | 41.5 | 536 | 0 | – |
| c3_yuv420 | 20.0 | 41.4 | 534 | 0 | – |
| c2f_yuv420 | 20.5 | 41.2 | 532 | 0 | 38.1 |
| c2f_pan2_yuv420 | 21.5 | 39.9 | 515 | 0 | 40.2 |
| gelan_yuv420 | 22.0 | 39.6 | 511 | 0 | – |
| **c2f_thick_yuv420** | **23.0** | **39.4** | 509 | 0 | **FAIL (0B62)** |

dmesg after the sweep: only the camera init events (0A06, 0A12 pre-trim, 0A0C
genlock 28 µs).  No stalls, no CSI faults, no phase drift.

- Invoke time is 20–23 ms for every architecture: the pipeline is bound by the
  USB transfer of the 460 KB YUV420 pair, not by the network (s236 found the same
  for RGB at 921 KB: 25.6 ms of input transfer alone).  Halving the tensor halves
  the invoke: c2f 38.1 → 20.5 ms, c2f_pan2 40.2 → 21.5 ms.
- The thesis champion `c2f_thick` **runs** in YUV420 (23 ms, 39 fps) — on RGB it
  fails to invoke on the single-EP Edge TPU firmware (s236, 0B62).  The YUV420
  export is therefore the deployable form of the champion.
- Pipeline rate 39–41.5 fps against a 45 fps camera: the remaining gap is the
  PXP repack + prep hand-off, consistent with s241 (YUV 40 fps).
- `detections_total` = 0 in all runs: the lab scene has no training-class
  objects in view; this experiment measures the data path, not accuracy
  (accuracy lives in s241 / the paper's mAP tables).

## Pass 1 anomaly → OP-S10-W24-T6 (reproduced, root-caused, fixed; builds 1695–1702)

Pass 1 uploaded the six models over HTTP/NCM in the same boot right before the
sweep.  msblock and c3 ran at 43 fps, the third model (c2f) ran at only 12 fps
(186 frames / 15 s, invoke still 20.6 ms), and after the fourth `tpu.load`
(c2f_pan2, "Model loaded" printed) the MicroPython REPL went silent while HTTP
kept answering.  Pass 2 (fresh boot, no upload) was clean.

### Pass 3 — reproduction (`run_repro1.log`, build 1694)

Same sequence (upload, then sweep).  Model 3 (c2f) logged one `E:0B62`
(SendInstructions URB timeout, invoke rc −2); on model 4 another `0B62` and the
board **reset** (new USB device number, CDC-ACM I/O error on the host).  The
board had written a crash record before resetting (`/log/crash_prev.txt`):

```
*** CRASH: code=0x0FF4 PC=0x0000EA8A LR=0x0000EA77 CFSR=0x00008200 BFAR=0x00604F90
  r0=0x00604F90 r1=0x00000B57 r2=0x2002ACFC r3=0x00000000   PRECISERR
```

`PC` is inside `caam_in_job_ring_add` (CAAM crypto driver, never used by
SentAI) with a garbage base pointer in r0: a wild jump from a corrupted stack.
The J-Link could not be attached (SWD connect times out on this board, W24-T8),
so the analysis is from the crash record and the code.

### Root cause 1 — URB completion metadata on the caller's stack

The synchronous TPU transfer paths (`BulkOutTransferInternal`,
`BulkOutTransferStaged`, `BulkInTransferInternal`, `ReadEvent` in
`libs/tpu/edgetpu_driver.cc`) kept `UsbTransferMetadata` on the stack and passed
its address to the EHCI completion callback.  On a URB timeout the code
deliberately does **not** cancel the URB (cancelling a partial bulk-OUT corrupts
the TPU — documented, confirmed), so the transfer completes **later**, after the
caller returned, and the callback wrote bytes/status/timestamps and gave a
semaphore through a pointer into a dead stack frame — i.e. into whatever the
task had put there since.  Under a heavy consumer (JPEG encode, NCM upload,
camera) that frame belongs to live code → wild return → HardFault.

**Fix (1695):** static per-pipe `UrbSlot` (bulk OUT/IN share one slot — they are
issued strictly sequentially under `EdgeTpuManager::mutex_`; the event endpoint
has its own) and an opaque **sequence token** passed to the callback instead of a
pointer.  A completion whose token ≠ the slot's current sequence is an orphan: it
is counted (`g_sentai_tpu_stale_completions`) and ignored.  Call-site order is
arm (seq++) → drain the binary semaphore → submit, so an orphan completing in
the window can neither match nor leave a pending give.

### Root cause 2 — heap semaphore freed while the control URB is in flight

Found by fault injection on 1701 (URB timeout forced to 1 ms via `sys.poke32`,
160 KB chunks): the board reset with a `configASSERT` breadcrumb that the new
assert record resolved to
`queue.c: !((pvItemToQueue == NULL) && (pxQueue->uxItemSize != 0))`, i.e. an
`xSemaphoreGive` on a handle that was no longer a semaphore.  `CSRTransfer`
created a heap binary semaphore per CSR access and `vSemaphoreDelete`d it on its
2 s timeout while the control transfer stayed in flight; the late completion
gave the freed handle, whose memory had been reused by a queue.

**Fix (1702):** static control slot + token + a static mutex (CSR access is not
confined to Invoke: `GetTemperature`/`DumpTpuErrorCsrs` run on the MicroPython
task).  Counters `csr_timeouts`, `csr_mutex_timeouts`.

### Also fixed on the way

- `EdgeTpuManager::packages_` (keyed by the custom-op buffer address inside the
  model vector) was never cleared: one `EdgeTpuPackage` leaked per `tpu.load`,
  and a new model whose buffer landed on a reused address was served the OLD
  executable.  `UnregisterPackagesInRange()` is called for the model being
  freed (slot 0 and slots 1..2; per-range, not global, because slot
  interpreters keep raw package pointers — embeded-reviewer finding).
- `configASSERT` / `CHECK` breadcrumbs now carry the failing expression and
  file (`r0` / `BFAR` fields = string addresses, resolve with `gdb x/s` on the
  ELF); `CHECK` is one out-of-line call per site (`sentai_check_fail`), ITCM
  flat.
- `sentai.tpu.urb_sync_stats()` exposes the synchronous-URB counters.

### Verification

| run | build | sequence | result |
|---|---|---|---|
| repro2 | 1695 | upload + 6-model sweep | 6/6 clean, 0 fails, 0 timeouts |
| repro3 | 1696 | upload + sweep | 6/6 clean; 81 240 URBs, 0 `take_failed`, 0 stale |
| flood A | 1701 | 1 ms URB timeout, 160 KB chunks | reset: configASSERT queue.c (root cause 2) |
| flood B | 1702 | same | **board alive**: 2 CSR timeouts, 13 bulk timeouts, frames dropped; TPU wedged (every invoke 0B62) until `sentai.tpu.recover()` → "invokes restored without reflash", then 472 frames / 10 s, 0 fails |
| repro4 | 1702 | upload + sweep, **same boot** that survived flood B | 6/6 clean, 0 fails (20.0–23.0 ms, 38–41 fps); counters unchanged from the flood (13 / 2), 99 116 URBs total |

Residuals: (a) after orphaned transfers the TPU itself needs `tpu.recover()`
(PMIC power-cycle + reload); the pipeline does not call it automatically yet
(W24-T7); (b) `g_sentai_tpu_stale_completions` stayed 0 in the floods because
`usb_host_edgetpu.c` already drops a completion whose transfer is no longer the
pipe's active one — the token is the backstop for the arm→submit window and
cross-pipe orphans; (c) J-Link SWD attach does not connect on this board (W24-T8).

## Files

- `run_yuv420.py` — driver (upload + sweep; `--no-upload` to reuse the SD copies).
- `results_b1694_r2.csv`, `run_b1694_r2.log` — pass 2 (the result above).
- `run_b1694.log` — pass 1 (REPL silence after the 4th load).

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

### W24-T7 — in-pipeline auto-recovery (build 1703)

InferTask now recovers the TPU itself: after `after_k` consecutive invoke
failures (default 5, ≈125 ms at 40 fps) it logs `0x0421`, backs off
250 ms × attempt, power-cycles the rail and reloads the model
(`sentai_tpu_recover_in_pipeline`, the only caller allowed to reload while the
pipeline runs — PrepTask is blocked on the semaphores InferTask stops giving).
At most `max_attempts` per run (default 3); when exhausted it logs `0x0423`,
stops the pipeline (safe state) and reports the TPU missing to health (LED
code 5).  `sentai.pipeline.auto_recover([k[, n]])`, counters in
`pipeline.infer_stats()` (`recover_attempts/ok/fail/giveup`).

| scenario | build | outcome |
|---|---|---|
| 1.5 s URB-timeout flood while running | 1703, 1705 | 5 fails → 1 attempt → recovered; run continued at full rate, 0 further fails |
| sustained flood | 1703 | 3 attempts (reload OK each time, invokes still failing under the flood) → give-up `0x0423:5`, `running()` False, `sys.health()['tpu']` False; after the flood `tpu.recover()` + `start()` → 348 frames / 8 s, 0 fails |
| sustained flood | **1704** | **hang**: attempt 1 never returned — `EdgeTpuManager::OpenDevice()` waited for the USB instance in an *unbounded* loop while holding the manager mutex (the TPU did not re-enumerate under the flood, DFU included); a REPL `tpu.recover()` then blocked on that mutex and the REPL died. Pre-existing bounded-wait violation, exposed by the automatic path. |
| sustained flood | 1705 | bounded path: 3 attempts, give-up at t = 7 s, REPL answered in 0.3 s throughout, health `tpu` False + LED code 5; flood off → `tpu.recover()` 1.8 s → health `tpu` True, LED code cleared; transient flood afterwards self-healed again |

Bounds added in 1705: `OpenDevice` waits ≤ 3 s for the USB instance (was
∞), the loader retries it 4× (≤ ~16 s, SERR `0x0B75` on give-up, was 15×∞),
`EdgeTpuTask::SetPower` waits ≤ 200 ms for PGOOD (was a bare busy-loop).
The 1705 run's attempts completed quickly, so the new bounds were not hit in
this session; they exist so the failure mode of 1704 degrades into a logged,
bounded failed attempt instead of a hang.

Residuals: (a) `recover_ok` counts successful *reloads*, not restored invokes
— under a persistent fault all three attempts "succeed" and the give-up
follows; (b) `g_sentai_tpu_stale_completions` stayed 0 in the floods because
`usb_host_edgetpu.c` already drops a completion whose transfer is no longer the
pipe's active one — the token is the backstop for the arm→submit window and
cross-pipe orphans; (c) J-Link SWD attach does not connect on this board (W24-T8).

## Camera 30 fps vs 45 fps, and the USB 2.0 budget (build 1705, flash)

Question raised 2026-10-31: does anything in the TPU path stall at a 45 fps
camera, and how much USB bandwidth does ~40 fps inference actually use?
Same six models, fresh boot per run (`sentai.sys.reset()`), no upload,
20 s per model (`run_b1705_fps30.*`, `run_b1705_fps45.*`):

| camera | pipeline fps (6 models) | invoke ms avg | frames (6 × 20 s) | invoke fails | URBs | URB timeouts (bulk / CSR) | stale |
|---|---|---|---|---|---|---|---|
| 30 fps (33.3 ms frame) | 29.5 for all six (camera-bound) | 19.0 / 19.0 / 19.0 / 20.0 / 20.0 / 21.1 | 617 each | 0 | 77 184 | 0 / 0 | 0 |
| 45 fps (22.2 ms frame) | 42.3 / 42.1 / 42.0 / 40.6 / 40.2 / 40.1 (invoke-bound) | 20.0 / 20.0 / 20.0 / 21.0 / 21.8 / 23.0 | 884 / 881 / 878 / 848 / 841 / 838 | 0 | 107 650 | 0 / 0 | 0 |

Nothing stalls at 45 fps: zero invoke failures and zero synchronous-URB
timeouts in both runs (and in the five earlier 45 fps sweeps on 1695–1702).
The pass-1 silence / reset at 45 fps was the W24-T6 crash class plus the
package leak, both fixed; it was never a bandwidth limit.  The only effect of
the camera rate on the TPU is ~1 ms extra per invoke at 45 fps (20.0 vs
19.0 ms), i.e. SDRAM/bus contention from the CSI + PXP traffic, not USB.

**Per-invoke USB traffic (dual-input YUV420 model):**

| item | bytes | notes |
|---|---|---|
| Y plane 640×480×1 | 307 200 | bulk OUT, 36 KB chunks (default `tpu.chunk_size`) |
| UV plane 320×240×2 | 153 600 | bulk OUT |
| input total | **460 800** | 13 bulk-OUT URBs |
| instructions + parameters | model-dependent, tens of KB | instruction stream is re-sent per invoke (parameter caching keeps weights on the TPU) |
| outputs (detection heads) + event | a few KB | bulk IN + interrupt |
| URBs per invoke | ≈ 21 | 107 650 URBs / 5 170 invokes |

Bus arithmetic (USB 2.0 high-speed bulk: 13 × 512 B per 125 µs microframe =
53.2 MB/s theoretical; the RT1176 EHCI path sustains ≈ 35–40 MB/s in practice,
measured in s240 from the RGB-vs-YUV invoke delta: 460.8 KB more input cost
~13 ms):

- 460.8 KB of input per invoke ≈ **12–13 ms of bus time** out of the 20–23 ms
  invoke → the invoke is **transfer-bound**, with ~7–10 ms left for
  instruction upload, TPU compute, output read and the completion event.
- At 42 fps the input alone is 460.8 KB × 42 ≈ **19.4 MB/s ≈ 155 Mbit/s**,
  i.e. roughly half of the practical bulk capacity and ~36 % of the
  480 Mbit/s signalling rate.  The bus is busy ~55–60 % of the time (the
  transfer phase of each invoke); it is not oversubscribed.
- The RGB (3-byte) models move 921.6 KB per invoke → 35 ms → 28 fps max,
  which is why the YUV420 export is the deployable form: halving the input
  bytes is what buys the 40 fps, not the compute.

So 40 fps is real and within USB 2.0, but USB is the bottleneck of the
invoke: a faster TPU would not help, a smaller input (or on-TPU
decimation) would.

## Files

- `run_yuv420.py` — driver (upload + sweep; `--no-upload` to reuse the SD copies).
- `run_b1705_fps30.{log,stdout}`, `results_b1705_fps30.csv`, `run_b1705_fps45.{log,stdout}`, `results_b1705_fps45.csv` — the 30 vs 45 fps comparison above.
- `results_b1694_r2.csv`, `run_b1694_r2.log` — pass 2 (the result above).
- `run_b1694.log` — pass 1 (REPL silence after the 4th load).

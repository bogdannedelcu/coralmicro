# experiment.md — current state and open work

Snapshot date: 2026-04-22 (late)
Branch: feature/ov5640-camera-support
Device build: **#762** (multi_ep firmware=OFF, zero-copy bulk path shipped)

## Two independent workstreams

### A.  OV5640 brightness (still pending build)
Files on disk, NOT flashed:
- `examples/sentai_runtime/flow_task.cc` — `gray_stretch` auto-level
- `examples/sentai_runtime/modsentai_flow.c` — MP binding
- `examples/sentai_runtime/sentai_slow_bridge.cc` — TPU OpenDevice retry
- `examples/sentai_runtime/diag/drivers/_e39_gray_stretch_sweep.py`

### B.  TPU USB throughput — V13 SHIPPED

## Breakdown (measured via `sentai.diag.tpu_perf()`, DWT cycles @ 800 MHz)

For a 512×512 uint8 YOLO invoke (50 samples, pure TPU, no camera contention):

| stage | cycles/invoke | ms/invoke | bytes/invoke | notes |
|-------|---------------|-----------|--------------|-------|
| params | ~76 k | 0.1 | 2.75 KB | cached on TPU, small residual |
| instructions | ~3.7 M | 4.6 | 371 KB | 2 sends/invoke |
| input | ~7.8 M | 9.7 | 811 KB | 2 sends/invoke, 512×512×3 |
| output | ~0.5 M | 0.6 | 10.75 KB | 1 read |
| event | ~20 k | 0.025 | 16 B (IRQ status) | 1 per invoke |
| **USB I/O total** | | **~15 ms** | ~1.2 MB | |
| **measured total** | | **31 ms** | | TPU compute = ~16 ms |

**Key insight:** TPU compute floor is ~16 ms for this model.  Any
remaining optimization has to attack USB I/O, which is already down
to ~15 ms — most of it is the 811 KB input.

## 🔥 V17 BREAKTHROUGH (2026-04-22 late)

**From 31 ms/invoke (32 FPS) to 13.65 ms/invoke (73 FPS) = 2.27× speedup.**

Via a chunk-size sweep `sentai.diag.tpu_chunk_size(n)` with n in 4-160
KB, found a **sharp performance cliff between 36 KB and 38 KB**:

| chunk KB | avg ms/invoke | FPS |
|---------:|--------------:|----:|
| 16 | 13.9 | 72 |
| 20 | 14.8 | 68 |
| 24 | 14.9 | 67 |
| 28 | 14.2 | 70 |
| 32 | 14.7 | 68 |
| **33** | **12.9 ✨** | **77** |
| 34 | 14.5 | 69 |
| 36 | 14.3 | 70 |
| **38** | **37.97 ⛔** | 26 |
| 40 | 37.6 | 27 |
| 48 | 34.9 | 29 |
| 56 | 32.9 | 30 |
| 64 | 30.7 | 33 |
| 96 | 34.9 | 29 |
| 128 | 37.2 | 27 |

**Hypothesis**: The EdgeTPU's bulk-OUT receive FIFO is ~32-36 KB.
URBs that fit inside stream into the TPU's inference pipe without
back-pressure, allowing the TPU to START COMPUTING on chunk N-1
while USB feeds chunk N.  URBs over ~36 KB overflow the FIFO,
stall the USB, serialize feed-then-compute.  This **demolishes the
earlier "16 ms TPU compute floor" theory** — compute overlaps with
feed.  What we saw as "16 ms" was compute + stall on 64 KB chunks.

At 33 KB chunks, total invoke ≈ MAX(usb_feed, tpu_compute), not
sum.  Both happen to be ~13-14 ms for this model.  That matches
the 72-77 FPS we now measure cleanly across 100-sample batches.

**Default changed to 33 KB** in edgetpu_driver.cc.  Stability
confirmed across 3 × 100-invoke runs (13.63, 13.65, 13.79 ms).

## V-series progression

| ver | change | avg ms | I/O ms | notes |
|-----|--------|--------|--------|-------|
| V0 | baseline (build #737) | 32 | 16-17 | 32 KB staging in DTCM, sem create/delete |
| V7 | persistent sema + bytes-fix | 32 | 16 | same perf, correctness win |
| V9 | zero-copy + ~64 KB chunks | **30** | 8-10 | skip SDRAM→DTCM memcpy; USB_HostSend does cache clean |
| V10 | 128 KB chunks | 36 | — | REGRESSED — NXP EHCI QTD chain overhead |
| V11 | 64 KB chunks (aligned) | 31 | 10 | stable |
| V12 | multi_ep firmware + EP routing | 80 | fail | TPU DFU'd multi_ep bin, host didn't open EP2/EP3 pipes |
| V14 | + async-input toggle (2 URBs in flight on input pipe) | 31.23 | 9.0 | works: 462/462 callbacks OK, ~0.4 ms saved, mostly hidden by compute floor |
| V15 | + desc_cache infra (skip params+ins on token match) | FAIL | — | model needs instructions EVERY invoke — TPU hangs when skipped |
| **device** (build final) | zero-copy + 64 KB + async-input + desc_cache-OFF | **30.75** | **9-10** | shipping — 100-sample avg |

## Async path findings

Tried via `sentai.diag.tpu_async_input(1)`: restructures SendInputs
to use `USB_HostEdgeTpuBulkOutSendAsync` (per-transfer callback pool
bypasses the NXP pipe->callbackFn collision), submits chunk [nxt]
BEFORE waiting on chunk [cur] so two URBs sit in the EHCI schedule
simultaneously.  Result: `async_stats` shows **462/462 callbacks
fired successfully, 0 submit fails, 0 cb fails**.  The path works.

But gain is only ~0.4 ms/invoke because **TPU compute is the floor**
— at ~16 ms silicon compute, shaving USB I/O from 10 → 9 ms only
shows up if it falls below the compute time.  Pipelining wins
compound only when we can ALSO start the NEXT invoke's input upload
while the CURRENT invoke's compute runs (application-level double
buffer).

## Why desktop libedgetpu can look "10× faster"

The comparison is misleading when the target model differs.  On the
SAME TPU silicon, our 512×512 YOLO takes ~16 ms of COMPUTE — that's
hardware, not software.  Smaller models (MobileNetV2 224×224) take
~2 ms compute, so USB overhead dominates and desktop's 3-concurrent
+ 1 MB chunks get close to 4 ms total.

For OUR model specifically:
- Desktop libedgetpu: ~22-25 ms/invoke (16 ms compute + 5-8 ms
  optimal-USB + native dispatch)
- Our current: 30.75 ms/invoke (16 ms compute + 9-10 ms USB +
  2-3 ms MicroPython/interpreter dispatch)
- Gap to desktop: ~5-8 ms, not 25 ms

Remaining 5-8 ms gap levers (all require significant work):
1. multi_ep firmware + TRUE 3-pipe concurrency (V12 failed DFU
   bootstrap)
2. EHCI-QH-chain keeping the pipe continuously fed across Send*
   boundaries (eliminate the ~1 ms gap between last input URB IOC
   and first instruction URB submit)
3. Reduce MicroPython call overhead (2-3 ms per `invoke()` — replace
   with a native C helper that runs the full 30-invoke loop)

## What WORKED (shipped in #762)

1. **Zero-copy bulk transfers.**  `BulkOutTransfer` /
   `BulkInTransfer` in [libs/tpu/edgetpu_driver.cc](../../../libs/tpu/edgetpu_driver.cc)
   now submit DIRECTLY from the caller's source pointer (tensor
   arena / flatbuffer / output tensor) instead of staging through a
   32 KB DTCM buffer.  `USB_HostSend` handles cache coherency via
   its own `DCACHE_CleanByRange` / `DCACHE_CleanInvalidateByRange`
   calls (see usb_host_hci.c:413 / :501).  **~50% cut in USB I/O
   time per invoke.**

2. **64 KB chunks.**  Old code was capped at 32 KB by the staging
   buffer AND by the `uint16_t length` arg on
   `USB_HostEdgeTpuBulkOutSend`.  Widened that arg to `uint32_t`
   and picked 64 KB as the sweet spot (128 KB regressed — NXP
   EHCI QTD-chain setup overhead dominates the per-chunk save).

3. **Persistent semaphore** (`InitBulkSema` in edgetpu_driver.cc).
   Skip `xSemaphoreCreateBinary` / `vSemaphoreDelete` per chunk.
   Negligible timing win (sub-ms), but a stability win: no heap
   churn in the TPU hot path.

4. **BulkInTransfer memcpy bytes fix.**  Latent bug: old code
   memcpy'd `chunk_size` regardless of short-read; a short read
   would leak stale staging bytes past the real payload.  Now uses
   `bytes_received`.

5. **Per-stage DWT instrumentation** exposed as
   `sentai.diag.tpu_perf([reset]) -> {cyc_*, n_*, by_*}`.  Also
   `sentai.diag.tpu_multi_ep(enable)` for routing toggle.

## What DIDN'T work (and why)

1. **Explicit `.ocram_bss → m_ocram` mapping.**  V2/V3 tried moving
   the staging buffer from default DTCM (m_data, single-cycle,
   uncached) to OCRAM (cached, AXI path).  Regressed 3 ms/invoke.
   Reverted.  The original code's comment claiming OCRAM placement
   was wrong — the orphan `.ocram_bss,"aw",%nobits @` attribute
   landed in m_data by default (linker orphan rules), which was
   actually the fast path.

2. **True async pipelining (V4).**  Added per-transfer callback
   pool in usb_host_edgetpu.c so multiple URBs could be outstanding
   on the same pipe with their own `{user_cb, user_param}` routing.
   Compiled and submitted cleanly but callbacks never fired — the
   NXP EHCI has some pipe-level coupling we haven't traced.
   Infrastructure sits in the file (`USB_HostEdgeTpuBulkOut
   SendAsync`, `EdgeTpuPerXferDispatch`, `s_async_ctx_pool`) for a
   future debug pass.

3. **Multi_ep firmware (V12).**  CMake flag `SENTAI_TPU_MULTI_EP
   =ON` flips the apex_firmware blob to `apex_latest_multi_ep_bin`.
   Host toggle `sentai.diag.tpu_multi_ep(1)` flips runtime routing.
   But first invoke fails with `USB_HostEdgeTpuBulkOutSend failed`
   — EP2 / EP3 pipes aren't opened by `USB_HostEdgeTpuOpenData
   Interface` when the new firmware enumerates.  Needs investigation:
   either the TPU descriptor needs `multi_bo_ep=1` set BEFORE
   enumeration (chicken-and-egg), or our class driver needs to
   explicitly open additional bulk endpoints from the descriptor.

## Remaining architectural levers (NOT yet exploited)

| lever | potential gain | effort |
|-------|---------------|--------|
| multi_ep + per-pipe async | ~2× I/O throughput | DFU descriptor boot sequence fix |
| NXP EHCI MAX_QTD > 8 | allow 256+ KB URBs | test carefully, V10 showed DCACHE time dominates |
| TPU model with smaller input (e.g. 320×320) | 2.5× less input data | retrain model |
| Overlap next-invoke prep with current TPU compute | saves camera→tensor-arena ~14 ms | app-level double buffer |

## Files modified in build #762

- `libs/tpu/edgetpu_driver.cc` — zero-copy, 64 KB chunks, persistent
  sema, BulkIn bytes fix
- `libs/tpu/usb_host_edgetpu.c` — async API infrastructure (not wired
  into hot path yet), widened BulkOutSend length to uint32_t
- `libs/tpu/usb_host_edgetpu.h` — async API declarations, signature
  update
- `libs/tpu/edgetpu_executable.cc` — DWT cycle counters for per-stage
  breakdown
- `examples/sentai_runtime/modsentai_diag.c` — `sentai.diag.tpu_perf()`
  and `sentai.diag.tpu_multi_ep()` bindings

## How to pick up next time

1. **If chasing more TPU speed:** investigate V12 failure path — why
   `USB_HostEdgeTpuGetPipeIndexFromEndpoint(..., EP2_OUT)` returns
   -1 after multi_ep firmware DFU.  Read
   `USB_HostEdgeTpuOpenDataInterface` in `libs/tpu/usb_host_edgetpu.c`
   to see which endpoints it actually opens.  Likely fix: re-enumerate
   OR set multi_bo_ep=1 via control transfer BEFORE calling
   OpenInterface.
2. **If chasing the V4 async path:** the per-transfer callback pool
   (`EdgeTpuPerXferDispatch` in usb_host_edgetpu.c) dispatches via
   `transfer->callbackFn` which EHCI calls at completion (line 3671
   / 3714 of usb_host_ehci.c).  The bug is somewhere between submit
   and the EHCI completing.  Start with a LOG printf at IOC time to
   see if EHCI even sees completion.
3. **If benchmarking the new shipped V13:** run
   `examples/sentai_runtime/diag/drivers/_e20_tpu_raw.py` and
   `sentai.diag.tpu_perf(True)` then sample at end to see per-stage
   split.

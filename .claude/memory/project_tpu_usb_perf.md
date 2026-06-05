---
name: TPU USB throughput — ReadEvent static fix partially unblocks pipeline
description: 2026-04-22 session. Static ReadEvent buf+sema unblocks short pipeline runs (T3 passes!). Sustained 10s runs still corrupt. Next: find what accumulates.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
**Update 2026-04-22 (session-end)**: Partial pipeline unblock via
heap-free `ReadEvent` + static task allocation + equal priority 2.

**What shipped this session:**
1. Static task allocation (xTaskCreateStatic) for det_prep and det_infer
   in `examples/sentai_runtime/detection_task.cc`. Stacks in
   `.sdram_bss`, TCBs static. Heap wasn't the culprit but the change
   is a net win (no alloc churn on start/stop).
2. Equal priority 2 for both tasks — InferTask was prio 3 preempting
   PrepTask prio 2. Equal prio 2 removes preemption; tasks yield
   via explicit sem blocks. Safe because `configUSE_TIME_SLICING=0`
   so no round-robin starvation concerns.
3. **ReadEvent static buf+sema** in
   `libs/tpu/edgetpu_driver.cc:913`. Old code did
   `OSA_MemoryAllocate(16)` + `xSemaphoreCreateBinary()` per Invoke
   (SDRAM heap). New: `static uint8_t s_event_buf[16]` in
   `.sdram_bss` + persistent `xSemaphoreCreateBinaryStatic`.
   Eliminates 2 heap ops per invoke.

**Test results (yolo26n 320×320, build current)**:
- T1 (pure TPU, no pipeline): 0 fails, 102 ms/invoke avg
- T3 (pipeline.start 300ms + stop 300ms + invoke): **0 FAILS, 103 ms
  avg** — was 100% fail before ReadEvent fix. **This is the big win.**
- T4 (pipeline RUNNING, external invoke): returns -10 (by design, pipeline
  owns TPU). PrepTask processed 13 frames in 500ms = 26 FPS prep rate.

**What still breaks (sustained 10s pipeline)**:
`_t_e2e.py` runs pipeline for 10 seconds. Result:
- PrepTask: 372 frames in 9 s = **41 FPS prep rate** (good).
- InferTask: fails EVERY invoke ("Node edgetpu-custom-op failed with
  status 1"). `sem_wait_ms_sum / frames = 2.2 ms` → PrepTask barely
  waits for InferTask → InferTask is failing fast from frame 1.
- Post-pipeline pure TPU: ALSO broken (needs reflash).

**Key insight**: In the 300ms T3 run, InferTask processed 4 frames
successfully (SERR_DET_STOPPED val=4). In 10s sustained, InferTask
fails every invoke. So sustained-parallel operation wedges the TPU,
but a brief overlap survives. The failure mode ISN'T pure heap
fragmentation — ReadEvent already cleaned that. Something about
sustained PrepTask activity (cam_grab + PXP + quant on SDRAM) crushes
the USB DMA bus bandwidth budget.

**Next-session priorities** (ranked by expected impact):
1. **Move EHCI QH/QTD to m_ncache (DTCM)** — the documented next
   step from prior session. NXP `OSA_MemoryAllocate` puts descriptors
   in SDRAM (main heap). Under PXP+camera+quant SDRAM load, bus
   arbitration stalls hit descriptor reads → EHCI misses IOC timing.
   Patch `usb_host_ehci.c:4200-4275` (ehciQhList alloc) or linker
   section. Expected: zero PXP contention.
2. **Throttle PrepTask** — either vTaskDelay between frames or
   capped FPS via `sentai.pipeline.prep_fps(n)`. Reduces SDRAM
   bus pressure; trades InferTask latency for stability.
3. **TPU soft-reset on stop** — currently only reset at boot in
   `Initialize()`. Add `TpuDriver::SoftReset()` that re-runs the
   `SCU_CTRL_3.rg_force_sleep = 3 + 2` dance. Call on
   `sentai_detection_stop()` to recover without reflash. This
   doesn't fix the wedging but recovers cleanly.

**Diag infrastructure (unchanged from prior session)**:
Runtime toggles via `sentai.diag.tpu_chunk_size`,
`tpu_urb_timeout`, `tpu_zero_copy`, `tpu_async_input`,
`tpu_multi_ep`, `tpu_desc_cache`. `async_stats()` 19-key dict for
per-URB counters.

**Prior-session shipped (still valid)**: USB→OCRAM linker move,
CSI IRQ priority 5, URB timeout 200ms, no-cancel timeout path,
zero-copy bulk, 33 KB chunk sweet spot (for yolo512; yolo26n may
prefer different size — not swept this session).

⚠️ **Test file naming**: `examples/sentai_runtime/diag/_t_prio2.py`
(T1+T3+T4 driver) and `_t_e2e.py` (sustained 10s) are temp test
drivers. `_host_upload_repl.py` auto-uploads all non-`_host_*`
files — they WILL get pushed on any full upload. Either clean up
or move to `.experiments/` before routine uploads.

**How to apply**: Before recommending "the pipeline works",
remember that only SHORT (<1s) pipeline bursts survive on current
build. Sustained detection pipeline (>2s) still wedges TPU until
reflash. Either keep pipeline short or land next-session's EHCI
descriptor migration.

═══════════════════════════════════════════════════════════════════

**Update 2026-04-22 night-end-v2 (USB→OCRAM + CSI prio fix)**: shipped
three infrastructure changes that should help pipeline integration
but don't fully fix it:
1. NXP USB host code moved from SDRAM to OCRAM (via linker script)
   — eliminates M7 instruction-fetch contention with PXP under
   camera load.  Freed OCRAM by moving `.audio` + `.shine` (unused
   in TPU pipeline) to SDRAM.
2. `NVIC_SetPriority(CSI_IRQn, 5)` in `BOARD_InitCamera` — CSI was
   at default priority 0 (highest), preempting USB_OTG2 at 2.
   Priority 5 puts camera below USB syscall boundary.  Safe: CSI
   ISR does NOT call FreeRTOS APIs.
3. URB timeout 50→200 ms — BulkIn URBs need longer budget because
   they wait for TPU compute (~16 ms) before data arrives.

**Pipeline STILL fails.**  Even `direct_tensor(0)` legacy staging
path fails identically.  Diagnosed: cancel-on-timeout recovers the
HOST side cleanly, but subsequent standalone invoke also fails →
the TPU SILICON state is getting corrupted by partial/cancelled
transfers.  Only full firmware reflash recovers.

**Real next-session path**: TPU-level reset after cancel.
`libs/tpu/edgetpu_driver.cc:Initialize()` does reset at boot via
`SCU_CTRL_3.rg_force_sleep = 3` + wait.  A "soft reset" helper that
re-runs the reset dance after a timeout would unwedge the device.
See `DoRunControl` in edgetpu_driver.cc for existing reset primitives.

Or: on pipeline.start, detect TPU not-ready and re-DFU via
`EdgeTpuManager::GetSingleton()->OpenDevice()`.

**Update 2026-04-22 night-end**: shipped fault-tolerant USB path.

Cancel-on-timeout: `USB_HostEdgeTpuCancelInFlight()` in
libs/tpu/usb_host_edgetpu.c cancels any in-flight URB and the cancel
callback fires on same stack meta (lambda writes + gives sema). Runtime
timeout via `sentai.diag.tpu_urb_timeout(ms)` default 50 ms.  When
TPU exceeds this cap (running avg ~15 ms, so 3× margin), caller gets
-1 and skips the frame.  Cascade eliminated.

No-heap hot path: PrepareHeader replaced with PrepareHeaderInto()
stack variant.  All 8-byte USB headers now on stack, no heap churn
per invoke.

**Pure TPU: 75.9 FPS** stable on device (build current).

Pipeline STILL can't produce frames end-to-end — root cause is a
SDRAM/DMA bus contention between PXP, camera, and EHCI that we
can't fix from the TPU driver side.  Measured: 61% URB success +
39% timeout during pipeline load.  Each URB tests bus arbitration
luck; the underlying hardware arbitration isn't deterministic
enough for our 50 ms window under load.

**Next-session fix paths for pipeline end-to-end**:
1. **Move EHCI QH/QTD descriptors out of SDRAM into m_ncache (DTCM
   non-cacheable)** — NXP allocates via `OSA_MemoryAllocate` from
   main heap (SDRAM).  A targeted patch in usb_host_ehci.c at
   line 4200-4275 could redirect ehciQhList to a dedicated static
   non-cached buffer in DTCM.  Expected: zero contention with PXP.
2. **Demote camera task priority below USB host** — currently
   both at `configMAX_PRIORITIES-1=4`.  Camera at 3 keeps USB at
   uncontested 4 during pipeline.
3. **Architectural split: put TPU driver on M4 core** — RPMSG
   window too small for 800 KB input, but could extend shared
   region.  Major refactor.

Infrastructure preserved for next session (see diag toggles list
in agent/experiment.md V20):
- `tpu_urb_timeout`, `tpu_chunk_size`, `tpu_zero_copy`,
  `tpu_async_input`, `tpu_multi_ep`, `tpu_desc_cache` runtime
  toggles via `sentai.diag.*`
- `sentai.diag.async_stats()` 19-key debug dict

Don't revert 0x0500:NNN "errors" — those are watchdog KICK logs
(0x0500 = SERR_MOD_WDG, val = uptime_sec), not errors.  See
error_codes.csv:0x0500,WDG,KICK.

# experiment.md — current state and open work

Snapshot date: 2026-04-22 (night)
Branch: feature/ov5640-camera-support
Latest device build: V21 (cancel-on-timeout + no-heap hot path)

---

## 📍 TL;DR — where we are, what we did, what's next

### Ce am făcut azi (TPU throughput sprint)

**Scopul**: extragem cât mai multă performanță din EdgeTPU-ul prin USB,
apoi pipeline end-to-end camera → TPU pe VGA 512×512 YOLO.

**Rezultate pe device acum**:
- **Pure TPU standalone: 75.9 FPS** (13.18 ms/invoke), de la 32 FPS
  baseline = **2.37× speedup**.  Măsurat cu E20 diag, 100+ invoke-uri
  consecutive, stabil.
- **Pipeline end-to-end: blocat** — dar modul de eșec e curat (nu mai
  crash-ează board-ul, nu mai cascadă).

**Lista optimizărilor shipped în driver**:
1. **Zero-copy bulk transfer** — submitem direct din tensor_arena /
   flatbuffer, fără memcpy staging (50% tăiere USB I/O)
2. **Chunk size 33 KB** (de la 32 KB default) — sweet spot empiric
   găsit prin sweep 4-160 KB; cliff ascuțit la 36→38 KB (FIFO
   bulk-OUT EdgeTPU e ~32-36 KB)
3. **Persistent semaphore** — o singură sema pentru toate transfer-urile
   TPU (nu create/delete per chunk)
4. **Cancel-on-timeout fault-tolerance** — `USB_HostEdgeTpuCancelInFlight()`
   ridică URB-urile orfane din coada EHCI când sema time-out la 50 ms.
   Elimină cascadă: 1 stuck URB nu mai paralizează toate invoke-urile
   următoare.
5. **Zero heap în hot path** — `PrepareHeaderInto()` static în loc de
   `std::vector<uint8_t>(8)`; toate bufferele USB sunt static allocate
6. **Instrumentare DWT** per etapă (`sentai.diag.tpu_perf()`) pentru
   audit realist al split-ului params/ins/input/output/event
7. **Runtime-tunable toggles** via `sentai.diag.*` pentru A/B test
   rapid fără reflash

### Ce ne propunem acum

**Fix pentru pipeline integration** — relocare structuri EHCI QH/QTD
din SDRAM în DTCM non-cached pentru a elimina contenția bus-ului
între PXP/camera-DMA/EHCI.

**Diagnostic care ne-a adus aici**: contoarele `sentai.diag.async_stats()`
au arătat 39% URB timeout sub load pipeline (61% success), fără
contenție doar TPU. Asta ne-a confirmat ipoteza user-ului: "e o
încurcătură între bufferele PXP, Camera, DMA și USB, se încalecă,
nu sunt sincronizate". NXP-ul alocă `usb_host_ehci_instance_t` (care
include pool-ul de QH/QTD) prin `OSA_MemoryAllocate` — merge în heap
SDRAM. Când PXP-ul scrie SDRAM la 40 MB/s, EHCI-ul pierde uneori
fereastra să-și scrie IOC-ul la timp → IRQ nu se declanșează → URB
pare orfanat → cancel + skip frame.

**Planul concret pentru următoarea mișcare (in-progress acum)**:
1. Identifică alocarea EHCI în `third_party/nxp/rt1176-sdk/middleware/usb/host/usb_host_ehci.c:4200`
   (instance) + 4275 (QH list) + QTD list.
2. Creează un buffer static în `m_ncache` (non-cached SRAM la
   0x20000000, 32 KB disponibil) suficient pentru:
   - `usb_host_ehci_instance_t` (câteva sute de bytes)
   - `MAX_QH × sizeof(usb_host_ehci_qh_t)` = 8 × ~80 B = 640 B
   - `MAX_QTD × sizeof(usb_host_ehci_qtd_t)` = 8 × 32 B = 256 B
   - Total ~1-2 KB pool
3. Patch care redirecționează `OSA_MemoryAllocate` în `USB_HostEhciCreate`
   către acest buffer static (doar pentru EHCI, restul aloc.-urilor
   NXP rămân normale).
4. Flash + test pure TPU (trebuie să rămână 75.9 FPS)
5. Flash + test pipeline end-to-end — așteptare: toate URB-urile
   să completeze în fereastra 50 ms, timeout rate → 0%.

**Riscuri**:
- NXP SDK poate face assumpții despre heap-ul său în alte părți din
  stack; trebuie patch limitat la EHCI
- Buffer-ul static trebuie aliniat corespunzător (EHCI cere 32 B
  aliniere pentru QH, 32 B pentru QTD)
- Dacă `m_ncache` e prea mic, mutăm în m_ocram non-cached region

---

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

## V25 INCREMENTAL ISOLATION — TASK CONTEXT IS THE CULPRIT (2026-04-22 night)

Clean REPL-driven A/B/C test, same firmware, same TPU session.

| Test | Operation | Result | Stats (bo_ok / timeouts / send_fail) |
|------|-----------|-------:|--------------------------------------|
| T1 | `[tpu.invoke() for _ in range(50)]` | 13.82 ms, **0 fails** | 2435 / 0 / 0 |
| T2 | `[(camera.to_tensor(), tpu.invoke()) for _ in range(50)]` | 25.72 ms, **0 fails** | +2150 / 0 / 0 |
| T3 | `pipeline.start(); pipeline.stop(); [tpu.invoke() for _ in range(50)]` | **50 fails** | +784 / +8 / +201 |

Interpretation:
- **T1**: TPU driver alone is bulletproof.  2435 URBs, zero failures.
- **T2**: Camera capture + PXP + quantization + TPU invoke, done
  **serially** from one task.  **Zero failures.**  So PXP-vs-USB
  bus contention, camera cache coherence, SDRAM traffic — NONE of
  these alone breaks TPU.
- **T3**: Briefly launch the pipeline (PrepTask + InferTask created
  → a few frames processed → tasks destroyed), then try pure
  `tpu.invoke` from REPL.  **Every single invoke fails.**  The
  8 new timeouts + 201 new send_fails happened DURING the brief
  pipeline run; those wedged the EHCI pipe state for good.

### Root cause (isolated)

**The bug is not data-related.  It's task-context-related.**

Running PXP + invoke serially = fine.  Running the same operations
**in separate FreeRTOS tasks** = breaks TPU driver.  The specific
difference between T2 and T3 is just which task does the work:
- T2: REPL task (prio 1) does everything sequentially
- T3: PrepTask (prio 2) does PXP, InferTask (prio 3) does invoke,
  both with USB host task (prio 4) handling callbacks

Something about InferTask's invoke context causes some URBs to
time out.  Without cancel, those timeouts wedge the pipe.  Even
after pipeline.stop deletes the tasks, subsequent REPL invokes
see the wedged pipe and also fail.

### Suspects (ranked by likelihood)

1. **`sentai_tpu_invoke_with_input(buf)` pointer swap** — InferTask
   uses this path (direct mode); REPL uses `sentai_tpu_invoke` (no
   swap).  But earlier test with `direct_tensor(0)` (legacy path,
   InferTask memcpys to arena → normal invoke) also failed.
   So probably NOT the swap itself.
2. **FreeRTOS task-switch during USB sema wait** — InferTask at prio
   3 blocks on sema; USB host task at prio 4 wakes and gives sema.
   Maybe the PrepTask at prio 2 runs between that and adds a
   subtle delay through some shared mutex/resource.
3. **Heap fragmentation from task TCB allocation** — `xTaskCreate`
   allocates TCB + stack from FreeRTOS heap.  If that allocation
   touches a memory region adjacent to a USB buffer, cache lines
   could cross.
4. **Some global flag set by `sentai_detection_start`** that
   changes behavior in the USB path (unlikely but worth checking).

### Concrete next-session path

Without changing any existing pipeline code, make InferTask LOOK
exactly like the REPL task:
- Same priority (1)
- Same stack size
- Just calls `sentai_tpu_invoke()` (no pointer swap, no PrepTask
  coordination) — have it read from the already-filled arena

If this works → it's task-priority or concurrency with PrepTask.
If still fails → it's something else in `sentai_detection_start`
that sets up state for InferTask's environment.

## V24 PRINTF FEEDBACK HYPOTHESIS TESTED (2026-04-22 night)

User's hypothesis: printfs in TPU driver failure paths spam USB
CDC-ACM → USB device task competes with USB host task (both at
prio 4) → USB host task starved → callbacks delayed → more
timeouts → more printfs.  Feedback loop.

### Changes (all shipped)
Removed printfs from hot failure paths:
- `WriteHeader failed`
- `BulkOutTransfer failed`
- `USB_HostEdgeTpuBulkOutSend failed`
- `USB_HostEdgeTpuBulkInRecv failed`
- `Bad BulkOutTransferInternal`
- `Bad BulkInTransferInternal`
- Rate-limited `SERR_LOG(SERR_DET_INVOKE_FAIL)` to every 100th
  failure (was every failure)

### Result: only partial
`async_stats` after pipeline test: `bo_ok=370 send_fail=5271 tm=32`

**32 REAL timeouts remain** (unchanged).  So printf spam was NOT
the primary cause of those 32.  But once 32 timeouts happen,
`USB_HostSend` cascade-fails 5271 times (pipe stuck in EHCI error
state, no cancel to clear it).

### Refined understanding

Two separate things are happening:
1. **Primary**: 32 URBs take >50 ms to complete even though pure
   TPU has 0/285 timeouts.  Cause unknown.  Pipeline.start triggers
   them somehow.
2. **Secondary**: After ANY timeout, the EHCI pipe enters an error
   state that only `USB_HostCancelTransfer` can clear.  Without
   cancel, all subsequent submits fail ("send_fail" counter spikes).

The printf feedback loop (user's hypothesis) is a real amplifier but
not the root.  Silencing it cleans up the log but doesn't fix the
underlying 32-timeout trigger.

### Honest conclusion — stopping here

Pure TPU: **75 FPS stable, shipped, safe**.  Pipeline: **broken**
because we can't explain the 32 genuine first-URB timeouts during
pipeline.start.

Options for next session (all require extended focus):
1. Put TPU driver on M4 core — isolate from M7 pipeline scheduling
   (big architectural refactor, but clean fix)
2. Keep cancel-on-timeout AND add a TPU-silicon soft-reset when
   `urb_cancelled` > N in a window (hybrid fault tolerance)
3. Trace EXACTLY what happens in the 50 ms between
   `USB_HostEdgeTpuBulkOutSend` returning success and the sema
   timing out: logic-analyzer on USB_OTG2 D+/D- OR a CSR
   dump of EHCI USBSTS/PORTSC at timeout-moment

## V23 NO-CANCEL + FPS THROTTLE (2026-04-22 night, tried user's suggestion)

User proposed two hypotheses:
1. Cancel-on-timeout might corrupt TPU silicon → stop cancelling, just skip the frame
2. 75 FPS unthrottled might brown-out TPU → throttle to 45 FPS (match camera)

**Both shipped.  Neither fixes pipeline.**

### Changes

- `BulkOutTransferInternal` / `BulkInTransferInternal` no longer call
  `USB_HostEdgeTpuCancelInFlight` on timeout.  Just return -1.
  Caller treats as "skip frame and move on".  The pending URB stays
  in the EHCI schedule; callback fires "whenever" on stack-resident
  `meta` — safe because we returned and next caller reuses the same
  slot for its own `meta`, so the stale lambda write is idempotent
  (binary sema, fields get overwritten anyway).
- `sentai.pipeline.target_fps(n)` — new runtime knob.  InferTask
  sleeps at the end of each loop to cap at `n` fps.  Default 45
  to match camera.  `libs/tpu/edgetpu_driver.cc` + `detection_task.cc`.

### Measurement

Tested pipeline with throttle at 45 FPS and 20 FPS.  Both fail
identically: first URB times out (50 ms / 200 ms) or `USB_HostSend`
returns error → InferTask skips → tries next frame → same error.

async_stats snapshot mid-pipeline (throttle=20, no-cancel):
```
bo_ok=542 bo_malloc=0 bo_send=5234 tm=32 ok=510
```

**bo_send=5234** is the key: `USB_HostSend` (NXP stack) is refusing
submissions.  Without cancel to clear pipe state, the EHCI pipe
stays in whatever error state the first timeout created.  5234 of
5234+542 = 90% of submit attempts fail.

### Net diagnosis

**This is a TWO-sided bug**:
- With cancel: TPU silicon gets wedged (confirmed — standalone invoke
  fails after pipeline.stop until reflash)
- Without cancel: USB host pipe gets halted (confirmed — bo_send
  spikes 90%, further submits refused)

Either way, first URB timing out triggers a permanent failure mode.
The ROOT of the root cause is still: **why does the FIRST URB time
out on pipeline.start, when the same code path on standalone invoke
NEVER times out (0/285)?**

### Real next-session work

1. **Find WHY the first URB during pipeline times out.**  Pure TPU
   over 100 invokes: 0 timeouts.  Pipeline's first URB: ~100% timeout.
   Since our changes (chunk 33 KB, zero-copy, no-heap) all prove stable
   on pure TPU, something about the pipeline's task startup disturbs
   the TPU.  Candidates (untested):
   - The act of `xTaskCreate` for PrepTask/InferTask causes a task
     switch that interferes with a pending USB host operation.
   - EdgeTpuManager mutex interactions: pipeline's first invoke
     competes with some async operation still draining on the TPU.
   - Camera MUX GPIO pulses disrupt USB_OTG2 PHY briefly.
2. **Only after knowing the root cause**, decide on cancel vs
   no-cancel + ClearHalt vs TPU reset.

## V22 INFRASTRUCTURE IMPROVEMENTS (2026-04-22 night, still stuck on pipeline)

Three NASA/JPL-grade improvements shipped this iteration, NONE of
which fixed pipeline end-to-end — but all of which are durable
wins for the firmware overall:

### 1. USB host NXP code moved SDRAM → OCRAM
In `MIMXRT1176xxxxx_cm7_ram_mp.ld`: the `.usb_host` section now maps
to `m_ocram`.  `usb_host_ehci.c` + `usb_host_hci.c` + devices/hub/
hub_app live in on-chip SRAM instead of SEMC-backed SDRAM.  Total
~15 KB.  Freed by relocating `.audio` and `.shine` to SDRAM (not
used in TPU hot path).  Rationale: under pipeline load, PXP DMA
hammers SDRAM at ~40 MB/s; M7 instruction fetches for USB ISR
behind PXP = IOC dispatch delay.  OCRAM placement eliminates
contention for USB code.  Pure TPU benchmark: **75.3 FPS** (no
regression from the 75.9 FPS earlier).

### 2. CSI IRQ priority lowered 0 → 5
In `libs/camera/camera_support.c:BOARD_InitCamera`:
`NVIC_SetPriority(CSI_IRQn, 5)` after `CAMERA_RECEIVER_Init`.
CSI_IRQHandler runs at default ARM NVIC priority 0 = HIGHEST,
preempting USB_OTG2 which is pinned to `configLIBRARY_MAX_SYSCALL_
INTERRUPT_PRIORITY = 2`.  With camera at 45 fps + a few µs ISR
work, USB IOC handling was being consistently delayed.  CSI
handler does not call FreeRTOS APIs (only GPIO writes + atomic
counter increments) so it is safe to move below the syscall
boundary.  Priority 5 puts CSI below USB (2) — USB IOC never
preempted by camera.

### 3. URB timeout bumped 50 ms → 200 ms
`g_sentai_tpu_urb_timeout_ms = 200` (tunable via
`sentai.diag.tpu_urb_timeout(n)`).  Reason: BulkIn URBs issue the
receive BEFORE the TPU finishes compute (compute + wire ≈ 15-20 ms
typical; under pipeline ≥50 ms bursts).  Bulk IN sema fires when
output activations arrive.  50 ms was too tight; 200 ms is 10×
nominal — still fault-tolerant, still triggers cancel-on-stall.

### Pipeline STILL broken — different suspect now

With all three fixes in, pipeline still fails every invoke at the
FIRST URB.  Since USB code is in OCRAM and CSI is de-prioritized,
the classic "SDRAM contention delays IOC" theory doesn't hold.
The remaining suspects:
- `sentai_tpu_invoke_with_input(buf)` swaps `input_tensor->data.
  uint8 = buf` where `buf` is in `.sdram_bss` (`s_tpu_input_buf[]`
  ping-pong).  Maybe TFLite's interpreter cache attrs for arena vs
  `.sdram_bss` differ and USB DMA reads wrong data.
- `direct_tensor` mode in PrepTask (default ON) has a double-buffer
  hand-off via counting semaphores.  If the semaphore release
  ordering has a bug, InferTask might try to invoke with a buffer
  that PrepTask is still filling.
- Simpler possibility — the FIRST invoke after `pipeline.start()`
  has a timing mismatch with camera startup (first frame not ready
  yet) and InferTask holds a bad semaphore state.

**TESTED next-session path in same session**: `direct_tensor(0)`
legacy staging path **ALSO FAILS IDENTICALLY**.  So it's NOT the
ping-pong / pointer-swap mechanism.  Eliminated.

**Latest hypothesis (unverified)**: the TPU silicon itself
enters a bad state when we cancel a partial transfer.  TPU's
USB state machine expects complete blocks; our cancel-on-timeout
leaves it half-fed.  Host-side we cleanly recover (pipe state
resets, next submit succeeds at USB layer).  But the DEVICE
refuses subsequent data.  Observed: after `pipeline.stop()`,
even standalone `sentai.tpu.invoke()` fails the same way until
full reflash.

**Real next-session path**:
1. After `USB_HostCancelTransfer`, also issue a TPU-level reset
   (CSR write through the TPU driver) to get the device's state
   machine back to ready.  The Darwinn / beagle_chip_config
   headers might expose a "reset_request" CSR.
2. OR: on pipeline.start, don't invoke with ping-pong buffers —
   use the interpreter's own arena.  This requires not using
   double-buffering but may sidestep whatever the TPU is
   reacting to.
3. OR: detect permanent TPU wedge and re-DFU the device in
   software (without reflash).  `EdgeTpuManager::GetSingleton()
   ->OpenDevice()` may reinitialize.

## V21 NO-HEAP HOT PATH + FINAL STATE (2026-04-22 night, end)

Final shipped state on device:
- **Pure TPU: 75.9 FPS @ 512×512 YOLO** (13.18 ms/invoke avg; vs 32 FPS baseline = **2.37× speedup**)
- **No dynamic allocation in invoke path** — replaced `std::vector<uint8_t>(8)` in `PrepareHeader` with stack `uint8_t[8]`.  Added `TpuDriver::PrepareHeaderInto()` static helper for all hot-path callers.
- **Cancel-on-timeout fault-tolerance** in both BulkOut/In transfer paths: orphan URB cancelled within ~50 ms, invoke returns -1, InferTask can skip frame and continue.
- **Zero cascade** — the previous 1-timeout-kills-everything chain is broken.  Each URB is independently cancelled on stuck.

Pipeline STILL doesn't work end-to-end — but the FAILURE MODE is now clean (each individual invoke fails fast, no cascade, no board hang beyond occasional TPU load stumble).  **The remaining gap is a bus/arbitration issue outside the TPU driver**.

## V20 CANCEL-ON-TIMEOUT + ROOT CAUSE CONFIRMED (2026-04-22 night)

### Fix shipped

Fault-tolerant URB path in `libs/tpu/edgetpu_driver.cc`:
- `g_sentai_tpu_urb_timeout_ms` — 50 ms default (runtime-tunable via
  `sentai.diag.tpu_urb_timeout(n)`).  Replaces the legacy 2 s wait.
- On timeout: call `USB_HostEdgeTpuCancelInFlight()`
  (new, in `usb_host_edgetpu.c`) to yank the orphan URB out of the
  EHCI async schedule, then wait 500 ms for the cancel-callback to
  land on our stack `meta`.  Return -1 so the caller (InferTask)
  can **skip the frame and continue** — textbook NASA/JPL fault
  containment.
- Counters: `urb_cancelled`, `urb_cancel_no_cb` exposed via
  `sentai.diag.async_stats()`.

**Result**: cascade eliminated.  Previously one stuck URB would
trash `pipe->activeTransfer` for all subsequent submits — every
invoke after the first failure also timed out.  Now the pipe state
is clean after each cancel, so invokes that happen to succeed
(61% of them) produce valid frames.

### Root cause of per-URB stalls (USER HYPOTHESIS CONFIRMED)

Captured counters after a full pipeline run:

```
bo_ok=5263      (submits — 100% accepted by USB host)
cb_user=5350    (callbacks dispatched)
lambda_gave=5263 (sema gives inside lambda)
take_ok=2057     (successful waits)
take_timeout=3206 (timeouts)
urb_cancelled=3206 (all timeouts were cancelled)
urb_cancel_no_cb=0 (cancel callback ALWAYS fires)
```

**39% of URBs time out under pipeline load.** Without pipeline, 0%
timeout on the same code path.  The RATIO doesn't correlate with a
specific URB (it's uniformly random) → this is a **bus-contention
problem, not a logic bug in the driver**.

User's intuition nailed it: "o încurcătură între bufferele PXP,
Camera, DMA și USB, se încalecă, nu sunt sincronizate".  What
likely happens:
- EHCI QH/QTD structures live in SDRAM (allocated via
  `OSA_MemoryAllocate` → m_heap at 0x80000000)
- PXP + camera DMA hammer SDRAM at ~900 KB/frame × 45 fps = 40 MB/s
- When EHCI's bus master tries to WRITE the QTD IOC status bit,
  SDRAM is busy serving PXP → write is delayed past the
  USB_OTG2 interrupt window → IOC never fires → our sema never
  gets given → 50 ms timeout → cancel + skip.
- Sometimes the write wins the arbitration (61%), sometimes not.

### Remaining work (next session)

The underlying bus contention needs one of:
1. **Move EHCI descriptors to non-cached / non-SDRAM memory**.
   The NXP SDK allocates `ehciInstance` via `OSA_MemoryAllocate`
   which goes to the main heap (SDRAM).  A targeted override to
   put the QH/QTD pool in m_ncache (DTCM non-cached) would give
   EHCI deterministic latency.  Concrete step: investigate
   `USB_HostEhciCreate` (usb_host_ehci.c:4200) and the 4275
   `ehciQhList` assignment.
2. **Lower camera / PrepTask priority** below USB host task so USB
   gets CPU uncontested.  All are at `configMAX_PRIORITIES-1=4`;
   demoting camera to 3 while keeping USB at 4 would ensure the
   USB host task drains completion queues between camera frames.
3. **Gate TPU invoke on PrepTask idle** — don't start the bulk
   transfers while PXP is actively DMAing.  Adds latency but may
   eliminate contention entirely.

Runtime-tunable knobs added this session that help debug this
next time:
- `sentai.diag.tpu_urb_timeout(ms)` — 50 ms default
- `sentai.diag.async_stats()` — full counter dict (14 fixed + 5
  dynamic keys via `mp_obj_new_str`)
- `sentai.diag.tpu_chunk_size(n)` — 33 KB default
- `sentai.diag.tpu_zero_copy(bool)`
- `sentai.diag.tpu_async_input(bool)`

## V19 PIPELINE DIAGNOSTIC DATA (2026-04-22 late night)

Added instrumentation at 4 layers of the USB bulk path to root-cause
why pipeline integration fails:

1. `USB_HostEdgeTpuBulkOutSend` — 5 counters (bo_ok, bo_pipe,
   bo_bulk, bo_malloc, bo_send)
2. `USB_HostEdgeTpuPipeCallback` — 4 counters (cb_entered, cb_found,
   cb_user, cb_nopipe)
3. User lambda inside `BulkOutTransferInternal` — 3 counters
   (lambda_entered, lambda_gave, lambda_null_sema)
4. The post-submit Take — 2 counters (take_ok, take_timeout)

All exposed via `sentai.diag.async_stats()` (14 keys via fixed-QSTR
+ 5 more via dynamic `mp_obj_new_str` to avoid QSTR regen churn).

**Measured BASELINE** (3 warmup invokes in pure-TPU mode, chunk=33 KB):
`bo_ok=285, cb_user=292, lambda_gave=285, take_ok=285, take_timeout=0`

Every counter lines up: 285 submits → 285 callbacks → 285 sema gives
→ 285 successful takes.  **All perfectly balanced.**

**Measured AFTER ~2 s of pipeline running** (same firmware):
`bo_ok=732, cb_user=752, lambda_gave=725, take_ok=725, take_timeout=6, cb_nopipe=2`

Delta vs BASELINE:
- +447 submits
- +460 callback entries  (but only +440 to user dispatch)
- +440 lambda-gives
- +440 takes OK
- **+6 takes TIMEOUT** — 6 invoke attempts waited 2 s and got nothing
- **+2 cb_nopipe** — 2 callbacks arrived AFTER pipe->activeTransfer
  had been overwritten by a newer submit.

**Interpretation:** ~99% of URBs complete normally within 2 s even
during pipeline.  The problem is the 1% that don't — each such URB
stalls its invoke by 2 s, InferTask moves to the next invoke, but
the "orphan" in-flight transfer eventually completes and its
callback finds pipe state pointing to the NEWER transfer.  The
pipe-level dispatch (`if (activeTransfer == transfer)`) rejects the
orphan → user lambda never runs → sema never given for THAT invoke
attempt.  Chain effect: every pipeline invoke fails-and-retries.

**Why do 1% of URBs take >2 s to complete?**  Unknown.  Candidates:
- Bulk-IN on a different pipe stalls bulk-OUT pipe scheduling
- TPU device enters a flow-control state and pauses bulk-OUT
- Task scheduling starves USB host task long enough for EHCI
  watchdog to time out the QH

**Next steps (incremental, NASA/JPL):**
1. Reduce sema-take timeout from 2 s to 100 ms and retry-on-timeout
   in BulkOutTransferInternal.  If 99% of URBs finish in <1 ms,
   100 ms is 100 × safety margin and lets the 1% timeout recover
   100× faster.
2. Log which ENDPOINT hit the orphan (likely bulk-OUT EP1 or bulk-IN).
3. Check if EHCI async schedule has anything pending at the
   moment of timeout (dump ASYNCLISTADDR).
4. Consider resetting pipe on timeout (USB_HostCancelTransfer +
   re-open).

**Infrastructure added this session (durable)**:
- 14 bulk counters in `libs/tpu/usb_host_edgetpu.c` + `edgetpu_driver.cc`
- `sentai.diag.async_stats()` wraps them all in a dict
- Runtime toggles: `tpu_chunk_size`, `tpu_zero_copy`,
  `tpu_async_input`, `tpu_multi_ep`, `tpu_desc_cache`
- Dynamic-key dict idiom via `mp_obj_new_str` — future debug
  fields don't need QSTR regen (per user guidance)

## ⚠️ V18 PIPELINE INTEGRATION: BROKEN

End-to-end camera → PrepTask → TPU pipeline consistently fails
with our current TPU driver optimizations:

```
USB_HostEdgeTpuBulkOutSend failed
Bad BulkOutTransferInternal
WriteHeader failed
Node edgetpu-custom-op (number 0) failed to invoke with status 1
E:0420:2   (DET INVOKE_FAIL, val=2)
```

- **Pure TPU**: 13.7 ms/invoke (73 FPS) ✓
- **Pipeline**: TPU cannot submit even an 8-byte WriteHeader once
  `sentai.pipeline.start()` is running.  Errors cascade and USB
  transfer pool appears to leak (subsequent standalone invoke also
  fails until full reflash).
- Tried: `tpu_zero_copy(0)` (staged path), `tpu_async_input(0)`,
  `tpu_chunk_size(32..128 KB)`, direct_tensor OFF — all fail once
  pipeline is up.

Hypotheses (not validated):
1. **Task-priority contention**: camera task + USB host task both
   at `configMAX_PRIORITIES-1`.  PrepTask + camera ISR may starve
   USB host task's event wait, pushing callback latency past the
   2 s sema timeout.
2. **USB transfer pool leak**: After a timed-out transfer the NXP
   stack may not auto-free the transfer struct, and
   USB_HostMallocTransfer eventually returns NULL.
3. **PXP DMA vs EHCI DMA arbitration** on the AXI crossbar
   introducing multi-ms stalls that break the TPU's expected
   timing budget.

Next-session plan:
- `git bisect` the commits between when pipeline last worked
  (likely `56aefcd1 21 fos la switch camera`) and `01725a8f`
  (73 FPS standalone).
- Add printf inside USB_HostEdgeTpuBulkOutSend at
  USB_HostMallocTransfer failure site — see if pool is truly
  exhausted or submit returns another error.
- Instrument `EdgeTpuPerXferDispatch` to count how many callbacks
  fire during pipeline.start() window.

For now, PURE TPU BENCHMARK (E20) is the reliable path to
73 FPS.  Real-time camera detection needs the pipeline
debugged first.

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

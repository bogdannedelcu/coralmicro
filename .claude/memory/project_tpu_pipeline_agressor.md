---
name: TPU pipeline works end-to-end on fresh boot (7.4 FPS) - staged test results contaminated
description: 2026-04-22 late. Confirmed pipeline RUNS (37 ok/2 fail over 5s = 7.4 FPS). Earlier S7-S9 failures were cross-test TPU contamination, not pipeline bugs.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
🎯 **BREAKTHROUGH (2026-04-22 final)**: Pipeline now **42.5 FPS**
end-to-end (from 1.8 FPS) by moving the TPU tensor buffer from SDRAM
to OCRAM.  Zero fails, 100% success rate, 21 ms/invoke.

**Extra fix: `g_camera_frame_seq` now sensor-rate accurate**.  The
NXP CSI driver's re-arm path under heavy consumer drain causes IRQ
to fire ~2× per sensor frame (one for FB1-done edge, one for FB2-
done edge — both buffers completing in the same sensor interval
when buffers are hot).  Fixed in `libs/camera/camera_support.c`:
`CSI_IRQHandler()` now samples SR BEFORE the NXP driver clears it
and increments `g_camera_frame_seq` only when the FB2-done flag is
set.  This gives one tick per real sensor frame in pipeline mode.
Fixes a latent off-by-2 bug in `sentai_cam_get_raw_with_recovery`'s
post-MUX-switch drain threshold (threshold=2 previously meant "wait
1 sensor frame"; now correctly means "wait 2 sensor frames").  Idle
mode reads half (22.5 Hz) because CSI drops half the flags when the
buffer queue is full — only relevant for passive monitoring, not
any in-firmware decision logic.

**Root cause confirmed**: When the tensor buffer lived in SDRAM, the
USB EHCI DMA master read from SDRAM → competed with CSI DMA (camera
writes to SDRAM) on the SEMC bus → intermittent stalls long enough
to wedge the TPU silicon.  Moving the 786 KB tensor to OCRAM gives
the USB a contention-free path via the crossbar; CSI keeps SDRAM
to itself.

**What shipped**:
1. `.libjpeg` moved OCRAM → SDRAM (103 KB freed)
2. `.sentai_slow` moved OCRAM → SDRAM (63 KB freed)
3. `.micropython` moved OCRAM → SDRAM (208 KB freed)
4. Merged OCRAM1 (0x20240000, 512 KB) + OCRAM2 (0x202C0000, 512 KB)
   into a single `m_ocram` spanning 0x20240000..0x2033E000 (1016 KB).
   RPMSG window moved to the end at 0x2033E000..0x20340000.
5. New `.tpu_input` section at m_ocram for tensor ping-pong.
6. Collapsed double-buffer ping-pong → **single 786 KB tensor buffer**
   in OCRAM.  Counting sems `s_sem_bufs_free` and `s_sem_prep_done_c`
   dropped max 2→1 so PrepTask strictly waits for InferTask's USB
   transfer to complete before overwriting the buffer.
7. TPU pipeline numbers (yolo_1 512×512 fresh boot):
   - Pure TPU: 75.2 FPS (13.3 ms/invoke)
   - Pipeline: 41.4 FPS (22.5 ms/invoke, 207 ok / 0 fail in 5 s)

**Why M4 migration was rejected**: RPMSG shared window is 8 KB —
can't transport 786 KB tensor.  Direct shared-SDRAM would still hit
SEMC bus.  Moving USB_OTG2 IRQ to M4 alone doesn't solve bus
arbitration.  OCRAM relocation is the correct fix.

**Lose parallelism, gain correctness + 23× speedup**: theoretical
max was prep(15ms) + invoke(13ms) ≈ 28 ms/frame = 35 FPS.  We
measured 22.5 ms so InferTask's 13 ms invoke runs slightly overlapped
with PrepTask's next-frame cam_grab (up until the sem_bufs_free
take forces wait).

**Pre-breakthrough historical findings**:

**Correct model is yolo_1 (512×512)**, not yolo26n.  File:
`/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite`.
yolo_1 pure TPU: **72.8 FPS** (13.7 ms/invoke) confirmed intact.

**Throttle sweep** (`sentai.pipeline.prep_fps(n)`,
`sentai.pipeline.target_fps(n)` — both added this session):
- PREP=30 TPU=60 → 10 ok / 113 fail in 5 s (8% success, 2 FPS e2e)
- PREP=20 TPU=40 → 0 ok / 93 fail (TPU already wedged from run #1)
- PREP=15 TPU=30 → 0 ok / 76 fail
- PREP=10 TPU=20 → 0 ok / 51 fail

**Observation**: the 10 successful invokes in the first throttled run
averaged 11 ms (FASTER than pure TPU 13.7 ms).  So successful invokes
execute cleanly; failures are a binary "USB pipe dead" mode.  Once
dead, standalone tpu.invoke() also fails until reflash.

**Cross-test corruption cascade confirmed**: every subsequent test
after any failure sees 100% fail.  Must reflash between variants.

**Tech debt cleanup this session**:
- Removed `sentai.pipeline.serialize_prep()` (didn't help, 0% success)
- Removed `sentai.diag.cam_skip_dcache()` (breaks DMA coherency)
- Removed dead `g_sentai_cam_skip_dcache` / getter/setter globals
- Kept useful diag: `debug_prep_mode`, `debug_no_invoke`,
  `infer_stats`/`infer_reset`, `prep_fps`, `target_fps`

**Next-session concrete options** (user-driven):
1. **Move TPU driver to M4 core** — user's suggestion.  EHCI on M4,
   RPMSG shared region for tensor buffers.  Major refactor.  Removes
   M7-vs-USB CPU contention (still has SEMC bus contention).
2. **Move camera driver to M4** — inverse.  CSI on M4, camera ring
   buffer in shared region.  Easier RPMSG surface but same bus.
3. **Free OCRAM and put tensor buffers there** — currently 11 KB free
   in OCRAM.  Moving `.libjpeg` (103 KB) or `.sentai_slow` (63 KB) out
   frees ~100 KB.  Still not enough for 786 KB yolo_1 tensor.
4. **Use eDMA to stage tensor into a smaller OCRAM ring** in bursts
   timed to avoid EHCI contention.  Complex, speculative.

**Metrics remain per user convention**: use yolo_1 only, compare
"pure TPU" vs "pipeline end-to-end" FPS on fresh-boot runs.

═══════════════════════════════════════════════════════════════════

**Earlier (yolo26n) correction notes — kept for history**:

Fresh-boot A/B showed:
- Fresh boot + full pipeline (default): **37 invoke ok / 2 fail over 5 s
  = 7.4 FPS end-to-end**, ~119 ms/invoke.
- Fresh boot + skip DCACHE_Invalidate: 0 ok / 201 fail (skip breaks
  DMA coherency, NOT an improvement).

**Pipeline works.** Earlier "S7/S8/S9 fail 100%" readings were because
once a single test wedged the TPU, every subsequent test in the same
boot saw the wedged state — the failures cascaded across test cases,
not across runtime within one test.

Interpretation:
- On ~95% of frames the InferTask invoke succeeds and pipeline delivers
  a detection; ~5% occasionally fail (E:0420:2) but do not cascade.
- One-shot pipeline runs (<1 s) pass all invokes.
- Long sustained runs probabilistically hit a concurrency window where
  a single invoke fails AND the TPU state gets stuck.  That wedge
  persists across a pipeline.stop()/start() cycle.

**Recovery path**: Currently only full firmware reflash.  A TPU-level
soft-reset (SCU_CTRL_3.rg_force_sleep dance) invoked on pipeline.stop()
would let subsequent runs start from a clean device state — still the
#1 correctness ask for a production pipeline.

**Diag toggles left in firmware (useful for future iterations)**:
- `sentai.pipeline.debug_prep_mode(n)` — 0 full, 1 mock, 2 cam only, 3 cam+PXP
- `sentai.pipeline.debug_no_invoke(bool)` — skip TPU invoke in InferTask
- `sentai.pipeline.serialize_prep(bool)` — hold sem_free until after invoke
  (tried, didn't fix corruption — kept for reference)
- `sentai.pipeline.infer_stats() -> {ok, fail, ms_sum, last_rc}`
- `sentai.diag.cam_skip_dcache(bool)` — skip 615 KB DCACHE_Invalidate in
  camera GET path (tested: breaks DMA coherency, default MUST be 0)

**How to apply**: A/B tests involving the TPU MUST reflash between
variants, or the first failing variant locks state for all following
variants.  `scripts/flashtool.py -e sentai_runtime` between runs.

═══════════════════════════════════════════════════════════════════

**Historical (pre-correction) — superseded by above**:
**Isolated the agressor via staged PrepTask mocks (2026-04-22)**.

Added runtime toggles `sentai.pipeline.debug_prep_mode(n)` and
`sentai.pipeline.debug_no_invoke(bool)` so we can build the
pipeline up piece-by-piece and measure InferTask success/fail
counters live via `sentai.pipeline.infer_stats()`.

`debug_prep_mode` levels in
`examples/sentai_runtime/detection_task.cc:prep_task_fn`:
  0 = FULL (cam_grab + PXP + quant)
  1 = MOCK (vTaskDelay only, no hardware)
  2 = CAM (cam_grab + return + memset dst, no PXP/quant)
  3 = PXP (cam_grab + PXP, no quant)
  4 = same as 0

**Run matrix (2 s each, yolo26n 320×320, POST check = 5 standalone
invokes after pipeline.stop)**:

| Stage | prep | invoke | infer ok/fail | POST invokes |
|-------|------|--------|---------------|--------------|
| S1    | MOCK | skip   | 182 / 0       | 0 fail ✓     |
| S3    | CAM  | skip   | 179 / 0       | 0 fail ✓     |
| S4    | PXP  | skip   | 179 / 0       | 0 fail ✓     |
| S5    | FULL | skip   | 183 / 0       | 0 fail ✓     |
| S6    | MOCK | real   |  33 / 0       | 0 fail ✓     |
| S7    | CAM  | real   |   0 / 131     | 5 fail ✗     |
| S8    | PXP  | real   |   0 / 156     | 5 fail ✗     |
| S9    | FULL | real   |   0 / 171     | 5 fail ✗     |

**Breakpoint is between S6 and S7.**  Adding camera-grab activity
concurrent with a real TPU USB bulk transfer wedges the TPU.  Every
subsequent invoke (including standalone post-stop) fails until
reflash.  PrepTask alone doing cam+PXP+quant+memset at 90 Hz is
SAFE; the TPU invoke path alone is SAFE; the combination is fatal.

**Root cause (high confidence)**: SDRAM bus contention between
CSI DMA (camera buffer fill via SEMC) and EHCI DMA (USB host
reading QH/QTD descriptors + bulk-OUT data from SDRAM).  The CSI
IRQn priority lowering from prior session helped M7 ISR ordering
but doesn't affect the hardware DMA arbitration on the SEMC bus.

**Note**: S6 ran at ~108 ms/invoke steady (32/2 sec ≈ 16 FPS).
That's the legacy-path pipelined TPU throughput without PrepTask
interference.  Our earlier 102 ms/invoke T1 baseline matches.

**Next-session fix paths (ranked)**:
1. **Move EHCI QH/QTD out of SDRAM** — the documented #1 from
   prior session.  NXP `OSA_MemoryAllocate` puts descriptors in
   SDRAM; patch `usb_host_ehci.c` to redirect the QH/QTD list
   into `m_ncache` (DTCM non-cacheable) or a dedicated static
   buffer.  Eliminates contention with CSI.  Main unknown: how
   the NXP stack picks its allocator.
2. **Move camera frame buffer out of SDRAM** — inverse approach.
   OV5640 640×480×2 = 615 KB; won't fit in DTCM/OCRAM but could
   fit 320×240 grayscale in OCRAM.  Requires CSI DMA destination
   re-route.
3. **Throttle PrepTask explicitly** — have PrepTask wait on a
   "TPU idle" sema so cam_grab can't run while TPU USB transfer
   is in flight.  Serialises camera+TPU; costs parallelism but
   makes correctness deterministic.  Cheapest to implement.
4. **Move TPU driver to M4** — prior session's option 3; major
   refactor with RPMSG window constraints.

**Diag infrastructure added this session (persistent)**:
- `sentai.pipeline.debug_prep_mode(n)` — 0..4
- `sentai.pipeline.debug_no_invoke(bool)`
- `sentai.pipeline.infer_stats() -> {ok, fail, ms_sum, last_rc}`
- `sentai.pipeline.infer_reset()`

Staged test driver in
`examples/sentai_runtime/diag/_t_stage.py`.

**How to apply**: Before recommending "pipeline needs X", re-run
the staged matrix on any firmware change that touches
CSI/PXP/EHCI/USB — the breakpoint must move from S6→S7 to S7→S8
(at minimum) to count as progress.

---
name: project_option_b_packed420_pipeline_w24_t15_t16
description: "2026-11-01 — option B (packed YUV420 capture + PXP byte-drop) measured in the TPU pipeline (s260): VGA45 44 fps, SXGA30 13.8 fps now USB-bound (70 ms/invoke), PXP 62 → 25 ms; camera+PXP alone SXGA 22.5 fps; slice-chasing PXP (s261) cuts EOF→ready latency to 3.4 ms, bit-exact; CPU polls of camera buffers need D-cache invalidate; staging buffer must be chosen by size not by model"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-01T10:18:23.216Z
---

**EXP-s260 (W24-T16) + EXP-s261 (W24-T15 step 1), builds 1733–1741, board 58eb, one camera.**

- **Option B in the detection pipeline** (`camera.packed420(1)` before init → CSI
  receiver at half width + sensor 0x4300=0x5F, plus 0x460C=0x22/0x3824=0x01 on SXGA;
  PrepTask runs `sentai_pxp_pack420_drop` = PXP RGB888(32-bit)→RGB888P(24-bit), the
  zero byte dropped → `[C,Y1,Y2]` 1.5 B/px, same byte count as Y+UV):
  VGA45 41.7 → 44.0 fps (sensor rate), PXP 7.0 → 3.6 ms; SXGA30 12.7 → 13.8 fps,
  PXP 61.6 → 25.2 ms, invoke 76 → 70 ms; the SXGA pipeline is now **USB-bound**
  (70 ms per 1.84 MB + 0.3 MB instr at 35 MB/s = 14.2 fps cap).  SDRAM per SXGA
  frame 13.5 → 8.6 MB.  Zero errors.  Next fps lever at SXGA = bytes per invoke
  (W24-T13 instruction caching), not memory.
- **Camera + PXP without TPU** (`pipeline.debug_no_invoke(1)`; build 1735 releases
  the staging buffer in that mode, before it stalled after one frame): VGA 45/45 fps
  both modes; SXGA legacy 17.5 fps vs option B 22.5 fps.
- **Benchmark hygiene:** the first pipeline run after `camera.init()` is slow
  (20–24 fps at VGA45) because the camera autotune/genlock steals frames for ~10 s
  → always discard a warm-up run.  `sensor_frames()` only advances while frames are
  dequeued.  AEC night mode (0x3A00 bit 2) was off; dark room did not matter.
- **Slice-chasing PXP (s261, `camera.chase_test(n)`):** sync on CSI CR3.FRMCNT, read
  DMASA_FB1/FB2 ~150 µs later (equal = single active buffer), poison the padding
  byte (byte 3 of the last pixel, always rewritten 0) of each slice's last row in
  both candidates during vblank, chase the one that clears.  Bit-exact 64/64 runs.
  Latency EOF→ready: SXGA30 B 26.9 → 3.4 ms (8 slices), VGA45 B 3.0 → 0.4 ms;
  legacy SXGA PXP per slice (6.4 ms) is slower than the camera (4.1 ms/slice) so it
  falls behind; option B keeps up.  Throughput unchanged by design.
- **Gotchas found:** (1) CPU polling of camera frame buffers returns stale D-cache
  data while the CSI writes SDRAM — invalidate the line before reading, clean after
  writing (DMA consumers unaffected). (2) `yuv_in_buf()` picks the 1.84 MB SDRAM pair
  only when a 2-input model is loaded; `sentai_pipeline_staging_buf()` now selects
  by size (SXGA test without a model wrote 1.84 MB into the 921 KB OCRAM buffer →
  precise bus error at 0x20400000). (3) CSI write window ≈ active lines only:
  VGA45 packed 10.9 ms / legacy 21.7 ms; SXGA30 21.8 / 43.6 ms.
- **"SXGA30" row = 22.6 fps real** (CSI FRMCNT; `sensor_frames()` over-counts ~15 %).
  True-30 row added: `camera.init(1, 36)` (PLL 0x70, sysdiv 2, mipi_div 1, settle
  0x12; SDK patch 0007, build 1742): camera + PXP option B 28.6 prepared fps (PXP
  32.6 ms ≈ frame period), legacy 15.2 fps and the full legacy pipeline HANGS the
  board (REPL silent, USB alive, no crash log; recover with flashtool re-flash);
  option B pipeline 13.6 fps, 0 errors (USB-bound).  Frame rate ∝ PLL mult /
  (sysdiv × mipi_div); mipi_div also divides PCLK.
- **PXP 16x16 blocks (knob `camera.pxp_block16`, default OFF):** without the TPU
  they cut the camera repack 30–40 % under CSI load (true-30 B byte-drop 32.5 →
  19.6 ms → 30/30 frames prepared; legacy 62 → 44 ms), identical output — but
  WITH the TPU streaming the long PXP bursts starve the EHCI reads: invoke 70 → 77
  ms, SXGA pipeline 13.8 → 12.5 fps, VGA 44 → 42.5 (build 1746).  Use only for
  camera-only workloads (flow/markers/recording).
  Why 28.6 not 30 before: PXP 32.5 + 1 ms hand-off > 33.2 ms period → one frame
  skipped every ~14 (deterministic slip, not jitter); CSI 74 + PXP 132 MB/s = SEMC
  practical limit.
- **OCRAM input ring (s262, builds 1749–1752, `pipeline.ocram_ring(1)`):** 3 × 245 760 B
  slots in the idle `.tpu_input` OCRAM buffer; PrepTask produces packed rows per slice,
  the TPU driver's SendInputs maps the staging pointer to ring slots, waits per slice,
  releases per slot, one frame in flight (`frame_done` gate — without it the 2-frame
  cap reset the ring mid-transfer → desync).  SXGA option B **13.0 → 15.6 fps**,
  invoke 74 → 61 ms (= 35 MB/s intake), 0 errors, outputs byte-identical to staging.
  Model hint order: Y in 3 overlapping tiles (9-row halo), then UV.  URB 36 vs 120 KB
  no difference.  SDRAM per SXGA frame now 4.9 MB (CSI + PXP read only).
- **CRITICAL BUG FIXED (build 1751):** the direct-tensor path (default 1) never fed the
  Edge TPU: `input_tensor(k)->data` patch does not reach the EvalTensor the op reads
  (TFLM copies the pointer at allocation) → output identical for bars/natural/gradient
  and equal to a bare invoke on the arena.  Fix: `g_sentai_tpu_input_override[k]` set
  by the invoke wrappers, consumed in `EdgeTpuExecutable::Invoke` (+ int8 transform on
  it).  Speed numbers unaffected; detection quality on that path before 2026-11-01 was
  on stale data.  Test: s262 `input_path_check2.py` (output fingerprint per scene).
- ch569w lessons checked: SentAI already opens the TPU at kMax (GCB 500/AXI 250/8051
  500); instructions are re-sent per inference by design; warm Coral stalls M3 run-state.
- "Retrain" for option B is probably unnecessary: packed `[C,Y1,Y2]` is a
  space-to-depth of Y along x plus U/V row interleave, so the planar stem's weights
  re-index exactly into an H×W/2×3 stem with stride (2,1); only the TPU compiler
  mapping of the asymmetric stride needs checking.
- Open: W24-T15 step 2 (PrepTask starts the repack mid-frame), step 3 (OCRAM line
  ring; does the CSI DMA alternate FB1/FB2 mid-frame?), W24-T16 production (3-channel
  stem retrain, frame-layout flag for JPEG/flow/markers).

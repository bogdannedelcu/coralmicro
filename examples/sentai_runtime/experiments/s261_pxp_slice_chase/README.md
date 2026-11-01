# EXP-s261 — PXP repack that chases the CSI writer inside the frame (OP-S10-W24-T15 step 1)

**Board** 58eb, **builds** 1735–1741, **date** 2026-11-01, one camera, colour bars on
the sensor (static content, so two consecutive frames compare equal).

**Question** (operator): must the PXP wait for the complete frame, or can it repack
slice by slice while the camera is still writing, starting at ~25 % of the frame?

## Method

`sentai.camera.chase_test(n)` (firmware, pipeline stopped, caller owns the PXP):

1. prime the stream (4 dequeue/return cycles — frames only flow while someone
   dequeues) and sync on the CSI frame counter (`CSI_CR3.FRMCNT`);
2. ~150 µs later read the two armed buffers (`DMASA_FB1/FB2`; equal when the camera
   task has handed the CSI a single buffer) and poison the padding byte (byte 3 of
   the last pixel, always rewritten as 0 by the receiver) of the last row of every
   slice in both candidates — still inside the vertical blank;
3. the candidate whose slice-0 marker clears first is the in-flight buffer; for each
   following slice wait for its marker, then run the PXP on that slice only (option B
   byte-drop or legacy XYUV→YUV420) into staging slot 0;
4. dequeue the next completed frame, run the full-frame PXP into slot 1, `memcmp`.

Reported per run: frame write time (sync → last slice), **latency from end-of-frame
to repack done**, per-slice PXP time, output equality.  `run_chase.py`, 8 runs per
point.  n = 1 is the baseline (full-frame PXP after EOF).

## Results (`results_b1740.txt` VGA, `results_b1741.txt` SXGA)

| phase | n slices | CSI write window | PXP total | **latency EOF → ready** | equal |
|---|---|---|---|---|---|
| VGA45 legacy XYUV | 1 | 21.7 ms | 5.7 ms | 5.7 ms | 8/8 |
| | 2 / 4 / 8 | 21.7 | 5.7–6.1 | 2.8 / 1.3 / **0.6** | 8/8 |
| VGA45 option B packed | 1 | 10.9 ms | 3.0 ms | 3.0 ms | 8/8 |
| | 2 / 4 / 8 | 10.9 | 3.0–3.2 | 1.5 / 0.8 / **0.4** | 8/8 |
| SXGA30 legacy XYUV | 1 | 43.6 ms | 51.3 ms | 51.3 ms | 8/8 |
| | 2 / 4 / 8 | 47–51 | 51.5 | 26.1 / 13.0 / **6.5** | 8/8 |
| SXGA30 option B packed | 1 | 21.8 ms | 26.9 ms | 26.9 ms | 8/8 |
| | 2 / 4 / 8 | 25–27 | 27.5 | 13.8 / 6.9 / **3.4** | 8/8 |

* Slicing works and is bit-exact in every mode: the PXP output of 8 chased slices
  equals the full-frame repack of the same content (64 of 64 runs).
* The latency after end-of-frame drops to one slice's worth of PXP: at SXGA30 option
  B from 26.9 ms to 3.4 ms, at VGA45 from 3.0 to 0.4 ms.  The buffer is ready for the
  TPU ~3 ms after the last line instead of ~27 ms.
* Throughput is unchanged by design: the PXP does the same total work (3.0 / 27.5 ms
  per frame), only spread inside the frame.  Camera + PXP without the TPU already
  deliver (s260 `prep_only.py`) VGA 45 of 45 fps in both modes and SXGA 17.5 fps
  (legacy, PXP 51 ms) / **22.5 fps (option B, PXP 27 ms)**.  In legacy SXGA the PXP
  per slice (6.4 ms) is slower than the camera writes a slice (4.1 ms), so the chase
  falls behind and the window stretches to 50 ms; in option B it keeps up.
* Side observations: the CSI write window at VGA45 is 10.9 ms in packed mode and
  21.7 ms in legacy (two pixel-link cycles per pixel pair vs one per pixel); at
  SXGA30 21.8 vs 43.6 ms with the sensor's 33 ms period (the write window is the
  active-line time, the rest is blanking).

## Two findings on the way

1. **CPU reads of the camera buffers can be stale.**  The first chase versions never
   saw a marker clear although `FRMCNT` advanced and `peek32` showed the CSI had
   overwritten the word: the poll loop was served from the M7 data cache while the
   CSI wrote SDRAM.  The test now invalidates the cache line before each marker read
   and cleans it after each write (`SCB_InvalidateDCache_by_Addr` /
   `SCB_CleanDCache_by_Addr`).  DMA consumers (PXP, TPU) are unaffected; any CPU
   code that re-reads a camera buffer while it is being written must do the same.
2. **Staging-buffer selection was model-dependent.**  `yuv_in_buf()` picks the 1.84 MB
   SDRAM pair only when a 2-input model is loaded; with no model the SXGA test wrote
   1.84 MB into the 921 KB OCRAM buffer → precise bus error at 0x20400000 (build 1740,
   `crash_prev.txt`).  `sentai_pipeline_staging_buf()` now selects by size.

## Step 4 answered: the CSI DMA does not restart mid-frame (`csi_slice_probe.py`, `csi_slice_rate.py`, build 1752)

Receiver height set to 128 or 480 lines (`csi_geom(640, H)`, packed YUV420, SXGA
true-30 row): the CSI keeps running, captures the top H lines of every frame, and the
DMA frame-done rate stays **30–31/s** in all three cases (960 / 480 / 128) while a
consumer dequeues (`sensor_frames` = DMA done events, `frame_count`, CSI `FRMCNT`).
A mid-frame FB1/FB2 restart would have shown 60 or 225 per second.  The DMA "frame"
is SOF-gated: after IMAG_PARA lines it waits for the next frame start.  A CSI line
ring in OCRAM is therefore **not possible** on the RT1176 CSI; the OCRAM gain was
taken on the TPU side instead (EXP-s262: PXP output ring in OCRAM, 15.6 fps).

## What this enables (next steps, not done here)

* PrepTask integration (W24-T15 step 2): start the repack when the frame is ~1/n
  written instead of after `GetRawFrame`; with option B at SXGA the TPU input is
  ready ~3 ms after the last line.  Needs the in-flight buffer from the camera
  stack (today derived from `DMASA_FB1/FB2`) and the marker/cache discipline above.
* The OCRAM line ring (W24-T15 step 3) — the CSI DMA alternates FB1/FB2 per frame
  only; mid-frame alternation is still untested.

## Files

`run_chase.py`, `marker_probe.py` (poke/peek proof that the CSI rewrites the padding
byte), `chase_b1740.log`, `chase_b1741.log`, `results_b1740.txt`, `results_b1741.txt`,
`marker_probe.log`.  Firmware: `sentai_pxp_chase_test()` in `sentai_runtime.cc`,
`sentai.camera.chase_test(n)`.

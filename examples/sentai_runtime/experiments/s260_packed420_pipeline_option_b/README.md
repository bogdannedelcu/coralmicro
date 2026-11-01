# EXP-s260 — Option B speed test: packed YUV420 capture + PXP byte-drop in the TPU pipeline (OP-S10-W24-T16)

**Board** 58eb, **build** 1733, **date** 2026-11-01, one camera (cam 0), dark room
(same scene in every phase).  Models: the trained s257 `c2f_yuv420_int8_edgetpu`
(VGA) and the s258 `c2f_yuv420_sxga_relu_int8_edgetpu` (SXGA); detections are not
evaluated — the models read the same number of input bytes in both modes, so the
invoke time is representative, the content is not.

## What option B is

EXP-s259 showed the OV5640 "YUV420 for MIPI only" stream (0x4300 = 0x5F) lands in
memory at 2 B/px as `[C, Y1, Y2, 0]` per pixel pair (C = U on even rows, V on odd
rows).  Option B keeps that buffer as the camera frame and replaces the PrepTask's
PXP repack (XYUV8888 -> Y plane + UV plane, reads 4 B/px) with a PXP copy that reads
the buffer as RGB888 (32-bit unpacked, W/2 x H) and writes RGB888P (24-bit packed):
it just drops the zero byte -> `[C, Y1, Y2]` = 1.5 B/px, exactly the byte count of
Y + UV, written into the same staging buffer the TPU reads.  A production model would
take this as a 3-channel H x W/2 input (stem retrain); USB payload per invoke is
unchanged.

Firmware (default off): `sentai.camera.packed420(1)` before `init()` → CSI receiver
at half width (24-bit bus) and, after the OV5640 driver init, `0x4300 = 0x5F` on the
active sensor plus `0x460C = 0x22`, `0x3824 = 0x01` on the SXGA rows (manual DVP PCLK
divider, s259); PrepTask calls `sentai_pxp_pack420_drop()` when the flag is set;
`sentai.pipeline.dump_staging(path, slot, n)` dumps the staging buffer for checks.

## Results (`run_option_b.py --seconds 15 --tag b1733_wu`, `results_b1733_wu.txt`)

| phase | camera | pipeline fps | invoke ms | cycle ms | PXP ms / frame | grab ms | sem wait ms | fail |
|---|---|---|---|---|---|---|---|---|
| legacy XYUV + repack | VGA 45 | 41.7 | 20.0 | 9.8 | 7.0 | 0.1 | 4.8 | 0 |
| **option B** | VGA 45 | **44.0** | 19.2 | 9.6 | **3.6** | 2.0 | 6.5 | 0 |
| legacy XYUV + repack | SXGA 30 | 12.7 | 76.0 | 71.5 | 61.6 | 1.1 | 8.8 | 0 |
| **option B** | SXGA 30 | **13.8** | **70.3** | 57.2 | **25.2** | 1.1 | 30.3 | 0 |
| legacy XYUV + repack | SXGA 15 | 10.9 | 68.6 | 69.2 | 44.7 | 44.9 | 0.0 | 0 |
| **option B** | SXGA 15 | 10.9 | 65.2 | 65.2 | **19.0** | 69.8 | 0.0 | 0 |

* VGA45: the pipeline was already camera-bound; option B reaches the sensor rate
  (44.0 of 45) and halves the PXP time (7.0 -> 3.6 ms); invoke 20.0 -> 19.2 ms
  (less SDRAM contention on the EHCI reads).
* SXGA30: PXP 61.6 -> 25.2 ms (2.4x; it reads 2.46 instead of 4.9 MB and the CSI
  writes half as much), invoke 76 -> 70 ms.  The pipeline is now purely USB-bound:
  70.3 ms per invoke = 14.2 fps ceiling, measured 13.8 (sem wait 30 ms = InferTask is
  the bottleneck, PrepTask idles).  Legacy was PXP/SDRAM-bound (cycle 71.5 ms).
* SXGA15: camera-phase bound in both modes (grab 45–70 ms), as in s258.
* Option B SDRAM traffic per SXGA frame: 2.46 (CSI) + 2.46 (PXP read) + 1.84 (PXP
  write) + 1.84 (EHCI) = 8.6 MB vs 13.5 MB legacy.
* Zero failures, zero URB/EHCI errors in all six phases.

Layout check (`--bars-dump`, `stage_vga_b.png`): staging rows 0/1 after the byte-drop
with colour bars = C [118,26,139,20,235,113,229,135] (U) / [130,146,20,20,235,235,
107,126] (V), Y1 = Y2 = [253,238,197,182,77,62,21,5] → `[C,Y1,Y2]` as designed,
image correct.  (The SXGA staging dump is 1.84 MB and `/api/raw` refuses it; the VGA
check covers the same PXP operation.)

## Camera + PXP only, no TPU (`prep_only.py --tag b1735`)

Operator question: without the USB limit, how many frames per second come out ready
for the Edge TPU?  `sentai.pipeline.debug_no_invoke(1)` runs PrepTask (grab + PXP)
with InferTask skipping the transfer (build 1735 also releases the staging buffer in
that mode; before, the dual-input path waited for a transfer callback that never
came and PrepTask stalled after one frame).

| phase | prepared frames / s | PXP ms | grab wait ms | prep total ms |
|---|---|---|---|---|
| VGA45 legacy | 45.0 | 6.0 | 15.4 | 22.3 |
| VGA45 option B | 45.0 | 3.0 | 16.5 | 22.3 |
| SXGA30 legacy | 17.5 | 51.3 | 5.7 | 57.4 |
| **SXGA30 option B** | **22.5** | 27.0 | 17.7 | 44.7 |

VGA is the sensor rate in both modes.  At SXGA30 the legacy repack alone caps the
pipeline at 17.5 fps; option B lifts the camera + PXP ceiling to 22.5 fps (the sensor
in the MIPI YUV420 format delivers ~26 frame-counter ticks/s and the grab lands on
every other frame boundary when PXP + grab exceed one period).  With the TPU the
pipeline is USB-bound at 13.8 fps (above), so the prep side now has headroom.

## The "SXGA 30 fps" row was 22.6 fps — and a true-30 row (`cadence_probe.py`, build 1742)

Operator question: if the PXP keeps up (27 ms < 33 ms), why 22.5 and not 30 prepared
frames/s?  The CSI hardware frame counter (`CSI_CR3.FRMCNT`) answers: the sensor row
selected by `camera.init(1, 30)` at 1280x960 delivers **22.6 frames/s**, idle or
loaded, legacy or packed.  The row was cloned from the SDK's 720P/30 clock row (PLL
84x, sysdiv 2, MIPI div 1) but the SXGA frame has 984 lines instead of ~740, so
30 x 740/984 = 22.6.  `sentai.camera.sensor_frames()` over-counts by ~15 % (26/s), which
is where the "30 fps" belief in s258 came from (s258's "camera 30" numbers were at
22.6 fps, its "camera 15" numbers at ~11.3).

Frame rate scales with PLL mult / (sysdiv x mipi_div), so a row with mult 112 (0x70,
PLL 896 MHz = the VGA/60 row's PLL) and the same dividers gives a true 30 fps.  Added
as `fsl_ov5640.c` fps key **36** (SDK patch 0007), D-PHY settle 0x12.

| SXGA row | CSI FRMCNT | mode | camera + PXP only | full pipeline (TPU) |
|---|---|---|---|---|
| fps key 30 (PLL 84x) | 22.6 /s | legacy | 17.5 fps, PXP 51 ms | 12.7 fps |
| fps key 30 | 22.6 /s | option B | 22.5 fps, PXP 27 ms | 13.8 fps |
| **fps key 36 (PLL 112x)** | **30.0 /s** | legacy | 15.2 fps, PXP 62 ms | **board hang** (REPL silent, USB alive, no crash log; recovered by re-flash) |
| fps key 36 | 30.0 /s | option B | **28.6 fps, PXP 32.6 ms** | 13.6 fps, invoke 71 ms, 0 errors |

* With a real 30 fps camera the legacy path (4 B/px CSI = 147 MB/s + PXP read 4 B/px)
  starves the rest of the SoC again — the s258 "total collapse" regime that the
  W24-T12 EHCI/SEMC tuning had cured only at 22.6 fps.  Option B halves the CSI
  write and survives with zero errors.
* Camera + PXP in option B at a true 30 fps: 28.6 prepared frames/s with the PXP at
  32.6 ms, i.e. right at the frame period — the slice-chasing repack (s261) would
  spread it inside the frame.  The TPU pipeline stays at 13.6 fps: USB-bound.

## Method notes

* Each phase is a fresh boot; the first pipeline run after `camera.init()` is slower
  (20–24 fps at VGA45, grab 15–23 ms) because the camera autotune/genlock started by
  init steals frames for several seconds; `probe_fps.py` showed 24.3 then 41.2 fps
  on the same boot.  The benchmark therefore discards an 8 s warm-up run.
* `sentai.camera.sensor_frames()` does not advance while no consumer dequeues frames,
  so the "cam/s" column reads 0 before the pipeline starts; `camera.fps()` reports 45.
* AEC night mode (0x3A00 bit 2) was already off (0x58).

## Verdict

Option B does what the estimate said: VGA at the sensor ceiling with half the PXP
load, SXGA from 12.7 to 13.8 fps with the PXP off the critical path (2.4x faster) and
the pipeline now limited only by the 35 MB/s Edge TPU intake (70 ms per 1.84 MB
invoke).  The remaining lever at SXGA is bytes per invoke (instruction caching,
W24-T13), not memory.  Next (operator): slice-based repack that starts while the CSI
is still writing the frame (W24-T15 step 1, EXP-s261).

## Files

`run_option_b.py` (benchmark), `probe_fps.py` (warm-up finding), `results_b1733*.txt`,
`run_b1733*.log`, `stage_vga_b.png`/`.bin` (byte-drop output with colour bars).

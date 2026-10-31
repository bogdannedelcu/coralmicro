# EXP-s256 — Camera → JPEG → SD card: throughput, stability, encoder speed

**WBS:** OP-S10-W24-T1 (chain performance + stability), OP-S10-W24-T2 (encoder
speed, native-YUV path).  **Board:** rev 1.1 unit 58eb, two OV5640 at VGA 45 fps,
microSD 64 GB on USDHC1.  **Date:** 2026-10-31.  **Builds:** 1688 (baseline,
-Os islow), 1689 (libjpeg -O2 + JDCT_IFAST), 1692 (first YUV path, striped),
1693 (YUV path fixed), 1694 (phase-estimator fix; persisted to flash).

## Question

How many frames per second survive the continuous chain camera → JPEG → FileX
write on the SD card, single camera and 1:1 alternation, when one frame in N is
saved; does the card run in high-speed mode; does anything in the chain wedge
over minutes; and does the encoder use what the RT1176 offers.

## Method

`cam_to_sd.py --mode {single,alt} --every N --minutes M [--yuv 1]`.  Board side is
orchestration only (MicroPython): every frame is dequeued with `peek_col()`, every
N-th frame is encoded with `sentai.camera.jpeg(q[, yuv])` (C: PXP + libjpeg) and
written with `sentai.fs.write()` (C: FileX) to `/rec/fNNNNN.jpg`, 100 names
rotating.  One CSV row per 30 s window (`results_<tag>.csv`); the per-stage
timings printed by the C side are in `run_<tag>.log`.

Pass criteria: 0 write errors, 0 CSI stalls, 0 CSI queue faults over the whole
run; phase stays within the drift thresholds; the saved JPEG decodes and matches
the RGB-path image of the same scene.

## Results

### Matrix (build 1688 unless noted; q = 75; "enc" = whole C capture incl. PXP)

| run | windows | consumed fps | saves | enc ms | write ms | kB/s | errs | stall | CSI faults |
|---|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| single, every 45, 3 min | 6 | 39.0 | 156 | 115 | 17 | 18 | 0 | 0 | 0 |
| alt 1:1, every 45, 3 min | 6 | 38.5 | 156 | 117 | 17 | 20 | 0 | 0 | 0 |
| single, every 10, 3 min | 6 | 26.0 | 476 | 118 | 22 | 70 | 0 | 0 | 0 |
| alt 1:1, every 10, 3 min | 6 | 26.0 | 481 | 117 | 19 | 65 | 0 | 0 | 0 |
| alt 1:1, every 15, **10 min soak** | 20 | 30.2 | 1233 | 116 | 20 | 47 | 0 | 0 | 0 |
| alt, every 15, 1 min, build 1692 RGB -O2 IFAST | 2 | 31.0 | 125 | 109 | 21 | 46 | 0 | 0 | 0 |
| alt, every 15, 1 min, build 1694 **YUV path** | 2 | 35.0 | 141 | 69 | 27 | 89 | 0 | 0 | 0 |

Single camera and 1:1 alternation behave identically: the MUX flip is deferred
(F16 fix) and costs nothing on this path.  The consumed fps is purely
`45 − saves_per_second × enc_ms / 1000` — the encode blocks the MicroPython loop,
frames queue in the 4 CSI buffers and the surplus is dropped by the sensor, never
by a stall.  Nothing wedged: 10 minutes alternating with a save every 15 frames
gave 0 errors, 0 stalls, 0 CSI faults, phase −3..−5 µs throughout.

### SD card

`sentai.sdcard.bus()` → `(49500000, 1, 4, 'SDR25 high-speed')`: 3.3 V SDR25,
49.5 MHz, 4-bit.  No UHS (needs the 1.8 V switch; VSELECT is parked).  FileX
write of a 13–40 KB JPEG costs 14–29 ms (≈1.2–1.7 MB/s, journal on); sequential
read 16 MB/s.  The card is not the bottleneck at any save period tested.

### Encoder (per-stage, from the C printf, 640×480 q75)

| path | build | drain | PXP | split | jpeg | total |
|---|--:|--:|--:|--:|--:|--:|
| RGB: XRGB8888 → PXP RGB888 → libjpeg (colour convert + downsample in software) | 1688 -Os islow | 22 | 11 | – | 83 | 116 |
| RGB, libjpeg -O2 + JDCT_IFAST | 1692 | 22 | 11 | – | 75 | 108 |
| **YUV: sensor YUYV → CSI XYUV8888 → PXP Y + UV 4:2:0 → libjpeg raw_data_in** | 1694 | 23 | 10 | 5 | 30 | 68 |

"drain" is the 22–23 ms wait for a fresh frame (one 45 fps period), not CPU.
The YUV path removes the RGB→YCbCr conversion and the chroma downsampling from
libjpeg entirely (the OV5640 ISP emits YUV; the PXP does the 2×2 chroma
decimation) — JPEG CPU time 75 → 30 ms, 2.5×.  The RT1176 PXP has **no CSC2**
(`FSL_FEATURE_PXP_HAS_NO_CSC2`), so RGB→YUV in hardware was not an option; the
native camera YUV mode (`sentai.camera.yuv_mode(1)` before `init`) is the same
input format the Edge TPU YUV420 models use (W24-T3).

### Bug found and fixed on the way (build 1692 → 1693)

The first YUV JPEGs showed 16-row-period green/black stripes
(`sample_yuv_path_b1692.jpg`).  Reproduced on the host by compiling the vendored
NXP libjpeg (IJG **9b**) sources with gcc and feeding the Y/UV planes dumped by
`sentai.camera.yuv420_test()`: identical stripes, so neither the PXP nor the ARM
build was at fault.  Instrumenting `forward_DCT` showed the chroma components
with `DCT_h/v_scaled_size = 16`: IJG 9 implements 2×2 chroma subsampling *inside
the DCT* when `do_fancy_downsampling` is TRUE (the default) and expects
full-resolution chroma — in `raw_data_in` mode we hand it already-subsampled
planes, so it read 16 rows from an 8-row buffer.  One line fixes it:
`cinfo.do_fancy_downsampling = FALSE` (`libs/libjpeg/jpeg.cc`).

### Colour / byte-order verification (operator request)

Same scene, RGB path (build 1688 from flash) and YUV path (build 1694) 30 s
apart, both cameras — `ab_rgb_vs_yuv_2x2.jpg`:

| camera | RGB path mean YCbCr | YUV path mean YCbCr |
|---|---|---|
| cam0 | 114 / 139 / 129 | 112 / 141 / 128 |
| cam1 | 118 / 148 / 130 | 118 / 147 / 132 |

Red cable, green plant, blue cloth and yellow box render identically on both
paths, so the CSI XYUV lane order (byte2 Y, byte1 U, byte0 V) and the PXP UV
plane order (byte0 = Cb) are right; no missing sensor initialisation in YUV mode.

### Camera 1 white-balance stall (observed once — OP-S10-W24-T5, open)

Earlier in the session the two cameras showed different hues (cam0 purple,
cam1 yellow-green).  Register diff: cam1's AWB gains 0x3400–0x3405 sat at
unity and never moved while cam0's were live (R 1.2, B 2.1); all AWB control
registers 0x5180–0x519F identical.  Context: warm `sys.reset()` out of a
YUV-mode session into an RGB-mode init (the sensors are not power-cycled by a
SoC reset).  Toggling 0x3406 1→0 restarted cam1's AWB.  A clean cold init had
both AWB engines running from the first second.  Not reproduced yet; tracked as
W24-T5.  Not a colour-path issue.

### Phase estimator false alarm (build 1693 → 1694)

The 1693 run logged `PHASE_DRIFT` with phi = 22282 µs (= T − 4) followed by
`PHASE_DRIFT_END` at −3 µs.  `sentai_cam_phase_from_log()` chose the
alternation regime by counting same-camera intervals; a consumer that blocks
longer than one period (the 70 ms encode) leaves same-camera entries 2T apart in
the FB-done log, T came out doubled and phi = T_real − 4.  Now the flip-pair
estimate (exact 2T sum) is tried first and T is folded/sanity-checked against the
VTS-derived period.  1694 run: no drift events.

## Files

- `cam_to_sd.py` — driver; `results_*.csv`, `run_*.log` — every run above.
- `sample_rgb_path_b1692.jpg`, `sample_yuv_path_b1692.jpg` (striped, bug),
  `sample_yuv_fixed_b1693_*.jpg`, `ab_*_cam{0,1}_*.jpg`, `ab_rgb_vs_yuv_2x2.jpg`.
- `sample_planes_host_decode_*.jpg` — host decode of the dumped planes (dark
  lab, lights were off: Y max 38 — the "near-black Y plane" was the scene).

## Next

- W24-T3: Edge TPU inference on the YUV420 dual-input models (EXP-s257).
- W24-T4: C recorder task only if a use case needs more than ~12 saves/s at
  full consumed rate; the encode must then run off the consumer loop (double
  buffer) — the card has 10× headroom.
- W24-T5: reproduce the cam1 AWB stall.

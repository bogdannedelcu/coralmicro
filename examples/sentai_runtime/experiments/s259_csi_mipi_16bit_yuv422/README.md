# EXP-s259 — Camera frames at 2 bytes/pixel on the RT1176 (OP-S10-W24-T14)

**Board** 58eb (rev 1.1), **builds** 1723–1732, **date** 2026-11-01, one camera only
(the active camera; no MUX switching in any run).

**Question** (operator): ERR051248 is said to force 4 B/px (XRGB8888 / XYUV8888) for
every MIPI capture on this SoC.  Is there really no workaround?  Could a RAW / generic
8-bit tag, another sensor format, or the CSI register bits the SDK never programs cut
the camera->SDRAM write (4.9 MB per SXGA frame) and the PXP read of the same bytes —
the SDRAM bottleneck measured in s258 (PXP repack 62 ms at SXGA, camera 30 fps)?
Target: Y at SXGA plus U and V at VGA (= YUV 4:2:0) for the Edge TPU.

## Result in one paragraph

**Yes.**  The OV5640 "YUV420 (for MIPI only)" output (`0x4300 = 0x5F`, CSI-2 legacy
YUV420 8-bit, 1.5 B/px on the wire) passes the VIDEO_MUX reformatter and the 24-bit
CSI receiver stores it, with the receiver width set to half the sensor width, as
**2 bytes/pixel**: per pixel pair `[C, Y1, Y2, 0]`, where `C` is U on even rows and V
on odd rows.  That is the exact 4:2:0 content (full-resolution luma, 2x2 chroma), i.e.
the TPU model's Y plane and UV plane, in half the CSI traffic.  Verified pixel-exact
with the sensor colour bars (8 bars, rows identical) and with natural scenes at
640x480 (VGA30/45/60 rows) and at 1280x960 (SXGA15/30 rows).  The SXGA rows need one
extra sensor setting: **`0x460C = 0x22`, `0x3824 = 0x01`** (DVP PCLK divider manual,
divide by 1); with the SDK's "auto" divider the SXGA rows deliver the right bytes in
the wrong order.  Everything else (RAW8 / generic-8 tags, the CSICR18 "data from MIPI"
bits, the CSI gray-scale converter, Y8, the 16-bit receiver) is a dead end — details
below.

| capture | CSI write per frame | PXP read | stored layout |
|---|---|---|---|
| today: YUV422 -> XYUV8888 | VGA 1.23 MB, SXGA 4.9 MB | same | `[V,U,Y,0]` per pixel |
| **packed YUV420** | VGA 0.61 MB, SXGA 2.46 MB | 2 B/px (or none, see below) | `[C,Y1,Y2,0]` per pixel pair, C = U/V by row |

## What the documents say (all read in full this session)

* **ERR051248** (IMXRT1170ACE Rev 1.6 §2.22): "raw data and YUV422 (10 bit) formats
  to the MIPI_CSI2 block are not supported" (Video Mux Controller issue).  YUV422 8-bit,
  YUV420 8-bit and RGB are supported formats — the errata never made them 4 B/px.
* **RT1170 RM Rev 3 §46.3.3** (MIPI CSI-2 RX `data_out`): YUV422 8-bit → 16-bit
  half-pixels (`[7:0]` U/V, `[15:8]` Y); Legacy YUV420 8-bit (DT 0x1A) → `[7:0]` U/V,
  `[15:8]` Y1, `[23:16]` Y2 (two pixels per clock, UYY/VYY lines); RGB888 → 24-bit.
  Unsupported types are output as the whole packet minus CRC.
* **RM §50 VIDEO_MUX**: the pixel reformatting block feeds a 24-bit pixel link into
  the CSI parallel input (`VID_MUX_CTRL[CSI_SEL]=1`).  Registers PLM_CTRL, YUV420_CTRL
  (`FST_LN_DATA_TYPE`), CFG_DT_DISABLE.
* **RM §45.6.16 CSI_CR18**: DATA_FROM_MIPI (bit 22), MIPI_YU_SWAP (21),
  MIPI_DOUBLE_CMPNT (20), MIPI_DATA_FORMAT (30:25) — the i.MX7/8MM recipe
  (`imx7-media-csi.c`: TWO_8BIT_SENSOR + DATA_FROM_MIPI + YUV422_8B + DOUBLE_CMPNT)
  stores YUYV at 2 B/px on those SoCs.  Never programmed by the SDK or this firmware.
* **CSI_CR20**: gray-scale mode (QRCODE_EN, QR_DATA_FORMAT) converts YUV input to Y8;
  10BIT_NEW_EN selects data[7:0] instead of data[9:2] for 8-bit input.
* **OV5640 datasheet §7.15 FORMAT CONTROL 0x4300[7:4]**: 0 RAW, 1 Y8, 2 YUV444 (not at
  full resolution), 3 YUV422 (SDK: 0x3F = UYVY), 4 YUV420, 5 **YUV420 (for MIPI only)**
  (0x5E VYY/UYY, 0x5F UYY/VYY), 6 RGB565.  `0x460C` VFIFO CTRL0C bit 1 = "DVP PCLK
  divider control by 0x3824[4:0]" (manual) vs auto.  `0x3035[3:0]` "scale divider for
  MIPI: MIPI PCLK/SERCLK can be slowed down".  `0x4814` is reserved in the datasheet.

## Firmware hooks (shipped, default off, diagnostics only)

* `sentai.camera.csi_mode(bus, cr18_or, cr20_or, wdiv)` — CSI receiver bus 24 (4 B/px,
  default) / 16 (2 B/px) / 8 (1 B/px); raw CSICR18 / CSICR20 bits OR'd in after
  `CSI_Init()` (which rewrites CR18) and before `CSI_Start()`; `wdiv` 2 halves the
  receiver width (two pixels per pixel-link cycle).
* `sentai.camera.csi_geom(w_cycles, h_lines)` — receiver-only geometry override, so the
  sensor ISP output size (0x3808..0x380B) can be re-programmed after init.
* `sentai.camera.csi_settle(v)` — MIPI D-PHY T-HSSETTLE override for the next init.
* `sentai.camera.save_raw(path, offset, nbytes)` — dump a slice of the latest raw frame
  buffer (whatever the receiver stored) to the user partition; the host pulls it over
  `/api/raw` and rebuilds the image (`grab_packed.py`).
* Downstream consumers (PXP, JPEG, flow, pipeline) still assume XYUV 4 B/px; nothing
  in the production path changes.  Implementation: `libs/camera/camera_support.c`
  (`g_sentai_csi_*`, `BOARD_InitCamera`, `BOARD_InitMipiCsi`), bindings in
  `modsentai_camera.c`, `sentai_cam_save_raw` in `sentai_runtime.cc`.

## Runs (chronological)

### 1. `run_csi16.py` — receiver bus width and the CR18 MIPI bits (`csi16_b1723_*.log`)

| var | bus | CR18 OR | frames | colour bars, row 0 |
|---|---|---|---|---|
| L | 24 | — | yes | lanes [V-like, U-like, **Y**, 0] |
| A | 16 | DATA_FROM_MIPI+FMT(1E)+DOUBLE | yes | lane0 = **V**, lane1 = **U**, rows identical |
| B | 16 | A + YU_SWAP | yes | byte-identical to A |
| C | 16 | DATA_FROM_MIPI+FMT(1E) | yes | byte-identical to A |
| D | 16 | none (the 2026-04-25 configuration) | yes | byte-identical to A |
| E | 8 | DATA_FROM_MIPI+FMT(1E), 2 cycles/px | none | — |

* The CR18 MIPI bits do nothing on this SoC (A = B = C = D byte for byte): the CSI
  only sees the VIDEO_MUX pixel link on its parallel input.
* The 16-bit receiver works (full rate, 8 bars of 80 px, rows identical) but samples
  pixel-link bits [15:0] = {U,V}; Y is on [23:16].  The April "2x2-tile garbage" was
  the UV plane.

### 2. `dt_sweep.py` / `dt_probe.py` — tagging the stream as RAW8 / generic-8 / RGB565

Writing `0x4814[5:0]` changes nothing downstream (cfg 5–8 byte-identical to the
legacy / 16-bit captures); the 8-bit receiver with 2 cycles per pixel never completes
a line.  `dt_probe.py` used the RX payload-disable bits as a data-type detector:
disabling YUV422_8BIT (`CFG_DISABLE_PAYLOAD_0` bit 14) stops the frames (the tag is
0x1E) and no 0x4814 value brings them back → 0x4814 is not a data-type override.
Real sensor RAW mode (0x4300=0x00, 0x501F=0x01, 8-bit) does stream through
VIDEO_MUX as `[R,0,0,0]` per pixel at full rate (Bayer alternation visible), so
"not supported" in ERR051248 is not "no data"; but Bayer at 1 B/px needs a demosaic
nobody has, so it was not pursued.

### 3. `fmt_sweep.py` / `y8_sweep.py` — sensor formats with fewer bits (`fmt_b1723`, `y8_b1724`)

| sensor 0x4300 | receiver | result |
|---|---|---|
| 0x10 Y8 | 24-bit | `[Y,0,0,0]`, full rate; 16-bit: `[Y,0]`; 8-bit: 1 B/px but data[9:2] (= Y>>2; 10BIT_NEW_EN has no effect) |
| 0x10 Y8 vs legacy Y | — | **not luma**: corr 0.23 with the YUV422 Y lane of the same scene, bars not monotonic — some other ISP tap |
| CR20 gray-scale converter | 8 / 24-bit | no frames (CSI stalls) |
| 0x4F YUV420 (DVP style) | 24 / 16-bit | no frames |
| **0x5F / 0x5E YUV420 (MIPI only)** | 24-bit, width/2 | **full rate, `[C,Y1,Y2,0]` per pixel pair** (bars: U row / V row alternate, Y1 = Y2 = legacy Y values) |
| 0x5F | 16-bit, width/2 | `[C,Y1]` per pixel pair = 1 B/px, half-horizontal luma + chroma |

### 4. `grab_packed.py` — whole frames, visual check (build 1725)

VGA packed YUV420 vs legacy XYUV of the same scene: image correct (colours, geometry;
`p420_640x480_b1725.png` vs `xyuv_640x480_b1725.png`, note the legacy capture is
rotated 180° by the default vflip).  SXGA packed: scrambled stripes
(`p420_1280x960_b1725.png`).

### 5. `sxga_packed_probe.py`, `w420_sweep.py` — why SXGA breaks (builds 1725–1731)

* The SXGA bytes are the right *values* (colour-bar Y/U/V levels) in the wrong
  *places*: lane 3 is always 0 (alignment holds), but all three data lanes carry Y and
  chroma mixed; the byte stream autocorrelates at 8 and 64 B; rows look like rotated
  `[C,Y,Y]` triplets.  Not a transport (D-PHY) problem: MIPI RX error registers clean,
  frame rate full.
* Not the width: the same 800x600 output (ISP scaler re-sized with `csi_geom`) is
  perfect on the VGA30/45/60 timing rows and scrambled on the SXGA row.
* Not the MIPI lane rate: VGA90 (same PLL row as SXGA30: 0x3035=0x21) scrambles at
  640x480; an SXGA row with PLL 336 MHz (lane 336 Mbps) still scrambles; VGA90 timing
  with MIPI divider 2 (fake fps 91) is clean but also halves PCLK.  (Experimental
  fsl_ov5640.c rows 31–35/91 were used for this and removed again.)
* **The knob: `0x460C` bit 1 (manual DVP PCLK divider) + `0x3824`.**  The VGA rows
  use 0x22 / 0x02, the SXGA rows 0x20 (auto) / 0x04.  On the SXGA30 row:

| 0x460C / 0x3824 | 422 before switch | 420 frames | bars row-to-row corr |
|---|---|---|---|
| 0x20 / 0x04 (SDK) | 29/s | 26/s | −0.13 (scrambled) |
| 0x22 / 0x02 | 30/s | 26/s | −0.26 |
| 0x22 / 0x04 | 29/s | 26/s | −0.14 |
| **0x22 / 0x01** | 29/s | 26/s | **1.00** (clean, `p420_sxga_w1280x960_b1731_vf22d1.png`) |
| 0x22 / 0x08 | 29/s | 26/s | −0.25 |

  SXGA15 row with 0x22 / 0x01: clean as well.  Natural scene at SXGA30
  (`p420_sxga_w1280x960_b1731_nat_d1.png`, dark room) row-to-row corr 0.99; final
  re-check on build 1732 (`b1732_final`).

## Why this matters for the TPU path

* SDRAM traffic per SXGA frame today: CSI 4.9 MB + PXP read 4.9 + PXP write 1.84 +
  EHCI read 1.84 = 13.5 MB (s258).  With the packed capture: CSI 2.46 + repack read
  2.46 + write 1.84 + EHCI 1.84 = 8.6 MB, or 2.46 + 2.46 = 4.9 MB if the model eats the
  packed layout directly (then USB carries 2.46 MB instead of 1.84, +18 ms per invoke).
* The repack to the model's planar Y + UV is not a PXP job: no PXP input format
  matches `[C,Y1,Y2,0]`.  Options for the follow-up task (proposed W24-T16):
  (a) retrain the stem on the packed 4-channel H x W/2 input (zero-copy from the
  camera buffer; at VGA the input grows 461 -> 614 KB); (b) PXP as an ARGB8888 ->
  RGB888P byte-dropping copy = 1.5 B/px `[C,Y1,Y2]` for a 3-channel stem (USB bytes
  identical to today); (c) an M7 SIMD repack to planar (memory-bound, ~15–20 ms at
  SXGA).  Any of them removes the 4 B/px CSI write and the XYUV PXP read, which were
  the SDRAM contention of s258.
* Production integration needs: sensor row with 0x4300=0x5F (+ 0x460C/0x3824 on
  SXGA) from init, receiver width/2 in `BOARD_InitCamera`, a frame-layout flag for
  every consumer (JPEG, flow, markers, PXP paths), and AEC/AWB re-validation (the
  format switch resets vflip; the ISP statistics still run).

## Dead ends (do not retry)

CSICR18 DATA_FROM_MIPI / MIPI_DOUBLE_CMPNT / MIPI_YU_SWAP / MIPI_DATA_FORMAT (no
effect); 0x4814 as data-type override (none); 8-bit receiver with 2 cycles per pixel
(no frames); CSI CR20 gray-scale converter (stalls); OV5640 Y8 (not luma); DVP-style
YUV420 0x4F (no frames); live PLL re-clocking (stream dies; needs rows from init);
MIPI divider / lane-rate theories for the SXGA scramble (the PCLK divider was it).

## Future direction recorded by the operator (not run here)

**CSI line ring in OCRAM + PXP per slice (proposed OP-S10-W24-T15).**  The PXP works
in 8-line blocks from arbitrary addresses, so it could follow the CSI writer slice by
slice; SDRAM traffic only drops if the CSI writes its small ring into OCRAM (then both
the CSI write and the PXP read leave the SDRAM).  Blocker to test: the CSI DMA
alternates FB1/FB2 after "a frame" defined by IMAG_PARA; declaring IMAGE_HEIGHT =
slice height may make it alternate continuously mid-frame (RM is silent on whether
the DMA restarts before the next SOF; the RT1050 "CSI->LCDIF buffer lines" mode no
longer exists on RT1170).

## Files

Scripts: `run_csi16.py`, `dt_sweep.py`, `dt_probe.py`, `fmt_sweep.py`, `y8_sweep.py`,
`grab_packed.py`, `sxga_packed_probe.py`, `w420_sweep.py`.  Logs: `csi16_b1723_*`,
`dt_b1723.*`, `dtprobe_b1724.*`, `fmt_b1723.*`, `y8_b1724.*`, `grab_b1725.log`,
`sxgaprobe_*`, `w420_*`.  Images: `xyuv_*.png` (legacy), `p420_*.png` (packed YUV420
rebuilt on the host), `p420_*.bin` (raw packed frames).

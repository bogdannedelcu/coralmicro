---
name: project_packed_yuv420_capture_2bpp_w24_t14
description: "2026-11-01 — ERR051248 workaround FOUND (EXP-s259, W24-T14): OV5640 \"YUV420 for MIPI only\" (0x4300=0x5F) + 24-bit CSI receiver at half width stores exact 4:2:0 content at 2 B/px ([C,Y1,Y2,0] per pixel pair, U/V rows alternate); SXGA rows need 0x460C=0x22 + 0x3824=0x01; CSICR18 MIPI bits, 0x4814, Y8, CR20 gray-scale, 8-bit receiver are dead ends"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-01T07:01:17.155Z
---

**EXP-s259 / OP-S10-W24-T14 (builds 1723–1732, board 58eb, one camera).**

- **Works:** sensor `0x4300 = 0x5F` (CSI-2 legacy YUV420 8-bit, UYY/VYY lines,
  1.5 B/px on the wire) through VIDEO_MUX into the 24-bit CSI receiver with the
  receiver width = sensor width / 2 → memory `[C, Y1, Y2, 0]` per pixel pair, C = U on
  even rows, V on odd rows.  Pixel-exact on colour bars and natural scenes at VGA
  (30/45/60 rows) and SXGA (15/30 rows).  CSI write halves (VGA 1.23 → 0.61 MB, SXGA
  4.9 → 2.46 MB).  **SXGA rows additionally need `0x460C = 0x22` + `0x3824 = 0x01`**
  (manual DVP PCLK divider ÷1); with the SDK's auto divider the bytes are right but
  scrambled (rotated [C,Y,Y] triplets, autocorr at 8/64 B).  Not the width, not the
  MIPI lane rate, not the PLL (all tested).
- **ERR051248** (IMXRT1170ACE §2.22) only says RAW and YUV422-10bit from MIPI are
  unsupported; it never forced 4 B/px.  The 4 B/px came from the VIDEO_MUX pixel
  reformatter (YUV422 → 24-bit YUV444 pixel link, memory order [V,U,Y,0]).
- **Dead ends (don't retry):** CSICR18 DATA_FROM_MIPI/DOUBLE_CMPNT/YU_SWAP/
  MIPI_DATA_FORMAT have no effect on RT1176 (CSI sees only the pixel link);
  16-bit receiver stores pixel-link bits [15:0] = {U,V} (the April "2x2 garbage" was
  the UV plane); 8-bit receiver reads data[9:2] (10BIT_NEW_EN no effect) and needs
  one cycle per pixel; 0x4814 is NOT a data-type override (payload-disable probe:
  tag stays 0x1E); OV5640 Y8 (0x10) is not luma (corr 0.23); CR20 gray-scale mode
  stalls the CSI; DVP-style YUV420 (0x4F) gives no frames; live PLL/0x3035
  re-clocking kills the stream (needs rows from init).  Real RAW8 Bayer does pass
  VIDEO_MUX as [R,0,0,0] (so "unsupported" ≠ "no data").
- **No PXP input format matches the packed layout** → TPU path options (W24-T16):
  retrain the stem on H×W/2×4 packed input (zero-copy), PXP ARGB8888→RGB888P
  byte-drop to 1.5 B/px for a 3-channel stem, or M7 SIMD repack to planar.
- Hooks shipped (default off): `camera.csi_mode(bus, cr18_or, cr20_or, wdiv)`,
  `csi_geom(w,h)`, `csi_settle(v)`, `save_raw(path, off, n)` + host `grab_packed.py`
  (`/api/raw` slices < 1 MB).  Operator's W24-T15 idea: CSI line ring in OCRAM +
  PXP per slice (blocker: does the CSI DMA restart mid-frame).
- Lesson: colour bars + row-to-row correlation + lane statistics on the raw buffer
  found every layout in one run each; "garbage" twice turned out to be real data in
  another arrangement.

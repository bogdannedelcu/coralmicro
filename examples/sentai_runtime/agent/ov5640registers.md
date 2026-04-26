# OV5640 register reference — sentai_runtime usage

Compiled from datasheet excerpts + empirical use during the
2026-04-26 cam_id / synthetic-pattern sessions.  All access is
8-bit I2C reads/writes via `sentai.camera.reg_read(cam_id, reg)`
and `sentai.camera.reg_write(cam_id, reg, val)`.  Slave address
of OV5640 = 0x3C (7-bit) on both I2C1 and I2C2 (each camera on
its own bus on Coral Dev Board Micro).

---

## System control

| Reg | Name | RW | Default | Notes |
|-----|------|----|---------|-------|
| 0x3008 | SYSTEM_CTRL0 | RW | 0x02 | bit 7 = SOFTWARE_RESET (set, then write 0x02 normal); bit 6 = standby |
| 0x3103 | SCCB_PROGRAM | RW | 0x03 | NXP init writes 0x11 + 0x3008=0x82 for full reset, then 0x03 |

**Common values:**
- `0x3008 = 0x82` → software reset (write before init reload)
- `0x3008 = 0x42` → standby (sensor stops streaming, MIPI clock idle)
- `0x3008 = 0x02` → normal operation

---

## AEC / AGC (auto-exposure / auto-gain)

| Reg | Name | RW | Default | Notes |
|-----|------|----|---------|-------|
| 0x3500 | EXPOSURE_HIGH | RW | 0x00 | Exposure[19:16] |
| 0x3501 | EXPOSURE_MID  | RW | 0x00 | Exposure[15:8]  |
| 0x3502 | EXPOSURE_LOW  | RW | 0x00 | Exposure[7:0] (lines × 16) |
| 0x3503 | AEC_PK_MANUAL | RW | 0x00 | bit 0 = AEC manual; bit 1 = AGC manual |
| 0x3508 | GAIN_HIGH     | RW | 0x00 | Gain[9:8] (1/16 step) |
| 0x3509 | GAIN_LOW      | RW | 0x10 | Gain[7:0] (combined 10-bit, default = 1.0×) |

**Manual mode usage (synthetic pattern):**
- BLACK rail: 0x3503=0x03, exposure=0x000010, gain=0x010 → bytes < 0x20
- WHITE rail: 0x3503=0x03, exposure=0xFFFFF0, gain=0x3FF → bytes > 0xE0

---

## AEC target stable / fast zones

Used by `sentai_cam_aec_set(cam_id, high, low)`:

| Reg | Name | Notes |
|-----|------|-------|
| 0x3A0F | WPT (stable range high) | high = ~target ÷ 255 × 256 |
| 0x3A10 | BPT (stable range low)  | low ≤ high |
| 0x3A11 | VPT (fast zone high)    | high + 0x40 (capped at 0xFF) |
| 0x3A1B | WPT2 | mirror of 0x3A0F |
| 0x3A1E | BPT2 | mirror of 0x3A10 |
| 0x3A1F | VPT2 | low − 0x20 (floor 0) |

**Reference points:**
- NXP default: high=0x30 low=0x28 → ~18% target (dim indoors)
- OmniVision AN: high=0x78 low=0x68 → ~45% target (general)
- Bright room: high=0x60 low=0x50 → ~35% target

---

## AGC gain ceiling

Used by `sentai_cam_gain_ceiling_set(cam_id, ceiling)`:

| Reg | Name | Notes |
|-----|------|-------|
| 0x3A18 | GAIN_CEILING_HIGH | bits[1:0] = ceiling[9:8] |
| 0x3A19 | GAIN_CEILING_LOW  | ceiling[7:0] |

**Ceiling value (10-bit, 1/16 step):**
- 0x07C = NXP default ≈ 7.75× (too low for dim indoor)
- 0x0F8 ≈ 15.5×
- 0x1F0 ≈ 31× (recommended low-light, more noise)
- 0x3FF = max (~62.9×)

---

## Test pattern (synthetic injection)

| Reg | Name | RW | Default | Notes |
|-----|------|----|---------|-------|
| 0x503D | PRE_ISP_TEST_SETTING_1 | RW | 0x00 | bit 7 = enable; bit[3:2] = style |

**Bit[3:2] color-bar style:**
- `00` = standard 8 color bar (vertical: WHITE, YELLOW, CYAN, GREEN, MAGENTA, RED, BLUE, BLACK)
- `01` = gradual change at vertical mode 1
- `10` = gradual change at horizontal
- `11` = gradual change at vertical mode 2

**Empirical caveat (sentai 2026-04-26):** the test-pattern stream is
processed by ISP (AWB + CMX) before output, so leftmost WHITE bar
does NOT come out as `0xFF, 0xFF, 0xFF` — observed ~`0xCF, 0xAF, 0xC0`
on cam0.  For exact-byte predictability we use AGC-manual rails
instead (see synthetic test pattern API in
`sentai_runtime.cc:sentai_cam_test_pattern`).

---

## Embedded data line (sensor metadata in pixel stream)

| Reg | Name | RW | Default | Notes |
|-----|------|----|---------|-------|
| 0x501F | FORMAT_MUX_CTRL | RW | 0x00 | bit 7 = embedded data enable; bits[6:0] = ISP format |
| 0x4707 | EMBEDDED_LINE_COUNT | RW | varies | Number of embedded data lines at top of frame |

**Empirical caveat:** enabling 0x501F bit 7 + 0x4707 = 1 did NOT
produce structured metadata bytes in the captured framebuffer —
ISP appears to overwrite the embedded line region.  Investigation
needed to bypass ISP, OR may not be wired up in this firmware's
CSI/MIPI receiver path.  See conversation 2026-04-26.

---

## Resolution / windowing

| Reg | Name | Notes |
|-----|------|-------|
| 0x3800/01 | X_ADDR_ST_H/L | crop start X |
| 0x3802/03 | Y_ADDR_ST_H/L | crop start Y |
| 0x3804/05 | X_ADDR_END_H/L | crop end X |
| 0x3806/07 | Y_ADDR_END_H/L | crop end Y |
| 0x3808/09 | DVP_HORIZONTAL_OUTPUT | output width  |
| 0x380A/0B | DVP_VERTICAL_OUTPUT   | output height |
| 0x380C/0D | TIMING_HTS | total horizontal size (line length) |
| 0x380E/0F | TIMING_VTS | total vertical size (frame height incl. blanking) |

**Used by NXP CSI driver via `OV5640_GetResolutionParam` →
table-driven write of resolution params.**

---

## PLL / clock

| Reg | Name | Notes |
|-----|------|-------|
| 0x3035 | SC_PLL_CTRL1 | clock divider |
| 0x3036 | SC_PLL_CTRL2 | PLL multiplier |
| 0x3824 | PCLK_DIV    | pixel clock divider |
| 0x4837 | PCLK_PERIOD | MIPI period |

**Used by `OV5640_GetClockConfig` → fps × resolution → clock params.**

Sentai patches in
`patches/coralmicro-rt1176-sdk/0001-ov5640-vga-30fps-pclkperiod.patch`:
- VGA45: pllCtrl1=0x14, pllCtrl2=0x68, pclkPeriod=0x14
- VGA60: pllCtrl1=0x14, pllCtrl2=0x70, pclkPeriod=0x0c
- VGA90: pllCtrl1=0x21, pllCtrl2=0x54, pclkPeriod=0x0a (DEAD-END:
  110 MB/s SDRAM saturates SEMC under TPU load)
- SXGA15/30 added 2026-04-24 (SXGA support session)

---

## MIPI CSI-2 interface

| Reg | Name | Notes |
|-----|------|-------|
| 0x4814 | MIPI_CTRL14 | DT (Data Type) override; default = 0 (auto) |

**Use case:** distinct virtual channel ID per camera could route
each camera to its own buffer pool in the CSI-2 receiver — but
RT1176 CSI receiver doesn't currently dual-VC route via NXP driver.

---

## ISP / color matrix (Color Module Control)

| Reg range | Module | Notes |
|-----------|--------|-------|
| 0x5180-0x5188 | AWB control / pre-gain | Auto white balance gains per channel |
| 0x5380-0x539B | CMX (Color Matrix) | RGB → YUV transform coefficients + offsets |
| 0x5500-0x5510 | DPC (Defect Pixel Correction) | |
| 0x5800-0x583F | LENC (Lens shading correction) | per-channel gain map |

**Setting these to bypass values during synthetic pattern would
allow exact OV5640 test-pattern bytes to reach the framebuffer —
not currently implemented.**

---

## OV5640 chip ID

| Reg | Name | Notes |
|-----|------|-------|
| 0x300A | CHIP_ID_HIGH | should read 0x56 |
| 0x300B | CHIP_ID_LOW  | should read 0x40 |

NXP `OV5640_Detect` reads these to confirm sensor presence on
each I2C bus.

---

## Common manipulation idioms (sentai)

```python
# Read a register
val = sentai.camera.reg_read(cam_id, 0x503D)

# Write a register
sentai.camera.reg_write(cam_id, 0x503D, 0x80)  # enable color bars

# Software reset (then re-init via init() to reload defaults)
sentai.camera.reg_write(cam_id, 0x3103, 0x11)
sentai.camera.reg_write(cam_id, 0x3008, 0x82)
# then reload by sentai.camera.stop() + init(1)

# Preset bundle (bright_indoor / nxp_stock / daylight / low_light)
sentai.camera.isp_preset(cam_id, "bright_indoor")

# Synthetic test pattern via AGC rails
sentai.camera.test_pattern(cam_id, 1)  # BLACK rail
sentai.camera.test_pattern(cam_id, 2)  # WHITE rail
sentai.camera.test_pattern(cam_id, 0)  # restore auto
```

---

## References

- OV5640 Camera Module Hardware Application Note (Omnivision public)
- NXP MIMXRT1170RM rev. K — CSI / MIPI CSI-2 RX chapters
- `third_party/nxp/rt1176-sdk/components/video/camera/device/ov5640/fsl_ov5640.c` — vendor init table + clock configs
- `libs/camera/camera_support.c` — sentai-side OV5640 wiring (power, reset, MUX, init mode dispatch)
- `examples/sentai_runtime/sentai_runtime.cc` — sentai application-layer wrappers (`sentai_cam_aec_set`, `sentai_cam_gain_ceiling_set`, `sentai_cam_isp_preset`, `sentai_cam_test_pattern`)

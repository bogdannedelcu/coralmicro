---
name: project_ov5640_exposure_calc_todo
description: OV5640 exposure vs fps — CORRECTED 2026-11-02: the AEC budget is in LINES (max = VTS-4 = 980 of 984 lines) so it already equals one frame period at every fps; what does NOT scale with fps are the 50/60 Hz banding steps (0x3A08-0x3A0B computed for the VGA30 line time) — that is the remaining TODO
metadata:
  type: project
  originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
  modified: 2026-11-02T04:42:30.412Z
---

**Measured 2026-11-02 (build 1763, VGA30, both cameras):** `0x3503 = 0x00` (AEC + AGC auto),
exposure `0x3500-02 = 0x003D40` → 980 lines, VTS `0x380E/F = 0x03D8` = 984, HTS 1896, AGC only
2.1× / 2.0× under the lab light, night mode off (`0x3A00 = 0x58`, bit 2 clear → the frame rate
never drops), gain ceiling 15.5× (`0x3A18/19 = 0x007C`), `0x3A02/03 = 0x3D80`, `0x3A14/15 = 0x0E40`.

**Why the old premise (2026-04-25, "SXGA15 exposure wrong by 3x") was wrong:** the AEC exposure
cap is in sensor LINES and is clamped at VTS − 4 while night mode is off.  Our fps rows (SDK
patches 0001/0007) change the PLL / PCLK and keep VTS = 984, so the line time scales with fps and
the maximum exposure is automatically one frame period in every mode (33 ms VGA30, 22 ms VGA45,
16.7 ms VGA60, 67 ms SXGA15).  Nothing to recompute for the exposure budget — it is maximised by
construction.

**Remaining TODO (real):** the anti-flicker band steps are in lines for the VGA30 line time only:
B50 `0x3A08/09 = 0x0127` = 295 lines = 10.0 ms, B60 `0x3A0A/0B = 0x00F6` = 246 lines = 8.33 ms at
33.9 µs/line.  At VGA45/60 and SXGA15 the line time differs, so under mains-powered or PWM lamps
the flicker filter steps the exposure in the wrong quanta.  Fix: after init at fps f, write
B50/B60 = round((1/100 s, 1/120 s) / line_time(f)) and 0x3A0D/0E max bands = floor(VTS / step).
Irrelevant under daylight / DC light.

**How to apply:** when a scene flickers at VGA45/60 or SXGA, suspect the band steps first, not the
exposure cap.  `isp_preset()` only sets the AEC target window + gain ceiling; `aec_set()` the window.

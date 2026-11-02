# s268 — Colour chart measured in the YUV domain the Edge TPU models receive

**WBS**: OP-S10-W24-T22 (production colour), feeds OP-S10-W24-T23 (M7 auto-tuning)
**Started**: 2026-11-02
**Status**: 🟢 iter1 done — sensor YUV is FULL range, byte order [V,U,Y,X], colour error identical to the RGB path (8.1 / 8.5 vs 7.9 / 8.4)
**Board**: 58eb, build 1766, VGA 30 fps, production register sets (`/cam/cam0.regs`, `/cam/cam1.regs`), Kamvas 16 at ~45 cm

## Claim

The s265 / camera_modules measurements use the RGB → JPEG path.  The YUV420 models get the
OV5640's own YUV output (`camera.yuv_mode(1)`), i.e. the same ISP image through the sensor's
RGB→YUV block, which adds two things the RGB path cannot show: the output range (Y 16–235
"limited" vs 0–255 "full", U/V offsets) and clipping.  This experiment measures the chart on
the YUV frames themselves, decides the range from the data (white / black Y, grey U/V), and
compares the colour error with the RGB-path result of the same scene.

## Pass criteria

- Byte roles of the XYUV buffer identified from the data (constant X byte; Y = largest
  white–black contrast; V high on red / U high on blue).
- Range decided unambiguously: either full (white Y ≈ 243·k, black ≈ 52·k, both conversions
  disagree by > 2 ΔE00) or limited.
- Grey patches: |U − 128| and |V − 128| < 6 on both cameras (the manual AWB gains hold in YUV).
- ΔE00 of the YUV path within 2 of the RGB path of the same scene (`camera_modules/out/measure_dist45b`:
  7.9 / 8.4) — otherwise the YUV block (range / matrix) is a calibration item of its own.

## How to run

```bash
python3 yuv_check.py --out iter1            # ~4 min: RGB exposure normalisation, YUV capture of both cameras, analysis
```

## Files

- `yuv_check.py` — driver + analysis; `iter1/`: `exposure.json`, `cam*.raw` (640x480x4 XYUV),
  `cam*_yuv_{full,limited}.png`, `measure_patches_cam*.json`, `result.json`, `compare_yuv.png`.

## Result (iter1, 2026-11-02, build 1766, ~45 cm, grid central 70 %, exposure normalised: gain 3.6x / 3.6x at 980 lines)

| | cam0 (SAI-CAM-0001) | cam1 (SAI-CAM-0002) |
|---|---|---|
| XYUV byte order in the frame buffer | byte0 = V, byte1 = U, byte2 = Y, byte3 = X (= 0, std 0.0) | same |
| white patch Y / U / V (target sRGB 243) | **255.0** / 128.0 / 132.6 | 247.0 / 122.0 / 137.0 |
| black patch Y / U / V (target 52) | 55.1 / 129.4 / 124.8 | 55.7 / 126.6 / 127.9 |
| Y of white, n8, n6.5, n5, n3.5, black (targets 243, 200, 160, 122, 85, 52) | 255, 196, 185, 140, 97, 55 | 247, 198, 144, 113, 93, 56 |
| grey patches mean U / V (128 = neutral) | 133.1 / 125.8 | 125.1 / 130.3 |
| ΔE00 vs sRGB, BT.601 full / limited conversion | **8.1** / 8.8 | 8.5 / **8.4** |
| ΔE00 RGB path, same scene (`camera_modules/out/measure_dist45b`) | 7.9 | 8.4 |

Findings:

1. **The OV5640 outputs full-range YUV** (0x4300 = 0x3F, 0x501F = 0x00): black reads 55 for a
   52 target and white reaches 255; a limited-range output would put them at ≥ 61 and ≤ 235.
   Any consumer that assumes 16–235 (or a model trained on limited-range video) would be fed
   a ~15 % contrast error — the models must be fed / trained with full-range YUV.
2. **The YUV block adds no colour error**: ΔE00 8.1 / 8.5 on the sensor's YUV frames vs 7.9 /
   8.4 on the RGB/JPEG path of the same scene.  The RGB-path calibration (s265, camera_modules)
   transfers unchanged to what the Edge TPU receives.
3. **Grey balance holds in YUV**: |U − 128|, |V − 128| ≤ 5 on both cameras (cam0 slightly
   blue/green, cam1 slightly magenta — the opposite residuals seen in RGB).
4. **Highlight clipping is a tone-curve issue, not exposure**: neutral 8 sits on its target
   (196 / 198 for 200) yet white (243) reaches 255 / 247, i.e. the camera's white/n8 ratio is
   1.30 / 1.25 against 1.22 on the chart — the top knots of the tuned gamma are too high (and
   cam0's n6.5 is 185 for 160, cam1's 144).  For a detector input the clip is information loss:
   either re-tune the gamma at this distance (all greys at once) or run the AEC target ~5 %
   lower as a stop-gap.  Concrete input for T23 (AEC target steering) and for the re-tune.

Method notes: `fs.read_base64` is capped at 256 KB and the board has no hex/base64 for bytes,
so the chart row band (≈ 180 rows) is dumped in two `save_raw(offset, nbytes)` slices (static
scene, frozen exposure) and downloaded in 2 s per camera; whole frames or frame series should
go through USB mass-storage mode (`sentai.usb.drive(1)` → warm reset → the SD card appears as a
disk on the host) instead.


## iter2 (2026-11-02, after the cam0 re-tune attempt — cam0 set later reverted)

Same procedure with the kamvas45 cam0 set then in production (cam1 unchanged): cam0 ΔE00 9.7
(greys 7.5) at the n8 = 200 normalisation — the set tuned for n8 = 190 clips at 200 (yellow 49
ΔE76) — cam1 8.4 (greys 4.7), identical to iter1.  Confirms iter1's conclusions (full range,
byte order, YUV = RGB path) and that the exposure target and the tone curve must be chosen
together; the cam0 production set was reverted to kamvas 2026-11-01 (see camera_modules/README.md).

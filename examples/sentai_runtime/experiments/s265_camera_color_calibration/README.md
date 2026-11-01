# s265 — Camera colour calibration against a monitor (both OV5640 cameras)

**WBS**: OP-S10-W24-T18 (new task, appended under W24)
**Started**: 2026-11-01
**Status**: 🟡 running
**Board**: 58eb (rev 1.1), build 1756, VGA 30 fps, legacy XYUV capture, JPEG q90

## Claim

Both SentAI cameras look at the same monitor (Philips 234E5, 1920x1080, HDMI-3 on the
RTX 4090).  A known colour grid shown on the monitor lets us measure each camera's colour
response and tune the OV5640 ISP (AWB gains, colour matrix, saturation, exposure) so that
(a) both cameras agree with each other and (b) the captured colours match the sRGB values
sent to the monitor.  The procedure is recorded step by step so it can be replayed as a
tuning script on other cameras of the same model (`tune_colors.py`, the deliverable).

## Pass criteria

- Grid patches located in both cameras' frames automatically (no manual pixel picking).
- Mean colour error per patch (CIEDE2000 or RGB RMSE) after tuning lower than before, on
  both cameras; the inter-camera difference on the 6 grey patches below 5 RGB levels.
- The tuning writes only sensor registers that survive as a replayable list
  (`iterN/tuned_regs_camN.json`); applying that list on a fresh boot reproduces the result.
- HW: board enumerated as 1fc9:c0a1, build counter 1756 unchanged (no firmware change
  needed unless a binding is missing).

## How to run

```bash
# 1. keep the monitor awake (the Unreal power saver blanks it every 3 s otherwise)
touch /tmp/mps_pause
# 2. show a pattern on the monitor
python3 show_pattern.py --grid          # 24-patch ColorChecker-like grid
python3 show_pattern.py --solid 808080  # uniform grey
python3 show_pattern.py --quit
# 3. capture one JPEG per camera + colour registers
python3 capture.py --out iter1 --tag grid_before
# 4. analyse / tune (see JOURNAL.md for the exact sequence)
```

## Files

- `board_repl.py` — prompt-synchronised REPL driver (no `time` module on the board).
- `capture.py` — JPEG per camera + AWB/AEC/SDE/CMX register dump → `<iter>/<tag>_cam<N>.jpg`, `<tag>_regs.json`.
- `show_pattern.py` — fullscreen GTK pattern on the Wayland session; the shown PNG and the
  patch geometry land in `patterns/`.
- `JOURNAL.md` — chronological log of every step, result and dead end (operator request:
  the experiment must be reproducible for other cameras of the same model).
- `iter1/` — captures and results of the first pass.

## OV5640 colour-pipeline registers: inventory vs. what the tuner touches

Source: OV5640 datasheet v2.03 (tables 7-13 BLC, 7-21 ISP top, 7-22 AWB, 7-24 CMX, 7-25
gamma, 7-26 SDE, 7-28 LENC) + the NXP SDK init table in `fsl_ov5640.c`.  Pipeline order
in the sensor: BLC → (AWB gain) → LENC → raw gamma → CMX → gamma (YST) → SDE → scale.

| block | registers | role | tuner stage | status |
|---|---|---|---|---|
| AEC / AGC | 0x3500–0x3502 exposure, 0x350A/B gain, 0x3503 manual bits | brightness | `expo` (gain; exposure left at its 30 fps maximum) | done |
| AWB gains | 0x3400–0x3405 R/G/B 12-bit (0x400 = 1x), 0x3406 manual | grey neutrality | `awb` | done |
| BLC | 0x4000–0x4005 control, **0x4009 black level target** (10-bit) | shadow floor | not yet | candidate: black (52) reads 17–28 |
| ISP enables | 0x5000 (bit7 LENC, bit5 raw gamma, bit2/1 pixel cancel, bit0 CIP), 0x5001 (bit7 SDE, bit5 scale, bit2 UV average, bit1 CMX, bit0 AWB) | which blocks run | read only | SDK: 0x5000=0xA7, 0x5001=0xA3 |
| AWB engine | 0x5180–0x519E advanced AWB | auto mode only | irrelevant once 0x3406 = 1 | — |
| CMX | 0x5380 mode (1.7/2.6), 0x5381–0x5389 magnitudes (CMX1–3 → Y, 4–6 → U, 7–9 → V), 0x538A/B sign bits | hue / saturation per axis | `cmx` (magnitudes; signs kept as SDK) | in progress |
| Gamma | 0x5480 control, 0x5481–0x548F YST00–YST0E, 0x5490 YST0F | tone curve | `tone` (0x5481–0x548F) | in progress; 0x5490 not touched |
| SDE | 0x5580 enables, 0x5581/82 hue cos/sin, 0x5583/84 saturation U/V, 0x5585–0x5587 Y offset / gain / bright, 0x5588 signs, 0x5589–0x558B UV-adjust thresholds | global hue, saturation, contrast, brightness, luma-dependent desaturation | not yet | candidate after CMX: 3–4 parameters, cheap Jacobian |
| LENC | 0x5800–0x5823 green 6x6 matrix, 0x5824–0x5847 B/R 6x6 nibbles, 0x5848/49 centre | lens / colour shading across the field | not yet | visible: magenta-blue halo towards the frame edges; needs a uniform-field pattern, separate stage |
| CIP / DPC | 0x5300–0x530C sharpness/denoise, 0x5500–0x5510 | not colour | — | — |
| Output format | 0x4300 (0x30 YUYV here), 0x501F | byte order, not colour | — | — |

Known limitation: the monitor is the reference; its own gamma and primaries are not
calibrated, so "real colours" means "the sRGB values sent to a consumer monitor at default
settings".  A physical ColorChecker under daylight would be the next step for absolute
accuracy; the procedure (locate → measure → closed-loop register tuning) stays the same.

## Iter results table (iter1, 2026-11-01; dE76 vs the sRGB values sent, 24 patches, grid in the common region)

| Step | Change | cam0 mean dE76 (greys) | cam1 mean dE76 (greys) | Notes |
|---|---|---|---|---|
| stock | auto AEC/AWB, SDK ISP tables | 17.7 (11.0) | 41.4 (33.1) | white clipped, blue cast, cam1 purple |
| expo + awb | manual AGC ('neutral 8' G = 200), manual AWB on mid greys | 11.2 (10.7) | 13.8 (6.5) | greys neutral on both |
| + tone | 15 gamma knots, Gauss-Newton on a 24-step grey ramp | 10.6 (7.5) | 18.0 (6.6) | shadows lifted; run after the `reset` stage |
| + sde / cmx / ccm | saturation, CMX Jacobian, analytic CCM | **8.7 (7.8)** | **17.4 (6.5)** | matrices barely move the error: residual is not 3x3-linear |
| centre mode | each patch alone at the frame centre | 9.3 | 17.5 | cam1 error is field-position dominated (centre green, edges pink) |

Dead ends (details in `JOURNAL.md`): gamma-2.2 model for the AWB loop (oscillates),
GNOME dialogs over the pattern (two runs invalidated → visibility guard), X11 window on the
SSH-forwarded display, per-patch window restarts (> 1 s to map → persistent server), fixed
vs automatic UV-adjust saturation (no effect), lower exposure for cam1 (no effect), stray
light / glare (refuted: patch colour independent of the surround).

## Result (status 2026-11-01 evening)

- **cam0**: calibrated against the monitor to mean dE76 8.7 (from 17.7); greys within 8;
  worst patches cyan / red ≈ 18.  Replayable register set `iter1/tuned_regs_cam0_final.json`.
- **cam1**: 17.4 (from 41.4); greys 6.5; but yellow, magenta, moderate red stay far off
  (dE 25–50) and the error depends on where the patch sits in the frame: cam1 renders the
  frame centre green-cyan and the edges pink on a grey field, and turns ColorChecker yellow
  orange and moderate red orange at the centre.  Not fixable by AWB / CMX / SDE / tone.
  Hardware suspicion: the cam1 module's optics (IR-cut filter missing or different) — see the
  physical checks listed in the journal (TV remote, swap modules, move the board back).
- **Method / tooling** (the deliverable for other OV5640 modules): `locate.py` (monitor →
  camera homography), `tune_colors.py` with stages `reset, expo, tone, awb, sde, cmx, ccm,
  measure` and `--centre`, `--apply <json>` for replay, visibility guard; `shading_map.py`,
  `primaries.py`, `glare_test.py`, `rectify.py` as diagnostics.  Open: LENC stage (needs a
  diffuser flat field, not the screen), ΔE2000 + hold-out validation, physical ColorChecker
  for absolute colour, group-write (0x3212) of the final register set.

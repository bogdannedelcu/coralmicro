# Camera colour calibration / ISP colour tuning — state of the art and what it means for s265

**WBS**: OP-S10-W24-T18 (s265 companion note)
**Date**: 2026-11-01
**Scope**: literature and tooling survey written for the OV5640 pair on the SentAI board
(sensor-internal ISP, YUV422 out, no host ISP) tuned against a consumer monitor.  Everything
marked **[unverified]** could not be checked against a primary source in this session; everything
marked **[inferred]** is our own arithmetic on documented register values, not a vendor statement.

Context assumed (see `README.md` / `JOURNAL.md`): BLC 0x4000–0x4009, AWB gains 0x3400–0x3406,
LENC 0x5800–0x5849, CMX 0x5380–0x538B, gamma 0x5480–0x5490, SDE 0x5580–0x558B; 24-patch
ColorChecker-like sRGB grid on a Philips 234E5 (uncalibrated, IPS, W-LED) a few cm from the lenses;
Gray-code homography; closed-loop measured-Jacobian Gauss–Newton on registers; metric CIE76 in Lab
with both sides treated as sRGB.

---

## 1. The professional workflow (order, spaces, fitting, metrics)

### 1.1 Order of operations and why

Every public ISP tuning flow uses the same skeleton: **black level → lens shading → white balance
→ colour correction matrix → tone curve (gamma) → saturation/other "look" controls**.

- libcamera's tuning-guide draft draws the pipeline as `Sensor → BLC → AWB → LSC → CCM → Gamma`
  and gives the only rationale most guides bother to state: after BLC/AWB/LSC "the grays are
  preserved, but colors are still not correct", which is what the CCM then fixes
  ([patchwork.libcamera.org/patch/20906](https://patchwork.libcamera.org/patch/20906/raw/)).
- Black level **must** come off before any multiplicative stage.  A 2026 libcamera software-ISP
  fix describes the bug of applying black level after white balance: it is "well visible when the
  white balance gains are significantly different from 1.0" and corrupts both CCM and non-CCM
  paths; correct order "black level -> gains -> gamma"
  ([patchwork.libcamera.org/patch/26486](https://patchwork.libcamera.org/patch/26486/)).
- Lens shading is corrected before WB/CCM because colour shading is a *spatially varying* white
  balance error; the Raspberry Pi ALSC separates luminance shading from colour shading and applies
  per-colour-temperature G/R and G/B tables ([RPi Camera Algorithm and Tuning Guide §5.10](https://pip-assets.raspberrypi.com/categories/1259-audio-camera-and-display/documents/RP-009070-TC-3-Camera%20Algorithm%20and%20Tuning%20Guide.pdf)).
  libcamera notes the dependency explicitly: "Currently lsc is ignored in the ccm calculation",
  listed as a known source of colour error (same draft as above).
- **Linear vs. gamma-encoded space**: BLC, LSC, WB gains and the CCM are all linear operations on
  linear sensor data; the gamma/tone curve is the first non-linear stage and sits after the CCM.
  The RPi guide §5.11 adds the practical warning that the "theoretical 2.2 gamma" (i.e. a pure
  `y = x^(1/2.2)`) "is rarely used in practice as it gives rise to rather washed-out low contrast
  images" — real tuning files use an S-shaped inverse-gamma with a toe.
- Saturation (and hue) are applied last, as a YCbCr-domain scaling.  RPi implements it as
  `C_new = A^-1 · diag(1, s, s) · A · C` with `A` the RGB→YCbCr matrix, i.e. a chroma gain around the
  CCM (RPi guide §5.12.2).

The order is not aesthetic: each stage is tuned assuming the previous ones are already correct and
frozen.  Tuning WB with a wrong black level, or a CCM with WB still in auto, bakes one block's error
into another's parameters, which is exactly what the JOURNAL step-5 divergence and the step-7
"CMX did not move" symptoms look like.

### 1.2 How the CCM is fitted

Standard formulation (Kakarala & Wei, OmniVision, 2022): with `M` the 3×k matrix of measured
(black-level-corrected, white-balanced, **linear**) camera RGB and `R` the 3×k reference, minimise
`||C·M − R||_F`.  Unconstrained closed form `C_u = R W Mᵀ (M W Mᵀ)⁻¹` with an optional positive
weighting `W`.  When the CCM runs after WB the rows are constrained to sum to 1 so that
`R = G = B` stays neutral: `min ||CM − R||² s.t. C·1 = 1` — a linear equality-constrained least
squares problem with closed form; the paper derives exactly how much fitting error the constraint
costs and notes it is "not always used" ([arXiv:2201.09935](https://arxiv.org/abs/2201.09935)).
The white-point-preserving variant is due to Finlayson & Drew, *Constrained least-squares
regression in color spaces*, J. Electronic Imaging 6(4), 1997 ([record](https://ueaeprints.uea.ac.uk/id/eprint/40612/))
and *White-point preserving color correction*, CIC 1997.

What the shipping tools actually do:

- **Raspberry Pi ctt** (`rpi-ctt` 1.4.0, `algorithms/ccm.py`, [PyPI](https://pypi.org/project/rpi-ctt/)):
  reference = Macbeth sRGB 8-bit → `eotf_sRGB` → linear 16-bit; per image, WB from the 6 grey
  patches as a *ratio of sums* Σr/Σg, Σb/Σg (comment: an unweighted mean of per-pixel ratios lets
  the dark greys "corrupt the balance so the greys can't be neutralised"); brightness normalised to
  the reference mean; initial linear least squares; then `scipy.optimize.minimize` over **6 free
  parameters** (`r3 = 1 − r1 − r2` etc., rows forced to sum to 1) minimising **mean ΔE2000** in
  Lab (options: max ΔE, or the sum over a chosen patch subset); reports mean and worst-patch ΔE
  before/after; CCMs captured at the same colour temperature are averaged.  The header comment
  says minimising the *average* ΔE was "the most color accurate and vivid" across their cameras;
  minimising the *maximum* gives "generally good results but overall the colors are less accurate".
  The RPi guide §6.9.13 admits the fit is done "in RGB space as this is most simple" and that
  "measuring the fit error in Lab* space" would be "arguably better" — which the current ctt now does.
- **OpenCV `ccm`** ([ccm.hpp](https://raw.githubusercontent.com/opencv/opencv_contrib/4.x/modules/mcc/include/opencv2/mcc/ccm.hpp),
  [ccm.cpp](https://raw.githubusercontent.com/opencv/opencv_contrib/4.x/modules/mcc/src/ccm.cpp)):
  step 1 linearise the *measured* colours (identity for raw, gamma with default 2.2, or per-channel
  polynomial fit `polyfit(R_s, R_dl)` of default degree 3 — the docs warn the polynomial should be
  monotonic on [0,1]); step 2 3×3 or 4×3 (affine) CCM; loss = chosen distance (CIE76, CIE94,
  **CIEDE2000**, CMC, RGB, linear-RGB) between CCM output and reference in Lab; initial value by
  least squares or by white balance (diagonal of channel-mean ratios); nonlinear refinement with
  `cv::DownhillSolver` (Nelder–Mead); weights per patch or `L*^weights_coeff`; patches with any
  channel outside `[0, 0.98]` are masked as saturated by default.
- **colour-science** (`colour.matrix_colour_correction`, [docs](https://colour.readthedocs.io/en/latest/generated/colour.matrix_colour_correction.html)):
  plain multiple linear regression, with optional polynomial augmentation (Cheung 2004, up to 35
  terms) or root-polynomial (Finlayson, Mackiewicz & Hurlbert, IEEE TIP 2015, degree 1–4).  No
  white-point constraint offered.  Root-polynomials are the standard answer when a 3×3 is not
  enough but exposure invariance must be kept; irrelevant to us because the OV5640 hardware is a
  3×3.

Weighting and regularisation in practice: greys up-weighted or exactly constrained (row sums);
saturated/clipped patches masked (OpenCV); optional L*-based weights; Tikhonov only when the
patch set is rank-deficient (24 patches are not).  Nobody fits a CCM through measured Jacobians —
the matrix is linear in its inputs, so one capture with the matrix at a known value gives the whole
Jacobian analytically (see §6).

### 1.3 Metrics

- **ΔE2000** is the de-facto metric; Imatest calls it "the best choice", with `kL = kC = kH = 1`,
  and documents ΔE76 as "not very accurate, especially for strongly saturated colors"
  ([Imatest Colorcheck reference](https://docs.imatest.com/5.2/colorcheck_ref)).  Use the Sharma,
  Wu & Dalal 2005 implementation notes and test vectors to validate any ΔE00 code
  ([rochester.edu](https://hajim.rochester.edu/ece/sites/gsharma/ciede2000/)).
- Report **mean and RMS (σ) or 95th percentile and max**, not only the mean; Imatest: "The RMS
  value is of interest because it gives more weight to the larger errors."
- Separate lightness from chroma: Imatest reports ΔC (ΔE with the L term dropped) and a
  saturation-corrected ΔC("corr") where the camera's mean chroma over the 18 colour patches is
  first scaled to the reference mean chroma — this isolates hue/CCM error from a global saturation
  setting.  Mean saturation itself is explicitly *not* a quality indicator ("can be easily adjusted").
- **Grey neutrality**: ΔC (or a/b offsets) on the 6 neutral patches separately; ctt's `de_norm`
  removes the best global luminance scale before computing ΔE because a row-sum-1 CCM "carries no
  brightness — absolute lightness is set downstream by AEC/tone".
- Exposure/tone diagnostics: Imatest derives gamma from grey patches 2–5 (patches 1 and 6
  excluded because they clip) and the ideal 8-bit grey levels 236/195/157/119/83/52.

---

## 2. Targets and illuminants

- **ColorChecker Classic (24)**: reference data exist as X-Rite/GretagMacbeth Lab-D50 (2005 set
  and a post-November-2014 set after a pigment reformulation).  BabelColor publishes averaged
  **spectral reflectances of 30 pre-2014 charts** plus derived sRGB/Lab; the two X-Rite reference
  sets differ by mean ΔE00 0.85, max 1.95 (7 patches > 1.0), and chart-to-chart variation is
  largest in lightness and on the white patch ([babelcolor.com](https://babelcolor.com/colorchecker-2.htm)).
  Recommendation there: for sRGB use values derived from Lab D50 rather than X-Rite's sRGB list.
  The Python `colour` package ships both datasets
  ([colour.characterisation datasets](https://colour.readthedocs.io/en/v0.4.2/_modules/colour/characterisation/datasets/colour_checkers/chromaticity_coordinates.html)).
- **ColorChecker Digital SG (140)**: X-Rite does not publish its spectral data
  ([X-Rite FAQ](https://www.xrite.com/service-support/faq_colorchecker_sg_spectral_data)); OpenCV
  `ccm` supports it as `COLORCHECKER_DigitalSG`.  **IT8.7** targets come with per-batch
  colorimetric data and are the scanner/photo standard; not needed for 3×3 ISP work **[not
  researched further]**.
- **ISO 17321-1:2012** defines DSC colour characterisation with Method A (spectral sensitivities)
  and Method B (target based) and an informative Annex B *sensitivity metamerism index* (SMI); it
  states up front that DSC spectral responses "do not, in general, match those of a typical human
  observer" ([ANSI preview](https://webstore.ansi.org/preview-pages/ISO/preview_ISO+17321-1-2012.pdf)).
- **Illuminants and light boxes**: libcamera's draft lists 2400 K incandescent, 2700 K and 5000 K
  fluorescent, 6500 K daylight, and suggests a cheap variable-CCT video key-light as a first
  light box; the RPi guide §6.6.1 asks for charts "from about 2500K up to about 8000K", one chart
  image per distinct CCT, 18 % grey at roughly 16–20 % signal, chart lit evenly at ~45°, no stray
  light.  The K230 (Kendryte) guide uses a DNP light box with D65, D50, TL84, F12 and A, captures
  the chart and then the empty grey wall under each, and derives AWB white points from the CCM
  files per illuminant ([K230 ISP tuning guide](https://www.kendryte.com/k230_rtos/en/main/advanced_development_guide/how_to_tune_isp.html)).
- **Per-illuminant CCMs**: RPi stores "a list of colour temperatures, each followed by a 3x3
  matrix" and interpolates on the AWB's CCT estimate (§5.12); AWB itself is calibrated by imaging
  Macbeth greys under each illuminant and fitting a quadratic "CT curve" in (r, b) chromaticity
  (§5.8, ctt `awb.py` uses `np.polyfit(..., 2)` plus transverse limits).  Per-illuminant CCMs
  exist because camera sensitivities violate the Luther condition — only a camera whose
  sensitivities are a linear transform of the CIE CMFs has an illuminant-independent exact 3×3
  ([Finlayson & Zhu, arXiv:2003.12645](https://arxiv.org/pdf/2003.12645v1)); for everyone else the
  best matrix is illuminant- and reflectance-set-dependent (metamerism).  The Cambridge
  colour-accuracy project puts it bluntly: colour correction is "an underdetermined problem" and
  "visible inaccuracies persist" ([cl.cam.ac.uk](https://www.cl.cam.ac.uk/research/rainbow/projects/color_accurate/)).

---

## 3. Using a display as the calibration target

### 3.1 When it is acceptable

For **relative** tasks: matching two cameras to each other, equalising grey neutrality *under the
display's white*, measuring and shaping the tone curve, fixing a global saturation scale, and
regression-testing that a replayed register set reproduces a result.  Both cameras see the same
emissive stimulus, so inter-camera agreement measured on a display is valid.  Emissive targets are
not inherently bad: DiCarlo, Montgomery & Trovinger (HP Labs, CIC 2004) built an *emissive
calibration chart* of 16 narrow-band LEDs and showed it "produced a more accurate camera
calibration than both reflective charts" and comparable to a monochromator — but their method
estimates the camera's spectral responsivities from **known LED spectra**, not a 3×3 from XYZ
matches ([IS&T CIC 12, art00052](https://library.imaging.org/admin/apis/public/api/ist/website/downloadArticle/cic/12/1/art00052)).
That is the key difference from pointing a camera at a monitor.

### 3.2 What goes wrong

1. **Three narrow-ish primaries vs. broadband reflectances (metamerism).**  A W-LED backlight is a
   blue InGaN peak near 450 nm plus a broad YAG phosphor (FWHM ~120 nm for single-YAG), filtered by
   RGB colour filters ([pcmonitors.info](https://pcmonitors.info/articles/the-evolution-of-led-backlights/);
   [UCF LSA 2017](https://lcd.creol.ucf.edu/Publications/2017/lsa201743a.pdf)).  Two stimuli with the
   same XYZ for the CIE observer (a display patch vs. a paint chip) give different camera RGB
   because the camera fails the Luther condition.  Display-side literature documents the same
   effect for human observers with narrow-band backlights ("CIE 2 degree observer ... is not
   actually accurate for narrow-bandwidth displays", [Light Illusion](https://lightillusion.com/perceptual_match_guide.html);
   ICC DevCon 2016 soft-proofing failures with LED monitors, [Bai](https://archive.color.org/devcon/devcon2016/presentations/Bai_DevCon2016.pdf)).
   Consequence: a CCM fitted on display metamers is the best matrix *for that display's spectra*;
   its error on real objects is unknown until measured.  A display white balance is an even
   clearer case: the AWB gains that neutralise the monitor's white are the gains for a stimulus
   that no daylight or room light reproduces.  **A display cannot calibrate the deployment white
   point; it can only calibrate "white = this monitor".**
2. **Display gamut, gamma, white point are unknown** on an uncalibrated consumer panel.  Patches
   near the sRGB gamut boundary may be clipped by the display; the EOTF is typically not exactly
   the sRGB curve; the preset white is typically 6500–7500 K-ish **[unverified for the 234E5; no
   measured review found]**.  Treating the *sent code values* as the ground truth makes the camera
   inherit the monitor's errors.
3. **Backlight PWM and rolling shutter** → horizontal bands whose phase varies with exposure; the
   mechanism is row-by-row readout sampling a modulated source ([arXiv:2605.21079](https://arxiv.org/pdf/2605.21079)).
   Mitigation is an exposure that spans many modulation periods (our 30 fps max exposure already
   covers ~2 refresh periods and many PWM cycles); verify with a short-exposure uniform-grey
   capture.  Imatest advises rejecting a monitor "if it contains banding" and offers a flicker
   test ([Imatest screen patterns](https://www.imatest.com/docs/screenpatterns)).
4. **Viewing angle.**  At a few cm from a 23″ panel the lens edge rays hit the LCD 30°+ off-axis;
   IPS is far better than TN but still shifts gamma and chroma with angle.  Any LENC fitted
   against a flat screen at that distance will **bake the panel's angular response into the
   sensor's shading table**.
5. **Pixel grid / moiré / screen-door.**  Interference between the display's RGB sub-pixel grid and
   the Bayer sampling; standard fixes are defocus, distance or angle
   ([Elo support](https://elosupport.elotouch.com/hc/en-us/articles/31652854428311-Why-do-I-get-wavy-lines-in-a-photo-when-I-try-taking-a-picture-of-my-Elo-monitor-s-display)).
   Averaging the inner 70 % of a patch (≈100 × 80 camera px here) is the right mitigation for a
   fixed-focus module.
6. **Glare / self-reflection / room light** on the panel, and display black level (IPS ≈ 1000:1 →
   black ≈ 0.1 % of white, plus flare) contaminating the dark patches.

### 3.3 Mitigations

- Calibrate the display with a colorimeter (ArgyllCMS/DisplayCAL to D65 = xy 0.3127/0.3290 and
  a known EOTF; verification reports give avg/max ΔE of the result, e.g. 0.28/1.01 in a typical
  run) ([DisplayCAL forum](https://hub.displaycal.net/forums/topic/are-my-calibration-results-ok/),
  [D65 vs 6500 K](https://hub.displaycal.net/forums/topic/some-questions-about-calibration-for-a-noob-enthusiast/)).
  This fixes the targets *for the human observer* and gives measured XYZ per patch; it does **not**
  remove the metamerism issue of §3.2-1.
- Without a colorimeter: measure the display's EOTF and white with the camera itself in **linear
  mode** (OV5640 raw gamma off, see §4) and use the measured emitted light, not the code value,
  as target.
- Full-screen solid patches for the tone curve and uniform fields (no spatial averaging error),
  N ≥ 4 frames averaged per measurement, dark room, pattern-visible sanity check before each run
  (already in the JOURNAL rules after the GNOME dialog incident).
- Keep the display for relative work and validate absolute colour on a physical ColorChecker
  under daylight / the deployment light (§6).

---

## 4. OmniVision-specific guidance (OV5640)

### 4.1 What OmniVision publishes

- **Datasheet v2.03** ([OV5640.pdf, acmesystems mirror](https://www.acmesystems.it/www/roadrunner_isc/OV5640.pdf)):
  ISP chapter order is 5.2 LENC → 5.3 AWB → 5.4 raw gamma → 5.5 DPC → 5.6 CIP (de-noise, RAW→RGB
  interpolation, edge enhancement) → 5.7 CMX → 5.8 UV average → 5.9 scaling → 5.10 UV adjust →
  5.11 SDE → 5.12 format.  §5.4: "Raw gamma compensates the image in the RAW domain", registers
  0x5480–0x5490 (YST00–YST0F), enable 0x5000[5].  **The gamma curve therefore precedes demosaic and
  the CMX; the CMX operates on gamma-encoded RGB** — the opposite of the textbook order in §1.1
  **[inferred from the datasheet structure, not measured]**.  §5.2 LENC and §5.3 advanced AWB both
  say "contact your local OmniVision FAE" — there is no public LENC/AWB calibration procedure.
  §4.7 BLC uses optically shielded lines with a 10-bit target in 0x4009 (default 0x10); §7.4 AWB
  gains are 12-bit with 0x400 = 1.0×, manual via 0x3406[0].
- **CMX (§5.7)**: "cancel out crosstalk and convert color space"; the datasheet formula reads
  `CMX = RGB2YUV × CCM × [[1,-0.25,0.75],[1,-0.25,-0.25],[1,0.75,-0.25]]` **[meaning of the
  right-hand factor unverified — its inverse is [[¼,½,¼],[0,-1,1],[1,-1,0]], i.e. (R+2G+B)/4, B−G,
  R−G, which suggests an internal colour-difference representation]**.  Registers: 0x5380[1]
  precision "1.7 mode" (×1/128) or "2.6 mode" (×1/64); CMX1–3 "for Y", CMX4–6 "for U", CMX7–9
  "for V"; signs: 0x538B bits 0–7 = CMX1–CMX8, 0x538A bit 0 = CMX9 (1 = negative).
- **SDE / UV adjust (§5.10–5.11)**: hue = cos/sin pair 0x5581/0x5582 with signs in 0x5588[5:4,1:0];
  saturation U/V 0x5583/0x5584; Y offset/gain/brightness 0x5585–0x5587 (Y gain 0x20 = 1.0);
  **UV adjust auto mode** when 0x5580[1] = 1 and 0x5588[6] = 0: chroma gain falls linearly from
  0x5583 (at AGC gain ≤ th1 = 0x5589) to 0x5584 (at AGC gain ≥ th2 = {0x558A[0],0x558B}); manual
  mode needs 0x5588[6] = 1.
- **Software Application Note rev 2.13** ([PDF](https://cdn.shopify.com/s/files/1/0803/9211/files/OV5640-Software-Application-Note.pdf?41)):
  "Lens Correction setting should be tuned for every module ... Please find lens correction
  settings of modules tuned by OVT FAE"; init table comments `CMX1 for Y ... CMX9 for V`,
  `0x538a sign[9]`, `0x538b sign[8:1]`; the stock init writes `0x5580=0x06` (saturation + contrast
  enable), `0x5583=0x40, 0x5584=0x10, 0x5589=0x10, 0x558a=0x00, 0x558b=0xf8` — i.e. **UV adjust
  in auto mode, desaturating from 0x40 at ≤1.0× AGC to 0x10 at 15.5× AGC** (AGC code in 1/16
  steps).  The "Saturation ±N" user functions are implemented by rewriting CMX4–CMX9 (the U/V rows)
  inside a group-write (0x3212), leaving CMX1–3 fixed; "Brightness/Contrast" use 0x5585–0x5588.
  No CCM/AWB tuning *procedure* is in the note — only tables.

### 4.2 Decoding the stock matrix [inferred]

Stock table (app note, NXP SDK `fsl_ov5640.c`, Linux `ov5640.c`): `1e 5b 08 | 0a 7e 88 | 7c 6c 10`,
signs `0x538A=0x01, 0x538B=0x98` → CMX4, CMX5, CMX8, CMX9 negative.  In 1.7 mode:

```
Y:  +0.2344 +0.7109 +0.0625   row sum +1.008
U:  -0.0781 -0.9844 +1.0625   row sum  0.000
V:  +0.9688 -0.8438 -0.1250   row sum  0.000
```

So the register matrix is a full RGB→YUV matrix applied to the (CCM-corrected) RGB, with the
grey-preserving structure built in: Y row sums to 1, U and V rows sum to 0.  Factorising with
BT.601 whose chroma rows are doubled (`U = B − Y`, `V = R − Y`, i.e. `2·(−0.169,−0.331,0.5)` and
`2·(0.5,−0.419,−0.081)`) gives a plausible mild CCM with row sums 1.008:

```
C ≈ [[ 0.913  0.120 -0.026 ]
     [-0.098  1.181 -0.076 ]
     [ 0.166 -0.162  1.004 ]]
```

whereas factorising with plain BT.601 gives an implausible one (diagonal 1.6–1.9).  Hypothesis:
the OV5640's internal RGB2YUV is BT.601 luma with unscaled colour differences, and the SDE
saturation gains (0x5583/0x5584 = 0x40 ≈ 0.5) bring U/V to Cb/Cr range.  The datasheet register
defaults (`20 64 08 | 30 90 C0 | A0 98 08`) and the app-note "Saturation +3" row obey the same
row-sum structure.  **This is our arithmetic, not an OmniVision statement.**

### 4.3 Public register-level examples

- Linux mainline `drivers/media/i2c/ov5640.c` ([source](https://raw.githubusercontent.com/torvalds/linux/master/drivers/media/i2c/ov5640.c)):
  fixed CMX table (same as the app note), V4L2 hue/contrast/saturation controls mapped onto SDE
  (`OV5640_REG_SDE_CTRL0/1/3/4/5`), AWB manual via 0x3406; a May 2026 series by Kieran Bingham
  names the 0x518x AWB registers ("AWB B block", "AWB Top limit") but documents no procedure
  ([lore mirror](https://lore.jethome.com/lkml/agWG_dvrlSVAc65g@zed/)).
- Zephyr `drivers/video/ov5640.c` carries the same CMX block and saturation tables
  ([mirror](https://pigweed.googlesource.com/third_party/github/zephyrproject-rtos/zephyr/+/627fa91e42fb3d1cd0b156c1a89f9600a455dc35/drivers/video/ov5640.c)).
- NXP SDK `fsl_ov5640.c` (in-tree, patched by this repo) — same table.
- Raspberry Pi / libcamera have no OV5640 tuning (they bypass sensor ISPs and tune their own);
  Arducam/TI/STM32 OV5640 examples found only reuse the OmniVision table **[unverified beyond the
  searches run]**.  STM32's ISP IQTune wiki exists for the MP2/N6 ISP path but could not be fetched
  this session ([wiki.st.com](https://wiki.st.com/stm32mpu/wiki/How_to_tune_a_camera_sensor_using_the_STM32_ISP_IQTune)) **[unverified]**.
- No public OmniVision tuning tool or CMX/LENC procedure was found; everything beyond the datasheet
  is FAE/NDA.

---

## 5. Open-source tooling to reuse or borrow from

| Tool | Reusable for us | Notes |
|---|---|---|
| `rpi-ctt` 1.4.0 (`pip download rpi-ctt`), `algorithms/ccm.py`, `awb.py`, `alsc.py` | **fitting logic**: ratio-of-sums WB from greys; LS init + ΔE00 refinement with rows fixed to sum 1; mean/max/patch-subset objectives; per-CT averaging; flat-field averaging for ALSC with rotated captures | DNG/Bayer input assumed; our data are post-ISP YUV, so only the maths transfers |
| RPi Camera Algorithm and Tuning Guide (PDF above) | the capture protocol: chart central/straight, CCT + lux in file names, one chart per CCT, flat-field images rotated 180°, 18 % grey at 16–20 % | BSD-licensed, authoritative |
| OpenCV `mcc` + `ccm` ([group docs](https://docs.opencv.org/4.x/de/df4/group__color__correction.html)) | chart detection (`mcc::CCheckerDetector`), `ColorCorrectionModel` with ΔE00 loss, saturation masking, weights, polynomial linearisation | Python bindings; good for the *validation* side (fit a reference CCM from JPEG patches and compare to what the sensor achieved) |
| `colour-science` | `delta_E_CIE2000`, ColorChecker 2005/2014 datasets, sRGB EOTF/OETF, `matrix_colour_correction` (Cheung/Finlayson/Vandermonde) | ctt itself imports it |
| ArgyllCMS / DisplayCAL | calibrate + verify the monitor if a colorimeter is available (avg/max ΔE report) | only fixes the human-observer side |
| Imatest docs (free) | metric definitions: ΔE00, ΔC, saturation-corrected ΔC, mean + RMS, grey-patch gamma | software is commercial |
| Sharma CIEDE2000 test data | unit test for our ΔE00 | — |
| K230 ISP tuning guide | the light-box protocol (D65/D50/TL84/F12/A, 80 % centre level for LSC, grey-wall capture) | MATLAB tool, not reusable |

---

## 6. Recommendations for the s265 pipeline (ranked)

### R1 — Freeze every hidden automatic before tuning anything (highest value, zero cost)

Already done: AEC/AGC (0x3503 = 0x03), AWB (0x3406 = 1).  **Missing**: UV-adjust auto mode.  With
the stock `0x5580=0x02/0x06`, `0x5588[6]=0`, chroma gain depends on AGC: at our cam0 gain 31/16 =
1.94× it is ≈ 0x3D (−5 %), at cam1 47/16 = 2.94× ≈ 0x3A (−10 %) **[inferred from the datasheet
formula]**.  This couples the `expo` stage into saturation and makes cam1 systematically less
saturated than cam0 — consistent with the JOURNAL's "cam1 blue/yellow axis strongly desaturated,
cam0 same pattern at a third of the amplitude".  Set `0x5588 |= 0x40`, `0x5583 = 0x5584 = 0x40`
(datasheet default), hue `0x5581 = 0x80, 0x5582 = 0x00`, Y gain `0x5586 = 0x20`, Y offset/bright
`0x5585 = 0x5587 = 0`.  Hue rotation and Y gain/offset are strict subsets of what CMX and the
gamma knots already do; keep them neutral so the parameter set is not degenerate.

### R2 — Add a linear "measurement mode" and use it for everything except the CMX

Disable raw gamma (`0x5000[5] = 0`) during measurement.  With the stock CMX known (§4.2) the output
is then a linear function of scene radiance, so: (a) the AWB gains become exactly multiplicative
→ one analytic step `g_R = G/R, g_B = G/B` from the ratio-of-sums over the greys (no secant
exponent hacks); (b) the camera becomes a relative photometer for the **monitor's actual EOTF**
per channel — sample the 24-step ramp, fit per-channel `L(code)`, and use the measured relative
luminance (not the sent code) as the tone target; (c) black floor and flat field are measured in
linear units; (d) 8-bit quantisation at the dark end is coarse (sRGB 52 ≈ 3.3 % linear ≈ 8 LSB),
so take a second capture at a known longer exposure for the dark half of the ramp.  Re-enable gamma
afterwards.  **Verify first** that gamma-off does not change CIP/other block behaviour; the bit is
documented but its interaction with the rest is **[unverified]**.

### R3 — Re-order the stages

`reset → freeze (R1) → linear mode (R2) → exposure → LENC → AWB → gamma on → tone → CMX → validate`.
Exposure target: white patch at ≈ 90 % of full scale in linear mode (currently white clips at 255
on cam1 after AWB — a clipped white biases both the tone fit and the CCM; OpenCV masks > 0.98 for
this reason).  LENC before AWB (colour shading is spatially varying WB), AWB before tone, tone
before CMX only because the CMX sits *after* gamma in this sensor (§4.1) and must be fitted in the
operating configuration.

### R4 — Fit the CMX analytically in the sensor's own (gamma-encoded) space; use GN only to polish

The register matrix is linear in its input, so measured Jacobians are unnecessary and the
sign-bit problem disappears.  With gamma on, SDE neutral (R1), and the stock matrix `M_0` loaded:

1. Measured output per patch: decode the JPEG to RGB, convert back to the sensor's Y'CbCr with
   the JFIF/BT.601 full-range matrix the encoder used → `y_i` (3-vector).  Clipped patches masked.
2. Recover the CMX input: `x_i = M_0⁻¹ y_i` (gamma-encoded camera RGB; `M_0` is invertible, det ≠ 0).
3. Target: `t_i = A_601 · sRGB_OETF(L_i)` where `L_i` is the patch's **measured** linear display
   colour (R2, or colorimeter) expressed in linear sRGB; fall back to the nominal sRGB code only if
   nothing better exists.
4. Solve the equality-constrained LS `min Σ w_i ||M x_i − t_i||²` with `M_Y·1 = 1`, `M_U·1 = 0`,
   `M_V·1 = 0` (closed form via Lagrange multipliers or by eliminating one coefficient per row as
   ctt does: 6 free parameters).  Weights: greys ×2 is fine; or minimise ΔE00 with
   `scipy.optimize.minimize` from the LS solution, exactly like ctt.
5. Quantise: 1.7 mode `reg_k = round(|M_k| · 128)` (max 1.99), or switch to 2.6 mode (`0x5380[1]=1`,
   ×64, max 3.98) if any |coefficient| ≥ 2; sign bits per §4.1 layout.  Write all 11 registers
   inside a group (`0x3212 = 0x03 … 0x13 … 0xa3`) as the app note does, to avoid a torn frame.
6. One closed-loop re-measure; if residual > quantisation noise, one GN round with the analytic
   Jacobian `∂y/∂M = x_iᵀ` (no perturbation captures needed).

If instead a linear-space CCM `C` is wanted for documentation, `CMX_new = A_ov · C` with
`A_ov = [[0.299, 0.587, 0.114], [−0.338, −0.662, 1.0], [1.0, −0.838, −0.162]]` is the mapping
implied by §4.2 — **[inferred]**; test it by writing `A_ov · I` and checking that greys stay
neutral and colours barely change relative to stock.

### R5 — Treat the display honestly

- Absolute colour claims need a **physical ColorChecker Classic** (≈ €100; use the 2014+ X-Rite
  reference values for a new chart) under daylight and under the deployment indoor light; the
  display-derived CMX is validated — or re-fitted per illuminant — against it.  Expect the
  display-fitted matrix to be off on real reflectances by an amount we cannot predict (§3.2-1).
- Store monitor-derived AWB gains as a `display_D65ish` preset only.  Deployment white balance
  comes from a grey card under the actual light, or from the sensor's own AWB with the tuned CMX
  (RPi/K230 both tune AWB from greys per illuminant, never from a screen).
- Do **not** fit LENC against the screen at a few cm (viewing-angle shading, §3.2-4).  Use a
  diffuser (plain paper over the panel, or a paper + lamp flat field) and follow the ctt recipe:
  several captures with the camera rotated/moved, averaged, per colour temperature.
- Tone targets from the *measured* display EOTF (R2) rather than sRGB codes; if a colorimeter is
  ever available, calibrate the panel (D65, sRGB EOTF) and record the verification report in
  `iterN/`.
- Check PWM banding once (short-exposure uniform grey); keep exposure ≥ 2 refresh periods.

### R6 — Validation protocol

- Metric: ΔE00 (validated against Sharma's test vectors), reported as mean / 95th pct / max over
  the 18 colour patches, plus ΔC on the 6 greys, plus saturation-corrected ΔC (Imatest "corr") to
  separate hue from saturation.  Keep ΔE76 only for continuity with iter1.
- Hold-out: fit on 18 patches, report on the 6 held out (ctt's `patches` mode is the precedent),
  rotating folds; with only 6 CMX degrees of freedom overfitting is small but the 15 gamma knots
  on 24 greys *do* overfit noisy ramps (cam1's `96 96 96` plateau) — average N ≥ 4 frames and
  consider a 3-parameter parametric curve (toe, gamma, shoulder) sampled to knots.
- Repeatability: apply `tuned_regs_camN.json` on a cold boot twice; the two measurement sets must
  agree within the frame-noise floor (measure it: 10 captures, stock registers).
- Second illuminant: the physical chart of R5 under daylight (and the room's LEDs); report the
  display-fitted vs. chart-fitted CMX difference as the number that bounds "true colour" claims.
- Inter-camera: ΔE00 between cam0 and cam1 renderings of the same patches (relative metric that
  the display *can* support).

### R7 — Minimal replayable tuning script for a stock OV5640 module

1. Preconditions: board alive, build counter, camera init, pattern process alive, **pattern-visible
   check** (white/black reference means within tolerance of the locate run) — abort otherwise.
2. `reset`: write the SDK defaults for CMX/gamma/SDE/BLC, AWB/AEC to auto, then **freeze** (R1).
3. `locate`: Gray-code homography per camera (existing), store `H` and the common rectangle.
4. `linear`: gamma off; exposure to 90 % on white; flat field → LENC (if a diffuser is present,
   else skip and log "LENC not tuned"); greys → AWB gains analytic; ramp → display EOTF per channel.
5. `tone`: gamma on; targets from the measured EOTF; GN on knots with smoothness, N-frame averaging.
6. `cmx`: analytic constrained fit (R4) → quantise → group write → one verification capture.
7. `validate` (R6) → `iterN/tuned_regs_camN.json` with every register written, in write order,
   plus the metrics, firmware build, pattern geometry and monitor identity (EDID) as provenance.
8. `replay`: apply the JSON on a cold boot and re-run `validate` only.

---

## 7. Sources

- Raspberry Pi, *Camera Algorithm and Tuning Guide* (RP-009070-TC-3), §5.8, §5.10–5.12, §6.4–6.9 — https://pip-assets.raspberrypi.com/categories/1259-audio-camera-and-display/documents/RP-009070-TC-3-Camera%20Algorithm%20and%20Tuning%20Guide.pdf
- `rpi-ctt` 1.4.0 source (`apps/ctt/ctt/algorithms/{ccm,awb,alsc}.py`) — https://pypi.org/project/rpi-ctt/
- libcamera tuning-guide draft (v2) — https://patchwork.libcamera.org/patch/20906/raw/ ; software-ISP black-level order fix — https://patchwork.libcamera.org/patch/26486/
- Kendryte K230 ISP Image Tuning Guide — https://www.kendryte.com/k230_rtos/en/main/advanced_development_guide/how_to_tune_isp.html
- Kakarala & Wei (OmniVision), *What is the cost of adding a constraint in linear least squares?*, arXiv:2201.09935 (2022) — https://arxiv.org/abs/2201.09935
- Finlayson & Drew, *Constrained least-squares regression in color spaces*, J. Electron. Imaging 6(4) 484–493, 1997 — https://ueaeprints.uea.ac.uk/id/eprint/40612/ ; *White-point preserving color correction*, CIC 5, 1997 — https://library.imaging.org/cic/articles/5/1/art00051
- Finlayson & Zhu, *Designing color filters that make cameras more colorimetric* (Luther condition), arXiv:2003.12645 — https://arxiv.org/pdf/2003.12645v1
- Rowlands, *Color conversion matrices in digital cameras: a tutorial*, Opt. Eng. 59(11) 110801, 2020 — https://www.spiedigitallibrary.org/journals/optical-engineering/volume-59/issue-11/110801/Color-conversion-matrices-in-digital-cameras-a-tutorial/10.1117/1.OE.59.11.110801.pdf **[not readable this session; cited for completeness]**
- OpenCV `ccm` module header and implementation — https://raw.githubusercontent.com/opencv/opencv_contrib/4.x/modules/mcc/include/opencv2/mcc/ccm.hpp , https://raw.githubusercontent.com/opencv/opencv_contrib/4.x/modules/mcc/src/ccm.cpp ; group docs — https://docs.opencv.org/4.x/de/df4/group__color__correction.html
- colour-science `matrix_colour_correction` — https://colour.readthedocs.io/en/latest/generated/colour.matrix_colour_correction.html ; ColorChecker datasets — https://colour.readthedocs.io/en/v0.4.2/_modules/colour/characterisation/datasets/colour_checkers/chromaticity_coordinates.html
- Imatest, *Colorcheck algorithms and reference formulas* — https://docs.imatest.com/5.2/colorcheck_ref ; screen patterns / flicker — https://www.imatest.com/docs/screenpatterns
- Sharma, Wu & Dalal, *The CIEDE2000 color-difference formula: implementation notes, supplementary test data…*, CR&A 30(1) 2005 — https://hajim.rochester.edu/ece/sites/gsharma/ciede2000/
- BabelColor, *ColorChecker data (2005 vs post-Nov-2014, 30-chart spectral average)* — https://babelcolor.com/colorchecker-2.htm ; X-Rite SG spectral-data FAQ — https://www.xrite.com/service-support/faq_colorchecker_sg_spectral_data ; Wikipedia ColorChecker — https://en.wikipedia.org/wiki/ColorChecker
- ISO 17321-1:2012 preview (Method A/B, Annex B SMI) — https://webstore.ansi.org/preview-pages/ISO/preview_ISO+17321-1-2012.pdf
- DiCarlo, Montgomery & Trovinger (HP Labs), *Emissive chart for imager calibration*, IS&T/SID CIC 12, 2004 — https://library.imaging.org/admin/apis/public/api/ist/website/downloadArticle/cic/12/1/art00052
- pcmonitors.info, *The Evolution of LED Backlights* — https://pcmonitors.info/articles/the-evolution-of-led-backlights/ ; Chen et al., LSA 2017 (phosphor FWHM) — https://lcd.creol.ucf.edu/Publications/2017/lsa201743a.pdf
- Light Illusion, *Perceptual colour matching* (narrow-band display metamerism) — https://lightillusion.com/perceptual_match_guide.html ; Bai, ICC DevCon 2016 — https://archive.color.org/devcon/devcon2016/presentations/Bai_DevCon2016.pdf
- Cambridge Rainbow group, *Color accurate imaging* — https://www.cl.cam.ac.uk/research/rainbow/projects/color_accurate/
- Flicker banding mechanism, arXiv:2605.21079 — https://arxiv.org/pdf/2605.21079 ; Blur Busters PWM photos — https://blurbusters.com/?p=564 ; moiré when photographing displays — https://elosupport.elotouch.com/hc/en-us/articles/31652854428311-Why-do-I-get-wavy-lines-in-a-photo-when-I-try-taking-a-picture-of-my-Elo-monitor-s-display
- DisplayCAL forums (verification ΔE, D65 xy) — https://hub.displaycal.net/forums/topic/are-my-calibration-results-ok/ , https://hub.displaycal.net/forums/topic/some-questions-about-calibration-for-a-noob-enthusiast/
- OmniVision OV5640 datasheet v2.03 — https://www.acmesystems.it/www/roadrunner_isc/OV5640.pdf ; OV5640 Software Application Note rev 2.13 — https://cdn.shopify.com/s/files/1/0803/9211/files/OV5640-Software-Application-Note.pdf?41
- Linux `ov5640.c` — https://raw.githubusercontent.com/torvalds/linux/master/drivers/media/i2c/ov5640.c ; Bingham AWB register documentation series (2026) — https://lore.jethome.com/lkml/agWG_dvrlSVAc65g@zed/ ; Zephyr `ov5640.c` — https://pigweed.googlesource.com/third_party/github/zephyrproject-rtos/zephyr/+/627fa91e42fb3d1cd0b156c1a89f9600a455dc35/drivers/video/ov5640.c
- Philips 234E5 (IPS) review, no measurements — https://www.techadvisor.com/article/713947/philips-234e5-review.html

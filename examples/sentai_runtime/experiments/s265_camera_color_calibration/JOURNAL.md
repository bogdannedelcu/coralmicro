# s265 journal — camera colour calibration

Chronological.  Every step that touched the board, the monitor or the analysis is here,
with the exact command and the outcome, so the procedure can be replayed on another pair
of OV5640 modules.  Times are local (2026-11-01, evening).

## Setup

- **Board**: 58eb, SentAI v1.1 build 1756, both cameras rotated 270° by the firmware
  (`0x3820=0x47`, `0x3821=0x01`), VGA 640x480 at 30 fps, legacy 4 B/px XYUV capture, JPEG q90
  via `sentai.camera.save_jpeg` and `sentai.fs.read_base64` over the REPL (`/dev/ttyACM0`).
  HTTP over USB-NCM is not enabled (needs `sentai.usb.ip(1)`), so everything goes over the REPL.
- **Monitor**: Philips 234E5 (EDID "PHL 234E5", serial UHB1403017732), 23", 1920x1080@60,
  HDMI-3 of the RTX 4090 (`card2-HDMI-A-3`), GNOME/Wayland session (`wayland-0`).
  It was in power-save mode 3 (off) when we started: `~/work/unreal/monitor_power_saver.sh`
  re-asserts `org.gnome.Mutter.DisplayConfig.PowerSaveMode=3` every 3 s while UnrealEditor
  is not running.  Fix: `touch /tmp/mps_pause` (the script's own pause switch) + set the
  property to 0.  DRM went `dpms=Off/disabled` → `On/enabled`.  Remove the pause file after
  the experiment.
- **Fullscreen on Wayland**: GTK 3 `Gtk.Window.fullscreen_on_monitor()` works from a tty
  shell with `WAYLAND_DISPLAY=wayland-0` and the user's DBus address.  The Unreal project
  uses X11 `_NET_WM_FULLSCREEN` atoms via python-xlib (XWayland) and Mutter
  `ApplyMonitorsConfig` for rotation; not needed here (one landscape monitor).
- **Board state found**: after the last reboot the camera was not initialised
  (`select()` and `reg_read()` returned -1).  `sentai.camera.init()` → 0, sensor 30 fps,
  genlock phase 21 µs, pre-trim 12 lines.  The embed MicroPython has no `time`/`utime`
  module: host-side sleeps between REPL commands instead.

## Step 1 — baseline capture (stock ISP, whatever is on the monitor)

`python3 capture.py --out iter1 --tag initial`

| cam | JPEG | AWB R gain | G | B | AGC (0x350A/B) | exposure 0x3500-02 |
|---|---|---|---|---|---|---|
| 0 | 61 803 B | 1566 | 1024 | 1377 | 0x16 | 0x00 0x3D 0x40 |
| 1 | 55 535 B | 1423 | 1024 | 1835 | 0x15 | 0x00 0x3D 0x40 |

AWB is in auto mode on both (`0x3406=0x00`), AEC auto (`0x3503=0x00`), SDE enabled
(`0x5001=0xA3`, `0x5580=0x02`, saturation U/V `0x40/0x10`).  The two cameras already sit
at different white-balance points (cam1 has 33 % more blue gain than cam0) while looking
at the same scene — that is the inter-camera mismatch to be removed.

The desktop at that moment showed a black/white test chart with a red bull's-eye; the
monitor fills the whole frame of both cameras (no monitor edge visible).  `iter1/initial_cam*.jpg`.
cam0 slightly pink-grey, cam1 clearly purple on the white areas.

## Step 2 — full-screen 24-patch grid, both cameras

`python3 show_pattern.py --grid` (ColorChecker Classic sRGB values, 6x4, mid-grey
background, `patterns/pattern_grid.png` + `pattern_grid.json`), then
`python3 capture.py --out iter1 --tag grid_full`.

- Each camera sees only about 3.5 x 3.5 patches of the 6 x 4 grid, and different parts of
  the screen (cam0 the right half, cam1 the left-centre; the two frames also look rotated
  180° with respect to each other).  The cameras are a few cm from the monitor, so one
  full-screen grid is not visible to either camera in full.
- AWB moved a lot with the scene: cam0 B gain 1377 → 2067, R 1566 → 1241; cam1 B 1835 → 1863,
  R 1423 → 1290; AGC 0x16/0x15 → 0x28 on both.  Scene-dependent auto white balance is the
  first thing a calibration has to freeze.
- Grey patches: bluish on cam0, lavender on cam1.  Dust spots visible on both sensors.

**Decision**: locate the monitor in each camera automatically (Gray-code structured light,
`locate.py`: white, black, 5 vertical + 4 horizontal stripe patterns → monitor cell per
camera pixel → homography monitor→camera on cell centroids), then draw the grid inside
the screen rectangle both cameras see (`show_pattern.py --grid --region x,y,w,h`), or one
grid per camera if the common rectangle is too small.

## Step 3 — monitor localisation (`locate.py`, Gray code)

`python3 locate.py --out iter1` — 11 patterns per camera (white, black, 5 vertical + 4
horizontal Gray-code stripe sets; 60 x 67.5 px cells), 22 captures in ~3 min total.

| cam | cells used | residual median / max (px) | screen rect seen (x, y, w, h) |
|---|---|---|---|
| 0 | 141 / 143 | 0.95 / 2.68 | 843, 229, 907, 653 |
| 1 | 136 / 137 | 0.82 / 2.38 | 492, 229, 907, 655 |

Both cameras see ~907 x 653 screen px (about 47 % x 60 % of the monitor); the two views are
offset by 351 px horizontally and cam1's frame is rotated 180° relative to cam0 (corner
order reversed in `cam_corners_on_screen`).  Common rectangle: **x 843, y 229, w 555, h 653**.
Outputs: `iter1/locate.json` (H monitor→camera and inverse per camera), `locate_cam*_overlay.png`
(decoded cell map; clean, no stray cells), `sl_cam*_*.jpg` (the raw captures).

## Step 4 — grid inside the common rectangle, baseline measurement

`python3 show_pattern.py --grid --region 843,229,555,653 --layout 4x6` (portrait layout, 24
patches of ~116 x 95 screen px), `capture.py --tag grid_before`, `analyze.py --tag grid_before`
(patch = inner 70 % of the screen rectangle mapped through H; metric dE76 = CIE76 in Lab, both
sides treated as sRGB).  All 24 patches in frame on both cameras.

| cam | mean dE76 | greys dE76 | worst | AWB R / B gain | AGC |
|---|---|---|---|---|---|
| 0 | 17.7 | 11.0 | yellow 34.5 | 1422 / 1536 | 0x2C (2.75x) |
| 1 | 41.4 | 33.1 | yellow 75.7 | 1374 / 1942 | 0x2C |

Findings: white clips at 255 on both (sRGB 243 target) while black (52) reads 31–35, so the
auto exposure over-exposes the grid (the AEC targets the frame mean, the grey surround is
mid-tone) and the tone curve is too steep; blue is in excess on both cameras (cam1 B ≈ 254
on most patches, greys read lavender).  `iter1/grid_before_analysis_cam*.png`.

## Step 5 — tuning, exposure + white balance (`tune_colors.py`)

Order chosen: exposure first on the green channel (the AWB green gain is fixed at 1.0x so the
two stages do not interact), then AWB on the mid greys, then (later) the colour matrix.

- **expo**: `0x3503 = 0x03` freezes AEC/AGC at the current values (exposure was already at
  its maximum for 30 fps, 0x3D40, so the AGC gain is the lever).  Target: 'neutral 8' green
  = 200.  cam0 AGC 44 → 31 (2.75x → 1.94x) in 3 iterations; cam1 44 → 47 (2.94x) in 2.  The
  gamma-2.2 model of the gain response (output ∝ gain^(1/2.2)) fitted the AGC well.
- **awb, first attempt = dead end**: same gamma-2.2 assumption for the AWB gains → the loop
  oscillated and diverged (cam0 B gain 1515 → 1095 → 1835 → 919 → 3124, dE 18.7 → 87.1).
  Measured response of the output to an AWB gain is close to linear, even slightly
  super-linear (exponent ≈ 1.3; the SDE saturation stage amplifies chroma), so the step was
  2–3x too large.  Fix: exponent estimated per step by a secant in log domain (clamped
  0.3–2.0), damping 0.7.
- **awb, second run**: cam0 converged in 5 iterations (R 1467, B 1287 → mid grey
  117.2 / 116.8 / 118.2), cam1 in 3 (R 1277, B 1227 → 153.3 / 154.7 / 156.6).

| cam | mean dE76 before → after | greys | worst remaining |
|---|---|---|---|
| 0 | 17.7 → 11.2 | 10.7 | dark skin 21.7 |
| 1 | 41.4 → 13.8 | 6.5 | yellow 33.3 |

Registers so far in `iter1/tuned_regs_cam{0,1}.json` (0x3503, 0x350A/B, 0x3406, 0x3400–0x3405).
Visual check `iter1/compare_expo_awb.png`: both cameras now render the chart neutral; cam1's
blue cast is gone, the mid-grey surround reads grey on cam0 and slightly green-grey on cam1.

Grey ramps after expo + awb (mean RGB vs sRGB target):

| patch | target | cam0 | cam1 |
|---|---|---|---|
| white | 243 | 225 / 238 / 232 | 255 / 255 / 255 |
| neutral 8 | 200 | 192 / 204 / 192 | 209 / 210 / 210 |
| neutral 6.5 | 160 | 136 / 135 / 140 | 180 / 182 / 186 |
| neutral 5 | 122 | 96 / 100 / 95 | 127 / 127 / 127 |
| neutral 3.5 | 85 | 48 / 56 / 48 | 64 / 65 / 65 |
| black | 52 | 18 / 26 / 17 | 28 / 27 / 24 |

Both tone curves crush the shadows (black 52 → 17–28, neutral 3.5 85 → 48–65) and the two
cameras differ in the mid-tones (neutral 6.5: 136 vs 182) although 'neutral 8' was equalised
by the exposure stage → the gamma curve is the next lever, per camera.  Remaining chroma
errors are hue/saturation (yellow too desaturated: B 85–122 instead of 31; red too orange;
blue too cyan) → colour matrix.

Tool gotcha fixed: the capture counter restarted at 1 for every run, so `tune_cam0_011_after.jpg`
(end of the diverged first AWB run) sorted after `007_after.jpg` (the good run); the counter now
continues from the files already present.

## Step 6 — tone curve + colour matrix (`tune_colors.py --stages expo,tone,awb,cmx`)

- **tone**: the monitor switches to a 24-step grey ramp (`show_pattern.py --ramp`, 8 … 250 in
  the same 4x6 layout/region); the 15 gamma bytes 0x5481–0x548F (Y output at the OV5640 knot
  inputs 8, 16, 32, 64, 96, 128, 160, 192, 208, 224, 240, 248, 252, 254 + slope) are tuned by
  Gauss-Newton: Jacobian from +12 perturbations (15 captures), least squares on
  (measured luma − target) over the 24 greys with Tikhonov + second-difference smoothness,
  damping 0.7, clipped 0–255, forced monotonic.  2 rounds, then the grid is restored.
  Model-free on purpose: no assumption about the knot semantics is needed to replay it on
  another sensor.
- **cmx**: same scheme on the 9 colour-matrix magnitudes 0x5381–0x5389 (sign bits 0x538A/B
  untouched), error = RGB delta over the 24 patches, greys weighted x2, step +16, 2 rounds.
- Run log: `iter1/tune_full_run1.out`, per-capture audit `iter1/tune_cam{0,1}.jsonl`.

**Run 1 = dead end, invalid data.**  From the first capture of the run 'neutral 8' read 49
instead of 200 and the exposure stage pushed the AGC to 15x; the grey ramp came back
non-monotonic (56 57 52 50 252 253 198 70 …) and the tone/CMX stages optimised garbage
(cam0 gamma → 17 1F 2F 4F 58 70 A8 A8 …, CMX 55/166/214/235 …; cam1 similar; dE after 35).
Cause, seen in `iter1/tune_cam0_024_tone0_base.jpg` and `popup_check_cam0.jpg`: a GNOME
Shell **modal dialog** (white rounded box, centred on the screen) sat on top of the pattern
and the shell dimmed everything behind it.  The operator, who was at the machine, identified
it as a **user / password (authentication) prompt**; what raised it is unknown (nothing in
the tuning chain asks for privileges).  It was gone after the operator's intervention and
the pattern window restart (`dialog_killed_cam0.jpg` shows the bare desktop).  Screenshots
over D-Bus (`org.gnome.Shell.Screenshot`) are denied in this GNOME, so the camera itself
was the only way to see the screen.
Second trap found while fixing this: the SSH shell carries `DISPLAY=localhost:10.0` (X
forwarding), so a GTK window started with `GDK_BACKEND=x11` silently opened on the remote
X server.  `show_pattern.py` now forces `DISPLAY=:0` + Mutter's XWayland authority file
(`/run/user/<uid>/.mutter-Xwaylandauth.*`) and runs through XWayland; `xwininfo -root -tree`
on `:0` shows "s265 pattern" at 1920x1080+0+0 and `x11_grid2_cam0.jpg` shows the grid again.
The tuner gained a `reset` stage (SDK defaults for CMX, gamma, SDE; AWB/AEC back to auto)
to recover from a run like this one.  Rule for the replay script: **check that the pattern
is actually visible before tuning** (a white/black reference capture whose means must match
the locate run within a few levels, else abort).

## Step 7 — run 2, valid (`tune_full_run2.out`, stages reset → expo → tone → awb → cmx)

| cam | expo (AGC) | tone: ramp luma rms | awb R / B gain | cmx rms | mean dE76 after | greys | worst |
|---|---|---|---|---|---|---|---|
| 0 | 31 (1.94x), 3 it | 17.8 → 6.5 | 1471 / 1267, 6 it | 18.7 → 17.5 | **10.6** (stock 17.7) | 7.5 | magenta 23.8 |
| 1 | 47 (2.94x), 2 it | 29.1 → 11.9 | 1290 / 1213, 7 it | 26.0 → 23.8 | **18.0** (stock 41.4) | 6.6 | yellow 53.1 |

Tuned gamma: cam0 `18 2A 3A 5C 69 73 7D 87 90 99 A6 B5 CD E1 F1`, cam1
`15 22 35 5A 61 64 77 8E 96 96 96 9D B6 C1 D6` (the shadow lift is what the SDK S-curve lacked;
cam1's curve is noisier because its ramp readings are noisier at 2.9x gain).  Greys now within
5–10 dE on both cameras (black 52 → 50/56/46 and 42/41/33).

What is left is chroma, mostly on cam1: yellow (231,199,31) reads (206,162,121), magenta
(187,86,149) → (171,143,143), i.e. the blue/yellow axis is strongly desaturated; cam0 has the
same pattern at a third of the amplitude.  The CMX stage did not fix it because its step was
too timid (Tikhonov 0.05·trace, damping 0.7, 2 rounds: rms moved 26.0 → 23.8 only).
Next: SDE saturation U/V (0x5583/0x5584, 2 parameters) first as a global chroma scale, then
CMX with step 20, damping 0.8, Tikhonov 0.01, 3 rounds, then an AWB touch-up.
`iter1/compare_run2.png` = stock vs run 2 vs the reference image.

### Check: does one camera have inverted / swapped colour channels?  (operator suspicion)

Test on the 14 saturated patches: fit measured → target with each of the 6 channel
permutations (per-channel gain + offset allowed, so a cast cannot mask a swap) and compare
hue angles in Lab for the six primaries.

| capture | identity rms | best non-identity rms | verdict |
|---|---|---|---|
| cam0 stock | 20.7 | 38.2 (BGR) | no swap |
| cam1 stock | 21.1 | 41.7 (BGR) | no swap |
| cam0 run 2 | 16.7 | 35.0 | no swap |
| cam1 run 2 | 25.0 | 41.9 | no swap |

Hue angles (target → measured): cam0 primaries all within ±13° except cyan (+34°, reads
bluer).  cam1: red +15°, green −6°, blue −5° fine, but **yellow chroma 78 → 30 and hue −24°,
magenta chroma 51 → 11 (+36°), cyan −53°**; in the stock capture cam1's yellow had chroma 18
at hue 16° (it looked salmon-pink, which is what reads as "inverted" by eye).  So: no channel
inversion on either camera, but cam1 has a much weaker and skewed response on the
yellow–blue / magenta–green axes than cam0 with identical registers.  Hypothesis to test
next: a hardware difference between the two modules (IR-cut filter or lens batch), measured
by the 3x3 response matrix of each camera to full-screen R, G, B primaries with the CMX
bypassed (0x5001 bit1 = 0) and AWB manual at unity.

Operator (looking at the contact sheets): "blue and red look swapped on one camera".  That is
the 180° mounting rotation of cam1: in the raw frames the patch positions are mirrored, so
blue sits where red is on cam0.  `rectify.py` warps both cameras into monitor coordinates
through the locate homographies: `iter1/rectified_stock.png`, `rectified_run3.png` show all
24 patches in the same place on both cameras with the right hues.  No swap.

## Step 8 — run 3, chroma stages (`tune_chroma_run3.out`: sde → cmx → awb)

| cam | sde U/V sat | cmx rms | awb R / B | mean dE76 | greys | worst |
|---|---|---|---|---|---|---|
| 0 | 64/16 → 72/48 | 17.3 → 16.7 | 1396 / 1278 | 10.6 → **9.0** | 7.4 | cyan 17.9, red 17.7 |
| 1 | 64/16 → 72/89 | 23.6 → 22.9 | 1215 / 1228 | 18.0 → **17.5** | 6.6 | yellow 51.1, magenta 45 |

The chroma stages move the registers but hardly the error.  Sanity test of the ISP blocks
(`isp_test_cam*_*.jpg`): SDE grey mode (0x5580 = 0x22) turns both cameras grey (RGB spread
124 → 0.7), CMX bypass (0x5001 bit1 = 0) scrambles the colours, restore brings them back —
the registers do act, the Jacobians are large (a +20 on one CMX coefficient moves a grey by
30 levels), so the solver is not blind; the residual simply is not reachable by a 3x3 matrix
or a saturation scale.  cam1's yellow (B 116 instead of 31) and magenta (desaturated) are
the bulk of its error and do not respond to U/V gain.

## Step 9 — colour shading (`shading_map.py`, uniform grey 150 full screen, 8x6 cells)

| cam | G min..max (falloff) | R/G min..max | B/G min..max | pattern |
|---|---|---|---|---|
| 0 | 108..167 (35 %) | 0.86..1.02 | 0.94..1.07 | green-ish centre, R/G and B/G rise towards all edges |
| 1 | 111..162 (31 %) | 0.89..1.11 | 0.95..1.09 | pronounced **pink ring** at the edges (R/G 1.04–1.11 all around, 0.89 centre) |

`iter1/shading_map.png`.  Both modules have residual colour shading with the SDK's stock
LENC table (the NXP table was tuned for their module batch); cam1 has the stronger, ring-shaped
red excess.  Patches near the grid edge in cam1's frame (its right column: foliage, blue,
orange-yellow, yellow, neutral 8, black) read washed-out and bluish in `rectified_run3.png`,
consistent with position-dependent colour error.

### Is cam1 a defective camera or a defective board?

Evidence: (1) the stock cam1 frame is strongly magenta/purple while cam0 is near neutral
with identical registers; (2) cam1 needed 33 % more blue AWB gain than cam0 on the same scene;
(3) cam1's blue channel stays high on yellow/orange/red patches (yellow B 116 vs cam0 66);
(4) a pink ring in the shading map.  All four are the textbook signature of a **missing or
different IR-cut filter** (or a lens/filter batch mismatch) on the cam1 module.  The board
only carries the MIPI bit stream; a board fault would corrupt pixels or lines, not shift
hues smoothly.  Two quick physical checks for the operator: point a TV remote at both cameras
(a camera without IR-cut shows the remote's LED as a bright white-violet spot, one with the
filter barely sees it), or swap the two modules between the connectors and see whether the
magenta cast follows the module.

## Step 10 — full-screen primaries (`primaries.py`, operator's suggestion)

Centre 40 % of the frame, current tuned state (manual AEC/AWB), so the driven channel clips
at 255; the leakage into the other two channels is the information:

| screen | cam0 RGB | cam1 RGB | note |
|---|---|---|---|
| red | 255 / 58 / 31 | 254 / 82 / 25 | cam1 +24 green leak |
| green | 99 / 254 / 76 | 118 / 245 / 108 | cam1 +19 red, +32 blue leak |
| blue | 50 / 104 / 254 | 32 / 142 / 255 | cam1 +38 green leak |
| yellow | 254 / 255 / 89 | 255 / 255 / 109 | cam1 +20 blue leak |
| white | 255 / 255 / 255 | 254 / 240 / 239 | |
| black | 13 / 9 / 12 | 1 / 8 / 7 | |

cam1 separates the monitor primaries clearly worse than cam0 (every off-channel 20–40 levels
higher): lower colour purity across the board, the global signature of a different spectral
response (IR-cut filter / CFA batch), on top of the ring-shaped shading.  `iter1/primaries.png`.
A stronger colour-correction matrix is the remedy for the global part; fitted analytically
from the 24 patches (`stage_ccm`: C = lstsq(out → target), row-normalised, CMX' = K·C·K⁻¹·CMX
with K = BT.601) instead of the Jacobian search.

## Step 11 — analytic CCM, exposure, and the yellow mystery on cam1 (`tune_ccm_run4.out`, …)

- **Analytic CCM (`stage_ccm`)**: the least-squares fit itself predicts almost no gain
  (cam0 RGB rms 16.6 → 16.3 predicted, cam1 24.0 → 23.9; the fitted C is within ±0.1 of
  identity).  So the residual is **not** a 3x3-matrix problem: it is non-linear or
  position-dependent.  Final: cam0 dE 8.7, cam1 17.4 (`tuned_regs_cam*.json` updated with the
  CMX magnitudes + sign bits).
- **Exposure clipping?** cam1 re-exposed with 'neutral 8' at 165 instead of 200 (white 232,
  nothing clipped): yellow still (204,152,116), dE 56 → raw clipping is not the cause.
- **Yellow screen shading map** (`shading_map.py --hex b3b300`, `shading_b3b300_map.png`):
  cam0 B/G 0.25 (centre) … 0.47 (edges), smooth; **cam1 B/G 0.6–0.7 in the central disc,
  0.3 at the edges, R/G 1.1 centre vs 1.02 edges** — a pale pink-beige disc in the middle of
  a yellow field.  The anomaly is spatial and specific to the colour of the light (for grey
  the centre of cam1 is *greener* than the edges, R/G 0.89 vs 1.1).
- **Stray light refuted** (`glare_test.py`): a 220 px yellow square at each camera's frame
  centre on black / white / grey backgrounds: cam0 (238,212,60) / (247,222,79) / (239,214,63),
  cam1 (254,184,82) / (254,190,80) / (254,185,82).  cam1's rendering of the square does not
  depend on the surround, so it is not veiling glare, haze or screen reflection; but even on
  black cam1 renders ColorChecker yellow as orange (R clipped, G 184, B 82) where cam0 gets
  (238,212,60).

Open hypotheses for cam1, both needing hands on the hardware:
1. **Module optics** (IR-cut interference filter missing or with a different cutoff — its
   pass band shifts with the angle of incidence, which gives exactly a centre-vs-edge colour
   difference that depends on the wavelength of the light).  A white-LED monitor emits almost
   no near-IR, so a *missing* filter alone would not show on this target; a *shifted* visible
   cutoff would.
2. **LCD viewing angle**: the cameras are a few cm from the panel and look at different parts
   of it at different angles; an LCD's colour and gamma change with viewing angle.  cam1's
   module is mounted rotated 180°, so any tilt is mirrored.  Philips 234E5 is an IPS-class
   panel (small but non-zero shift).

Discriminating tests (operator): (a) TV remote at both lenses (IR filter present or not),
(b) rotate the whole board 180° or swap the two modules — if the pale-centre yellow follows the
module it is the camera, if it stays with the screen position it is the monitor angle,
(c) move the board ~30 cm back (angles shrink; if cam1 normalises it was geometry).

### The dialog, identified (second occurrence, ~20:45)

`iter1/screen_check_cam0.jpg` caught it in full: GNOME Shell's keyring prompt,
"Authentication required — The login keyring did not get unlocked when you logged into your
computer", with Cancel / Unlock.  It is drawn by gnome-shell itself (no separate prompter
process to close) and dims the whole screen behind it, so every capture taken while it is up
is invalid: the centre-mode measurement (`measure_centre.out`: cam1 all patches 254/244/236 =
the dialog's white box, cam0 everything dark) and the fixed-saturation A/B test
(`uvadj_*.jpg`) both have to be discarded.  Only the operator can dismiss it (Cancel, or
unlock the keyring once with the login password so it stops coming back).  Two guards were
added so this can never again silently corrupt a run: `tune_colors.py` aborts when the grid's
white patch reads < 150 or white − black < 80 ("pattern not visible"), and `show_pattern.py`
now keeps one persistent fullscreen window that swaps images through a control file instead
of restarting (the X11 restart took > 1 s to map, so the first centre-mode captures saw the
desktop even without the dialog).

### Research note and the UV-adjust hypothesis (dead end)

`COLOR_CALIBRATION_SOTA.md` (research agent, 467 lines, 58 sources) is in the folder.  Its
first recommendation: the stock SDE configuration (0x5588 bit 6 = 0, 0x5583 = 0x40,
0x5584 = 0x10, thresholds 0x5589–0x558B) runs "UV adjust" in automatic mode, desaturating
chroma with the AGC gain — which would couple the exposure stage into saturation and hit
cam1 (2.94x) harder than cam0 (1.94x).  A/B test on a clean screen (`uvadj_ab.json`,
`uvadj_{auto,fixed}_cam*.jpg`): auto vs fixed saturation (0x5588 |= 0x40, 0x5583 = 0x5584 =
0x40) changes nothing — cam1 mean dE 19.7 → 19.7, yellow (198,152,120) → (203,152,120);
cam0 8.6 → 8.7.  At our gains the effect is below the noise; the cam1 yellow problem is not
this.  Other points from the note that do apply: the OV5640 gamma is a RAW-domain block
before the CMX (so the matrix acts on gamma-encoded data and must be fitted in the operating
configuration, as done); fit the CMX analytically with Y-row-sum 1 / U,V-row-sum 0
constraints; do not fit LENC against a screen a few cm away (the panel's angular colour shift
would be baked in); use ΔE2000 and hold-out patches for validation; a physical ColorChecker
is required for any absolute claim.

Decision for the tuning method independent of the cause: colour measurement must stop
mixing field position with colour.  Next version of the grid stage shows each patch at the
camera's frame centre (sequential, 24 captures per camera) for the CMX/SDE fit, and the LENC
stage flattens the field separately on uniform grey.

## Step 12 — patches one at a time at the frame centre (`--centre`, `measure_centre2.out`)

Persistent window, 220 px square on grey 118 at the point each camera's optical centre
looks at, 0.7 s settle, 24 captures per camera (~50 s).

| cam | mean dE76 grid | mean dE76 centre | yellow grid → centre | magenta | red | neutral 5 |
|---|---|---|---|---|---|---|
| 0 | 8.7 | 9.3 | (239,212,64) → (238,213,79) | (164,93,144) → (207,100,172) | (206,69,50) → (199,67,63) | (110,113,108) → (112,128,119) |
| 1 | 19.7 | 17.5 | (204,152,116) → **(254,185,83)** | (159,138,137) → (179,140,132) | (207,79,30) → (207,80,46) | (97,100,100) → (125,148,143) |

At the centre cam1's yellow loses most of its blue excess (B 116 → 83) but turns orange
(G 185, R clipped), and the mid grey turns green-cyan (125/148/143) — the same centre-green /
edge-pink shading seen on the grey field.  So on cam1 the colour error is dominated by the
field position and the measurement must be paired with a LENC fix (not against the screen,
per the research note) before any matrix fit can mean anything.  cam0 is consistent between
grid and centre (dE 8.7 vs 9.3): its remaining error is a genuine matrix/tone residual.
Settle time is not a factor: the same four patches read identically at 0.7 / 1.5 / 2.5 s.

## Step 13 — replay test (reproducibility)

Full register snapshot per camera (47 registers: 0x3503, 0x350A/B, 0x3500–02, 0x3400–0x3406,
0x5381–0x538B, 0x5481–0x5490, 0x5580/83/84/88/89/8A/8B) → `iter1/tuned_regs_cam{0,1}_final.json`.
Then `sentai.sys.reset()` (board back in 5 s), `camera.init()`, stock measurement, and
`tune_colors.py --apply <json>` on each camera:

| cam | stock after reboot | after replay | tuned value earlier today |
|---|---|---|---|
| 0 | 20.0 | **8.7** | 8.7 |
| 1 | 42.8 | **19.6** | 17.4 – 19.7 |

The saved set reproduces the tuned state on a cold boot (cam0 exactly, cam1 within its
measurement spread).  Apply order in the JSON is the dict order; no group write was needed
for a static apply.  This is the artefact a tuning script for other modules has to produce.

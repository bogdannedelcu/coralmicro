---
name: project_s265_camera_color_calibration_w24_t18
description: "2026-11-01 EXP-s265 (OP-S10-W24-T18): both OV5640 cameras calibrated against a monitor grid — tooling (locate.py Gray-code homography, tune_colors.py closed-loop register stages, replay JSON), results cam0 dE76 17.7→8.7, cam1 41.4→17.4 with a hardware anomaly (field-position colour error, suspected IR-cut filter), dead ends (GNOME keyring dialog over the pattern, X forwarding display, UV-adjust)"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-02T04:38:27.528Z
---

**EXP-s265 `examples/sentai_runtime/experiments/s265_camera_color_calibration/` (W24-T18), board 58eb build 1756, no firmware change.**
Operator goals: each camera as close as possible to the real (sRGB) colours, every step journaled
(`JOURNAL.md`) so a tuning script can be replayed on other OV5640 modules from stock.

- **Setup gotchas (host):** the calibration monitor is a Philips 234E5 on the RTX 4090 HDMI-3; the
  Unreal `monitor_power_saver.sh` blanks it every 3 s unless `/tmp/mps_pause` exists.  Show patterns
  with `show_pattern.py` = ONE persistent GTK window via XWayland (`GDK_BACKEND=x11`, `DISPLAY=:0`
  forced — the SSH shell carries `DISPLAY=localhost:10.0` and the window silently opens remote;
  XAUTHORITY = `/run/user/1000/.mutter-Xwaylandauth.*`), image swapped through a control file
  (window restarts take > 1 s to map).  Screenshots over D-Bus are denied; the camera is the only
  way to see the screen.  **GNOME keyring "Authentication required" dialog** appeared twice over the
  pattern and dimmed the screen → two runs produced garbage; `tune_colors.py` now aborts when the
  white patch < 150 or white−black < 80.  Dismissing/unlocking the keyring is the operator's call.
- **Board gotchas:** embed MicroPython has no `time` module (host sleeps between REPL commands);
  `camera.init()` is needed after every reboot before `select()`/`reg_read()` work (they return -1);
  `camera.reg_read/reg_write(cam, reg[, val])` exist; `fs.read_base64` is the download path (HTTP
  needs `usb.ip(1)`).  cam1 is mounted rotated 180° vs cam0 (patches mirrored — looks like a R/B swap
  to the eye; `rectify.py` warps both into monitor coordinates to compare).
- **Method that works:** Gray-code structured light (white, black, 5 vertical + 4 horizontal) →
  homography monitor→camera per camera (residual < 1 px; each camera sees ~907x653 screen px, common
  rect 843,229,555,653) → 4x6 ColorChecker grid in the common rect → closed-loop stages on sensor
  registers: `expo` (0x3503 manual, AGC until 'neutral 8' G = 200; exposure already at the 30 fps
  max), `awb` (0x3406 manual, R/B gains on mid greys — response is ~linear, NOT gamma-2.2: the
  gamma model oscillated/diverged), `tone` (gamma 0x5481–0x548F by measured-Jacobian Gauss-Newton on
  a 24-step grey ramp), `cmx`/`sde` (Jacobian GN) and `ccm` (analytic: CMX' = K·C·K⁻¹·CMX).  Results
  dE76 cam0 17.7 → 8.7, cam1 41.4 → 17.4; replay of the 47-register JSON after `sys.reset()` +
  `camera.init()` reproduces 8.7 / 19.6.  The matrix stages barely help: residual is not 3x3-linear.
- **cam1 anomaly (open, hardware suspected):** stock frame purple, 33 % more blue AWB gain, blue
  stays high on yellow/orange, pink ring in the grey shading map, centre green / edges pink; at the
  frame centre ColorChecker yellow → orange (254,185,83), moderate red → orange (198,142,64), grey →
  green-cyan.  Not stray light (patch colour independent of surround), not raw clipping, not the
  SDE "UV adjust" auto desaturation (A/B identical), not fixable by AWB/CMX/SDE/tone.  Candidates:
  IR-cut filter missing/different on the module, or LCD viewing angle × module tilt.  Physical checks
  for the operator: TV remote at both lenses, swap modules, move the board back 30 cm.
- **Research note** `COLOR_CALIBRATION_SOTA.md`: OV5640 gamma is RAW-domain before the CMX (fit the
  matrix in the operating configuration), don't fit LENC against a screen a few cm away, use ΔE2000 +
  hold-out patches, a physical ColorChecker is needed for absolute claims; the display can't set the
  deployment white point.
- **LENC done (`lenc_tune.py`, paper sheet on the white monitor as diffuser):** G matrix 0x5800–0x5823
  6x6 = luminance gain for all channels (matrix row 0 = image bottom), BR bytes 0x5824–0x583C 5x5 high
  nibble = B gain, low nibble = R gain (weak); measured Jacobian (86 captures) + damped LS flattened
  colour shading to ±4 % on both cameras in 2 rounds.  Most of cam1's "pink ring" on the bare LCD was
  the panel's viewing angle (through the diffuser cam1 ≈ cam0).  Grid dE after LENC + AWB: cam0 8.4,
  cam1 19.6 — unchanged → cam1's residual is RAW-domain G/B crosstalk (green screen → B 109 vs 76, blue
  → G 133 vs 104) that the OV5640 cannot undo because its gamma runs BEFORE the colour matrix; fix is
  physical (module/IR filter).  Do NOT white-balance on the paper (spectrum shifts warm); AWB on grid greys.
  Full sets incl. LENC: `iter1/tuned_regs_cam{0,1}_final2.json` (109 regs).
- **Later the same night:** ΔE2000 added to analyze.py (final Philips: cam0 5.7, cam1 10.7). Printed
  A4 chart (`make_chart_pdf.py`, `locate_print.py` from corner fiducials) — runs under lamps invalid
  (light changed during runs, AGC 30-44x, PWM flicker): rule = one steady illuminant, AGC <= 4x, lamps
  at 45°. **Huion Kamvas 16 (sRGB mode) as reference**: same registers → cam0 ΔE00 7.6 vs 5.7 on the
  Philips (~2 ΔE00 reference uncertainty). A lens a few cm from a panel sees a 25-40 % luminance
  gradient across the field (angular falloff, worse on the matte pen display) → spread-out ramps and
  grids are unusable there; `--centre` mode (one patch at the frame centre) + `tone_centre` (knot update
  from the measured ramp, no Jacobian) + expo/awb at the centre: **cam0 ΔE00 4.5 (greys 3.8), cam1 7.8
  (4.4)**; grid mode is 2x worse for the same registers. Register sets `iter3_kamvas/tuned_regs_cam*_kamvas.json`.
  Expo stage: output ∝ gain^e with e 0.45–1 → secant exponent (fixed 1/2.2 oscillated).
- **GNOME keyring prompt** ("Authentication required") kept popping over the display (autologin +
  custom `keyring-unlock.service`; keyring deleted → broken collection): ruined 3 runs. Fixed by the
  operator (blank-password login keyring + `systemctl --user restart gnome-keyring-daemon.service`);
  Claude Code's permission classifier refuses the blank-keyring step ("security weaken") — don't retry
  it, let the operator run it with `!`. Guards: grid white/black and centre-mode bright/dark checks.
- **cam1 root cause ESTABLISHED (RAW-gamma-off diagnostic):** with 0x5000 bit5 cleared (sensor gamma
  bypassed, linear output) a fitted 3x3 matrix takes cam1 from RGB rms 13.4 → 2.5 vs linearised sRGB
  targets (yellow exact; C off-diagonals −0.20/−0.18 = strong G/B crosstalk). The OV5640 applies gamma to
  RAW before the CMX, so no in-sensor matrix can undo it (CMX/CCM stages: no gain on grid or centre
  data). Module is fine; fix is architectural: linear sensor output + CMX in linear domain + sRGB LUT
  downstream (M7 256-entry LUT, PXP LUT mode, or models on linear input), or a lower-crosstalk module.
  OTP 0x3D00-1F is all zero on both modules (no per-module id) → identify by connector + AWB fingerprint.
  Hold-out (6 patches) and `--avg N` frame averaging added to tune_colors.py.
- **EXP-s266 (W24-T19, alternation vs monitor time code)**: Gray-code stripe counter at the Kamvas frame
  clock (`timecode_display.py`, pixbuf scaling — python3-gi-cairo is NOT installed, Cairo draw fails),
  board reads column 40 per dequeued frame via `camera.peek_col(40)` with per-camera band rows from the
  s265 homographies; VGA 30 fps ratio(1,1): 960 frames / 30 s, 33.43 ms, 0 stale/dup, per-camera step
  4 display frames, cam1 one display frame later (rolling shutter + rotated module). Display jitter 59.2 Hz
  ± 1.8 ms is the residual. Exposure 3 ms / gain 6x for the code; cameras restored to the Kamvas sets after.
  Mode sweep (`run_mode.sh`): VGA45 PASS (0 broken codes) — but a mode change shifts the image ~20 rows,
  so locate.py must be re-run per mode; VGA60 inconclusive (7 % frames split between two display frames
  = display tearing at camera period ≈ display period, or camera mix; needs a redundant two-copy code);
  SXGA: TWO FIRMWARE BUGS fixed in builds 1757–1760 — `sentai_cam_capture_jpeg` used compile-time
  DEMO_CAMERA_WIDTH/HEIGHT as the source size (every SXGA JPEG → -5 "cam capture fail"; now runtime
  g_runtime_cam_w/h, SXGA JPEG encoded at 640x480, packed → -7) and `sentai_cam_peek_col` used the VGA
  pitch/bounds (two interleaved columns at SXGA; now runtime geometry, packed pitch (w/2)*4, rows =
  g_runtime_cam_h, new 2nd arg `byte`: 0 = B/X, 1 = Y1 packed). Bug class: compile-time VGA constants
  in paths that run at SXGA/packed — a review agent audited it (see s266 README / its report).
  SXGA15 legacy 1:1: cadence 66.86 ms sd 0.01, cam1 240/240, 0 stale (PASS); cam0 column 40 ambiguous
  at SXGA (geometry differs from 2x VGA homography, open). SXGA30 packed: the REPL `peek_col` loop only
  keeps up with every other 2.5 MB frame → use `camera.frame_log(64)` for cadence; cam1 packed codes
  garbage (open). Exposure per mode: lines scale with line time (VGA45 150 lines ≈ 3.4 ms, VGA60 200,
  SXGA15 50, SXGA30 100).
- **W24-T20 DONE (2026-11-02, builds 1762–1763, EXP-s267 smoke PASS in VGA30 / SXGA15 / SXGA30 packed):**
  all 9 audit findings fixed. Rules now in force: readers of raw frames use `cam_src_w/h()` /
  `cam_src_pitch()` / `cam_src_packed()` (sentai_runtime.cc top); `sentai_cam_get_width/height()` =
  SOURCE geometry, `set_resolution()` is output-only via `sentai_cam_get_out_width/height()`; D-cache
  maintenance in camera.cc = `LiveFrameBytes()`; packed YUV420 refused by jpeg (-7), flow.start (-4,
  SERR 0x0E21), RGB-model pipeline.start (-9, SERR 0x0448), aux slots skipped (0x0448 val 2),
  zero-copy grey -1. Smoke-test gotchas: `Repl.value()` returns only the LAST line (use `cmd()` for
  dmesg), `packed420(1)` returns 1, `flow.start` raises on error, prep producers fire only with a
  slot enabled (`prep_enable(0)`), dark monitor → peek_col all zeros.
- **SIM link is broken at HEAD (pre-existing, OP-S10-W24-T21 TODO):** ~26 ARM-only W24 symbols
  (cam geometry, pack420_drop, chase, TPU URB stats, sysfs, imu, health); the `"%nobits @"`
  section trick must be guarded for SIM (GAS x86 rejects it).
- **s266 SXGA30 packed 2:2 FIXED (iter8, build 1764):** root cause = sensor-period estimate folded to 16.7 ms by a plausibility check against the PREVIOUS estimate (boot default 22288 us); genlock trims wrong by 2x, 0x0A0D val 40, drift monitor forced ratio_unit 2. Fix: window from the nominal period of the fps key + reset at init (`cam_nominal_period_us`). Now SXGA30 packed / SXGA15 / VGA30 all 1:1 with genlock ok (frame_log_check.py reads the ring WHILE a consumer dequeues). Earlier note kept for history: **s266 SXGA30 packed finding (iter7, 1763):** `ratio(1,1)` alternates 2:2 (FB-done ring tags
  1001…, two entries per `seq` → the alternation counter advances once per two frames in this mode,
  "frame_count half rate" family) and the two sensors are ~17 ms out of phase (33.4/16.4/33.4/50.6 ms);
  `stats()["period_us"]`=16713 there is the 60 Hz grid, not the sensor period. Ring was read with no
  consumer → re-check with the pipeline running. cam0 SXGA column 40 ambiguity is geometry, not cache.
- **Production integration DONE (W24-T22, build 1765):** /cam/cam0.regs + /cam/cam1.regs on the user partition (camera_modules/make_regs.py --upload, 96 regs each, exposure freeze + manual AWB dropped by default), applied automatically at the end of every camera.init() (group-hold batches, refused address classes, E:0A13/0A14), `camera.load_regs(cam[,path])` on demand; read-back verified both cameras. **Measured policy (build 1766, camera_modules/measure_variants.sh):** auto exposure + tuned manual AWB gains = ΔE00 8.7 / 12.4 (equals the calibration; frozen exposure 9.1 / 13.9) while the sensor's own AWB gives 17.9 / 17.8 → make_regs.py default keeps the AWB gains (103 regs). **GOTCHA:** OV5640 group-hold (0x3212 03/13/A3) batches read back correctly but leave part of the ISP set ineffective (ΔE76 21.8/35.7 vs 15.2/21.9 with direct writes) → never apply colour sets through group hold. When a module moves/changes: update modules.json, re-run make_regs.py --upload.
- Open: the linear
  pipeline option above, print target under one steady illuminant, s266 soak / VGA45 / select().

- **Board rotated 180° test (2026-11-02, camera_modules/measure_position.sh):** per-patch grid-mode errors on the Kamvas swing ±10–15 ΔE00 on BOTH cameras when the geometry changes → mostly panel viewing angle, not module; means cam0 8.7→9.5, cam1 12.4→10.0 (gap 3.7→0.5). Module-intrinsic residual of SAI-CAM-0002: magenta/moderate red ~22 in both positions. Grid-mode numbers on a display at a few cm are position-dominated; use centre mode or compare means only.
- **Distance test (board ~45 cm, grid central 70 %, exposure normalised on neutral 8):** cam0 ΔE00 7.9, cam1 8.4 — modules within 0.5 ΔE00 once the panel angle is removed; cam1 signature = yellow→orange (crosstalk), cam0 yellow dark. With the screen on black at distance, AUTO exposure clips the bright patches (frame-mean AEC) → always normalise exposure (`measure_position.sh <tag> 0.7 expo`). Next step if needed: re-tune at this distance with all 24 patches at once.
- **TODO W24-T23 (operator 2026-11-02): colour auto-tuning on the M7 at zero FPS cost** — M7 white balance from hardware statistics (sensor AWB/AVG regs or the existing PXP prep slot), per-module correction on top, 0x3400–0x3405 at ~1 Hz; AEC target steering; metric = detector score on reference images. Hard constraint: no full-frame per-pixel pass on the M7, FPS unchanged.
- **Re-tune at 45 cm NOT adopted (s265 iter4_dist45):** tune-after 6.4 for cam0 was within the run-to-run spread (~1 ΔE00); A/B at equal exposure normalisation (n8=190): new 7.8 vs old kamvas set 6.7; cam1 tone stage over-fits the noisy ramp (9.4). Production sets unchanged (kamvas 2026-11-01 both). Rule: on this display setup differences < 1.5 ΔE00 need repeated A/B or a physical chart; exposure target and tone curve are entangled (a set tuned at n8=190 clips at 200). **s268: OV5640 YUV output is FULL range (0–255), buffer bytes [V,U,Y,X], YUV path ΔE00 = RGB path (8.1/8.5 vs 7.9/8.4); grey U/V within ±5 of 128.**
- **EXP-s269 (2026-11-02): on-board detection had been silently DEAD** — `sentai_tpu_detect` returned -2 for the headless two-head p3p4 exports (all iarna_*/yuv420 models) → 0 detections in every pipeline run since; FIXED build 1769 (`yolo_decode_heads`, port of s236/host_decode.py) + `sentai.tpu.detect()` binding. STILL OPEN: `tpu.load_image`+`tpu.invoke` arena path does not feed the TPU (fingerprint constant; pipeline direct path OK); virtual camera compiled out (W24-T11) → no camera-independent baseline; display recall on EDAPA frames = 0 (people 16–40 px, no candidates on them). Never trust a speed sweep as a detection test. MSC upload: drive(1) → /run/media/bogdan/0000-0001 → copy → udisksctl unmount → drive(0). FAT path limit: long file names fail fs.read/load_image (-3) → use short names.
- **Host baseline (s269 host_eval.py, CPU int8 c2f / c2f_thick on raw EDAPA 640x480 frames): recall only 0.09–0.28** — the current detectors (trained on cover_v1 / iarna) barely detect EDAPA people even without display/camera; the 0 through the display is model + 0.4x scale, not a broken chain. A detector-based metric needs an EDAPA-trained model or frames from the training domain. models_yuv/*.tflite are edgetpu-compiled (no CPU run).

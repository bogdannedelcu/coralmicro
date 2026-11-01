---
name: project_s265_camera_color_calibration_w24_t18
description: "2026-11-01 EXP-s265 (OP-S10-W24-T18): both OV5640 cameras calibrated against a monitor grid — tooling (locate.py Gray-code homography, tune_colors.py closed-loop register stages, replay JSON), results cam0 dE76 17.7→8.7, cam1 41.4→17.4 with a hardware anomaly (field-position colour error, suspected IR-cut filter), dead ends (GNOME keyring dialog over the pattern, X forwarding display, UV-adjust)"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-01T21:29:15.442Z
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
- **W24-T20 TODO (audit 2026-11-02, `agent/audit_runtime_geometry_2026-11-02.md`):** 9 more places use
  compile-time VGA constants as SOURCE geometry at SXGA/packed. #1 = D-cache invalidate/clean in
  libs/camera/camera.cc sized 640x480x4 (m_ncamera is write-back cacheable: MPU region 9 = all SDRAM) →
  rows ≥ 240 of an SXGA frame can be stale for every CPU reader (incl. the fixed peek_col). Then aux
  slots (markers/places/flow), RGB model prep, flow publisher, capture_rgb/to_tensor/aifes, jpeg_yuv,
  peek5_*, yuv420_extract_test, native_res(), set_resolution() as source pitch. Fix next session.
- Open: production integration (register set per camera after init, group write 0x3212), the linear
  pipeline option above, print target under one steady illuminant, s266 soak / VGA45 / select().

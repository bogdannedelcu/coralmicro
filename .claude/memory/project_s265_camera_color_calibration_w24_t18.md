---
name: project_s265_camera_color_calibration_w24_t18
description: "2026-11-01 EXP-s265 (OP-S10-W24-T18): both OV5640 cameras calibrated against a monitor grid — tooling (locate.py Gray-code homography, tune_colors.py closed-loop register stages, replay JSON), results cam0 dE76 17.7→8.7, cam1 41.4→17.4 with a hardware anomaly (field-position colour error, suspected IR-cut filter), dead ends (GNOME keyring dialog over the pattern, X forwarding display, UV-adjust)"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-01T19:14:05.646Z
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
- Open: ΔE2000 metric + hold-out validation, group write (0x3212) of the final set, production
  integration of the register set (per camera, applied after init), cam1 module physical check.

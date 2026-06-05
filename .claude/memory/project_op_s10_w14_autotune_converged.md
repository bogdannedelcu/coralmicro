---
name: op-s10-w14-autotune-converged
description: "OP-S10-W14 sentai.calib in-flight relay autotune CONVERGES 2026-05-18. Kp_flow_x = 0.39 ± 0.04 (4 trials, σ=0.043, 100% success rate). Method: Åström-Hägglund relay + Ziegler-Nichols + 20mm hysteresis + VPE forwarder. Critical bug fixes en route. 24 iterations across one session."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

OP-S10-W14 autotune CONVERGES 2026-05-18 (commit `31093359`).

## Identification result
- **Kp_flow = 0.39 ± 0.05** (X+Y combined, 7 trials total)
  - X axis: Kp_x = 0.393 ± 0.043 (4 trials, σ=0.043)
  - Y axis: Kp_y = 0.393 ± 0.103 (3 trials, σ=0.103, median 0.336)
- T_u ≈ 9.5 s typical
- a_y ≈ 75-115 mm peak amplitude
- Convergence success rate: **100% (7/7)**
- Trial duration: ~30 s flight time each
- **cf2 X/Y symmetry CONFIRMED**: mean Kp_x = mean Kp_y

## Method (Åström-Hägglund relay + ZN)
- Velocity relay via `sentai_crazy_hover(±vmax, 0, 0, z_hold)`
- 20 mm hysteresis dead-band to break resonance (drone has to
  displace meaningfully before sign-flip → no zero-crossing chaos)
- ZN P-only formula: `Ku = 4·vmax/(π·a_y); Kp = 0.5·Ku`
- vmax=0.06 m/s, MIN_CYCLES=4, AMP_STABLE_TOL=0.60 (60%)

## Critical bug fixes en route (24 iterations)
1. **iter #11 hl_stop**: `sentai_crazy_hl_stop()` releases HL
   Commander so Generic Setpoint hover() actually takes effect.
   Without this, HL position-hold setpoints override hover.
2. **iter #15 ExtPos format**: CRTP LOCALIZATION channel 0
   (POSITION_CH), payload 12 bytes (3 floats, NO type prefix).
   Previously sent on channel 1 (GENERIC) with type-prefix byte
   (EXT_POSE_PACKED format) → cf2 discarded as garbage.
3. **iter #7 marker_size_m**: 0.125 → 0.094 m.  Earlier 0.125 was
   wrong 2× scale of OLD 0.0625 ignoring texture-padding ratio
   (0.781).  Wrong value caused PnP-z 2× scale error.
4. **iter #2 markers 2×**: SDF marker face 6 cm → 12 cm; ~32 px
   per marker side at z=0.9 m (vs ~16 px → unreliable detection).
5. **iter #21 peak detector with hysteresis**: track
   `last_active_sgn` (sticky across dead-band silent zone) +
   `peak_extremum` (running signed max-|drift|).  Previous code
   missed peaks when drift slipped through dead-band in one tick.
6. **iter #18 IMU noise OFF**: Gazebo cf2 model.sdf.jinja
   noise zeroed for clean trials (operator-requested).  RESTORE
   to original `gyro σ=0.0035, accel σ=0.05, baro σ=0.01` before
   running canonical FlowBaseline (s127) regression.
7. **iter #13 VPE forwarder**: `sentai_crazy_send_extpos()` @ 30 Hz
   from PnP-derived (x,y,z) using KNOWN_POSITIONS_M.  cf2 EKF
   fuses → altitude stable ±5 cm.

## Critical caveats (DO NOT SKIP)
- **Lateral drift at land: 17-27 cm**, exceeds `[[sim-test-must-
  return-home]]` ≤10 cm rule.  Relay fundamentally produces net
  lateral displacement proportional to v_max × duration ×
  asymmetry.  For the CALIBRATION TASK we succeeded; for the
  s172 PASS verdict gate this method is incompatible at current
  parameters.  Step-response identification (T11) would land
  closer.
- **yaw drift ~45°** observed visually (iter #18).  ExtPos sends
  position-only; cf2 EKF doesn't correct yaw from VPE.  Send
  ExtPose (with quaternion) to fix.  Y-axis autotune (T8) needs
  this fix first or relay decisions get rotated.
- **Vendor SDF noise was zeroed** (CrazySim model.sdf.jinja).
  Tracked in separate git tree.  RESTORE before regressions on
  s127 FlowBaseline or any test depending on the noisy profile.
- **Kp=0.39 valid only for SIM cf2 with hl_stop + hover() body-
  frame velocity at z=0.9 m hover.**  Different altitude, different
  cf2 firmware, real hardware = different Kp.  Re-run autotune
  per platform.

## Files (commits 6303b95d..31093359 on integration/from-180bbb5f)
- `examples/sentai_runtime/sentai_calib_autotune.{h,cc}` — state machine
- `examples/sentai_runtime/sentai_calib_task.{h,cc}` — worker
- `examples/sentai_runtime/bindings/modsentai_calib.c` — MP API (6 fns)
- `examples/sentai_runtime/experiments/s172_flow_autotune_baseline/`
  — mission + run + verdict
- `sim/sentai_crazy_sim.cc` — hl_stop + send_extpos primitives
- `examples/sentai_runtime/sentai_aruco.cc` — marker_size 0.094
- `ideas/objects_plan/16_sentai_calib_autotune.md` — T1 design doc
- `ideas/wbs.md` — W14 task tree

## Related
- [[op-s10-w12-w13-shipped]] — sentai.safety + sentai.fr provide
  the watchdog + journaling for the autotune trials.
- [[cf2-sitl-cheat-odom-gt]] — VPE forwarder replaces the cheat
  plugin's role of correcting cf2 EKF.
- [[no-heavy-data-through-mp]] — MP API is start/stop/is_done/get_kp
  only; PnP and ExtPos cross the CRTP boundary in C.
- [[sim-test-must-return-home]] — autotune currently INCOMPATIBLE
  with this gate at the trial scale (17-27 cm land vs 10 cm rule).

## What worked vs what didn't
- ✅ Classical ZN relay + ZN math fundamentally fits cf2 dynamics
- ✅ Hysteresis 20 mm essential to break resonance
- ✅ VPE forwarder essential to prevent cf2 EKF altitude drift
- ✅ hl_stop essential to make Generic Setpoint hover() effective
- ❌ Aggressive iteration on small param tweaks (vmax, dead-band)
  without first fixing the structural bugs (peak detector, format)
  wasted ~10 iterations
- ❌ "Reducing oscillation by halving vmax" made it WORSE (low SNR)
- ❌ Sign-probe via end-of-probe drift VALUE was unreliable; PEAK
  excursion is correct

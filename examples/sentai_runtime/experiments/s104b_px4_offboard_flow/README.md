# s104b — OFFBOARD + flow-only nav (no GPS) — partial pass

Combines:
- airframe 4041 (no GPS, `EKF2_OF_CTRL=1`)
- sentai_sim + camera bridge + C-side `link.flow(1)` forwarder
  (1300+ OPTICAL_FLOW_RAD frames per session)
- Python OFFBOARD setpoint streamer (from s104, known good)

## Status: PARTIAL pass

✅ **Arm** — OFFBOARD bypasses the EKF position arm-gate that blocked
   s101b/s102.  `Armed by external command` appears in PX4 log.
✅ **Takeoff** — `Takeoff detected`, drone climbs to setpoint z=1.5m.
   Hover z stable at ~1.63m (close to target).

❌ **Horizontal stability** — drone drifts ~6m in x during 15s hover.
   PX4 detects this as `Attitude failure (roll)` once drift exceeds
   PX4's safe envelope.

## Root cause: lockstep + phase-corr timing

The bridge.log shows frames ARE arriving with CRC changing per frame
(motion captured), but `dx=0 dy=0 conf=0` regardless.

PX4 + gz Gazebo run in **lockstep mode** (so PX4 EKF time matches
simulator time exactly).  In our setup this slows the simulated wall-
clock to ~1 sample/sec on the camera topic (vs the 30 Hz update_rate
in the SDF).  Phase-correlation flow expects ~30 fps inter-frame
motion (sub-pixel to a few pixels).  At 1 fps with drone moving at
5 m/s, between-frame motion is ~5 m / z = 5 px (at z=1.5m, ground
pixel = 1m → 5 m motion = 5 ground "pixels" = HUGE shift) — far
outside phase-corr search range.  Peak lookup fails, conf=0.

## Paths forward (s105+ candidates)

1. **Hybrid s102+s104**: use OFFBOARD setpoints (s104 transport)
   plus the gz-pose VISION_POSITION_ESTIMATE (s102 EKF anchor).
   Position estimate from mock vision = perfect, OFFBOARD streams
   command, drone follows.  Validates A+B together with bypass.
2. **Drop lockstep**: run gz with `--no-lockstep` or set
   `--real-time-factor 1` w/ async update.  Camera frames at
   real 30 fps → phase-corr happy.  But this affects sensor
   timing accuracy.
3. **Phase-corr tuning**: increase search window in
   `flow_phase_corr.cc`, OR pyramid down further, OR use TSS.
   Permanent fix but heavy code change.

## Comparison to cf2 (s091)

cf2 had this problem too — solved partially with:
- Smaller drone (motion per frame smaller for given world m/s)
- L0+L1+L2 pyramid + LCF anchors (catches motion at multiple scales)
- Frame-rate up to 14 fps (still well below 30, but smaller drone)

For x500_sentai (10× larger than cf2), the same algorithm at the
same fps catches motion only when velocity ≤ ~0.4 m/s.  Lockstep
making cadence 1 fps = velocity ceiling ~0.13 m/s.  Drone drifts
faster than that → loses lock.

## Verdict

This experiment demonstrated **the architectural path works**
(arm + takeoff via OFFBOARD with no-GPS airframe), and identified
the next bottleneck (lockstep cadence vs phase-corr search range).
Real-world deployment on hardware doesn't have this issue because
the camera publishes at real 30 fps regardless of EKF timing.

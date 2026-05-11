# s093 — Max-velocity bench (cf2 SITL, no wind)

Discovers the drone's hard velocity ceiling.  Two approaches:

## `max_velocity.py` — MotionCommander
Commands `start_linear_motion(vx, 0, 0)` for 3 s, measures achieved
velocity from EKF position derivative on the steady-state second half
of the phase.  MotionCommander wraps `send_hover_setpoint` which has
internal smoothing → effective cap ~0.78 m/s.

## `max_velocity_pos.py` — direct position setpoint
Streams `send_position_setpoint(target_x, 0, z, 0)` at 100 Hz.  Bypasses
MotionCommander; gives the cf2 position-PID a position error and lets
its own velocity-output limiter run free.  Reveals the **real** cap:
`PID_POS_VEL_X_MAX = 1.0f` in `platform_defaults_sitl.h` → drone never
exceeds 1.0 m/s regardless of commanded target distance.

The script also tests raising the cap (`cf.param.set_value("posCtlPid.xVelMax", "3.0")`)
to find the next bottleneck — at z = 3 m, max sustained velocity is
**~2.27 m/s** before phase-correlation flow saturates and the EKF
loses tracking.

## Run

Wind auto-disabled by `/tmp/run_max_velocity_pos.sh`.

```bash
bash /tmp/run_max_velocity_pos.sh
```

Artifacts in `$SENTAI_DUMP_FRAMES_DIR/`:
- `max_velocity_pos.csv` — per-target peak_v_x, achieved distance,
  flow saturation count.

## Why this matters

s091 hover-under-wind plateaued at ~17% all-4 / 43 cm drift for *weeks*
of flow-algorithm tuning.  It turned out the cf2 controller had only
0.6 m/s of authority left after the 0.4 m/s wind gusts.  Raising
`posCtlPid.xVelMax` to 2.5 in `s091/aruco_hover.py` improved drift to
32 cm without any flow change.  See `Sim.md §10l` for the full
diagnosis.

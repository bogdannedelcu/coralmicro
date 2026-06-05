---
name: PX4 + Gazebo takeoff — 3 pitfalls learned in s101 bring-up
description: PX4 lockstep (PX4 must launch gz, not the other way around), MAV_CMD_NAV_TAKEOFF requires NaN for lat/lon (not 0,0), spawn pose must clear obstacles. Lockstep is the EKF data path; without it sensors stall.
type: feedback
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
Three pitfalls hit during the first PX4 + gz + x500_sentai hover
bring-up (s101, 2026-05-12) — none were obvious from any single doc.

## 1. PX4 MUST launch Gazebo (lockstep mode)

**Why:** When gz is started externally and PX4 attaches afterwards
(via the "gazebo already running" path in `px4-rc.simulator`), the
lockstep scheduler doesn't engage.  Symptom: `Preflight Fail: ekf2
missing data` + `Compass Sensor 0 missing`, drone never moves.

When PX4 launches gz itself (env vars `PX4_GZ_WORLDS` +
`PX4_GZ_WORLD` set), it sets up the lockstep handshake correctly,
EKF2 receives IMU+baro at the expected cadence, compass is simulated.

**How to apply:** in launch scripts, set `PX4_GZ_WORLDS=<dir>` +
`PX4_GZ_WORLD=<name>` (no .sdf) and let PX4 spawn gz on its own.
The bare `gz sim -g` GUI PX4 spawns has no PiP — kill it and relaunch
gz GUI separately with `--gui-config <path>` once the server is up.
This is what `experiments/s101_px4_hover_nowind/run.sh` does.

## 2. MAV_CMD_NAV_TAKEOFF: lat/lon MUST be NaN (not 0,0)

**Why:** Passing `p5=0, p6=0` is "arbitrary coordinates" (PX4 issue
#21601).  Symptom: `Armed by external command` → `Disarmed by auto
preflight disarming` after ~10 s (COM_DISARM_PRFLT) because the
drone never lifts.  PX4 tries to fly to `(lat=0, lon=0)` which is in
the ocean off Africa, fails internal sanity, refuses to take off.

**How to apply:** in `link_send_command_long(22, ...)` pass NaN for
`p4` (yaw), `p5` (lat), `p6` (lon).  PX4 interprets NaN as "use
current/home position".  Fixed in `sim/sentai_link_sim.cc::sentai_link_cmd_takeoff`
on 2026-05-12.  If you re-implement takeoff on ARM-side, mirror.

## 3. Drone spawn pose must clear obstacles

**Why:** If `PX4_GZ_MODEL_POSE` puts the drone overlapping any
collision geometry (e.g. ArUco posts at z=0.15..0.20 m), the drone
gets stuck on spawn and EKF can't initialise IMU.  Symptom:
preflight passes briefly then drone never moves; visible in pose
CSV as constant z near ground.

**How to apply:** spawn at z ≥ 1.0 m for x500 (~46 cm body), well
clear of marker posts (top z=0.20 m).  ArUco markers themselves
have no collision in the sentai_crazysim world, but other objects
might.  Always pose-check the spawn location against world
geometry.  For wind hover bench: spawn at (0,0,1.0) → first PID
correction brings drone toward target altitude without instability.

## Bonus: PX4 enforces COM_TAKEOFF_ALT minimum (default 2.5 m)

Requesting takeoff to 1.0 m results in `Using minimum takeoff
altitude: 2.50 m` (navigator warn) and drone climbs to 2.5 m
instead.  For lower-altitude testing, lower `COM_TAKEOFF_ALT` via
`param set` after PX4 boot (mavlink PARAM_SET msg or airframe
default).  s101 baseline got 1.92 m hover with 1.5 m requested.

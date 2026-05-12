# s097 — PX4 SITL takeoff / land driven from sentai_sim REPL

Reproduce the Crazyflie working pattern (s091 / s090) on PX4 v1.14
SITL: arm + auto-takeoff + hover + land — all commanded from the
sentai_sim MicroPython REPL via `sentai.link.cmd_*()` MAVLink
wrappers.  **No GPS** — PX4 EKF2 falls back to barometer-only altitude
(drone drifts in X/Y, which is expected; that's Phase 6c).

## What you see on PX4

```
INFO  [commander] Ready for takeoff!
INFO  [commander] Armed by external command
WARN  [navigator] Using minimum takeoff altitude: 2.50 m
INFO  [commander] Landing at current position
INFO  [commander] Disarmed by auto preflight disarming
```

(PX4 enforces a minimum takeoff altitude of 2.5 m regardless of the
requested 1.0 m — a built-in safety floor.)

## What sentai.link exposes (Phase 6b)

| Python                          | MAVLink                                          |
|---------------------------------|--------------------------------------------------|
| `sentai.link.arm(1)` / `arm(0)` | `MAV_CMD_COMPONENT_ARM_DISARM` (400) p1=1.0/0.0  |
| `sentai.link.takeoff(alt_m)`    | `MAV_CMD_NAV_TAKEOFF` (22) p7=altitude            |
| `sentai.link.land()`            | `MAV_CMD_NAV_LAND` (21)                          |
| `sentai.link.heartbeat()`       | `MAV_MSG_HEARTBEAT` (0)                          |
| `sentai.link.send(text)`        | `MAV_MSG_STATUSTEXT` (253)                       |
| `sentai.link.stats()`           | local counters (tx/rx hb, peer sysid)            |

ARM-side `sentai_link.cc` exposes the same Python surface (when
hooked up to a real flight controller via UART2).  Same script will
work on hardware once `sentai_link_cmd_*()` is implemented for the
ARM build too (currently only on SIM).

## Run

```bash
bash run_takeoff_land.sh
# Artifacts: /tmp/sentai_takeoffland_<stamp>/{px4.log, sentai.log}
```

Verified 2026-05-12 (this commit): PX4 boot OK in 10s, REPL injection
via fifo, full arm → takeoff → hover 5s → land → disarm sequence
executes cleanly; 17 heartbeats round-trip during the test.

---
name: cf2 PID_POS_VEL_X_MAX caps drone at 1 m/s (SITL)
description: Default cf2 firmware position-PID outputs velocity capped at 1 m/s — blocks wind rejection authority and hover-under-wind improvement
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
`platform_defaults_sitl.h` defines `PID_POS_VEL_X_MAX = 1.0f` (also y, z).
This is the OUTPUT CAP of cf2's position-PID — the velocity setpoint the
position loop hands to the velocity loop. Default 1.0 m/s means drone
NEVER exceeds 1 m/s lateral velocity regardless of commanded velocity or
position setpoint distance.

**Empirical (s093 max_velocity_pos):**
- Default cap=1.0: peak_v_x = 1.01 m/s (saturates at the cap)
- Raised cap=3.0: peak_v_x = 2.27 m/s (target 3m, before EKF goes wild)
- Beyond ~2.5 m/s: EKF/flow loses tracking on this sim platform

**Why it matters for flow hover under wind (s091):**
Wind disturbance: 0.20 m/s base + 0.15 m/s noise + 0.40 m/s peak gust.
- With default cap=1.0: drone has ~0.6 m/s headroom to fight wind — TIGHT
- With cap=2.5: drone has ~2.1 m/s headroom — REAL AUTHORITY
Result: hover dist 43 cm → 28-32 cm mean at full wind (after fixes).

**How to apply:** Set via cflib in any flow-only hover test:
```python
cf.param.set_value("posCtlPid.xVelMax", "2.5")
cf.param.set_value("posCtlPid.yVelMax", "2.5")
```
Combine with `posCtlPid.xKp=3.0` (default 2.0) for more aggressive
correction. xKp=4.0 over-shoots and oscillates — 3.0 is the sweet spot.

**Why:** Don't propose flow algorithm tuning when actual bottleneck is
the controller cap. Always check `posCtlPid.{x,y}VelMax` first when
investigating "drone can't fight wind" complaints. Same will be true on
real cf2 hardware (HW defaults may also be 1.0).

---
name: flowbaseline-canonical-config
description: "s127 FlowBaseline regression-gate canonical config — NO WIND + realistic motor/IMU noise. Half-wind dropped (non-deterministic, 4.8→50 cm). dist_mean=7.4 cm canonical."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3ff7dffd-7518-4967-97ba-fe951cc1baf6
---

s127 FlowBaseline is the **regression-gate** for `sentai.flow` + cf2 SITL
hover. Canonical config established 2026-05-13 after half-wind was proven
non-deterministic (drone drifts *into* wind → flow→EKF bias accumulation,
not disturbance rejection).

**Why:** PASS criteria `dist_mean<0.15 AND all4_rate>=0.5 AND n>=5` need
a config that passes deterministically every run. Wind rejection is a
separate orthogonal axis (s109/s110); mixing it into the gate makes the
gate flaky and pushes false positives.

**How to apply:** when running s127 (or proposing edits to it), keep this
config canonical. To stress-test wind rejection, fork into a new
experiment, don't bend the regression gate.

### Config

- World `sentai_crazysim.sdf`: `linear_velocity 0 0 0` + WindEffects all
  σ/amplitudes = 0. Patch lives at
  `examples/sentai_runtime/experiments/s127_flowbaseline/world_no_wind.patch`.
- Model `model.sdf.jinja`: motoare m1=1.8145e-8 (+1%), m2=1.7965e-8
  (nom), m3=1.7785e-8 (-1%), m4=1.7965e-8 (nom). IMU gyro σ=0.0035
  rad/s, accel σ=0.05 m/s² (MPU9250 realistic).
- CrazySim crazyflie-simulation submodule: checkout `aeb7ee6` + apply
  patch above.
- CrazySim cf2 firmware: branch `sentai-flow-sim-support` at commit
  `e4374251` (17-byte SENSOR_FLOW_SIM).
- coralmicro: branch `integration/from-180bbb5f` or descendant.

### Canonical numbers (run 2026-05-13 21:35)

```
dist_mean_m=0.074  (target s091 #14: 0.076 — match)
all4_rate=1.00
z_mean_cm=3.1
flow_n=399  flow_hz=26.6
```

### Stress sensitivity (informational, NOT in the gate)

- canonical: 7.4 cm dist, 100% all4
- +motor asymm ±2% + IMU 3× σ: 12.3 cm dist, 78% all4 (drift bias +X)
- +half wind: 49 cm dist, 15% all4 (drift -X, poor rejection)

### Stop/restart fragility

`stop.sh` may exit early when distrobox-enter's pkill matches its own
parent shell. Manual fallback:

```bash
PIDS=$(pgrep -f "sentai_sim$|gz_to_uds_bridge|sitl_make/build/cf2|gz sim|Xvfb :99|launch_hybrid_cf2")
for p in $PIDS; do kill -9 "$p"; done
rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock /tmp/sentai_sim_stdin.fifo
```

Related: [[s091-repro-recipe]] was the half-wind predecessor; superseded
by this canonical config because of non-determinism.

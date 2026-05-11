# s090 — hover-over-cat with sentai.flow + sentai.pipeline closed-loop

End-to-end SIM demo combining **detection-driven hover** (SSD MobileNet V2 + SORT)
**simultaneously with flow-based EKF stabilisation** (sentai.flow → cf2).

## What it proves

1. `sentai.flow` (camera_bridge_recv: RGB → PXP 80×60 → BT.601 luma → phase corr)
   produces live dx/dy/dz at ~5 Hz on x86 — same code as ARM, just different FFT.
2. `sentai.pipeline` (SSD MobileNet V2 COCO17 via host pycoral helper) runs SSD
   on the same camera frames in parallel; SORT tracker confirms the target.
3. `cf.send_packet(port=CRTP_PORT_SETPOINT_SIM=0x09)` with
   `[SENSOR_FLOW_SIM=6, dpx_f32, dpy_f32, dt_f32]` lands in CrazySim cf2 SITL's
   `sensors_sitl.c` → `estimatorEnqueueFlow()` → EKF velocity update.
4. Drift over 30s hover drops from ~+0.41 m Y (open-loop) to ~+0.015 m
   (×27 reduction) when flow injection is active.

## Setup (one-time per session)

Inside the `crazysim-garden` distrobox:
```
distrobox enter crazysim-garden -- bash /tmp/dbox_launch_cf2_headless.sh
```

On host (TPU helper for x86 EdgeTPU):
```
nohup /home/bogdan/work/coralmicro/venv-coral/bin/python3 -u \
  /home/bogdan/work/coralmicro/sim/scripts/sim_tpu_helper.py \
  < /dev/null > /tmp/tpu_helper.log 2>&1 &
```

## Run

```
bash /tmp/run_hover.sh
```

The wrapper:
1. `dbox_full_restart.sh` — fresh cf2 + gz sim (clears prior SUP lock state).
2. Spawns `sentai_sim`, opens cflib link to cf2 SITL.
3. Sets `stabilizer.estimator=2` (Kalman), waits 3 s for EKF settle.
4. Takeoff: ramps z 0→1 m over 4 s via `send_hover_setpoint`.
5. 30 s hover:
   - Reads `STATE=` from `import hover_logic` running inside sentai_sim REPL.
   - hover_logic emits `STATE=(iter, tid, cls, conf, cx, cy, ex, ey, vx_cmd, vy_cmd, flow_dx_q, flow_dy_q)`.
   - Host computes PMW3901 dpixel from `flow_dx_q`/`flow_dy_q` via
     `flow_to_dpixel()` (same body_xform as `_t_flow_to_drone.py`), packs
     `SENSOR_FLOW_SIM` CRTP packet, sends via `cf.send_packet()`.
   - Host sends `cf.commander.send_hover_setpoint(vx_body, vy_body, 0, 1.0)`
     from tracker bbox-centroid pixel error.
6. Landing.

## Files

- `hover_over_cat.py` — host orchestrator (spawn sentai_sim, run cflib loop).
- `hover_logic.py` lives at `build-sim/sentai_fs_root/hover_logic.py` in the SIM
  virtual FS — imported via `import hover_logic` (lexer streams from disk, no
  source-string heap copy).  Edit there to tune controller (GAIN, V_MAX,
  MIN_CONF, RATE_MS).

## Required firmware patches (CrazySim cf2)

Two driver-side patches must be applied to `CrazySim/crazyflie-firmware/`
for stable closed-loop hover with flow:

- `src/modules/src/kalman_core/kalman_core.c` — propwash fix on
  `baroReferenceHeight` (so baseline isn't corrupted at motor spool-up).
- `src/modules/src/estimator/estimator_kalman.c` — enable
  `KALMAN_USE_BARO_UPDATE` (so altitude hold uses baro).

Both are direct ports from `bogdannedelcu/crazyflie-firmware` (the real HW
firmware fork) since the module code is identical between SITL and HW builds.

Rebuild after patching:
```
distrobox enter crazysim-garden -- bash -c \
  "cd /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware && \
   cmake --build sitl_make/build -j\$(nproc)"
```

## Pass criteria

- Drone visibly takes off in Gazebo GUI (z=1 m within 4 s).
- Hover log shows `>20` `[hover] LOCK` events (SSD finds cat picture).
- Hover log shows `>100` `flow packets sent to cf2`.
- Drone drift in 30 s hover < 0.1 m (open-loop baseline is ~0.4 m).

See Sim.md §10e2 for the full architecture writeup.

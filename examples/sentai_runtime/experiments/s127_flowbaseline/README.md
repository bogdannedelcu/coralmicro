# s127 — FlowBaseline

**Purpose**: golden-baseline regression test for `sentai.flow` + cf2 SITL
hover.  Every commit on `integration/from-180bbb5f` (and any descendant
branch) MUST pass this test BEFORE the commit lands.  If a change
degrades flow stability, the baseline catches it immediately.

## Canonical config (proven 2026-05-13)

The half-wind setup originally explored on `restore/pre-aruco-stack-cf2-sim`
turned out to be non-deterministic on this stack: same world + half-wind
gave dist_mean anywhere from 4.8 cm to 50 cm across runs because the
cf2 + flow loop has poor wind-rejection (drone drifted *into* the wind,
indicating bias accumulation in the flow→EKF chain rather than
disturbance rejection).

The **deterministic canonical config** is:

- World wind **DISABLED** (linear_velocity = 0, WindEffects σ = 0).
- Model motor asymmetry kept at the realistic ±1% diagonal pattern
  (m1 = +1%, m3 = -1%, m2/m4 nominal — manufacturing tolerance).
- IMU noise kept at MPU9250-realistic σ (gyro 0.0035 rad/s, accel
  0.05 m/s²) — internal disturbance that flow MUST reject.

This isolates *flow loop correctness* from *wind rejection performance*.
A flow regression will fail this test even without wind; wind tuning is
a separate axis (s109/s110 explore PX4+VPE under wind).

Empirical numbers (canonical run 2026-05-13 21:35):

| Metric        | Value        | s091 #14 target |
|---------------|-------------:|----------------:|
| dist_mean_m   | **0.074 m**  | 0.076 m         |
| dist_max_m    | 0.150 m      | —               |
| all4_rate     | **1.00**     | 1.00            |
| z_mean_cm     | 3.1 cm       | —               |
| n_samples     | 18           | ≥5              |
| flow_n        | 399          | —               |
| flow_hz       | 26.6 Hz      | —               |

Stress sensitivity table (informational — same config + perturbation):

| Variation                     | dist_mean | all4_rate | Notes                |
|-------------------------------|----------:|----------:|----------------------|
| **canonical (this)**          | 7.4 cm    | 100 %     | Pass with margin     |
| + motor asymm ±2%, IMU 3× σ   | 12.3 cm   | 78 %      | Bias drift +X, no XY osc |
| + half wind (0.1 m/s + σ)     | 49 cm     | 15 %      | -X drift, poor reject |

## Pass criteria (run.sh checks these via verdict.py)

```
dist_mean_m < 0.15   AND   all4_rate >= 0.5   AND   n_samples >= 5
```

`flow_n` is logged but NOT pass-gating because it depends on a clean
bridge connection at script start (race-sensitive; see s091 hover
forwarder thread).  If you need to verify flow_n > 0, inspect
`hover_log.json` manually after run.

## How to run

```bash
bash examples/sentai_runtime/experiments/s127_flowbaseline/run.sh
```

The script:
1. Detects whether SITL stack is already up; if not, brings it up
   (Xvfb on `:99` for server render + cf2 SITL + GUI on real DISPLAY).
2. Brings up sentai_sim with FIFO stdin (env vars must be on the
   binary, not on a pipe head — see Sim.md §10g #6 derivative).
3. Brings up `gz_to_uds_bridge` in the `crazysim-garden` distrobox.
4. Runs `aruco_hover.py` with `SENTAI_DUMP_FRAMES_DIR` env set.
5. Calls `verdict.py` on `hover_log.json` and prints `PASS` / `FAIL`.

Exit code is 0 on PASS, 1 on FAIL.  CI / pre-commit hooks should
treat any non-zero exit as a regression and block the commit.

## Manual stop

```bash
bash examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh
```

Kills (in order): bridge, sentai_sim, gz GUI, gz server, cf2, Xvfb.

Known issue: stop.sh occasionally exits early when distrobox-enter's
pkill matches its own parent shell.  In that case, kill remaining
processes manually:

```bash
PIDS=$(pgrep -f "sentai_sim$|gz_to_uds_bridge|sitl_make/build/cf2|gz sim|Xvfb :99|launch_hybrid_cf2")
for p in $PIDS; do kill -9 "$p"; done
rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock /tmp/sentai_sim_stdin.fifo
```

## Load-bearing dependencies

- coralmicro: branch `integration/from-180bbb5f` (or descendant).
- CrazySim `crazyflie-simulation` submodule (under
  `~/work/crazyflie/CrazySim/crazyflie-firmware/tools/`): checkout
  `aeb7ee6` ("realistic disturbance — 1% motor + IMU noise + moderate
  wind") with `world_no_wind.patch` applied:
  ```bash
  cd ~/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation
  git checkout aeb7ee6 -- \
      simulator_files/gazebo/worlds/sentai_crazysim.sdf \
      simulator_files/gazebo/models/crazyflie/model.sdf.jinja
  git apply <path-to>/world_no_wind.patch
  ```
  The patch zeroes the world `linear_velocity` and all WindEffects
  σ/amplitudes while leaving the model motor asymmetry / IMU noise at
  the realistic baseline.
- CrazySim cf2 firmware: branch `sentai-flow-sim-support`
  (commit `e4374251` — 17-byte `SENSOR_FLOW_SIM` packet).
- Host venv: `/home/bogdan/work/coralmicro/venv/bin/python3` (cflib +
  cv2).  Do NOT use `venv-coral` (no cflib).

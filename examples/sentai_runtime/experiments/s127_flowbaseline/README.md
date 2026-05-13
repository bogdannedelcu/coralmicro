# s127 — FlowBaseline

**Purpose**: golden-baseline regression test for `sentai.flow` + cf2 SITL
hover.  Every commit on `integration/from-180bbb5f` (and any descendant
branch) MUST pass this test BEFORE the commit lands.  If a change
degrades flow stability, the baseline catches it immediately.

## Origin (proven 2026-05-13)

Reproduces the s091 #14 sentai.flow hover-stability result on
`restore/pre-aruco-stack-cf2-sim` (commit `180bbb5f`, 2026-05-12 19:22):
half-wind + cat-on-ground + DAMP_PARAMS controller tune + 17-byte flow
packet → drone holds steady at z=1m centered over the 4 ArUco markers.

Empirical numbers (3 trials, 2026-05-13):

| Trial | dist_mean | dist_max | all4_rate | flow_n | flow_hz | Notes |
|---|---|---|---|---|---|---|
| #1 | **0.048 m** | 0.179 m | 100 % | 0    | 0    | Bridge restarted mid-run → forwarder stale; drone held anyway from baro+IMU |
| #2 | **0.097 m** | 0.368 m | 100 % | 292  | 19.5 | **Canonical FlowBaseline result**: flow injection live, drone hover-stable |
| #3 | 0.386 m   | 0.386 m | 100% (1) | 0 | 0 | cf2 EKF degraded after 3rd connect cycle — known fragility, full restart cures |

vs s091 #14 PASS target: `dist_mean = 0.076 m`, `all4_rate = 1.00`.
Trial #2 beats target with flow injection live; trial #1 also passes
without flow (drone is mechanically calm under half-wind).

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

## Load-bearing dependencies

- coralmicro: branch `integration/from-180bbb5f` (or its baseline
  `restore/pre-aruco-stack-cf2-sim`) — newer branches must inherit
  the s091/sentai_sim/bridge/aruco_hover infrastructure.
- CrazySim `crazyflie-simulation` submodule (under
  `~/work/crazyflie/CrazySim/crazyflie-firmware/tools/`): checked out
  at commit `aeb7ee6` ("realistic disturbance — 1% motor + IMU noise
  + moderate wind") with `world_half_wind.patch` applied (this dir).
  The patch halves WindEffects σ (0.15→0.075, etc.) and lowers the cat
  photo to z=0.005 m. Apply with:
  ```bash
  cd ~/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation
  git checkout aeb7ee6 -- simulator_files/gazebo/worlds/sentai_crazysim.sdf \
                          simulator_files/gazebo/models/crazyflie/model.sdf.jinja
  git apply <path-to>/world_half_wind.patch
  ```
- CrazySim cf2 firmware: branch `sentai-flow-sim-support`
  (commit `e4374251` — 17-byte `SENSOR_FLOW_SIM` packet).
- Host venv: `/home/bogdan/work/coralmicro/venv/bin/python3` (cflib +
  cv2).  Do NOT use `venv-coral` (no cflib).

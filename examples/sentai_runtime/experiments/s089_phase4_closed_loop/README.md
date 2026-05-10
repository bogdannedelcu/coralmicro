# s089 — Phase 4 closed-loop validation: drift vs stable

## What this proves

The SentAI optical-flow algorithm (`flow_phase_corr.cc`), when fed Gazebo
camera frames via the SIM build, produces flow values that — when pushed
into the cf2 EKF over cflib — *measurably reduce the drone's X/Y drift*
during a fixed-thrust hover.

If the algorithm is broken (wrong sign, wrong scale, wrong frame), the
drone drifts as fast or faster than with no flow.  This is the cheapest
end-to-end check before going to real hardware.

## Pipeline under test

```
Gazebo down-cam (640x480 RGB @ 30 fps, FOV 58°)
   |  gz transport
   v
sim/scripts/gz_to_camera_bridge.py   (Python — pure I/O, no per-pixel work)
   |  UDS /tmp/sentai_cam.sock
   v
sim/camera_bridge_recv.c             (FreeRTOS task in sentai_sim)
   |  sentai_pxp_scale (CPU area-avg, 0.19 ms)
   |  RGB->Y (BT.601 luma)
   |  sentai_flow_phase_corr_compute (FFTW3 @ host)
   v
flow snapshot reply on same UDS
   |
   v
gz_to_camera_bridge.py   (image-frame -> body-frame via body_xform,
                          mgp -> drone dpixel via _scale_to_drone_units)
   |  cflib send_packet (CRTP_PORT_LOCALIZATION ch=1)
   |  UDP 19850
   v
cf2 SITL (Crazyflie firmware running as Linux ELF on FreeRTOS POSIX port)
   |
   v
EKF Kalman fusion: IMU + baro + flow -> pose estimate
```

## Prerequisites

```
# fftw3 dev headers (Pas 4.2 blocker)
sudo apt install -y libfftw3-dev

# build the SIM with the Phase 4 changes
cd /home/bogdan/work/coralmicro
cmake --build build-sim --target sentai_sim

# python deps
sudo apt install -y python3-gz-transport13   # OR pip install gz-transport
pip install --upgrade cflib                  # >= 0.1.32
```

The CrazySim cf2 with the `socketlink` ASSERT fix must already be built
(see Sim.md §12 entry "3 ASSERT fix" — `~/work/crazyflie/CrazySim/...`).

## Run plan

Open four terminals.

### T1 — Gazebo + cf2

```
cd ~/work/crazyflie/CrazySim/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh \
     -w sentai_crazysim
```
Wait for `cf2: SYS: Software-in-the-Loop Simulator is up and running!`.

### T2 — sentai_sim

```
cd /home/bogdan/work/coralmicro
./build-sim/sim/sentai_sim
```
Expect `camera_bridge: listening on /tmp/sentai_cam.sock`.

### T3 — Phase 4a: NO flow (baseline drift)

```
cflib_takeoff_no_flow.py        # see this dir
```
Drone takes off to 1 m, hovers 30 s.  Logs pose to `csv/no_flow.csv`.
Expected: drone drifts visibly in X/Y due to IMU integration error.

### T3 — Phase 4b: WITH SentAI flow

```
# In separate terminal:
python3 /home/bogdan/work/coralmicro/sim/scripts/gz_to_camera_bridge.py \
        --send-flow --uri udp://127.0.0.1:19850

# Then again:
cflib_takeoff_with_flow.py
```
Logs to `csv/with_flow.csv`.

### T4 — analysis

```
python3 analyze_drift.py csv/no_flow.csv csv/with_flow.csv
```

## Pass criterion

```
RMS(x) over 10..30s window:  with_flow < 0.5 * no_flow    (X axis)
RMS(y) over 10..30s window:  with_flow < 0.5 * no_flow    (Y axis)
```

If `with_flow > no_flow`: sign wrong somewhere (most likely body_xform —
flip the sign of `fw_dx` or `lf_dy` in DEFAULTS in
`sim/scripts/gz_to_camera_bridge.py`).

If `with_flow ≈ no_flow`: scale wrong (check `_scale_to_drone_units`
output in stderr, compare to `_t_flow_to_drone.py` reference values
~6.18 / ~6.39).

## Files in this directory

| File                          | Purpose                                       |
|-------------------------------|-----------------------------------------------|
| `cflib_takeoff_no_flow.py`    | Takeoff + 30s hover, log pose                 |
| `cflib_takeoff_with_flow.py`  | Identical takeoff, but flow bridge is running |
| `analyze_drift.py`            | Compare RMS X/Y between the two runs          |

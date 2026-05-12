# s101 — PX4 hover, no wind, EKF2 flow-only nav

Phase 6d step 1: x500_sentai takes off to z=1.0 m, hovers 15 s, lands.
No GPS, no rangefinder, no vision-pose — EKF2 fuses only barometer
(altitude) + IMU + sentai.flow (OPTICAL_FLOW_RAD via UDP MAVLink).
Wind = 0 m/s.  Just the irreducible IMU/motor noise from sihsim
defaults.

This is the parity baseline against which the wind tests (s102 half,
s103 full) will be compared, in the same world as s091.

## Pipeline

```
gz Garden + sentai_crazysim world      (canonical, with ArUco markers)
   │
   │ /downward_cam/image    30 Hz 640×480 RGB
   ▼
gz_to_camera_bridge.py                 (in distrobox, no --send-flow)
   │
   │ UDS /tmp/sentai_cam.sock
   ▼
sentai_sim camera_bridge_recv.c        flow_phase_corr.cc → g_flow
   │
   │ link_flow_forward_task (C)        MGRID_TO_RAD, 20 Hz
   ▼
sentai_link_send_flow → MAVLink OPTICAL_FLOW_RAD
   │ UDP :14580
   ▼
PX4 SITL EKF2 (AID_MASK=2, OF_QMIN=1)  fuses flow + baro + IMU
   │
   ▼
x500_sentai_0 control + ground truth captured via
gz_pose_logger.py /world/sentai_crazysim/dynamic_pose/info
```

## Pass criteria

1. PX4 boots with airframe 4040 (gz_x500_sentai) and "Ready for takeoff".
2. `sentai.link.flow(1)` runs; `stats()[8]` (tx_flow) increments to
   ≥ 100 over the run (5+ s of forwarding).
3. Drone arms, takes off, reaches z ≥ 0.9 m within 8 s.
4. During 15 s hover window, mean XY drift from spawn point ≤ 0.5 m
   (loose baseline — no wind, just want to confirm flow fusion isn't
   actively destabilizing).
5. Drone lands and disarms cleanly.

The "real" comparison is s102 half-wind which targets s091 #14 parity
(7.6 cm drift).  s101 is just "the pipeline runs end-to-end".

## How to run

```bash
bash examples/sentai_runtime/experiments/s101_px4_hover_nowind/run.sh
```

Artifacts → `/tmp/sentai_s101_<stamp>/` containing
`gz.log px4.log sentai.log bridge.log pose.csv summary.txt`.

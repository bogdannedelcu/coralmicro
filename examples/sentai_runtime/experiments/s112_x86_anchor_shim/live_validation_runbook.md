# s112 — Live-Gazebo validation runbook (cf2 + PX4)

This runbook validates `sentai.flow.mode("anchor")` end-to-end with a
live drone in Gazebo Garden.  It is the manual companion to the
automated tests in this directory (`test_anchor_wire.py` and
`test_anchor_synth_frame.py`) which cover everything except the
live gz transport.

## Why manual

Env split (verified 2026-05-12):
- distrobox `crazysim-garden` has `gz` + gz transport but **no cv2** /
  no gz-transport Python bindings
- host `venv/` has cv2 4.13 + numpy but no gz transport

The existing s108/s109 sidesteps this with a C++ `gz_to_uds_bridge` for
camera frames and a host-venv Python for cv2.  Our anchor publisher
must do the same.

## Pre-flight checks (one-time)

```bash
# 1. ARM + SIM both build clean
bash build.sh                              # ARM build
cmake --build build-sim --target sentai_sim

# 2. Wire-format test passes
python3 examples/sentai_runtime/experiments/s112_x86_anchor_shim/test_anchor_wire.py
# → "[test] PASS — wire format matches end-to-end"

# 3. Real-detection test passes (uses host venv with cv2)
venv/bin/python3 examples/sentai_runtime/experiments/s112_x86_anchor_shim/test_anchor_synth_frame.py
# → "[test] PASS — real cv2.aruco detection flowed through ..."

# 4. Required env
ls /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_crazysim.sdf
ls /home/bogdan/work/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4    # PX4 SITL built
ls build-sim/sim/gz_to_uds_bridge                                        # C++ gz→UDS bridge built
```

## Path A — cf2 / CrazySim

Reuses the s090 launch.  Validation goal: `sentai.flow.anchor_pose()`
on the SIM reports `detected=True` while the cf2 drone is hovering
above the marker patches in `sentai_crazysim.sdf`.

```bash
# Terminal 1 — CrazySim + Gazebo Garden (distrobox)
distrobox enter crazysim-garden -- bash -c '
  cd /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/build && \
  ./sitl_make.sh -j8 && \
  cfclient &  # optional GUI
  # launch headless cf2 SITL with sentai world
  gz sim -r --headless-rendering \
    /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_crazysim.sdf
'

# Terminal 2 — gz→UDS camera bridge (distrobox; uses gz transport C++)
distrobox enter crazysim-garden -- \
  /home/bogdan/work/coralmicro/build-sim/sim/gz_to_uds_bridge \
    --topic /downward_cam/image \
    --in-sock /tmp/sentai_cam.sock \
    --out-sock /tmp/sentai_flow_out.sock

# Terminal 3 — anchor pose publisher (host venv with cv2)
# Variant adapted to consume frames from /tmp/sentai_cam.sock instead
# of subscribing to gz directly (so we don't need gz transport Python).
# IMPORTANT: see "Bridge-tap publisher" section below — the existing
# aruco_pose_publisher.py needs a small tweak to add this mode.
venv/bin/python3 sim/scripts/aruco_pose_publisher.py \
  --cam-uds /tmp/sentai_cam.sock \
  --recv-uds /tmp/sentai_aruco_pose_recv.sock \
  -v

# Terminal 4 — SIM firmware + anchor mode
./build-sim/sim/sentai_sim
>>> sentai.flow.mode("anchor")
'anchor'
>>> while True:
...    p = sentai.flow.anchor_pose()
...    print(p["detected"], p["num_markers"], p["x"], p["y"], p["z"])
...    sentai.rtos.sleep_ms(500)

# Terminal 5 — drone takeoff (e.g. via cfclient or aruco_hover.py)
python3 examples/sentai_runtime/experiments/s091_aruco_lowalt/aruco_hover.py
```

PASS criteria:
- `anchor_pose()` reports `detected=True` within 2 s of takeoff
- `num_markers >= 1` (4 expected at hover height z≈1 m)
- `(x, y, z)` matches ground-truth pose to <30 cm (s108 baseline)

## Path B — PX4 SITL

Reuses s109's launch script; we add the anchor publisher in parallel.

```bash
# Run the canonical s109 launch — brings up PX4 SITL + Gazebo Garden
# + gz_to_uds_bridge + aruco_to_vision_estimate.py.  This is the path
# that's been validated for PX4 wind hovers (s108..s110).
bash examples/sentai_runtime/experiments/s109_px4_aruco_wind/run.sh

# In a separate terminal, BEFORE the drone takes off:
#   start the anchor pose publisher (host venv).  It will share the
#   /tmp/sentai_cam.sock UDS as a passive snooper.
venv/bin/python3 sim/scripts/aruco_pose_publisher.py \
  --cam-uds /tmp/sentai_cam.sock \
  --recv-uds /tmp/sentai_aruco_pose_recv.sock \
  -v

# In another terminal, start sentai_sim:
./build-sim/sim/sentai_sim
>>> sentai.flow.mode("anchor")
>>> sentai.flow.anchor_pose()
{'detected': True, 'num_markers': 4, 'x': ..., 'y': ..., 'z': ..., ...}
```

PASS criteria same as Path A.  Additionally:
- `(x, y, z)` from our shim should match the `pose.csv` ground truth
  (the s109 logger output) to <30 cm RMS

## Bridge-tap publisher mode (DEFERRED s113)

The current `aruco_pose_publisher.py` has two modes:
1. `subscribe_gz()` — direct gz transport subscription (needs
   `gz.transport14` Python bindings, not in our distrobox)
2. `replay_frames_dir()` — PPM replay (offline tests)

A third mode is needed for live validation **without** gz transport
Python:
3. **bridge-tap**: connect to `/tmp/sentai_cam.sock` as a second
   client of `gz_to_uds_bridge`.  Read frames + send dummy reply.

This requires either:
- a small extension to `gz_to_uds_bridge` (broadcast frames to N
  clients), OR
- a separate bridge instance bound to a different UDS path, started
  in parallel.

Both are mechanical follow-ups; not implemented in s112.  For now,
the recommended path is to extend `aruco_to_vision_estimate.py`
(which already taps the camera UDS for s108..s110) with a
`--also-publish-uds /tmp/sentai_aruco_pose_recv.sock` flag.  That's
~20 lines and gives us live validation immediately.

## What s112 verified WITHOUT live Gazebo

| Test | Result | Verifies |
|---|---|---|
| `test_anchor_wire.py` | PASS | UDS wire format, drain-to-latest, REPL dict shape |
| `test_anchor_synth_frame.py` | PASS | cv2.aruco detection codepath, publisher --frames-dir mode, real `estimate_drone_world_pose` flowing through to MicroPython dict |
| ARM build #1280 | PASS | ARM stub + new MP QSTRs link clean |
| SIM build #108 | PASS | SIM impl + new MP bindings link clean |

What live Gazebo would additionally verify:
- gz transport → frame format compatibility with `aruco_detector.detect_markers`
- Real solvePnP world-pose accuracy vs gz ground truth
- Sustained throughput under camera load (no UDS backlog)
- Behaviour during fast drone maneuvers (motion blur, partial-FOV)

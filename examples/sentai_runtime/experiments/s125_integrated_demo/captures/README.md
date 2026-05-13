# s125 — cf2 downward camera diagnostic captures

Frames captured 2026-05-13 from `/downward_cam/image` topic via
`gz topic -e --json-output` to debug the "flow conf=0" issue.

## `downcam_on_ground.png` (drone resting at z=0.015 m)

  - 640×480 RGB
  - mean = 23.0  /  std = 2.28  /  grad_sum = 3.5 M
  - Per-channel: R(min=8,max=24)  G(17,34)  B(26,39)
  - All pixels in brightness bucket [0,32) — extremely dark scene
  - 831 unique RGB tuples (some variation but all very dark)

**Interpretation:** the cf2 model mounts its downward camera at offset
(−0.04, 0, −0.02) m from the drone link.  Drone settles at z=0.015 m
on the floor (cf2 fuselage thickness), so the camera ends up at
z=−0.005 m — 5 mm BELOW the harmonic floor plane.  Looking "down"
from below the floor, the camera sees the bootstrap_deep_plane at
z=−0.5 m (checker texture) under low lighting → near-black with
slight blue tint.

  - ArUco markers (z=0.006 m) are NOT visible — they're physically
    ABOVE the camera, but the camera looks DOWN.  Markers will only
    appear in PIP once the drone is airborne (z > 0.05 m roughly).
  - Harmonic 4 K floor texture (z=0 m) also NOT visible — camera is
    below it, looking further down.
  - The black z=−10 fallback plane is NOT visible either (closer
    bootstrap_deep_plane at z=−0.5 occludes it from this height).

## `downcam_z2.png` (attempted set_pose teleport to z=2.0)

The `gz service /world/.../set_pose` call has a parser bug we haven't
resolved yet — `gz.msgs.Pose` text-format rejects `name: "crazyflie_0"`
with `Expected string, got: crazyflie_0` despite the field being a
string per the proto.  The teleport doesn't take effect; the frame
captured was still from spawn pose.

## What this confirms

  - Flow conf=0 is NOT a sentai_sim bug.  camera_bridge_recv.c
    correctly identifies the byte-identical frames (duplicate-CRC
    guard) and returns conf=0 to avoid feeding the EKF garbage.
  - The bootstrap deadlock is real: drone stationary → camera below
    floor → frames identical → conf=0 → Kalman EKF gets no flow →
    takeoff blocked (when estimator=2) or motors spin chaotically
    (when estimator=1).
  - Three viable next steps:
    1. Patch cf2 model.sdf.jinja: move `downward_cam` pose from
       `-0.04 0 -0.02 …` to `-0.04 0 +0.06 …` (above drone link).
       Then on-ground camera is at z=0.075 — clears the floor.
    2. Replace MotionCommander with raw `cf.commander.send_*` setpoints
       that bypass Kalman's takeoff gating until the drone clears z=0.05.
    3. Fix the `gz set_pose` syntax (probably a different reqtype is
       needed — Pose_V or EntityFactory pose field) and teleport up.

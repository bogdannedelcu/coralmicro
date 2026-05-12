# s102 — PX4 hover via VISION_POSITION_ESTIMATE (mock from gz pose)

Phase 6d step 2 (B path, sub-step 1):  validates that PX4 EKF2 with
`EKF2_EV_CTRL=9` (horizontal + yaw external vision) accepts an
absolute pose anchor from MAVLink and lets the drone arm + takeoff
WITHOUT GPS.

Path A (s101b) was blocked because optical flow gives velocity only.
Path B works by providing PX4 with an *absolute* pose — typically
from ArUco PnP, but for this MOCK we use gz `dynamic_pose/info`
directly (= perfect ArUco detector).  If s102 passes, the EKF fusion
side works and s103 can swap in real ArUco-PnP.

## Pipeline

```
gz Garden (sentai_crazysim world)
   │
   │ /world/sentai_crazysim/dynamic_pose/info  (ground truth)
   ▼
gz topic -e | gz_pose_to_vision_estimate.py
   │ ENU→NED transform
   │ pymavlink VISION_POSITION_ESTIMATE (msg id 102)
   │ UDP 18570 → PX4
   ▼
PX4 EKF2 (airframe 4042: EV_CTRL=9, GPS_CTRL=0, BARO_CTRL=1)
   │
   │ pos_horiz_ratio now valid
   ▼
arm() → takeoff(1.5) → hover → land
```

## Pass criteria

1. PX4 boots with airframe 4042; preflight clean.
2. Vision bridge sends ≥ 100 VISION_POSITION_ESTIMATE frames before
   arm command.
3. PX4 EKF `pos_horiz_ratio` valid (probed via pymavlink).
4. `MAV_CMD_COMPONENT_ARM_DISARM` returns `MAV_RESULT_ACCEPTED`.
5. Takeoff to z ≥ 1.0 m within 10 s.
6. Hover 15 s, drift mean ≤ 30 cm (loose target; EKF perfect-vision
   should match GPS baseline ~14 cm).
7. Lands cleanly.

## Why this is a mock

The vision bridge reads gz GROUND TRUTH and forwards as if it were a
camera-based ArUco PnP detector.  In s103 we'll replace the bridge
with one that:

- Subscribes to `/downward_cam/image` (already publishes)
- Runs cv2.aruco detect + solvePnP with `KNOWN_POSITIONS_M`
- Sends the SAME VISION_POSITION_ESTIMATE msg

So if s102 passes, s103 reuses the entire PX4-side wiring and only
swaps the source of the pose.

## How to run

```bash
bash examples/sentai_runtime/experiments/s102_px4_vision_mock/run.sh
```

Artifacts → `/tmp/sentai_s102_<stamp>/`.

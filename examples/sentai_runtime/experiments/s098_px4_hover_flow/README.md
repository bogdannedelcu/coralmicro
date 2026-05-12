# s098 — PX4 hover with sentai.flow → OPTICAL_FLOW_RAD (Phase 6c)

Reproduce the s091 final pattern (cf2 hover holding position via
sentai.flow) on PX4 SITL.  Drone flies in Gazebo, downward camera
publishes frames, sentai_sim computes phase-correlation flow, sends
OPTICAL_FLOW_RAD MAVLink to PX4, PX4 EKF2 fuses → position hold
WITHOUT GPS.

## Status (2026-05-12)

| Step | Status |
|---|---|
| `sentai.link.send_flow(dx, dy, dt, q, dist)` wire | ✅ DONE — 56 B OPTICAL_FLOW_RAD frames reach PX4 |
| PX4 EKF2 acknowledges flow uORB publish | ✅ DONE — `handle_message_optical_flow_rad` publishes to `sensor_optical_flow` |
| Gazebo world with PX4 drone (x500) + downward cam + markers | ⏳ NOT DONE |
| sentai_sim reading frames from gz, computing flow | ✅ DONE — works for cf2 via `sentai_crazysim.sdf` already |
| Loop: read flow → convert mgrid→rad → send_flow to PX4 | ⏳ NOT DONE |
| EKF2 param tuning (AID_MASK, OF_QMIN, HGT_MODE) | ⏳ NOT DONE |

So the WIRE is plumbed end-to-end, but the FULL pipeline (gz world
with PX4-controlled drone + downward camera + sentai.flow loop) is
not yet wired into a runnable bench.

## What blocks "just run it"

1. PX4 model `x500` (or similar) needs a `downward_cam` sensor in its
   SDF, publishing to a gz topic that our `camera_bridge_recv.c`
   reads via /tmp/sentai_cam.sock.  The cf2 model already has this;
   adding the same sensor to x500 is straightforward but mechanical.
2. The PX4 model needs to spawn into the SAME `sentai_crazysim.sdf`
   world that holds the ArUco markers — easiest is to add x500 as a
   second model in that world (cf2 can be removed or kept).
3. PX4 mavlink parameters need tweaking for flow-only nav:
   ```
   EKF2_AID_MASK = 2          # enable optical flow fusion only
   EKF2_HGT_MODE = 0          # baro for altitude
   EKF2_OF_QMIN  = 1          # accept low-quality flow
   EKF2_RNG_AID  = 0          # we provide distance in the flow msg
   COM_ARM_WO_GPS = 1         # allow arming without GPS
   ```
4. Loop: a MicroPython script that does
   ```python
   import sentai, time
   sentai.link.init()
   last_t = time.ticks_us()
   while True:
       dx, dy, conf, _, _, _, _ = sentai.flow.read()
       now = time.ticks_us()
       dt = now - last_t; last_t = now
       # convert mgrid (1 mgrid = 1/1000 L0-pixel) to radians
       # 1 L0-pixel = 12.6 mrad (HFOV 58°/640 * 8x decim)
       dx_rad = (dx / 1000.0) * 0.0126
       dy_rad = (dy / 1000.0) * 0.0126
       q = min(255, conf)
       sentai.link.send_flow(dx_rad, dy_rad, dt, q, 1.0)
       time.sleep_ms(50)
   ```

## Phase 6 progression summary

- **6a** (s096): REPL over MAVLink TUNNEL — DONE
- **6b** (s097): arm/takeoff/land via sentai.link.cmd_* — DONE
- **6c** (this folder): send_flow wire — DONE
- **6d** (next session): full gz integration — drone + camera + flow loop

## Why the deferral

Full integration requires adding a PX4 vehicle model to
`sentai_crazysim.sdf` with the downward camera attached, plus param
plumbing for EKF2 flow-only nav.  Both are mechanical work but
non-trivial; should be its own focused session rather than rushed at
the tail of this commit.

The framework is complete: all the PIECES (REPL, control commands,
flow wire, gz camera stream, sentai.flow algorithm) exist and have
been individually verified.  6d is just wiring them up.

# s101b — PX4 flow-only nav (no GPS), no wind

Phase 6d step 2: turn off GPS, fuse optical flow into PX4 EKF2, take
off + hover + land using flow as the X/Y position source.  Baro
remains the altitude source (mirrors cf2+PMW3901 deployment model).

Pipeline now full:

```
gz Garden (sentai_crazysim world with ArUco)
   │
   │ /downward_cam/image    30 Hz 640×480 RGB
   ▼
gz_to_uds_bridge (C++ in distrobox, no Python gz bindings needed)
   │
   │ UDS /tmp/sentai_cam.sock
   ▼
sentai_sim camera_bridge_recv.c → flow_phase_corr.cc → g_flow
   │
   │ link_flow_forward_task (C, 20 Hz, mgrid→rad)
   ▼
sentai_link_send_flow → MAVLink OPTICAL_FLOW_RAD (UDP 14580)
   │
   ▼
PX4 SITL EKF2 (airframe 4041: AID_MASK=2, OF_QMIN=1, HGT_MODE=1, no GPS)
```

## Pass criteria

1. `sentai.link.flow(1)` runs; `stats()[8]` (tx_flow) ≥ 100 within
   first 5 s before any arm command (proves the EKF gets pre-arm
   convergence data).
2. PX4 boots cleanly with airframe 4041 (`COM_ARM_WO_GPS=1`).
3. Drone arms (would FAIL without flow because EKF has no position).
4. Takeoff to z ≥ 1.5 m within 10 s.
5. Hover 15 s, drift mean ≤ 50 cm (loose first-target, no wind).
6. Lands cleanly.

## Comparison frame

- s101 GPS baseline: drift mean 14 cm (sub-20 cm regime).
- s091 cf2 + flow no-wind: drift typically 5-10 cm with anchored
  setpoint hold.
- We expect 20-30 cm because PX4 in AUTO_HOLD with flow-only has
  no setpoint-anchor (just position-hold from EKF fusion).

## STATUS 2026-05-12 — BLOCKED on PX4 EKF semantics

After ~2 h of bring-up, the camera-bridge → C-side flow forwarder →
PX4 OPTICAL_FLOW_RAD path is plumbed end-to-end (1300+ flow frames
sent per session, accepted by PX4).  Airframe 4041 (no GPS, flow-only)
boots cleanly.  SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION are accepted
(PX4 logs "New NED origin (LLA)").

But PX4 EKF2 in this config still returns `pos_horiz_ratio: NaN` →
arm rejected with `MAV_RESULT_TEMPORARILY_REJECTED`.

**Root cause** (per PX4 forum + GitHub issue #22250 research):
Optical flow is fundamentally a *velocity* sensor, not a *position*
sensor.  PX4 EKF2 cannot establish absolute horizontal position from
flow alone, even with origin/home set.  Quote from the PX4 forum:
"Optical flow sensor outputs velocity measurements and not position.
Therefore the position will eventually drift and you cannot 'visually
navigate' with only optical flow without GPS."

Note: cf2 firmware works around this by integrating flow velocity in
its own controller — there's no PX4-style EKF stage that demands an
absolute position estimate before arming.

## Paths forward (s102+ candidates)

1. **OFFBOARD mode with continuous SET_POSITION_TARGET_LOCAL_NED**
   — bypass PX4's AUTO.TAKEOFF flow.  Drone takes whatever target
   we stream.  Closest analogue to cf2's cflib `send_position()`.
2. **VISION_POSITION_ESTIMATE from ArUco PnP** — host-side ArUco
   detector + PnP gives absolute drone pose; feed PX4 EKF2 via
   `VISION_POSITION_ESTIMATE` MAVLink msg.  PX4's `x500_vision`
   airframe is exactly this pattern.
3. **Pre-load EKF position via external EKF2_AID hooks** — patch
   PX4 to accept a synthetic "I am at origin" injection before
   arming.

(1) is the most direct.  (2) reuses s091 ArUco PnP code, drift
bounded by marker visibility, closest to "fly using cameras alone."
(3) requires PX4 source edits.

Current s101b artifacts: pipeline plumbing verified; takeoff blocked
pending one of the paths above.

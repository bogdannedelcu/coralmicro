---
name: cf2-sitl-cheat-odom-gt
description: "HARD RULE.  cf2 SITL in CrazySim has a Gazebo plugin (`gz-sim-odometry-publisher-system`) that publishes ground-truth pose on `/cf_<id>/odom` → `crazysim_plugin.cpp:OdomCallback` injects it into cf2 firmware as `CrtpExtPose`.  Result: cf2 EKF receives perfect pose at 200 Hz and ALL prior cf2-side drift claims (s127 7.4 cm, s130 1.2 cm, s142 1.8 cm, s147+ closures, etc.) are MASKED CHEATS, not real flow/PnP performance.  Disabled 2026-05-17 in `model.sdf.jinja`.  Replacement: ArUco PnP → `cf.extpos.send_extpos` in host scripts."
metadata:
  type: feedback
  node_type: memory
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

## The cheat

CrazySim's stock cf2 model SDF includes the plugin
`gz-sim-odometry-publisher-system` that publishes Gazebo's own
ground-truth pose for the cf2 link on `/cf_<id>/odom` at 200 Hz.  The
`gz_crazysim_plugin` subscribes to that topic and forwards each pose
into the cf2 firmware as a `CrtpExtPose_s` external-pose update —
bypassing IMU/flow/vision entirely.

Source: `model.sdf.jinja` line 393 (before disable) +
`crazysim_plugin.cpp:OdomCallback` line 178-189.

## Why this is a load-bearing problem

**Every** cf2-side drift number this project has ever published was
computed with the cheat active.  The "drift" actually measured was
cf2 EKF noise around the cheat's perfect pose — typically ≤ 10 cm
even under wind.  Real-stack drift (IMU + optical flow + ArUco VPE)
is unknown.

Affected baselines (all SUSPECT until rerun with cheat off):
- s127 FlowBaseline (7.4 cm canonical) — see
  `[[flowbaseline-canonical-config]]`
- s130 image-only nav (1.2 cm position err)
- s142 HexPatrol (1.8 cm closure)
- s147..s150 MP-migration closures (8-11 cm)
- s162/s163 SlamTask (lifecycle only, not flight — OK)

## Disable procedure (already applied 2026-05-17)

```xml
<!-- DISABLED 2026-05-17 — anti-cheat ([[cf2-sitl-cheat-odom-gt]]). -->
<!--
<plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
    <dimensions>3</dimensions>
    <odom_publish_frequency>200</odom_publish_frequency>
    <odom_topic>/cf_{{ cf_id }}/odom</odom_topic>
</plugin>
-->
```

Backup: `model.sdf.jinja.pre_no_cheat_20260517`.

Verification: `distrobox enter crazysim-garden -- gz topic -l` must
NOT list `/cf_0/odom`.  If it does, the patch is not applied.

## How to keep cf2 stable post-disable

**Mandatory replacements** (none of these were in place before
2026-05-17):

1. **VPE forwarder**: send PnP-derived pose to cf2 via
   `cf.extpos.send_extpos(x, y, z)`.  Implemented in
   `aruco_hover.py:_vpe_send_if_due` — gated by `SENTAI_VPE_MODE`
   env var (`continuous` | `init_only` | `off`).
2. **POSITION setpoint**, not velocity: the old
   `mc.start_linear_motion(0, 0, 0)` only requested "zero velocity",
   which let the drone STAY wherever it drifted to in the first few
   seconds (no anchor).  Use
   `cf.commander.send_position_setpoint(0, 0, z_target, 0)` to
   actively pull the drone back to the commanded location.
3. **Gentle PID**: PnP noise (~cm) makes the old aggressive PID
   (`xKp=3.0`, `xVelMax=2.5`) overshoot and oscillate.  Use
   `xKp=1.0`, `xVelMax=0.5` post-cheat.
4. **VPE rate ≤ 5 Hz**: matches PnP cadence; 20 Hz spams identical
   noisy samples and the cf2 PID chases the jitter.
5. **Correct `KNOWN_POSITIONS_M` and `MARKER_SIZE_M` in
   `aruco_detector.py`** — must match the world SDF marker poses
   exactly; PnP returns nonsense otherwise and VPE poisons the EKF.

## Anti-cheat invariants

- GT topic `/world/.../dynamic_pose/info` MAY be consumed *host-side
  only*, *post-mortem*, for verdict comparison plots (see
  `[[sentai-sim-air-gapped-from-truth]]`).  NEVER injected into cf2
  or sentai_sim.
- VPE input MUST be vision-derived (PnP on real camera frames or a
  documented equivalent).  Marker world coordinates are a-priori map
  knowledge (printed pad has known geometry), NOT GT.
- If any test produces a drift number better than the empirical
  IMU+flow no-VPE baseline (≈ 10 cm/15 s hover, see
  `[[flowbaseline-canonical-config]]` POST-cheat-removal), audit for
  re-introduced cheat before celebrating.

## Cross-references

- `[[op-s8-w1-cf2-sim-honest]]` — project tracking the recovery WP
- `[[sentai-sim-air-gapped-from-truth]]` — broader anti-cheat rule
- `[[experiments-start-from-origin]]` — reproducibility complement
- `[[flowbaseline-canonical-config]]` — superseded by post-cheat baseline (TBD)
- Plugin source: `/home/bogdan/work/crazyflie/CrazySim/.../crazysim_plugin.{cpp,h}`
- Plugin disable: `/home/bogdan/work/crazyflie/CrazySim/.../model.sdf.jinja:393`

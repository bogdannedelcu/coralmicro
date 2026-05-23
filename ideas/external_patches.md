# External repository patch log

Chronological journal of patches applied OUTSIDE the coralmicro git tree.
Append-only.  English only.  Format: see `[[external-repo-patch-log]]` skill.

## 2026-05-17 OP-S8-W1-T1 — disable gz-sim-odometry-publisher cheat plugin

**Repo**: `/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie`
**File**: `model.sdf.jinja`
**Reason**: Crisis OP-S8-W1 — upstream CrazySim ships the
`gz-sim-odometry-publisher-system` plugin enabled by default.  Combined
with `crazysim_plugin.cpp::OdomCallback` (which subscribes to
`/cf_X/odom` and forwards as `CrtpExtPose`), cf2 EKF receives perfect
GT pose at 200 Hz → ALL prior cf2-side drift claims are masked
cheats.  See [[cf2-sitl-cheat-odom-gt]] + [[op-s8-w1-cf2-sim-honest]].
**Patch type**: edit (wrap `<plugin>` block in XML comment)
**Backup**: `model.sdf.jinja.pre_no_cheat_20260517`
**Revert command**:
```bash
cp /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja.pre_no_cheat_20260517 \
   /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja
```
**Status**: canonical-now (the cheat must STAY disabled per anti-cheat HR)
**Notes**: DO NOT revert.  If you find the cheat re-enabled, restore THIS
patch.  Upstream CrazySim doesn't intend it as a cheat — they ship it for
their lighthouse / mocap demos — but for us it bypasses every vision
subsystem.

## 2026-05-21 OP-S10-W21-T4d-phase2 — iter-85+ TOF injection (REVERTED 2026-05-22)

**Repo**: `/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo`
**Files**: `plugins/CrazySim/crazysim_plugin.cpp`,
          `models/crazyflie/model.sdf.jinja`
**Reason**: Session 2 chicken-and-egg fix — cf2 SITL can't take off
without Z source.  Patch rewrote `OdomCallback` to forward only z (not
full pose) as `SENSOR_TOF_SIM` (VL53L1x emulation) AND re-enabled the
odometry-publisher plugin so the callback fires.  Acknowledged as a
hack pending the proper fix (RPYT thrust ramp).
**Patch type**: edit (3 noise stddevs zeroed + plugin re-enable + callback rewrite)
**Backups**:
- `plugins/CrazySim/crazysim_plugin.cpp.s187_tof_backup`
- `models/crazyflie/model.sdf.jinja.s187_tof_backup`
**Revert command**:
```bash
cd /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation
git checkout -- simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp
git checkout -- simulator_files/gazebo/models/crazyflie/model.sdf.jinja
# then re-apply 2026-05-17 comment-out of OdometryPublisher per the entry above
```
**Status**: reverted (2026-05-22, OP-S10-W21-T7 / s190)
**Notes**: Reverted because it injects GT into cf2 EKF (anti-cheat
violation per [[sentai-sim-air-gapped-from-truth]]).  Replacement
strategy: RPYT thrust ramp (port 3 ch 0) for pre-airborne climb, then
ExtPos (port 6 ch 0) with vision-PnP-derived position once markers are
visible.  See `s190_calib_bringup_rpyt_hl_handoff/README.md`.

## 2026-05-22 OP-S10-W21-T7 — disable baro subscription in gz_crazysim_plugin

**Repo**: `/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim`
**File**: `crazysim_plugin.cpp`
**Reason**: cf2 SITL was dropping 13k+ packets per s190 run.
Plugin subscribes to Gazebo baro topic and pushes ~50 pkt/s into
cf2's socketlink queue (depth 2000) even though cf2 firmware has
`KALMAN_USE_BARO_UPDATE` commented out (per the no-baro HR).  Wasted
queue capacity contributed to overflow when other packets surged.
HW-parity: real CF Brushless has baro on the chip but firmware
ignores it — matching SIM behaviour requires either disabling baro
at sensor level (this patch) or wasting CRTP bandwidth.
**Patch type**: edit (comment out one Subscribe call in initializeSubsAndPub)
**Backup**: `crazysim_plugin.cpp.pre_baro_disable_20260522`
**Revert command**:
```bash
cp /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp.pre_baro_disable_20260522 \
   /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp
distrobox enter crazysim-garden -- cmake --build /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/sitl_make/build/build_crazysim_gz --target gz_crazysim_plugin
```
**Status**: applied + plugin .so rebuilt (libgz_crazysim_plugin.so)
**Notes**: Pairs with KALMAN_USE_BARO_UPDATE disable below — together
they remove baro from the SIM data path entirely.  cf2 then has
ONLY IMU as native sensor (matches real flow_deck-less CF Brushless).

## 2026-05-XX — cf2 firmware Kalman: KALMAN_USE_BARO_UPDATE disabled

**Repo**: `/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware`
**File**: `src/modules/src/estimator/estimator_kalman.c`
**Reason**: Operator hard rule "no baro" — real CF Brushless ships
with baro disabled; SIM must mirror.  Without this patch, cf2 EKF
fuses baro and gets a free Z source, again masking real flow/vision
performance.
**Patch type**: edit (`#define KALMAN_USE_BARO_UPDATE` commented out)
**Backup**: none — git tracks the change; revert via `git checkout --`.
**Revert command**:
```bash
cd /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware
git checkout -- src/modules/src/estimator/estimator_kalman.c
```
**Status**: canonical-now (the no-baro stance is permanent for SentAI)
**Notes**: Pair with the OdometryPublisher disable above — together
they leave cf2 with ONLY IMU + (optional) vision-derived ExtPos as
state sources, which matches the real flow_deck-less CF Brushless.

## 2026-05-23 OP-S10-W17-T10 — add asymmetric 7th WhyCon marker `N`

**Repo**: `/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware`
**File**: `tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_whycon_small.sdf`
**Reason**: The small WhyCon pad originally has 6 markers in a
rectangular symmetric layout (NW/NE/W/E/SW/SE).  Symmetric layouts
produce a dual-solution PnP problem — for any frame, there exists a
mirror-branch camera pose that fits the projection equations with
near-identical reprojection error but yaw flipped 180°.  Per the
T10 offline ablation (368-frame Gazebo dataset, see
`dataset/TD-S10-B2/.../validation_20260523_141003/`), this caused
~6 OpenCV pose outliers of ~1 m in the validator's exhaustive
permutation path.  Adding one asymmetric marker breaks the symmetry
and constrains the correspondence solver to a single branch.
**Patch type**: addition (insert `<model name="whycon_N">` after
`<model name="whycon_SE">`, before camera-mount section)
**Marker position**: `+0.02 +0.10 0.005` (matches the position the
B1 dataset generator already injects into its run-local world copy,
so flight uses the SAME 7-marker layout that the offline ablation
validated)
**Backup**: `sentai_whycon_small.sdf.pre_marker_N_20260523`
**Revert command**:
```bash
cp /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_whycon_small.sdf.pre_marker_N_20260523 \
   /home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_whycon_small.sdf
```
**Status**: applied (canonical for flight tests as of 2026-05-23 evening)
**Notes**: Layout now 7 markers — `NW, NE, W, E, SW, SE, N`.  Any
mission using this world MUST register all 7 in `MARKER_WORLD` and
pass marker_n=7 to `sentai.calib.run_bringup(...)`.  Pre-existing
experiments that still register only 6 markers (s187, s190, s191)
will detect the 7th marker as a "false positive" relative to their
hardcoded layout and may need updating before re-running.

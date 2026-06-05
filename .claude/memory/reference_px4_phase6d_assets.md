---
name: PX4 Phase 6d asset map — where everything lives
description: Pointers for x500_sentai model, canonical s091 world (CrazySim vendor tree), camera mount parity table, ArUco marker poses. Required for apples-to-apples cf2↔PX4 comparison.
type: reference
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
The canonical sentai_crazysim world (with 4 ArUco markers id0..3 used
by s091 wind hover) lives in the **CrazySim vendor tree**, NOT in
coralmicro:

  `/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_crazysim.sdf`

Internal world name: `sentai_crazysim` (file is `sentai_crazysim.sdf`).
GZ resource path must include the `worlds/` dir for textures.

Do NOT use `sim/gazebo/sentai_crazysim_world.sdf` from coralmicro for
PX4 hover bench — it's a stripped variant WITHOUT ArUco markers (only
color cubes + cat).  Kept around for camera-bridge smoke tests only.

PX4 drone model `x500_sentai`:

  `/home/bogdan/work/px4/PX4-Autopilot/Tools/simulation/gz/models/x500_sentai/`

Spawned by env vars `PX4_GZ_MODEL=x500_sentai PX4_GZ_WORLD=sentai_crazysim`
on top of `PX4_SYS_AUTOSTART=4001`.  Spawn name in PX4 is `x500_sentai_0`.

Camera mount on x500_sentai matches cf2 orientation (pitch=π/2 yaw=π,
HFOV 1.0123, 640×480 R8G8B8, 30 Hz, topic `/downward_cam/image`).
Position is `0 0 -0.05` (centered, 5 cm below CoM); cf2 is `-0.04 0
-0.02`.  Difference irrelevant for flow algorithm; matters only for
ArUco PnP ground-truth (cam-to-CoM compensation).

Full table + launch recipe documented in Sim.md §10n.

Smoke test that proves the spawn works:
`experiments/s100_px4_gz_smoke/run.sh`.

ArUco markers (id0..3) at fixed positions `KNOWN_POSITIONS_M` from
`s090_hover_over_cat/aruco_detector.py`:
- 0: (+0.15, +0.10, 0.15) — top z=0.20
- 1: (-0.15, +0.10, 0.15)
- 2: (-0.15, -0.10, 0.15)
- 3: (+0.15, -0.10, 0.15)

Effective ArUco size 0.0625 m (texture has 22% white padding;
`MARKER_SIZE_M = 0.0625` in aruco_detector, not 0.08).

**World walls enlarged 2× on 2026-05-12** to fit x500 (~46 cm vs cf2
~10 cm wingspan).  Enclosure now 6×6×3 m (was 3×3×1.5 m); ground
plane 8×8 (was 4×4).  Backup at `sentai_crazysim.sdf.pre_x500_walls_20260512`.
Marker/cat/wind positions UNCHANGED → s091 drift numbers remain
the comparison baseline.

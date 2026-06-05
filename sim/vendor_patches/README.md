# SentAI external sim/emulator deltas (vendored from OLD PC, 2026-06-05)

Purpose: make the `coralmicro` git repository self-sufficient for rebuilding
the SentAI sim/emulator environment on a fresh machine.  The four external
git repos that live outside `coralmicro/` (CrazySim, crazyflie-firmware,
crazyflie-simulation, PX4-Autopilot) all carry SentAI-specific
deviations from upstream.  Those deviations live here as committed bytes
so a NEW PC can re-apply them deterministically.

## Layout

```
sim/vendor_patches/
├── README.md                       # this file
├── manifest/
│   ├── versions.txt                # pinned commits + tool versions
│   └── distrobox_recipe.md         # crazysim-garden bootstrap
├── crazysim/                       # outer CrazySim repo
│   ├── INFO.txt                    # remote, HEAD, recent commits, local-only
│   ├── delta_vs_upstream.patch     # working-tree diff vs origin/main
│   └── status.txt                  # `git status` snapshot
├── crazyflie-firmware/             # cf2 firmware (already a fork)
│   ├── INFO.txt
│   ├── delta_vs_upstream.patch
│   └── status.txt
├── crazyflie-simulation/           # gazebo worlds/models/plugins submodule
│   ├── INFO.txt
│   ├── delta_vs_upstream.patch
│   ├── status.txt
│   └── files/                      # wholly-new SentAI assets ready to copy
│       └── simulator_files/gazebo/
│           ├── worlds/{sentai_*.sdf, s125_*.sdf}
│           ├── worlds/materials/textures/{aruco_*, imagenet_cat.png,
│           │                              suburbs_bg.png, whycon_*, harmonic_*}
│           ├── materials/textures/{harmonic_*, whycon_*}
│           ├── models/crazyflie/model.sdf.jinja       # MODIFIED (anti-cheat)
│           ├── plugins/CrazySim/crazysim_plugin.cpp   # MODIFIED (baro disable)
│           └── gui/CrazySimDashboard/{*.cc,*.hh,*.qml,...}   # new Qt GUI
└── px4/
    ├── INFO.txt
    ├── delta_vs_upstream.patch     # empty (PX4 source unmodified)
    ├── status.txt
    └── files/
        ├── Tools/simulation/gz/models/x500_sentai/    # ENTIRE SentAI Gazebo model
        └── airframes-from-build/{4040_,4041_,4042_,4043_}gz_x500_sentai{,_flow,_vision,_gps_vpe}
```

## Source of truth per external repo

| Repo                    | Remote                                                         | Branch          | Pin commit    |
|-------------------------|----------------------------------------------------------------|-----------------|---------------|
| CrazySim (outer)        | https://github.com/llanesc/CrazySim.git                        | main            | `3ec8b55`     |
| crazyflie-firmware      | https://github.com/bogdannedelcu/crazysim-crazyflie-firmware.git | sentai-flow-sim-support | `e437425` |
| crazyflie-simulation    | https://github.com/llanesc/crazyflie-simulation.git            | (detached)      | `235aa99`     |
| PX4-Autopilot           | https://github.com/PX4/PX4-Autopilot.git                       | release/1.14    | `1555f2b`     |

`crazyflie-firmware` is already in our org fork — the working tree's
extra `estimator_kalman.c` kalman-no-baro tweak is in
`delta_vs_upstream.patch` and should either be applied at apply-time or
pushed as a follow-up commit on `sentai-flow-sim-support`.

`crazyflie-simulation` is **NOT** forked yet.  It carries 8 local
SentAI-only commits + working-tree-only uncommitted edits.  Either:
- fork it under `bogdannedelcu/crazysim-crazyflie-simulation` and push
  `235aa99` + a follow-up commit for the uncommitted bits, then update
  crazyflie-firmware's submodule pointer; **or**
- treat the patch + files/ here as the canonical re-apply payload.

## Apply on a NEW PC

Run the in-repo installers first to clone upstream into the standard
locations:

```bash
bash sim/scripts/install_gazebo_harmonic.sh    # only if host Gazebo needed
bash sim/scripts/install_crazysim.sh           # clones to ~/work/crazyflie/CrazySim/
bash sim/scripts/install_px4_sitl.sh           # clones to ~/work/px4/PX4-Autopilot
```

Then apply the SentAI deltas:

```bash
# 1. CrazySim submodule pointer (just bumps to our cf2 fw fork pin)
cd ~/work/crazyflie/CrazySim
git apply $REPO/sim/vendor_patches/crazysim/delta_vs_upstream.patch

# 2. crazyflie-firmware: switch to our fork at the SentAI branch
cd crazyflie-firmware
git remote set-url origin https://github.com/bogdannedelcu/crazysim-crazyflie-firmware.git
git fetch origin
git checkout sentai-flow-sim-support
# Working-tree kalman-no-baro patch:
git apply $REPO/sim/vendor_patches/crazyflie-firmware/delta_vs_upstream.patch

# 3. crazyflie-simulation submodule: check out the SentAI-aware commit
cd tools/crazyflie-simulation
git fetch origin
git checkout 235aa999ae368c6db33bccc05355fd987bb17d7e
# Then drop the wholly-new asset files (gui/, worlds/, textures/) and apply
# the working-tree patch on top of the checkout:
cp -rT $REPO/sim/vendor_patches/crazyflie-simulation/files/ ./
git apply $REPO/sim/vendor_patches/crazyflie-simulation/delta_vs_upstream.patch

# 4. PX4: drop x500_sentai model + airframes
cd ~/work/px4/PX4-Autopilot
cp -rT $REPO/sim/vendor_patches/px4/files/Tools ./Tools
mkdir -p build/px4_sitl_default/etc/init.d-posix/airframes
cp $REPO/sim/vendor_patches/px4/files/airframes-from-build/* \
   build/px4_sitl_default/etc/init.d-posix/airframes/
# NOTE: the airframes are normally staged into build/ during a PX4
# CMake build.  Because they were never added to ROMFS source on the
# OLD PC, you need to recopy them after any `make px4_sitl_default`
# rebuild.  A proper fix is to upstream them into
# ROMFS/px4fmu_common/init.d-posix/airframes/ + CMakeLists.txt — see
# manifest/distrobox_recipe.md "Notes on PX4 SentAI airframes".

# 5. crazysim-garden distrobox (Gazebo Garden 7.9) — see
# manifest/distrobox_recipe.md
```

## Deltas inventory + cross-check vs `ideas/external_patches.md`

The following deltas were captured.  Items already documented in
`ideas/external_patches.md` or `Sim.md` are marked DOC; items NOT
previously documented are marked **GAP** — those are the most valuable
findings.

### crazyflie-simulation (the SentAI gazebo submodule)

| Path                                                              | Type     | Documented? |
|-------------------------------------------------------------------|----------|-------------|
| `simulator_files/gazebo/models/crazyflie/model.sdf.jinja`         | edit     | DOC (`external_patches.md` 2026-05-17 no-cheat + larger diff is **GAP**) |
| `simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp`     | edit     | DOC (2026-05-22 baro disable) |
| `simulator_files/gazebo/worlds/sentai_crazysim.sdf` (committed)   | new      | DOC (Sim.md §2341)            |
| `simulator_files/gazebo/worlds/sentai_crazysim.sdf` (working-tree on top of commit) | edit | **GAP** |
| `simulator_files/gazebo/worlds/sentai_whycon_small.sdf`           | new      | DOC (`external_patches.md` 2026-05-23 marker N) |
| `simulator_files/gazebo/worlds/sentai_whycon.sdf`                 | new      | **GAP**                       |
| `simulator_files/gazebo/worlds/s125_demo.sdf`                     | new      | **GAP** (s125 experiment)     |
| `simulator_files/gazebo/worlds/s125_simple.sdf`                   | new      | **GAP** (s125 experiment)     |
| `simulator_files/gazebo/worlds/materials/textures/aruco_4x4_50_id[0-3].png` | new | **GAP** (used by sentai_crazysim.sdf?) |
| `simulator_files/gazebo/worlds/materials/textures/imagenet_cat.png` | new   | **GAP** (probably used by s230 markers) |
| `simulator_files/gazebo/worlds/materials/textures/suburbs_bg.png`   | new   | **GAP**                       |
| `simulator_files/gazebo/worlds/materials/textures/whycon_krajnik.png` | new | **GAP** (WhyCon target)       |
| `simulator_files/gazebo/worlds/materials/textures/harmonic_alt200_4k.png` | new (24 MB) | **GAP** |
| `simulator_files/gazebo/materials/textures/harmonic_alt200_4k.png` (duplicate of above) | new (24 MB) | **GAP** |
| `simulator_files/gazebo/materials/textures/whycon_krajnik.png` (duplicate) | new | **GAP** |
| `simulator_files/gazebo/gui/CrazySimDashboard/*`                  | new      | **GAP** (entire Qt GUI module) |

8 SentAI-prefixed local commits ahead of `origin/crazysim` (`235aa99`
.. `6360bee`) are listed in `crazyflie-simulation/INFO.txt`; the patch
captures only the working-tree delta on top of `235aa99`.

### crazyflie-firmware (`bogdannedelcu/crazysim-crazyflie-firmware.git`)

| Path                                                              | Type     | Documented? |
|-------------------------------------------------------------------|----------|-------------|
| `src/modules/src/estimator/estimator_kalman.c` (working tree)     | edit     | DOC (`external_patches.md` 2026-05-21 kalman/baro story; current state is no-baro) |
| Commit `40d34708` socketlink overflow drop+count                  | committed| DOC (briefing item #5)        |
| Commit `e4374251` accept extended SENSOR_FLOW_SIM stddev          | committed| **GAP** (newer than docs)     |
| Commit `6b8188da` stable-altitude port                            | committed| **GAP** (newer than docs)     |
| Commit `a5257a08` flowdeck sensor sim + SITL estimator selection  | committed| **GAP**                       |
| Commit `1b2c497c` SITL sensor noise + camera + ext pose tuning    | committed| **GAP**                       |
| Commit `b56a6152` colorled_sitl replacement for ledring_sitl      | committed| **GAP**                       |
| Commit `67d41ba3` LED + LED ring SITL stubs                       | committed| **GAP**                       |
| Commit `e650a260` Multi-ranger + LED ring + ledseq for SITL       | committed| **GAP**                       |

All of these are commits on `sentai-flow-sim-support`.  They are
canonical via the fork URL; the gap is doc, not bytes.

### CrazySim outer repo

| Path                              | Type     | Documented? |
|-----------------------------------|----------|-------------|
| Submodule pointer for `crazyflie-firmware` | edit | DOC (we use the fork) |
| `UART_RPYT_AGENT_PROMPT.md`       | new (untracked) | **GAP** (operator/agent note, possibly safe to ignore but worth knowing) |

### PX4-Autopilot

| Path                                                              | Type     | Documented? |
|-------------------------------------------------------------------|----------|-------------|
| `Tools/simulation/gz/models/x500_sentai/` (full dir)              | new      | DOC partly (Sim.md §2304 says path; **GAP**: no patch/asset bytes tracked) |
| `build/.../airframes/4040_gz_x500_sentai`                         | new      | DOC partly (Sim.md §2507/2706 names them; **GAP**: never staged into ROMFS source) |
| `build/.../airframes/4041_gz_x500_sentai_flow`                    | new      | same      |
| `build/.../airframes/4042_gz_x500_sentai_vision`                  | new      | same      |
| `build/.../airframes/4043_gz_x500_sentai_gps_vpe`                 | new      | same      |

PX4 source repo carries **zero** committed source modifications; all
deltas are wholly-new files.

## Dangling dependencies (no source on this OLD PC)

- `crazyflie-lib-python` (cflib) — directory `~/work/crazyflie/crazyflie-lib-python/` does not exist on OLD PC; cflib is installed only into `~/work/crazyflie/.venv/`.  The library is referenced by host bridge scripts and mission tooling.  Install path on NEW PC: pip-install matching the OLD PC's `.venv` version (see `manifest/versions.txt`; cflib `__version__` attribute is not exposed — pin via the venv's installed version).
- `~/work/crazyflie/crazyswarm2_ws/src/crazyswarm2` submodule (`2334b7a3`) sits inside CrazySim and is pinned; install via CrazySim's own bootstrap.

## What this commit does NOT include

- Compiled artifacts: `crazysim` build output, PX4 build/, `libgz_crazysim_plugin.so` — rebuild from source.
- `libedgetpu.so.1` — rebuild from `https://github.com/google-coral/libedgetpu` per the SHA in `manifest/versions.txt`.
- `apex_latest_single_ep.bin` — ships with libedgetpu source; SHA in
  `manifest/versions.txt` is the source of truth.
- Distrobox container image bytes — recreated via `manifest/distrobox_recipe.md`.
- `.pre_*`, `.s127_pre_*`, `.windbackup`, `.checker_backup` snapshot files
  — local backups, noise.
- PX4 runtime state (`dataman`, `etc/`, `parameters.bson`,
  `parameters_backup.bson` in PX4 cwd) — regenerated at SITL startup.

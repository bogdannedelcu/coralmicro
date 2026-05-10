# sim/scripts/

Host-side helper scripts for the SIM build prerequisites and ops.

## Files

- **`install_gazebo_harmonic.sh`** — installs Gazebo Harmonic on Ubuntu 24.04
  (Noble) from the official OSRF apt repo.  Required for Phase 4+ (camera
  socket → Gazebo image bridge, closed-loop hover in simulation).  One-time
  install (~500 MB download); after this, `gz sim` is on PATH.

  Run: `bash sim/scripts/install_gazebo_harmonic.sh` (will prompt for sudo
  password once).  Verifies install with `gz --version`.

  Source: https://gazebosim.org/docs/harmonic/install_ubuntu

- **`install_crazysim.sh`** — clones [CrazySim](https://github.com/llanesc/CrazySim)
  (Crazyflie SITL with Gazebo support) into `/home/bogdan/work/crazyflie/CrazySim/`
  (NOT into coralmicro/, per Sim.md rule #2.3) and installs build deps.
  Required for Phase 3+ (CrazySim is the simulated Crazyflie that the SentAI
  optical-flow updates target via radio CRTP).

  CrazySim CMake auto-detects Gazebo Garden (gz-sim7) OR Harmonic (gz-sim8)
  — README is outdated (says "Garden") but the plugin code already handles
  Harmonic via gz-msgs10 / gz-transport13.  We use Harmonic.

  Run: `bash sim/scripts/install_crazysim.sh` (clones ~815 MB, installs
  build deps via apt).  After this, build SITL per CrazySim README:
  `cd ~/work/crazyflie/CrazySim/crazyflie-firmware && make cf2_defconfig && make sitl_make -j$(nproc)`

## Why these live in the repo (not /tmp)

`/tmp` is wiped on reboot and invisible to git.  Anything that's a
durable build/deploy prerequisite belongs alongside the code.  Same
discipline as `examples/sentai_runtime/experiments/sNNN_*/` (see
`CLAUDE.md`).

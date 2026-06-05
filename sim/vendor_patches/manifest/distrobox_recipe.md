# `crazysim-garden` distrobox recipe (snapshot from OLD PC, 2026-06-05)

The OLD PC runs Gazebo Garden 7.9 inside an Ubuntu 22.04 distrobox named
`crazysim-garden` (host = Debian 13; Garden 7.9 is the only Gazebo version
that works with cf2 SITL — Harmonic 8.x is **BANNED** per `Sim.md` because
of a hard sensor-calibration deadlock).

## Existing in-repo install scripts (use these on NEW PC)

```bash
bash sim/scripts/install_gazebo_harmonic.sh     # only if host needs Harmonic
bash sim/scripts/install_crazysim.sh            # CLONES upstream CrazySim
bash sim/scripts/install_px4_sitl.sh            # CLONES upstream PX4
```

Then APPLY THE VENDORED DELTAS from `sim/vendor_patches/` (see top-level
README.md in this directory) to restore SentAI-specific worlds, plugins,
firmware patches, PX4 model + airframes.

## Distrobox container inspection (OLD PC running state)

```text
NAME             | STATUS     | IMAGE                          | CREATED
crazysim-garden  | Up 27 hr   | docker.io/library/ubuntu:22.04 | 2026-05-10
```

The container was created with default distrobox flags (no `--init`,
no `--nvidia`).  No explicit creation command is recorded in shell
history, so it was likely created interactively.  The fastest reproduction
on a fresh machine is:

```bash
# 1. Create the container
distrobox create --name crazysim-garden --image ubuntu:22.04

# 2. Enter it once to bootstrap
distrobox enter crazysim-garden

# 3. Inside the container, install Gazebo Garden 7.9 from the OSRF apt repo
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
     -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
     | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update
sudo apt-get install -y gz-garden

# 4. Verify
gz sim --versions   # must report 7.9.x
```

Once Garden is installed inside the container, the existing
`sim/scripts/install_crazysim.sh` and `install_px4_sitl.sh` can run
**inside the container** to build cf2 SITL + PX4 against Garden.  The
working bridge in `examples/sentai_runtime/host/sentai_crazy_cpx_udp_bridge.py`
runs **on the host** and talks to the SITL stack inside the container
via the shared `--net=host` network model that distrobox provides by
default.

## Notes on PX4 SentAI airframes (specific to NEW PC reproduction)

The four SentAI airframe scripts (`4040_gz_x500_sentai` etc.) exist
ONLY in `build/px4_sitl_default/etc/init.d-posix/airframes/` on the
OLD PC — they are NOT staged in PX4 ROMFS source.  This means a clean
PX4 build on NEW PC will not produce them.  Vendored copies are under
`sim/vendor_patches/px4/files/airframes-from-build/`; see the top-level
`sim/vendor_patches/README.md` for the apply-time staging recipe.

This was likely an oversight in OLD-PC setup (the files should have
been added to `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`
so PX4's normal build path stages them).  Fixing that properly is a
follow-up; for now the vendored copies are the source of truth.

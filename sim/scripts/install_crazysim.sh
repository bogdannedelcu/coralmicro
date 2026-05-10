#!/bin/bash
# install_crazysim.sh
# Clones CrazySim (Crazyflie SITL with Gazebo support) into the operator's
# crazyflie/ workspace and installs build dependencies.
#
# DOES NOT vendor crazyflie-firmware into coralmicro/ (per Sim.md rule #2.3).
# Lives at /home/bogdan/work/crazyflie/CrazySim/ alongside the operator's
# other crazyflie repos.
#
# After this script: build with the README instructions
# (cd CrazySim/crazyflie-firmware && make cf2_defconfig && make sitl_make)
# then launch a Gazebo world with the SITL drone.
#
# Run with: bash sim/scripts/install_crazysim.sh
# (sudo will prompt for password once at the start)

set -e
set -o pipefail

CRAZY_ROOT=/home/bogdan/work/crazyflie
CRAZYSIM_DIR=$CRAZY_ROOT/CrazySim

echo "=== [1/4] Verify Gazebo Harmonic is installed ==="
if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR: 'gz' not found.  Run sim/scripts/install_gazebo_harmonic.sh first."
    exit 1
fi
GZ_VER=$(gz sim --versions 2>/dev/null | head -1)
echo "Gazebo Sim: $GZ_VER"
if ! [[ "$GZ_VER" =~ ^8\. ]]; then
    echo "WARNING: expected Gazebo Sim 8.x.x (Harmonic).  Found: $GZ_VER"
    echo "         CrazySim plugin auto-detects 7 or 8 — should work, but untested."
fi

echo ""
echo "=== [2/4] Install build deps (apt) ==="
sudo apt-get install -y \
    cmake build-essential git \
    libgz-cmake3-dev libgz-sim8-dev libgz-plugin2-dev \
    libgz-transport13-dev libgz-msgs10-dev \
    protobuf-compiler libgoogle-glog-dev libeigen3-dev libxml2-utils \
    python3-pip python3-jinja2 swig

echo ""
echo "=== [3/4] Clone CrazySim (recursive) into $CRAZYSIM_DIR ==="
mkdir -p "$CRAZY_ROOT"
if [ -d "$CRAZYSIM_DIR/.git" ]; then
    echo "Already cloned — pulling latest"
    (cd "$CRAZYSIM_DIR" && git pull --recurse-submodules)
else
    git clone --recursive https://github.com/llanesc/CrazySim.git "$CRAZYSIM_DIR"
fi

echo ""
echo "=== [4/4] Verify cflib Python (must be source-installed, NOT pip) ==="
if /home/bogdan/work/crazyflie/.venv/bin/python -c "import cflib" 2>/dev/null; then
    echo "cflib found in venv-coral / .venv"
    /home/bogdan/work/crazyflie/.venv/bin/python -c "import cflib; import os; print('cflib path:', os.path.dirname(cflib.__file__))"
else
    echo "cflib MISSING — install from source per CrazySim README:"
    echo "  cd $CRAZY_ROOT && git clone https://github.com/bitcraze/crazyflie-lib-python"
    echo "  cd crazyflie-lib-python && pip install -e ."
fi

echo ""
echo "============================================================"
echo "===  CRAZYSIM CLONED + DEPS INSTALLED  ====================="
echo "============================================================"
echo ""
echo "Location: $CRAZYSIM_DIR"
echo "Size: $(du -sh $CRAZYSIM_DIR | cut -f1)"
echo ""
echo "Next: build SITL firmware (CMake-based, NOT make sitl_make):"
echo "  cd $CRAZYSIM_DIR/crazyflie-firmware"
echo "  make cf2_defconfig"
echo "  cmake -B sitl_make/build -S sitl_make"
echo "  cmake --build sitl_make/build -j\$(nproc)"
echo ""
echo "  Note: requires 'python' in PATH (Ubuntu 24.04 only has 'python3')."
echo "  Fix: 'sudo apt install python-is-python3' OR"
echo "       'ln -sfv /usr/bin/python3 ~/.local/bin/python'"
echo ""
echo "Then launch single drone in Gazebo (auto-opens GUI):"
echo "  bash $CRAZYSIM_DIR/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh"
echo ""
echo "Connect via cflib: URI 'udp://127.0.0.1:19850' (mirrors radio:// API)."

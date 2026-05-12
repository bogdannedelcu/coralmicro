#!/bin/bash
# install_px4_sitl.sh
# Clones PX4-Autopilot release/1.14 into the operator's px4/ workspace and
# installs the build dependencies INSIDE the crazysim-garden distrobox
# (Garden 7.9 + gz-transport12 + gz-msgs9, which v1.14 natively supports).
#
# DOES NOT vendor PX4 into coralmicro/ (per Sim.md rule §2.3).
# Lives at /home/bogdan/work/px4/PX4-Autopilot/ alongside other SITL repos.
#
# Compatibility verified (Sim.md §10m):
#   - PX4 v1.14 src/modules/simulation/gz_bridge/CMakeLists.txt requires
#     gz-transport12  →  matches our distrobox stack exactly.
#   - PX4 default gz world uses the same 7 plugins as sentai_crazysim.sdf.
#   - MAVLink UDP ports (14540/14550, 18570) disjoint from CrazySim (19850).
#
# After this script: build SITL with the README instructions printed at
# the end.
#
# Run with: bash sim/scripts/install_px4_sitl.sh

set -e
set -o pipefail

PX4_ROOT=/home/bogdan/work/px4
PX4_DIR=$PX4_ROOT/PX4-Autopilot
PX4_BRANCH=release/1.14
DISTROBOX_NAME=crazysim-garden

echo "=== [1/4] Verify crazysim-garden distrobox exists ==="
if ! distrobox list 2>/dev/null | grep -q "$DISTROBOX_NAME"; then
    echo "ERROR: distrobox '$DISTROBOX_NAME' not found."
    echo "       Run sim/scripts/install_crazysim.sh first to create it."
    exit 1
fi
echo "Distrobox '$DISTROBOX_NAME' present."
GZ_VER=$(distrobox enter $DISTROBOX_NAME -- gz sim --versions 2>/dev/null | head -1)
echo "Gazebo Sim inside distrobox: $GZ_VER"
if ! [[ "$GZ_VER" =~ ^7\. ]]; then
    echo "WARNING: expected Garden 7.x.  Found: $GZ_VER"
    echo "         PX4 v1.14 may fail to find gz-transport12."
fi

echo ""
echo "=== [2/4] Install PX4 build deps INSIDE the distrobox ==="
# Most of these overlap with what CrazySim already pulled in; apt is
# idempotent so re-running is safe.
distrobox enter $DISTROBOX_NAME -- sudo apt-get install -y \
    python3-empy python3-jinja2 python3-toml python3-numpy python3-yaml \
    python3-pip python3-pkg-resources \
    ninja-build cmake build-essential git \
    libopencv-dev protobuf-compiler libeigen3-dev \
    libgz-sim7-dev libgz-transport12-dev libgz-msgs9-dev \
    genromfs gawk

# PX4 also wants kconfiglib via pip; system has no apt pkg.
distrobox enter $DISTROBOX_NAME -- pip3 install --user --upgrade \
    kconfiglib jsonschema pyserial pyyaml

echo ""
echo "=== [3/4] Clone PX4-Autopilot ($PX4_BRANCH, recursive) into $PX4_DIR ==="
mkdir -p "$PX4_ROOT"
if [ -d "$PX4_DIR/.git" ]; then
    echo "Already cloned — checking out $PX4_BRANCH and updating submodules"
    (cd "$PX4_DIR" && git fetch origin "$PX4_BRANCH" && git checkout "$PX4_BRANCH" \
       && git submodule update --init --recursive)
else
    git clone --branch "$PX4_BRANCH" --recursive --depth 1 \
        https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
fi

echo ""
echo "=== [4/4] Sanity: verify cmake can find gz-transport12 inside distrobox ==="
distrobox enter $DISTROBOX_NAME -- bash -c "cd $PX4_DIR && \
    cmake -P /dev/stdin <<'EOF' 2>&1 | head -5
find_package(gz-transport NAMES gz-transport12 QUIET)
if(gz-transport_FOUND)
    message(STATUS \"gz-transport12 OK (\${gz-transport_DIR})\")
else()
    message(FATAL_ERROR \"gz-transport12 NOT found — install libgz-transport12-dev\")
endif()
EOF"

echo ""
echo "============================================================"
echo "===  PX4 v1.14 CLONED + DEPS INSTALLED  ===================="
echo "============================================================"
echo ""
echo "Location: $PX4_DIR"
echo "Branch:   $PX4_BRANCH"
echo "Size:     $(du -sh $PX4_DIR 2>/dev/null | cut -f1)"
echo ""
echo "Next: build SITL firmware INSIDE the distrobox (gz-* libs live there):"
echo "  distrobox enter $DISTROBOX_NAME -- bash -c \\"
echo "    'cd $PX4_DIR && make px4_sitl_default'"
echo ""
echo "Build output: $PX4_DIR/build/px4_sitl_default/bin/px4"
echo ""
echo "Quick smoke (PX4 + gz + x500 quad, fresh world):"
echo "  distrobox enter $DISTROBOX_NAME -- bash -c \\"
echo "    'cd $PX4_DIR && make px4_sitl gz_x500'"
echo ""
echo "MAVLink (instance 0): UDP 14540 (offboard remote), 14550 (legacy),"
echo "                       18570 (GCS).  Test with QGroundControl or:"
echo "  pip install pymavlink && python3 -c \\"
echo "    'from pymavlink import mavutil; m=mavutil.mavlink_connection(\"udp:127.0.0.1:14540\"); m.wait_heartbeat(); print(m.target_system)'"
echo ""
echo "Coexistence with CrazySim (same gz instance + sentai_crazysim.sdf world):"
echo "  See Sim.md §10m for the spawn-into-existing-world recipe."

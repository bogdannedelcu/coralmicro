#!/bin/bash
# Hybrid launcher: Xvfb on :99 for gz server render (fixes Sim.md §10d
# render-thread starvation) + GUI gz sim -g on real DISPLAY=:1 with
# sentai_gui.config so the operator sees the PIP /downward_cam/image.
# Runs INSIDE crazysim-garden distrobox.
set -e

WORLD="${1:-sentai_crazysim}"
GUI_CONFIG="/home/bogdan/work/coralmicro/sim/gazebo/sentai_gui.config"
SRC_PATH="/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware"
BUILD_PATH="$SRC_PATH/sitl_make/build"

# Save the real display BEFORE we override
REAL_DISPLAY="${DISPLAY:-:1}"

# Env: gz plugin paths, textures
export CF2_SIM_MODEL=gz_crazyflie
source "$SRC_PATH/tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash" \
    "$SRC_PATH" "$BUILD_PATH"
export GZ_SIM_RESOURCE_PATH="/home/bogdan/work/coralmicro/sim/gazebo:${GZ_SIM_RESOURCE_PATH}"
export PYTHONPATH="/usr/lib/python3/dist-packages:/usr/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"

cleanup() {
    pkill -x cf2 2>/dev/null || true
    pkill -9 ruby 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 Xvfb 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

echo "[hybrid] killing prior cf2/gz/Xvfb"
pkill -x cf2 2>/dev/null || true
pkill -9 Xvfb 2>/dev/null || true
sleep 1

echo "[hybrid] starting Xvfb on :99 (drives server-side OGRE2 render)"
Xvfb :99 -screen 0 1024x768x24 > /tmp/xvfb.log 2>&1 &
echo $! > /tmp/xvfb.pid
sleep 1

echo "[hybrid] starting gz sim SERVER (world=$WORLD) on DISPLAY=:99"
DISPLAY=:99 stdbuf -oL -eL gz sim -s -r -v 3 \
    "$SRC_PATH/tools/crazyflie-simulation/simulator_files/gazebo/worlds/${WORLD}.sdf" \
    > /tmp/gz_server.log 2>&1 &
SERVER_PID=$!
echo $SERVER_PID > /tmp/gz_server.pid
sleep 8       # 2026-05-22: bumped 4→8 for spawn-service readiness

# Spawn the drone
WORKING_DIR="$BUILD_PATH/0"
mkdir -p "$WORKING_DIR"
cd "$WORKING_DIR"
python3 \
    "$SRC_PATH/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py" \
    "$SRC_PATH/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja" \
    "$SRC_PATH/tools/crazyflie-simulation/simulator_files/gazebo" \
    --cffirm_udp_port 19950 --cflib_udp_port 19850 --cf_id 0 --cf_name cf \
    --output-file /tmp/crazyflie_0.sdf

echo "[hybrid] spawning crazyflie_0 at (0, 0, 0.0) — retry up to 3× / 5s timeout"
SPAWN_OK=0
for spawn_try in 1 2 3; do
    if gz service -s /world/${WORLD}/create \
        --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
        --req 'sdf_filename: "/tmp/crazyflie_0.sdf", pose: {position: {x:0, y:0, z: 0.0}}, name: "crazyflie_0", allow_renaming: 1' \
        2>&1 | grep -q "data: true"; then
        echo "[hybrid] spawn attempt $spawn_try → OK"
        SPAWN_OK=1
        break
    fi
    echo "[hybrid] spawn attempt $spawn_try → retry in 2 s"
    sleep 2
done
if [ "$SPAWN_OK" != "1" ]; then
    echo "[hybrid] ABORT — spawn failed after 3 retries.  See /tmp/gz_server.log."
    exit 2
fi

echo "[hybrid] starting cf2 SITL"
stdbuf -oL -eL "$BUILD_PATH/cf2" 19950 > /tmp/cf2.log 2> /tmp/cf2.err &
echo $! > /tmp/cf2.pid

cd - >/dev/null

echo "[hybrid] starting GUI on REAL display=$REAL_DISPLAY with sentai_gui.config"
DISPLAY=$REAL_DISPLAY gz sim -g --gui-config "$GUI_CONFIG"

#!/bin/bash
# s125 — launch CrazySim Gazebo with s125_demo world + cf2 drone, then
# run the orchestrator that takes off, walks a square, lands, and builds
# a sentai.places world model.
#
# Run from the host (not from inside distrobox):
#   bash examples/sentai_runtime/experiments/s125_integrated_demo/scripts/run_demo.sh
#
# Internally re-enters distrobox crazysim-garden where cflib + gz live.
set -e

REPO=$(cd "$(dirname "$0")"/../../../../.. && pwd)
CRAZYSIM=/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo

# Sanity check: world + texture in place, sentai_sim built.
[ -f "$CRAZYSIM/worlds/s125_demo.sdf" ] || {
  echo "FATAL: world not staged.  cp $REPO/examples/.../world/s125_demo.sdf $CRAZYSIM/worlds/"
  exit 1
}
[ -f "$CRAZYSIM/materials/textures/harmonic_alt200_4k.png" ] || {
  echo "FATAL: harmonic texture not staged."
  exit 1
}
[ -x "$REPO/build-sim/sim/sentai_sim" ] || {
  echo "FATAL: sentai_sim not built — run: cmake --build build-sim --target sentai_sim"
  exit 1
}

echo "[s125] launching Gazebo + cf2 (background)"
bash "$CRAZYSIM/launch/sitl_singleagent.sh" -w s125_demo -m crazyflie -x 0 -y 0 &
SITL_PID=$!

cleanup() {
  echo "[s125] cleanup"
  kill -9 $SITL_PID 2>/dev/null || true
  pkill -x cf2 2>/dev/null || true
  pkill -9 "gz sim" 2>/dev/null || true
  pkill -9 ruby 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT

# Give Gazebo + cf2 time to come up (GUI loads, drone spawns, EKF settles)
echo "[s125] waiting 12 s for sim to stabilize…"
sleep 12

# Run orchestrator inside distrobox (where cflib lives)
echo "[s125] starting orchestrator"
distrobox enter crazysim-garden -- python3 "$REPO/examples/sentai_runtime/experiments/s125_integrated_demo/scripts/orchestrator.py"

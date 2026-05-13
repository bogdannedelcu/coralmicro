#!/bin/bash
# s125 — launch CrazySim Gazebo with s125_demo world + cf2 drone, then
# run the orchestrator that takes off, walks a square, lands, and builds
# a sentai.places world model.
#
# Run from the host (not from inside distrobox):
#   bash examples/sentai_runtime/experiments/s125_integrated_demo/scripts/run_demo.sh
#
# Per Sim.md §10w.1: gz sim MUST run inside the crazysim-garden distrobox
# (host has Harmonic 8, plugin needs Garden 7).  Per §10w.2: textures
# resolve via a worlds/materials -> ../materials symlink.  This script
# applies both idempotently.
set -e

REPO=$(cd "$(dirname "$0")"/../../../../.. && pwd)
CRAZYSIM=/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo

# 1. Stage world SDF + texture into CrazySim install (idempotent).
mkdir -p "$CRAZYSIM/materials/textures"
cp -u "$REPO/examples/sentai_runtime/experiments/s125_integrated_demo/world/s125_demo.sdf" \
      "$CRAZYSIM/worlds/"
cp -u "$REPO/sim/gazebo/worlds/assets/harmonic_tiles/harmonic_alt200_4k.png" \
      "$CRAZYSIM/materials/textures/"

# 2. Symlink worlds/materials -> ../materials so <albedo_map>materials/...
#    resolves correctly (§10w.2).
if [ ! -e "$CRAZYSIM/worlds/materials" ]; then
  ln -s ../materials "$CRAZYSIM/worlds/materials"
fi

# 3. Verify sentai_sim is built (orchestrator pipes into it).
[ -x "$REPO/build-sim/sim/sentai_sim" ] || {
  echo "FATAL: sentai_sim not built — run:"
  echo "  cmake --build $REPO/build-sim --target sentai_sim"
  exit 1
}

# 4. Clean any stale processes (§10w.6 reverse-spawn order).
cleanup() {
  echo "[s125] cleanup"
  pkill -f "orchestrator.py" 2>/dev/null || true
  pkill -x cf2 2>/dev/null || true
  pkill -9 "gz sim" 2>/dev/null || true
  pkill -9 ruby 2>/dev/null || true
}
trap cleanup SIGINT SIGTERM EXIT
cleanup
sleep 1

# 5. Inside distrobox: start CrazySim SITL (Garden 7.9 gz + Garden-built
#    plugin + cf2 firmware binary).  The launcher backgrounds gz sim -s -r,
#    spawns cf2 (listening on UDP 19950 for the plugin, UDP 19850 for cflib),
#    and opens gz sim -g for the GUI.  ALL of these run inside distrobox.
echo "[s125] launching CrazySim SITL inside distrobox crazysim-garden"
distrobox enter crazysim-garden -- \
  bash "$CRAZYSIM/launch/sitl_singleagent.sh" -w s125_demo -m crazyflie -x 0 -y 0 \
  > /tmp/s125_sitl.log 2>&1 &
SITL_HOST_PID=$!

# 6. Wait for cf2 firmware to be up (it listens on UDP 19850 for cflib).
#    distrobox shares the host network namespace so 127.0.0.1 from the host
#    reaches the cf2 process inside the container.
echo "[s125] waiting for cf2 firmware on udp://127.0.0.1:19850 (up to 30 s)…"
for i in $(seq 1 30); do
  if nc -zu 127.0.0.1 19850 2>/dev/null; then
    echo "[s125] cf2 UDP port open (after ${i}s)"
    break
  fi
  sleep 1
done

# Extra grace for Gazebo GUI to come up + EKF to settle
sleep 5

# 7. Run orchestrator on the HOST using its venv (cflib lives there) —
#    NOT in distrobox.  Reasons:
#      a) sentai_sim was built against host glibc (2.38); distrobox is on
#         glibc 2.35 and would fail with `version `GLIBC_2.38' not found`.
#      b) Distrobox doesn't have cflib by default; the host venv does.
#      c) cflib speaks UDP to 127.0.0.1:19850 which traverses the shared
#         network namespace and reaches cf2 inside the container.
echo "[s125] starting orchestrator on HOST"
# Tee both to a log file AND the operator's terminal so progress messages
# (takeoff, waypoint, places.info(), …) appear live next to the Gazebo GUI.
"$REPO/venv/bin/python3" -u \
  "$REPO/examples/sentai_runtime/experiments/s125_integrated_demo/scripts/orchestrator.py" \
  2>&1 | tee /tmp/s125_orchestrator.log

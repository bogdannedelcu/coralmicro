#!/bin/bash
# respawn_sitl.sh — stop + restart cf2 SITL + Gazebo so the drone
# physically respawns at world origin (0, 0, 0.5).  Per
# [[experiments-start-from-origin]] this is MANDATORY before every
# closed-loop flight test — `kalman.resetEstimation` zeros the EKF but
# does NOT physically move cf2.  Reusing a leftover cf2 from the prior
# mission means the test runs from a biased starting position and is
# non-reproducible (see operator pushback 2026-05-14, 2026-05-17).
#
# Usage:  bash sim/scripts/respawn_sitl.sh [world=sentai_crazysim]
# Exit 0 once UDP 19850 is listening + Gazebo GUI is up.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORLD="${1:-sentai_crazysim}"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
WORKDIR=/tmp/respawn_sitl
mkdir -p "$WORKDIR"

echo "[respawn] stopping prior cf2 + gz + Xvfb"
distrobox enter crazysim-garden -- pkill -9 -f gz_to_uds_bridge 2>/dev/null || true
distrobox enter crazysim-garden -- pkill -9 -f 'gz sim'        2>/dev/null || true
distrobox enter crazysim-garden -- pkill -9 -f sitl_make       2>/dev/null || true
distrobox enter crazysim-garden -- pkill -9 Xvfb               2>/dev/null || true
sleep 2

echo "[respawn] launching fresh SITL stack (world=$WORLD)"
distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
    > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
disown $! 2>/dev/null || true

echo "[respawn] waiting for cf2 UDP 19850..."
deadline=$(( $(date +%s) + 150 ))
until ss -lun 2>/dev/null | grep -q ':19850'; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[respawn] FAIL — SITL never came up; see $WORKDIR/sitl.log"
        exit 1
    fi
    sleep 2
done
# Give Gazebo a moment to actually spawn the model + cf2 to boot estimator
sleep 4
echo "[respawn] cf2 respawned at world origin (UDP 19850 ready)"

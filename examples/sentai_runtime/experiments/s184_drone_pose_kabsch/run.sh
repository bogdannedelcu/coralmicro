#!/bin/bash
# s184 -- OP-S10-W19-T6b drone-pose Kabsch + yaw-anchor smoke.
# Pure synthetic; no Gazebo, no flight, no camera bridge.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
DRIVER_SRC="$SCRIPT_DIR/test_drone_pose_s184.py"
DRIVER_DST="$FS_ROOT/test_drone_pose_s184.py"
WORKDIR=/tmp/s184_drone_pose_kabsch
mkdir -p "$WORKDIR"
LOG="$WORKDIR/sentai_sim.log"

if [ ! -x "$SIM_BIN" ]; then
    echo "[s184] sentai_sim missing -- building."
    cmake --build "$REPO_ROOT/build-sim" --target sentai_sim
fi
if [ ! -f "$DRIVER_SRC" ]; then
    echo "[s184] FAIL -- driver missing at $DRIVER_SRC" >&2
    exit 1
fi

mkdir -p "$FS_ROOT"
cp "$DRIVER_SRC" "$DRIVER_DST"

# Drive sentai_sim through stdin: import driver once, exit on EOF.
( cd "$FS_ROOT" && echo "import test_drone_pose_s184" | \
    timeout 15 "$SIM_BIN" ) > "$LOG" 2>&1 || true

python3 "$SCRIPT_DIR/verdict.py" "$LOG"

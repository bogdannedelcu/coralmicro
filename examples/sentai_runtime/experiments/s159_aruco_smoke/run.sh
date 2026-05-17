#!/bin/bash
# s159 — sentai.aruco synthetic smoke (OP-S6-W3-T7).
# Pure C-side synthesis; no Gazebo, no camera bridge.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
DRIVER_SRC="$SCRIPT_DIR/test_aruco_s159.py"
WORKDIR=/tmp/s159_aruco_smoke
mkdir -p "$WORKDIR"
LOG="$WORKDIR/sentai_sim.log"

if [ ! -x "$SIM_BIN" ]; then
    echo "[s159] sentai_sim missing — building."
    cmake --build "$REPO_ROOT/build-sim" --target sentai_sim
fi
if [ ! -f "$DRIVER_SRC" ]; then
    echo "[s159] FAIL — driver missing at $DRIVER_SRC" >&2
    exit 1
fi

mkdir -p "$FS_ROOT"
cp "$DRIVER_SRC" "$FS_ROOT/test_aruco_s159.py"

( cd "$FS_ROOT" && echo "import test_aruco_s159" | \
    timeout 15 "$SIM_BIN" ) > "$LOG" 2>&1 || true

python3 "$SCRIPT_DIR/verdict.py" "$LOG"

#!/bin/bash
# s186 -- sentai.calib smoke on WhyCon square pad (OP-S10-W19-T8).
# Pure synthetic; no Gazebo, no flight, no camera bridge.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
DRIVER_SRC="$SCRIPT_DIR/test_calib_s186.py"
DRIVER_DST="$FS_ROOT/test_calib_s186.py"
WORKDIR=/tmp/s186_calib_whycon_synth
mkdir -p "$WORKDIR"
LOG="$WORKDIR/sentai_sim.log"

if [ ! -x "$SIM_BIN" ]; then
    echo "[s186] sentai_sim missing -- building."
    cmake --build "$REPO_ROOT/build-sim" --target sentai_sim
fi
if [ ! -f "$DRIVER_SRC" ]; then
    echo "[s186] FAIL -- driver missing at $DRIVER_SRC" >&2
    exit 1
fi

mkdir -p "$FS_ROOT"
cp "$DRIVER_SRC" "$DRIVER_DST"

# Stale calib state would taint T6 round-trip.
( cd "$FS_ROOT" && rm -f cam_calib.json )

( cd "$FS_ROOT" && echo "import test_calib_s186" | \
    timeout 15 "$SIM_BIN" ) > "$LOG" 2>&1 || true

python3 "$SCRIPT_DIR/verdict.py" "$LOG"

#!/bin/bash
# s189 -- OP-S10-W21-T3 Kp persist round-trip smoke.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
DRIVER_SRC="$SCRIPT_DIR/test_calib_kp_s189.py"
DRIVER_DST="$FS_ROOT/test_calib_kp_s189.py"
WORKDIR=/tmp/s189_calib_kp_persist
mkdir -p "$WORKDIR"
LOG="$WORKDIR/sentai_sim.log"

if [ ! -x "$SIM_BIN" ]; then
    cmake --build "$REPO_ROOT/build-sim" --target sentai_sim
fi

mkdir -p "$FS_ROOT"
cp "$DRIVER_SRC" "$DRIVER_DST"
( cd "$FS_ROOT" && rm -f calib.ini cam_calib.json )

( cd "$FS_ROOT" && echo "import test_calib_kp_s189" | \
    timeout 15 "$SIM_BIN" ) > "$LOG" 2>&1 || true

python3 "$SCRIPT_DIR/verdict.py" "$LOG"

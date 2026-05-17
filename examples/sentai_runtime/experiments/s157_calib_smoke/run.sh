#!/bin/bash
# s157 — sentai.calib smoke (OP-S6-W1-T5).
# Pure synthetic; no Gazebo, no flight, no camera bridge.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
DRIVER_SRC="$SCRIPT_DIR/test_calib_s157.py"
DRIVER_DST="$FS_ROOT/test_calib_s157.py"
WORKDIR=/tmp/s157_calib_smoke
mkdir -p "$WORKDIR"
LOG="$WORKDIR/sentai_sim.log"

if [ ! -x "$SIM_BIN" ]; then
    echo "[s157] sentai_sim missing — building."
    cmake --build "$REPO_ROOT/build-sim" --target sentai_sim
fi
if [ ! -f "$DRIVER_SRC" ]; then
    echo "[s157] FAIL — driver missing at $DRIVER_SRC" >&2
    exit 1
fi

# Stage the MP test driver into sentai_fs_root/ (gitignored build dir).
mkdir -p "$FS_ROOT"
cp "$DRIVER_SRC" "$DRIVER_DST"

# Stale calib state would taint T6 round-trip (the host-stdio fallback
# of sentai_calib_save writes ./cam_calib.json in the cwd of sentai_sim,
# which is sentai_fs_root/ when launched via the run.sh).
( cd "$FS_ROOT" && rm -f cam_calib.json )

# Drive sentai_sim through stdin: one-shot `import test_calib_s157`, exit.
( cd "$FS_ROOT" && echo "import test_calib_s157" | \
    timeout 15 "$SIM_BIN" ) > "$LOG" 2>&1 || true

python3 "$SCRIPT_DIR/verdict.py" "$LOG"

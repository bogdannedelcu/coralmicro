#!/bin/bash
# s155 — Loop closure on sentai.servo paradigm.
# Enforces [[experiments-start-from-origin]].
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
RESPAWN="$REPO_ROOT/sim/scripts/respawn_sitl.sh"
HEX_HELPERS="$REPO_ROOT/examples/sentai_runtime/experiments/s142_hex_descriptor_patrol/hex_helpers.py"
WORKDIR=/tmp/s155_loop_closure_servo
mkdir -p "$WORKDIR"

echo "[s155] respawning cf2 SITL at world origin"
bash "$RESPAWN" sentai_crazysim

echo "[s155] staging mission + hex_helpers"
cp "$HEX_HELPERS" "$FS_ROOT/"
cp "$SCRIPT_DIR/mission_s155.py" "$FS_ROOT/"

echo "[s155] running mission_s155"
echo "import mission_s155; r = mission_s155.run(); print('FINAL:', r['status'])" \
    | timeout 120 "$SIM_BIN" 2>&1 | tee "$WORKDIR/run.log" | tail -10

echo "[s155] verdict"
python3 "$SCRIPT_DIR/verdict.py"

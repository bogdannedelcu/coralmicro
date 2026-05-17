#!/bin/bash
# s154 — LOST recovery on sentai.servo paradigm.
# Enforces [[experiments-start-from-origin]].
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
RESPAWN="$REPO_ROOT/sim/scripts/respawn_sitl.sh"
WORKDIR=/tmp/s154_lost_recovery_servo
mkdir -p "$WORKDIR"

echo "[s154] respawning cf2 SITL at world origin"
bash "$RESPAWN" sentai_crazysim

echo "[s154] staging mission"
cp "$SCRIPT_DIR/mission_s154.py" "$FS_ROOT/"

echo "[s154] running mission_s154"
echo "import mission_s154; r = mission_s154.run(); print('FINAL:', r['status'])" \
    | timeout 120 "$SIM_BIN" 2>&1 | tee "$WORKDIR/run.log" | tail -10

echo "[s154] verdict"
python3 "$SCRIPT_DIR/verdict.py"

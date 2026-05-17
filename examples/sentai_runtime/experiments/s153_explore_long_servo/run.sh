#!/bin/bash
# s153 — long-distance exploration on sentai.servo paradigm.
#
# Enforces [[experiments-start-from-origin]] by ALWAYS respawning the
# SITL stack before the run, so cf2 starts at world (0, 0, 0.5).
# Mission has a hard origin-bias assertion that aborts if not at origin.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
RESPAWN="$REPO_ROOT/sim/scripts/respawn_sitl.sh"
WORKDIR=/tmp/s153_explore_long_servo
mkdir -p "$WORKDIR"

echo "[s153] respawning cf2 SITL at world origin"
bash "$RESPAWN" sentai_crazysim

echo "[s153] staging mission"
cp "$SCRIPT_DIR/mission_s153.py" "$FS_ROOT/"

echo "[s153] running mission_s153"
echo "import mission_s153; r = mission_s153.run(); print('FINAL:', r['status'])" \
    | timeout 120 "$SIM_BIN" 2>&1 | tee "$WORKDIR/run.log" | tail -10

echo "[s153] verdict"
python3 "$SCRIPT_DIR/verdict.py"

#!/bin/bash
# s192 — minimal WhyCon detection regression against T10's detector port.
# WBS: OP-S10-W21-T12 regression.  Built on s191's launch infrastructure.
#
# Pipeline:
#   1. cleanup stragglers from prior runs
#   2. (re-)build sentai_sim
#   3. stage mission_s192.py into FS root
#   4. launch sim stack (Gazebo + cf2 + bridge + gt_recorder)
#   5. run mission inside sentai_sim REPL
#   6. snapshot artifacts into iter sub-folder
#   7. run verdict
#   8. cleanup

set -e

ITER_TAG="${1:-iter1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
EXP_DIR="$REPO_ROOT/examples/sentai_runtime/experiments/s192_calib_whycon_postT10"
ITER_DIR="$EXP_DIR/$ITER_TAG"
mkdir -p "$ITER_DIR"

WORKDIR="$ITER_DIR"
WORLD="${WORLD:-sentai_whycon_small}"
LAUNCH_SIM="$REPO_ROOT/sim/scripts/launch_sim.sh"
CLEANUP_SIM="$REPO_ROOT/sim/scripts/launch_sim_cleanup.sh"
SENTAI_SIM="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

# FR dir per iter (overrides default in main_sim.c).
export SENTAI_FR_DIR="$WORKDIR/fr_current"

# ---- trap: kill children + clean processes on ANY exit ---------------
cleanup_on_exit() {
    echo "[s192/$ITER_TAG] trap: killing background processes"
    bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
}
trap cleanup_on_exit EXIT INT TERM

# ---- 1. cleanup ------------------------------------------------------
echo "[s192/$ITER_TAG] pre-cleanup (kill stragglers)"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
rm -rf "$WORKDIR/fr_current" "$WORKDIR/launch_sim.pids" "$WORKDIR"/*.log 2>/dev/null || true
mkdir -p "$WORKDIR/fr_current/frames"

# ---- 2. build sentai_sim --------------------------------------------
echo "[s192/$ITER_TAG] building sentai_sim"
cmake --build "$REPO_ROOT/build-sim" --target sentai_sim 2>&1 | tail -3

# ---- 3. stage mission -----------------------------------------------
echo "[s192/$ITER_TAG] staging mission_s192.py → $FS_ROOT/"
mkdir -p "$FS_ROOT"
cp "$EXP_DIR/mission_s192.py" "$FS_ROOT/mission_s192.py"

# ---- 4. launch sim stack --------------------------------------------
echo "[s192/$ITER_TAG] launching sim stack (world=$WORLD)"
bash "$LAUNCH_SIM" "$WORLD" "$WORKDIR" &
LAUNCH_PID=$!

sleep 8

# ---- 5. run mission in sentai_sim REPL ------------------------------
echo "[s192/$ITER_TAG] running mission in sentai_sim REPL"
REPL_LOG="$WORKDIR/sentai_repl.log"
echo "import mission_s192; r = mission_s192.run(); print('FINAL:', r.get('status','?'), 'pass:', r.get('pass',False))" \
    | timeout 90 "$SENTAI_SIM" > "$REPL_LOG" 2>&1 || true

# ---- 6. snapshot artifacts ------------------------------------------
echo "[s192/$ITER_TAG] snapshotting artifacts"
[[ -f "$FS_ROOT/mission_s192_journal.txt" ]] \
    && cp "$FS_ROOT/mission_s192_journal.txt" "$WORKDIR/"
[[ -f "$FS_ROOT/mission_s192_summary.json" ]] \
    && cp "$FS_ROOT/mission_s192_summary.json" "$WORKDIR/"

# ---- 7. verdict -----------------------------------------------------
if [[ -x "$EXP_DIR/verdict_s192.py" ]]; then
    echo "[s192/$ITER_TAG] running verdict"
    python3 "$EXP_DIR/verdict_s192.py" "$WORKDIR" > "$WORKDIR/verdict.log" 2>&1 || true
    cat "$WORKDIR/verdict.log"
fi

# ---- 8. cleanup -----------------------------------------------------
echo "[s192/$ITER_TAG] cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
echo "[s192/$ITER_TAG] done. artefacts in $WORKDIR"

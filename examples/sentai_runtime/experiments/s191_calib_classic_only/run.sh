#!/bin/bash
# s191 — calib RPYT-only cascaded PD (OP-S10-W21-T12).
# Supersedes s190.  Classic Commander throughout (no HL handoff).
#
# Pipeline:
#   1. Cleanup (launch_sim_cleanup.sh, safe even on first run)
#   2. Build sentai_sim
#   3. Stage mission_s191.py into FS root
#   4. Start sim stack via launch_sim.sh (Gazebo + cf2 + bridge + gt_recorder)
#   5. Run mission in sentai_sim REPL
#   6. Snapshot artifacts (journal, summary, FR, gt.jsonl)
#   7. Run verdict_s191.py
#   8. Cleanup

set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
EXP_DIR="$REPO_ROOT/examples/sentai_runtime/experiments/s191_calib_classic_only"
WORKDIR="$EXP_DIR"

WORLD="${WORLD:-sentai_whycon_small}"
LAUNCH_SIM="$REPO_ROOT/sim/scripts/launch_sim.sh"
CLEANUP_SIM="$REPO_ROOT/sim/scripts/launch_sim_cleanup.sh"
SENTAI_SIM="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

# FR dir per experiment (overrides default in main_sim.c).
export SENTAI_FR_DIR="$WORKDIR/fr_current"

# ---- Trap: kill children + clean processes on ANY exit (incl. Ctrl-C) -
# Without this, ^C during build/launch leaves Gazebo + cf2 + gt_recorder
# orphans accumulating across iters → SIM gets slower over time.
cleanup_on_exit() {
    echo "[s191] trap: killing background processes"
    bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
}
trap cleanup_on_exit EXIT INT TERM

# ---- 1. cleanup --------------------------------------------------------
echo "[s191] pre-cleanup (kill stragglers from prior runs)"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
rm -rf "$WORKDIR/fr_current" "$WORKDIR/launch_sim.pids" "$WORKDIR"/*.log 2>/dev/null || true
mkdir -p "$WORKDIR/fr_current/frames"

# ---- 2. build sentai_sim ----------------------------------------------
echo "[s191] building sentai_sim"
cmake --build "$REPO_ROOT/build-sim" --target sentai_sim 2>&1 \
    | tail -5

# ---- 3. stage mission --------------------------------------------------
echo "[s191] staging mission_s191.py → $FS_ROOT/"
mkdir -p "$FS_ROOT"
cp "$EXP_DIR/mission_s191.py" "$FS_ROOT/mission_s191.py"

# ---- 4. launch sim stack ----------------------------------------------
echo "[s191] launching sim stack (world=$WORLD)"
bash "$LAUNCH_SIM" "$WORLD" "$WORKDIR" &
LAUNCH_PID=$!

# Wait for sim stack to settle (Gazebo + cf2 boot, camera frames flowing).
sleep 8

# ---- 5. run mission in sentai_sim REPL --------------------------------
echo "[s191] running mission in sentai_sim REPL"
REPL_LOG="$WORKDIR/sentai_repl.log"
echo "import mission_s191; r = mission_s191.run(); print('FINAL:', r['status'])" \
    | timeout 240 "$SENTAI_SIM" > "$REPL_LOG" 2>&1 || true

# ---- 6. snapshot artifacts --------------------------------------------
echo "[s191] snapshotting artifacts"
# journal.txt + summary.json are written by mission to $FS_ROOT — copy back.
[[ -f "$FS_ROOT/mission_s191_journal.txt" ]] \
    && cp "$FS_ROOT/mission_s191_journal.txt" "$WORKDIR/"
[[ -f "$FS_ROOT/mission_s191_summary.json" ]] \
    && cp "$FS_ROOT/mission_s191_summary.json" "$WORKDIR/"

# ---- 7. verdict --------------------------------------------------------
if [[ -x "$EXP_DIR/verdict_s191.py" ]]; then
    echo "[s191] running verdict"
    python3 "$EXP_DIR/verdict_s191.py" "$WORKDIR" > "$WORKDIR/verdict.log" 2>&1 || true
    tail -20 "$WORKDIR/verdict.log"
fi

# ---- 8. cleanup --------------------------------------------------------
echo "[s191] cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true

echo "[s191] done. logs in $WORKDIR"

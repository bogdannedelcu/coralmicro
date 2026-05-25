#!/bin/bash
# s194 — closed-loop PD per-axis ID with level-settle capture.
#
# Drives sentai.calib.run_bringup() inside sentai_sim through the
# full cf2 SITL + WhyCon stack.  The axis ID phase is replaced by a
# closed-loop PD that parks the drone at symmetric ±δ targets along
# each body axis, with 150 ms zero-attitude level-settle before each
# capture.  See mission_s194.py `_phase4_5_axis_id_rpyt` and README.md.
#
# Anti-cheat invariants ([[sentai-sim-air-gapped-from-truth]]):
#   - sentai_sim consumes camera frames + cf2 CRTP telemetry only.
#   - gt_recorder runs HOST-SIDE; output JSONL feeds verdict.py for
#     post-mortem only.  NEVER fed back to sentai_sim.
#
# WBS: OP-S10-W21-T15.

set -e

ITER_TAG="${1:-iter1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
EXP_DIR="$REPO_ROOT/examples/sentai_runtime/experiments/s194_calib_axis_id_pd_closed_loop"
ITER_DIR="$EXP_DIR/$ITER_TAG"
mkdir -p "$ITER_DIR"

WORKDIR="$ITER_DIR"
WORLD="${WORLD:-sentai_whycon_small}"
LAUNCH_SIM="$REPO_ROOT/sim/scripts/launch_sim.sh"
CLEANUP_SIM="$REPO_ROOT/sim/scripts/launch_sim_cleanup.sh"
SENTAI_SIM="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

# FR dir per iter.
export SENTAI_FR_DIR="$WORKDIR/fr_current"

cleanup_on_exit() {
    echo "[s194/$ITER_TAG] trap: killing background processes"
    bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
}
trap cleanup_on_exit EXIT INT TERM

# ---- 1. cleanup ------------------------------------------------------
echo "[s194/$ITER_TAG] pre-cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
rm -rf "$WORKDIR/fr_current" "$WORKDIR/launch_sim.pids" "$WORKDIR"/*.log 2>/dev/null || true
mkdir -p "$WORKDIR/fr_current/frames"

# ---- 2. build sentai_sim --------------------------------------------
echo "[s194/$ITER_TAG] building sentai_sim"
cmake --build "$REPO_ROOT/build-sim" --target sentai_sim 2>&1 | tail -3

# ---- 3. stage mission -----------------------------------------------
echo "[s194/$ITER_TAG] staging mission_s194.py → $FS_ROOT/"
mkdir -p "$FS_ROOT"
cp "$EXP_DIR/mission_s194.py" "$FS_ROOT/mission_s194.py"

# ---- 4. launch sim stack --------------------------------------------
echo "[s194/$ITER_TAG] launching sim stack (world=$WORLD)"
bash "$LAUNCH_SIM" "$WORLD" "$WORKDIR" &
disown $! 2>/dev/null || true
sleep 10

# ---- 5. run mission in sentai_sim REPL ------------------------------
echo "[s194/$ITER_TAG] running mission in sentai_sim REPL"
REPL_LOG="$WORKDIR/sentai_repl.log"
echo "import mission_s194; r = mission_s194.run(); print('FINAL:', r.get('status','?'), 'phase:', r.get('phase_reached',-1), 'reject:', r.get('reject_name',''))" \
    | timeout 360 "$SENTAI_SIM" > "$REPL_LOG" 2>&1 || true

# ---- 6. snapshot artifacts ------------------------------------------
echo "[s194/$ITER_TAG] snapshotting artifacts"
[[ -f "$FS_ROOT/mission_s194_journal.txt" ]] \
    && cp "$FS_ROOT/mission_s194_journal.txt" "$WORKDIR/"
[[ -f "$FS_ROOT/mission_s194_summary.json" ]] \
    && cp "$FS_ROOT/mission_s194_summary.json" "$WORKDIR/"

# ---- 7. verdict -----------------------------------------------------
if [[ -x "$EXP_DIR/verdict.py" ]]; then
    echo "[s194/$ITER_TAG] running verdict"
    python3 "$EXP_DIR/verdict.py" "$WORKDIR" > "$WORKDIR/verdict.log" 2>&1 || true
    cat "$WORKDIR/verdict.log"
fi

# ---- 8. cleanup -----------------------------------------------------
echo "[s194/$ITER_TAG] cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
echo "[s194/$ITER_TAG] done. artefacts in $WORKDIR"

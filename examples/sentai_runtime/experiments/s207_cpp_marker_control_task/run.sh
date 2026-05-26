#!/bin/bash
# s207 - B5 C++ migration harness for B4 marker-control.

set -e

ITER_TAG="${1:-iter1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
EXP_DIR="$REPO_ROOT/examples/sentai_runtime/experiments/s207_cpp_marker_control_task"
ITER_DIR="$EXP_DIR/$ITER_TAG"
mkdir -p "$ITER_DIR"

WORKDIR="$ITER_DIR"
WORLD="${WORLD:-sentai_whycon_small}"
LAUNCH_SIM="$REPO_ROOT/sim/scripts/launch_sim.sh"
CLEANUP_SIM="$REPO_ROOT/sim/scripts/launch_sim_cleanup.sh"
SENTAI_SIM="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
CALIB_INI_SRC="${CALIB_INI_SRC:-$REPO_ROOT/examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/iter16_strict_calib_ini_e2e_b4/calib.ini}"

export SENTAI_FR_DIR="$WORKDIR/fr_current"

cleanup_on_exit() {
    echo "[s207/$ITER_TAG] trap: killing background processes"
    bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
}
trap cleanup_on_exit EXIT INT TERM

echo "[s207/$ITER_TAG] pre-cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
rm -rf "$WORKDIR/fr_current" "$WORKDIR/launch_sim.pids" "$WORKDIR"/*.log 2>/dev/null || true
mkdir -p "$WORKDIR/fr_current/frames"

echo "[s207/$ITER_TAG] building sentai_sim"
cmake --build "$REPO_ROOT/build-sim" --target sentai_sim 2>&1 | tail -3

echo "[s207/$ITER_TAG] staging mission_s207.py and calib.ini -> $FS_ROOT/"
mkdir -p "$FS_ROOT/system"
rm -f "$FS_ROOT/mission_s207.py" \
      "$FS_ROOT/mission_s207_status.txt" \
      "$FS_ROOT/system/calib.ini"
cp "$EXP_DIR/mission_s207.py" "$FS_ROOT/mission_s207.py"
cp "$CALIB_INI_SRC" "$FS_ROOT/system/calib.ini"

echo "[s207/$ITER_TAG] launching sim stack (world=$WORLD)"
bash "$LAUNCH_SIM" "$WORLD" "$WORKDIR" &
LAUNCH_PID=$!
wait "$LAUNCH_PID"

echo "[s207/$ITER_TAG] running mission in sentai_sim REPL"
REPL_LOG="$WORKDIR/sentai_repl.log"
echo "import mission_s207, sentai; r = mission_s207.run(); sentai.rtos.sleep_ms(200); sentai.fr.task_stop(); print('FINAL:', r.get('status','?'), 'phase:', r.get('phase','?'))" \
    | timeout 180 "$SENTAI_SIM" > "$REPL_LOG" 2>&1 || true

echo "[s207/$ITER_TAG] snapshotting artifacts"
[[ -f "$FS_ROOT/mission_s207_status.txt" ]] \
    && cp "$FS_ROOT/mission_s207_status.txt" "$WORKDIR/"
[[ -f "$FS_ROOT/system/calib.ini" ]] \
    && cp "$FS_ROOT/system/calib.ini" "$WORKDIR/calib.ini"

if [[ -x "$EXP_DIR/verdict_s207.py" ]]; then
    echo "[s207/$ITER_TAG] running post-mortem verdict"
    python3 "$EXP_DIR/verdict_s207.py" "$WORKDIR" > "$WORKDIR/verdict.log" 2>&1 || true
    cat "$WORKDIR/verdict.log"
fi

if [[ -x "$EXP_DIR/plot_s207.py" ]]; then
    echo "[s207/$ITER_TAG] running post-mortem plots"
    python3 "$EXP_DIR/plot_s207.py" "$WORKDIR" > "$WORKDIR/plots.log" 2>&1 || true
    cat "$WORKDIR/plots.log"
fi

echo "[s207/$ITER_TAG] cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
echo "[s207/$ITER_TAG] done. artifacts in $WORKDIR"

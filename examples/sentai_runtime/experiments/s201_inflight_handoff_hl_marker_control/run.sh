#!/bin/bash
# s201 - B4 minimal in-flight handoff smoke.

set -e

ITER_TAG="${1:-iter1_handoff_hover_smoke}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
EXP_DIR="$REPO_ROOT/examples/sentai_runtime/experiments/s201_inflight_handoff_hl_marker_control"
ITER_DIR="$EXP_DIR/$ITER_TAG"
mkdir -p "$ITER_DIR"

WORKDIR="$ITER_DIR"
WORLD="${WORLD:-sentai_whycon_small}"
LAUNCH_SIM="$REPO_ROOT/sim/scripts/launch_sim.sh"
CLEANUP_SIM="$REPO_ROOT/sim/scripts/launch_sim_cleanup.sh"
SENTAI_SIM="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
CALIB_JSON="${CALIB_JSON:-$REPO_ROOT/examples/sentai_runtime/experiments/s197_sota_calib_orientation_guarded/iter27_camera_landing_contract/mission_s197_calibration.json}"

export SENTAI_FR_DIR="$WORKDIR/fr_current"

cleanup_on_exit() {
    echo "[s201/$ITER_TAG] trap: killing background processes"
    bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
}
trap cleanup_on_exit EXIT INT TERM

echo "[s201/$ITER_TAG] pre-cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
rm -rf "$WORKDIR/fr_current" "$WORKDIR/launch_sim.pids" "$WORKDIR"/*.log 2>/dev/null || true
mkdir -p "$WORKDIR/fr_current/frames"

echo "[s201/$ITER_TAG] generating calib.ini from $CALIB_JSON"
python3 "$EXP_DIR/make_calib_ini.py" "$CALIB_JSON" "$WORKDIR/calib.ini"

echo "[s201/$ITER_TAG] building sentai_sim"
cmake --build "$REPO_ROOT/build-sim" --target sentai_sim 2>&1 | tail -5

echo "[s201/$ITER_TAG] staging mission_s201.py and calib.ini -> $FS_ROOT/"
mkdir -p "$FS_ROOT/system"
rm -f "$FS_ROOT/mission_s201.py" \
      "$FS_ROOT/mission_s201_journal.txt" \
      "$FS_ROOT/mission_s201_summary.json" \
      "$FS_ROOT/system/calib.ini"
cp "$EXP_DIR/mission_s201.py" "$FS_ROOT/mission_s201.py"
cp "$WORKDIR/calib.ini" "$FS_ROOT/system/calib.ini"

echo "[s201/$ITER_TAG] launching sim stack with GUI if launch_sim supports it (world=$WORLD)"
bash "$LAUNCH_SIM" "$WORLD" "$WORKDIR" &
disown $! 2>/dev/null || true
sleep 10

echo "[s201/$ITER_TAG] running mission in sentai_sim REPL"
REPL_LOG="$WORKDIR/sentai_repl.log"
( cd "$FS_ROOT" && \
  echo "import mission_s201; r = mission_s201.run(); print('FINAL:', r.get('status','?'), 'phase:', r.get('phase','?'))" \
    | timeout 240 "$SENTAI_SIM" > "$REPL_LOG" 2>&1 ) || true

echo "[s201/$ITER_TAG] snapshotting artifacts"
[[ -f "$FS_ROOT/mission_s201_journal.txt" ]] \
    && cp "$FS_ROOT/mission_s201_journal.txt" "$WORKDIR/"
[[ -f "$FS_ROOT/mission_s201_summary.json" ]] \
    && cp "$FS_ROOT/mission_s201_summary.json" "$WORKDIR/"
[[ -f "$FS_ROOT/system/calib.ini" ]] \
    && cp "$FS_ROOT/system/calib.ini" "$WORKDIR/calib_staged.ini"

if [[ -x "$EXP_DIR/verdict_s201.py" ]]; then
    echo "[s201/$ITER_TAG] running post-mortem verdict"
    python3 "$EXP_DIR/verdict_s201.py" "$WORKDIR" > "$WORKDIR/verdict.log" 2>&1 || true
    cat "$WORKDIR/verdict.log"
fi

echo "[s201/$ITER_TAG] cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
echo "[s201/$ITER_TAG] done. artifacts in $WORKDIR"

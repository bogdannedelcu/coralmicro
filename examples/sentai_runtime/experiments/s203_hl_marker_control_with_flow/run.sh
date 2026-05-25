#!/bin/bash
# s203 - B4 Generic image-frame marker control with sentai.flow assistance.

set -e

ITER_TAG="${1:-iter_generic_image_motion_smoke}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
EXP_DIR="$REPO_ROOT/examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow"
ITER_DIR="$EXP_DIR/$ITER_TAG"
mkdir -p "$ITER_DIR"

WORKDIR="$ITER_DIR"
WORLD="${WORLD:-sentai_whycon_small}"
LAUNCH_SIM="$REPO_ROOT/sim/scripts/launch_sim.sh"
CLEANUP_SIM="$REPO_ROOT/sim/scripts/launch_sim_cleanup.sh"
SENTAI_SIM="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
CALIB_INI="${CALIB_INI:-$FS_ROOT/system/calib.ini}"

export SENTAI_FR_DIR="$WORKDIR/fr_current"

cleanup_on_exit() {
    echo "[s203/$ITER_TAG] trap: killing background processes"
    bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
}
trap cleanup_on_exit EXIT INT TERM

echo "[s203/$ITER_TAG] pre-cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
rm -rf "$WORKDIR/fr_current" "$WORKDIR/launch_sim.pids" "$WORKDIR"/*.log 2>/dev/null || true
mkdir -p "$WORKDIR/fr_current/frames"

echo "[s203/$ITER_TAG] using calib.ini from $CALIB_INI"
if [[ ! -f "$CALIB_INI" ]]; then
    echo "[s203/$ITER_TAG] missing calib.ini; run accepted B3/s197 first or set CALIB_INI" >&2
    exit 2
fi
cp "$CALIB_INI" "$WORKDIR/calib.ini"

echo "[s203/$ITER_TAG] building sentai_sim"
cmake --build "$REPO_ROOT/build-sim" --target sentai_sim 2>&1 | tail -5

echo "[s203/$ITER_TAG] staging mission_s203.py and calib.ini -> $FS_ROOT/"
mkdir -p "$FS_ROOT/system"
rm -f "$FS_ROOT/mission_s203.py" \
      "$FS_ROOT/mission_s203_journal.txt" \
      "$FS_ROOT/mission_s203_summary.json"
cp "$EXP_DIR/mission_s203.py" "$FS_ROOT/mission_s203.py"
if [[ "$CALIB_INI" != "$FS_ROOT/system/calib.ini" ]]; then
    rm -f "$FS_ROOT/system/calib.ini"
    cp "$WORKDIR/calib.ini" "$FS_ROOT/system/calib.ini"
fi

echo "[s203/$ITER_TAG] launching sim stack with GUI if launch_sim supports it (world=$WORLD)"
bash "$LAUNCH_SIM" "$WORLD" "$WORKDIR" &
disown $! 2>/dev/null || true
sleep 10

echo "[s203/$ITER_TAG] running mission in sentai_sim REPL"
REPL_LOG="$WORKDIR/sentai_repl.log"
( cd "$FS_ROOT" && \
  echo "import mission_s203; r = mission_s203.run(); print('FINAL:', r.get('status','?'), 'phase:', r.get('phase','?'))" \
    | SENTAI_DUMP_FRAMES_DIR="$WORKDIR/fr_current/frames" \
      SENTAI_DUMP_FRAMES_EVERY="${SENTAI_DUMP_FRAMES_EVERY:-60}" \
      SENTAI_DUMP_RAW_EVERY="${SENTAI_DUMP_RAW_EVERY:-0}" \
      timeout 240 "$SENTAI_SIM" > "$REPL_LOG" 2>&1 ) || true

echo "[s203/$ITER_TAG] snapshotting artifacts"
[[ -f "$FS_ROOT/mission_s203_journal.txt" ]] \
    && cp "$FS_ROOT/mission_s203_journal.txt" "$WORKDIR/"
[[ -f "$FS_ROOT/mission_s203_summary.json" ]] \
    && cp "$FS_ROOT/mission_s203_summary.json" "$WORKDIR/"
[[ -f "$FS_ROOT/system/calib.ini" ]] \
    && cp "$FS_ROOT/system/calib.ini" "$WORKDIR/calib_staged.ini"

if [[ -x "$EXP_DIR/verdict_s203.py" ]]; then
    echo "[s203/$ITER_TAG] running post-mortem verdict"
    python3 "$EXP_DIR/verdict_s203.py" "$WORKDIR" > "$WORKDIR/verdict.log" 2>&1 || true
    cat "$WORKDIR/verdict.log"
fi

if [[ -x "$EXP_DIR/plot_s203.py" ]]; then
    echo "[s203/$ITER_TAG] generating post-mortem plots"
    python3 "$EXP_DIR/plot_s203.py" "$WORKDIR" > "$WORKDIR/plots.log" 2>&1 || true
    cat "$WORKDIR/plots.log"
fi

echo "[s203/$ITER_TAG] cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
echo "[s203/$ITER_TAG] done. artifacts in $WORKDIR"

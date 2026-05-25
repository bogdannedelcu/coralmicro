#!/bin/bash
# s199 - A3 calibration with post-hover visual axis-envelope survey.
#
# s194 is used only as launcher/journal/artifact precedent.  The mission logic
# in mission_s199.py intentionally avoids s194's failed control assumptions.

set -e

ITER_TAG="${1:-iter1}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
EXP_DIR="$REPO_ROOT/examples/sentai_runtime/experiments/s199_calib_envelope_survey"
ITER_DIR="$EXP_DIR/$ITER_TAG"
mkdir -p "$ITER_DIR"

WORKDIR="$ITER_DIR"
WORLD="${WORLD:-sentai_whycon_small}"
LAUNCH_SIM="$REPO_ROOT/sim/scripts/launch_sim.sh"
CLEANUP_SIM="$REPO_ROOT/sim/scripts/launch_sim_cleanup.sh"
SENTAI_SIM="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

export SENTAI_FR_DIR="$WORKDIR/fr_current"

cleanup_on_exit() {
    echo "[s199/$ITER_TAG] trap: killing background processes"
    bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
}
trap cleanup_on_exit EXIT INT TERM

echo "[s199/$ITER_TAG] pre-cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
rm -rf "$WORKDIR/fr_current" "$WORKDIR/launch_sim.pids" "$WORKDIR"/*.log 2>/dev/null || true
mkdir -p "$WORKDIR/fr_current/frames"

echo "[s199/$ITER_TAG] building sentai_sim"
cmake --build "$REPO_ROOT/build-sim" --target sentai_sim 2>&1 | tail -3

echo "[s199/$ITER_TAG] staging mission_s199.py -> $FS_ROOT/"
mkdir -p "$FS_ROOT"
rm -f "$FS_ROOT/mission_s199.py" \
      "$FS_ROOT/mission_s199_journal.txt" \
      "$FS_ROOT/mission_s199_summary.json" \
      "$FS_ROOT/mission_s199_calibration.json" \
      "$FS_ROOT/calib.ini"
cp "$EXP_DIR/mission_s199.py" "$FS_ROOT/mission_s199.py"

echo "[s199/$ITER_TAG] launching sim stack with GUI if launch_sim supports it (world=$WORLD)"
bash "$LAUNCH_SIM" "$WORLD" "$WORKDIR" &
disown $! 2>/dev/null || true
sleep 10

echo "[s199/$ITER_TAG] running mission in sentai_sim REPL"
REPL_LOG="$WORKDIR/sentai_repl.log"
echo "import mission_s199; r = mission_s199.run(); print('FINAL:', r.get('status','?'), 'phase:', r.get('phase','?'))" \
    | timeout 180 "$SENTAI_SIM" > "$REPL_LOG" 2>&1 || true

echo "[s199/$ITER_TAG] snapshotting artifacts"
[[ -f "$FS_ROOT/mission_s199_journal.txt" ]] \
    && cp "$FS_ROOT/mission_s199_journal.txt" "$WORKDIR/"
[[ -f "$FS_ROOT/mission_s199_summary.json" ]] \
    && cp "$FS_ROOT/mission_s199_summary.json" "$WORKDIR/"
[[ -f "$FS_ROOT/mission_s199_calibration.json" ]] \
    && cp "$FS_ROOT/mission_s199_calibration.json" "$WORKDIR/"
[[ -f "$FS_ROOT/calib.ini" ]] \
    && cp "$FS_ROOT/calib.ini" "$WORKDIR/"

if [[ -x "$EXP_DIR/verdict_s199.py" ]]; then
    echo "[s199/$ITER_TAG] running post-mortem verdict"
    python3 "$EXP_DIR/verdict_s199.py" "$WORKDIR" > "$WORKDIR/verdict.log" 2>&1 || true
    cat "$WORKDIR/verdict.log"
fi

echo "[s199/$ITER_TAG] cleanup"
bash "$CLEANUP_SIM" "$WORKDIR" 2>/dev/null || true
echo "[s199/$ITER_TAG] done. artifacts in $WORKDIR"

#!/bin/bash
# s163 — SlamTask end-to-end SIM smoke runner.
#
# Boots sentai_sim, kicks off t_slam_e2e.py over stdin REPL, and
# pushes synthetic frames into /tmp/sentai_cam.sock in parallel via
# inject_frames.py.  Verdict: 5/5 gates green.
#
# Usage: bash run.sh    (from repo root)
# Exit 0 on PASS, 1 on FAIL.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

mkdir -p "$FS_ROOT"
cp "$SCRIPT_DIR/t_slam_e2e.py" "$FS_ROOT/"

# Clean stale sockets so the bridge can re-bind.
rm -f /tmp/sentai_cam.sock

LOG="$SCRIPT_DIR/s163_run.log"
INJ="$SCRIPT_DIR/s163_inject.log"
rm -f "$LOG" "$INJ"

# Launch sentai_sim with the REPL `import` line on stdin.  Run in
# background so we can drive the injector in parallel.
(
    cd "$FS_ROOT"
    echo 'import t_slam_e2e' | "$SIM_BIN"
) >"$LOG" 2>&1 &
SIM_PID=$!

# Wait for the bridge to be listening on the UDS.
for i in $(seq 1 50); do
    if [ -S /tmp/sentai_cam.sock ]; then break; fi
    sleep 0.1
done
if [ ! -S /tmp/sentai_cam.sock ]; then
    echo "[s163] FAIL — bridge socket never appeared"
    kill -9 "$SIM_PID" 2>/dev/null || true
    exit 1
fi

# Push frames in parallel.  start_slam() inside MP runs almost
# immediately after sentai_sim boots, so by the time we get here the
# slot refcount is set and SLOT_RGB_64 will fire on the first frame.
python3 "$SCRIPT_DIR/inject_frames.py" >"$INJ" 2>&1 &
INJ_PID=$!

# Wait for sentai_sim to exit (it does after t_slam_e2e prints VERDICT).
wait "$SIM_PID" || true
wait "$INJ_PID" 2>/dev/null || true

echo "---- sentai_sim log ----"
cat "$LOG"
echo "---- inject log ----"
cat "$INJ"

if grep -q '\[VERDICT\] s163 slam_task SIM e2e: PASS' "$LOG"; then
    echo "[s163] PASS"
    exit 0
fi
echo "[s163] FAIL"
exit 1

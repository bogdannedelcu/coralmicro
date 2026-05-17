#!/bin/bash
# s160 — sentai.aruco altitude-sweep detection over the static
# aruco_4x4_50 quartet in sentai_crazysim world.
#
# Same FIFO-stdin pattern as s156: sentai_sim must be alive BEFORE
# the camera bridge connects, so we keep its stdin open via a named
# pipe and write the mission `import` line later.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
RESPAWN="$REPO_ROOT/sim/scripts/respawn_sitl.sh"
CRTP_LOG="$REPO_ROOT/build-sim/sentai_fs_root/crtp_log.py"
BRIDGE_BIN="$REPO_ROOT/build-sim/sim/gz_to_uds_bridge"
WORKDIR=/tmp/s160_aruco_alt_sweep
mkdir -p "$WORKDIR"

cleanup() {
    [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null || true
    [ -n "$KEEPER_PID" ] && kill -9 "$KEEPER_PID" 2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -f gz_to_uds_bridge 2>/dev/null || true
    rm -f "$WORKDIR/stdin.fifo"
}
trap cleanup EXIT

echo "[s160] respawning cf2 SITL at world origin"
bash "$RESPAWN" sentai_crazysim

echo "[s160] staging mission + crtp_log"
[ -f "$FS_ROOT/crtp_log.py" ] || \
    cp "$REPO_ROOT/examples/sentai_runtime/experiments/s146_pose_feedback/crtp_log.py" \
       "$FS_ROOT/crtp_log.py"
cp "$SCRIPT_DIR/mission_s160.py" "$FS_ROOT/"

# FIFO-stdin pattern.
FIFO="$WORKDIR/stdin.fifo"
rm -f "$FIFO"
mkfifo "$FIFO"
tail -f /dev/null > "$FIFO" &
KEEPER_PID=$!

"$SIM_BIN" < "$FIFO" > "$WORKDIR/sim.log" 2>&1 &
SIM_PID=$!
echo "[s160] sentai_sim PID=$SIM_PID; waiting for /tmp/sentai_cam.sock"

deadline=$(( $(date +%s) + 10 ))
until [ -S /tmp/sentai_cam.sock ]; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[s160] FAIL — sentai_sim never opened cam.sock; see $WORKDIR/sim.log"
        tail -20 "$WORKDIR/sim.log"
        exit 1
    fi
    sleep 0.2
done
echo "[s160] cam.sock ready; launching bridge"

distrobox enter crazysim-garden -- bash -c \
    "$BRIDGE_BIN --topic /downward_cam/image \
                 --in-sock /tmp/sentai_cam.sock \
                 --out-sock /tmp/sentai_flow_out.sock" \
    > "$WORKDIR/bridge.log" 2>&1 < /dev/null &
disown $! 2>/dev/null || true

sleep 3
if [ ! -S /tmp/sentai_flow_out.sock ]; then
    echo "[s160] FAIL — bridge did not open flow_out.sock"
    tail -10 "$WORKDIR/bridge.log"
    exit 1
fi
echo "[s160] bridge up; running mission_s160"

echo "import mission_s160; r = mission_s160.run(); print('FINAL:', r['status'])" > "$FIFO"

deadline=$(( $(date +%s) + 240 ))
until grep -q "^FINAL:" "$WORKDIR/sim.log" 2>/dev/null; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[s160] FAIL — mission did not complete in 240s"
        tail -40 "$WORKDIR/sim.log"
        exit 1
    fi
    sleep 1
done

echo "[s160] mission completed; verdict"
tail -5 "$WORKDIR/sim.log"
echo "---"
python3 "$SCRIPT_DIR/verdict.py" "$FS_ROOT/mission_s160_summary.json"

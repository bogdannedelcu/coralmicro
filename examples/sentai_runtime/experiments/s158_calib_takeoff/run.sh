#!/bin/bash
# s158 — OP-S6-W1-T7: mission_s153 + takeoff Kabsch calibration.
# First live-Gazebo run with sentai.calib + sentai.aruco end-to-end.
# Same FIFO-stdin pattern as s156/s160 so the camera bridge can connect
# to sentai_sim before the mission starts.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
RESPAWN="$REPO_ROOT/sim/scripts/respawn_sitl.sh"
BRIDGE_BIN="$REPO_ROOT/build-sim/sim/gz_to_uds_bridge"
WORKDIR=/tmp/s158_calib_takeoff
mkdir -p "$WORKDIR"

cleanup() {
    [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null || true
    [ -n "$KEEPER_PID" ] && kill -9 "$KEEPER_PID" 2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -f gz_to_uds_bridge 2>/dev/null || true
    rm -f "$WORKDIR/stdin.fifo"
}
trap cleanup EXIT

echo "[s158] respawning cf2 SITL at world origin"
bash "$RESPAWN" sentai_crazysim

echo "[s158] staging mission"
cp "$SCRIPT_DIR/mission_s158.py" "$FS_ROOT/"
# Drop any stale calib file so we observe a true fresh-calibration run.
rm -f "$FS_ROOT/cam_calib.json"

FIFO="$WORKDIR/stdin.fifo"
rm -f "$FIFO"
mkfifo "$FIFO"
tail -f /dev/null > "$FIFO" &
KEEPER_PID=$!

# Launch sentai_sim FROM fs_root so the host-stdio fallback in
# sentai_calib_save writes cam_calib.json into the staging area
# (verdict.py looks for it there).
( cd "$FS_ROOT" && exec "$SIM_BIN" < "$FIFO" > "$WORKDIR/sim.log" 2>&1 ) &
SIM_PID=$!
echo "[s158] sentai_sim PID=$SIM_PID; waiting for cam.sock"

deadline=$(( $(date +%s) + 10 ))
until [ -S /tmp/sentai_cam.sock ]; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[s158] FAIL — sentai_sim never opened cam.sock"
        tail -20 "$WORKDIR/sim.log"
        exit 1
    fi
    sleep 0.2
done
echo "[s158] cam.sock ready; launching bridge"

distrobox enter crazysim-garden -- bash -c \
    "$BRIDGE_BIN --topic /downward_cam/image \
                 --in-sock /tmp/sentai_cam.sock \
                 --out-sock /tmp/sentai_flow_out.sock" \
    > "$WORKDIR/bridge.log" 2>&1 < /dev/null &
disown $! 2>/dev/null || true
sleep 3
if [ ! -S /tmp/sentai_flow_out.sock ]; then
    echo "[s158] FAIL — bridge did not open flow_out.sock"
    tail -10 "$WORKDIR/bridge.log"
    exit 1
fi
echo "[s158] bridge up; running mission_s158"

echo "import mission_s158; r = mission_s158.run(); print('FINAL:', r['status'])" > "$FIFO"

deadline=$(( $(date +%s) + 240 ))
until grep -q "^FINAL:" "$WORKDIR/sim.log" 2>/dev/null; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[s158] FAIL — mission did not complete in 240s"
        tail -40 "$WORKDIR/sim.log"
        exit 1
    fi
    sleep 1
done

echo "[s158] mission completed; verdict"
tail -3 "$WORKDIR/sim.log"
echo "---"
python3 "$SCRIPT_DIR/verdict.py" "$FS_ROOT/mission_s158_summary.json"

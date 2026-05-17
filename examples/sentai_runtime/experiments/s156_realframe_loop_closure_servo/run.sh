#!/bin/bash
# s156 — Real-frame loop closure on sentai.servo paradigm.
#
# Orchestration pattern: sentai_sim must be alive BEFORE the camera bridge
# can connect (bridge does `connect(2)` on /tmp/sentai_cam.sock with a 15s
# retry window; once sentai_sim exits the socket dies and the bridge dies
# on SIGPIPE — see [[s156-camera-bridge-explained]]).  Using a FIFO as
# sentai_sim's stdin keeps the process alive between commands.
#
# Pipeline (camera frames just flow into sentai_sim's internal buffer; the
# mission calls grab_gray() to read the latest — no per-call handshake):
#
#   gz /downward_cam/image
#       ↓
#   gz_to_uds_bridge (distrobox, connect-with-retry)
#       ↓  /tmp/sentai_cam.sock (UDS, SCM1 protocol)
#       ↓
#   camera_bridge_recv (FreeRTOS task in sentai_sim)
#       ↓  s_grab_rgb_src buffer
#       ↓
#   sentai.camera.grab_gray()  ← mission_s156 reads here

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
RESPAWN="$REPO_ROOT/sim/scripts/respawn_sitl.sh"
HEX_HELPERS="$REPO_ROOT/examples/sentai_runtime/experiments/s142_hex_descriptor_patrol/hex_helpers.py"
BRIDGE_BIN="$REPO_ROOT/build-sim/sim/gz_to_uds_bridge"
WORKDIR=/tmp/s156_realframe_loop_closure_servo
mkdir -p "$WORKDIR"

cleanup() {
    [ -n "$SIM_PID" ] && kill -9 "$SIM_PID" 2>/dev/null || true
    [ -n "$KEEPER_PID" ] && kill -9 "$KEEPER_PID" 2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -f gz_to_uds_bridge 2>/dev/null || true
    rm -f "$WORKDIR/stdin.fifo"
}
trap cleanup EXIT

echo "[s156] respawning cf2 SITL at world origin"
bash "$RESPAWN" sentai_crazysim

echo "[s156] staging mission + hex_helpers"
cp "$HEX_HELPERS" "$FS_ROOT/"
cp "$SCRIPT_DIR/mission_s156.py" "$FS_ROOT/"

# FIFO-stdin pattern so sentai_sim stays alive between commands
FIFO="$WORKDIR/stdin.fifo"
rm -f "$FIFO"
mkfifo "$FIFO"

# Keep FIFO open (tail -f on /dev/null wedges its write end open so
# the first echo doesn't block waiting for a reader)
tail -f /dev/null > "$FIFO" &
KEEPER_PID=$!

# Launch sentai_sim reading from FIFO; redirect output to a log
"$SIM_BIN" < "$FIFO" > "$WORKDIR/sim.log" 2>&1 &
SIM_PID=$!
echo "[s156] sentai_sim PID=$SIM_PID, waiting for /tmp/sentai_cam.sock"

deadline=$(( $(date +%s) + 10 ))
until [ -S /tmp/sentai_cam.sock ]; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[s156] FAIL — sentai_sim never opened cam.sock; see $WORKDIR/sim.log"
        tail -20 "$WORKDIR/sim.log"
        exit 1
    fi
    sleep 0.2
done
echo "[s156] cam.sock ready; launching bridge"

distrobox enter crazysim-garden -- bash -c \
    "$BRIDGE_BIN --topic /downward_cam/image \
                 --in-sock /tmp/sentai_cam.sock \
                 --out-sock /tmp/sentai_flow_out.sock" \
    > "$WORKDIR/bridge.log" 2>&1 < /dev/null &
disown $! 2>/dev/null || true

# Bridge takes ~1-2s to subscribe and start forwarding frames
sleep 3
if [ ! -S /tmp/sentai_flow_out.sock ]; then
    echo "[s156] FAIL — bridge did not open flow_out.sock"
    tail -10 "$WORKDIR/bridge.log"
    exit 1
fi
echo "[s156] bridge up; running mission_s156"

echo "import mission_s156; r = mission_s156.run(); print('FINAL:', r['status'])" > "$FIFO"

# Wait for mission completion — look for FINAL: in sim.log
deadline=$(( $(date +%s) + 180 ))
until grep -q "^FINAL:" "$WORKDIR/sim.log" 2>/dev/null; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[s156] FAIL — mission did not complete in 180s"
        tail -30 "$WORKDIR/sim.log"
        exit 1
    fi
    sleep 1
done

echo "[s156] mission completed; verdict"
tail -5 "$WORKDIR/sim.log"
echo "---"
python3 "$SCRIPT_DIR/verdict.py"

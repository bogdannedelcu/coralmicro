#!/bin/bash
# s165 — 1m square drift with Gazebo GT comparison (OP-S10-W11-T5.A v2).
#
# Same mission as s164.  Adds:
#   - parallel gt_recorder.py subscribing to /world/.../dynamic_pose/info
#     and writing /tmp/s165_square_drift_gt/gt_poses.jsonl with
#     host-monotonic timestamps
#   - verdict.py joins EKF + GT timelines and emits a comparison plot.
#
# Anti-cheat: GT consumed HOST-SIDE ONLY (verdict-time, post-mortem)
# per [[sentai-sim-air-gapped-from-truth]].

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s165_square_drift_gt
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
MISSION="$SCRIPT_DIR/mission_square.py"
GT_RECORDER="$SCRIPT_DIR/gt_recorder.py"
VERDICT="$SCRIPT_DIR/verdict.py"
STOP_SH="$REPO_ROOT/examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh"

WORLD=sentai_crazysim

is_cf2_up() { ss -lun 2>/dev/null | grep -q ":19850"; }

respawn_cf2_at_origin() {
    if is_cf2_up; then
        echo "[s165] tearing down running SITL so cf2 respawns at origin"
        bash "$STOP_SH" > /dev/null 2>&1 || true
        for p in $(pgrep -f "build-sim/sim/sentai_sim$|gz_to_uds_bridge|sitl_make/build/cf2|gz sim|gt_recorder"); do
            kill -9 "$p" 2>/dev/null || true
        done
        rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock 2>/dev/null
        sleep 1
    fi
}

ensure_sitl_up() {
    if is_cf2_up; then
        echo "[s165] cf2 SITL up on UDP 19850"
        return 0
    fi
    echo "[s165] launching SITL stack via $LAUNCH_HYBRID"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
        > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s165] FAIL — SITL never came up"
            return 1
        fi
        sleep 2
    done
    echo "[s165] cf2 UDP 19850 ready"
}

bridge_alive() {
    [ -S /tmp/sentai_flow_out.sock ] && \
        python3 -c "
import socket, sys
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(0.5)
try: s.connect('/tmp/sentai_flow_out.sock'); s.close(); sys.exit(0)
except: sys.exit(1)" 2>/dev/null
}

ensure_bridge_after_sim() {
    local deadline=$(( $(date +%s) + 30 ))
    until [ -S /tmp/sentai_cam.sock ]; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s165] FAIL — sentai_sim never opened cam.sock"
            return 1
        fi
        sleep 0.3
    done
    if bridge_alive; then
        echo "[s165] gz_to_uds_bridge already up"
        return 0
    fi
    [ -S /tmp/sentai_flow_out.sock ] && rm -f /tmp/sentai_flow_out.sock
    echo "[s165] launching gz_to_uds_bridge"
    distrobox enter crazysim-garden -- bash -c \
        "$REPO_ROOT/build-sim/sim/gz_to_uds_bridge \
            --cam-topic /downward_cam/image \
            --cam-sock /tmp/sentai_cam.sock \
            --out-sock /tmp/sentai_flow_out.sock" \
        > "$WORKDIR/bridge.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    local d2=$(( $(date +%s) + 15 ))
    until [ -S /tmp/sentai_flow_out.sock ]; do
        if [ $(date +%s) -ge $d2 ]; then
            echo "[s165] FAIL — bridge did not open flow_out.sock"
            return 1
        fi
        sleep 0.3
    done
    echo "[s165] flow_out.sock ready"
}

start_gt_recorder() {
    rm -f "$WORKDIR/gt_poses.jsonl"
    echo "[s165] starting GT recorder (host-side, anti-cheat compliant)"
    S165_GT_OUT="$WORKDIR/gt_poses.jsonl" "$VENV_PY" "$GT_RECORDER" \
        > "$WORKDIR/gt_recorder.log" 2>&1 &
    GT_PID=$!
    echo "[s165] GT recorder PID=$GT_PID"
    # Give it a moment to subscribe and start logging.
    sleep 1.5
}

stop_gt_recorder() {
    if [ -n "${GT_PID:-}" ] && kill -0 "$GT_PID" 2>/dev/null; then
        kill -TERM "$GT_PID" 2>/dev/null || true
        wait "$GT_PID" 2>/dev/null || true
    fi
    # Belt-and-suspenders: kill any leftover gt_recorder + the gz topic
    # subprocess inside distrobox.
    pkill -f "gt_recorder.py" 2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -f "gz topic -e -t /world/${WORLD}/dynamic_pose" 2>/dev/null || true
}

run_mission() {
    echo "[s165] running mission_square.py"
    "$VENV_PY" "$MISSION" > "$WORKDIR/mission_host.log" 2>&1
}

verdict() {
    "$VENV_PY" "$VERDICT"
}

# ─── Main ───
respawn_cf2_at_origin
ensure_sitl_up        || exit 1
ensure_bridge_after_sim &
BRIDGE_WAIT_PID=$!

start_gt_recorder

run_mission && mission_rc=0 || mission_rc=$?
wait "$BRIDGE_WAIT_PID" 2>/dev/null || true
stop_gt_recorder

if [ "$mission_rc" -ne 0 ]; then
    echo "[s165] mission rc=$mission_rc — see $WORKDIR/mission_host.log"
fi
verdict && rc=0 || rc=$?
echo "[s165] exit code $rc"
exit $rc

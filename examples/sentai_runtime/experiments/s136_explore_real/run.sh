#!/bin/bash
# s135 — L6 sentai.explore RELEVANT exploration test (behavior-gated).
#
# Replaces s135 (closure-only).  Per [[sim-test-must-return-home]]:
# every hard gate (approach motion, return motion, RETURNING entry,
# lifter convergence, closure) must PASS — not just FSM shape.
# Force-respawns cf2 at origin per [[experiments-start-from-origin]].
#
# Usage:   bash run.sh
# Exit 0 on PASS, 1 on FAIL.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s136_explore_real
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
MISSION="$SCRIPT_DIR/mission_explore.py"
VERDICT="$SCRIPT_DIR/verdict.py"
STOP_SH="$REPO_ROOT/examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh"

WORLD=sentai_crazysim    # same world as s127/s128/s129/s130/s132

is_cf2_up() { ss -lun 2>/dev/null | grep -q ":19850"; }

respawn_cf2_at_origin() {
    if [ "${S136_NO_RESPAWN:-0}" = "1" ]; then
        echo "[s136] S136_NO_RESPAWN=1 — skipping respawn (NOT reproducible!)"
        return 0
    fi
    if is_cf2_up; then
        echo "[s136] tearing down running SITL so cf2 respawns at origin"
        bash "$STOP_SH" > /dev/null 2>&1 || true
        for p in $(pgrep -f "build-sim/sim/sentai_sim$|gz_to_uds_bridge|sitl_make/build/cf2|gz sim"); do
            kill -9 "$p" 2>/dev/null || true
        done
        rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock 2>/dev/null
        sleep 1
    fi
}

ensure_sitl_up() {
    if is_cf2_up; then
        echo "[s136] cf2 SITL up on UDP 19850"
        return 0
    fi
    echo "[s136] launching SITL stack via $LAUNCH_HYBRID"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
        > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s136] FAIL — SITL never came up; see $WORKDIR/sitl.log"
            return 1
        fi
        sleep 2
    done
    echo "[s136] cf2 UDP 19850 ready"
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
            echo "[s136] FAIL — sentai_sim never opened cam.sock"
            return 1
        fi
        sleep 0.3
    done
    if bridge_alive; then
        echo "[s136] gz_to_uds_bridge already up + accepting"
        return 0
    fi
    if [ -S /tmp/sentai_flow_out.sock ]; then
        echo "[s136] stale flow_out.sock (bridge dead); removing"
        rm -f /tmp/sentai_flow_out.sock
    fi
    echo "[s136] launching gz_to_uds_bridge inside distrobox"
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
            echo "[s136] FAIL — bridge did not open flow_out.sock"
            return 1
        fi
        sleep 0.3
    done
    echo "[s136] flow_out.sock ready"
}

run_mission() {
    echo "[s136] running mission_explore.py (origin-locked closure)"
    "$VENV_PY" "$MISSION" > "$WORKDIR/mission_host.log" 2>&1
}

verdict() {
    "$VENV_PY" "$VERDICT"
}

# ─── Main ───
respawn_cf2_at_origin
ensure_sitl_up         || exit 1
ensure_bridge_after_sim &
BRIDGE_WAIT_PID=$!

run_mission && mission_rc=0 || mission_rc=$?
wait "$BRIDGE_WAIT_PID" 2>/dev/null || true

if [ "$mission_rc" -ne 0 ]; then
    echo "[s136] mission_explore.py rc=$mission_rc — see $WORKDIR/mission_host.log + repl.transcript"
fi
verdict && rc=0 || rc=$?
echo "[s136] exit code $rc"
exit $rc

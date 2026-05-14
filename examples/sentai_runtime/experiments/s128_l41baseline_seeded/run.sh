#!/bin/bash
# s128 L4.1Baseline runner — pre-seeded marker tour gate.
#
# Differences vs s127/run.sh:
#   - sentai_sim is NOT spawned by this script.  mission_l41.py spawns
#     it itself (with bidirectional pipes) so it can drive the REPL.
#   - If a pre-existing sentai_sim is hogging /tmp/sentai_cam.sock, we
#     kill it so mission_l41.py can start a fresh one with clean state.
#
# Usage:   bash run.sh
# Exit 0 on PASS, 1 on FAIL.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s128_l41baseline
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
MISSION="$SCRIPT_DIR/mission_l41.py"
VERDICT="$SCRIPT_DIR/verdict.py"

WORLD=sentai_crazysim

is_cf2_up() { ss -lun 2>/dev/null | grep -q ":19850"; }

ensure_sitl_up() {
    if is_cf2_up; then
        echo "[s128] cf2 SITL already up on UDP 19850"
        return 0
    fi
    echo "[s128] launching SITL stack via $LAUNCH_HYBRID"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
        > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    echo "[s128] waiting for UDP 19850..."
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s128] FAIL — SITL never came up; see $WORKDIR/sitl.log"
            return 1
        fi
        sleep 2
    done
    echo "[s128] cf2 UDP 19850 ready"
}

kill_existing_sim() {
    # mission_l41.py wants to spawn its own sentai_sim with PIPEs.  Any
    # pre-existing instance (left over from s127 or an earlier s128 run)
    # is holding /tmp/sentai_cam.sock and will conflict.
    local PIDS
    PIDS=$(pgrep -f "build-sim/sim/sentai_sim$" || true)
    if [ -n "$PIDS" ]; then
        echo "[s128] killing stale sentai_sim PIDs: $PIDS"
        for p in $PIDS; do kill -TERM "$p" 2>/dev/null || true; done
        sleep 1
        for p in $PIDS; do kill -KILL "$p" 2>/dev/null || true; done
    fi
    rm -f /tmp/sentai_cam.sock
}

ensure_bridge_up() {
    # gz_to_uds_bridge dies silently on SIGPIPE when its flow_out consumer
    # disconnects.  Probe the listener for liveness instead of stat'ing
    # the socket file.
    if [ -S /tmp/sentai_flow_out.sock ] && \
       python3 -c "
import socket, sys
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(0.5)
try: s.connect('/tmp/sentai_flow_out.sock'); s.close(); sys.exit(0)
except: sys.exit(1)" 2>/dev/null; then
        echo "[s128] gz_to_uds_bridge already up + accepting"
        return 0
    fi
    if [ -S /tmp/sentai_flow_out.sock ]; then
        echo "[s128] stale flow_out.sock (bridge dead); removing"
        rm -f /tmp/sentai_flow_out.sock
    fi
    # NOTE: gz_to_uds_bridge will connect to /tmp/sentai_cam.sock which
    # we just deleted.  mission_l41.py spawns sentai_sim FIRST and we
    # must give it time to open the listener before launching the bridge.
    # mission_l41.py blocks on REPL startup, but the cam socket appears
    # shortly after — give a small grace window.
    echo "[s128] (bridge launch deferred — mission_l41 spawns sim first)"
}

run_mission() {
    echo "[s128] running mission_l41.py (will spawn sentai_sim + fly cf2)"
    # Tee the host script's own stderr/stdout to a log so the orchestrator
    # output is also captured for post-mortem.  Exit status of the
    # mission decides whether verdict is even attempted, but verdict
    # also re-checks summary.json so a partial run is still inspectable.
    "$VENV_PY" "$MISSION" > "$WORKDIR/mission_host.log" 2>&1
}

ensure_bridge_after_sim() {
    # Bring the bridge up AFTER mission_l41 has spawned sentai_sim (the
    # cam.sock listener).  mission_l41 itself blocks on REPL startup
    # which keeps the listener alive; if mission_l41 finished cleanly
    # this is a no-op (bridge had nothing to attach to anyway).
    local deadline=$(( $(date +%s) + 30 ))
    until [ -S /tmp/sentai_cam.sock ]; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s128] FAIL — sentai_sim never opened cam.sock"
            return 1
        fi
        sleep 0.3
    done
    if [ ! -S /tmp/sentai_flow_out.sock ]; then
        echo "[s128] launching gz_to_uds_bridge inside distrobox"
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
                echo "[s128] FAIL — bridge did not open flow_out.sock"
                return 1
            fi
            sleep 0.3
        done
    fi
    echo "[s128] flow_out.sock ready"
}

verdict() {
    "$VENV_PY" "$VERDICT"
}

# ─── Main orchestration ───
ensure_sitl_up         || exit 1
kill_existing_sim
ensure_bridge_up       || true   # informational; may need post-sim launch
# mission_l41.py spawns sentai_sim; we must launch the bridge in
# parallel as soon as cam.sock appears (the bridge attaches to it).
ensure_bridge_after_sim &
BRIDGE_WAIT_PID=$!

run_mission && mission_rc=0 || mission_rc=$?
wait "$BRIDGE_WAIT_PID" 2>/dev/null || true

if [ "$mission_rc" -ne 0 ]; then
    echo "[s128] mission_l41.py exited rc=$mission_rc — see $WORKDIR/mission.log + repl.transcript"
fi
verdict && rc=0 || rc=$?
echo "[s128] exit code $rc"
exit $rc

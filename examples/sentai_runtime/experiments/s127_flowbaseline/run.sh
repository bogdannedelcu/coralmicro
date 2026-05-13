#!/bin/bash
# s127 FlowBaseline runner — proves sentai.flow + cf2 SITL hover hasn't
# regressed.  See ../s127_flowbaseline/README.md for context.
#
# Usage:    bash run.sh
# Exit 0 on PASS, 1 on FAIL.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s127_flowbaseline
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
ARUCO_HOVER="$REPO_ROOT/examples/sentai_runtime/experiments/s091_aruco_lowalt/aruco_hover.py"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"

WORLD=sentai_crazysim

cleanup_stale_sockets() { rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock; }

is_cf2_up() { ss -lun 2>/dev/null | grep -q ":19850"; }

ensure_sitl_up() {
    if is_cf2_up; then
        echo "[s127] cf2 SITL already up on UDP 19850"
        return 0
    fi
    echo "[s127] launching SITL stack via $LAUNCH_HYBRID"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
        > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    echo "[s127] waiting for UDP 19850..."
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s127] FAIL — SITL never came up; see $WORKDIR/sitl.log"
            return 1
        fi
        sleep 2
    done
    echo "[s127] cf2 UDP 19850 ready"
}

ensure_sentai_sim_up() {
    if [ -S /tmp/sentai_cam.sock ]; then
        # Verify the listener is alive (otherwise the file is stale)
        local SIM_PID
        SIM_PID=$(pgrep -f "build-sim/sim/sentai_sim$" | head -1)
        if [ -z "$SIM_PID" ]; then
            echo "[s127] stale sentai_cam.sock, replacing"
            cleanup_stale_sockets
        else
            echo "[s127] sentai_sim already up (PID=$SIM_PID)"
            # Reuse its SENTAI_DUMP_FRAMES_DIR for aruco_hover.
            local EXISTING_DIR
            EXISTING_DIR=$(cat "/proc/$SIM_PID/environ" 2>/dev/null \
                          | tr '\0' '\n' | grep '^SENTAI_DUMP_FRAMES_DIR=' \
                          | head -1 | cut -d= -f2-)
            if [ -n "$EXISTING_DIR" ] && [ -d "$EXISTING_DIR" ]; then
                echo "$EXISTING_DIR" > "$WORKDIR/frames_dir"
                echo "[s127] reusing frames dir: $EXISTING_DIR"
            else
                echo "[s127] WARN — running sentai_sim has no SENTAI_DUMP_FRAMES_DIR"
                # Fall back to a fresh dir aruco_hover can write metadata into
                # (sentai_sim won't dump frames there, but PnP can still run
                # on its in-memory shared buffer via /tmp/sentai_flow_out.sock).
                local FRESH="$WORKDIR/sentai_frames_$(date +%Y%m%d_%H%M%S)"
                mkdir -p "$FRESH"
                echo "$FRESH" > "$WORKDIR/frames_dir"
            fi
            return 0
        fi
    fi
    local FIFO=/tmp/sentai_sim_stdin.fifo
    [ -p "$FIFO" ] || mkfifo "$FIFO"
    # Keep stdin open (otherwise REPL EOFs and sentai_sim exits).
    # Detach from the launcher's pipe (stdin from /dev/null, stderr to
    # /dev/null) so this background process doesn't keep the parent
    # script's stdout pipe open when the launcher exits.
    sleep infinity > "$FIFO" 2>/dev/null < /dev/null &
    disown $! 2>/dev/null || true
    local FRAMES_DIR="$WORKDIR/sentai_frames_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$FRAMES_DIR"
    echo "$FRAMES_DIR" > "$WORKDIR/frames_dir"
    echo "[s127] sentai_sim frames → $FRAMES_DIR"
    env SENTAI_DUMP_FRAMES_DIR="$FRAMES_DIR" \
        SENTAI_DUMP_FRAMES_EVERY=15 \
        SENTAI_DUMP_RAW_EVERY=15 \
        SENTAI_SIM_ROOT="$REPO_ROOT/build-sim/sentai_fs_root" \
        "$REPO_ROOT/build-sim/sim/sentai_sim" < "$FIFO" \
        > "$WORKDIR/sim.log" 2>&1 &
    local SIM_PID=$!
    disown $SIM_PID 2>/dev/null || true
    echo $SIM_PID > "$WORKDIR/sim.pid"
    local deadline=$(( $(date +%s) + 15 ))
    until [ -S /tmp/sentai_cam.sock ]; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s127] FAIL — sentai_sim did not open /tmp/sentai_cam.sock"
            return 1
        fi
        sleep 1
    done
    echo "[s127] sentai_sim PID=$SIM_PID, cam.sock ready"
}

ensure_bridge_up() {
    # gz_to_uds_bridge dies silently on SIGPIPE when its flow_out consumer
    # disconnects (e.g., aruco_hover exiting normally).  The sock file
    # persists even after death, so "stat" + pgrep aren't reliable —
    # actually probe the listener via a tiny connect() test.
    if [ -S /tmp/sentai_flow_out.sock ] && \
       python3 -c "
import socket, sys
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(0.5)
try: s.connect('/tmp/sentai_flow_out.sock'); s.close(); sys.exit(0)
except: sys.exit(1)" 2>/dev/null; then
        echo "[s127] gz_to_uds_bridge already up + accepting"
        return 0
    fi
    if [ -S /tmp/sentai_flow_out.sock ]; then
        echo "[s127] stale flow_out.sock (bridge dead); removing"
        rm -f /tmp/sentai_flow_out.sock
    fi
    echo "[s127] launching gz_to_uds_bridge inside distrobox"
    distrobox enter crazysim-garden -- bash -c \
        "$REPO_ROOT/build-sim/sim/gz_to_uds_bridge \
            --cam-topic /downward_cam/image \
            --cam-sock /tmp/sentai_cam.sock \
            --out-sock /tmp/sentai_flow_out.sock" \
        > "$WORKDIR/bridge.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 15 ))
    until [ -S /tmp/sentai_flow_out.sock ]; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s127] FAIL — gz_to_uds_bridge did not open flow_out.sock"
            return 1
        fi
        sleep 1
    done
    echo "[s127] flow_out.sock ready"
}

run_hover() {
    local FRAMES_DIR
    FRAMES_DIR=$(cat "$WORKDIR/frames_dir")
    echo "[s127] running aruco_hover.py (15s hover @ z=1m)"
    env SENTAI_DUMP_FRAMES_DIR="$FRAMES_DIR" "$VENV_PY" "$ARUCO_HOVER" \
        > "$WORKDIR/hover.log" 2>&1
}

verdict() {
    "$VENV_PY" "$SCRIPT_DIR/verdict.py"
}

# Main
ensure_sitl_up         || exit 1
ensure_sentai_sim_up   || exit 1
ensure_bridge_up       || exit 1
run_hover              || true       # script may set non-zero on link issue; let verdict decide
verdict                && rc=0 || rc=$?
echo "[s127] exit code $rc"
exit $rc

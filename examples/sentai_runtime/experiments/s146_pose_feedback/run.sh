#!/bin/bash
# s146 — Pose feedback (CRTP LOG, pure MP) runner.
#
# --offline (default):  Wire-format unit tests, no SITL.
# --live:               Full Gazebo + cf2 SITL + crtp_log live test.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
MODE="${1:---offline}"

SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
STOP_SH="$REPO_ROOT/examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh"

WORKDIR="/tmp/s146_pose_feedback"
mkdir -p "$WORKDIR"

stage() {
    echo "[s146] staging $* → $FS_ROOT/"
    for f in "$@"; do
        cp "$SCRIPT_DIR/$f" "$FS_ROOT/"
    done
}

is_cf2_up() { ss -lun 2>/dev/null | grep -q ":19850"; }

ensure_sitl_up() {
    if is_cf2_up; then
        echo "[s146] cf2 SITL already up"
        return 0
    fi
    echo "[s146] launching SITL stack"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" sentai_crazysim \
        > "$WORKDIR/sitl.log" 2>&1 &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s146] FAIL — SITL never came up; see $WORKDIR/sitl.log"
            return 1
        fi
        sleep 2
    done
    echo "[s146] cf2 UDP 19850 ready"
}

case "$MODE" in
    --offline)
        echo "[s146] mode = OFFLINE (wire-format unit tests)"
        stage crtp_log.py t_crtp_log_offline.py
        echo "import t_crtp_log_offline" | timeout 30 "$SIM_BIN" 2>&1 | tee "$WORKDIR/offline.log"
        if grep -q "t_crtp_log_offline PASS" "$WORKDIR/offline.log"; then
            echo "[s146] OFFLINE PASS"
            exit 0
        fi
        echo "[s146] OFFLINE FAIL — see $WORKDIR/offline.log"
        exit 1
        ;;
    --live)
        echo "[s146] mode = LIVE (Gazebo + cf2 SITL)"
        ensure_sitl_up || exit 1
        stage crtp_log.py t_crtp_log_live.py
        echo "import t_crtp_log_live" | timeout 60 "$SIM_BIN" 2>&1 | tee "$WORKDIR/live.log"
        if grep -q "t_crtp_log_live PASS" "$WORKDIR/live.log"; then
            echo "[s146] LIVE PASS"
            exit 0
        fi
        echo "[s146] LIVE FAIL — see $WORKDIR/live.log"
        exit 1
        ;;
    *)
        echo "Usage: $0 [--offline|--live]"
        exit 2
        ;;
esac

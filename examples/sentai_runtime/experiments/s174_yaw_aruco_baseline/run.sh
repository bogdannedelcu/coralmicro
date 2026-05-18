#!/bin/bash
# s172 — Flow loop autotune baseline (OP-S10-W14-T6).
#
# Mission runs INSIDE sentai_sim MP.  Host side only:
#   1. brings up SITL + bridge + GT recorder
#   2. stages the MP file into sentai_fs_root/
#   3. triggers `import mission_yaw_aruco; mission_yaw_aruco.run()`
#   4. runs verdict (PASS/FAIL on Kp range + landing distance)
#
# HARD RULE [[missions-run-in-sentai-only]] preserved — NO cflib /
# autotune logic on host.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s174_yaw_aruco_baseline
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
MISSION_FILE="$SCRIPT_DIR/mission_yaw_aruco.py"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
STOP_SH="$REPO_ROOT/examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh"
GT_RECORDER="$REPO_ROOT/sim/scripts/gt_recorder.py"
VERDICT="$SCRIPT_DIR/verdict.py"
WORLD=sentai_crazysim

MISSION_TIMEOUT_S=50          # autotune dur ~30s + takeoff+settle+land ~10s + slack

is_cf2_up() { ss -lun 2>/dev/null | grep -q ":19850"; }

respawn_from_origin() {
    echo "[s174] unconditional teardown"
    bash "$STOP_SH" > "$WORKDIR/stop.log" 2>&1 || true
    for p in $(pgrep -f "build-sim/sim/sentai_sim$|gz_to_uds_bridge|sitl_make/build/cf2|gz sim|gt_recorder"); do
        kill -9 "$p" 2>/dev/null || true
    done
    distrobox enter crazysim-garden -- pkill -9 -f "gz topic -e -t /world/${WORLD}/dynamic_pose" 2>/dev/null || true
    rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock /tmp/sentai_sim_stdin.fifo 2>/dev/null
    sleep 1.5
}

ensure_sitl_up() {
    echo "[s174] launching SITL stack"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" > "$WORKDIR/sitl.log" 2>&1 &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s174] FAIL — SITL never came up"; return 1
        fi
        sleep 2
    done
    echo "[s174] cf2 UDP 19850 ready"
}

ensure_bridge_up() {
    echo "[s174] launching gz_to_uds_bridge"
    distrobox enter crazysim-garden -- bash -c \
        "$REPO_ROOT/build-sim/sim/gz_to_uds_bridge \
            --cam-topic /downward_cam/image \
            --cam-sock /tmp/sentai_cam.sock \
            --out-sock /tmp/sentai_flow_out.sock" \
        > "$WORKDIR/bridge.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
}

start_gt_recorder() {
    rm -f "$WORKDIR/gt_poses.jsonl"
    echo "[s174] starting gt_recorder"
    GT_RECORDER_OUT="$WORKDIR/gt_poses.jsonl" \
    GT_RECORDER_WORLD="$WORLD" \
        "$VENV_PY" "$GT_RECORDER" > "$WORKDIR/gt_recorder.log" 2>&1 &
    GT_PID=$!
    sleep 1.5
}

stop_gt_recorder() {
    [ -n "${GT_PID:-}" ] && kill -TERM "$GT_PID" 2>/dev/null || true
    sleep 0.5
    pkill -9 -f "sim/scripts/gt_recorder.py" 2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -f "gz topic -e -t /world/${WORLD}/dynamic_pose" 2>/dev/null || true
}

stage_mission() {
    echo "[s174] staging $MISSION_FILE → $FS_ROOT/"
    mkdir -p "$FS_ROOT"
    cp "$MISSION_FILE" "$FS_ROOT/"
    rm -f "$FS_ROOT/mission_yaw_aruco_journal.txt" \
          "$FS_ROOT/mission_yaw_aruco_summary.json"
}

run_mission() {
    local STAMP="$(date +%Y%m%d_%H%M%S)"
    local FR_DIR="$WORKDIR/fr_$STAMP"
    mkdir -p "$FR_DIR" "$FR_DIR/frames"
    ln -sfn "$FR_DIR" "$WORKDIR/fr_current"
    echo "$FR_DIR" > "$WORKDIR/fr_dir"
    echo "[s174] sentai.fr per-trial dir → $FR_DIR (symlink fr_current)"
    echo "[s174] running mission_yaw_aruco (timeout ${MISSION_TIMEOUT_S}s)"
    echo "import mission_yaw_aruco; r = mission_yaw_aruco.run(); print('FINAL_STATUS:', r['status'], 'kp_x:', r['kp_x'], 'is_done:', r['is_done'], 'aborted:', r['aborted_by_safety'])" \
        | env SENTAI_SIM_ROOT="$FS_ROOT" \
          timeout "$MISSION_TIMEOUT_S" "$SIM_BIN" \
        > "$WORKDIR/repl.log" 2>&1 || true
}

verdict_run() { "$VENV_PY" "$VERDICT"; }

# ─── Main ─────────────────────────────────────────────────────────
respawn_from_origin
ensure_sitl_up        || exit 1
sleep 2
stage_mission
ensure_bridge_up
start_gt_recorder
run_mission
stop_gt_recorder
verdict_run && rc=0 || rc=$?
echo "[s174] exit code $rc"
exit $rc

#!/bin/bash
# s170 — SecurityArucoBaseline (OP-S10-W12-T6).
#
# Operator-named 2026-05-18.  Per operator: "decolam si lasam sa faca
# hover 30 secunde sau pana cand se activeaza safety. Cu Flow activat,
# atat."
#
# Mission code runs INSIDE sentai_sim's MP runtime — host only:
#   1. brings up SITL stack + sentai_sim + camera bridge + GT recorder
#   2. stages mission_security_aruco.py into the SIM virtual FS
#   3. triggers `import mission_security_aruco; mission_security_aruco.run()`
#      via REPL stdin
#   4. post-processes dumped camera frames with cv2.aruco to produce
#      operator-friendly hardlinks `t<ms>_n<dets>_f<fseq>.ppm`
#   5. runs verdict
#
# HARD RULE [[missions-run-in-sentai-only]]: NO cflib / mission logic
# on host.  Python host-side allowed only for stack launcher, GT
# recorder (post-mortem), cv2 inspection (post-mortem), and verdict.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s170_security_aruco_baseline
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"
MISSION_FILE="$SCRIPT_DIR/mission_security_aruco.py"
INSPECT_DUMP="$SCRIPT_DIR/inspect_dump.py"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
STOP_SH="$REPO_ROOT/examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh"
GT_RECORDER="$REPO_ROOT/sim/scripts/gt_recorder.py"
VERDICT="$SCRIPT_DIR/verdict.py"
WORLD=sentai_crazysim

MISSION_TIMEOUT_S=60       # HARD CAP per operator 2026-05-18 — kill if mission hangs

is_cf2_up() { ss -lun 2>/dev/null | grep -q ":19850"; }

respawn_from_origin() {
    echo "[s170] unconditional teardown"
    bash "$STOP_SH" > "$WORKDIR/stop.log" 2>&1 || true
    for p in $(pgrep -f "build-sim/sim/sentai_sim$|gz_to_uds_bridge|sitl_make/build/cf2|gz sim|gt_recorder"); do
        kill -9 "$p" 2>/dev/null || true
    done
    distrobox enter crazysim-garden -- pkill -9 -f "gz topic -e -t /world/${WORLD}/dynamic_pose" 2>/dev/null || true
    rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock /tmp/sentai_sim_stdin.fifo 2>/dev/null
    sleep 1.5
}

ensure_sitl_up() {
    echo "[s170] launching SITL stack"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
        > "$WORKDIR/sitl.log" 2>&1 &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s170] FAIL — SITL never came up"; return 1
        fi
        sleep 2
    done
    echo "[s170] cf2 UDP 19850 ready"
}

ensure_bridge_up() {
    echo "[s170] launching gz_to_uds_bridge"
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
    echo "[s170] starting gt_recorder"
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
    echo "[s170] staging $MISSION_FILE → $FS_ROOT/"
    mkdir -p "$FS_ROOT"
    cp "$MISSION_FILE" "$FS_ROOT/"
    rm -f "$FS_ROOT/mission_security_aruco_journal.txt" \
          "$FS_ROOT/mission_security_aruco_summary.json"
}

run_mission() {
    local STAMP="$(date +%Y%m%d_%H%M%S)"
    local FRAMES_DIR="$WORKDIR/sentai_frames_$STAMP"
    # ── sentai.fr Flight Recorder per-trial dir ──────────────────
    # Mission MP hardcodes path "/tmp/s170.../fr_current"; we point
    # it at a per-trial dir via symlink so multiple trials don't
    # overwrite each other.
    local FR_DIR="$WORKDIR/fr_$STAMP"
    mkdir -p "$FRAMES_DIR" "$FR_DIR/frames"
    ln -sfn "$FR_DIR" "$WORKDIR/fr_current"
    echo "$FRAMES_DIR" > "$WORKDIR/frames_dir"
    echo "$FR_DIR" > "$WORKDIR/fr_dir"
    echo "[s170] sentai_sim raw frames    → $FRAMES_DIR"
    echo "[s170] sentai.fr per-trial dir  → $FR_DIR (symlink fr_current)"
    echo "[s170] running mission_security_aruco (timeout ${MISSION_TIMEOUT_S}s)"
    echo "import mission_security_aruco; r = mission_security_aruco.run(); print('FINAL_STATUS:', r['status'], 'aborted:', r['aborted'], 'elapsed:', r['hover_elapsed_s'], 'ticks:', r['tick_journal_writes'])" \
        | env SENTAI_SIM_ROOT="$FS_ROOT" \
              SENTAI_DUMP_FRAMES_DIR="$FRAMES_DIR" \
              SENTAI_DUMP_FRAMES_EVERY=3 \
          timeout "$MISSION_TIMEOUT_S" "$SIM_BIN" \
        > "$WORKDIR/repl.log" 2>&1 || true
}

post_process_frames() {
    local FRAMES_DIR
    FRAMES_DIR=$(cat "$WORKDIR/frames_dir" 2>/dev/null)
    [ -z "$FRAMES_DIR" ] && return 0
    [ ! -d "$FRAMES_DIR" ] && return 0
    local INSPECT_DIR="$WORKDIR/inspect_$(basename "$FRAMES_DIR" | sed 's/sentai_frames_//')"
    mkdir -p "$INSPECT_DIR"
    echo "$INSPECT_DIR" > "$WORKDIR/inspect_dir"
    echo "[s170] inspect dump → $INSPECT_DIR"
    SENTAI_DUMP_FRAMES_DIR="$FRAMES_DIR" \
    INSPECT_DIR="$INSPECT_DIR" \
        "$VENV_PY" "$INSPECT_DUMP" 2>&1 || true
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
post_process_frames
verdict_run && rc=0 || rc=$?
echo "[s170] exit code $rc"
exit $rc

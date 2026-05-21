#!/bin/bash
# s187 — calib bringup Gazebo end-to-end (anti-cheat compliant).
#
# Drives sentai.calib.run_bringup() inside sentai_sim through the
# cf2 SITL + WhyCon square pad stack.  See README.md for the
# acceptance gate.  Mirror of s182/run.sh per [[gz-world-edit]]
# convention; reuses the same world (sentai_whycon SDF already has
# the iter-11 square pad geometry).
#
# WBS: OP-S10-W21-T4 phase-2 acceptance gate.
#
# Anti-cheat invariants ([[sentai-sim-air-gapped-from-truth]]):
#   - sentai_sim consumes camera frames + cf2 CRTP telemetry only.
#   - gt_recorder is HOST-SIDE; output JSONL feeds verdict_s187.py
#     for post-mortem only.  NEVER fed back to sentai_sim.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s187_calib_bringup_gazebo
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
GZ_BRIDGE_BIN="$REPO_ROOT/build-sim/sim/gz_to_uds_bridge"
GT_RECORDER="$REPO_ROOT/sim/scripts/gt_recorder.py"
VERDICT="$SCRIPT_DIR/verdict_s187.py"
MISSION_SRC="$SCRIPT_DIR/mission_s187.py"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

WORLD=sentai_whycon

is_cf2_up()    { ss -lun 2>/dev/null | grep -q ":19850"; }
cleanup_socks(){ rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock 2>/dev/null; }

cleanup_all() {
    echo "[s187] cleanup"
    for p in $(pgrep -f "build-sim/sim/sentai_sim$|gz_to_uds_bridge|gz_to_camera_bridge|sitl_make/build/cf2|gz sim|gt_recorder"); do
        kill -9 "$p" 2>/dev/null || true
    done
    distrobox enter crazysim-garden -- \
        pkill -9 -f "gz topic -e -t /world/${WORLD}/dynamic_pose" \
        2>/dev/null || true
    cleanup_socks
    sleep 1.5
}

trap cleanup_all EXIT INT TERM

# ---- 0. fresh respawn -------------------------------------------------
cleanup_all
sleep 0.5

# ---- 1. precondition: SIM build fresh --------------------------------
# Bringup orchestrator changes invalidate stale binaries; rebuild target.
echo "[s187] rebuilding sentai_sim"
(cd "$REPO_ROOT/build-sim" && cmake --build . --target sentai_sim) \
    > "$WORKDIR/sim_build.log" 2>&1
echo "[s187] sentai_sim build OK"

# ---- 2. launch SITL stack --------------------------------------------
echo "[s187] launching SITL stack (world=$WORLD)"
distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
    > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
disown $! 2>/dev/null || true
deadline=$(( $(date +%s) + 150 ))
until is_cf2_up; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[s187] FAIL — SITL never came up; see $WORKDIR/sitl.log"
        exit 1
    fi
    sleep 2
done
echo "[s187] cf2 UDP 19850 ready"

# ---- 3. launch camera bridge inside distrobox ------------------------
echo "[s187] launching gz_to_uds_bridge"
distrobox enter crazysim-garden -- bash -c \
    "$GZ_BRIDGE_BIN \
        --cam-topic /downward_cam/image \
        --cam-sock /tmp/sentai_cam.sock \
        --out-sock /tmp/sentai_flow_out.sock" \
    > "$WORKDIR/gz_bridge.log" 2>&1 < /dev/null &
BRIDGE_PID=$!
disown $BRIDGE_PID 2>/dev/null || true
sleep 3

# ---- 4. start gt_recorder (post-mortem only) -------------------------
mkdir -p "$WORKDIR/gt"
GT_OUT="$WORKDIR/gt/cf2_gt.jsonl"
echo "[s187] starting gt_recorder for crazyflie_0 → $GT_OUT"
GT_RECORDER_WORLD="$WORLD" \
GT_RECORDER_MODEL=crazyflie_0 \
GT_RECORDER_OUT="$GT_OUT" \
    "$VENV_PY" "$GT_RECORDER" > "$WORKDIR/gt_recorder.log" 2>&1 &
GT_PID=$!
sleep 2

# ---- 5. stage mission + run inside sentai_sim ------------------------
mkdir -p "$FS_ROOT"
# Pre-create FR output tree (mkdir_p in sentai_fr.cc is single-level).
# FR persists inside the experiment folder so frames+scalars survive
# in git as durable thesis artefacts.
FR_DIR="$SCRIPT_DIR/fr_current"
rm -rf "$FR_DIR"
mkdir -p "$FR_DIR/frames"
cp "$MISSION_SRC" "$FS_ROOT/mission_s187.py"
echo "[s187] running mission inside sentai_sim (budget 240s)"
(echo "import mission_s187; r = mission_s187.run(); print('FINAL:', r['status'])" \
    | timeout 280 "$SIM_BIN" > "$WORKDIR/sentai_repl.log" 2>&1) || true

# ---- 6. stop recorders ----------------------------------------------
echo "[s187] stopping gt_recorder + bridge"
kill -INT "$GT_PID" 2>/dev/null || true
kill "$BRIDGE_PID" 2>/dev/null || true
sleep 1
distrobox enter crazysim-garden -- \
    pkill -9 -f "gz topic -e -t /world/${WORLD}/dynamic_pose" \
    2>/dev/null || true

# ---- 7. snapshot artifacts + verdict ---------------------------------
JOURNAL="$FS_ROOT/mission_s187_journal.txt"
SUMMARY="$FS_ROOT/mission_s187_summary.json"
cp -f "$JOURNAL" "$SCRIPT_DIR/journal.txt" 2>/dev/null || true
cp -f "$SUMMARY" "$SCRIPT_DIR/summary.json" 2>/dev/null || true
cp -f "$GT_OUT"  "$SCRIPT_DIR/cf2_gt.jsonl" 2>/dev/null || true

echo "[s187] running verdict"
"$VENV_PY" "$VERDICT" \
    --summary "$SCRIPT_DIR/summary.json" \
    --journal "$SCRIPT_DIR/journal.txt" \
    --out-dir "$SCRIPT_DIR" || true
echo "[s187] done"

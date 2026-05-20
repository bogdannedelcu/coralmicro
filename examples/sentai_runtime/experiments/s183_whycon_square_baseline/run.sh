#!/bin/bash
# s183 — WhyCon Gazebo evaluation (anti-cheat compliant).
#
# Orchestrator: launches SITL stack (cf2 + Gazebo + gz_to_camera_bridge)
# inside crazysim-garden distrobox, starts gt_recorder host-side, runs
# the mission inside sentai_sim, then invokes verdict to produce the
# thesis-grade X/Y/Z plots.
#
# WBS: OP-S10-W19-T4 step 2.
#
# Anti-cheat invariants ([[sentai-sim-air-gapped-from-truth]]):
#   - sentai_sim consumes camera frames + cf2 CRTP telemetry only.
#   - gt_recorder is HOST-SIDE; output JSONL is consumed only by
#     verdict.py (post-mortem analysis).  NEVER fed back to sentai_sim.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s183_whycon_square
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
# Bridge is C++ (build-sim/sim/gz_to_uds_bridge), MUST run inside
# crazysim-garden distrobox where gz transport libs are available
# (host venv lacks `gz` Python bindings).  iter-6 first run was
# silent-n=0 because we used the Python wrapper host-side.
GZ_BRIDGE_BIN="$REPO_ROOT/build-sim/sim/gz_to_uds_bridge"
GT_RECORDER="$REPO_ROOT/sim/scripts/gt_recorder.py"
VERDICT="$SCRIPT_DIR/verdict.py"
MISSION_SRC="$SCRIPT_DIR/mission_s183.py"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

WORLD=sentai_whycon

is_cf2_up()    { ss -lun 2>/dev/null | grep -q ":19850"; }
cleanup_socks(){ rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock 2>/dev/null; }

cleanup_all() {
    echo "[s183] cleanup"
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

# ---- 1. launch SITL stack --------------------------------------------
echo "[s183] launching SITL stack (world=$WORLD)"
distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
    > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
disown $! 2>/dev/null || true
deadline=$(( $(date +%s) + 150 ))
until is_cf2_up; do
    if [ $(date +%s) -ge $deadline ]; then
        echo "[s183] FAIL — SITL never came up; see $WORKDIR/sitl.log"
        exit 1
    fi
    sleep 2
done
echo "[s183] cf2 UDP 19850 ready"

# ---- 2. launch camera bridge (C++ binary inside distrobox) -----------
echo "[s183] launching gz_to_uds_bridge inside crazysim-garden"
distrobox enter crazysim-garden -- bash -c \
    "$GZ_BRIDGE_BIN \
        --cam-topic /downward_cam/image \
        --cam-sock /tmp/sentai_cam.sock \
        --out-sock /tmp/sentai_flow_out.sock" \
    > "$WORKDIR/gz_bridge.log" 2>&1 < /dev/null &
BRIDGE_PID=$!
disown $BRIDGE_PID 2>/dev/null || true
sleep 3

# ---- 3. start gt_recorder (host-side post-mortem only) ---------------
mkdir -p "$WORKDIR/gt"
GT_OUT="$WORKDIR/gt/cf2_gt.jsonl"
echo "[s183] starting gt_recorder for crazyflie_0 → $GT_OUT"
GT_RECORDER_WORLD="$WORLD" \
GT_RECORDER_MODEL=crazyflie_0 \
GT_RECORDER_OUT="$GT_OUT" \
    "$VENV_PY" "$GT_RECORDER" > "$WORKDIR/gt_recorder.log" 2>&1 &
GT_PID=$!
sleep 2

# ---- 4. stage mission into sentai_fs_root + run inside sentai_sim ----
mkdir -p "$FS_ROOT"
# Pre-create FR output tree (mkdir_p in sentai_fr.cc is single-level).
rm -rf "$WORKDIR/fr_current"
mkdir -p "$WORKDIR/fr_current/frames"
cp "$MISSION_SRC" "$FS_ROOT/mission_s183.py"
echo "[s183] running mission inside sentai_sim"
(echo "import mission_s183; r = mission_s183.run(); print('FINAL:', r['status'])" \
    | timeout 180 "$SIM_BIN" > "$WORKDIR/sentai_repl.log" 2>&1) || true

# ---- 5. stop recorders + verdict -------------------------------------
echo "[s183] stopping gt_recorder + bridge"
kill -INT "$GT_PID" 2>/dev/null || true
kill "$BRIDGE_PID" 2>/dev/null || true
sleep 1
distrobox enter crazysim-garden -- \
    pkill -9 -f "gz topic -e -t /world/${WORLD}/dynamic_pose" \
    2>/dev/null || true

JOURNAL="$FS_ROOT/mission_s183_journal.txt"
SUMMARY="$FS_ROOT/mission_s183_summary.json"
cp -f "$JOURNAL" "$SCRIPT_DIR/journal.txt" 2>/dev/null || true
cp -f "$SUMMARY" "$SCRIPT_DIR/summary.json" 2>/dev/null || true
cp -f "$GT_OUT"  "$SCRIPT_DIR/cf2_gt.jsonl" 2>/dev/null || true

echo "[s183] running verdict"
"$VENV_PY" "$VERDICT" \
    --journal "$SCRIPT_DIR/journal.txt" \
    --gt "$SCRIPT_DIR/cf2_gt.jsonl" \
    --out-dir "$SCRIPT_DIR" || true
echo "[s183] done"

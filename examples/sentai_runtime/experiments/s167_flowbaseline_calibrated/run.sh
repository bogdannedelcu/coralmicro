#!/bin/bash
# s167 — FlowBaseline2 (calibrated, 4-phase) with Gazebo GT comparison.
# WBS: OP-S8-W1 path toward T8 with takeoff calibration phase.
#
# 4 phases (see mission_flowbaseline2.py):
#   F1  takeoff to 0.6 m   (wide FOV — all 4 markers in view)
#   F2a static pin @ 0.6 m, VPE @ 20 Hz, PnP-z baseline collection
#   F2b axes probe ±4 cm × 4 directions → derive BODY_XFORM empirically
#   F3  climb to 1.0 m + 10 s measurement hover, VPE @ 5 Hz
#   F4  land
#
# HARD RULES (per [[experiments-start-from-origin]] +
# [[cf2-sitl-cheat-odom-gt]] + [[sentai-sim-air-gapped-from-truth]]):
#   - cf2 ALWAYS respawned at origin (no is_up guard).
#   - Spawn pose asserted post-respawn: |x|<0.05 AND |y|<0.05 AND z<0.10.
#   - GT recorded via canonical sim/scripts/gt_recorder.py, host-side
#     only.  Verdict consumes JSONL post-mortem; never injected back.
#   - Gazebo GUI mandatory ([[gazebo-gui-required]]).
#
# Usage:    bash run.sh
# Exit 0 on PASS (F3 GT gate), 1 on FAIL.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s167_flowbaseline_calibrated
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
MISSION="$SCRIPT_DIR/mission_flowbaseline2.py"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
STOP_SH="$REPO_ROOT/examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh"
GT_RECORDER="$REPO_ROOT/sim/scripts/gt_recorder.py"
VERDICT="$SCRIPT_DIR/verdict.py"

WORLD=sentai_crazysim

is_cf2_up()    { ss -lun 2>/dev/null | grep -q ":19850"; }
cleanup_socks(){ rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock 2>/dev/null; }

respawn_from_origin() {
    echo "[s167] unconditional teardown (no is_up guard)"
    bash "$STOP_SH" > "$WORKDIR/stop.log" 2>&1 || true
    for p in $(pgrep -f "build-sim/sim/sentai_sim$|gz_to_uds_bridge|sitl_make/build/cf2|gz sim|gt_recorder"); do
        kill -9 "$p" 2>/dev/null || true
    done
    distrobox enter crazysim-garden -- pkill -9 -f "gz topic -e -t /world/${WORLD}/dynamic_pose" 2>/dev/null || true
    cleanup_socks
    sleep 1.5
    if is_cf2_up; then
        echo "[s167] FAIL — UDP 19850 still bound after teardown"; return 1
    fi
}

ensure_sitl_up() {
    echo "[s167] launching SITL stack via $LAUNCH_HYBRID"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
        > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s167] FAIL — SITL never came up; see $WORKDIR/sitl.log"
            return 1
        fi
        sleep 2
    done
    echo "[s167] cf2 UDP 19850 ready"
}

assert_spawn_at_origin() {
    echo "[s167] querying gz model -m crazyflie_0 --pose"
    local pose_txt
    pose_txt=$(distrobox enter crazysim-garden -- \
                gz model -m crazyflie_0 --pose 2>"$WORKDIR/gz_model_err.log" \
                | tee "$WORKDIR/gz_model_pose.txt") || true
    "$VENV_PY" - "$pose_txt" "$WORKDIR/spawn_pose.json" <<'PY' || return 1
import json, re, sys
raw = sys.argv[1]; out = sys.argv[2]
m_xyz = None; in_pose = False
for line in raw.splitlines():
    if "Pose" in line and "XYZ" in line:
        in_pose = True; continue
    if in_pose:
        m = re.match(r'\s*\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]', line)
        if m:
            m_xyz = m; break
if m_xyz is None:
    m_xyz = re.search(r'\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]', raw)
if m_xyz is None:
    print(f"[spawn] could NOT parse pose:\n{raw[:400]}", file=sys.stderr); sys.exit(2)
x, y, z = float(m_xyz.group(1)), float(m_xyz.group(2)), float(m_xyz.group(3))
ok = abs(x) < 0.05 and abs(y) < 0.05 and z < 0.10
open(out, "w").write(json.dumps({"x": x, "y": y, "z": z, "at_origin": ok,
    "tol": {"x": 0.05, "y": 0.05, "z": 0.10}}, indent=2))
print(f"[spawn] cf2 pose = ({x:+.3f}, {y:+.3f}, {z:+.3f})  at_origin={ok}",
      file=sys.stderr)
sys.exit(0 if ok else 3)
PY
    local rc=$?
    [ "$rc" -ne 0 ] && { echo "[s167] FAIL — spawn-at-origin assert"; return 1; }
    echo "[s167] spawn at origin OK"
}

ensure_sentai_sim_up() {
    local FIFO=/tmp/sentai_sim_stdin.fifo
    [ -p "$FIFO" ] || mkfifo "$FIFO"
    sleep infinity > "$FIFO" 2>/dev/null < /dev/null &
    disown $! 2>/dev/null || true
    local FRAMES_DIR="$WORKDIR/sentai_frames_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$FRAMES_DIR"
    echo "$FRAMES_DIR" > "$WORKDIR/frames_dir"
    echo "[s167] sentai_sim frames → $FRAMES_DIR"
    # FRAMES_EVERY=3 → ~10 Hz frame dump for denser visual inspection
    # (operator-requested 2026-05-18).
    env SENTAI_DUMP_FRAMES_DIR="$FRAMES_DIR" \
        SENTAI_DUMP_FRAMES_EVERY=3 SENTAI_DUMP_RAW_EVERY=15 \
        SENTAI_SIM_ROOT="$REPO_ROOT/build-sim/sentai_fs_root" \
        "$REPO_ROOT/build-sim/sim/sentai_sim" < "$FIFO" \
        > "$WORKDIR/sim.log" 2>&1 &
    local SIM_PID=$!; disown $SIM_PID 2>/dev/null || true
    echo $SIM_PID > "$WORKDIR/sim.pid"
    local deadline=$(( $(date +%s) + 15 ))
    until [ -S /tmp/sentai_cam.sock ]; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s167] FAIL — sentai_sim did not open cam.sock"; return 1
        fi
        sleep 0.5
    done
    echo "[s167] sentai_sim PID=$SIM_PID, cam.sock ready"
}

ensure_bridge_up() {
    echo "[s167] launching gz_to_uds_bridge"
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
            echo "[s167] FAIL — gz_to_uds_bridge did not open flow_out.sock"; return 1
        fi
        sleep 0.5
    done
    echo "[s167] flow_out.sock ready"
}

start_gt_recorder() {
    rm -f "$WORKDIR/gt_poses.jsonl"
    echo "[s167] starting canonical gt_recorder ($GT_RECORDER)"
    GT_RECORDER_OUT="$WORKDIR/gt_poses.jsonl" \
    GT_RECORDER_WORLD="$WORLD" \
        "$VENV_PY" "$GT_RECORDER" > "$WORKDIR/gt_recorder.log" 2>&1 &
    GT_PID=$!
    echo "[s167] GT recorder PID=$GT_PID"
    sleep 1.5
}

stop_gt_recorder() {
    # SIGTERM the Python wrapper; do NOT wait (podman/distrobox don't
    # always propagate signals → wait hangs forever).  Then KILL host
    # leftovers + reach into the container to kill the gz topic process.
    if [ -n "${GT_PID:-}" ] && kill -0 "$GT_PID" 2>/dev/null; then
        kill -TERM "$GT_PID" 2>/dev/null || true
    fi
    sleep 0.5
    pkill -9 -f "sim/scripts/gt_recorder.py" 2>/dev/null || true
    pkill -9 -f "gt_recorder.py" 2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -f "gz topic -e -t /world/${WORLD}/dynamic_pose" 2>/dev/null || true
}

run_mission() {
    local FRAMES_DIR
    FRAMES_DIR=$(cat "$WORKDIR/frames_dir")
    # Fresh inspect-dir per trial — operator-visible visual log of
    # what the camera saw + how many markers were detected.
    local INSPECT_DIR="$WORKDIR/inspect_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$INSPECT_DIR"
    echo "$INSPECT_DIR" > "$WORKDIR/inspect_dir"
    echo "[s167] inspect-dir for this trial: $INSPECT_DIR"
    echo "[s167] running mission_flowbaseline2.py"
    env SENTAI_DUMP_FRAMES_DIR="$FRAMES_DIR" \
        SENTAI_INSPECT_DIR="$INSPECT_DIR" \
        "$VENV_PY" "$MISSION" > "$WORKDIR/mission.log" 2>&1
}

verdict() { "$VENV_PY" "$VERDICT"; }

# ─── Main ───
respawn_from_origin   || exit 1
ensure_sitl_up        || exit 1
sleep 3
assert_spawn_at_origin || { echo "[s167] aborting trial — drone not at origin"; exit 1; }
ensure_sentai_sim_up  || exit 1
ensure_bridge_up      || exit 1
start_gt_recorder
run_mission           || true
stop_gt_recorder
verdict && rc=0 || rc=$?
echo "[s167] exit code $rc"
exit $rc

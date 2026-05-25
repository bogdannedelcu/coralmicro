#!/bin/bash
# s166 — FlowBaseline with Gazebo GT comparison (OP-S8-W1-T7+T8).
#
# Combines s127 FlowBaseline mission (aruco_hover.py 15s hover @ z=1m,
# sentai.flow + VPE) with s165 GT-recorder pattern.  Replaces the cheat-
# masked s127 baseline ([[cf2-sitl-cheat-odom-gt]]).
#
# HARD RULES enforced by this runner:
#   - cf2 ALWAYS respawned at origin (no "is_up" guard, no reuse).
#     Per operator rule "experiments start from landing place".
#   - Spawn pose asserted post-respawn: |x|<0.05 AND |y|<0.05 AND z<0.10
#     (T6 from diary/2026-05-17.md — originally ideas/op_s8_w1_handoff.md).
#     Mission aborts if violated.
#   - GT consumed HOST-SIDE ONLY at verdict time, never injected into
#     cf2 firmware or sentai_sim ([[sentai-sim-air-gapped-from-truth]]).
#   - Gazebo GUI required ([[gazebo-gui-required]]) — launch_hybrid_cf2
#     starts both server + GUI when display available.
#
# Usage:    bash run.sh
# Exit 0 on PASS (GT-based gate), 1 on FAIL.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s166_flowbaseline_gt
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
ARUCO_HOVER="$REPO_ROOT/examples/sentai_runtime/experiments/s091_aruco_lowalt/aruco_hover.py"
LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
STOP_SH="$REPO_ROOT/examples/sentai_runtime/experiments/s127_flowbaseline/stop.sh"
GT_RECORDER="$SCRIPT_DIR/gt_recorder.py"
VERDICT="$SCRIPT_DIR/verdict.py"

WORLD=sentai_crazysim

is_cf2_up()    { ss -lun 2>/dev/null | grep -q ":19850"; }
cleanup_socks(){ rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock 2>/dev/null; }

# T6: unconditional teardown — every trial starts from origin.
respawn_from_origin() {
    echo "[s166] T6 — unconditional teardown (no is_up guard)"
    bash "$STOP_SH" > "$WORKDIR/stop.log" 2>&1 || true
    for p in $(pgrep -f "build-sim/sim/sentai_sim$|gz_to_uds_bridge|sitl_make/build/cf2|gz sim|gt_recorder"); do
        kill -9 "$p" 2>/dev/null || true
    done
    cleanup_socks
    sleep 1.5
    if is_cf2_up; then
        echo "[s166] FAIL — UDP 19850 still bound after teardown"
        return 1
    fi
}

ensure_sitl_up() {
    echo "[s166] launching SITL stack via $LAUNCH_HYBRID"
    distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
        > "$WORKDIR/sitl.log" 2>&1 < /dev/null &
    disown $! 2>/dev/null || true
    local deadline=$(( $(date +%s) + 150 ))
    until is_cf2_up; do
        if [ $(date +%s) -ge $deadline ]; then
            echo "[s166] FAIL — SITL never came up; see $WORKDIR/sitl.log"
            return 1
        fi
        sleep 2
    done
    echo "[s166] cf2 UDP 19850 ready"
}

# T6 step 2: assert spawn pose is at origin before mission starts.
assert_spawn_at_origin() {
    echo "[s166] querying gz model -m crazyflie_0 --pose"
    local pose_txt
    pose_txt=$(distrobox enter crazysim-garden -- \
                gz model -m crazyflie_0 --pose 2>"$WORKDIR/gz_model_err.log" \
                | tee "$WORKDIR/gz_model_pose.txt") || true
    # Output format: lines like "[Pose_3d=X Y Z R P Yaw]" or proto-text.
    # Be tolerant — look for first three "X: <num>" / "Y: ..." entries.
    "$VENV_PY" - "$pose_txt" "$WORKDIR/spawn_pose.json" <<'PY' || return 1
import json, re, sys
raw = sys.argv[1]
out = sys.argv[2]
# gz model -m … --pose output (Garden) looks like:
#   Model: [N]
#     - Name: crazyflie_0
#     - Pose [ XYZ (m) ] [ RPY (rad) ]:
#       [X Y Z]
#       [R P Y]
# Strategy: find the bracketed XYZ triplet on the FIRST line that
# contains exactly 3 floats inside [...] after the "Pose" label.
m_xyz = None
in_pose = False
for line in raw.splitlines():
    if "Pose" in line and "XYZ" in line:
        in_pose = True
        continue
    if in_pose:
        m = re.match(r'\s*\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]', line)
        if m:
            m_xyz = m
            break
if m_xyz is None:
    # Fallback: first bracketed 3-float pattern anywhere.
    m_xyz = re.search(r'\[\s*([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s*\]', raw)
if m_xyz is None:
    print(f"[spawn] could NOT parse pose from gz model output:\n{raw[:400]}", file=sys.stderr)
    sys.exit(2)
x, y, z = float(m_xyz.group(1)), float(m_xyz.group(2)), float(m_xyz.group(3))
ok = abs(x) < 0.05 and abs(y) < 0.05 and z < 0.10
rec = {"x": x, "y": y, "z": z, "at_origin": ok,
       "tol": {"x": 0.05, "y": 0.05, "z": 0.10}}
open(out, "w").write(json.dumps(rec, indent=2))
print(f"[spawn] cf2 pose = ({x:+.3f}, {y:+.3f}, {z:+.3f})  at_origin={ok}",
      file=sys.stderr)
sys.exit(0 if ok else 3)
PY
    local rc=$?
    if [ "$rc" -ne 0 ]; then
        echo "[s166] FAIL — spawn-at-origin assert failed (rc=$rc, see $WORKDIR/spawn_pose.json)"
        return 1
    fi
    echo "[s166] spawn at origin OK ($(cat "$WORKDIR/spawn_pose.json" | tr -d '\n' | head -c 120))"
}

ensure_sentai_sim_up() {
    local FIFO=/tmp/sentai_sim_stdin.fifo
    [ -p "$FIFO" ] || mkfifo "$FIFO"
    sleep infinity > "$FIFO" 2>/dev/null < /dev/null &
    disown $! 2>/dev/null || true
    local FRAMES_DIR="$WORKDIR/sentai_frames_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$FRAMES_DIR"
    echo "$FRAMES_DIR" > "$WORKDIR/frames_dir"
    echo "[s166] sentai_sim frames → $FRAMES_DIR"
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
            echo "[s166] FAIL — sentai_sim did not open /tmp/sentai_cam.sock"
            return 1
        fi
        sleep 0.5
    done
    echo "[s166] sentai_sim PID=$SIM_PID, cam.sock ready"
}

ensure_bridge_up() {
    echo "[s166] launching gz_to_uds_bridge"
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
            echo "[s166] FAIL — gz_to_uds_bridge did not open flow_out.sock"
            return 1
        fi
        sleep 0.5
    done
    echo "[s166] flow_out.sock ready"
}

start_gt_recorder() {
    rm -f "$WORKDIR/gt_poses.jsonl"
    echo "[s166] starting GT recorder (host-side, anti-cheat compliant)"
    S166_GT_OUT="$WORKDIR/gt_poses.jsonl" \
    S166_GT_WORLD="$WORLD" \
        "$VENV_PY" "$GT_RECORDER" > "$WORKDIR/gt_recorder.log" 2>&1 &
    GT_PID=$!
    echo "[s166] GT recorder PID=$GT_PID"
    sleep 1.5
}

stop_gt_recorder() {
    if [ -n "${GT_PID:-}" ] && kill -0 "$GT_PID" 2>/dev/null; then
        kill -TERM "$GT_PID" 2>/dev/null || true
        wait "$GT_PID" 2>/dev/null || true
    fi
    pkill -f "s166_flowbaseline_gt/gt_recorder.py" 2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -f "gz topic -e -t /world/${WORLD}/dynamic_pose" 2>/dev/null || true
}

run_hover() {
    local FRAMES_DIR
    FRAMES_DIR=$(cat "$WORKDIR/frames_dir")
    echo "[s166] running aruco_hover.py (15s hover @ z=1m)"
    env SENTAI_DUMP_FRAMES_DIR="$FRAMES_DIR" \
        "$VENV_PY" "$ARUCO_HOVER" > "$WORKDIR/hover.log" 2>&1
}

verdict() {
    "$VENV_PY" "$VERDICT"
}

# ─── Main ───
respawn_from_origin   || exit 1
ensure_sitl_up        || exit 1
sleep 3                                       # let plugins settle before pose query
assert_spawn_at_origin || { echo "[s166] aborting trial — drone not at origin"; exit 1; }
ensure_sentai_sim_up  || exit 1
ensure_bridge_up      || exit 1
start_gt_recorder
run_hover             || true                 # let verdict decide
stop_gt_recorder
verdict && rc=0 || rc=$?
echo "[s166] exit code $rc"
exit $rc

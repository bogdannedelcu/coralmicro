#!/bin/bash
# launch_sim.sh — canonical host-side SIM launcher.
#
# Starts the full SIM stack:
#   1. Distrobox-side: Gazebo server (Xvfb :99) + cf2 SITL + Gazebo GUI
#      via launch_hybrid_cf2.sh
#   2. Distrobox-side: gz_to_uds_bridge (camera frames → /tmp/sentai_cam.sock)
#   3. Host-side: gt_recorder (post-mortem GT JSONL — read by verdict only)
#
# Args:
#   $1 WORLD       — world SDF basename (default: sentai_crazysim)
#   $2 WORKDIR     — where logs / pids / artifacts land (default: $PWD)
#
# Side effects:
#   Writes PID files: $WORKDIR/launch_sim.pids (one PID per line)
#   Writes logs:      $WORKDIR/{gz_server,cf2,gz_to_uds_bridge,gt_recorder}.log
#
# Cleanup is the caller's responsibility — read launch_sim.pids and kill.
# The companion launch_sim_cleanup.sh handles this in one call.
#
# Anti-cheat: gt_recorder output (gt.jsonl) is HOST-SIDE ONLY, read only by
# verdict scripts post-mortem.  Never injected into sentai_sim per
# [[sentai-sim-air-gapped-from-truth]].

set -e

WORLD="${1:-sentai_crazysim}"
WORKDIR="${2:-$PWD}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

LAUNCH_HYBRID="$REPO_ROOT/sim/scripts/launch_hybrid_cf2.sh"
GT_RECORDER="$REPO_ROOT/sim/scripts/gt_recorder.py"
VENV_PY="$REPO_ROOT/venv/bin/python3"
[[ -x "$VENV_PY" ]] || VENV_PY=python3

mkdir -p "$WORKDIR"
PIDFILE="$WORKDIR/launch_sim.pids"
: > "$PIDFILE"

echo "[launch_sim] world=$WORLD workdir=$WORKDIR"

# ---- 1. Distrobox-side: Gazebo + cf2 SITL + GUI ------------------------
# launch_hybrid_cf2.sh kills its own prior processes per its header
# (pkill cf2, ruby, gz sim, Xvfb).  We background it so this wrapper can
# continue and also start gt_recorder + camera bridge concurrently.
echo "[launch_sim] starting Gazebo + cf2 SITL via launch_hybrid_cf2.sh"
distrobox enter crazysim-garden -- bash "$LAUNCH_HYBRID" "$WORLD" \
    > "$WORKDIR/launch_hybrid.log" 2>&1 &
HYBRID_PID=$!
echo "$HYBRID_PID" >> "$PIDFILE"

# Give Gazebo + cf2 ~12 s to come up.  launch_hybrid_cf2.sh now sleeps
# 8 s after gz sim -s, then retries spawn up to 3× / 5 s timeout each.
# Add 4 s margin for cf2 SITL connect + initial telemetry.
sleep 12

# ---- 2. Distrobox-side: camera bridge ---------------------------------
# Reads Gazebo /downward_cam/image, scales to 320x240 grayscale, posts
# on /tmp/sentai_cam.sock.  Sentai_sim's camera task subscribes to that
# socket per the air-gap rule.
GZ_BRIDGE="$REPO_ROOT/build-sim/sim/gz_to_uds_bridge"
if [[ -x "$GZ_BRIDGE" ]]; then
    echo "[launch_sim] starting gz_to_uds_bridge"
    distrobox enter crazysim-garden -- "$GZ_BRIDGE" \
        > "$WORKDIR/gz_to_uds_bridge.log" 2>&1 &
    BRIDGE_PID=$!
    echo "$BRIDGE_PID" >> "$PIDFILE"
else
    echo "[launch_sim] WARN: gz_to_uds_bridge not built at $GZ_BRIDGE"
fi

# ---- 3. Host-side: gt_recorder ----------------------------------------
# Subscribes to /world/$WORLD/dynamic_pose/info, writes JSONL.
# Anti-cheat clean: gt.jsonl is read only by verdict scripts, never by
# sentai_sim.
GT_OUT="${SENTAI_FR_DIR:-$WORKDIR/fr}/gt.jsonl"
mkdir -p "$(dirname "$GT_OUT")"
echo "[launch_sim] starting gt_recorder → $GT_OUT"
"$VENV_PY" "$GT_RECORDER" \
    --world "$WORLD" \
    --model crazyflie_0 \
    --out "$GT_OUT" \
    > "$WORKDIR/gt_recorder.log" 2>&1 &
GT_PID=$!
echo "$GT_PID" >> "$PIDFILE"

# ── 4. Readiness gate: wait for camera frames + measure FPS ────────────
# Without this, mission may start before Gazebo+bridge pipeline is live,
# causing 0-marker detection that fakes a "no markers visible" failure.
# Also reports observed FPS so timing issues are visible up-front.
echo "[launch_sim] waiting for first camera frame..."
WAIT_S=0
while [[ $WAIT_S -lt 15 ]]; do
    if grep -q "seq=" "$WORKDIR/gz_to_uds_bridge.log" 2>/dev/null; then
        break
    fi
    sleep 0.5
    WAIT_S=$((WAIT_S + 1))
done

if [[ $WAIT_S -ge 15 ]]; then
    echo "[launch_sim] WARN: no camera frames after 15 s — bridge or Gazebo stalled"
else
    # Measure FPS over a short window.  Prefer bridge_recv seq/timestamps
    # because [bridge] seq= lines are diagnostic and may be sparse before the
    # scene starts moving, which made healthy 30 Hz streams look like 2-3 Hz.
    COUNT_BEFORE=$(grep -c "^\[bridge\] seq=" "$WORKDIR/gz_to_uds_bridge.log" 2>/dev/null || echo 0)
    sleep 4
    COUNT_AFTER=$(grep -c "^\[bridge\] seq=" "$WORKDIR/gz_to_uds_bridge.log" 2>/dev/null || echo 0)
    FPS=$(awk '
        /^\[bridge_recv\] seq=/ {
            split($2, seq_kv, "=");
            split($3, stamp_kv, "=");
            seq = seq_kv[2] + 0;
            stamp = stamp_kv[2] + 0.0;
            if (n == 0) {
                seq0 = seq;
                stamp0 = stamp;
            }
            seq1 = seq;
            stamp1 = stamp;
            n += 1;
        }
        END {
            if (n >= 2 && stamp1 > stamp0) {
                printf "%.1f", (seq1 - seq0) / (stamp1 - stamp0);
            }
        }
    ' "$WORKDIR/gz_to_uds_bridge.log" 2>/dev/null || true)
    if [[ -z "$FPS" ]]; then
        FPS=$(( (COUNT_AFTER - COUNT_BEFORE) / 4 ))
    fi
    echo "[launch_sim] camera FPS ≈ $FPS Hz (target 30 Hz; <15 → CPU contention)"

    # Also probe Gazebo real-time factor (single sample, best-effort).
    RTF=$(timeout 1 distrobox enter crazysim-garden -- \
        gz topic -e -t /stats 2>/dev/null \
        | grep -m1 "real_time_factor" \
        | awk '{print $NF}' || echo "?")
    echo "[launch_sim] Gazebo RTF = $RTF (target ~1.0)"
fi

# Sanity: gt_recorder needs ~1 s to connect to gz topic
sleep 1

echo "[launch_sim] all up.  pids → $PIDFILE"
echo "[launch_sim] view with: tail -F $WORKDIR/*.log"
echo "[launch_sim] kill with: bash $REPO_ROOT/sim/scripts/launch_sim_cleanup.sh $WORKDIR"

#!/bin/bash
# launch_sim_cleanup.sh — aggressive kill of everything launch_sim.sh
# might have spawned.  Robust against single-quoted command args and
# distrobox-side processes.
#
# Args:
#   $1 WORKDIR  (default: $PWD) — where launch_sim.pids may live

WORKDIR="${1:-$PWD}"
PIDFILE="$WORKDIR/launch_sim.pids"

if [[ -f "$PIDFILE" ]]; then
    echo "[cleanup] killing PIDs from $PIDFILE"
    while read -r pid; do
        [[ -z "$pid" ]] && continue
        kill -TERM "$pid" 2>/dev/null || true
    done < "$PIDFILE"
    sleep 1
    while read -r pid; do
        [[ -z "$pid" ]] && continue
        kill -9 "$pid" 2>/dev/null || true
    done < "$PIDFILE"
fi

# ── Host-side aggressive pattern kills ─────────────────────────────────
# pkill -f matches the FULL command line — IMPORTANT: `gz topic` and
# `gz 'topic'` are different strings.  Match by sub-binary names that
# are guaranteed-stable.
echo "[cleanup] host-side pattern kills"
pkill -x cf2 2>/dev/null || true
pkill -9 -f "build-sim/sim/sentai_sim" 2>/dev/null || true
pkill -9 -f "gz_to_uds_bridge" 2>/dev/null || true
pkill -9 -f "gt_recorder" 2>/dev/null || true
pkill -9 -f "launch_hybrid_cf2" 2>/dev/null || true
pkill -9 Xvfb 2>/dev/null || true

# gz topic processes — multiple wrappers + the actual gz-transport-topic
# binary.  Kill by binary name (most reliable).
pkill -9 -x gz-transport-topic 2>/dev/null || true
pkill -9 -f "/usr/libexec/gz/transport.*topic" 2>/dev/null || true
# The /bin/sh wrappers that spawn gz topic — match by topic name
pkill -9 -f "dynamic_pose" 2>/dev/null || true

# gz sim (server + GUI).  Process comm is "ruby" (gz is a Ruby wrapper)
# so pkill -x gz doesn't match; use -f to match full command line.
pkill -9 -f "gz sim" 2>/dev/null || true

# ── Distrobox-side ─────────────────────────────────────────────────────
# Each pkill in its own distrobox call so quoting is simple.  Errors
# suppressed (process not present is fine).
if command -v distrobox &>/dev/null; then
    echo "[cleanup] distrobox-side kills"
    distrobox enter crazysim-garden -- pkill -9 -x cf2          2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -f "gz sim"     2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -f "dynamic_pose"  2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -x gz-transport-topic 2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 Xvfb            2>/dev/null || true
    distrobox enter crazysim-garden -- pkill -9 -f gz_to_uds_bridge 2>/dev/null || true
fi

sleep 1

# ── Verify ─────────────────────────────────────────────────────────────
LEFT=$(ps -ef 2>/dev/null \
    | grep -E "gz-transport-topic|gz_to_uds_bridge|build-sim/sim/sentai_sim|dynamic_pose|launch_hybrid_cf2|gt_recorder" \
    | grep -v grep | wc -l)

if [[ "$LEFT" -gt 0 ]]; then
    echo "[cleanup] WARN: $LEFT SIM-pattern processes still alive after kill"
    ps -ef 2>/dev/null \
        | grep -E "gz-transport-topic|gz_to_uds_bridge|build-sim/sim/sentai_sim|dynamic_pose|launch_hybrid_cf2|gt_recorder" \
        | grep -v grep \
        | awk '{print "  PID=" $2 " CMD=" substr($0, index($0,$8))}' \
        | head -5
    echo "[cleanup] force-killing leftover PIDs"
    ps -ef 2>/dev/null \
        | awk '/gz-transport-topic|gz_to_uds_bridge|build-sim\/sim\/sentai_sim|dynamic_pose|launch_hybrid_cf2|gt_recorder/ && !/grep/ {print $2}' \
        | xargs -r kill -9 2>/dev/null || true
fi

echo "[cleanup] done"

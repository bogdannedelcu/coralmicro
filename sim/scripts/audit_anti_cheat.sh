#!/bin/bash
# audit_anti_cheat.sh — verify no SentAI code consumes Gazebo ground truth.
#
# Hard rule (2026-05-17): SentAI sensors must be fed ONLY by camera images
# + drone telemetry (CRTP LOG).  Reading `/world/.../dynamic_pose/info`
# or other gz state from inside SentAI is forbidden.  See sim/ANTI_CHEAT.md.
#
# Scope: scans live SentAI code paths.  Known-legacy PX4 mock experiments
# (s100-s109) are excluded by name — they're not in the active validation
# path and are documented as exempted in ANTI_CHEAT.md.
#
# Exits 0 if clean, 1 with locations otherwise.

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

FAIL=0

# Patterns that signal a ground-truth subscribe / read from inside
# SentAI-facing code.  We grep for the high-signal patterns only —
# generic "world_pose" is excluded because functions like
# `estimate_drone_world_pose` are vision-based PnP on camera frames
# (the allowed path) and would produce false positives.
GT_PATTERNS=(
    'gz topic -e -t /world/[^ ]*/dynamic_pose'
    'gz topic -e -t /world/[^ ]*/model'
    'dynamic_pose/info'
    'ground_truth'
    'gz_truth'
)

# Code paths SentAI sensor pipeline lives in.  Tests and runners are
# included because a test that subscribes to truth is just as bad.
SCAN_PATHS=(
    'libs/'
    'examples/sentai_runtime/'
    'sim/'
)

# Folders excluded as documented legacy.
EXCLUDES=(
    'examples/sentai_runtime/experiments/s100_'
    'examples/sentai_runtime/experiments/s101_'
    'examples/sentai_runtime/experiments/s101b_'
    'examples/sentai_runtime/experiments/s102_'
    'examples/sentai_runtime/experiments/s103_'
    'examples/sentai_runtime/experiments/s104_'
    'examples/sentai_runtime/experiments/s104b_'
    'examples/sentai_runtime/experiments/s105_'
    'examples/sentai_runtime/experiments/s106_'
    'examples/sentai_runtime/experiments/s107_'
    'examples/sentai_runtime/experiments/s108_'
    'examples/sentai_runtime/experiments/s109_'
    'sim/scripts/gz_pose_to_vision_estimate.py'
    'sim/scripts/gz_pose_logger.py'
    'sim/scripts/gz_pose_logger_text.py'
    'sim/ANTI_CHEAT.md'
    'sim/scripts/audit_anti_cheat.sh'
    'sim/gazebo/gz_to_uds_bridge.cc'    # contains the anti-pattern in a rejection comment
    'examples/sentai_runtime/agent/'    # historical experiment.md narrative
    # HOST-side post-mortem GT recorder + verdict scripts.  These run
    # OUTSIDE sentai_sim (host venv-python), write to gt_poses.jsonl,
    # and that file is consumed ONLY by verdict.py for pass/fail
    # comparison AFTER sentai_sim has exited.  Never injected back
    # into sentai_sim — verified by reading every s17X run.sh
    # (audit timestamp 2026-05-18).
    'sim/scripts/gt_recorder.py'                      # canonical tool
    'examples/sentai_runtime/experiments/s165_square_drift_gt/'  # uses GT for verdict only
    'examples/sentai_runtime/experiments/s166_flowbaseline_gt/'  # uses GT for verdict only
    'examples/sentai_runtime/experiments/s167_flowbaseline_calibrated/' # GT post-mortem only
    'examples/sentai_runtime/experiments/s170_security_aruco_baseline/' # GT post-mortem only
    'examples/sentai_runtime/experiments/s172_flow_autotune_baseline/'  # GT post-mortem only
    'examples/sentai_runtime/experiments/s173_flow_hold_validation/'    # GT post-mortem only
    'sim/scripts/README.md'
    'examples/sentai_runtime/Sim.md'    # documentation
)

EXCLUDE_GREP=""
for x in "${EXCLUDES[@]}"; do
    EXCLUDE_GREP="$EXCLUDE_GREP --exclude-dir=$(basename "$x") --exclude=$(basename "$x")"
done

echo "[audit] scanning for ground-truth leak patterns…"
for p in "${GT_PATTERNS[@]}"; do
    # Plain grep over the scan paths, then filter out exact-path excludes.
    HITS=$(grep -rnE "$p" "${SCAN_PATHS[@]}" 2>/dev/null || true)
    if [ -z "$HITS" ]; then continue; fi
    # Filter out exempted paths.
    FILTERED=""
    while IFS= read -r line; do
        skip=0
        for x in "${EXCLUDES[@]}"; do
            case "$line" in
                "$x"*) skip=1; break ;;
            esac
        done
        if [ "$skip" -eq 0 ]; then
            FILTERED="$FILTERED$line"$'\n'
        fi
    done <<< "$HITS"
    FILTERED=$(echo "$FILTERED" | sed '/^$/d')
    if [ -n "$FILTERED" ]; then
        echo
        echo "[audit] FAIL — pattern '$p' present in live code:"
        echo "$FILTERED" | sed 's/^/  /'
        FAIL=1
    fi
done

if [ $FAIL -eq 0 ]; then
    echo "[audit] PASS — no ground-truth leak in scanned SentAI paths"
    exit 0
fi

echo
echo "[audit] FAIL — see sim/ANTI_CHEAT.md.  Refactor to use camera frames"
echo "        or CRTP LOG telemetry only.  Do not pattern after legacy"
echo "        s101-s109 PX4 mock scripts."
exit 1

#!/bin/bash
# s131 — lifter math replay (Python prototype, no Gazebo required).
#
# Phase A: synth_pass.py        — controlled synthetic test (strict)
# Phase B: replay_pass.py       — s130 captured frames (informational)
# Phase C: verdict.py           — combined PASS/FAIL summary
#
# Phase B is auto-SKIPPED if /tmp/s130_image_only_nav/image_vs_cf2.json
# is absent (i.e. s130 has not been run on this machine).
#
# Env:
#   S131_VERBOSE=1     per-frame trace
#   S131_SYNTH_ONLY=1  skip Phase B (don't open captured frames)
#
# Exit codes:
#   0  PASS (synth + replay both green, or synth green + replay skipped)
#   1  synth FAIL (math broken — DO NOT proceed to L5 C++)
#   2  synth PASS but replay FAIL (integration regression; investigate)
set -eu
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
WORKDIR=/tmp/s131_lifter_replay
mkdir -p "$WORKDIR"

VENV_PY="$REPO_ROOT/venv/bin/python3"
SYNTH="$SCRIPT_DIR/synth_pass.py"
REPLAY="$SCRIPT_DIR/replay_pass.py"
VERDICT="$SCRIPT_DIR/verdict.py"

if [ ! -x "$VENV_PY" ]; then
    echo "[s131] FAIL — venv python missing: $VENV_PY"
    exit 1
fi

echo "[s131] phase A — synth_pass.py"
"$VENV_PY" "$SYNTH" > "$WORKDIR/synth.log" 2>&1 && synth_rc=0 || synth_rc=$?
cat "$WORKDIR/synth.log"

if [ "${S131_SYNTH_ONLY:-0}" = "1" ]; then
    echo "[s131] S131_SYNTH_ONLY=1 — skipping phase B"
    replay_rc=0
else
    echo "[s131] phase B — replay_pass.py"
    "$VENV_PY" "$REPLAY" > "$WORKDIR/replay.log" 2>&1 \
        && replay_rc=0 || replay_rc=$?
    cat "$WORKDIR/replay.log"
fi

echo "[s131] phase C — verdict.py"
"$VENV_PY" "$VERDICT" || verdict_rc=$?
verdict_rc=${verdict_rc:-0}

echo "[s131] synth_rc=$synth_rc replay_rc=$replay_rc verdict_rc=$verdict_rc"
exit "$verdict_rc"

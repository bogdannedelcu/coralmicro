#!/usr/bin/env bash
# s179 — HW ArUco detect perf bench driver.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
VENV_PY="$REPO_ROOT/venv/bin/python3"
if [ ! -x "$VENV_PY" ]; then
    VENV_PY="$(command -v python3)"
fi

echo "[s179] using python: $VENV_PY"
"$VENV_PY" "$SCRIPT_DIR/bench_runner.py" \
    --port /dev/ttyACM0 \
    --pgm "$SCRIPT_DIR/frame.pgm" \
    --remote-name "aruco_test.pgm" \
    --iters 10 \
    --results "$SCRIPT_DIR/results.txt" \
    "$@"

echo ""
echo "[s179] results:"
cat "$SCRIPT_DIR/results.txt"

#!/usr/bin/env bash
# s133 — L6 sentai.explore skeleton SIM smoke.
# Run from the repo root:
#   bash examples/sentai_runtime/experiments/s133_explore_skeleton/run.sh
#
# Pass criterion: "VERDICT: PASS" in the output and exit code 0.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SIM="${REPO_ROOT}/build-sim/sim/sentai_sim"
FS_ROOT="${REPO_ROOT}/build-sim/sentai_fs_root"
DRIVER_SRC="${REPO_ROOT}/examples/sentai_runtime/experiments/s133_explore_skeleton/_t_06_explore.py"

if [[ ! -x "$SIM" ]]; then
    echo "FAIL: sentai_sim binary not found at $SIM"
    echo "      build first: cmake --build build-sim --target sentai_sim"
    exit 2
fi
if [[ ! -d "$FS_ROOT" ]]; then
    echo "FAIL: SIM FS root not found at $FS_ROOT"
    exit 2
fi

cp "$DRIVER_SRC" "$FS_ROOT/t06_explore.py"

OUT="$(timeout 30 bash -c "echo 'import t06_explore' | '$SIM'" 2>&1)"
echo "$OUT"

if echo "$OUT" | grep -q "VERDICT: PASS"; then
    echo
    echo "s133 OK — L6 explore SIM skeleton passes all assertions."
    exit 0
else
    echo
    echo "s133 FAIL — see output above."
    exit 1
fi

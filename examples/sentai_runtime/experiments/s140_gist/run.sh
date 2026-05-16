#!/usr/bin/env bash
# s140 — GIST SIM smoke runner.
# Drops the driver in sentai_fs_root, imports via REPL.

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SIM="${REPO_ROOT}/build-sim/sim/sentai_sim"
FS_ROOT="${REPO_ROOT}/build-sim/sentai_fs_root"
DRIVER="${REPO_ROOT}/examples/sentai_runtime/experiments/s140_gist/_t_gist.py"

[[ -x "$SIM" ]] || { echo "FAIL: sentai_sim missing"; exit 2; }
cp "$DRIVER" "$FS_ROOT/t_gist.py"

OUT="$(timeout 30 bash -c "echo 'import t_gist' | '$SIM'" 2>&1)"
echo "$OUT"

if echo "$OUT" | grep -q "VERDICT: PASS"; then
    echo "s140 OK"
    exit 0
fi
echo "s140 FAIL"
exit 1

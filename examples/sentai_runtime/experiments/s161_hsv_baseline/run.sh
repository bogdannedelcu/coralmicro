#!/usr/bin/env bash
# s161 — HSV descriptor SIM smoke + anti-regression runner.
# Mirror of s140 (GIST) / s141 (baseline).  Drops the driver into
# sentai_fs_root and imports via the REPL.

set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SIM="${REPO_ROOT}/build-sim/sim/sentai_sim"
FS_ROOT="${REPO_ROOT}/build-sim/sentai_fs_root"
DRIVER="${REPO_ROOT}/examples/sentai_runtime/experiments/s161_hsv_baseline/_t_hsv.py"

[[ -x "$SIM" ]] || { echo "FAIL: sentai_sim missing"; exit 2; }
cp "$DRIVER" "$FS_ROOT/t_hsv.py"

OUT="$(timeout 30 bash -c "echo 'import t_hsv' | '$SIM'" 2>&1)"
echo "$OUT"

if echo "$OUT" | grep -q "VERDICT: PASS"; then
    echo "s161 OK"
    exit 0
fi
echo "s161 FAIL"
exit 1

#!/usr/bin/env bash
set -euo pipefail

ITER="${1:-iter1}"
HERE="$(cd "$(dirname "$0")" && pwd)"

python3 "$HERE/run_s234.py" --iter "$ITER"

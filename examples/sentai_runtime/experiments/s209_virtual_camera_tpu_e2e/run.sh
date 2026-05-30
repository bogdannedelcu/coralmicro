#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../../../.."
python3 examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/run_s209.py "$@"

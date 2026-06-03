#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$ROOT/emu/output/sentai_edgetpu_physical_probe"

g++ -std=c++17 -O2 -Wall -Wextra \
  -I"$ROOT/emu/host/tflite_shim" \
  -I/home/bogdan/work/libedgetpu/tflite/public \
  "$ROOT/emu/host/sentai_edgetpu_physical_probe.cc" \
  -Wl,-rpath,/home/bogdan/work/edge/edgetpu/libedgetpu/direct/k8 \
  /home/bogdan/work/edge/edgetpu/libedgetpu/direct/k8/libedgetpu.so.1 \
  -o "$OUT"

echo "$OUT"

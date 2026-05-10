#!/bin/bash
# build_gz_bridge.sh — build the Garden gz-transport12 → UDS shim.
#
# This script MUST be run inside the `crazysim-garden` distrobox where
# Gazebo Garden 7.9 (libgz-transport12-dev / libgz-msgs9-dev) is
# installed.  Harmonic is banned for SentAI — see Sim.md "3 known-issue".
#
# Usage:
#   distrobox enter crazysim-garden
#   bash sim/gazebo/build_gz_bridge.sh
#
# Output binary lands in build-sim/sim/gz_to_uds_bridge (host-visible
# because build-sim/ lives under the bind-mounted homedir; survives any
# distrobox restart and is co-located with sentai_sim).

set -e
set -o pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$REPO_ROOT/sim/gazebo/gz_to_uds_bridge.cc"
OUT_DIR="$REPO_ROOT/build-sim/sim"
OUT="$OUT_DIR/gz_to_uds_bridge"

mkdir -p "$OUT_DIR"

if ! command -v pkg-config >/dev/null; then
  echo "ERROR: pkg-config missing — sudo apt install pkg-config" >&2
  exit 1
fi

if ! pkg-config --exists gz-transport12 gz-msgs9; then
  echo "ERROR: gz-transport12 or gz-msgs9 dev pkg missing" >&2
  echo "Inside the distrobox: sudo apt install libgz-transport12-dev libgz-msgs9-dev" >&2
  exit 1
fi

echo "[build_gz_bridge] gz-transport12 $(pkg-config --modversion gz-transport12)"
echo "[build_gz_bridge] gz-msgs9       $(pkg-config --modversion gz-msgs9)"
echo "[build_gz_bridge] -> $OUT"

g++ -O2 -std=c++17 -Wall -Wno-comment "$SRC" \
    $(pkg-config --cflags --libs gz-transport12 gz-msgs9) \
    -lpthread \
    -o "$OUT"

ls -la "$OUT"
echo "[build_gz_bridge] DONE"

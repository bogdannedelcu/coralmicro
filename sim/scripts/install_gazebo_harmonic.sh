#!/bin/bash
# install_gazebo_harmonic.sh
# Installs Gazebo Harmonic on Ubuntu 24.04 (Noble) from the official OSRF apt repo.
# Source: https://gazebosim.org/docs/harmonic/install_ubuntu
#
# Run with: bash /tmp/install_gazebo_harmonic.sh
# (sudo will prompt for your password once at the start)

set -e
set -o pipefail

echo "=== [1/7] Pre-flight check ==="
if ! command -v lsb_release >/dev/null 2>&1; then
    echo "lsb_release not found, will install it below"
fi
echo "Ubuntu: $(. /etc/os-release; echo $PRETTY_NAME)"
echo "Architecture: $(dpkg --print-architecture)"

echo ""
echo "=== [2/7] sudo apt-get update ==="
sudo apt-get update

echo ""
echo "=== [3/7] sudo apt-get install dependencies (curl, lsb-release, gnupg) ==="
sudo apt-get install -y curl lsb-release gnupg

echo ""
echo "=== [4/7] Download OSRF Gazebo signing key ==="
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo ""
echo "=== [5/7] Add OSRF Gazebo apt repo ==="
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

echo ""
echo "=== [6/7] sudo apt-get update (refresh with new repo) ==="
sudo apt-get update

echo ""
echo "=== [7/7] Install gz-harmonic (this is the big one, ~500 MB download) ==="
sudo apt-get install -y gz-harmonic

echo ""
echo "============================================================"
echo "===  DONE  =================================================="
echo "============================================================"
echo ""
echo "Verifying install:"
which gz
gz --version || true
echo ""
echo "Quick sanity (non-blocking):"
gz sim --versions 2>/dev/null || true
echo ""
echo "If you see 'Gazebo Sim, version X.Y.Z' above, install succeeded."
echo "Tell Claude 'gz install done' and we'll continue with the drone+camera setup."

#!/bin/bash
set -e
MODELS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/models"
if [ ! -d "$MODELS_DIR" ]; then
    echo "Downloading clearpath_simulator jazzy models (approx 280MB)..."
    cd /tmp
    wget -qO jazzy.tar.gz https://github.com/clearpathrobotics/clearpath_simulator/archive/refs/heads/jazzy.tar.gz
    mkdir -p cp_sim
    tar -xzf jazzy.tar.gz -C cp_sim --strip-components=1
    mv cp_sim/clearpath_gz/meshes "$MODELS_DIR"
    rm -rf cp_sim jazzy.tar.gz
    echo "Models downloaded to $MODELS_DIR"
else
    echo "Models already exist at $MODELS_DIR"
fi

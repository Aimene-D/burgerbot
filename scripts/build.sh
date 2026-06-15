#!/bin/bash
# Build the burgerbot ros2_ws workspace
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"

echo "==> Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "==> Building workspace: $WS_DIR"
cd "$WS_DIR"
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "==> Build complete. Source your workspace with:"
echo "      source $WS_DIR/install/setup.bash"

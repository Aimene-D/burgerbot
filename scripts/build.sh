#!/bin/bash
# Build the burgerbot ros2_ws workspace
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"

echo "==> Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "==> Building workspace: $WS_DIR"
cd "$WS_DIR"

# Conda's python3 shadows the system python3 needed by ament/cmake.
# Strip conda from PATH for the build so cmake finds the right interpreter.
if [[ -n "$CONDA_PREFIX" || "$PATH" == */conda* || "$PATH" == */miniconda* ]]; then
    echo "    (conda detected — using system python3 for the build)"
    CLEAN_PATH="$(echo "$PATH" | tr ':' '\n' \
        | grep -Ev "/(mini)?conda|/anaconda" \
        | tr '\n' ':' | sed 's/:$//')"
    PYTHONPATH="" PATH="$CLEAN_PATH" \
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

echo ""
echo "==> Build complete. Source your workspace with:"
echo "      source $WS_DIR/install/setup.bash"

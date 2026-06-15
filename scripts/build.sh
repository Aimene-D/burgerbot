#!/bin/bash
# Build the burgerbot ros2_ws workspace
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"

echo "==> Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "==> Building workspace: $WS_DIR"
cd "$WS_DIR"

# Conda's python3 intercepts cmake's Python discovery in two ways:
#   a) it appears first in PATH  → fixed by stripping conda from PATH
#   b) cmake caches the path in build/*/CMakeCache.txt from a previous run
#      → cmake reads the absolute cached path directly, ignoring PATH entirely
# Fix: clear every CMakeCache.txt so cmake re-discovers python3, AND strip
# conda from PATH, AND pin Python3_EXECUTABLE explicitly via cmake args.
if [[ -n "$CONDA_PREFIX" || "$PATH" == */conda* || "$PATH" == */miniconda* ]]; then
    echo "    (conda detected — clearing cmake cache and pinning system python3)"
    find build/ -name "CMakeCache.txt" -delete 2>/dev/null || true

    CLEAN_PATH="$(echo "$PATH" | tr ':' '\n' \
        | grep -Ev "/(mini)?conda|/anaconda" \
        | tr '\n' ':' | sed 's/:$//')"
    # Do NOT clear PYTHONPATH — source setup.bash added the ROS2 site-packages
    # (ament_package, catkin_pkg, etc.) to it. We only need PATH clean so that
    # conda's python3 binary isn't invoked directly; cmake is pinned explicitly.
    PATH="$CLEAN_PATH" \
        colcon build --symlink-install --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPython3_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_EXECUTABLE=/usr/bin/python3
else
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

echo ""
echo "==> Build complete. Source your workspace with:"
echo "      source $WS_DIR/install/setup.bash"

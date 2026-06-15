#!/usr/bin/env bash
# One-time setup: install ROS 2 Jazzy + debug tools inside WSL2 Ubuntu 24.04.
# After this, use scripts/connect_rviz.sh to visualise the robot from Windows.
#
# Run from a WSL2 Ubuntu terminal:
#   cd /mnt/c/Users/<you>/Documents/burgerbot
#   ./scripts/setup_wsl2.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 1. ROS 2 Jazzy apt repository ───────────────────────────────────────────
echo "==> Adding ROS 2 Jazzy repository..."
sudo apt-get update -q
sudo apt-get install -y -q curl software-properties-common
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu noble main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ── 2. Install packages ──────────────────────────────────────────────────────
echo "==> Installing ROS 2 Jazzy desktop + debug tools..."
sudo apt-get update -q
sudo apt-get install -y -q \
    ros-jazzy-desktop \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-rqt-tf-tree \
    python3-colcon-common-extensions

# ── 3. Build the workspace ───────────────────────────────────────────────────
echo "==> Building workspace..."
source /opt/ros/jazzy/setup.bash
cd "$SCRIPT_DIR/../ros2_ws"

# Conda intercepts python3, making cmake use its Python instead of the system
# one that has catkin_pkg. Strip conda paths for the duration of the build.
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

# ── 4. Mirrored networking check ─────────────────────────────────────────────
echo ""
WSLCONFIG="$USERPROFILE/.wslconfig"
if grep -q "networkingMode=mirrored" "$WSLCONFIG" 2>/dev/null; then
    echo "==> WSL2 mirrored networking: already configured."
else
    echo "==> WSL2 mirrored networking: NOT configured — DDS will not work!"
    echo "    Add this to %USERPROFILE%\\.wslconfig on Windows and run 'wsl --shutdown':"
    echo ""
    echo "        [wsl2]"
    echo "        networkingMode=mirrored"
    echo ""
fi

echo "==> Setup complete. Use from this WSL2 terminal:"
echo "      ./scripts/connect_rviz.sh <ROBOT_IP>        # RViz"
echo "      ./scripts/connect_rviz.sh <ROBOT_IP> rqt    # rqt"

#!/usr/bin/env bash
# Connect this machine (native ROS 2 / WSL2) to the remote robot and launch a GUI tool.
#
# Usage:
#   scripts/connect_rviz.sh <ROBOT_IP> [rviz|rqt]   (default: rviz)
#
# Examples:
#   scripts/connect_rviz.sh 192.168.1.42
#   scripts/connect_rviz.sh 192.168.1.42 rqt
#
# Prerequisites on this machine / in WSL2:
#   sudo apt install ros-jazzy-desktop ros-jazzy-rqt-common-plugins ros-jazzy-rqt-tf-tree
#   ./scripts/build.sh
#
# WSL2 users: WSLg (Windows 11) provides the display automatically.
# Linux users: DISPLAY must be set (usually :0).
set -e

ROBOT_IP="${1:?Usage: $0 <ROBOT_IP> [rviz|rqt]}"
TOOL="${2:-rviz}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"
CYCLONE_XML="$SCRIPT_DIR/../docker/cyclone_peers.xml"

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "Workspace not built. Run ./scripts/build.sh first." >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROBOT_IP="$ROBOT_IP"
export CYCLONEDDS_URI="file://${CYCLONE_XML}"

echo "Connecting to robot at ${ROBOT_IP} (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"

case "$TOOL" in
    rviz) exec ros2 launch burgerbot debug.launch.py ;;
    rqt)  exec rqt ;;
    *)    echo "Unknown tool '$TOOL'. Use 'rviz' or 'rqt'." >&2; exit 1 ;;
esac

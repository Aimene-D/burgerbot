#!/usr/bin/env bash
# Launch RViz on this machine and connect it to a remote robot's ROS 2 graph.
#
# Usage:  scripts/connect_rviz.sh <ROBOT_IP> [extra ros2 launch args...]
# Example: scripts/connect_rviz.sh 192.168.1.42
#
# Prerequisites on this machine:
#   - ROS 2 Jazzy installed  (sudo apt install ros-jazzy-desktop)
#   - Workspace built once   (./scripts/build.sh)
#   - Same LAN as the robot
#
# The robot must be running (in separate terminals on the remote host):
#   docker compose -f docker/docker-compose.yml run --rm bringup
#   docker compose -f docker/docker-compose.yml run --rm slam
set -e

ROBOT_IP="${1:?Usage: $0 <ROBOT_IP> [extra launch args]}"
shift  # remaining args forwarded to ros2 launch

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
exec ros2 launch burgerbot debug.launch.py "$@"

#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================================="
echo "    Building Autonomous Robotics Simulation Workspace     "
echo "=========================================================="

cd "$WORKSPACE_ROOT/ros2_ws"

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "=========================================================="
echo "                   Build Succeeded!                       "
echo "  Run: source ros2_ws/install/setup.bash                  "
echo "=========================================================="

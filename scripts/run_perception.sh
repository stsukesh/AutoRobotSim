#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$WORKSPACE_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/ros2_ws/install/setup.bash"
fi

TARGET_COLOR="${1:-red}"
ENABLE_SERVOING="${2:-false}"

echo ">>> Launching OpenCV Visual Perception Node (Tracking: $TARGET_COLOR, Servoing: $ENABLE_SERVOING)..."
ros2 launch auto_robot_perception perception.launch.py \
    target_color:="$TARGET_COLOR" \
    enable_servoing:="$ENABLE_SERVOING"

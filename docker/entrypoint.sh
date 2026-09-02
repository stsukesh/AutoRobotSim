#!/bin/bash
set -e

# Source ROS 2 Jazzy setup
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
    source "/workspace/ros2_ws/install/setup.bash"
fi

exec "$@"

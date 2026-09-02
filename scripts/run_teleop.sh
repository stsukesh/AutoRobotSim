#!/bin/bash
set -e

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

echo "=========================================================="
echo "      TELEOP KEYBOARD CONTROLLER (Publishing /cmd_vel)    "
echo "  Use i, j, k, l, comma keys to steer the robot.         "
echo "=========================================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard

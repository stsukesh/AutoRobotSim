#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================================="
echo "    Starting ROS 2 Humble Simulation Container            "
echo "=========================================================="

# Grant X11 access for GUI display
xhost +local:root || true
xhost +local:robot || true

cd "$WORKSPACE_ROOT/docker"
docker compose up -d --build

echo ""
echo "Container started successfully!"
echo "To enter the ROS 2 container shell, run:"
echo "  docker exec -it auto_robot_sim_container bash"
echo ""
echo "Or attach using VSCode Remote Containers / Dev Containers."

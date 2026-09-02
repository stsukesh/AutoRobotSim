#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================================="
echo "    Running Autonomous Robotics Simulation Test Suite     "
echo "=========================================================="

cd "$WORKSPACE_ROOT"

# Run pytest on all tests
pytest -v tests/

echo "=========================================================="
echo "               All Unit Tests Passed!                     "
echo "=========================================================="

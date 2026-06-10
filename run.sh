#!/bin/bash
# run.sh — activate venv, source workspace, and launch the full system.
#
# Usage:
#   bash run.sh                  # launch with defaults (sorts all 3 colours)
#
# First-time setup:
#   bash setup_env.sh

set -e

VENV_DIR="$HOME/sort_robo_env"
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -d "$VENV_DIR" ]; then
    echo "[run] Virtual environment not found at $VENV_DIR"
    echo "[run] Run 'bash setup_env.sh' first."
    exit 1
fi

echo "[run] Activating venv ..."
source "$VENV_DIR/bin/activate"

echo "[run] Sourcing ROS2 Humble ..."
source /opt/ros/humble/setup.bash

echo "[run] Sourcing workspace ..."
source "$WS_DIR/install/setup.bash"

echo "[run] Launching pick_and_place ..."
ros2 launch sort_robo_bringup pick_and_place.launch.py "$@"

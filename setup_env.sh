#!/bin/bash
# setup_env.sh — create a Python virtual environment for sort_robo_ws
#
# Uses --system-site-packages so ROS2 (rclpy, sensor_msgs, cv_bridge, etc.)
# and the system cv2 (python3-opencv 4.5.4) are inherited. Then installs
# numpy<2 inside the venv to shadow the globally-installed numpy 2.x that
# breaks the apt-installed cv2.
#
# Run ONCE:
#   bash setup_env.sh
#
# Then activate for every new terminal:
#   source ~/sort_robo_env/bin/activate

set -e

VENV_DIR="$HOME/sort_robo_env"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check venv support is installed
if ! python3 -c "import ensurepip" 2>/dev/null; then
    echo "[setup_env] ERROR: python3-venv is not installed."
    echo "  Run:  sudo apt install python3.10-venv"
    echo "  Then re-run this script."
    exit 1
fi

echo "[setup_env] Creating venv at $VENV_DIR ..."
python3 -m venv --system-site-packages "$VENV_DIR"

echo "[setup_env] Activating venv ..."
source "$VENV_DIR/bin/activate"

echo "[setup_env] Installing Python requirements ..."
pip install --upgrade pip --quiet
pip install -r "$SCRIPT_DIR/requirements.txt"

echo ""
echo "========================================="
echo "  Environment ready: $VENV_DIR"
echo "========================================="
echo ""
echo "For every NEW terminal, activate with:"
echo "  source ~/sort_robo_env/bin/activate"
echo ""
echo "Then build (first time or after code changes):"
echo "  cd ~/sort_robo_ws"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo ""
echo "Then run:"
echo "  bash ~/sort_robo_ws/run.sh"
echo "  (or: ros2 launch sort_robo_bringup pick_and_place.launch.py)"

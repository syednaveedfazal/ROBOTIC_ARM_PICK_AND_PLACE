#!/bin/bash
set -e

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source the built workspace (if it exists)
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Forward Gazebo resource path so models are found
export GZ_SIM_RESOURCE_PATH=/ros2_ws/install/share:$GZ_SIM_RESOURCE_PATH

exec "$@"

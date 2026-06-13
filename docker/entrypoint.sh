#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi
exec "$@"
docker compose -f docker/docker-compose.yml build --no-cache
docker compose -f docker/docker-compose.yml up
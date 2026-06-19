#!/usr/bin/env bash

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"

echo "Starting the MQTT and ROS 2 services..."
docker compose up -d

echo "Building the ROS 2 workspace and launching the demo..."
docker compose exec ros2 bash -lc '
  set -e
  cd /workspace
  colcon build --symlink-install
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  exec ros2 launch resilient_amr_dispatch demo.launch.py "$@"
' bash "$@"

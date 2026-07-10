#!/usr/bin/env bash
# Run the ROS 2 IMU launch file after systemd has prepared the IIO buffer.
# ROS 2 Humble setup scripts intentionally inspect optional unset variables.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/cat/ros2_ws/install/setup.bash

exec ros2 launch lsm6dsox_ros imu.launch.py

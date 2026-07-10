#!/usr/bin/env bash
# Run the IMU node directly so systemd receives the node's actual exit status.
# ROS 2 Humble setup scripts intentionally inspect optional unset variables.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/cat/ros2_ws/install/setup.bash

exec /home/cat/ros2_ws/install/lsm6dsox_ros/lib/lsm6dsox_ros/imu_publisher \
  --ros-args \
  -r __node:=lsm6dsox_imu_publisher \
  -r /imu/data:=/imu/data

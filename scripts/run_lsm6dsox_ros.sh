#!/usr/bin/env bash
# Run the IMU node directly so systemd receives the node's actual exit status.
# ROS 2 Humble setup scripts intentionally inspect optional unset variables.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/cat/ros2_ws/install/setup.bash

calibration_file="${LSM6DSOX_CALIBRATION_FILE:-$HOME/.config/lsm6dsox/imu_calibration.yaml}"
ros_args=(--ros-args -r __node:=lsm6dsox_imu_publisher -r /imu/data:=/imu/data)

if [[ -n "${FRAME_ID:-}" ]]; then
  ros_args+=(-p "frame_id:=${FRAME_ID}")
fi

if [[ -f "$calibration_file" ]]; then
  ros_args+=(--params-file "$calibration_file")
fi

exec /home/cat/ros2_ws/install/lsm6dsox_ros/lib/lsm6dsox_ros/imu_publisher \
  "${ros_args[@]}"

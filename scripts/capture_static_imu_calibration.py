#!/usr/bin/env python3
"""Capture stationary IMU statistics and write ROS 2 calibration parameters."""

import argparse
import math
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class RunningStats:
    def __init__(self):
        self.count = 0
        self.mean = [0.0, 0.0, 0.0]
        self.m2 = [0.0, 0.0, 0.0]

    def add(self, values):
        self.count += 1
        for index, value in enumerate(values):
            delta = value - self.mean[index]
            self.mean[index] += delta / self.count
            self.m2[index] += delta * (value - self.mean[index])

    @property
    def variance(self):
        if self.count < 2:
            return [0.0, 0.0, 0.0]
        return [value / (self.count - 1) for value in self.m2]


class StaticCalibrationCapture(Node):
    def __init__(self, topic):
        super().__init__("lsm6dsox_static_calibration_capture")
        self.gyro = RunningStats()
        self.accel = RunningStats()
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, topic, self.callback, qos)

    def callback(self, message):
        self.gyro.add([message.angular_velocity.x, message.angular_velocity.y,
                       message.angular_velocity.z])
        self.accel.add([message.linear_acceleration.x,
                        message.linear_acceleration.y,
                        message.linear_acceleration.z])


def yaml_vector(values):
    return "[" + ", ".join(f"{value:.12g}" for value in values) + "]"


def main():
    parser = argparse.ArgumentParser(
        description="Capture stationary IMU gyro bias and covariance values.")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--topic", default="/imu/data")
    parser.add_argument("--output", default=os.path.expanduser(
        "~/.config/lsm6dsox/imu_calibration.yaml"))
    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be positive")

    rclpy.init()
    node = StaticCalibrationCapture(args.topic)
    deadline = node.get_clock().now().nanoseconds + int(args.duration * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.gyro.count < 100:
        raise RuntimeError(f"only captured {node.gyro.count} samples")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        "# Generated while the IMU was stationary.\n"
        "# accel_stationary_mean includes gravity; do not use it as a bias\n"
        "# until the sensor mounting orientation is known.\n"
        "lsm6dsox_imu_publisher:\n"
        "  ros__parameters:\n"
        f"    gyro_bias: {yaml_vector(node.gyro.mean)}\n"
        "    angular_velocity_covariance_diagonal: "
        f"{yaml_vector(node.gyro.variance)}\n"
        "    linear_acceleration_covariance_diagonal: "
        f"{yaml_vector(node.accel.variance)}\n"
        f"# sample_count: {node.gyro.count}\n"
        f"# accel_stationary_mean_mps2: {yaml_vector(node.accel.mean)}\n"
    )
    print(f"captured {node.gyro.count} stationary samples")
    print(f"gyro_bias_radps={yaml_vector(node.gyro.mean)}")
    print(f"accel_mean_mps2={yaml_vector(node.accel.mean)}")
    print(f"wrote {output}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

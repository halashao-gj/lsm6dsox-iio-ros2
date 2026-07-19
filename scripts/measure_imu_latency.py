#!/usr/bin/env python3

"""Measure ROS 2 IMU receive rate, timestamp spacing, and delivery latency."""

import argparse
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def percentile(values, fraction):
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


class ImuLatency(Node):
    def __init__(self, target):
        super().__init__("imu_latency_measurement")
        self.target = target
        self.latencies_ms = []
        self.arrivals_ns = []
        self.stamps_ns = []
        self.create_subscription(
            Imu, "/imu/data", self.callback, qos_profile_sensor_data
        )

    def callback(self, msg):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        arrival_ns = time.time_ns()
        latency_ms = (arrival_ns - stamp_ns) / 1_000_000.0

        # Reject samples that clearly use a different clock domain.
        if -1000.0 < latency_ms < 10000.0:
            self.latencies_ms.append(latency_ms)
            self.arrivals_ns.append(arrival_ns)
            self.stamps_ns.append(stamp_ns)


def print_distribution(prefix, values):
    print(f"{prefix}_min_ms={min(values):.3f}")
    print(f"{prefix}_mean_ms={statistics.fmean(values):.3f}")
    print(f"{prefix}_median_ms={statistics.median(values):.3f}")
    print(f"{prefix}_std_ms={statistics.pstdev(values):.3f}")
    print(f"{prefix}_p95_ms={percentile(values, 0.95):.3f}")
    print(f"{prefix}_p99_ms={percentile(values, 0.99):.3f}")
    print(f"{prefix}_max_ms={max(values):.3f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=1000)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    if args.samples < 2:
        parser.error("--samples must be at least 2")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    rclpy.init()
    node = ImuLatency(args.samples)
    deadline = time.monotonic() + args.timeout

    try:
        while len(node.latencies_ms) < node.target and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        count = len(node.latencies_ms)
        if count < 2:
            raise RuntimeError(f"only received {count} valid samples")

        duration_s = (node.arrivals_ns[-1] - node.arrivals_ns[0]) / 1_000_000_000.0
        timestamp_deltas_ms = [
            (current - previous) / 1_000_000.0
            for previous, current in zip(node.stamps_ns, node.stamps_ns[1:])
        ]
        duplicate_timestamps = sum(delta == 0 for delta in timestamp_deltas_ms)
        backward_timestamps = sum(delta < 0 for delta in timestamp_deltas_ms)

        print(f"samples={count}")
        print(f"duration_s={duration_s:.6f}")
        print(f"receive_rate_hz={(count - 1) / duration_s:.3f}")
        print_distribution("latency", node.latencies_ms)
        print_distribution("timestamp_delta", timestamp_deltas_ms)
        print(f"duplicate_timestamps={duplicate_timestamps}")
        print(f"backward_timestamps={backward_timestamps}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

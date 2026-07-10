# LSM6DSOX Linux IIO Driver + ROS 2 IMU Pipeline

This project brings an LSM6DSOX 6-axis IMU up on an RK3576 LubanCat board using
a Linux I2C/IIO kernel module, then publishes the sensor stream into ROS 2 as
`sensor_msgs/msg/Imu`.

The goal is to demonstrate the full path from board-level device tree binding to
Linux IIO sensor interfaces, interrupt-driven buffered sampling, and a ROS 2
robotics-facing data pipeline.

## What is included

- `lsm6dsox_minimal/`: RK3576 I2C7-M1 device tree overlay for the LSM6DSOX.
- `lsm6dsox_driver/`: out-of-tree Linux I2C/IIO driver.
- `IIO-raw/`: small userspace reader for `/dev/iio:deviceX` buffered frames.
- `lsm6dsox_ros/`: ROS 2 Humble C++ publisher for `/imu/data`.
- `docs/validation.md`: hardware and ROS 2 validation summary.

## Implemented features

- Device tree match with `compatible = "study,lsm6dsox-minimal"`.
- I2C probe with `WHO_AM_I=0x6c` verification.
- Software reset, BDU enable, and 104 Hz accel/gyro configuration.
- IIO accel and angular velocity channels for X/Y/Z axes.
- `raw`, `scale`, and `sampling_frequency` IIO attributes.
- Data-ready INT1 interrupt routed to a threaded IRQ.
- IIO trigger and triggered buffer support.
- 24-byte buffered scan frame: accel XYZ, gyro XYZ, timestamp.
- ROS 2 publisher with IIO device auto-detection and launch file support.

## Validated hardware

- Board: RK3576 LubanCat
- Kernel: Linux 6.1.99-rk3576
- OS: Ubuntu 22.04.5 LTS
- ROS 2: Humble
- IMU: LSM6DSOX on I2C address `0x6a`

Observed validation results:

- IIO device registered as `name=lsm6dsox`.
- Static acceleration magnitude measured around `9.93 m/s^2`.
- `/imu/data` published at about `49.99 Hz` in the earlier sysfs polling node.
- IIO triggered buffer successfully produced 24-byte frames from `/dev/iio:device1`.
- Madgwick filter and rosbag recording were validated on the board.

## Build

Adjust paths if your kernel SDK is not in the default LubanCat SDK location.

```sh
make -C lsm6dsox_minimal
make -C lsm6dsox_driver KDIR=/path/to/kernel CROSS_COMPILE=/path/to/aarch64-none-linux-gnu-
make -C IIO-raw CROSS_COMPILE=/path/to/aarch64-none-linux-gnu-
```

Build the ROS 2 package:

```sh
source /opt/ros/humble/setup.bash
colcon build --packages-select lsm6dsox_ros
```

## Run

Install the device tree overlay as described in
`lsm6dsox_minimal/README.md`, load `lsm6dsox_driver.ko`, then enable the IIO
buffer or start the ROS 2 node.

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lsm6dsox_ros imu.launch.py
```

Raw buffered frame test:

```sh
sudo ./IIO-raw/iio_buffer_reader /dev/iio:device1 20
```

## Current limitations

- The driver is still an out-of-tree learning driver.
- Register access is direct SMBus access rather than `regmap`.
- ROS 2 diagnostics, calibration, covariance parameters, and automatic module
  loading are planned follow-ups.

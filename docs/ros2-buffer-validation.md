# ROS 2 IIO Buffer Pipeline Validation

Date validated on board: 2026-06-06 board time / 2026-07-10 host time

This validation proves that the ROS 2 publisher consumes the IIO character
device buffer directly instead of polling individual sysfs `*_raw` files.

## Pipeline

```text
LSM6DSOX
 -> INT1 data-ready IRQ
 -> IIO trigger
 -> IIO triggered buffer
 -> /dev/iio:device1
 -> lsm6dsox_ros imu_publisher
 -> /imu/data
```

## Board Build

The local ROS 2 package was synced to the board and rebuilt from a clean package
build/install directory:

```text
/home/cat/ros2_ws/src/lsm6dsox_ros
colcon build --packages-select lsm6dsox_ros
Summary: 1 package finished
```

The source used on the board includes the buffered IIO implementation:

```text
reading buffered IIO frames from %s (sysfs: %s)
kFrameSize = 24
poll()
```

## Runtime Setup

Validated IIO device:

```text
/sys/bus/iio/devices/iio:device1/name = lsm6dsox
/dev/iio:device1
trigger0/name = lsm6dsox-dev1
```

Before launching ROS 2, the IIO buffer was enabled with all scan elements,
`lsm6dsox-dev1` as the trigger, and a buffer length of 128.

The ROS 2 node log confirmed that it opened the IIO character device:

```text
reading buffered IIO frames from /dev/iio:device1 (sysfs: /sys/bus/iio/devices/iio:device1)
accel_scale=0.000598205 gyro_scale=0.000152716
```

## ROS 2 Topic Output

Topics observed:

```text
/imu/data
/parameter_events
/rosout
```

Sample `/imu/data` message:

```text
angular_velocity:
  x: -0.002596172
  y: 0.009926540000000001
  z: -0.0038179
linear_acceleration:
  x: 0.024526404999999998
  y: 0.126221255
  z: 9.97327376
```

Observed topic rate:

```text
average rate: about 102.5 Hz
min: 0.004s
max: 0.017s
std dev: about 0.001s
```

This is close to the driver configuration of 104 Hz.

## rosbag Evidence

Recorded bag:

```text
/home/cat/bags/lsm6dsox_iio_buffer_20260606_002822
```

Bag info:

```text
Duration: 6.053645111s
Messages: 622
Topic: /imu/data
Type: sensor_msgs/msg/Imu
Storage: sqlite3
Bag size: 265.0 KiB
```

## Cleanup

After validation, the ROS 2 node was stopped and the IIO buffer was disabled:

```text
buffer=0
```

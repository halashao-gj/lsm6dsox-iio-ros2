# Validation Notes

Date of main hardware validation: 2026-06-26.

## Kernel and IIO

The LSM6DSOX module was loaded on an RK3576 LubanCat board running
`Linux 6.1.99-rk3576`.

Key observed kernel logs:

```text
my_lsm6dsox 7-006a: my probe entered, addr=0x6a
my_lsm6dsox 7-006a: WHO_AM_I=0x6c
my_lsm6dsox 7-006a: BDU enabled, CTRL3_C=0x44
my_lsm6dsox 7-006a: accelerometer configured, CTRL1_XL=0x40
my_lsm6dsox 7-006a: gyroscope configured, CTRL2_G=0x40
my_lsm6dsox 7-006a: IIO device registered
```

Observed IIO sysfs state:

```text
/sys/bus/i2c/devices/7-006a
compatible = study,lsm6dsox-minimal
/sys/bus/iio/devices/iio:device1 name=lsm6dsox
```

Sample raw values:

```text
in_accel_x_raw=3231
in_accel_y_raw=-646
in_accel_z_raw=16324
in_anglvel_x_raw=-17
in_anglvel_y_raw=53
in_anglvel_z_raw=-24
in_accel_scale=0.000598205
in_anglvel_scale=0.000152716
```

## Triggered Buffer

The data-ready interrupt path was validated as:

```text
INT1 data-ready -> GPIO IRQ -> IIO trigger -> pollfunc -> IIO buffer
```

Observed logs:

```text
my_lsm6dsox 7-006a: IIO trigger registered
my_lsm6dsox 7-006a: IIO triggered buffer setup complete
my_lsm6dsox 7-006a: data-ready irq registered: irq=107
my_lsm6dsox 7-006a: IIO device registered
```

The IIO trigger appeared as `lsm6dsox-dev1`, and `/dev/iio:device1` produced
24-byte frames containing accel XYZ, gyro XYZ, and a timestamp.

## ROS 2

Validated on ROS 2 Humble:

```text
package: lsm6dsox_ros
node: lsm6dsox_imu_publisher
topic: /imu/data
message: sensor_msgs/msg/Imu
```

Observed earlier topic rate:

```text
average rate: 49.99 Hz
min: 0.006s
max: 0.036s
std dev: about 0.0017s
```

Observed sample message values:

```text
angular_velocity:
  x: -0.002901604
  y: 0.007788516
  z: -0.002901604
linear_acceleration:
  x: 2.364704365
  y: -0.588035515
  z: 9.64665383
```

The pipeline was also connected to `imu_filter_madgwick`, producing
`/imu/data_filtered`, and recorded with rosbag for replay validation.

## modprobe Deployment

On 2026-07-10, the driver was installed into the board module tree:

```text
/lib/modules/6.1.99-rk3576/extra/lsm6dsox_driver.ko
```

`depmod -a`, `modprobe lsm6dsox_driver`, and boot-time loading through
`/etc/modules-load.d/lsm6dsox.conf` were validated. After reboot, the module
auto-loaded, probed `7-006a`, registered the IIO device, and produced 48 bytes
from `/dev/iio:device1`, matching two 24-byte buffered scan frames.

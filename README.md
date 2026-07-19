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
- `scripts/`: IIO buffer enable/disable helpers for the board runtime.
- `config/udev/`: persistent non-root IIO device access rule.
- `systemd/`: manual-start ROS 2 service with failure recovery.
- `docs/validation.md`: hardware and ROS 2 validation summary.
- `docs/modprobe-deployment.md`: module installation, `modprobe`, and
  boot-time auto-load validation.
- `docs/ros2-buffer-validation.md`: ROS 2 validation using `/dev/iio:device1`
  buffered frames.
- `docs/iio-buffer-service-incident.md`: systemd/IIO buffer fault analysis,
  lifecycle fix, and interview-oriented troubleshooting notes.
- `docs/imu-calibration.md`: stationary gyro-bias capture, covariance
  parameters, and frame configuration.
- `docs/regmap-refactor.md`: rationale, migration mapping, and validation for
  the driver register-access refactor.
- `docs/performance/fifo-watermark-validation.md`: hardware FIFO design,
  correctness checks, and watermark 1 versus 8 performance measurements.
- `docs/dynamic-scale-validation.md`: runtime accel/gyro full-scale mappings,
  IIO sysfs commands, register verification, and ROS 2 validation.
- `docs/runtime-pm-suspend-validation.md`: runtime autosuspend design, direct
  access and buffer PM ownership, deep suspend/resume, and board measurements.
- `docs/driver-code-reading-guide.md`: Chinese architecture overview and a
  call-chain-based walkthrough of the complete kernel driver.

## Implemented features

- Device tree match with `compatible = "study,lsm6dsox-minimal"`.
- I2C probe with `WHO_AM_I=0x6c` verification.
- Standard `regmap-i2c` register access with read, write, masked update, and
  bulk-read paths.
- Software reset, BDU enable, and coherent 26/52/104/208 Hz accel/gyro ODR
  configuration through IIO sysfs.
- Runtime IIO scale configuration for accel +/-2/4/8/16 g and gyro
  +/-125/250/500/1000/2000 dps, with register verification and rollback.
- IIO accel and angular velocity channels for X/Y/Z axes.
- `raw`, `scale`, and `sampling_frequency` IIO attributes.
- FIFO-watermark INT1 interrupt with tagged accel/gyro/hardware-timestamp
  batch reads.
- IIO trigger and triggered buffer support with a configurable hardware FIFO
  watermark (default: 4 combined scans).
- 24-byte buffered scan frame: accel XYZ, gyro XYZ, timestamp.
- Calibrated 32-bit hardware timestamp ticks with rollover extension,
  backward/gap detection, and mapping into the selected IIO clock domain.
- Full FIFO overflow and I2C error recovery, explicit data-discontinuity and
  unknown-loss counters, FIFO tag-integrity checks, and repeatable buffer
  enable/disable rollback.
- Runtime PM with a 2-second autosuspend delay, direct-access reference
  handling, ODR power-down, and buffer-lifetime PM ownership.
- Deep system suspend/resume with IRQ synchronization, cached ODR/scale and
  FIFO restoration, bounded I2C resume retries, and explicit discontinuity
  reporting.
- ROS 2 publisher with IIO device auto-detection and launch file support.
- ROS 2 `/diagnostics` reporting with OK/WARN/ERROR FIFO health states.
- Parameterized frame ID, stationary gyro-bias correction, and covariance
  loading from a per-board calibration YAML.
- A manual-start systemd service and one-command IIO-to-ROS validation.

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
- `modprobe` deployment and boot-time auto-loading validated.
- ROS 2 publisher directly consumed `/dev/iio:device1` buffered frames and
  published `/imu/data` at about `102.5 Hz`.
- Madgwick filter and rosbag recording were validated on the board.
- The post-regmap systemd/IIO/ROS validation passed after a clean board reboot.
- At the same 104 Hz ODR, changing the hardware FIFO watermark from 1 to 8
  reduced measured IRQs by 88.9% and I2C messages by 74.8% in 5-second tests.
- The FIFO v2 default watermark 4 delivered about 104 scans/s with 26 IRQ/s;
  its enlarged burst and read-path cleanup kept the FIFO path to two I2C calls
  per IRQ.
- FIFO v3 parsed the LSM6DSOX hardware timestamp tag at all four supported
  ODRs. A forced overflow was discarded and recovered with an explicit ROS 2
  WARN diagnostic instead of synthetic samples.
- A 1,000-message local ROS 2 integration run at 104 Hz/watermark 4 delivered
  102.811 messages/s with 51.998 ms mean, 65.325 ms P95, and 68.652 ms maximum
  sample-to-subscriber latency. The publisher consumed 0.599% of one CPU in a
  steady-state 15-second window, with all FIFO and timestamp error counters at
  zero. Timestamp spacing was a stable 9.758 ms; absolute startup alignment is
  documented as a remaining measurement limitation.
- All four accel and five gyro scales were written and verified against
  `CTRL1_XL`/`CTRL2_G`; invalid and active-buffer writes were rejected without
  changing the previous scale.
- Runtime idle powered down accel, gyro, and timestamp registers; raw access
  woke them and buffer enable held a PM reference until buffer disable.
- Deep suspend/resume passed with the buffer both disabled and enabled. The
  active test restored 52 Hz, maximum accel/gyro ranges, watermark 4, FIFO and
  INT1, then continued streaming without overflow, backward timestamps, or
  unreported discontinuity.

## v1.0.0 Release Scope

The v1.0.0 release covers the complete hardware-to-ROS pipeline:

```text
Device Tree → I2C probe/regmap → FIFO watermark IRQ → IIO buffer
           → /dev/iio:deviceX → ROS 2 Imu + diagnostics → systemd service
```

Before a deployment or release, run this final board-side acceptance check:

```sh
cd /home/cat/ros2_ws/src/lsm6dsox_ros
./scripts/validate_lsm6dsox_pipeline.sh
```

The systemd unit is installed but deliberately remains disabled at boot. This
keeps IMU collection an explicit operational choice; enable it only after the
target deployment requires an always-on sensor stream.

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
buffer before starting the ROS 2 node.

Run these commands from the repository root on the board. The helper finds the
`name=lsm6dsox` IIO device and its matching trigger automatically. Install the
udev rule in `docs/udev-deployment.md` once before first use; it makes IIO
device access persistent for members of the `iio` group. The helper still uses
`sudo` for privileged IIO sysfs settings.

```sh
./scripts/enable_lsm6dsox_buffer.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lsm6dsox_ros imu.launch.py

# After stopping the ROS 2 node
./scripts/disable_lsm6dsox_buffer.sh
```

Override the default 128-frame kernel buffer or the default 4-scan hardware
FIFO watermark if needed:

```sh
BUFFER_LENGTH=256 ./scripts/enable_lsm6dsox_buffer.sh
FIFO_WATERMARK=4 ./scripts/enable_lsm6dsox_buffer.sh
```

The watermark trades batch latency for bus and interrupt overhead. At 104 Hz,
the default watermark 4 represents roughly 38.5 ms of samples per full batch;
use `FIFO_WATERMARK=2` when lower delivery latency matters more than IRQ rate.

Set sensor full scale while the buffer is disabled. IIO exposes SI scale per
raw LSB, so use a value listed by the corresponding `_available` attribute:

```sh
cat /sys/bus/iio/devices/iio:device1/in_accel_scale_available
cat /sys/bus/iio/devices/iio:device1/in_anglvel_scale_available
echo 0.004785645 | sudo tee \
  /sys/bus/iio/devices/iio:device1/in_accel_scale
```

For a manually controlled service that restarts the ROS 2 node after failures,
see `docs/systemd-deployment.md`. It is deliberately not enabled at boot.

Raw buffered frame test:

```sh
sudo ./IIO-raw/iio_buffer_reader /dev/iio:device1 20
```

## Current limitations

- The driver is still an out-of-tree learning driver.
- The board image's `iio-sensor-proxy` periodically reads raw channels and can
  keep postponing autosuspend; stop it when measuring true idle power.
- Hardware timestamp rollover extension is implemented but still needs a
  continuous 30-hour rollover validation run.
- Forced FIFO overflow is covered; I2C fault injection still needs dedicated
  adapter-level testing.
- Accelerometer six-face bias calibration and installation-axis calibration
  are not implemented; stationary accel means are recorded but gravity is not
  removed.
- The systemd service is not enabled at boot by default.

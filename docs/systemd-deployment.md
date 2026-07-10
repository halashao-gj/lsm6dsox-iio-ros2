# Manual-Start ROS 2 systemd Service

`lsm6dsox-ros.service` is intentionally installed but not enabled. Start it
only when an IMU data stream is needed. If the ROS 2 process exits with an
error, systemd waits two seconds and starts it again.

## Install

Run from the repository root on the LubanCat after building the ROS 2 workspace
and installing the udev rule:

```sh
./scripts/install_lsm6dsox_ros_service.sh
```

The unit expects this board workspace layout:

```text
/home/cat/ros2_ws
/home/cat/ros2_ws/src/lsm6dsox_ros
```

It does not run `systemctl enable`, so the service does not start at boot.

## Daily Use

```sh
sudo systemctl start lsm6dsox-ros.service
sudo systemctl status lsm6dsox-ros.service
sudo journalctl -u lsm6dsox-ros.service -f
sudo systemctl stop lsm6dsox-ros.service
```

Verify the publisher from a ROS 2 shell:

```sh
source /opt/ros/humble/setup.bash
source /home/cat/ros2_ws/install/setup.bash
ros2 topic hz /imu/data
```

## Runtime Behavior

On each start, systemd enables the IIO buffer as root, then launches ROS 2 as
the `cat` user. The udev deployment grants `cat` read access to the IIO device.
On an unexpected ROS 2 process failure, systemd runs the buffer cleanup step,
waits two seconds, then enables the buffer and starts the node again.

An explicit `systemctl stop` is treated as a normal stop and does not restart
the service. The cleanup step disables the IIO buffer in both cases.

## Board Validation

Validated on the RK3576 LubanCat with systemd 249:

```text
systemctl is-enabled lsm6dsox-ros.service -> disabled
systemctl start -> active (running)
SIGKILL of the main launch process -> automatic restart after 2 seconds
systemctl stop -> inactive and IIO buffer=0
```

The service process runs as `cat`, including membership in the `iio` group, and
the setup/cleanup scripts run with the elevated privileges required for IIO
sysfs writes.

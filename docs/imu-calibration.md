# IMU 静止标定与消息质量

`imu_publisher` 在 ROS 节点层提供 frame、gyro bias 和协方差参数。内核 IIO
驱动仍只输出传感器原始 SI 数据，因此重新标定不需要重载内核模块。

## 静止采集

保持 IMU 静止 30–60 秒，然后启动服务并运行：

```sh
sudo systemctl start lsm6dsox-ros.service
source /opt/ros/humble/setup.bash
source /home/cat/ros2_ws/install/setup.bash
./src/lsm6dsox_ros/scripts/capture_static_imu_calibration.py --duration 60
sudo systemctl restart lsm6dsox-ros.service
```

采集工具订阅 `/imu/data`，使用 Welford 在线算法计算均值和无偏样本方差，并写入：

```text
~/.config/lsm6dsox/imu_calibration.yaml
```

其中 gyro 均值作为 `gyro_bias`，发布时会从角速度中扣除；gyro 与 accel 方差写入
相应的 3×3 对角协方差。

静止 accel 均值包含重力，工具会保留为注释中的参考值，但不会把它自动作为 bias
扣除。必须先确定传感器安装方向，或完成六面标定，才能得到可安全应用的 accel bias。

## 启动时加载

`scripts/run_lsm6dsox_ros.sh` 默认查找：

```text
$HOME/.config/lsm6dsox/imu_calibration.yaml
```

存在时自动以 ROS `--params-file` 加载。可以使用
`LSM6DSOX_CALIBRATION_FILE=/path/to/file` 覆盖位置。

## Frame 参数

launch 文件的默认 frame 为 `imu_link`：

```sh
ros2 launch lsm6dsox_ros imu.launch.py frame_id:=imu_calibrated
```

systemd 服务可通过 manager 环境设置一次性覆盖：

```sh
sudo systemctl set-environment FRAME_ID=imu_calibrated
sudo systemctl restart lsm6dsox-ros.service
sudo systemctl unset-environment FRAME_ID
```

长期部署建议通过 systemd drop-in 或专用 EnvironmentFile 设置，而不是依赖临时
manager 环境。

## 参数格式

```yaml
lsm6dsox_imu_publisher:
  ros__parameters:
    gyro_bias: [0.0, 0.0, 0.0]
    angular_velocity_covariance_diagonal: [0.0, 0.0, 0.0]
    linear_acceleration_covariance_diagonal: [0.0, 0.0, 0.0]
```

方向未知仍用 `orientation_covariance[0] = -1` 表示节点不提供姿态估计。

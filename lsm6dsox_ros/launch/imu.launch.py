from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_path = LaunchConfiguration("device_path")
    frame_id = LaunchConfiguration("frame_id")
    imu_topic = LaunchConfiguration("imu_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "device_path",
            default_value="",
            description="IIO sysfs directory; empty means auto-detect by name.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="imu_link",
            description="Frame id written into the IMU message header.",
        ),
        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu/data",
            description="Output topic for sensor_msgs/msg/Imu messages.",
        ),
        Node(
            package="lsm6dsox_ros",
            executable="imu_publisher",
            name="lsm6dsox_imu_publisher",
            output="screen",
            parameters=[{
                "device_path": device_path,
                "frame_id": frame_id,
            }],
            remappings=[
                ("/imu/data", imu_topic),
            ],
        ),
    ])

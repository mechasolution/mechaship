import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mechaship_lidar_parameter = LaunchConfiguration(
        "mechaship_lidar_parameter",
        default=os.path.join(
            get_package_share_directory("mechaship_bringup"),
            "param",
            "mechaship_lidar.yaml",
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mechaship_lidar_parameter", default_value=mechaship_lidar_parameter
            ),
            LifecycleNode(
                package="ydlidar_ros2_driver",
                executable="ydlidar_ros2_driver_node",
                name="ydlidar_ros2_driver_node",
                output="screen",
                emulate_tty=True,
                parameters=[mechaship_lidar_parameter],
            ),
        ]
    )

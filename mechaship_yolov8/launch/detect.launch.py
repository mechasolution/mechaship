import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yolov8_params = LaunchConfiguration(
        "yolov8_params",
        default=os.path.join(
            get_package_share_directory("mechaship_yolov8"),
            "param",
            "yolov8_params.yaml",
        ),
    )
    yolov8_params_arg = DeclareLaunchArgument(
        "yolov8_params",
        default_value=yolov8_params,
    )

    yolov8_detect_node = Node(
        package="mechaship_yolov8",
        executable="detect_node",
        name="detect_node",
        output="screen",
        emulate_tty=True,
        parameters=[yolov8_params],
    )

    return LaunchDescription([yolov8_params_arg, yolov8_detect_node])
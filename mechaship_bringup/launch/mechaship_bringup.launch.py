import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir

from launch_ros.actions import Node, PushRosNamespace, LifecycleNode


def generate_launch_description():
    # LiDAR 드라이버
    mechaship_lidar_parameter = LaunchConfiguration(
        "mechaship_lidar_parameter",
        default=os.path.join(
            get_package_share_directory("mechaship_bringup"),
            "param/mechaship_lidar.yaml",
        ),
    )
    lidar_driver_launch_arg = DeclareLaunchArgument(
        "mechaship_lidar_parameter", default_value=mechaship_lidar_parameter
    )
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/mechaship_lidar.launch.py"]),
        launch_arguments={"mechaship_lidar_parameter": mechaship_lidar_parameter}.items(),
    )

    # csi 카메라
    camera_driver_node = Node(
        package="jetson_csi_camera_ros2_driver",
        executable="jetson_csi_camera_ros2_driver_node",
        name="jetson_csi_camera_ros2_driver_node",
    )

    # LiDAR TF2
    lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        arguments=["0", "0", "0", "0", "0", "0", "0", "base_link", "base_scan"],
    )

    # micro-ros
    micro_ros_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=["serial", "--dev", "/dev/ttyMCU"],
    )

    return LaunchDescription(
        [
            lidar_driver_launch_arg,
            lidar_driver_launch,
            lidar_tf_node,
            micro_ros_node,
            camera_driver_node,
        ]
    )
